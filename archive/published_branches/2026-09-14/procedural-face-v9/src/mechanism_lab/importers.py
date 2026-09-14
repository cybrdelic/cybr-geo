"""Import documented units into the same mesh/CAD contract used by native recipes.

STEP imports keep analytic solids. glTF imports retain node transforms and
metal/rough materials. OBJ/STL/PLY require an explicit unit choice. Static imports
receive inspection/explosion views, not invented functional joint constraints.
"""
from __future__ import annotations

from pathlib import Path
import hashlib
import json
import re
import numpy as np
import trimesh
from .core import Assembly, Material, View, cad_part, mesh_part

UNIT_SCALE = {'mm': 1.0, 'cm': 10.0, 'm': 1000.0, 'inch': 25.4}
# Right-handed Y-up -> Z-up. X remains the shaft direction.
YUP_TO_ZUP = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])


def safe_name(text: str, fallback: str = 'imported_part') -> str:
    text = re.sub(r'[^A-Za-z0-9_.-]+', '_', str(text)).strip('._')
    return text or fallback


def srgb_to_linear(rgb):
    rgb = np.asarray(rgb, dtype=float)
    return np.where(rgb <= .04045, rgb / 12.92, ((rgb + .055) / 1.055) ** 2.4)


def import_geometry(source, name=None, units=None, up_axis=None) -> Assembly:
    source = Path(source).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(source)
    ext = source.suffix.lower()
    if ext not in {'.glb', '.gltf', '.step', '.stp', '.obj', '.stl', '.ply'}:
        raise ValueError(f'Unsupported geometry format: {ext}')
    if ext in {'.glb', '.gltf'}:
        if units not in (None, 'm') or up_axis not in (None, 'Y'):
            raise ValueError('Standard glTF is metres/Y-up. Do not apply an extra import unit override.')
        units, up_axis = 'm', 'Y'
    elif ext in {'.step', '.stp'}:
        if units not in (None, 'mm'):
            raise ValueError('OpenCascade converts STEP length units to mm. Use units=mm here.')
        units, up_axis = 'mm', up_axis or 'Z'
    elif units not in UNIT_SCALE:
        raise ValueError('OBJ/STL/PLY require units: mm, cm, m or inch')
    up_axis = up_axis or 'Z'
    if up_axis not in ('Y', 'Z'):
        raise ValueError('up_axis must be Y or Z')
    factor = UNIT_SCALE[units]
    rotation = YUP_TO_ZUP if up_axis == 'Y' else np.eye(3)
    materials = [Material('Imported neutral metal', (.42, .45, .49), .8, .3)]
    parts = []
    if ext in {'.step', '.stp'}:
        import cadquery as cq
        shape = cq.importers.importStep(str(source)).val()
        solids = shape.Solids()
        if not solids:
            raise ValueError('STEP contains no solid bodies; surface-only CAD is not silently filled.')
        for i, solid in enumerate(solids):
            if up_axis == 'Y':
                solid = solid.rotate((0, 0, 0), (1, 0, 0), 90)
            parts.append(cad_part(f'STEP_{i+1:04}', solid,
                                  group='imported', provenance='imported-analytic-CAD'))
    else:
        loaded = trimesh.load_scene(source, process=False)
        for i, node in enumerate(loaded.graph.nodes_geometry):
            transform, geometry_id = loaded.graph[node]
            original = loaded.geometry[geometry_id]
            if not isinstance(original, trimesh.Trimesh) or not len(original.faces):
                continue
            mesh = original.copy()
            mesh.apply_transform(transform)
            vertices = (np.asarray(mesh.vertices) * factor) @ rotation.T
            material_index = 0
            visual_material = getattr(mesh.visual, 'material', None)
            if visual_material is not None:
                base = getattr(visual_material, 'baseColorFactor', None)
                if base is not None:
                    color = np.asarray(base)[:3].astype(float)
                    if color.max() > 1:
                        color /= 255
                    metal = getattr(visual_material, 'metallicFactor', 0.)
                    rough = getattr(visual_material, 'roughnessFactor', .45)
                    materials.append(Material(f'import_{i:04}', tuple(srgb_to_linear(color)),
                                              float(metal if metal is not None else 0),
                                              float(rough if rough is not None else .45)))
                    material_index = len(materials) - 1
            part_name = f'{i+1:04}_{safe_name(node)}'
            parts.append(mesh_part(part_name, vertices, np.asarray(mesh.faces), material_index,
                                   group='imported', provenance='imported-mesh',
                                   role='Imported rigid mesh; no motion semantics inferred'))
    if not parts:
        raise ValueError('Source contains no usable solid/triangle geometry')
    assembly = Assembly(safe_name(name or source.stem), parts, materials)
    lo, hi = assembly.bounds
    center, span = (lo+hi)/2, np.linalg.norm(hi-lo)
    half_height = max(1., span * .6)
    for i, part in enumerate(parts):
        direction = part.bounds.mean(axis=0) - center
        norm = np.linalg.norm(direction)
        if norm < 1e-7:
            direction = np.array([i-(len(parts)-1)/2, 0., 0.])
            norm = np.linalg.norm(direction)
        part.explode = direction / max(norm, 1e-8) * span * .6
    assembly.views = {
        'hero': View(45, 26, half_height, tuple(center), title=f'{assembly.name} / IMPORTED GEOMETRY'),
        'rear': View(220, 24, half_height, tuple(center), title=f'{assembly.name} / REAR'),
        'section': View(42, 22, half_height, tuple(center), section=(0, -1, 0),
                        title=f'{assembly.name} / GEOMETRIC SECTION'),
        'exploded': View(45, 26, half_height*2, tuple(center), explode=1,
                         title=f'{assembly.name} / INSPECTION OFFSETS'),
    }
    assembly.metadata = {
        'source_file': source.name,
        'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'source_units': units, 'source_up_axis': up_axis,
        'units': 'mm', 'up_axis': 'Z',
        'limitations': 'Static import. Embedded textures, skinning and source animations are not imported. '
                        'Materials use base color/metal/rough values only. Explosion offsets are not verified disassembly paths.',
    }
    return assembly


def build_descriptor(path):
    """Load a non-executable JSON import recipe. File path is relative to the recipe."""
    path = Path(path).resolve()
    spec = json.loads(path.read_text())
    if spec.get('schema') != 1 or spec.get('kind') != 'geometry-import':
        raise ValueError('Expected schema=1, kind=geometry-import')
    return import_geometry(path.parent / spec['source'], spec.get('name'),
                           spec.get('units'), spec.get('up_axis'))
