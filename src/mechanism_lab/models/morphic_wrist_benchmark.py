"""Hybrid benchmark wrapper for the CYBR Morphic Wrist.

Adds an implicit TPMS heat-spreader/damping insert to the analytic BREP-heavy
base model.  This intentionally demonstrates that modern geometry is hybrid:
BREP for interfaces/manufacturable surfaces, field geometry for porous TPMS, and
fine mesh routing for very small fibers.

The wrapper substitutes two verified constructions for exploratory base-study
features that were poor production citizens in this OpenCascade build:
- ruled two-section BREP gussets replace a self-intersecting drafted triangle;
- a segmented analytic BREP helix replaces a mathematically valid swept helix
  whose NURBS surface tessellated pathologically.  It is explicitly *not* called
  a true continuous helical surface.
Drafted extrusion and true continuous BREP spline sweeps remain exercised elsewhere.
"""
from __future__ import annotations

import math
from dataclasses import replace

import cadquery as cq

from ..advanced_geometry import gyroid_sheet_mesh
from ..core import Material, mesh_part
from . import morphic_wrist as _base

SPEC = _base.SPEC
SINTERED_TITANIUM = len(_base.MATERIALS)
SATIN_BLACK_ANODIZED = SINTERED_TITANIUM + 1
MATERIALS = [
    *_base.MATERIALS,
    Material(
        'Sintered titanium / additive finish', (0.34, 0.36, 0.38), .70, .43,
        ior=1.50, coat=.015, coat_rough=.30, anisotropy=.02,
        microfinish='bead-blasted', material_source='original benchmark additive finish',
    ),
    Material(
        'Satin black hard-anodized aluminium', (0.045, 0.052, 0.062), .46, .35,
        ior=1.48, coat=.08, coat_rough=.25, anisotropy=.05,
        microfinish='anodized', material_source='original benchmark exterior finish',
    ),
]


def _ruled_gusset(index: int):
    """Stable convex two-section tapered BREP rib oriented in the YZ plane."""
    angles = (math.pi/4, 3*math.pi/4, 5*math.pi/4, 7*math.pi/4)
    a = angles[index]
    ry, rz = 19.0 * math.cos(a), 15.0 * math.sin(a)
    n = math.hypot(ry, rz)
    uy, uz = ry/n, rz/n
    py, pz = -uz, uy

    def profile(r0, r1, w0, w1, scale=1.0):
        return [
            ((r0*uy - w0*py)*scale, (r0*uz - w0*pz)*scale),
            ((r1*uy - w1*py)*scale, (r1*uz - w1*pz)*scale),
            ((r1*uy + w1*py)*scale, (r1*uz + w1*pz)*scale),
            ((r0*uy + w0*py)*scale, (r0*uz + w0*pz)*scale),
        ]

    near = profile(6.0, 23.0, 2.2, 3.1, 1.0)
    far = profile(6.0, 23.0, 2.2, 3.1, .86)
    wp = cq.Workplane('YZ', origin=(64, 0, 0)).polyline(near).close()
    wp = wp.workplane(offset=6.0).polyline(far).close()
    shape = wp.loft(combine=True, ruled=True).val()
    if not shape.isValid():
        raise ValueError(f'ruled gusset {index+1} is invalid')
    return shape


def _segmented_brep_helix(*, x0, length, helix_radius, pitch, section_radius, lefthand=False):
    """Efficient BREP compound following a helical centerline."""
    turns = length / pitch
    segments = max(40, int(math.ceil(turns * 18)))
    sign = -1.0 if lefthand else 1.0
    points = []
    for i in range(segments + 1):
        u = i / segments
        a = sign * math.tau * turns * u
        points.append(cq.Vector(
            x0 + length*u,
            helix_radius*math.cos(a),
            helix_radius*math.sin(a),
        ))
    solids = []
    for a, b in zip(points, points[1:]):
        d = b - a
        solids.append(cq.Solid.makeCylinder(section_radius, d.Length, a, d.normalized()))
    solids.extend(cq.Solid.makeSphere(section_radius, p) for p in points[1:-1])
    result = cq.Compound.makeCompound(solids)
    if not result.isValid():
        raise ValueError('segmented BREP helix is invalid')
    return result


def _build_verified_base():
    """Build the base while replacing only verified-problematic study geometry."""
    original_cad_part = _base.cad_part
    original_helix = _base.helical_sweep

    routed_prefixes = (
        'MW_02_Conformal_channel_liner_',
        'MW_18_Spline_tendon_',
        'MW_19_Spline_flexure_',
        'MW_23_Copper_bus_',
    )

    def verified_cad_part(name, shape, material=0, **kw):
        if name.startswith('MW_05_Drafted_rib_'):
            idx = int(name.rsplit('_', 1)[1]) - 1
            shape = _ruled_gusset(idx)
            tags = [t for t in kw.get('tags', ()) if t != 'technique:drafted-extrusion']
            for tag in ('technique:ruled-tapered-loft', 'technique:gusset-rib'):
                if tag not in tags:
                    tags.append(tag)
            kw['tags'] = tuple(tags)
            kw['role'] = 'Ruled two-section tapered load-transfer gusset'
        elif name == 'MW_07_Helical_service_thread':
            tags = [t for t in kw.get('tags', ()) if t not in ('technique:true-helix', 'technique:brep-sweep')]
            tags.extend(('technique:segmented-brep-helix', 'technique:analytic-compound'))
            kw['tags'] = tuple(dict.fromkeys(tags))
            kw['role'] = 'Sampled analytic BREP helical service ridge; not a continuous swept-helix surface'
            kw['tolerance'] = max(float(kw.get('tolerance', .03)), .08)
            kw['angular'] = max(float(kw.get('angular', .05)), .10)
        elif name.startswith(routed_prefixes):
            kw['tolerance'] = max(float(kw.get('tolerance', .03)), .08)
            kw['angular'] = max(float(kw.get('angular', .05)), .12)
        return original_cad_part(name, shape, material, **kw)

    _base.cad_part = verified_cad_part
    _base.helical_sweep = _segmented_brep_helix
    try:
        return _base.build()
    finally:
        _base.cad_part = original_cad_part
        _base.helical_sweep = original_helix


def _polish_parts(parts):
    """Presentation-only material refinement; geometry and provenance are unchanged."""
    out = []
    for p in parts:
        if p.name == 'MW_01_Freeform_exoskeleton':
            p = replace(p, material=SATIN_BLACK_ANODIZED)
        elif p.name == 'MW_03_BCC_lattice_core':
            p = replace(p, material=SINTERED_TITANIUM)
        elif p.name == 'MW_26_Drafted_connector_shell':
            # The previous tan PEEK block visually dominated the hero.  A dark
            # strain-relief shell is still an authored material assignment, not a
            # geometry substitution.
            p = replace(p, material=8)
        out.append(p)
    return out


def _polish_views(views):
    """Use the Nitinol V2 PBR composition as the minimum visual-quality floor."""
    v = dict(views)
    v['hero'] = replace(
        v['hero'], az=236, el=16, scale=98, target=(94, 0, 0),
        focal_length_mm=72, camera_distance_mm=468, f_stop=6.3,
        environment_strength=.22, background_strength=.78,
        light_size=1.68, light_intensity=1.03, exposure=.99,
        hide=('implicit',),
        title='CYBR MORPHIC WRIST / HYBRID GEOMETRY BENCHMARK',
        note='Lofted BREP shell / spline routing / lattice core / antagonistic SMA banks / 2-DOF wrist architecture',
    )
    v['cutaway'] = replace(
        v['cutaway'], az=232, el=17, scale=92, target=(94, 0, 0),
        focal_length_mm=72, camera_distance_mm=445, f_stop=6.3,
        environment_strength=.23, background_strength=.77,
        light_size=1.68, light_intensity=1.04, exposure=1.0,
        hide=('shell', 'overmold'),
    )
    v['geometry_macro'] = replace(
        v['geometry_macro'], az=245, el=18, scale=56, target=(132, 0, 1),
        focal_length_mm=82, camera_distance_mm=318, f_stop=5.6,
        environment_strength=.24, background_strength=.75,
        light_size=1.75, light_intensity=1.05, exposure=1.01,
        hide=('shell', 'overmold', 'electronics', 'connector', 'implicit'),
        title='FREEFORM 2-DOF JOINT / BREP ROUTING MACRO',
        note='Nested lofted cradle / orthogonal bearing bores / spline tendons and flexures / revolved encoder hub',
    )
    v['lattice_macro'] = replace(
        v['lattice_macro'], az=228, el=22, scale=48, target=(58, 0, 0),
        focal_length_mm=78, camera_distance_mm=286, f_stop=6.3,
        environment_strength=.24, background_strength=.75,
        light_size=1.74, light_intensity=1.05, exposure=1.01,
        hide=('shell', 'overmold', 'sma_fibers', 'implicit', 'electronics', 'connector', 'sensor'),
        title='ANALYTIC BREP LATTICE / ORGANIC LOAD SPINE',
        note='OpenCascade BCC struts and nodes / lofted load path / ruled tapered gussets / spline service routing',
    )
    v['implicit_macro'] = replace(
        v['geometry_macro'], az=224, el=17, scale=30, target=(60, -23.5, 0),
        focal_length_mm=86, camera_distance_mm=205, f_stop=7.1,
        environment_strength=.25, background_strength=.74,
        light_size=1.80, light_intensity=1.06, exposure=1.02,
        hide=('shell', 'overmold', 'structure', 'lattice', 'sma_fibers', 'tendons', 'flexures', 'joint', 'output', 'bearings', 'sensor', 'electronics', 'connector', 'fasteners'),
        title='IMPLICIT TPMS / FIELD-GEOMETRY MACRO',
        note='Gyroid-sheet scalar field / VTK FlyingEdges extraction / intentionally non-BREP porous geometry',
    )
    v['exploded'] = replace(
        v['exploded'], az=232, el=22, scale=142, target=(92, 0, 0),
        focal_length_mm=72, camera_distance_mm=610, f_stop=9.0,
        environment_strength=.23, background_strength=.78,
        light_size=1.65, light_intensity=1.03, exposure=.99,
    )
    return v


def build():
    assembly = _build_verified_base()
    vertices, faces = gyroid_sheet_mesh(
        (42.0, 78.0, -29.0, -18.0, -10.0, 10.0),
        resolution=(46, 28, 34),
        periods=(1.6, 1.15, 1.35),
        thickness=.27,
    )
    tpms = mesh_part(
        'MW_28_Implicit_gyroid_insert',
        vertices,
        faces,
        SINTERED_TITANIUM,
        group='implicit',
        role='Implicit TPMS gyroid heat-spreader / damping insert benchmark',
        provenance='designed-concept',
        tags=('technique:implicit-tpms', 'technique:field-modeling', 'technique:flying-edges'),
    )
    metadata = dict(assembly.metadata)
    techniques = [
        t for t in metadata.get('geometry_techniques', ())
        if t != 'true OpenCascade helical sweep'
    ]
    techniques.extend([
        'ruled two-section tapered BREP gussets',
        'segmented analytic BREP helical path',
        'implicit TPMS / field modeling',
        'VTK FlyingEdges iso-surface extraction',
        'scale-aware BREP tessellation policy',
    ])
    metadata['geometry_techniques'] = techniques
    metadata['hybrid_geometry_contract'] = (
        'Analytic OpenCascade BREP for precision/interface geometry; implicit TPMS '
        'for porous field geometry; fine triangle tubes for 0.20 mm SMA fibers.'
    )
    metadata['helix_fidelity'] = (
        'Service ridge uses a truth-labeled segmented BREP helix because the '
        'continuous swept-helix NURBS face is pathological to tessellate in this OCP build.'
    )
    metadata['tessellation_policy'] = (
        'Precision surfaces keep 0.03-0.04 mm chord tolerance; long routed BREP '
        'curves use 0.08 mm because it remains sub-pixel at the benchmark frame scale.'
    )
    metadata['presentation_policy'] = (
        'Exterior hero hides only the implicit TPMS insert for silhouette/readability; '
        'cutaway and dedicated implicit macro expose it. Geometry remains in the assembly.'
    )
    parts = _polish_parts([*assembly.parts, tpms])
    views = _polish_views(assembly.views)
    return replace(assembly, parts=parts, materials=MATERIALS, views=views, metadata=metadata)


__all__ = ['SPEC', 'MATERIALS', 'build']
