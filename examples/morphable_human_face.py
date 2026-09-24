"""Morphable photoreal human head built on an attributed canonical scan topology.

This is intentionally scan-derived, not claimed as original anatomy. The canonical
Lee Perry-Smith surface supplies realistic human topology and measured texture/
displacement detail. CYBR GEO adds deterministic smooth semantic deformation fields
while preserving triangle connectivity and UVs.

Coordinates are CYBR GEO millimetres, Z-up, face toward -Y.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict, replace
from pathlib import Path
import importlib.util
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class MorphableFaceParameters:
    # Dimensionless fractional shape deltas. Keep routine identity variation
    # deliberately modest so the canonical human prior remains anatomically sane.
    skull_width: float = 0.0
    temple_width: float = 0.0
    cheek_width: float = 0.0
    jaw_width: float = 0.0
    lower_face_length: float = 0.0
    face_length: float = 0.0
    nose_width: float = 0.0
    mouth_width: float = 0.0
    # Millimetre front/back translations. Negative Y is outward/front.
    nose_projection_mm: float = 0.0
    brow_projection_mm: float = 0.0
    chin_projection_mm: float = 0.0
    # Small deterministic left/right difference; 0.0 is the canonical scan.
    asymmetry: float = 0.0


def _reference_module():
    path = ROOT / "examples" / "reference_scan_face.py"
    spec = importlib.util.spec_from_file_location("cybr_reference_scan_face", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _gaussian2(x, z, cx, cz, sx, sz):
    return np.exp(-0.5*((x-cx)/sx)**2 - 0.5*((z-cz)/sz)**2)


def _smoothstep(a, b, x):
    t = np.clip((x-a)/(b-a), 0.0, 1.0)
    return t*t*(3.0-2.0*t)


def _recompute_normals(vertices, faces):
    v = np.asarray(vertices, np.float64)
    f = np.asarray(faces, np.int32)
    cross = np.cross(v[f[:,1]]-v[f[:,0]], v[f[:,2]]-v[f[:,0]])
    n = np.zeros_like(v)
    for k in range(3):
        np.add.at(n, f[:,k], cross)
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    return n


def morph_vertices(vertices, p: MorphableFaceParameters):
    """Apply smooth regional deformations to the canonical human topology.

    The operations are intentionally continuous fields instead of hard region
    transforms. This preserves the scan's local curvature and high-frequency
    displacement while modifying large-scale identity proportions.
    """
    v = np.asarray(vertices, np.float64).copy()
    x, y, z = v.T

    # A normalized "frontness" gate prevents facial parameters from distorting
    # the posterior cranium. The head faces -Y in CYBR coordinates.
    front = _smoothstep(-10.0, -55.0, y)

    # Broad cranium / temple / cheek / jaw identity fields.
    cranium = _smoothstep(10.0, 75.0, z)
    temples = _gaussian2(x, z, 0.0, 58.0, 78.0, 38.0)
    cheeks = (_gaussian2(x, z,  42.0, 8.0, 28.0, 30.0) +
              _gaussian2(x, z, -42.0, 8.0, 28.0, 30.0))
    jaw = (_gaussian2(x, z,  48.0, -52.0, 34.0, 34.0) +
           _gaussian2(x, z, -48.0, -52.0, 34.0, 34.0))

    x *= 1.0 + p.skull_width * 0.65 * cranium
    x *= 1.0 + p.temple_width * 0.55 * temples
    x *= 1.0 + p.cheek_width * 0.55 * cheeks * front
    x *= 1.0 + p.jaw_width * 0.75 * jaw * front

    # Global and lower-face vertical proportion changes are anchored near the
    # orbital plane so the neck/base does not simply scale with the face.
    anchor_z = 32.0
    whole_face = _smoothstep(125.0, -105.0, z) * front
    z += (z-anchor_z) * p.face_length * 0.34 * whole_face
    lower = _smoothstep(8.0, -82.0, z) * front
    z += (z-8.0) * p.lower_face_length * 0.48 * lower

    # Nose: width is lateral scaling around the midline; projection is a
    # smoothly localized anterior displacement and retains the source profile.
    nose = _gaussian2(x, z, 0.0, 12.0, 18.0, 30.0) * front
    x *= 1.0 + p.nose_width * 0.65 * nose
    y -= p.nose_projection_mm * nose

    # Mouth: scale locally around the face center without moving the cheeks.
    mouth = _gaussian2(x, z, 0.0, -28.0, 34.0, 16.0) * front
    x *= 1.0 + p.mouth_width * 0.68 * mouth

    # Brow and chin projection fields.
    brow = (_gaussian2(x, z,  27.0, 48.0, 24.0, 15.0) +
            _gaussian2(x, z, -27.0, 48.0, 24.0, 15.0)) * front
    chin = _gaussian2(x, z, 0.0, -72.0, 34.0, 24.0) * front
    y -= p.brow_projection_mm * brow
    y -= p.chin_projection_mm * chin

    # Correlated asymmetry rather than mirror-breaking white noise. The
    # deformation remains low-frequency and sub-millimetric at normal settings.
    if p.asymmetry:
        side = np.tanh(x/24.0)
        y += p.asymmetry * 1.4 * side * (
            0.55*_gaussian2(x,z,0.0,18.0,68.0,70.0) +
            0.45*_gaussian2(x,z,0.0,-45.0,58.0,40.0)
        ) * front
        z += p.asymmetry * 0.7 * side * _gaussian2(x,z,0.0,10.0,72.0,75.0) * front

    return v


def build(parameters: MorphableFaceParameters = MorphableFaceParameters()):
    ref = _reference_module()
    assembly = ref.build()
    source = assembly.parts[0]
    vertices = morph_vertices(source.vertices, parameters)
    normals = _recompute_normals(vertices, source.faces)

    skin = replace(
        source,
        name="Morphable_human_face_and_neck",
        vertices=vertices,
        normals=normals,
        provenance="canonical-scan-morph",
        role="Lee Perry-Smith canonical scan deformed by CYBR GEO semantic identity fields",
        tags=tuple(source.tags) + ("morphable-face", "semantic-deformation"),
    )
    skin.portrait_uv = source.portrait_uv.copy()

    meta = dict(assembly.metadata)
    meta.update({
        "model_type": "scan-derived morphable human head",
        "canonical_topology": "Infinite, 3D Head Scan by Lee Perry-Smith / CC BY 3.0",
        "morph_parameters": asdict(parameters),
        "image_generation": False,
        "identity_generation_limit": (
            "These are smooth deformations of one attributed canonical scan, not a statistical "
            "population model and not independent scanned identities."
        ),
    })
    return replace(assembly, name="morphable_human_face", parts=[skin], metadata=meta)
