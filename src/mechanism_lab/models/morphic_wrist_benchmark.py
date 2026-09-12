"""Hybrid benchmark wrapper for the CYBR Morphic Wrist.

Adds an implicit TPMS heat-spreader/damping insert to the analytic BREP-heavy
base model.  This intentionally demonstrates that modern geometry is hybrid:
BREP for interfaces/manufacturable surfaces, field geometry for porous TPMS, and
fine mesh routing for very small fibers.

The wrapper also substitutes a verified ruled two-section BREP gusset for the
base study's experimental drafted triangular rib.  The original 4-degree draft
collapsed its thin triangular profile in OpenCascade; the replacement retains a
true tapered CAD rib without weakening validation.  Drafted extrusion remains
independently exercised by the service collar and connector shell.
"""
from __future__ import annotations

import math
from dataclasses import replace

import cadquery as cq

from ..advanced_geometry import gyroid_sheet_mesh
from ..core import mesh_part
from . import morphic_wrist as _base

MATERIALS = _base.MATERIALS
SPEC = _base.SPEC


def _ruled_gusset(index: int):
    """Stable convex two-section tapered BREP rib oriented in the YZ plane."""
    angles = (math.pi/4, 3*math.pi/4, 5*math.pi/4, 7*math.pi/4)
    a = angles[index]
    # Radial direction follows the same elliptic structural envelope as the base
    # study.  A perpendicular vector gives each profile finite width.
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


def _build_verified_base():
    """Build the base while replacing only the known invalid experimental ribs."""
    original = _base.cad_part

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
        return original(name, shape, material, **kw)

    _base.cad_part = verified_cad_part
    try:
        return _base.build()
    finally:
        _base.cad_part = original


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
        2,
        group='implicit',
        role='Implicit TPMS gyroid heat-spreader / damping insert benchmark',
        provenance='designed-concept',
        tags=('technique:implicit-tpms', 'technique:field-modeling', 'technique:flying-edges'),
    )
    metadata = dict(assembly.metadata)
    techniques = list(metadata.get('geometry_techniques', ()))
    techniques.extend([
        'ruled two-section tapered BREP gussets',
        'implicit TPMS / field modeling',
        'VTK FlyingEdges iso-surface extraction',
    ])
    metadata['geometry_techniques'] = techniques
    metadata['hybrid_geometry_contract'] = (
        'Analytic OpenCascade BREP for precision/interface geometry; implicit TPMS '
        'for porous field geometry; fine triangle tubes for 0.20 mm SMA fibers.'
    )
    return replace(assembly, parts=[*assembly.parts, tpms], metadata=metadata)


__all__ = ['SPEC', 'MATERIALS', 'build']
