"""Hybrid benchmark wrapper for the CYBR Morphic Wrist.

Adds an implicit TPMS heat-spreader/damping insert to the analytic BREP-heavy
base model.  This intentionally demonstrates that modern geometry is hybrid:
BREP for interfaces/manufacturable surfaces, field geometry for porous TPMS, and
fine mesh routing for very small fibers.
"""
from __future__ import annotations

from dataclasses import replace

from ..advanced_geometry import gyroid_sheet_mesh
from ..core import mesh_part
from .morphic_wrist import MATERIALS, SPEC, build as _base_build


def build():
    assembly = _base_build()
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
