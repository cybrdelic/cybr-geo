"""Advanced analytic geometry helpers for Mechanism Lab.

These helpers intentionally return CadQuery/OpenCascade shapes.  They are used by
benchmark models that need more than primitive extrusion/revolution while keeping
analytic CAD available for STEP/HLR/inspection workflows.
"""
from __future__ import annotations

import math
from typing import Iterable, Sequence

import cadquery as cq


def lofted_elliptic_shell(
    sections: Sequence[tuple[float, float, float, float, float]],
    wall: float,
):
    """Hollow multi-section BREP loft along +X.

    Each section is ``(x, y_radius, z_radius, y_offset, z_offset)``.  The inner
    loft is derived from the same centerline and subtracted, so the result stays
    a true hollow BREP instead of a tessellated visual shell.
    """
    if len(sections) < 2:
        raise ValueError('loft requires at least two sections')
    if wall <= 0:
        raise ValueError('wall must be positive')

    def _loft(shrink: float):
        x0, ry0, rz0, oy0, oz0 = sections[0]
        ry0 -= shrink
        rz0 -= shrink
        if min(ry0, rz0) <= 0:
            raise ValueError('wall collapses loft section')
        wp = cq.Workplane('YZ', origin=(x0, oy0, oz0)).ellipse(ry0, rz0)
        px, py, pz = x0, oy0, oz0
        for x, ry, rz, oy, oz in sections[1:]:
            ry -= shrink
            rz -= shrink
            if min(ry, rz) <= 0:
                raise ValueError('wall collapses loft section')
            wp = wp.workplane(offset=x - px).center(oy - py, oz - pz).ellipse(ry, rz)
            px, py, pz = x, oy, oz
        return wp.loft(combine=True, ruled=False).val()

    outer = _loft(0.0)
    inner = _loft(wall)
    # Extend the cavity slightly through both ends so there are no accidental caps.
    return outer.cut(inner)


def lofted_solid(sections: Sequence[tuple[float, float, float, float, float]]):
    """Solid asymmetric multi-section elliptic BREP loft along +X."""
    if len(sections) < 2:
        raise ValueError('loft requires at least two sections')
    x0, ry0, rz0, oy0, oz0 = sections[0]
    wp = cq.Workplane('YZ', origin=(x0, oy0, oz0)).ellipse(ry0, rz0)
    px, py, pz = x0, oy0, oz0
    for x, ry, rz, oy, oz in sections[1:]:
        wp = wp.workplane(offset=x - px).center(oy - py, oz - pz).ellipse(ry, rz)
        px, py, pz = x, oy, oz
    return wp.loft(combine=True, ruled=False).val()


def spline_sweep_tube(points: Sequence[tuple[float, float, float]], radius: float):
    """True BREP circular tube swept along a 3D interpolation spline."""
    if len(points) < 3 or radius <= 0:
        raise ValueError('spline sweep requires >=3 points and positive radius')
    vecs = [cq.Vector(*p) for p in points]
    edge = cq.Edge.makeSpline(vecs)
    wire = cq.Wire.assembleEdges([edge])
    tangent = vecs[1] - vecs[0]
    if tangent.Length < 1e-9:
        raise ValueError('degenerate first spline segment')
    plane = cq.Plane(origin=vecs[0], normal=tangent)
    return cq.Workplane(plane).circle(radius).sweep(wire, isFrenet=True, transition='round').val()


def helical_sweep(
    *,
    x0: float,
    length: float,
    helix_radius: float,
    pitch: float,
    section_radius: float,
    lefthand: bool = False,
):
    """Round-section true OpenCascade helix sweep with the helix axis along X."""
    if min(length, helix_radius, pitch, section_radius) <= 0:
        raise ValueError('helical dimensions must be positive')
    helix = cq.Wire.makeHelix(
        pitch,
        length,
        helix_radius,
        center=cq.Vector(x0, 0, 0),
        dir=cq.Vector(1, 0, 0),
        lefthand=lefthand,
    )
    # At the zero-angle helix point, the tangent is predominantly +Z with a small
    # +X component.  XY is therefore a robust starting section plane.
    return (
        cq.Workplane('XY', origin=(x0, helix_radius, 0))
        .circle(section_radius)
        .sweep(helix, isFrenet=True, transition='round')
        .val()
    )


def drafted_cylinder(x0: float, length: float, radius: float, taper_deg: float):
    """BREP drafted extrusion along X."""
    if min(length, radius) <= 0:
        raise ValueError('drafted cylinder dimensions must be positive')
    return (
        cq.Workplane('YZ', origin=(x0, 0, 0))
        .circle(radius)
        .extrude(length, taper=taper_deg)
        .val()
    )


def strut(p0, p1, radius: float):
    """Analytic cylindrical BREP strut between arbitrary 3D points."""
    a, b = cq.Vector(*p0), cq.Vector(*p1)
    d = b - a
    length = d.Length
    if length <= 1e-9 or radius <= 0:
        raise ValueError('invalid strut')
    return cq.Solid.makeCylinder(radius, length, a, d.normalized())


def bcc_lattice(
    *,
    origin: tuple[float, float, float],
    cells: tuple[int, int, int],
    pitch: tuple[float, float, float],
    strut_radius: float,
    node_radius: float | None = None,
):
    """BREP compound body-centred-cubic lattice.

    Struts and nodes remain analytic OpenCascade solids inside one compound.  This
    avoids converting a lattice showcase into a triangulated placeholder while
    also avoiding an unnecessarily expensive global fuse operation.
    """
    nx, ny, nz = cells
    if min(nx, ny, nz) < 1:
        raise ValueError('lattice needs at least one cell in each axis')
    sx, sy, sz = pitch
    if min(sx, sy, sz, strut_radius) <= 0:
        raise ValueError('invalid lattice dimensions')
    if node_radius is None:
        node_radius = strut_radius * 1.25
    ox, oy, oz = origin
    solids = []
    nodes = set()
    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                corners = [
                    (ox + (i + dx) * sx, oy + (j + dy) * sy, oz + (k + dz) * sz)
                    for dx in (0, 1) for dy in (0, 1) for dz in (0, 1)
                ]
                center = (ox + (i + .5) * sx, oy + (j + .5) * sy, oz + (k + .5) * sz)
                nodes.add(center)
                nodes.update(corners)
                solids.extend(strut(center, c, strut_radius) for c in corners)
    solids.extend(cq.Solid.makeSphere(node_radius, cq.Vector(*p)) for p in nodes)
    return cq.Compound.makeCompound(solids)


def toroidal_groove(major_radius: float, minor_radius: float, center=(0, 0, 0), axis=(1, 0, 0)):
    """Analytic torus useful as an O-ring/gland groove cutter."""
    if min(major_radius, minor_radius) <= 0:
        raise ValueError('torus dimensions must be positive')
    return cq.Solid.makeTorus(
        major_radius,
        minor_radius,
        cq.Vector(*center),
        cq.Vector(*axis),
    )
