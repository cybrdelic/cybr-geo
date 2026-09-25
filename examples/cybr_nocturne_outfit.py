"""CYBR NOCTURNE v2 — procedural technical couture for CYBR GEO.

This recipe rebuilds the outfit as explicit garment-like geometry.  It does not
use scans, photogrammetry, image generation, image textures, or baked geometry.
All shape is generated numerically from named panels, lofts, straps, seams,
hardware, footwear pieces, and a procedural faceless dress form.

Coordinate convention:
    +Z up
    +Y front
    millimetres

The design intentionally separates:
    - fitted underlayer
    - shaped coat front panels
    - shaped back/yoke/tail panels
    - lapels / collar / lining
    - leather harness and shoulder epaulettes
    - metal hardware
    - trousers and constructed boots
so that the silhouette can be changed without re-authoring the renderer.
"""
from __future__ import annotations

import math
from typing import Sequence

import numpy as np
import trimesh

from cybrgeo import Assembly, Material, Part


TAU = math.tau


# Material indices. Keep these synchronized with tools/render_cybr_nocturne_cybrlight.py.
MAT_MANNEQUIN = 0
MAT_WOOL = 1
MAT_TECH = 2
MAT_PURPLE_SATIN = 3
MAT_LEATHER = 4
MAT_METAL = 5
MAT_PURPLE_LEATHER = 6
MAT_RUBBER = 7
MAT_STITCH = 8


def _part(
    name: str,
    vertices: np.ndarray,
    faces: np.ndarray,
    material: int,
    *,
    group: str,
    role: str,
    metadata: dict | None = None,
) -> Part:
    vertices = np.asarray(vertices, dtype=np.float64)
    faces = np.asarray(faces, dtype=np.int64)
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    normals = np.asarray(mesh.vertex_normals, dtype=np.float64)
    return Part(
        name=name,
        vertices=vertices,
        faces=faces,
        normals=normals,
        material=material,
        group=group,
        role=role,
        metadata=metadata or {},
    )


def _ellipsoid(
    name: str,
    center: Sequence[float],
    radii: Sequence[float],
    material: int,
    *,
    group: str,
    role: str,
    subdivisions: int = 3,
) -> Part:
    mesh = trimesh.creation.icosphere(subdivisions=subdivisions, radius=1.0)
    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    vertices *= np.asarray(radii, dtype=np.float64)
    vertices += np.asarray(center, dtype=np.float64)
    return _part(name, vertices, mesh.faces, material, group=group, role=role)


def _box(
    name: str,
    center: Sequence[float],
    extents: Sequence[float],
    material: int,
    *,
    group: str,
    role: str,
) -> Part:
    mesh = trimesh.creation.box(extents=np.asarray(extents, dtype=float))
    mesh.apply_translation(np.asarray(center, dtype=float))
    return _part(name, mesh.vertices, mesh.faces, material, group=group, role=role)


def _oriented_box(
    name: str,
    start: Sequence[float],
    end: Sequence[float],
    width: float,
    depth: float,
    material: int,
    *,
    group: str,
    role: str,
) -> Part:
    """Rectangular strap aligned along the vector from start to end."""
    a = np.asarray(start, dtype=np.float64)
    b = np.asarray(end, dtype=np.float64)
    direction = b - a
    length = float(np.linalg.norm(direction))
    if length <= 1e-6:
        raise ValueError("oriented box endpoints must differ")
    direction /= length
    mesh = trimesh.creation.box(extents=(width, depth, length))
    transform = trimesh.geometry.align_vectors([0.0, 0.0, 1.0], direction)
    if transform is None:
        transform = np.eye(4)
    transform[:3, 3] = 0.5 * (a + b)
    mesh.apply_transform(transform)
    return _part(name, mesh.vertices, mesh.faces, material, group=group, role=role)


def _z_loft(
    name: str,
    sections: Sequence[tuple[float, float, float, float, float]],
    material: int,
    *,
    group: str,
    role: str,
    radial: int = 64,
) -> Part:
    """Closed loft along Z.

    Section tuple:
        (z, radius_x, radius_y, offset_x, offset_y)
    """
    if len(sections) < 2:
        raise ValueError("z loft needs >= 2 sections")
    vertices: list[list[float]] = []
    faces: list[list[int]] = []
    for z, rx, ry, ox, oy in sections:
        for i in range(radial):
            a = TAU * i / radial
            vertices.append([ox + rx * math.cos(a), oy + ry * math.sin(a), z])

    for ring in range(len(sections) - 1):
        a0 = ring * radial
        b0 = (ring + 1) * radial
        for i in range(radial):
            j = (i + 1) % radial
            faces.extend(
                [
                    [a0 + i, a0 + j, b0 + j],
                    [a0 + i, b0 + j, b0 + i],
                ]
            )

    low_center = len(vertices)
    z, _, _, ox, oy = sections[0]
    vertices.append([ox, oy, z])
    high_center = len(vertices)
    z, _, _, ox, oy = sections[-1]
    vertices.append([ox, oy, z])
    top = (len(sections) - 1) * radial
    for i in range(radial):
        j = (i + 1) % radial
        faces.append([low_center, j, i])
        faces.append([high_center, top + i, top + j])

    return _part(
        name,
        np.asarray(vertices),
        np.asarray(faces),
        material,
        group=group,
        role=role,
        metadata={"generator": "z_loft", "sections": len(sections), "radial": radial},
    )


def _y_loft(
    name: str,
    sections: Sequence[tuple[float, float, float, float, float]],
    material: int,
    *,
    group: str,
    role: str,
    radial: int = 48,
) -> Part:
    """Closed loft along Y, useful for footwear.

    Section tuple:
        (y, radius_x, radius_z, offset_x, offset_z)
    """
    if len(sections) < 2:
        raise ValueError("y loft needs >= 2 sections")
    vertices: list[list[float]] = []
    faces: list[list[int]] = []
    for y, rx, rz, ox, oz in sections:
        for i in range(radial):
            a = TAU * i / radial
            vertices.append([ox + rx * math.cos(a), y, oz + rz * math.sin(a)])

    for ring in range(len(sections) - 1):
        a0 = ring * radial
        b0 = (ring + 1) * radial
        for i in range(radial):
            j = (i + 1) % radial
            faces.extend(
                [
                    [a0 + i, b0 + j, a0 + j],
                    [a0 + i, b0 + i, b0 + j],
                ]
            )

    rear_center = len(vertices)
    y, _, _, ox, oz = sections[0]
    vertices.append([ox, y, oz])
    front_center = len(vertices)
    y, _, _, ox, oz = sections[-1]
    vertices.append([ox, y, oz])
    last = (len(sections) - 1) * radial
    for i in range(radial):
        j = (i + 1) % radial
        faces.append([rear_center, i, j])
        faces.append([front_center, last + j, last + i])

    return _part(
        name,
        np.asarray(vertices),
        np.asarray(faces),
        material,
        group=group,
        role=role,
        metadata={"generator": "y_loft", "sections": len(sections), "radial": radial},
    )


def _tube(
    name: str,
    points: Sequence[Sequence[float]],
    radii: Sequence[tuple[float, float]] | tuple[float, float],
    material: int,
    *,
    group: str,
    role: str,
    radial: int = 24,
    cap: bool = True,
) -> Part:
    """Elliptical tube following a polyline."""
    path = np.asarray(points, dtype=np.float64)
    if path.ndim != 2 or path.shape[1] != 3 or len(path) < 2:
        raise ValueError("tube needs at least 2 3D points")
    if isinstance(radii, tuple) and len(radii) == 2 and isinstance(radii[0], (float, int)):
        rr = [tuple(map(float, radii))] * len(path)
    else:
        rr = [tuple(map(float, pair)) for pair in radii]  # type: ignore[arg-type]
        if len(rr) != len(path):
            raise ValueError("radii count must match path")

    frames: list[tuple[np.ndarray, np.ndarray]] = []
    previous_b1: np.ndarray | None = None
    for i in range(len(path)):
        if i == 0:
            tangent = path[1] - path[0]
        elif i == len(path) - 1:
            tangent = path[-1] - path[-2]
        else:
            tangent = path[i + 1] - path[i - 1]
        tangent /= np.linalg.norm(tangent)
        reference = np.array([0.0, 0.0, 1.0])
        if abs(float(np.dot(tangent, reference))) > 0.92:
            reference = np.array([0.0, 1.0, 0.0])
        b1 = np.cross(tangent, reference)
        b1 /= np.linalg.norm(b1)
        if previous_b1 is not None and float(np.dot(b1, previous_b1)) < 0.0:
            b1 = -b1
        b2 = np.cross(tangent, b1)
        b2 /= np.linalg.norm(b2)
        previous_b1 = b1
        frames.append((b1, b2))

    vertices: list[list[float]] = []
    faces: list[list[int]] = []
    for point, (rx, ry), (b1, b2) in zip(path, rr, frames):
        for i in range(radial):
            a = TAU * i / radial
            p = point + math.cos(a) * rx * b1 + math.sin(a) * ry * b2
            vertices.append(p.tolist())

    for ring in range(len(path) - 1):
        a0 = ring * radial
        b0 = (ring + 1) * radial
        for i in range(radial):
            j = (i + 1) % radial
            faces.extend(
                [
                    [a0 + i, a0 + j, b0 + j],
                    [a0 + i, b0 + j, b0 + i],
                ]
            )

    if cap:
        c0 = len(vertices)
        vertices.append(path[0].tolist())
        c1 = len(vertices)
        vertices.append(path[-1].tolist())
        last = (len(path) - 1) * radial
        for i in range(radial):
            j = (i + 1) % radial
            faces.append([c0, j, i])
            faces.append([c1, last + i, last + j])

    return _part(name, np.asarray(vertices), np.asarray(faces), material, group=group, role=role)


def _curved_panel(
    name: str,
    sections: Sequence[tuple[float, float, float, float, float]],
    material: int,
    *,
    group: str,
    role: str,
    thickness: float = 6.0,
    samples_across: int = 12,
    face: str = "front",
    fold_amp: float = 0.0,
    fold_cycles: float = 0.0,
) -> Part:
    """Finite-thickness curved garment panel.

    Each section is:
        (z, left_x, right_x, base_y, bow_y)

    bow_y pushes the middle of the panel away from base_y.  For a front panel,
    positive bow_y follows the chest. For a back panel, negative bow_y follows
    the scapular/back contour.
    """
    if len(sections) < 2:
        raise ValueError("panel needs >= 2 sections")
    if thickness <= 0:
        raise ValueError("panel thickness must be > 0")
    sign = 1.0 if face == "front" else -1.0
    rows: list[list[np.ndarray]] = []
    for row_index, (z, x0, x1, base_y, bow_y) in enumerate(sections):
        row: list[np.ndarray] = []
        vertical = row_index / max(1, len(sections) - 1)
        for i in range(samples_across + 1):
            t = i / samples_across
            u = 2.0 * t - 1.0
            x = x0 + (x1 - x0) * t
            y = base_y + bow_y * (1.0 - u * u)
            if fold_amp and fold_cycles:
                # Longitudinal drape grows toward the hem while the panel edges
                # remain comparatively quiet.  This is explicit geometry, not
                # a normal-map or image texture.
                edge_fade = math.sin(math.pi * t) ** 1.4
                fold = fold_amp * vertical * edge_fade * math.sin(TAU * fold_cycles * t)
                y += sign * fold
            row.append(np.array([x, y, z], dtype=float))
        rows.append(row)

    outer: list[list[float]] = []
    inner: list[list[float]] = []
    for row in rows:
        for p in row:
            outer.append(p.tolist())
            q = p.copy()
            q[1] -= sign * thickness
            inner.append(q.tolist())

    vertices = outer + inner
    cols = samples_across + 1
    row_count = len(rows)
    inner_base = row_count * cols
    faces: list[list[int]] = []

    for r in range(row_count - 1):
        for c in range(cols - 1):
            a = r * cols + c
            b = a + 1
            d = (r + 1) * cols + c
            e = d + 1
            if face == "front":
                faces.extend([[a, e, b], [a, d, e]])
            else:
                faces.extend([[a, b, e], [a, e, d]])

            ia, ib, id_, ie = inner_base + a, inner_base + b, inner_base + d, inner_base + e
            if face == "front":
                faces.extend([[ia, ib, ie], [ia, ie, id_]])
            else:
                faces.extend([[ia, ie, ib], [ia, id_, ie]])

    # Close left/right boundaries.
    for r in range(row_count - 1):
        for c in (0, cols - 1):
            a = r * cols + c
            b = (r + 1) * cols + c
            ia, ib = inner_base + a, inner_base + b
            if c == 0:
                faces.extend([[a, ia, ib], [a, ib, b]])
            else:
                faces.extend([[a, b, ib], [a, ib, ia]])

    # Close top/bottom boundaries.
    for r in (0, row_count - 1):
        base = r * cols
        for c in range(cols - 1):
            a, b = base + c, base + c + 1
            ia, ib = inner_base + a, inner_base + b
            if r == 0:
                faces.extend([[a, b, ib], [a, ib, ia]])
            else:
                faces.extend([[a, ib, b], [a, ia, ib]])

    return _part(
        name,
        np.asarray(vertices),
        np.asarray(faces),
        material,
        group=group,
        role=role,
        metadata={
            "generator": "curved_panel",
            "sections": len(sections),
            "samples_across": samples_across,
            "thickness_mm": thickness,
            "face": face,
            "fold_amp_mm": fold_amp,
            "fold_cycles": fold_cycles,
        },
    )


def _polygon_panel(
    name: str,
    polygon_xz: Sequence[tuple[float, float]],
    y: float,
    depth: float,
    material: int,
    *,
    group: str,
    role: str,
    front: bool = True,
) -> Part:
    """Extruded X/Z polygon at fixed Y."""
    poly = np.asarray(polygon_xz, dtype=np.float64)
    if len(poly) < 3:
        raise ValueError("polygon panel needs >= 3 points")
    y0, y1 = (y + depth / 2.0, y - depth / 2.0) if front else (y - depth / 2.0, y + depth / 2.0)
    vertices = [[x, y0, z] for x, z in poly] + [[x, y1, z] for x, z in poly]
    n = len(poly)
    faces: list[list[int]] = []
    for i in range(1, n - 1):
        faces.append([0, i + 1, i])
        faces.append([n, n + i, n + i + 1])
    for i in range(n):
        j = (i + 1) % n
        faces.extend([[i, j, n + j], [i, n + j, n + i]])
    return _part(name, np.asarray(vertices), np.asarray(faces), material, group=group, role=role)


def _ring_buckle(
    name: str,
    center: Sequence[float],
    outer: tuple[float, float],
    bar: float,
    depth: float,
    material: int,
    *,
    group: str = "hardware",
    role: str = "Rectangular metal buckle",
) -> list[Part]:
    """Build a rectangular buckle from four bars."""
    cx, cy, cz = map(float, center)
    ow, oh = outer
    parts = [
        _box(f"{name}_top", (cx, cy, cz + 0.5 * (oh - bar)), (ow, depth, bar), material, group=group, role=role),
        _box(f"{name}_bottom", (cx, cy, cz - 0.5 * (oh - bar)), (ow, depth, bar), material, group=group, role=role),
        _box(f"{name}_left", (cx - 0.5 * (ow - bar), cy, cz), (bar, depth, oh - 2 * bar), material, group=group, role=role),
        _box(f"{name}_right", (cx + 0.5 * (ow - bar), cy, cz), (bar, depth, oh - 2 * bar), material, group=group, role=role),
    ]
    return parts


def _mannequin(parts: list[Part]) -> None:
    """Slender faceless dress form with relaxed arms close to the body."""
    role = "Faceless procedural dress-form geometry; not a scan"

    parts.append(
        _z_loft(
            "mannequin_torso",
            [
                (920, 224, 146, 0, -4),
                (1040, 235, 152, 0, -2),
                (1150, 210, 145, 0, 0),
                (1280, 248, 157, 0, 0),
                (1415, 294, 171, 0, -3),
                (1495, 274, 158, 0, -7),
            ],
            MAT_MANNEQUIN, group="mannequin", role=role, radial=64,
        )
    )
    parts.append(
        _z_loft(
            "mannequin_pelvis",
            [(760, 205, 145, 0, -7), (865, 238, 160, 0, -3), (955, 232, 156, 0, 0)],
            MAT_MANNEQUIN, group="mannequin", role=role, radial=56,
        )
    )
    parts.append(
        _z_loft(
            "mannequin_neck",
            [(1490, 63, 58, 0, -6), (1605, 68, 62, 0, -7)],
            MAT_MANNEQUIN, group="mannequin", role=role, radial=36,
        )
    )
    parts.append(
        _ellipsoid(
            "mannequin_head", (0, -12, 1735), (98, 87, 136),
            MAT_MANNEQUIN, group="mannequin", role=role, subdivisions=4,
        )
    )

    for side, sx in (("left", -1.0), ("right", 1.0)):
        path = [
            (sx * 278, -2, 1438),
            (sx * 325, 5, 1320),
            (sx * 350, 14, 1180),
            (sx * 365, 22, 1035),
            (sx * 368, 28, 930),
        ]
        parts.append(
            _tube(
                f"mannequin_{side}_arm", path,
                [(67, 63), (63, 59), (57, 53), (49, 46), (42, 39)],
                MAT_MANNEQUIN, group="mannequin", role=role, radial=30,
            )
        )
        parts.append(
            _ellipsoid(
                f"mannequin_{side}_hand", (sx * 368, 38, 842), (40, 29, 90),
                MAT_MANNEQUIN, group="mannequin", role=role, subdivisions=2,
            )
        )

    for side, x in (("left", -108), ("right", 108)):
        parts.append(
            _z_loft(
                f"mannequin_{side}_leg",
                [
                    (145, 72, 65, x, -10),
                    (380, 80, 71, x, -8),
                    (610, 90, 80, x, -4),
                    (790, 99, 88, x, 0),
                    (955, 112, 98, x, 0),
                ],
                MAT_MANNEQUIN, group="mannequin", role=role, radial=44,
            )
        )

def _underlayer(parts: list[Part]) -> None:
    parts.append(
        _z_loft(
            "high_neck_underlayer",
            [
                (930, 244, 158, 0, 0),
                (1080, 224, 148, 0, 2),
                (1225, 246, 156, 0, 3),
                (1385, 294, 173, 0, 0),
                (1485, 272, 157, 0, -4),
            ],
            MAT_TECH, group="underlayer",
            role="Fitted technical-knit underlayer", radial=72,
        )
    )
    parts.append(
        _z_loft(
            "underlayer_turtleneck",
            [(1475, 84, 73, 0, -3), (1598, 82, 71, 0, -5)],
            MAT_TECH, group="underlayer", role="High technical turtleneck", radial=44,
        )
    )
    parts.append(
        _box(
            "throat_zipper", (0, 79, 1535), (8, 6, 78),
            MAT_METAL, group="hardware", role="Minimal throat zipper hardware",
        )
    )

def _coat_panels(parts: list[Part]) -> None:
    """Tailored coat assembled from fitted upper panels and tapered long tails."""

    # Fitted upper front.  Low bow keeps the cloth close to the torso instead
    # of forming the rectangular chest seen in earlier passes.
    parts.append(
        _curved_panel(
            "coat_front_left_upper",
            [
                (985, -272, -58, 160, 12),
                (1110, -254, -54, 166, 14),
                (1260, -274, -48, 176, 16),
                (1400, -302, -96, 181, 14),
                (1490, -286, -136, 174, 10),
            ],
            MAT_WOOL, group="coat", role="Tailored left front coat panel",
            thickness=6.5, samples_across=16, face="front",
        )
    )
    parts.append(
        _curved_panel(
            "coat_front_right_upper",
            [
                (985, 58, 272, 160, 12),
                (1110, 54, 254, 166, 14),
                (1260, 48, 274, 176, 16),
                (1400, 96, 302, 181, 14),
                (1490, 136, 286, 174, 10),
            ],
            MAT_WOOL, group="coat", role="Tailored right front coat panel",
            thickness=6.5, samples_across=16, face="front",
        )
    )

    # Narrow side pieces close the torso without broadening the silhouette.
    for side, sx in (("left", -1.0), ("right", 1.0)):
        sections = [
            (985, -304, -246, 92, 6),
            (1110, -294, -232, 100, 7),
            (1260, -316, -252, 106, 7),
            (1400, -326, -274, 100, 6),
            (1488, -300, -268, 90, 5),
        ]
        if sx > 0:
            sections = [(z, -x1, -x0, y, bow) for z, x0, x1, y, bow in sections]
        parts.append(
            _curved_panel(
                f"coat_{side}_side_bodice", sections,
                MAT_WOOL, group="coat", role="Narrow sculpted coat side panel",
                thickness=6, samples_across=7, face="front",
            )
        )

    # Designed rear with a narrow technical centre panel and shaped side backs.
    parts.append(
        _curved_panel(
            "coat_back_center",
            [
                (975, -132, 132, -168, -9),
                (1110, -128, 128, -176, -10),
                (1260, -145, 145, -184, -11),
                (1400, -174, 174, -188, -11),
                (1490, -162, 162, -174, -8),
            ],
            MAT_TECH, group="coat_back", role="Structured centre-back textile panel",
            thickness=6.5, samples_across=16, face="back",
        )
    )
    for side, sx in (("left", -1.0), ("right", 1.0)):
        sections = [
            (975, -296, -132, -154, -8),
            (1110, -286, -128, -164, -9),
            (1260, -302, -145, -174, -10),
            (1400, -318, -172, -178, -10),
            (1490, -294, -160, -164, -8),
        ]
        if sx > 0:
            sections = [(z, -x1, -x0, y, bow) for z, x0, x1, y, bow in sections]
        parts.append(
            _curved_panel(
                f"coat_back_{side}", sections,
                MAT_WOOL, group="coat_back", role="Tailored side-back coat panel",
                thickness=6, samples_across=11, face="back",
            )
        )

    # Long tails: narrower at the hip and only gently flared at the hem.
    tails = {
        "coat_left_front_skirt": (
            [
                (990, -268, -70, 157, 10),
                (800, -286, -90, 145, 9),
                (600, -306, -120, 125, 7),
                (410, -324, -150, 98, 5),
                (255, -338, -176, 70, 3),
            ], "front"
        ),
        "coat_right_front_skirt": (
            [
                (990, 70, 268, 157, 10),
                (800, 90, 286, 145, 9),
                (600, 120, 306, 125, 7),
                (410, 150, 324, 98, 5),
                (255, 176, 338, 70, 3),
            ], "front"
        ),
        "coat_left_back_tail": (
            [
                (975, -292, -30, -151, -8),
                (790, -304, -34, -142, -7),
                (600, -319, -40, -123, -6),
                (410, -330, -48, -96, -4),
                (250, -340, -56, -70, -2),
            ], "back"
        ),
        "coat_right_back_tail": (
            [
                (975, 30, 292, -151, -8),
                (790, 34, 304, -142, -7),
                (600, 40, 319, -123, -6),
                (410, 48, 330, -96, -4),
                (250, 56, 340, -70, -2),
            ], "back"
        ),
    }
    for name, (sections, face) in tails.items():
        parts.append(
            _curved_panel(
                name, sections, MAT_WOOL, group="coat_skirt",
                role="Tapered long coat tail", thickness=6.0,
                samples_across=20, face=face, fold_amp=9.0, fold_cycles=2.0,
            )
        )

    # Upper rear skirt bridge covers the seat; the centre vent begins below it.
    parts.append(
        _curved_panel(
            "coat_back_vent_bridge",
            [
                (975, -92, 92, -158, -6),
                (880, -86, 86, -151, -5),
                (790, -74, 74, -142, -4),
                (710, -60, 60, -132, -3),
            ],
            MAT_WOOL, group="coat_skirt", role="Upper rear skirt bridge above centre vent",
            thickness=6.0, samples_across=10, face="back",
        )
    )

    # Deep amethyst lining is restricted to narrow slivers inside openings.
    for name, sections, face in (
        (
            "left_lining_flash",
            [(955, -73, -43, 149, 4), (720, -68, -39, 128, 3), (470, -62, -35, 96, 2), (275, -56, -32, 65, 1)],
            "front",
        ),
        (
            "right_lining_flash",
            [(955, 43, 73, 149, 4), (720, 39, 68, 128, 3), (470, 35, 62, 96, 2), (275, 32, 56, 65, 1)],
            "front",
        ),
        (
            "rear_vent_lining",
            [(900, -38, 38, -155, -3), (650, -44, 44, -134, -2), (430, -50, 50, -100, -2), (265, -55, 55, -67, -1)],
            "back",
        ),
    ):
        parts.append(
            _curved_panel(
                name, sections, MAT_PURPLE_SATIN, group="lining",
                role="Restrained deep-amethyst lining reveal",
                thickness=2.5, samples_across=6, face=face,
            )
        )

def _lapels_and_collar(parts: list[Part]) -> None:
    # Lapels are deliberately narrow and sit just outside the coat surface.
    parts.append(
        _polygon_panel(
            "left_lapel",
            [(-284, 1488), (-164, 1472), (-82, 1372), (-132, 1238), (-232, 1315)],
            205, 5.5, MAT_LEATHER, group="coat_trim",
            role="Narrow structured black-leather lapel",
        )
    )
    parts.append(
        _polygon_panel(
            "right_lapel",
            [(284, 1488), (174, 1470), (95, 1368), (148, 1240), (238, 1318)],
            205, 5.5, MAT_PURPLE_LEATHER, group="coat_trim",
            role="Narrow deep-amethyst leather lapel",
        )
    )
    parts.append(
        _oriented_box(
            "right_lapel_outer_binding",
            (276, 211, 1475), (150, 211, 1248),
            10, 5, MAT_LEATHER, group="coat_trim",
            role="Black binding on purple lapel",
        )
    )

    # Close-fitting open standing collar around the back and sides of the neck.
    parts.append(
        _curved_panel(
            "collar_back",
            [(1495, -112, 112, -160, -6), (1555, -103, 103, -153, -5), (1608, -88, 88, -140, -3)],
            MAT_WOOL, group="coat_trim", role="Close standing collar back",
            thickness=5.5, samples_across=14, face="back",
        )
    )
    parts.append(
        _polygon_panel(
            "left_collar_wing",
            [(-172, 1500), (-126, 1548), (-90, 1594), (-94, 1540), (-134, 1494)],
            173, 5.5, MAT_WOOL, group="coat_trim", role="Left collar wing",
        )
    )
    parts.append(
        _polygon_panel(
            "right_collar_wing",
            [(172, 1500), (126, 1548), (90, 1594), (94, 1540), (134, 1494)],
            173, 5.5, MAT_WOOL, group="coat_trim", role="Right collar wing",
        )
    )
    parts.append(
        _polygon_panel(
            "right_collar_amethyst_underlay",
            [(157, 1502), (122, 1539), (102, 1568), (105, 1540), (132, 1501)],
            166, 3, MAT_PURPLE_SATIN, group="lining",
            role="Restrained amethyst collar under-edge",
        )
    )

def _sleeves_gloves_cuffs(parts: list[Part]) -> None:
    for side, sx in (("left", -1.0), ("right", 1.0)):
        path = [
            (sx * 281, -2, 1438),
            (sx * 326, 6, 1320),
            (sx * 350, 15, 1180),
            (sx * 365, 23, 1035),
            (sx * 369, 29, 944),
        ]
        parts.append(
            _tube(
                f"coat_{side}_sleeve", path,
                [(79, 74), (74, 69), (66, 62), (56, 52), (48, 44)],
                MAT_WOOL, group="sleeves", role="Slim tailored coat sleeve", radial=36,
            )
        )
        for k, z in enumerate((995, 965)):
            parts.append(
                _z_loft(
                    f"{side}_cuff_band_{k+1}",
                    [(z - 9, 52, 48, sx * 368, 29), (z + 9, 53, 49, sx * 368, 29)],
                    MAT_LEATHER, group="cuffs", role="Leather cuff strap", radial=28,
                )
            )
        parts.extend(
            _ring_buckle(
                f"{side}_cuff_buckle", (sx * 406, 62, 968),
                (25, 21), 4, 6, MAT_METAL, role="Cuff buckle",
            )
        )
        parts.append(
            _ellipsoid(
                f"{side}_glove_palm", (sx * 368, 39, 842), (42, 31, 89),
                MAT_LEATHER, group="gloves", role="Tailored leather glove palm", subdivisions=2,
            )
        )
        for idx, off in enumerate((-21, -7, 7, 21)):
            x = sx * (368 + off)
            parts.append(
                _tube(
                    f"{side}_glove_finger_{idx+1}",
                    [(x, 55, 812), (x, 58, 765)], [(7, 6), (6, 5)],
                    MAT_LEATHER, group="gloves", role="Procedural glove finger", radial=10,
                )
            )

def _epaulettes(parts: list[Part]) -> None:
    """Compact layered shoulder straps inspired by couture epaulettes, not armor."""
    for side, sx in (("left", -1.0), ("right", 1.0)):
        straps = [
            ((sx * 278, 18, 1477), (sx * 340, 16, 1454), 20),
            ((sx * 286, 18, 1454), (sx * 350, 16, 1428), 18),
            ((sx * 294, 18, 1432), (sx * 356, 16, 1404), 16),
        ]
        for i, (a, b, width) in enumerate(straps):
            parts.append(
                _oriented_box(
                    f"{side}_epaulette_strap_{i+1}", a, b, width, 6,
                    MAT_LEATHER, group="epaulettes",
                    role="Compact layered leather shoulder strap",
                )
            )
            parts.extend(
                _ring_buckle(
                    f"{side}_epaulette_buckle_{i+1}",
                    (b[0], 25, b[2]), (21, 18), 4, 5,
                    MAT_METAL, role="Small shoulder buckle",
                )
            )

def _harness_and_belt(parts: list[Part]) -> None:
    parts.append(
        _oriented_box(
            "front_diagonal_harness",
            (-205, 218, 1455), (158, 211, 1085),
            27, 8, MAT_LEATHER, group="harness",
            role="Refined diagonal leather harness",
        )
    )
    parts.append(
        _oriented_box(
            "back_diagonal_harness",
            (-190, -212, 1448), (160, -210, 1100),
            25, 7, MAT_LEATHER, group="harness",
            role="Back continuation of diagonal harness",
        )
    )
    parts.extend(
        _ring_buckle(
            "front_harness_buckle", (-20, 224, 1268), (36, 31), 6, 6,
            MAT_METAL, role="Primary harness buckle",
        )
    )
    parts.extend(
        _ring_buckle(
            "back_harness_buckle", (14, -223, 1268), (34, 29), 6, 6,
            MAT_METAL, role="Back harness buckle",
        )
    )

    parts.append(
        _curved_panel(
            "front_waist_belt",
            [(1028, -250, 250, 195, 9), (1064, -250, 250, 195, 9)],
            MAT_LEATHER, group="harness", role="Tailored front waist belt",
            thickness=7, samples_across=18, face="front",
        )
    )
    parts.append(
        _curved_panel(
            "back_waist_belt",
            [(1028, -250, 250, -196, -9), (1064, -250, 250, -196, -9)],
            MAT_LEATHER, group="harness", role="Tailored back waist belt",
            thickness=7, samples_across=18, face="back",
        )
    )
    parts.extend(
        _ring_buckle(
            "waist_buckle", (62, 211, 1046), (48, 35), 7, 7,
            MAT_METAL, role="Primary waist buckle",
        )
    )
    parts.append(
        _oriented_box(
            "right_waist_hanging_tab",
            (274, 202, 1030), (280, 188, 858),
            28, 7, MAT_LEATHER, group="harness", role="Asymmetric hanging waist tab",
        )
    )

def _back_pattern_and_seams(parts: list[Part]) -> None:
    # Very low-relief centre-back construction.
    parts.append(
        _oriented_box(
            "back_spine_binding", (0, -208, 1478), (0, -205, 1000),
            18, 4.5, MAT_LEATHER, group="back_detail",
            role="Narrow centre-back leather spine",
        )
    )

    # Small sparse diamonds suggest stitched quilting rather than a cage.
    diamond_centres = [
        (-72.0, 1395.0), (0.0, 1395.0), (72.0, 1395.0),
        (-72.0, 1315.0), (0.0, 1315.0), (72.0, 1315.0),
        (-72.0, 1235.0), (0.0, 1235.0), (72.0, 1235.0),
    ]
    for index, (cx, cz) in enumerate(diamond_centres):
        top = (cx, -207.5, cz + 19.0)
        right = (cx + 16.0, -207.5, cz)
        bottom = (cx, -207.5, cz - 19.0)
        left = (cx - 16.0, -207.5, cz)
        for edge_index, (a, b) in enumerate(((top, right), (right, bottom), (bottom, left), (left, top))):
            parts.append(
                _oriented_box(
                    f"back_diamond_{index:02d}_edge_{edge_index}",
                    a, b, 1.5, 1.5, MAT_STITCH,
                    group="back_detail", role="Low-relief stitched diamond",
                )
            )

    seam_paths = {
        "left_front_princess_seam": [(-218, 198, 1430), (-202, 198, 1260), (-180, 190, 1060), (-155, 157, 760)],
        "right_front_princess_seam": [(218, 198, 1430), (202, 198, 1260), (180, 190, 1060), (155, 157, 760)],
        "left_back_princess_seam": [(-205, -200, 1430), (-187, -202, 1260), (-170, -195, 1060), (-150, -158, 760)],
        "right_back_princess_seam": [(205, -200, 1430), (187, -202, 1260), (170, -195, 1060), (150, -158, 760)],
    }
    for name, path in seam_paths.items():
        parts.append(
            _tube(
                name, path, (2.0, 2.0), MAT_STITCH,
                group="seams", role="Subtle raised seam geometry", radial=8,
            )
        )

def _trousers(parts: list[Part]) -> None:
    # Waist and hip shell prevents the dress form from showing through the coat opening.
    parts.append(
        _z_loft(
            "trouser_waist_and_hips",
            [
                (835, 214, 146, 0, -4),
                (900, 226, 152, 0, -2),
                (965, 220, 148, 0, 0),
                (1005, 205, 142, 0, 0),
            ],
            MAT_TECH, group="trousers", role="Tailored technical trouser waist and hip shell", radial=56,
        )
    )
    for side, x in (("left", -108), ("right", 108)):
        parts.append(
            _z_loft(
                f"{side}_trouser_leg",
                [
                    (170, 72, 64, x, -10),
                    (340, 75, 66, x, -8),
                    (530, 80, 70, x, -5),
                    (720, 89, 78, x, -2),
                    (900, 104, 90, x, 0),
                    (970, 110, 95, x, 0),
                ],
                MAT_TECH, group="trousers",
                role="Slim tailored technical trouser leg", radial=48,
            )
        )
        sx = -1.0 if side == "left" else 1.0
        parts.append(
            _oriented_box(
                f"{side}_trouser_outer_binding",
                (x + sx * 76, 9, 890), (x + sx * 62, 7, 235),
                10, 4, MAT_LEATHER, group="trousers",
                role="Narrow leather trouser side binding",
            )
        )
        if side == "left":
            parts.append(
                _z_loft(
                    "left_thigh_strap",
                    [(675, 96, 82, x, -1), (700, 97, 83, x, -1)],
                    MAT_LEATHER, group="trousers",
                    role="Asymmetric leather thigh strap", radial=38,
                )
            )
            parts.extend(
                _ring_buckle(
                    "left_thigh_buckle", (x - 84, 27, 687),
                    (25, 21), 4, 5, MAT_METAL, role="Thigh strap buckle",
                )
            )

def _boots(parts: list[Part]) -> None:
    for side, x in (("left", -108), ("right", 108)):
        parts.append(
            _z_loft(
                f"{side}_boot_shaft",
                [(48, 86, 73, x, -2), (135, 84, 71, x, -3), (225, 80, 67, x, -5), (300, 76, 64, x, -7)],
                MAT_LEATHER, group="footwear", role="Slim leather boot shaft", radial=42,
            )
        )
        parts.append(
            _y_loft(
                f"{side}_boot_upper",
                [
                    (-80, 71, 43, x, 66),
                    (-10, 87, 52, x, 67),
                    (100, 92, 58, x, 66),
                    (205, 88, 53, x, 62),
                    (285, 73, 41, x, 56),
                    (325, 48, 27, x, 51),
                ],
                MAT_LEATHER, group="footwear",
                role="Tapered constructed leather boot upper", radial=42,
            )
        )
        parts.append(
            _y_loft(
                f"{side}_boot_sole",
                [
                    (-86, 82, 14, x, 22),
                    (-5, 96, 14, x, 22),
                    (110, 100, 14, x, 22),
                    (220, 94, 13, x, 22),
                    (330, 61, 11, x, 22),
                ],
                MAT_RUBBER, group="footwear", role="Tapered rubber outsole", radial=38,
            )
        )
        parts.append(
            _box(
                f"{side}_boot_heel", (x, -47, 36), (140, 88, 54),
                MAT_RUBBER, group="footwear", role="Compact architectural boot heel",
            )
        )
        for idx, (y, z) in enumerate(((88, 122), (18, 210))):
            parts.append(
                _oriented_box(
                    f"{side}_boot_strap_{idx+1}",
                    (x - 78, y, z), (x + 78, y + 1, z),
                    15, 6, MAT_LEATHER, group="footwear", role="Leather boot strap",
                )
            )
            parts.extend(
                _ring_buckle(
                    f"{side}_boot_buckle_{idx+1}",
                    (x + 68, y + 11, z), (23, 19), 4, 5,
                    MAT_METAL, role="Boot strap buckle",
                )
            )
        parts.append(
            _polygon_panel(
                f"{side}_toe_cap",
                [(x - 58, 38), (x + 58, 38), (x + 65, 77), (x, 98), (x - 65, 77)],
                300, 14, MAT_LEATHER, group="footwear",
                role="Angular polished leather toe cap",
            )
        )

def build() -> Assembly:
    materials = [
        Material(
            name="Mannequin graphite",
            color=(0.045, 0.048, 0.055),
            metal=0.0,
            rough=0.78,
            microfinish="matte dress form",
            material_source="procedural",
        ),
        Material(
            name="Nocturne black wool",
            color=(0.018, 0.020, 0.026),
            metal=0.0,
            rough=0.63,
            coat=0.035,
            coat_rough=0.42,
            microfinish="dense tailored wool",
            material_source="procedural",
        ),
        Material(
            name="Graphite technical textile",
            color=(0.028, 0.033, 0.042),
            metal=0.0,
            rough=0.54,
            coat=0.04,
            coat_rough=0.35,
            microfinish="fine technical weave",
            material_source="procedural",
        ),
        Material(
            name="Deep amethyst satin",
            color=(0.090, 0.018, 0.135),
            metal=0.0,
            rough=0.20,
            coat=0.20,
            coat_rough=0.16,
            microfinish="dense satin lining",
            material_source="procedural",
        ),
        Material(
            name="Black calf leather",
            color=(0.020, 0.021, 0.024),
            metal=0.0,
            rough=0.27,
            coat=0.24,
            coat_rough=0.18,
            microfinish="fine grained leather",
            material_source="procedural",
        ),
        Material(
            name="Brushed gunmetal hardware",
            color=(0.12, 0.115, 0.105),
            metal=0.98,
            rough=0.16,
            anisotropy=0.72,
            microfinish="brushed gunmetal",
            material_source="procedural",
        ),
        Material(
            name="Deep amethyst leather",
            color=(0.105, 0.022, 0.155),
            metal=0.0,
            rough=0.24,
            coat=0.22,
            coat_rough=0.16,
            microfinish="dyed polished leather",
            material_source="procedural",
        ),
        Material(
            name="Carbon rubber",
            color=(0.008, 0.010, 0.014),
            metal=0.0,
            rough=0.92,
            microfinish="dense molded rubber",
            material_source="procedural",
        ),
        Material(
            name="Black seam binding",
            color=(0.030, 0.032, 0.038),
            metal=0.0,
            rough=0.40,
            coat=0.08,
            coat_rough=0.30,
            microfinish="raised textile stitch/binding",
            material_source="procedural",
        ),
    ]

    parts: list[Part] = []
    _mannequin(parts)
    _underlayer(parts)
    _coat_panels(parts)
    _lapels_and_collar(parts)
    _sleeves_gloves_cuffs(parts)
    _epaulettes(parts)
    _harness_and_belt(parts)
    _back_pattern_and_seams(parts)
    _trousers(parts)
    _boots(parts)

    metadata = {
        "design": "CYBR NOCTURNE v3.1 / tailored procedural couture",
        "authoring": "procedural geometry only",
        "reference_intent": "Reconstruct the high-fashion NOCTURNE design language as geometry, not as generated pixels",
        "units": "mm",
        "front_axis": "+Y",
        "up_axis": "+Z",
        "design_height_mm": 1862,
        "render_pair": "CYBR GEO geometry + CYBR LIGHT spectral path tracing",
        "construction": {
            "panel_based_coat": True,
            "separate_front_back_tail_panels": True,
            "rear_centre_vent": True,
            "separate_lining": True,
            "procedural_back_lattice": True,
            "directional_footwear_lofts": True,
            "named_harness_and_hardware": True,
        },
        "limitations": [
            "Fashion visualization geometry, not a production sewing pattern.",
            "No body scan, photogrammetry, image-generated mesh, or texture bake.",
            "No cloth FEM is solved here; the panel curvature and fold hierarchy are authored procedurally.",
        ],
    }
    return Assembly("CYBR NOCTURNE", parts, materials, metadata=metadata)


if __name__ == "__main__":
    assembly = build()
    print(assembly.validate())
