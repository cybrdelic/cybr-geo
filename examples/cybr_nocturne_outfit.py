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
    for z, x0, x1, base_y, bow_y in sections:
        row: list[np.ndarray] = []
        for i in range(samples_across + 1):
            t = i / samples_across
            u = 2.0 * t - 1.0
            x = x0 + (x1 - x0) * t
            y = base_y + bow_y * (1.0 - u * u)
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
    """Slim faceless dress form with a relaxed neutral stance."""
    role = "Faceless procedural dress-form geometry; not a scan"

    # Torso: narrower waist, realistic shoulder taper.
    parts.append(
        _z_loft(
            "mannequin_torso",
            [
                (910, 238, 150, 0, -4),
                (1030, 250, 158, 0, -2),
                (1140, 225, 150, 0, 0),
                (1280, 270, 165, 0, 0),
                (1430, 315, 178, 0, -3),
                (1510, 292, 162, 0, -8),
            ],
            MAT_MANNEQUIN,
            group="mannequin",
            role=role,
            radial=64,
        )
    )
    parts.append(
        _z_loft(
            "mannequin_pelvis",
            [
                (760, 220, 155, 0, -6),
                (865, 265, 175, 0, -2),
                (950, 255, 168, 0, 0),
            ],
            MAT_MANNEQUIN,
            group="mannequin",
            role=role,
            radial=56,
        )
    )
    parts.append(
        _z_loft(
            "mannequin_neck",
            [(1500, 67, 61, 0, -6), (1600, 74, 67, 0, -6)],
            MAT_MANNEQUIN,
            group="mannequin",
            role=role,
            radial=36,
        )
    )
    parts.append(
        _ellipsoid(
            "mannequin_head",
            (0, -12, 1724),
            (101, 90, 138),
            MAT_MANNEQUIN,
            group="mannequin",
            role=role,
            subdivisions=4,
        )
    )

    # Arms closer to the body than v1.
    for side, sx in (("left", -1.0), ("right", 1.0)):
        path = [
            (sx * 292, -2, 1445),
            (sx * 350, 5, 1320),
            (sx * 382, 18, 1180),
            (sx * 400, 28, 1030),
            (sx * 402, 35, 930),
        ]
        parts.append(
            _tube(
                f"mannequin_{side}_arm",
                path,
                [(72, 67), (67, 62), (61, 57), (53, 49), (45, 42)],
                MAT_MANNEQUIN,
                group="mannequin",
                role=role,
                radial=30,
            )
        )
        parts.append(
            _ellipsoid(
                f"mannequin_{side}_hand",
                (sx * 402, 46, 842),
                (43, 31, 92),
                MAT_MANNEQUIN,
                group="mannequin",
                role=role,
                subdivisions=2,
            )
        )

    for side, x in (("left", -116), ("right", 116)):
        parts.append(
            _z_loft(
                f"mannequin_{side}_leg",
                [
                    (150, 78, 70, x, -10),
                    (390, 87, 77, x, -8),
                    (620, 98, 87, x, -4),
                    (790, 108, 96, x, 0),
                    (950, 122, 108, x, 0),
                ],
                MAT_MANNEQUIN,
                group="mannequin",
                role=role,
                radial=44,
            )
        )


def _underlayer(parts: list[Part]) -> None:
    parts.append(
        _z_loft(
            "high_neck_underlayer",
            [
                (920, 268, 174, 0, 0),
                (1080, 245, 160, 0, 2),
                (1220, 268, 166, 0, 4),
                (1390, 312, 182, 0, 0),
                (1492, 290, 162, 0, -4),
            ],
            MAT_TECH,
            group="underlayer",
            role="Fitted technical-knit underlayer",
            radial=72,
        )
    )
    parts.append(
        _z_loft(
            "underlayer_turtleneck",
            [
                (1480, 93, 80, 0, -2),
                (1586, 90, 78, 0, -4),
            ],
            MAT_TECH,
            group="underlayer",
            role="High technical turtleneck",
            radial=44,
        )
    )
    # zipper / throat pull.
    parts.append(
        _box(
            "throat_zipper",
            (0, 86, 1534),
            (10, 8, 86),
            MAT_METAL,
            group="hardware",
            role="Minimal throat zipper hardware",
        )
    )


def _coat_panels(parts: list[Part]) -> None:
    """Tailored coat body from separate front/back panels."""

    # Upper front panels are fitted to the torso and leave a deliberate centre opening.
    parts.append(
        _curved_panel(
            "coat_front_left_upper",
            [
                (980, -305, -58, 166, 42),
                (1110, -292, -58, 172, 48),
                (1270, -315, -48, 180, 54),
                (1420, -336, -82, 182, 50),
                (1515, -320, -118, 170, 42),
            ],
            MAT_WOOL,
            group="coat",
            role="Tailored left front coat panel",
            thickness=7.5,
            samples_across=14,
            face="front",
        )
    )
    parts.append(
        _curved_panel(
            "coat_front_right_upper",
            [
                (980, 64, 305, 166, 42),
                (1110, 64, 292, 172, 48),
                (1270, 54, 315, 180, 54),
                (1420, 88, 336, 182, 50),
                (1515, 124, 320, 170, 42),
            ],
            MAT_WOOL,
            group="coat",
            role="Tailored right front coat panel",
            thickness=7.5,
            samples_across=14,
            face="front",
        )
    )

    # Side panels narrow the waist and flare below the hip.
    for side, sx in (("left", -1.0), ("right", 1.0)):
        parts.append(
            _curved_panel(
                f"coat_{side}_side_bodice",
                [
                    (960, sx * 300, sx * 215, 86, 18),
                    (1110, sx * 290, sx * 205, 102, 24),
                    (1280, sx * 318, sx * 218, 112, 28),
                    (1425, sx * 342, sx * 250, 102, 22),
                    (1510, sx * 324, sx * 270, 86, 18),
                ] if sx < 0 else [
                    (960, 215, 300, 86, 18),
                    (1110, 205, 290, 102, 24),
                    (1280, 218, 318, 112, 28),
                    (1425, 250, 342, 102, 22),
                    (1510, 270, 324, 86, 18),
                ],
                MAT_WOOL,
                group="coat",
                role="Sculpted coat side panel",
                thickness=7,
                samples_across=8,
                face="front",
            )
        )

    # Back: intentionally designed, not a black trapezoid.
    parts.append(
        _curved_panel(
            "coat_back_center",
            [
                (930, -142, 142, -168, -30),
                (1080, -136, 136, -176, -38),
                (1260, -160, 160, -188, -46),
                (1430, -205, 205, -192, -48),
                (1520, -186, 186, -175, -36),
            ],
            MAT_TECH,
            group="coat_back",
            role="Structured patterned center-back panel",
            thickness=7,
            samples_across=14,
            face="back",
        )
    )
    for side, sx in (("left", -1.0), ("right", 1.0)):
        if sx < 0:
            sections = [
                (930, -310, -140, -150, -26),
                (1080, -298, -134, -160, -30),
                (1260, -320, -158, -172, -36),
                (1430, -340, -202, -176, -38),
                (1515, -320, -184, -162, -30),
            ]
        else:
            sections = [
                (930, 140, 310, -150, -26),
                (1080, 134, 298, -160, -30),
                (1260, 158, 320, -172, -36),
                (1430, 202, 340, -176, -38),
                (1515, 184, 320, -162, -30),
            ]
        parts.append(
            _curved_panel(
                f"coat_back_{side}",
                sections,
                MAT_WOOL,
                group="coat_back",
                role="Tailored side-back coat panel",
                thickness=7,
                samples_across=10,
                face="back",
            )
        )

    # Long skirt/tails. Front is open; rear has a centre vent.
    parts.append(
        _curved_panel(
            "coat_left_front_skirt",
            [
                (980, -330, -64, 158, 34),
                (800, -352, -54, 150, 30),
                (600, -395, -40, 132, 20),
                (390, -430, -22, 105, 10),
                (260, -445, -10, 76, 4),
            ],
            MAT_WOOL,
            group="coat_skirt",
            role="Long left front coat skirt",
            thickness=7,
            samples_across=14,
            face="front",
        )
    )
    parts.append(
        _curved_panel(
            "coat_right_front_skirt",
            [
                (980, 64, 330, 158, 34),
                (800, 54, 352, 150, 30),
                (600, 40, 395, 132, 20),
                (390, 22, 430, 105, 10),
                (260, 10, 445, 76, 4),
            ],
            MAT_WOOL,
            group="coat_skirt",
            role="Long right front coat skirt",
            thickness=7,
            samples_across=14,
            face="front",
        )
    )
    parts.append(
        _curved_panel(
            "coat_left_back_tail",
            [
                (930, -312, -24, -152, -24),
                (760, -350, -28, -146, -22),
                (560, -392, -36, -130, -16),
                (360, -430, -48, -100, -8),
                (245, -442, -62, -76, -2),
            ],
            MAT_WOOL,
            group="coat_skirt",
            role="Left rear coat tail with centre vent",
            thickness=7,
            samples_across=14,
            face="back",
        )
    )
    parts.append(
        _curved_panel(
            "coat_right_back_tail",
            [
                (930, 24, 312, -152, -24),
                (760, 28, 350, -146, -22),
                (560, 36, 392, -130, -16),
                (360, 48, 430, -100, -8),
                (245, 62, 442, -76, -2),
            ],
            MAT_WOOL,
            group="coat_skirt",
            role="Right rear coat tail with centre vent",
            thickness=7,
            samples_across=14,
            face="back",
        )
    )

    # Purple lining exposed only at centre opening and rear vent.
    parts.append(
        _curved_panel(
            "left_lining_flash",
            [
                (940, -82, -42, 145, 12),
                (720, -80, -38, 132, 10),
                (480, -75, -34, 108, 8),
                (275, -70, -30, 70, 3),
            ],
            MAT_PURPLE_SATIN,
            group="lining",
            role="Deep amethyst front lining reveal",
            thickness=3,
            samples_across=4,
            face="front",
        )
    )
    parts.append(
        _curved_panel(
            "right_lining_flash",
            [
                (940, 42, 82, 145, 12),
                (720, 38, 80, 132, 10),
                (480, 34, 75, 108, 8),
                (275, 30, 70, 70, 3),
            ],
            MAT_PURPLE_SATIN,
            group="lining",
            role="Deep amethyst front lining reveal",
            thickness=3,
            samples_across=4,
            face="front",
        )
    )
    parts.append(
        _curved_panel(
            "rear_vent_lining",
            [
                (900, -48, 48, -158, -8),
                (660, -54, 54, -140, -8),
                (420, -64, 64, -108, -5),
                (260, -74, 74, -72, -2),
            ],
            MAT_PURPLE_SATIN,
            group="lining",
            role="Deep amethyst rear vent lining",
            thickness=3,
            samples_across=6,
            face="back",
        )
    )


def _lapels_and_collar(parts: list[Part]) -> None:
    # Wide, asymmetrical couture lapels.
    parts.append(
        _polygon_panel(
            "left_lapel",
            [(-316, 1512), (-132, 1488), (-54, 1360), (-118, 1200), (-292, 1325)],
            203,
            10,
            MAT_LEATHER,
            group="coat_trim",
            role="Structured black leather left lapel",
        )
    )
    parts.append(
        _polygon_panel(
            "right_lapel",
            [(318, 1512), (134, 1488), (62, 1365), (142, 1210), (300, 1330)],
            204,
            10,
            MAT_PURPLE_LEATHER,
            group="coat_trim",
            role="Deep amethyst leather right lapel",
        )
    )
    # Narrow dark border over purple lapel to integrate it.
    parts.append(
        _oriented_box(
            "right_lapel_outer_binding",
            (310, 210, 1504),
            (146, 214, 1218),
            13,
            8,
            MAT_LEATHER,
            group="coat_trim",
            role="Black binding on purple lapel",
        )
    )

    # Tall standing collar, built from front/back arcs.
    parts.append(
        _z_loft(
            "collar_outer",
            [
                (1495, 176, 126, 0, -12),
                (1560, 170, 122, 0, -20),
                (1625, 166, 116, 0, -28),
            ],
            MAT_WOOL,
            group="coat_trim",
            role="High sculpted standing collar",
            radial=60,
        )
    )
    # Purple collar under-edge peeks from behind.
    parts.append(
        _z_loft(
            "collar_amethyst_underlay",
            [
                (1508, 166, 116, 0, -14),
                (1565, 158, 110, 0, -20),
            ],
            MAT_PURPLE_SATIN,
            group="lining",
            role="Amethyst collar underlay",
            radial=56,
        )
    )


def _sleeves_gloves_cuffs(parts: list[Part]) -> None:
    for side, sx in (("left", -1.0), ("right", 1.0)):
        path = [
            (sx * 292, -2, 1445),
            (sx * 350, 7, 1320),
            (sx * 380, 20, 1180),
            (sx * 398, 30, 1030),
            (sx * 402, 37, 942),
        ]
        parts.append(
            _tube(
                f"coat_{side}_sleeve",
                path,
                [(86, 80), (79, 74), (71, 66), (61, 57), (52, 48)],
                MAT_WOOL,
                group="sleeves",
                role="Fitted tailored coat sleeve",
                radial=36,
            )
        )
        # leather cuff stack, restrained.
        for k, z in enumerate((1005, 970, 938)):
            parts.append(
                _z_loft(
                    f"{side}_cuff_band_{k+1}",
                    [
                        (z - 11, 58, 54, sx * 401, 34),
                        (z + 11, 60, 56, sx * 401, 34),
                    ],
                    MAT_LEATHER,
                    group="cuffs",
                    role="Layered leather cuff strap",
                    radial=28,
                )
            )
        parts.extend(
            _ring_buckle(
                f"{side}_cuff_buckle",
                (sx * 444, 76, 972),
                (30, 26),
                5,
                7,
                MAT_METAL,
                role="Cuff buckle",
            )
        )

        # Glove palm and finger group.
        parts.append(
            _ellipsoid(
                f"{side}_glove_palm",
                (sx * 402, 48, 842),
                (46, 34, 91),
                MAT_LEATHER,
                group="gloves",
                role="Tailored leather glove palm",
                subdivisions=2,
            )
        )
        finger_offsets = (-23, -8, 8, 23)
        for idx, off in enumerate(finger_offsets):
            x = sx * (402 + off)
            parts.append(
                _tube(
                    f"{side}_glove_finger_{idx+1}",
                    [(x, 66, 812), (x, 69, 762)],
                    [(8, 7), (7, 6)],
                    MAT_LEATHER,
                    group="gloves",
                    role="Procedural glove finger",
                    radial=12,
                )
            )


def _epaulettes(parts: list[Part]) -> None:
    """Layered leather shoulder treatment instead of bulky armor."""
    for side, sx in (("left", -1.0), ("right", 1.0)):
        # base pad
        if sx < 0:
            poly = [(-330, 1515), (-438, 1488), (-452, 1448), (-330, 1462)]
        else:
            poly = [(330, 1515), (438, 1488), (452, 1448), (330, 1462)]
        parts.append(
            _polygon_panel(
                f"{side}_epaulette_base",
                poly,
                18,
                12,
                MAT_LEATHER,
                group="epaulettes",
                role="Structured leather shoulder epaulette",
            )
        )
        # layered straps running over shoulder.
        for i, (inner_x, outer_x, z0, z1) in enumerate(
            [
                (318, 405, 1510, 1472),
                (332, 424, 1488, 1448),
                (348, 440, 1464, 1426),
            ]
        ):
            parts.append(
                _oriented_box(
                    f"{side}_epaulette_strap_{i+1}",
                    (sx * inner_x, -12, z0),
                    (sx * outer_x, 0, z1),
                    24,
                    9,
                    MAT_LEATHER,
                    group="epaulettes",
                    role="Layered shoulder leather strap",
                )
            )
            parts.extend(
                _ring_buckle(
                    f"{side}_epaulette_buckle_{i+1}",
                    (sx * (outer_x - 8), 18, z1 + 6),
                    (29, 24),
                    5,
                    7,
                    MAT_METAL,
                    role="Small shoulder buckle",
                )
            )


def _harness_and_belt(parts: list[Part]) -> None:
    # Front diagonal from left shoulder to right waist.
    parts.append(
        _oriented_box(
            "front_diagonal_harness",
            (-224, 218, 1480),
            (188, 220, 1060),
            34,
            11,
            MAT_LEATHER,
            group="harness",
            role="Refined diagonal leather harness",
        )
    )
    # Back continuation mirrors the visual language rather than leaving rear blank.
    parts.append(
        _oriented_box(
            "back_diagonal_harness",
            (-210, -218, 1470),
            (192, -218, 1090),
            32,
            10,
            MAT_LEATHER,
            group="harness",
            role="Back continuation of diagonal harness",
        )
    )
    parts.extend(
        _ring_buckle(
            "front_harness_buckle",
            (-26, 229, 1280),
            (44, 38),
            7,
            8,
            MAT_METAL,
            role="Primary diagonal harness buckle",
        )
    )
    parts.extend(
        _ring_buckle(
            "back_harness_buckle",
            (20, -230, 1285),
            (42, 36),
            7,
            8,
            MAT_METAL,
            role="Back harness buckle",
        )
    )
    # Tailored waist belt as discrete leather bands front/back and side connectors.
    parts.append(
        _curved_panel(
            "front_waist_belt",
            [(1030, -300, 300, 188, 20), (1072, -300, 300, 188, 20)],
            MAT_LEATHER,
            group="harness",
            role="Wide front waist belt",
            thickness=10,
            samples_across=18,
            face="front",
        )
    )
    parts.append(
        _curved_panel(
            "back_waist_belt",
            [(1030, -300, 300, -182, -18), (1072, -300, 300, -182, -18)],
            MAT_LEATHER,
            group="harness",
            role="Wide back waist belt",
            thickness=10,
            samples_across=18,
            face="back",
        )
    )
    parts.extend(
        _ring_buckle(
            "waist_buckle",
            (72, 210, 1050),
            (58, 42),
            8,
            9,
            MAT_METAL,
            role="Primary waist buckle",
        )
    )

    # Side hanging leather tab seen in the design reference.
    parts.append(
        _oriented_box(
            "right_waist_hanging_tab",
            (292, 188, 1035),
            (310, 178, 845),
            42,
            10,
            MAT_LEATHER,
            group="harness",
            role="Asymmetric hanging waist tab",
        )
    )
    parts.extend(
        _ring_buckle(
            "right_waist_tab_end",
            (310, 186, 850),
            (35, 30),
            6,
            8,
            MAT_METAL,
            role="Waist-tab end hardware",
        )
    )


def _back_pattern_and_seams(parts: list[Part]) -> None:
    """Raised stitch/lattice geometry gives the back authored information."""
    # Centre spine leather.
    parts.append(
        _oriented_box(
            "back_spine_binding",
            (0, -218, 1515),
            (0, -218, 940),
            30,
            7,
            MAT_LEATHER,
            group="back_detail",
            role="Structured centre-back leather spine",
        )
    )

    # Geometric yoke lattice: shallow black-on-black raised lines.
    z_low, z_high = 1120.0, 1490.0
    x_extent = 188.0
    for i in range(7):
        t = i / 6.0
        x0 = -x_extent + 2.0 * x_extent * t
        parts.append(
            _oriented_box(
                f"back_lattice_pos_{i}",
                (x0 - 100, -223, z_low),
                (x0 + 100, -223, z_high),
                5.0,
                4.0,
                MAT_STITCH,
                group="back_detail",
                role="Raised technical-stitch lattice",
            )
        )
        parts.append(
            _oriented_box(
                f"back_lattice_neg_{i}",
                (x0 + 100, -224, z_low),
                (x0 - 100, -224, z_high),
                5.0,
                4.0,
                MAT_STITCH,
                group="back_detail",
                role="Raised technical-stitch lattice",
            )
        )

    # Major front/back seam bindings.
    seam_paths = {
        "left_front_princess_seam": [(-225, 202, 1450), (-210, 204, 1280), (-188, 199, 1060), (-165, 170, 760)],
        "right_front_princess_seam": [(225, 202, 1450), (210, 204, 1280), (188, 199, 1060), (165, 170, 760)],
        "left_back_princess_seam": [(-205, -210, 1450), (-190, -214, 1270), (-175, -205, 1050), (-160, -170, 760)],
        "right_back_princess_seam": [(205, -210, 1450), (190, -214, 1270), (175, -205, 1050), (160, -170, 760)],
    }
    for name, path in seam_paths.items():
        parts.append(
            _tube(
                name,
                path,
                (3.0, 3.0),
                MAT_STITCH,
                group="seams",
                role="Raised seam/piping geometry",
                radial=10,
            )
        )


def _trousers(parts: list[Part]) -> None:
    for side, x in (("left", -116), ("right", 116)):
        parts.append(
            _z_loft(
                f"{side}_trouser_leg",
                [
                    (175, 80, 70, x, -10),
                    (340, 83, 72, x, -8),
                    (530, 88, 76, x, -5),
                    (720, 98, 84, x, -2),
                    (900, 116, 98, x, 0),
                    (970, 124, 104, x, 0),
                ],
                MAT_TECH,
                group="trousers",
                role="Slim tailored technical trouser leg",
                radial=48,
            )
        )
        # Angular outer-leg leather panel.
        sx = -1.0 if side == "left" else 1.0
        parts.append(
            _oriented_box(
                f"{side}_trouser_outer_binding",
                (x + sx * 90, 12, 900),
                (x + sx * 72, 10, 230),
                17,
                7,
                MAT_LEATHER,
                group="trousers",
                role="Leather trouser side binding",
            )
        )
        # Thigh strap on one side only, as in the reference.
        if side == "left":
            parts.append(
                _z_loft(
                    "left_thigh_strap",
                    [(675, 110, 91, x, -1), (707, 111, 92, x, -1)],
                    MAT_LEATHER,
                    group="trousers",
                    role="Asymmetric leather thigh strap",
                    radial=38,
                )
            )
            parts.extend(
                _ring_buckle(
                    "left_thigh_buckle",
                    (x - 98, 32, 691),
                    (30, 25),
                    5,
                    7,
                    MAT_METAL,
                    role="Thigh strap buckle",
                )
            )


def _boots(parts: list[Part]) -> None:
    for side, x in (("left", -116), ("right", 116)):
        # Ankle/shaft.
        parts.append(
            _z_loft(
                f"{side}_boot_shaft",
                [
                    (52, 104, 87, x, 0),
                    (145, 101, 84, x, -2),
                    (245, 94, 78, x, -4),
                    (320, 88, 74, x, -6),
                ],
                MAT_LEATHER,
                group="footwear",
                role="Constructed leather boot shaft",
                radial=44,
            )
        )
        # Foot upper, now directionally shaped along Y rather than a generic sphere.
        parts.append(
            _y_loft(
                f"{side}_boot_upper",
                [
                    (-92, 86, 52, x, 76),
                    (-20, 104, 65, x, 78),
                    (95, 110, 71, x, 78),
                    (205, 106, 67, x, 74),
                    (292, 90, 54, x, 66),
                    (340, 62, 37, x, 58),
                ],
                MAT_LEATHER,
                group="footwear",
                role="Directional sculpted leather boot upper",
                radial=44,
            )
        )
        # Sole has a flattened y-loft for tapered toe.
        parts.append(
            _y_loft(
                f"{side}_boot_sole",
                [
                    (-102, 103, 18, x, 28),
                    (-10, 116, 19, x, 28),
                    (115, 120, 19, x, 28),
                    (245, 114, 18, x, 28),
                    (350, 79, 15, x, 28),
                ],
                MAT_RUBBER,
                group="footwear",
                role="Tapered rubber outsole",
                radial=40,
            )
        )
        # Heel block and layered welt.
        parts.append(
            _box(
                f"{side}_boot_heel",
                (x, -58, 44),
                (178, 112, 72),
                MAT_RUBBER,
                group="footwear",
                role="Architectural boot heel",
            )
        )
        # Instep and ankle straps.
        for idx, (y, z) in enumerate(((110, 132), (40, 198), (-12, 270))):
            parts.append(
                _oriented_box(
                    f"{side}_boot_strap_{idx+1}",
                    (x - 92, y, z),
                    (x + 92, y + 2, z),
                    20,
                    8,
                    MAT_LEATHER,
                    group="footwear",
                    role="Leather boot strap",
                )
            )
            parts.extend(
                _ring_buckle(
                    f"{side}_boot_buckle_{idx+1}",
                    (x + 80, y + 14, z),
                    (28, 24),
                    5,
                    7,
                    MAT_METAL,
                    role="Boot strap buckle",
                )
            )
        # Angular polished toe cap.
        parts.append(
            _polygon_panel(
                f"{side}_toe_cap",
                [(x - 72, 42), (x + 72, 42), (x + 82, 94), (x, 118), (x - 82, 94)],
                316,
                22,
                MAT_LEATHER,
                group="footwear",
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
        "design": "CYBR NOCTURNE v2 / procedural technical couture",
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
