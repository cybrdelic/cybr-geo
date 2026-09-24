"""CYBR NOCTURNE: procedural technical-couture outfit for CYBR GEO.

The model is authored from geometry only.  No scan, photogrammetry asset, texture
bake, image-generation system, or prerecorded render is required.  Coordinates
are millimetres in CYBR GEO's native Z-up convention.

Run:
    lab build examples/cybr_nocturne_outfit.py
    python tools/render_cybr_nocturne_cybrlight.py --preset preview
"""
from __future__ import annotations

import math
from typing import Iterable, Sequence

import numpy as np
import trimesh

from cybrgeo import Assembly, Material, Part


TAU = math.tau


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
    # Recompute after every nonuniform procedural deformation.
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
    v = np.asarray(mesh.vertices, dtype=np.float64)
    v *= np.asarray(radii, dtype=np.float64)
    v += np.asarray(center, dtype=np.float64)
    return _part(name, v, mesh.faces, material, group=group, role=role)


def _closed_loft(
    name: str,
    sections: Sequence[tuple[float, float, float, float, float]],
    material: int,
    *,
    group: str,
    role: str,
    radial: int = 64,
    phase: float = 0.0,
) -> Part:
    """Closed elliptical loft.

    Each section is (z, radius_x, radius_y, x_offset, y_offset).
    """
    if len(sections) < 2:
        raise ValueError("loft needs at least two sections")
    vertices: list[list[float]] = []
    faces: list[list[int]] = []
    for z, rx, ry, ox, oy in sections:
        for i in range(radial):
            a = phase + TAU * i / radial
            vertices.append([ox + rx * math.cos(a), oy + ry * math.sin(a), z])

    rings = len(sections)
    for j in range(rings - 1):
        a0 = j * radial
        b0 = (j + 1) * radial
        for i in range(radial):
            ni = (i + 1) % radial
            a, b, c, d = a0 + i, a0 + ni, b0 + ni, b0 + i
            faces.extend(([a, b, c], [a, c, d]))

    bottom_center = len(vertices)
    z, _, _, ox, oy = sections[0]
    vertices.append([ox, oy, z])
    top_center = len(vertices)
    z, _, _, ox, oy = sections[-1]
    vertices.append([ox, oy, z])
    for i in range(radial):
        ni = (i + 1) % radial
        faces.append([bottom_center, ni, i])
        a = (rings - 1) * radial + i
        b = (rings - 1) * radial + ni
        faces.append([top_center, a, b])

    return _part(
        name,
        np.asarray(vertices),
        np.asarray(faces),
        material,
        group=group,
        role=role,
        metadata={"generator": "closed_elliptical_loft", "sections": len(sections), "radial": radial},
    )


def _open_shell(
    name: str,
    sections: Sequence[tuple[float, float, float, float, float]],
    material: int,
    *,
    group: str,
    role: str,
    thickness: float = 8.0,
    front_gap: float = math.radians(24.0),
    radial: int = 72,
) -> Part:
    """Finite-thickness open-front elliptical garment shell.

    The opening is centered on +Y, which is the front of the authored figure.
    The inner surface, hem, neckline, and front edges are explicitly closed.
    """
    if thickness <= 0:
        raise ValueError("thickness must be positive")
    count = radial + 1
    start = math.pi / 2 + front_gap
    stop = math.pi / 2 - front_gap + TAU
    angles = np.linspace(start, stop, count)

    vertices: list[list[float]] = []
    faces: list[list[int]] = []
    rings = len(sections)

    # Outer rings then inner rings.
    for inner in (False, True):
        for z, rx, ry, ox, oy in sections:
            irx = max(1.0, rx - thickness) if inner else rx
            iry = max(1.0, ry - thickness) if inner else ry
            for a in angles:
                vertices.append([ox + irx * math.cos(a), oy + iry * math.sin(a), z])

    outer_base = 0
    inner_base = rings * count

    for j in range(rings - 1):
        for i in range(count - 1):
            a = outer_base + j * count + i
            b = a + 1
            d = outer_base + (j + 1) * count + i
            c = d + 1
            faces.extend(([a, b, c], [a, c, d]))

            ia = inner_base + j * count + i
            ib = ia + 1
            id_ = inner_base + (j + 1) * count + i
            ic = id_ + 1
            faces.extend(([ia, ic, ib], [ia, id_, ic]))

    # Bottom and top thickness bands.
    for j in (0, rings - 1):
        for i in range(count - 1):
            o0 = outer_base + j * count + i
            o1 = o0 + 1
            i0 = inner_base + j * count + i
            i1 = i0 + 1
            if j == 0:
                faces.extend(([o0, i1, o1], [o0, i0, i1]))
            else:
                faces.extend(([o0, o1, i1], [o0, i1, i0]))

    # Two vertical front-opening edges.
    for edge in (0, count - 1):
        for j in range(rings - 1):
            o0 = outer_base + j * count + edge
            o1 = outer_base + (j + 1) * count + edge
            i0 = inner_base + j * count + edge
            i1 = inner_base + (j + 1) * count + edge
            if edge == 0:
                faces.extend(([o0, o1, i1], [o0, i1, i0]))
            else:
                faces.extend(([o0, i1, o1], [o0, i0, i1]))

    return _part(
        name,
        np.asarray(vertices),
        np.asarray(faces),
        material,
        group=group,
        role=role,
        metadata={
            "generator": "finite_thickness_open_shell",
            "sections": len(sections),
            "radial": radial,
            "thickness_mm": thickness,
            "front_gap_degrees": math.degrees(front_gap) * 2.0,
        },
    )


def _tube(
    name: str,
    points: Sequence[Sequence[float]],
    radii: Sequence[tuple[float, float]] | tuple[float, float],
    material: int,
    *,
    group: str,
    role: str,
    radial: int = 32,
    cap: bool = True,
) -> Part:
    """Elliptical tube following a polyline with locally generated frames."""
    p = np.asarray(points, dtype=np.float64)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) < 2:
        raise ValueError("tube needs at least two 3D points")
    if isinstance(radii, tuple) and len(radii) == 2 and isinstance(radii[0], (int, float)):
        rr = [tuple(map(float, radii))] * len(p)
    else:
        rr = [tuple(map(float, r)) for r in radii]  # type: ignore[arg-type]
        if len(rr) != len(p):
            raise ValueError("radii must match path point count")

    frames: list[tuple[np.ndarray, np.ndarray]] = []
    for i in range(len(p)):
        if i == 0:
            tangent = p[1] - p[0]
        elif i == len(p) - 1:
            tangent = p[-1] - p[-2]
        else:
            tangent = p[i + 1] - p[i - 1]
        tangent /= np.linalg.norm(tangent)
        ref = np.array([0.0, 0.0, 1.0])
        if abs(float(np.dot(tangent, ref))) > 0.92:
            ref = np.array([0.0, 1.0, 0.0])
        b1 = np.cross(tangent, ref)
        b1 /= np.linalg.norm(b1)
        b2 = np.cross(tangent, b1)
        b2 /= np.linalg.norm(b2)
        frames.append((b1, b2))

    vertices: list[list[float]] = []
    faces: list[list[int]] = []
    for point, (rx, ry), (b1, b2) in zip(p, rr, frames):
        for i in range(radial):
            a = TAU * i / radial
            q = point + math.cos(a) * rx * b1 + math.sin(a) * ry * b2
            vertices.append(q.tolist())

    for j in range(len(p) - 1):
        a0 = j * radial
        b0 = (j + 1) * radial
        for i in range(radial):
            ni = (i + 1) % radial
            a, b, c, d = a0 + i, a0 + ni, b0 + ni, b0 + i
            faces.extend(([a, b, c], [a, c, d]))

    if cap:
        c0 = len(vertices)
        vertices.append(p[0].tolist())
        c1 = len(vertices)
        vertices.append(p[-1].tolist())
        last = (len(p) - 1) * radial
        for i in range(radial):
            ni = (i + 1) % radial
            faces.append([c0, ni, i])
            faces.append([c1, last + i, last + ni])

    return _part(name, np.asarray(vertices), np.asarray(faces), material, group=group, role=role)


def _panel(
    name: str,
    polygon_xz: Sequence[tuple[float, float]],
    y_center: float,
    depth: float,
    material: int,
    *,
    group: str,
    role: str,
) -> Part:
    """Extruded X/Z panel used for lapels, flaps, plates and ribbing."""
    poly = np.asarray(polygon_xz, dtype=np.float64)
    if poly.ndim != 2 or poly.shape[1] != 2 or len(poly) < 3 or depth <= 0:
        raise ValueError("invalid panel")
    vertices: list[list[float]] = []
    for y in (y_center + depth / 2, y_center - depth / 2):
        vertices.extend([[x, y, z] for x, z in poly])
    n = len(poly)
    faces: list[list[int]] = []
    for i in range(1, n - 1):
        faces.append([0, i, i + 1])
        faces.append([n, n + i + 1, n + i])
    for i in range(n):
        j = (i + 1) % n
        faces.extend(([i, j, n + j], [i, n + j, n + i]))
    return _part(name, np.asarray(vertices), np.asarray(faces), material, group=group, role=role)


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
    """Rectangular strap/brace oriented between two arbitrary 3D points."""
    a = np.asarray(start, dtype=np.float64)
    b = np.asarray(end, dtype=np.float64)
    direction = b - a
    length = float(np.linalg.norm(direction))
    if length <= 1e-6:
        raise ValueError("oriented box requires distinct endpoints")
    direction /= length
    mesh = trimesh.creation.box(extents=(width, depth, length))
    transform = trimesh.geometry.align_vectors([0.0, 0.0, 1.0], direction)
    if transform is None:
        transform = np.eye(4)
    transform[:3, 3] = (a + b) * 0.5
    mesh.apply_transform(transform)
    return _part(
        name,
        np.asarray(mesh.vertices),
        np.asarray(mesh.faces),
        material,
        group=group,
        role=role,
    )


def _box(
    name: str,
    center: Sequence[float],
    extents: Sequence[float],
    material: int,
    *,
    group: str,
    role: str,
) -> Part:
    mesh = trimesh.creation.box(extents=extents)
    mesh.apply_translation(center)
    return _part(name, mesh.vertices, mesh.faces, material, group=group, role=role)


def _mannequin(parts: list[Part]) -> None:
    role = "Faceless procedural dress-form geometry; display reference, not a scan"
    parts.append(
        _closed_loft(
            "mannequin_torso",
            [
                (900, 250, 160, 0, 0),
                (1080, 265, 170, 0, 0),
                (1260, 310, 185, 0, 0),
                (1460, 335, 195, 0, 0),
                (1540, 300, 175, 0, 0),
            ],
            0,
            group="mannequin",
            role=role,
            radial=56,
        )
    )
    parts.append(
        _closed_loft(
            "mannequin_pelvis",
            [(760, 245, 170, 0, 0), (900, 285, 185, 0, 0), (980, 265, 170, 0, 0)],
            0,
            group="mannequin",
            role=role,
            radial=52,
        )
    )
    parts.append(_ellipsoid("mannequin_head", (0, -8, 1710), (105, 92, 142), 0, group="mannequin", role=role))
    parts.append(_tube("mannequin_neck", [(0, 0, 1515), (0, 0, 1600)], (72, 66), 0, group="mannequin", role=role, radial=32))

    arm_paths = {
        "left": [(-300, 0, 1450), (-505, 15, 1245), (-605, 35, 1010)],
        "right": [(300, 0, 1450), (505, 15, 1245), (605, 35, 1010)],
    }
    for side, path in arm_paths.items():
        parts.append(
            _tube(
                f"mannequin_{side}_arm",
                path,
                [(76, 72), (68, 64), (53, 49)],
                0,
                group="mannequin",
                role=role,
                radial=28,
            )
        )
        wrist = np.asarray(path[-1], dtype=float)
        parts.append(
            _ellipsoid(
                f"mannequin_{side}_hand",
                wrist + np.array([0, 5, -80]),
                (50, 34, 92),
                0,
                group="mannequin",
                role=role,
                subdivisions=2,
            )
        )

    for side, x in (("left", -138), ("right", 138)):
        parts.append(
            _closed_loft(
                f"mannequin_{side}_leg",
                [
                    (160, 87, 79, x, -5),
                    (420, 96, 86, x, -2),
                    (690, 112, 100, x, 0),
                    (940, 135, 118, x, 0),
                ],
                0,
                group="mannequin",
                role=role,
                radial=44,
            )
        )


def _garments(parts: list[Part]) -> None:
    shell_role = "Procedural finite-thickness outer garment shell"
    weave_role = "Procedural fitted technical-weave garment layer"

    # Fitted under-vest.
    parts.append(
        _closed_loft(
            "vest_body",
            [
                (955, 300, 178, 0, 0),
                (1090, 274, 166, 0, 2),
                (1280, 305, 180, 0, 2),
                (1468, 330, 195, 0, 0),
                (1508, 300, 174, 0, 0),
            ],
            2,
            group="underlayer",
            role=weave_role,
            radial=72,
        )
    )

    # Long asymmetrical open-front coat.
    parts.append(
        _open_shell(
            "coat_shell",
            [
                (380, 520, 300, 18, -16),
                (650, 455, 258, 8, -8),
                (900, 350, 212, 0, -2),
                (1040, 302, 190, 0, 0),
                (1210, 325, 202, 0, 0),
                (1415, 350, 215, 0, 0),
                (1538, 365, 222, 0, -2),
            ],
            1,
            group="coat",
            role=shell_role,
            thickness=10,
            front_gap=math.radians(22),
            radial=84,
        )
    )

    # Asymmetric skirt overlays create a couture silhouette rather than a uniform tube.
    parts.append(
        _panel(
            "coat_left_long_front",
            [(-350, 1040), (-210, 1040), (-165, 470), (-410, 300), (-510, 430)],
            205,
            12,
            1,
            group="coat",
            role="Asymmetric left front coat panel",
        )
    )
    parts.append(
        _panel(
            "coat_right_cutaway_front",
            [(205, 1040), (350, 1040), (460, 655), (360, 590), (205, 760)],
            205,
            12,
            1,
            group="coat",
            role="Short cutaway right front coat panel",
        )
    )

    # Lapels / inner flash.
    parts.append(
        _panel(
            "lapel_left",
            [(-300, 1495), (-98, 1370), (-175, 1080), (-305, 1260)],
            220,
            16,
            3,
            group="coat_trim",
            role="Satin-backed sculpted left lapel",
        )
    )
    parts.append(
        _panel(
            "lapel_right",
            [(300, 1495), (98, 1370), (170, 1125), (292, 1285)],
            220,
            16,
            3,
            group="coat_trim",
            role="Satin-backed sculpted right lapel",
        )
    )

    # Standing collar around the back of the neck.
    parts.append(
        _open_shell(
            "standing_collar",
            [(1510, 190, 140, 0, 0), (1605, 178, 130, 0, -4)],
            1,
            group="coat",
            role="Raised finite-thickness collar",
            thickness=9,
            front_gap=math.radians(38),
            radial=52,
        )
    )

    # Sleeves follow the mannequin A-pose and flare subtly at the shoulder.
    for side, sx in (("left", -1), ("right", 1)):
        points = [
            (sx * 305, 0, 1450),
            (sx * 450, 8, 1330),
            (sx * 520, 18, 1210),
            (sx * 590, 30, 1040),
        ]
        parts.append(
            _tube(
                f"coat_{side}_sleeve",
                points,
                [(102, 92), (91, 84), (78, 72), (64, 58)],
                1,
                group="coat",
                role="Tailored articulated coat sleeve",
                radial=40,
            )
        )
        parts.append(
            _tube(
                f"{side}_wrist_cuff",
                [(sx * 585, 30, 1070), (sx * 615, 36, 995)],
                [(70, 63), (66, 58)],
                4,
                group="hardware",
                role="Blackened-metal wrist cuff",
                radial=36,
            )
        )

    # Segmented shoulder scales.
    for side, sx in (("left", -1), ("right", 1)):
        for index, (x0, x1, z0, z1) in enumerate(
            [
                (310, 455, 1450, 1530),
                (410, 535, 1390, 1472),
                (492, 600, 1325, 1410),
            ]
        ):
            parts.append(
                _panel(
                    f"{side}_shoulder_scale_{index+1}",
                    [(sx * x0, z0), (sx * x1, z0 - 18), (sx * x1, z1 - 36), (sx * x0, z1)],
                    72,
                    34,
                    4,
                    group="hardware",
                    role="Layered articulated shoulder scale",
                )
            )

    # Harness/belt language.
    parts.append(
        _oriented_box(
            "diagonal_chest_harness",
            (-242, 203, 1450),
            (182, 207, 1045),
            42,
            14,
            4,
            group="hardware",
            role="Diagonal structural fashion harness",
        )
    )
    parts.append(
        _open_shell(
            "waist_belt",
            [(1008, 318, 194, 0, 0), (1052, 318, 194, 0, 0)],
            4,
            group="hardware",
            role="Open-front blackened-metal waist belt",
            thickness=18,
            front_gap=math.radians(10),
            radial=68,
        )
    )
    parts.append(
        _box(
            "belt_buckle",
            (0, 204, 1030),
            (76, 24, 58),
            5,
            group="hardware",
            role="Spectral indigo belt clasp",
        )
    )

    # Restrained indigo piping—small, real geometry, not emissive decals.
    trim_paths = {
        "lapel_left_trim": [(-294, 230, 1494), (-104, 230, 1370), (-174, 230, 1085)],
        "lapel_right_trim": [(294, 230, 1494), (104, 230, 1370), (170, 230, 1130)],
        "left_sleeve_trim": [(-315, 94, 1432), (-454, 96, 1320), (-525, 98, 1200)],
        "right_sleeve_trim": [(315, 94, 1432), (454, 96, 1320), (525, 98, 1200)],
    }
    for name, path in trim_paths.items():
        parts.append(_tube(name, path, (5.0, 5.0), 5, group="accent", role="Non-emissive spectral accent piping", radial=16))

    # Tapered technical trousers.
    for side, x in (("left", -140), ("right", 140)):
        parts.append(
            _closed_loft(
                f"{side}_trouser_leg",
                [
                    (185, 86, 74, x, -3),
                    (400, 92, 80, x, -1),
                    (650, 108, 96, x, 0),
                    (820, 120, 108, x, 0),
                    (960, 137, 122, x, 0),
                ],
                2,
                group="trousers",
                role="Tapered technical-weave trouser leg",
                radial=52,
            )
        )

        # Knee articulation as real raised ribs.
        for rib in range(5):
            z = 545 + rib * 24
            parts.append(
                _panel(
                    f"{side}_knee_rib_{rib+1}",
                    [(x - 83, z - 7), (x + 83, z - 7), (x + 78, z + 7), (x - 78, z + 7)],
                    94,
                    12,
                    4 if rib in (0, 4) else 6,
                    group="trousers",
                    role="Raised knee articulation rib",
                )
            )

        # Outer seam hardware line.
        sx = -1 if side == "left" else 1
        parts.append(
            _tube(
                f"{side}_trouser_side_trim",
                [(x + sx * 104, 12, 895), (x + sx * 96, 10, 610), (x + sx * 78, 8, 270)],
                (4.0, 4.0),
                5,
                group="accent",
                role="Non-emissive side-seam accent",
                radial=14,
            )
        )

    # Boots: fitted ankle shell + elongated toe + hard sole.
    for side, x in (("left", -140), ("right", 140)):
        parts.append(
            _closed_loft(
                f"{side}_boot_cuff",
                [(55, 108, 98, x, 2), (150, 101, 92, x, 3), (260, 92, 82, x, 0)],
                6,
                group="footwear",
                role="Carbon-rubber boot cuff",
                radial=48,
            )
        )
        parts.append(
            _ellipsoid(
                f"{side}_boot_upper",
                (x, 118, 88),
                (112, 228, 94),
                6,
                group="footwear",
                role="Sculpted elongated boot upper",
                subdivisions=3,
            )
        )
        parts.append(
            _box(
                f"{side}_boot_sole",
                (x, 92, 27),
                (220, 430, 44),
                4,
                group="footwear",
                role="Hard blackened sole plate",
            )
        )
        parts.append(
            _panel(
                f"{side}_toe_cap",
                [(x - 100, 70), (x + 100, 70), (x + 86, 132), (x - 86, 132)],
                315,
                24,
                4,
                group="footwear",
                role="Blackened-metal toe cap",
            )
        )

    # Small sternum hardware detail and collar fasteners.
    parts.append(_box("sternum_clasp", (0, 206, 1320), (42, 22, 72), 5, group="hardware", role="Central indigo hardware clasp"))
    for x in (-78, 78):
        parts.append(_ellipsoid(f"collar_fastener_{'l' if x < 0 else 'r'}", (x, 136, 1536), (13, 8, 13), 4, group="hardware", role="Collar fastener", subdivisions=2))


def build() -> Assembly:
    materials = [
        Material(
            name="Mannequin graphite",
            color=(0.075, 0.080, 0.090),
            metal=0.0,
            rough=0.82,
            coat=0.0,
            microfinish="matte dress form",
            material_source="procedural",
        ),
        Material(
            name="Obsidian shell textile",
            color=(0.014, 0.018, 0.024),
            metal=0.0,
            rough=0.56,
            coat=0.06,
            coat_rough=0.34,
            microfinish="dense technical twill",
            material_source="procedural",
        ),
        Material(
            name="Graphite technical weave",
            color=(0.040, 0.047, 0.058),
            metal=0.0,
            rough=0.72,
            coat=0.02,
            microfinish="fine woven underlayer",
            material_source="procedural",
        ),
        Material(
            name="Ink satin lining",
            color=(0.070, 0.018, 0.096),
            metal=0.0,
            rough=0.28,
            coat=0.18,
            coat_rough=0.20,
            microfinish="low-gloss satin",
            material_source="procedural",
        ),
        Material(
            name="Blackened anisotropic metal",
            color=(0.070, 0.066, 0.062),
            metal=0.96,
            rough=0.18,
            anisotropy=0.65,
            anisotropy_rotation=0.0,
            microfinish="brushed black alloy",
            material_source="procedural",
        ),
        Material(
            name="Spectral indigo trim",
            color=(0.040, 0.085, 0.310),
            metal=0.58,
            rough=0.22,
            coat=0.24,
            coat_rough=0.16,
            anisotropy=0.28,
            microfinish="blue-violet coated alloy",
            material_source="procedural",
        ),
        Material(
            name="Carbon rubber",
            color=(0.012, 0.015, 0.019),
            metal=0.0,
            rough=0.90,
            microfinish="fine molded rubber",
            material_source="procedural",
        ),
    ]

    parts: list[Part] = []
    _mannequin(parts)
    _garments(parts)

    metadata = {
        "design": "CYBR NOCTURNE / technical couture study",
        "authoring": "procedural geometry only",
        "units": "mm",
        "front_axis": "+Y",
        "up_axis": "+Z",
        "design_height_mm": 1852,
        "render_pair": "CYBR GEO geometry + CYBR LIGHT spectral path tracing",
        "limitations": [
            "Fashion/visualization study, not a sewing pattern.",
            "No cloth dynamics, body-fit scan, garment stress analysis, or manufacturing qualification.",
            "Mannequin is procedural display geometry and is not based on a scanned person.",
        ],
    }
    return Assembly("CYBR NOCTURNE", parts, materials, metadata=metadata)


if __name__ == "__main__":
    outfit = build()
    print(outfit.validate())
