"""Reference-grounded CYBR GEO reconstruction of 6503 Del Playa Drive.

The scene is built entirely from authored/procedural geometry in millimetres,
Z-up. It deliberately contains no scan mesh, photogrammetry, image billboard,
or generated texture. Visible dimensions are estimated from public reference
photographs and site context rather than a survey; uncertainty is recorded in
assembly metadata instead of being hidden.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np

from mechanism_lab.core import Assembly, Material, Part, View

TAU = math.tau


@dataclass(frozen=True)
class HouseConfig:
    # Reference-estimated architectural dimensions, millimetres.
    main_width: float = 18_800.0
    main_depth: float = 7_650.0
    lower_height: float = 2_620.0
    upper_height: float = 2_720.0
    slab_z: float = 2_620.0
    roof_z: float = 5_340.0
    front_wall_y: float = -620.0
    rear_wall_y: float = 7_030.0
    deck_front_y: float = -2_500.0
    deck_side_x: float = 10_050.0
    deck_thickness: float = 145.0
    rail_height: float = 1_040.0
    bluff_edge_y: float = -5_500.0
    seed: int = 6503

    def check(self) -> None:
        positive = (
            self.main_width,
            self.main_depth,
            self.lower_height,
            self.upper_height,
            self.deck_thickness,
            self.rail_height,
        )
        if min(positive) <= 0:
            raise ValueError("Architectural dimensions must be positive")
        if self.deck_front_y >= self.front_wall_y:
            raise ValueError("Ocean deck must project beyond front wall")
        if self.roof_z <= self.slab_z:
            raise ValueError("Roof must sit above upper-floor slab")


MATERIALS = [
    Material("Aged warm stucco", (0.34, 0.285, 0.205), 0.0, 0.86, microfinish="concrete", material_source="reference-authored"),
    Material("Aged stucco lighter patches", (0.405, 0.345, 0.255), 0.0, 0.90, microfinish="concrete", material_source="reference-authored"),
    Material("Dark lower cladding", (0.070, 0.048, 0.034), 0.0, 0.83, microfinish="wood", material_source="reference-authored"),
    Material("Weathered deck redwood", (0.175, 0.100, 0.055), 0.0, 0.82, microfinish="wood", material_source="reference-authored"),
    Material("Sun-bleached deck redwood", (0.235, 0.145, 0.082), 0.0, 0.88, microfinish="wood", material_source="reference-authored"),
    Material("Darkened exterior timber", (0.105, 0.058, 0.032), 0.0, 0.90, microfinish="wood", material_source="reference-authored"),
    Material("Window glass", (0.56, 0.69, 0.74), 0.0, 0.05, ior=1.52, opacity=0.27, material_source="reference-authored"),
    Material("Dark aluminum window frame", (0.055, 0.063, 0.066), 0.78, 0.32, microfinish="anodized", material_source="reference-authored"),
    Material("Aged concrete patio", (0.265, 0.255, 0.225), 0.0, 0.94, microfinish="concrete", material_source="reference-authored"),
    Material("Roof membrane", (0.185, 0.188, 0.185), 0.0, 0.95, microfinish="concrete", material_source="reference-authored"),
    Material("Galvanized fence", (0.42, 0.44, 0.43), 0.76, 0.42, microfinish="brushed", material_source="reference-authored"),
    Material("Dry bluff soil", (0.145, 0.092, 0.050), 0.0, 0.98, microfinish="concrete", material_source="procedural-site"),
    Material("Beach sand", (0.385, 0.315, 0.215), 0.0, 0.97, microfinish="concrete", material_source="procedural-site"),
    Material("Pacific water", (0.035, 0.115, 0.19), 0.0, 0.08, ior=1.333, material_source="procedural-site"),
    Material("Coastal shrub dark", (0.033, 0.082, 0.024), 0.0, 0.92, material_source="procedural-site"),
    Material("Coastal shrub light", (0.068, 0.135, 0.043), 0.0, 0.88, material_source="procedural-site"),
    Material("Dry coastal grass", (0.29, 0.25, 0.115), 0.0, 0.94, material_source="procedural-site"),
    Material("Palm trunk", (0.135, 0.088, 0.045), 0.0, 0.92, microfinish="wood", material_source="procedural-site"),
    Material("Palm / yucca leaf", (0.044, 0.115, 0.033), 0.0, 0.82, material_source="procedural-site"),
    Material("Neighbor off-white stucco", (0.455, 0.435, 0.365), 0.0, 0.89, microfinish="concrete", material_source="reference-context"),
    Material("White painted railing", (0.54, 0.545, 0.505), 0.0, 0.68, material_source="reference-context"),
    Material("Exterior door muted red", (0.29, 0.075, 0.046), 0.0, 0.72, material_source="reference-context"),
    Material("Ping-pong top", (0.08, 0.11, 0.12), 0.05, 0.62, material_source="reference-context"),
]


# Material indices retained as constants so the CYBR LIGHT bridge can preserve intent.
STUCCO, STUCCO_LIGHT, LOWER_DARK, WOOD, WOOD_LIGHT, WOOD_DARK, GLASS, FRAME, CONCRETE, ROOF, METAL, SOIL, SAND, WATER, GREEN_DARK, GREEN_LIGHT, DRY_GRASS, TRUNK, LEAF, NEIGHBOR, WHITE, RED_DOOR, TABLE = range(len(MATERIALS))


def _part(name: str, vertices: np.ndarray, faces: np.ndarray, material: int, group: str, role: str,
          normals: np.ndarray | None = None, provenance: str = "reference-authored") -> Part:
    vertices = np.asarray(vertices, dtype=float)
    faces = np.asarray(faces, dtype=np.int64)
    if normals is None:
        normals = np.zeros_like(vertices)
    return Part(
        name=name,
        vertices=vertices,
        faces=faces,
        normals=np.asarray(normals, dtype=float),
        material=int(material),
        group=group,
        role=role,
        provenance=provenance,
        center=vertices.mean(axis=0),
        finish_origin=tuple(vertices.mean(axis=0)),
    )


def _flat_mesh(vertices: np.ndarray, faces: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Duplicate triangle vertices so hard architectural edges stay hard."""
    vv: list[np.ndarray] = []
    ff: list[list[int]] = []
    nn: list[np.ndarray] = []
    for face in np.asarray(faces, dtype=int):
        tri = np.asarray(vertices, dtype=float)[face]
        normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        length = float(np.linalg.norm(normal))
        if length < 1e-12:
            continue
        normal /= length
        start = len(vv)
        vv.extend(tri)
        nn.extend([normal, normal, normal])
        ff.append([start, start + 1, start + 2])
    return np.asarray(vv), np.asarray(ff, dtype=np.int64), np.asarray(nn)


def _box_geometry(size: Iterable[float], center: Iterable[float], matrix: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    sx, sy, sz = map(float, size)
    cx, cy, cz = map(float, center)
    corners = np.array([
        [-sx, -sy, -sz], [sx, -sy, -sz], [sx, sy, -sz], [-sx, sy, -sz],
        [-sx, -sy, sz], [sx, -sy, sz], [sx, sy, sz], [-sx, sy, sz],
    ]) * 0.5
    if matrix is not None:
        corners = corners @ np.asarray(matrix, dtype=float).T
    corners += np.array([cx, cy, cz])
    # CCW viewed from outside.
    faces = np.array([
        [0, 2, 1], [0, 3, 2],
        [4, 5, 6], [4, 6, 7],
        [0, 1, 5], [0, 5, 4],
        [1, 2, 6], [1, 6, 5],
        [2, 3, 7], [2, 7, 6],
        [3, 0, 4], [3, 4, 7],
    ], dtype=np.int64)
    return _flat_mesh(corners, faces)


def _smooth_normals(vertices: np.ndarray, faces: np.ndarray) -> np.ndarray:
    normals = np.zeros_like(vertices, dtype=float)
    for f in faces:
        tri = vertices[f]
        n = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        if np.linalg.norm(n) > 1e-12:
            normals[f] += n
    lengths = np.linalg.norm(normals, axis=1)
    valid = lengths > 1e-12
    normals[valid] /= lengths[valid, None]
    normals[~valid] = (0, 0, 1)
    return normals


class Builder:
    def __init__(self, cfg: HouseConfig):
        self.cfg = cfg
        self.parts: list[Part] = []
        self.rng = np.random.default_rng(cfg.seed)

    def add(self, p: Part) -> Part:
        self.parts.append(p)
        return p

    def box(self, name: str, size: Iterable[float], center: Iterable[float], material: int,
            group: str, role: str, matrix: np.ndarray | None = None,
            provenance: str = "reference-authored") -> Part:
        v, f, n = _box_geometry(size, center, matrix)
        return self.add(_part(name, v, f, material, group, role, n, provenance))

    def beam_x(self, name: str, x0: float, x1: float, y: float, z: float, width_y: float, height: float,
               material: int, group: str, role: str) -> Part:
        return self.box(name, (abs(x1 - x0), width_y, height), ((x0 + x1) * 0.5, y, z), material, group, role)

    def beam_y(self, name: str, y0: float, y1: float, x: float, z: float, width_x: float, height: float,
               material: int, group: str, role: str) -> Part:
        return self.box(name, (width_x, abs(y1 - y0), height), (x, (y0 + y1) * 0.5, z), material, group, role)

    def cylinder(self, name: str, a: Iterable[float], b: Iterable[float], radius: float, material: int,
                 group: str, role: str, sides: int = 12, provenance: str = "reference-authored") -> Part:
        a = np.asarray(a, dtype=float)
        b = np.asarray(b, dtype=float)
        axis = b - a
        length = float(np.linalg.norm(axis))
        if length <= 0:
            raise ValueError("Cylinder endpoints must differ")
        w = axis / length
        ref = np.array((0.0, 0.0, 1.0)) if abs(w[2]) < 0.86 else np.array((1.0, 0.0, 0.0))
        u = np.cross(w, ref); u /= np.linalg.norm(u)
        v = np.cross(w, u)
        vertices: list[np.ndarray] = []
        normals: list[np.ndarray] = []
        for p in (a, b):
            for i in range(sides):
                q = math.cos(TAU * i / sides) * u + math.sin(TAU * i / sides) * v
                vertices.append(p + q * radius)
                normals.append(q)
        vertices.extend([a, b])
        normals.extend([-w, w])
        faces: list[list[int]] = []
        for i in range(sides):
            j = (i + 1) % sides
            faces.extend([[i, j, sides + j], [i, sides + j, sides + i]])
            faces.append([2 * sides, j, i])
            faces.append([2 * sides + 1, sides + i, sides + j])
        return self.add(_part(name, np.asarray(vertices), np.asarray(faces), material, group, role,
                              np.asarray(normals), provenance))

    def ellipsoid(self, name: str, center: Iterable[float], radii: Iterable[float], material: int,
                  group: str, role: str, rings: int = 6, sectors: int = 10) -> Part:
        center = np.asarray(center, dtype=float)
        radii = np.asarray(radii, dtype=float)
        verts: list[np.ndarray] = []
        for i in range(rings + 1):
            phi = -math.pi / 2 + math.pi * i / rings
            for j in range(sectors):
                theta = TAU * j / sectors
                p = np.array((math.cos(phi) * math.cos(theta), math.cos(phi) * math.sin(theta), math.sin(phi)))
                verts.append(center + p * radii)
        faces: list[list[int]] = []
        for i in range(rings):
            for j in range(sectors):
                k = (j + 1) % sectors
                a = i * sectors + j
                b = i * sectors + k
                c = (i + 1) * sectors + k
                d = (i + 1) * sectors + j
                faces.extend([[a, b, c], [a, c, d]])
        vv = np.asarray(verts)
        ff = np.asarray(faces, dtype=np.int64)
        nn = _smooth_normals(vv, ff)
        return self.add(_part(name, vv, ff, material, group, role, nn, "procedural-site"))

    def leaf_cluster(self, name: str, center: Iterable[float], radii: Iterable[float], material: int,
                     group: str, role: str, count: int = 48, leaf_length: tuple[float, float] = (140.0, 300.0),
                     leaf_width: tuple[float, float] = (55.0, 120.0)) -> Part:
        """Dense explicit two-sided leaf geometry; no alpha cards or image texture."""
        center = np.asarray(center, dtype=float)
        radii = np.asarray(radii, dtype=float)
        vertices: list[np.ndarray] = []
        faces: list[list[int]] = []
        normals: list[np.ndarray] = []
        for i in range(count):
            d = self.rng.normal(size=3)
            d /= max(np.linalg.norm(d), 1e-9)
            p = center + d * radii * (self.rng.random() ** (1.0 / 3.0))
            n = self.rng.normal(size=3)
            n /= max(np.linalg.norm(n), 1e-9)
            ref = np.array((0.0, 0.0, 1.0)) if abs(n[2]) < 0.88 else np.array((1.0, 0.0, 0.0))
            u = np.cross(n, ref); u /= max(np.linalg.norm(u), 1e-9)
            v = np.cross(n, u)
            length = self.rng.uniform(*leaf_length)
            width = self.rng.uniform(*leaf_width)
            mid = p + v * self.rng.uniform(-0.08, 0.08) * length
            base = len(vertices)
            vertices.extend([
                p - v * length * 0.50,
                mid - u * width * 0.50,
                p + v * length * 0.50,
                mid + u * width * 0.50,
                mid + n * self.rng.uniform(8.0, 24.0),
            ])
            faces.extend([
                [base + 0, base + 1, base + 4],
                [base + 1, base + 2, base + 4],
                [base + 2, base + 3, base + 4],
                [base + 3, base + 0, base + 4],
            ])
            normals.extend([n, n, n, n, n])
        return self.add(_part(name, np.asarray(vertices), np.asarray(faces, dtype=np.int64),
                              material, group, role, np.asarray(normals), "procedural-site"))

    def window(self, name: str, x: float, width: float, z0: float, z1: float, y: float,
               panes: int = 2, frame: float = 55.0, group: str = "fenestration") -> None:
        self.box(name + "_glass", (width, 28.0, z1 - z0), (x, y, (z0 + z1) * 0.5), GLASS, group,
                 "Reference-estimated glazing plane")
        self.box(name + "_interior_shadow", (max(80.0, width - 120.0), 18.0, max(120.0, z1 - z0 - 120.0)),
                 (x, y + 30.0, (z0 + z1) * 0.5), LOWER_DARK, group,
                 "Dark recessed plane behind exterior glazing; authored geometry, not an image")
        self.beam_x(name + "_frame_top", x - width / 2, x + width / 2, y - 18.0, z1, 70.0, frame, FRAME, group,
                    "Window frame")
        self.beam_x(name + "_frame_bottom", x - width / 2, x + width / 2, y - 18.0, z0, 70.0, frame, FRAME, group,
                    "Window frame")
        self.box(name + "_frame_l", (frame, 70.0, z1 - z0), (x - width / 2, y - 18.0, (z0 + z1) / 2), FRAME, group,
                 "Window frame")
        self.box(name + "_frame_r", (frame, 70.0, z1 - z0), (x + width / 2, y - 18.0, (z0 + z1) / 2), FRAME, group,
                 "Window frame")
        if panes > 1:
            for i in range(1, panes):
                xx = x - width / 2 + width * i / panes
                self.box(name + f"_mullion_{i}", (frame * 0.72, 76.0, z1 - z0), (xx, y - 20.0, (z0 + z1) / 2), FRAME, group,
                         "Sliding-door/window mullion")

    def side_window(self, name: str, y: float, width: float, z0: float, z1: float, x: float, panes: int = 2) -> None:
        self.box(name + "_glass", (28.0, width, z1 - z0), (x, y, (z0 + z1) / 2), GLASS, "fenestration",
                 "Reference-estimated side glazing")
        self.box(name + "_interior_shadow", (18.0, max(80.0, width - 120.0), max(120.0, z1 - z0 - 120.0)),
                 (x - 30.0, y, (z0 + z1) / 2), LOWER_DARK, "fenestration",
                 "Dark recessed plane behind side glazing; authored geometry, not an image")
        self.beam_y(name + "_top", y - width / 2, y + width / 2, x + 18, z1, 70, 55, FRAME, "fenestration", "Window frame")
        self.beam_y(name + "_bottom", y - width / 2, y + width / 2, x + 18, z0, 70, 55, FRAME, "fenestration", "Window frame")
        self.box(name + "_a", (70, 55, z1 - z0), (x + 18, y - width / 2, (z0 + z1) / 2), FRAME, "fenestration", "Window frame")
        self.box(name + "_b", (70, 55, z1 - z0), (x + 18, y + width / 2, (z0 + z1) / 2), FRAME, "fenestration", "Window frame")
        if panes > 1:
            for i in range(1, panes):
                yy = y - width / 2 + width * i / panes
                self.box(name + f"_mullion_{i}", (76, 42, z1 - z0), (x + 20, yy, (z0 + z1) / 2), FRAME, "fenestration", "Window mullion")

    def front_railing(self, x0: float, x1: float, y: float, deck_z: float, prefix: str, white: bool = False) -> None:
        timber = WHITE if white else WOOD
        baluster = WHITE if white else WOOD_DARK
        top_z = deck_z + self.cfg.rail_height
        self.beam_x(prefix + "_toprail", x0, x1, y, top_z, 82.0, 82.0, timber, "balcony", "Balcony top rail")
        self.beam_x(prefix + "_bottomrail", x0, x1, y, deck_z + 175.0, 58.0, 64.0, timber, "balcony", "Balcony bottom rail")
        spacing = 145.0 if white else 155.0
        count = max(2, int(abs(x1 - x0) // spacing))
        for i, x in enumerate(np.linspace(x0 + 75, x1 - 75, count)):
            self.box(prefix + f"_baluster_{i:03}", (34.0, 42.0, self.cfg.rail_height - 150.0),
                     (x, y, deck_z + 555.0), baluster, "balcony", "Vertical balcony baluster")
        post_count = max(2, int(abs(x1 - x0) // 2250) + 1)
        for i, x in enumerate(np.linspace(x0, x1, post_count)):
            self.box(prefix + f"_post_{i:02}", (92.0, 92.0, self.cfg.rail_height + 180.0),
                     (x, y, deck_z + (self.cfg.rail_height + 180.0) / 2 - 30), timber, "balcony", "Primary balcony post")

    def side_railing(self, y0: float, y1: float, x: float, deck_z: float, prefix: str) -> None:
        top_z = deck_z + self.cfg.rail_height
        self.beam_y(prefix + "_toprail", y0, y1, x, top_z, 82.0, 82.0, WOOD, "balcony", "Side balcony top rail")
        self.beam_y(prefix + "_bottomrail", y0, y1, x, deck_z + 175.0, 58.0, 64.0, WOOD, "balcony", "Side balcony bottom rail")
        count = max(2, int(abs(y1 - y0) // 155.0))
        for i, y in enumerate(np.linspace(y0 + 75, y1 - 75, count)):
            self.box(prefix + f"_baluster_{i:03}", (42.0, 34.0, self.cfg.rail_height - 150.0),
                     (x, y, deck_z + 555.0), WOOD_DARK, "balcony", "Vertical balcony baluster")
        for i, y in enumerate(np.linspace(y0, y1, max(2, int(abs(y1 - y0) // 2100) + 1))):
            self.box(prefix + f"_post_{i:02}", (92.0, 92.0, self.cfg.rail_height + 180.0),
                     (x, y, deck_z + (self.cfg.rail_height + 180.0) / 2 - 30), WOOD, "balcony", "Primary balcony post")


def _site(builder: Builder) -> None:
    cfg = builder.cfg
    xs = np.linspace(-23_000, 25_000, 76)
    ys = np.linspace(-24_000, 15_000, 64)
    verts: list[list[float]] = []
    for y in ys:
        for x in xs:
            edge = cfg.bluff_edge_y + 420 * math.sin(x / 4200) + 160 * math.sin(x / 1350)
            top_noise = (52 * math.sin(x / 1550) * math.sin(y / 1800) + 31 * math.sin((x + y) / 720) + 18 * math.sin(x / 330 + y / 510) + 11 * math.sin(x / 145 - y / 235))
            if y >= edge:
                z = -70 + top_noise * 0.18
            else:
                t = min(1.0, max(0.0, (edge - y) / 10_700.0))
                rill = (320 * math.sin(x / 910 + t * 2.0) + 115 * math.sin(x / 360 - t * 5.0) + 55 * math.sin(x / 155 + t * 8.0)) * (t ** 1.35)
                z = -70 - 8_150 * (t ** 0.62) + rill + top_noise * (0.15 + 0.85 * t)
            verts.append([x, y, z])
    faces: list[list[int]] = []
    w = len(xs)
    for j in range(len(ys) - 1):
        for i in range(len(xs) - 1):
            a = j * w + i; b = a + 1; d = (j + 1) * w + i; c = d + 1
            faces.extend([[a, b, c], [a, c, d]])
    vv = np.asarray(verts, dtype=float); ff = np.asarray(faces, dtype=np.int64)
    builder.add(_part("bluff_terrain", vv, ff, SOIL, "site", "Procedural eroded bluff tied to reference silhouette",
                      _smooth_normals(vv, ff), "procedural-site"))

    builder.box("beach_sand", (56_000, 22_000, 150), (0, -25_000, -8_250), SAND, "site", "Beach plane below the bluff", provenance="procedural-site")
    builder.box("ocean_volume", (90_000, 55_000, 500), (0, -54_000, -8_080), WATER, "site", "Pacific ocean context volume", provenance="procedural-site")
    builder.box("main_patioslab", (23_600, 9_200, 125), (1_800, 1_900, -5), CONCRETE, "site", "Reference-estimated concrete patio / yard")
    builder.box("east_patioslab", (5_700, 11_000, 120), (12_250, 1_150, -10), CONCRETE, "site", "East-side patio visible in listing reference")


def _architecture(builder: Builder) -> None:
    cfg = builder.cfg
    wall_y_center = (cfg.front_wall_y + cfg.rear_wall_y) / 2
    wall_depth = cfg.rear_wall_y - cfg.front_wall_y
    builder.box("lower_main_shell", (cfg.main_width, wall_depth, cfg.lower_height), (0, wall_y_center, cfg.lower_height / 2), STUCCO, "architecture", "Lower reference-estimated main volume")
    builder.box("lower_dark_bay", (4_150, 1_300, 2_360), (-5_650, -80, 1_180), LOWER_DARK, "architecture", "Dark lower ocean-facing bay visible in reference")
    builder.box("upper_main_shell", (cfg.main_width + 520, wall_depth + 180, cfg.upper_height), (80, wall_y_center + 40, cfg.slab_z + cfg.upper_height / 2), STUCCO, "architecture", "Upper reference-estimated main volume")

    builder.box("upper_patch_left", (2_600, 34, 1_260), (-7_180, cfg.front_wall_y - 20, 4_080), STUCCO_LIGHT, "weathering", "Subtle stucco tonal repair patch")
    builder.box("upper_patch_mid", (1_900, 34, 820), (3_250, cfg.front_wall_y - 22, 4_360), STUCCO_LIGHT, "weathering", "Subtle stucco tonal repair patch")

    builder.box("roof_slab", (cfg.main_width + 1_030, wall_depth + 840, 165), (50, wall_y_center + 130, cfg.roof_z + 82.5), ROOF, "roof", "Flat weathered roof slab / membrane")
    for i, x in enumerate((-5_600, -1_200, 2_950, 6_700)):
        builder.cylinder(f"roof_vent_{i}", (x, 2_000 + (i % 2) * 2_000, cfg.roof_z + 160), (x, 2_000 + (i % 2) * 2_000, cfg.roof_z + 520), 105, METAL, "roof", "Low roof vent", sides=16)
        builder.cylinder(f"roof_vent_cap_{i}", (x, 2_000 + (i % 2) * 2_000, cfg.roof_z + 515), (x, 2_000 + (i % 2) * 2_000, cfg.roof_z + 560), 160, METAL, "roof", "Roof vent cap", sides=18)

    builder.window("lower_dark_window", -5_800, 1_950, 620, 2_190, cfg.front_wall_y - 48, panes=3)
    builder.window("lower_center_slider", 450, 2_550, 180, 2_360, cfg.front_wall_y - 52, panes=2)
    builder.window("lower_right_slider", 5_460, 2_780, 160, 2_380, cfg.front_wall_y - 52, panes=2)
    builder.window("upper_left_slider", -5_900, 2_100, cfg.slab_z + 390, cfg.roof_z - 310, cfg.front_wall_y - 52, panes=2)
    builder.window("upper_center_narrow", -2_850, 1_080, cfg.slab_z + 520, cfg.roof_z - 420, cfg.front_wall_y - 52, panes=1)
    builder.window("upper_center_slider", 1_080, 2_350, cfg.slab_z + 330, cfg.roof_z - 300, cfg.front_wall_y - 52, panes=2)
    builder.window("upper_right_slider", 6_100, 2_700, cfg.slab_z + 270, cfg.roof_z - 300, cfg.front_wall_y - 52, panes=2)
    builder.side_window("upper_east_glazing", 3_180, 2_220, cfg.slab_z + 310, cfg.roof_z - 320, cfg.main_width / 2 + 285, panes=2)
    builder.side_window("lower_east_glazing", 850, 2_100, 240, 2_320, cfg.main_width / 2 + 22, panes=2)
    builder.box("upper_entry_door", (820, 55, 2_010), (-250, cfg.front_wall_y - 54, cfg.slab_z + 1_230), LOWER_DARK, "fenestration", "Upper exterior door")


def _decks_and_rails(builder: Builder) -> None:
    cfg = builder.cfg
    x0, x1 = -9_750.0, 9_980.0
    deck_y0, deck_y1 = cfg.deck_front_y, cfg.front_wall_y + 240.0
    deck_z = cfg.slab_z + 10.0
    board_w = 142.0
    gap = 8.0
    ys = np.arange(deck_y0 + board_w / 2, deck_y1 - board_w / 2 + 1, board_w + gap)
    for i, y in enumerate(ys):
        jitter = ((i * 17) % 7 - 3) * 0.8
        mat = WOOD_LIGHT if i % 4 == 0 else WOOD
        builder.box(f"front_deck_board_{i:02}", (x1 - x0, board_w, 38.0), ((x0 + x1) / 2, y, deck_z + jitter), mat, "balcony", "Individual weathered deck board")
    side_x0 = cfg.main_width / 2 - 80
    side_x1 = cfg.deck_side_x
    for i, x in enumerate(np.arange(side_x0 + board_w / 2, side_x1 - board_w / 2 + 1, board_w + gap)):
        builder.box(f"side_deck_board_{i:02}", (board_w, cfg.rear_wall_y - deck_y0 + 380, 38.0),
                    (x, (deck_y0 + cfg.rear_wall_y + 380) / 2, deck_z + ((i % 3) - 1) * 1.0),
                    WOOD_LIGHT if i % 5 == 0 else WOOD, "balcony", "Wrap-around side deck board")

    builder.beam_x("front_deck_fascia", x0, x1, deck_y0 + 20, deck_z - 95, 155, 225, WOOD_DARK, "balcony", "Weathered deck fascia")
    builder.beam_y("east_deck_fascia", deck_y0, cfg.rear_wall_y + 380, side_x1 - 20, deck_z - 95, 155, 225, WOOD_DARK, "balcony", "East wrap deck fascia")
    for i, x in enumerate(np.linspace(x0 + 450, x1 - 450, 9)):
        builder.box(f"front_deck_support_{i:02}", (95, 95, deck_z - 90), (x, deck_y0 + 330, (deck_z - 90) / 2), WOOD_DARK, "structure", "Reference-visible deck support post")
    for i, y in enumerate(np.linspace(deck_y0 + 430, cfg.rear_wall_y + 120, 5)):
        builder.box(f"east_deck_support_{i:02}", (95, 95, deck_z - 90), (side_x1 - 310, y, (deck_z - 90) / 2), WOOD_DARK, "structure", "Reference-visible wrap-deck support")

    builder.front_railing(x0, x1, deck_y0 - 24, deck_z + 30, "front_rail")
    builder.side_railing(deck_y0 - 20, cfg.rear_wall_y + 360, side_x1 + 10, deck_z + 30, "east_rail")

    nx0, nx1 = -15_900.0, -10_200.0
    nfront = -2_080.0
    for i, y in enumerate(np.arange(nfront + board_w / 2, -550, board_w + gap)):
        builder.box(f"neighbor_deck_board_{i:02}", (nx1 - nx0, board_w, 38), ((nx0 + nx1) / 2, y, deck_z), WHITE, "neighbor", "Neighbor deck context", provenance="reference-context")
    builder.front_railing(nx0, nx1, nfront, deck_z, "neighbor_front_rail", white=True)


def _neighbor(builder: Builder) -> None:
    builder.box("neighbor_lower_mass", (6_100, 7_250, 2_570), (-13_050, 2_950, 1_285), NEIGHBOR, "neighbor", "Visible west neighbor lower mass", provenance="reference-context")
    builder.box("neighbor_upper_mass", (6_100, 7_250, 2_650), (-13_050, 2_950, 3_895), NEIGHBOR, "neighbor", "Visible west neighbor upper mass", provenance="reference-context")
    builder.box("neighbor_roof", (6_450, 7_650, 145), (-13_050, 2_950, 5_295), ROOF, "neighbor", "Neighbor flat roof", provenance="reference-context")
    builder.box("neighbor_red_door", (820, 60, 1_980), (-14_360, -700, 3_850), RED_DOOR, "neighbor", "Muted red neighbor door", provenance="reference-context")
    builder.window("neighbor_slider", -11_740, 1_920, 3_050, 5_040, -700, panes=2, group="neighbor")


def _fence_and_patio_objects(builder: Builder) -> None:
    cfg = builder.cfg
    y = cfg.bluff_edge_y + 540
    x0, x1 = -11_900.0, 14_700.0
    for i, x in enumerate(np.linspace(x0, x1, 16)):
        builder.cylinder(f"bluff_fence_post_{i:02}", (x, y, 0), (x, y, 1_300), 24, METAL, "fence", "Property-edge fence post", sides=10)
    builder.cylinder("bluff_fence_top", (x0, y, 1_230), (x1, y, 1_230), 18, METAL, "fence", "Fence top wire", sides=8)
    builder.cylinder("bluff_fence_mid", (x0, y, 660), (x1, y, 660), 10, METAL, "fence", "Fence mid wire", sides=8)
    for j, z in enumerate((230, 420, 610, 800, 990, 1180)):
        builder.cylinder(f"fence_wire_h_{j}", (x0, y - 8, z), (x1, y - 8, z), 4.2, METAL, "fence", "Chain-link approximation horizontal wire", sides=6)
    span = 1_750.0
    for i, xa in enumerate(np.arange(x0, x1 - span, span)):
        builder.cylinder(f"fence_diag_a_{i}", (xa, y - 10, 170), (xa + span, y - 10, 1_180), 3.5, METAL, "fence", "Chain-link diagonal wire", sides=6)
        builder.cylinder(f"fence_diag_b_{i}", (xa, y + 10, 1_180), (xa + span, y + 10, 170), 3.5, METAL, "fence", "Chain-link diagonal wire", sides=6)

    builder.box("pingpong_top", (2_500, 1_370, 55), (12_280, -480, 760), TABLE, "patio_objects", "Reference-visible outdoor table")
    for i, (x, yy) in enumerate(((11_280, -930), (13_280, -930), (11_280, -30), (13_280, -30))):
        builder.box(f"pingpong_leg_{i}", (62, 62, 720), (x, yy, 380), FRAME, "patio_objects", "Outdoor table leg")
    builder.box("patio_bench_seat", (1_800, 360, 95), (14_250, 3_350, 500), WOOD, "patio_objects", "Simple weathered patio bench")
    for i, x in enumerate((13_620, 14_880)):
        builder.box(f"patio_bench_leg_{i}", (90, 300, 460), (x, 3_350, 240), WOOD_DARK, "patio_objects", "Bench leg")


def _vegetation(builder: Builder) -> None:
    rng = builder.rng
    for i in range(34):
        x = rng.uniform(-20_000, 20_000)
        y = rng.uniform(-8_200, -4_250)
        edge = builder.cfg.bluff_edge_y + 420 * math.sin(x / 4200) + 160 * math.sin(x / 1350)
        t = max(0.0, min(1.0, (edge - y) / 10_700.0))
        z = -70 - 8_150 * (t ** 0.62)
        if z < -2_050:
            continue
        builder.leaf_cluster(
            f"coastal_shrub_{i:02}", (x, y, z + 330),
            (rng.uniform(420, 900), rng.uniform(380, 820), rng.uniform(260, 500)),
            GREEN_LIGHT if i % 5 == 0 else GREEN_DARK, "vegetation",
            "Explicit coastal scrub foliage", count=34,
            leaf_length=(120, 250), leaf_width=(42, 90),
        )

    for i, y in enumerate(np.linspace(-250, 8_500, 10)):
        builder.leaf_cluster(
            f"east_hedge_{i:02}", (15_050 + rng.uniform(-120, 150), y, 690 + rng.uniform(-30, 100)),
            (650, 820, 690), GREEN_DARK, "vegetation",
            "Dense east property hedge", count=54, leaf_length=(120, 235), leaf_width=(38, 82),
        )

    for i, x in enumerate(np.linspace(-18_250, -10_900, 9)):
        builder.leaf_cluster(
            f"west_shrub_{i:02}", (x, -2_900 + rng.uniform(-350, 340), 610),
            (650, 560, 600), GREEN_DARK if i % 3 else GREEN_LIGHT, "vegetation",
            "West coastal hedge foliage", count=42, leaf_length=(125, 240), leaf_width=(42, 88),
        )

    base = np.array((10_450.0, -40.0, 0.0))
    tip = np.array((10_820.0, 150.0, 5_850.0))
    segments = 11
    for i in range(segments):
        a = base + (tip - base) * (i / segments)
        b = base + (tip - base) * ((i + 1) / segments)
        builder.cylinder(f"yucca_trunk_{i:02}", a, b, 215 - i * 10.5, TRUNK, "vegetation",
                         "Segmented weathered yucca/palm trunk", sides=14, provenance="procedural-site")
    crown = tip + np.array((0, 0, 90))
    for i in range(31):
        ang = TAU * i / 31 + (i % 4) * 0.06
        length = 1_180 + 470 * (0.25 + math.sin(i * 1.73) ** 2)
        end = crown + np.array((math.cos(ang) * length, math.sin(ang) * length, -90 + 620 * math.sin(i * 2.13)))
        mid = crown * 0.44 + end * 0.56 + np.array((0, 0, 330))
        width = 105 + 40 * math.sin(i * 1.31) ** 2
        tangent = end - crown; tangent /= np.linalg.norm(tangent)
        side = np.cross(tangent, np.array((0, 0, 1.0)))
        if np.linalg.norm(side) < 1e-6:
            side = np.array((1.0, 0, 0))
        side /= np.linalg.norm(side)
        verts = np.array([crown - side * width * 0.45, crown + side * width * 0.45,
                          mid + side * width * 0.60, end, mid - side * width * 0.60])
        faces = np.array([[0, 1, 2], [0, 2, 4], [4, 2, 3]], dtype=np.int64)
        builder.add(_part(f"yucca_leaf_{i:02}", verts, faces, LEAF, "vegetation",
                          "Procedural lance leaf", _smooth_normals(verts, faces), "procedural-site"))

    trunk0 = np.array((2_700.0, 8_650.0, 0.0))
    fork = np.array((2_850.0, 8_530.0, 4_850.0))
    top = np.array((2_950.0, 8_500.0, 7_450.0))
    builder.cylinder("mature_tree_trunk_0", trunk0, fork, 280, TRUNK, "vegetation",
                     "Mature context tree trunk", sides=16, provenance="procedural-site")
    builder.cylinder("mature_tree_trunk_1", fork, top, 205, TRUNK, "vegetation",
                     "Mature context tree upper trunk", sides=14, provenance="procedural-site")
    branch_tips = [
        (-1_150, 8_420, 7_300), (150, 7_780, 8_100), (1_450, 7_850, 8_350),
        (4_250, 7_850, 8_350), (5_550, 8_560, 7_900), (4_700, 9_600, 7_600),
        (2_850, 10_150, 8_100), (950, 9_760, 7_800),
    ]
    for i, bt in enumerate(branch_tips):
        root = fork + np.array((rng.uniform(-180, 180), rng.uniform(-150, 150), rng.uniform(250, 900)))
        builder.cylinder(f"mature_tree_branch_{i:02}", root, np.asarray(bt, dtype=float), 115 if i < 4 else 90,
                         TRUNK, "vegetation", "Mature context tree branch", sides=10, provenance="procedural-site")
    builder.leaf_cluster(
        "mature_tree_canopy", (2_450, 8_720, 8_020), (4_250, 2_350, 1_650),
        GREEN_DARK, "vegetation", "Explicit mature coastal tree canopy",
        count=430, leaf_length=(170, 390), leaf_width=(60, 145),
    )
    builder.leaf_cluster(
        "mature_tree_canopy_highlights", (2_150, 8_520, 8_180), (3_950, 2_100, 1_400),
        GREEN_LIGHT, "vegetation", "Sunward foliage variation",
        count=170, leaf_length=(150, 340), leaf_width=(52, 120),
    )


def build(config: HouseConfig | None = None) -> Assembly:
    cfg = config or HouseConfig()
    cfg.check()
    b = Builder(cfg)
    _site(b)
    _neighbor(b)
    _architecture(b)
    _decks_and_rails(b)
    _fence_and_patio_objects(b)
    _vegetation(b)

    shared = dict(
        projection="perspective",
        floor=False,
        tone_mapping="neutral",
        environment_strength=1.0,
        background_strength=1.0,
        background_color=(0.48, 0.64, 0.82),
        studio_style="outdoor",
        f_stop=11.0,
        exposure=1.05,
    )
    views = {
        "hero": View(az=318, el=24, scale=17_000, target=(1_100, -300, 2_700), focal_length_mm=43, title="6503 DEL PLAYA / OCEAN-BLUFF HERO", **shared),
        "deck": View(az=326, el=16, scale=10_500, target=(1_400, -800, 3_000), focal_length_mm=50, title="WRAP-AROUND WOOD DECK", **shared),
        "east": View(az=306, el=18, scale=12_000, target=(5_600, 1_000, 2_900), focal_length_mm=46, title="EAST PATIO / DECK SUPPORTS", **shared),
        "context": View(az=300, el=32, scale=26_000, target=(-500, -2_800, 1_700), focal_length_mm=45, title="EAST-END DEL PLAYA CONTEXT", **shared),
    }
    metadata = {
        "title": "6503 Del Playa / eastern Del Playa reference reconstruction",
        "source_repository": "cybrdelic/cybr-geo",
        "geometry_method": "CYBR GEO named procedural Part/Assembly geometry, millimetres, Z-up",
        "subject": "6503 Del Playa Drive, Isla Vista, California / Depressions Beach edge",
        "reference_urls": [
            "https://www.zillow.com/homedetails/6503-Del-Playa-Dr-2-Goleta-CA-93117/2079669030_zpid/",
            "https://www.californiabeaches.com/beach/depressions-beach/",
        ],
        "reference_observations": [
            "Two-level beige stucco oceanfront block with dark lower bay.",
            "Long weathered wood wrap-around upper deck with dense vertical balusters.",
            "East-side exposed deck supports, patio, leaning yucca/palm, table and bluff-edge fence.",
            "West neighboring light stucco building with white balcony rail is included only as visible context.",
        ],
        "no_image_generation": True,
        "no_photogrammetry": True,
        "no_scan_geometry": True,
        "no_scan_textures": True,
        "authorship": "All visible geometry and material parameters are explicit procedural/authored data.",
        "units": "mm",
        "axis": "Z-up; X approximately along shoreline; negative Y toward ocean",
        "uncertainty": {
            "survey_status": "No parcel survey, architectural drawings, or calibrated camera solution was available.",
            "dimensions": "Reference-estimated from public exterior photographs and site context; not claimed centimeter-exact.",
            "hidden_surfaces": "Hidden/interior details are intentionally omitted instead of invented.",
            "terrain": "Reference-matched procedural bluff context, not a DEM survey.",
        },
        "config": cfg.__dict__,
    }
    return Assembly("del_playa_6503_reference_reconstruction", b.parts, MATERIALS, views, metadata)


if __name__ == "__main__":
    assembly = build()
    print(assembly.name)
    print("parts", len(assembly.parts))
    print("triangles", sum(len(p.faces) for p in assembly.parts))
    print("bounds_mm", assembly.bounds.tolist())
