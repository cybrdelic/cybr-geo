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
from pathlib import Path
import json

import numpy as np

from mechanism_lab.core import Assembly, Material, Part, View

TAU = math.tau


@dataclass(frozen=True)
class HouseConfig:
    # Reference-estimated architectural dimensions, millimetres.
    main_width: float = 10_702.038834557388
    main_depth: float = 32_784.19833
    lower_height: float = 2_620.0
    upper_height: float = 2_720.0
    slab_z: float = 2_620.0
    roof_z: float = 5_340.0
    front_wall_y: float = 0.0
    rear_wall_y: float = 32_000.0
    deck_front_y: float = -1_950.0
    deck_side_x: float = 7_200.0
    deck_thickness: float = 145.0
    rail_height: float = 990.0
    bluff_edge_y: float = -5_900.0
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
    Material("Aged warm stucco", (0.45, 0.385, 0.285), 0.0, 0.86, microfinish="concrete", material_source="reference-authored"),
    Material("Aged stucco lighter patches", (0.51, 0.445, 0.34), 0.0, 0.90, microfinish="concrete", material_source="reference-authored"),
    Material("Dark lower cladding", (0.070, 0.048, 0.034), 0.0, 0.83, microfinish="wood", material_source="reference-authored"),
    Material("Weathered deck redwood", (0.12, 0.061, 0.045), 0.0, 0.82, microfinish="wood", material_source="reference-authored"),
    Material("Sun-bleached deck redwood", (0.18, 0.115, 0.090), 0.0, 0.88, microfinish="wood", material_source="reference-authored"),
    Material("Darkened exterior timber", (0.072, 0.040, 0.030), 0.0, 0.90, microfinish="wood", material_source="reference-authored"),
    Material("Window glass", (0.56, 0.69, 0.74), 0.0, 0.05, ior=1.52, opacity=0.27, material_source="reference-authored"),
    Material("Dark aluminum window frame", (0.055, 0.063, 0.066), 0.78, 0.32, microfinish="anodized", material_source="reference-authored"),
    Material("Aged concrete patio", (0.39, 0.365, 0.31), 0.0, 0.94, microfinish="concrete", material_source="reference-authored"),
    Material("Roof membrane", (0.185, 0.188, 0.185), 0.0, 0.95, microfinish="concrete", material_source="reference-authored"),
    Material("Galvanized fence", (0.42, 0.44, 0.43), 0.76, 0.42, microfinish="brushed", material_source="reference-authored"),
    Material("Dry bluff soil", (0.31, 0.22, 0.125), 0.0, 0.98, microfinish="concrete", material_source="procedural-site"),
    Material("Beach sand", (0.57, 0.49, 0.355), 0.0, 0.97, microfinish="concrete", material_source="procedural-site"),
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


MATERIALS.append(Material("Weathered terracotta brick", (0.26, 0.105, 0.060), 0.0, 0.9, microfinish="concrete", material_source="reference-authored"))
BRICK = len(MATERIALS)-1
MATERIALS.append(Material("Wet shoreline sand", (0.285,0.245,0.17), 0.0,0.46,material_source="procedural-site"))
WET_SAND=len(MATERIALS)-1
MATERIALS.append(Material("Park gravel", (0.41,0.395,0.345), 0.0,0.95,material_source="reference-context"))
PARK_GRAVEL=len(MATERIALS)-1
MATERIALS.append(Material("Coastal yellow flowers", (0.32,0.23,0.022), 0.0,0.82,material_source="reference-context"))
FLOWER=len(MATERIALS)-1

# Material indices retained as constants so the CYBR LIGHT bridge can preserve intent.
STUCCO, STUCCO_LIGHT, LOWER_DARK, WOOD, WOOD_LIGHT, WOOD_DARK, GLASS, FRAME, CONCRETE, ROOF, METAL, SOIL, SAND, WATER, GREEN_DARK, GREEN_LIGHT, DRY_GRASS, TRUNK, LEAF, NEIGHBOR, WHITE, RED_DOOR, TABLE = range(23)


def _part(name: str, vertices: np.ndarray, faces: np.ndarray, material: int, group: str, role: str,
          normals: np.ndarray | None = None, provenance: str = "reference-authored") -> Part:
    vertices = np.asarray(vertices, dtype=float)
    faces = np.asarray(faces, dtype=np.int64)
    triangles = vertices[faces]
    areas = np.linalg.norm(np.cross(triangles[:,1]-triangles[:,0], triangles[:,2]-triangles[:,0]),axis=1)
    faces = faces[areas > 1e-7]
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



_CHAMFER_FACES = None

def _chamfer_box(size, center, bevel=3.0, matrix=None):
    from scipy.spatial import ConvexHull
    global _CHAMFER_FACES
    h = np.asarray(size, dtype=float) * .5
    b = min(bevel, float(h.min()) * .28)
    points = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                signs = np.array((sx, sy, sz))
                for axis in range(3):
                    q = h - b
                    q[axis] = h[axis]
                    points.append(q * signs)
    points = np.asarray(points)
    if _CHAMFER_FACES is None:
        template = np.array([[sx*(1 if ax == 0 else .9), sy*(1 if ax == 1 else .9), sz*(1 if ax == 2 else .9)]
                             for sx in (-1,1) for sy in (-1,1) for sz in (-1,1) for ax in range(3)])
        faces = ConvexHull(template).simplices.copy()
        for face in faces:
            tri = template[face]
            if np.dot(np.cross(tri[1]-tri[0], tri[2]-tri[0]), tri.mean(axis=0)) < 0:
                face[1], face[2] = face[2], face[1]
        _CHAMFER_FACES = faces
    if matrix is not None:
        points = points @ np.asarray(matrix).T
    points += np.asarray(center)
    return _flat_mesh(points, _CHAMFER_FACES)

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
        v, f, n = (_chamfer_box(size, center, 2.5, matrix) if material in (WOOD, WOOD_LIGHT, WOOD_DARK, LOWER_DARK, FRAME, WHITE, BRICK)
                   else _box_geometry(size, center, matrix))
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


CONSTRAINTS_PATH = Path(__file__).with_name('site_constraints.json')


def site_constraints() -> dict:
    return json.loads(CONSTRAINTS_PATH.read_text())


def footprint_mm() -> np.ndarray:
    return np.asarray(site_constraints()['footprint']['local_m'], dtype=float) * 1000


def _polygon_triangles(points: np.ndarray) -> np.ndarray:
    """Ear clipping for the source outline; no learned or photo-derived mesh."""
    xy = np.asarray(points, float)
    area = np.sum(xy[:,0] * np.roll(xy[:,1], -1) - xy[:,1] * np.roll(xy[:,0], -1))
    indices = list(range(len(xy)))
    if area < 0:
        indices.reverse()
    faces = []
    def cross(a, b): return a[0]*b[1] - a[1]*b[0]
    while len(indices) > 3:
        found = False
        for i in range(len(indices)):
            a, b, c = indices[i-1], indices[i], indices[(i+1)%len(indices)]
            if cross(xy[b]-xy[a], xy[c]-xy[b]) <= 1e-6:
                continue
            inside = False
            for q in indices:
                if q in (a,b,c): continue
                if min(cross(xy[b]-xy[a],xy[q]-xy[a]), cross(xy[c]-xy[b],xy[q]-xy[b]), cross(xy[a]-xy[c],xy[q]-xy[c])) >= -1e-6:
                    inside = True; break
            if not inside:
                faces.append((a,b,c));indices.pop(i);found=True;break
        if not found:
            raise ValueError('Footprint is not a triangulable simple polygon')
    faces.append(tuple(indices))
    return np.asarray(faces, dtype=np.int64)


def _slab(b: Builder, name: str, outline: np.ndarray, z: float, thickness: float, material: int, group='architecture'):
    n = len(outline)
    vv = np.vstack((np.c_[outline,np.full(n,z)], np.c_[outline,np.full(n,z+thickness)]))
    top = _polygon_triangles(outline)
    faces = [*top[:,::-1].tolist(), *(top+n).tolist()]
    area = np.sum(outline[:,0]*np.roll(outline[:,1],-1)-outline[:,1]*np.roll(outline[:,0],-1))
    order = list(range(n)) if area>0 else list(reversed(range(n)))
    for a,c in zip(order,order[1:]+order[:1]):
        faces.extend([(a,c,c+n),(a,c+n,a+n)])
    v,f,normals = _flat_mesh(vv,np.asarray(faces))
    return b.add(_part(name,v,f,material,group,'Map-constrained polygon slab',normals))


def _wall(b: Builder, prefix: str, a, c, z0: float, z1: float, openings, material=STUCCO, thickness=190):
    """Build actual wall piers, sills and lintels around holes, in wall coordinates."""
    a,c = np.asarray(a,float),np.asarray(c,float)
    length = float(np.linalg.norm(c-a));u=(c-a)/length;out=np.array([u[1],-u[0]])
    matrix=np.array([[u[0],-u[1],0],[u[1],u[0],0],[0,0,1]])
    cuts=sorted(set([0.,length]+[max(0,min(length,x)) for o in openings for x in o[:2]]))
    def panel(name, l, r, low, high):
        if r-l<1 or high-low<1:return
        center=a+u*((l+r)/2)
        b.box(name,(r-l,thickness,high-low),(*center,(low+high)/2),material,'architecture','Wall panel around authored opening',matrix)
    for i,(l,r) in enumerate(zip(cuts,cuts[1:])):
        holes=[o for o in openings if o[0]<(l+r)/2<o[1]]
        spans=sorted((max(z0,o[2]),min(z1,o[3])) for o in holes)
        bottom=z0
        for j,(low,high) in enumerate(spans):
            panel(f'{prefix}_panel_{i}_{j}',l,r,bottom,low);bottom=max(bottom,high)
        panel(f'{prefix}_panel_{i}_top',l,r,bottom,z1)
    for j,(l,r,low,high,kind) in enumerate(openings):
        origin=a+u*((l+r)/2)+out*9
        # Single optical sheet, no two-surface sheet box with doubled reflectance.
        corners=np.array([np.r_[a+u*l+out*12,low],np.r_[a+u*r+out*12,low],np.r_[a+u*r+out*12,high],np.r_[a+u*l+out*12,high]])
        faces=np.array([[0,1,2],[0,2,3]])
        vv,ff,nn=_flat_mesh(corners,faces)
        name=f'{prefix}_opening_{j}'
        if kind=='door':
            b.box(name+'_door',(r-l,55,high-low),(*origin,(low+high)/2),LOWER_DARK,'fenestration','Observed opaque entry door',matrix)
        else:
            b.add(_part(name+'_glass',vv,ff,GLASS,'fenestration','Single authored glazing sheet in real wall opening',nn))
            panes=3 if kind=='grid' else 2
            for k in range(panes+1):
                xy=a+u*(l+(r-l)*k/panes)+out*26
                b.box(name+f'_mullion_{k}',(35 if k not in (0,panes) else 48,62,high-low),(*xy,(low+high)/2),FRAME,'fenestration','Slim aluminium mullion',matrix)
            if kind=='grid':
                for k in (1,2):
                    b.box(name+f'_crossbar_{k}',(r-l,64,28),(*origin,low+(high-low)*k/3),FRAME,'fenestration','Reference-visible divided window',matrix)
        for suffix,z in [('sill',low),('head',high)]:
            b.box(name+'_'+suffix,(r-l+82,95,55),(*origin,z),FRAME,'fenestration','Recessed sill or head',matrix)
        # Modest neutral recess returns; arrangement behind glass is explicitly unobserved.
        recess=a+u*((l+r)/2)-out*950
        b.box(name+'_room_back',(r-l+160,90,high-low+180),(*recess,(low+high)/2),STUCCO_LIGHT,'interior_proxy','Unobserved neutral depth proxy 0.95 m behind glazing',matrix,provenance='unobserved-presentation-proxy')
        bottomxy=a+u*((l+r)/2)-out*460
        b.box(name+'_room_sill',(r-l,980,75),(*bottomxy,low-35),CONCRETE,'interior_proxy','Unobserved recessed floor proxy',matrix,provenance='unobserved-presentation-proxy')


def _architecture(b: Builder):
    cfg=b.cfg;poly=footprint_mm()
    _slab(b,'lower_floor',poly,-90,125,CONCRETE)
    _slab(b,'upper_main_shell',poly,cfg.slab_z-135,135,STUCCO)
    upper_poly=np.vstack((poly[0],[-4600.,500.],poly[6:]))
    _slab(b,'roof_slab',upper_poly,cfg.roof_z,135,ROOF,'roof')
    # Source envelope is held fixed, facade heights/openings are manual reference estimates.
    area=np.sum(poly[:,0]*np.roll(poly[:,1],-1)-poly[:,1]*np.roll(poly[:,0],-1))
    if area<0:poly=poly[::-1]
    for i,(a,c) in enumerate(zip(poly,np.roll(poly,-1,axis=0))):
        length=float(np.linalg.norm(c-a));mid=(a+c)/2
        openings=[];upper=[]
        if mid[1]<600 and length>4000:
            # Main ocean facade, distances proportional to this measured map segment.
            openings=[(.075*length,.315*length,510,2220,'grid'),(.54*length,.85*length,160,2320,'slider')]
            upper=[(.045*length,.23*length,cfg.slab_z+250,cfg.roof_z-285,'slider'),(.29*length,.365*length,cfg.slab_z+600,cfg.roof_z-460,'slider'),(.425*length,.49*length,cfg.slab_z+110,cfg.roof_z-330,'door'),(.525*length,.765*length,cfg.slab_z+170,cfg.roof_z-310,'slider'),(.855*length,.985*length,cfg.slab_z+200,cfg.roof_z-295,'slider')]
        elif length>4500 and mid[0]>-5000:
            openings=[(.21*length,.66*length,190,2320,'slider')]
            upper=[(.12*length,.27*length,cfg.slab_z+130,cfg.roof_z-330,'door'),(.38*length,.80*length,cfg.slab_z+290,cfg.roof_z-335,'slider')]
        elif length>20000:
            # West wall largely occluded; no invented opening survey.
            upper=[]
        if not (mid[1]<600 and mid[0]<-3000):
            _wall(b,f'lower_wall_{i:02}',a,c,0,cfg.slab_z-130,openings)
    for i,(a,c) in enumerate(zip(upper_poly,np.roll(upper_poly,-1,axis=0))):
        length=float(np.linalg.norm(c-a));mid=(a+c)/2;openings=[]
        if mid[1]<2000 and length>6000:
            openings=[(.045*length,.23*length,cfg.slab_z+250,cfg.roof_z-285,'slider'),(.29*length,.365*length,cfg.slab_z+600,cfg.roof_z-460,'slider'),(.425*length,.49*length,cfg.slab_z+110,cfg.roof_z-330,'door'),(.525*length,.765*length,cfg.slab_z+170,cfg.roof_z-310,'slider'),(.855*length,.985*length,cfg.slab_z+200,cfg.roof_z-295,'slider')]
        elif length>4500 and mid[0]>-5000:
            openings=[(.12*length,.27*length,cfg.slab_z+130,cfg.roof_z-330,'door'),(.38*length,.80*length,cfg.slab_z+290,cfg.roof_z-335,'slider')]
        _wall(b,f'upper_wall_{i:02}',a,c,cfg.slab_z,cfg.roof_z,openings)
        u=(c-a)/length;matrix=np.array([[u[0],-u[1],0],[u[1],u[0],0],[0,0,1]])
        b.box(f'roof_fascia_{i}',(length+35,145,130),(*mid,cfg.roof_z+70),WOOD_DARK,'roof','Weathered roof edge fascia',matrix)
    # Lower projecting wood-clad bay and its small divided window, seen in ocean reference.
    _wall(b,'lower_dark_bay',(-5900,-520),(-3000,-520),0,2390,[(500,2300,560,2200,'grid')],LOWER_DARK,180)
    for i,x in enumerate(np.arange(-5850,-3000,115)):
        if -5400<x<-3600:
            for j,(z,h) in enumerate([(250,480),(2300,160)]):
                b.box(f'bay_cladding_{i}_{j}',(108,27,h),(x,-624,z),LOWER_DARK,'cladding','Individual vertical tongue-and-groove cladding')
        else:b.box(f'bay_cladding_{i}',(108,27,2360),(x,-624,1180),LOWER_DARK,'cladding','Individual vertical tongue-and-groove cladding')
    # Ocean-side low brick plinth/retaining lip seen below the ground-floor glazing.
    for row in range(4):
        for j,x in enumerate(np.arange(-2980-(row%2)*115,5850,240)):
            if x < -3030:continue
            b.box(f'front_plinth_brick_{row}_{j}',(230,108,64),(x,-1150,37+row*72),BRICK,'masonry','Individual staggered brick at visible patio plinth')
    for row in range(4):
        for j,y in enumerate(np.arange(-1080,4350,240)):
            b.box(f'east_plinth_brick_{row}_{j}',(108,230,64),(5900,y,37+row*72),BRICK,'masonry','Low east return of reference-visible brick lip')
    for i,(x,y) in enumerate([(-1000,4000),(-3300,11800),(-6800,22000),(-11600,28800)]):
        b.cylinder(f'roof_vent_{i}',(x,y,cfg.roof_z+125),(x,y,cfg.roof_z+465),75,METAL,'roof','Reference-compatible vent location, estimated',sides=18)
        b.cylinder(f'roof_vent_cap_{i}',(x,y,cfg.roof_z+460),(x,y,cfg.roof_z+505),115,METAL,'roof','Vent rain cap',sides=20)


def _deck_strip(b:Builder,prefix,a,c,width=1250,railing=True):
    a,c=np.asarray(a,float),np.asarray(c,float);length=np.linalg.norm(c-a);u=(c-a)/length;out=np.array([u[1],-u[0]])
    matrix=np.array([[u[0],-u[1],0],[u[1],u[0],0],[0,0,1]])
    z=b.cfg.slab_z+15
    # Standard-sized boards, staggered butt joints rather than 20-m-long planks.
    boards=int(length/146)
    for i in range(boards):
        along=(i+.5)*length/boards
        for j in range(max(1,int(math.ceil(width/2400)))):
            lo=j*2400;hi=min(width,(j+1)*2400)
            if hi-lo<15:continue
            xy=a+u*along+out*((lo+hi)/2)
            b.box(f'{prefix}_board_{i:03}_{j}',(length/boards-6,hi-lo-5,34),(*xy,z+b.rng.uniform(-.7,.7)),WOOD_LIGHT if b.rng.random()<.28 else WOOD,'balcony','Individual cross-deck weathered board',matrix)
    for i,t in enumerate(np.linspace(0,length,max(2,int(length/410)+1))):
        xy=a+u*t+out*(width/2)
        b.box(f'{prefix}_joist_{i}',(45,width,185),(*xy,z-114),WOOD_DARK,'structure','Exposed deck joist',matrix)
    edgea=a+out*width;edgec=c+out*width
    b.box(prefix+'_fascia',(length+80,90,220),(*(edgea+edgec)/2,z-94),WOOD_DARK,'balcony','Chamfered deck fascia',matrix)
    if not railing:return
    top=z+b.cfg.rail_height
    b.box(prefix+'_toprail',(length+80,85,65),(*(edgea+edgec)/2,top),WOOD,'balcony','Continuous cap rail',matrix)
    b.box(prefix+'_bottomrail',(length,48,57),(*(edgea+edgec)/2,z+140),WOOD_DARK,'balcony','Bottom rail',matrix)
    for i,t in enumerate(np.linspace(32,length-32,max(2,int(length/125)))):
        xy=edgea+u*t
        b.box(prefix+f'_baluster_{i:03}',(31,38,b.cfg.rail_height-158),(*xy,z+(b.cfg.rail_height+140)/2),WOOD_DARK,'balcony','Reference-dense vertical baluster',matrix)
    for i,t in enumerate(np.linspace(0,length,max(2,int(length/1850)+1))):
        xy=edgea+u*t
        b.box(prefix+f'_post_{i:02}',(88,88,b.cfg.rail_height+55),(*xy,z+(b.cfg.rail_height+55)/2),WOOD,'balcony','Deck guard post',matrix)
        under=xy-out*105
        b.box(prefix+f'_support_{i:02}',(90,90,z-100),(*under,(z-100)/2),WOOD_DARK,'structure','Exposed vertical support',matrix)
        # Fastener disks/bolt ends are modeled, not painted onto a map.
        for k,zz in enumerate((z+175,z+840)):
            q=np.r_[xy+out*46,zz];b.cylinder(prefix+f'_bolt_{i}_{k}',q,q+np.r_[out*5,0],8,METAL,'fasteners','Galvanized bolt head',sides=10)


def _decks_and_rails(b:Builder):
    _deck_strip(b,'front_deck',(-6350,-70),(5750,-70),width=1880)
    # Preserve conventional names used by downstream inspection/tests.
    next(p for p in b.parts if p.name=='front_deck_toprail').name='front_rail_toprail'
    for p in b.parts:
        if p.name.startswith('front_deck_baluster_'):p.name=p.name.replace('front_deck_baluster_','front_rail_baluster_')
    poly=footprint_mm()
    for i in range(6,17):
        if np.linalg.norm(poly[i+1]-poly[i])>1200:
            _deck_strip(b,f'east_deck_{i}',poly[i],poly[i+1],width=1160)
    # Light west neighbor rail, visually secondary.
    b.front_railing(-14450,-6700,-1350,b.cfg.slab_z+20,'neighbor_front_rail',white=True)


def terrain_z(x,y,cfg):
    edge=cfg.bluff_edge_y+260*math.sin(x/4300)+120*math.sin(x/1700)
    if y>=edge:return -120+8*math.sin(x/1700+y/3200)
    t=min(1,max(0,(edge-y)/5900))
    # Steeper irregular face with horizontal sediment breaks and deterministic rills.
    height=-140-7200*(t**.76)
    rills=(145*math.sin(x/690+2*t)+55*math.sin(x/260-4*t))*(math.sin(math.pi*t)**2)
    strata=46*math.sin(height/245+.12*math.sin(x/850))
    return height+rills+strata


def _height_mesh(b,name,xs,ys,fn,mat,role):
    xx,yy=np.meshgrid(xs,ys);zz=np.vectorize(fn)(xx,yy);v=np.c_[xx.ravel(),yy.ravel(),zz.ravel()]
    w=len(xs);a=(np.arange(len(ys)-1)[:,None]*w+np.arange(w-1)[None,:]).ravel()
    f=np.vstack([np.c_[a,a+1,a+w+1],np.c_[a,a+w+1,a+w]])
    b.add(_part(name,v,f,mat,'site',role,_smooth_normals(v,f),'procedural-site'))


def _site(b:Builder):
    cfg=b.cfg
    ys=np.unique(np.r_[np.linspace(-16000,-10500,32),np.linspace(-10500,-4500,88),np.linspace(-4500,38000,36),np.linspace(38000,110000,15)])
    xs=np.unique(np.r_[np.linspace(-500000,-38000,35),np.linspace(-38000,22000,190),np.linspace(22000,500000,45)])
    _height_mesh(b,'bluff_terrain',xs,ys,lambda x,y:terrain_z(x,y,cfg),SOIL,'Authored bluff; finer nonuniform mesh, not a DEM')
    _height_mesh(b,'beach_sand',np.linspace(-500000,500000,300),np.linspace(-38000,-11300,80),lambda x,y:-7480+(y+38000)*.006+20*math.sin(x/2400+y/3200),SAND,'Procedural beach below estimated bluff')
    _height_mesh(b,'ocean_surface',np.linspace(-600000,600000,350),np.linspace(-700000,-32500,190),lambda x,y:-7370+70*math.sin(x/1800+y/1450)+32*math.sin(x/420-y/710)+10*math.sin(x/165+y/195),WATER,'Explicit multi-scale coastal water surface')
    _height_mesh(b,'wet_shoreline',np.linspace(-500000,500000,300),np.linspace(-34300,-30600,18),lambda x,y:-7360+5*math.sin(x/4500),WET_SAND,'Authored damp shoreline band; no tide measurement')
    # Park ground and paths are reference/context estimates except the mapped path centerline.
    _height_mesh(b,'park_gravel_ground',np.linspace(10000,140000,50),np.linspace(-4200,100000,45),lambda x,y:-103+9*math.sin(x/2800+y/4700),PARK_GRAVEL,'Campus-side gravel context; extent/elevation estimated')
    points=np.array(site_constraints()['context']['paths'][0]['local_m'])*1000
    for i,(a,c) in enumerate(zip(points,points[1:])):
        u=(c-a)/np.linalg.norm(c-a);mat=np.array([[u[0],-u[1],0],[u[1],u[0],0],[0,0,1]])
        b.box(f'mapped_park_path_{i}',(np.linalg.norm(c-a)+200,1900,25),(*(a+c)/2,-78),CONCRETE,'site','OSM path centerline; width/elevation estimated',mat,provenance='mapped-centerline-estimated-section')
    # Open foreground plaza, benches and lamps correspond to visible park types, not surveyed positions.
    b.box('park_foreground_plaza',(17500,11800,90),(36000,4800,-74),PARK_GRAVEL,'site','Reference-visible gravel plaza; location and dimensions approximate',provenance='reference-context')
    for i,(a,c) in enumerate([((27000,-800),(27000,5500)),((27000,10700),(44500,10700))]):
        a=np.array(a);c=np.array(c);u=(c-a)/np.linalg.norm(c-a);mat=np.array([[u[0],-u[1],0],[u[1],u[0],0],[0,0,1]])
        b.box(f'park_seat_wall_{i}',(np.linalg.norm(c-a),420,410),(*(a+c)/2,150),CONCRETE,'site','Reference-visible low concrete seating wall; approximate position',mat,provenance='reference-context')
    for i,(x,y) in enumerate([(31500,1500),(37500,6500)]):
        b.box(f'park_bench_{i}_seat',(1700,470,55),(x,y,435),WHITE,'site','Reference-visible park bench type; approximate position',provenance='reference-context')
        for j,xx in enumerate((x-620,x+620)):
            b.box(f'park_bench_{i}_leg_{j}',(70,390,455),(xx,y,185),FRAME,'site','Bench support',provenance='reference-context')
    for i,(x,y) in enumerate([(24000,6800),(44000,22000),(16000,36500)]):
        b.cylinder(f'park_lamp_{i}',(x,y,-90),(x,y,3850),48,METAL,'site','Reference-visible unlit park lamp type; approximate position',sides=16,provenance='reference-context')
        b.cylinder(f'park_lamp_cap_{i}',(x,y,3860),(x,y,3910),290,METAL,'site','Broad shade of park path lamp',sides=24,provenance='reference-context')
    # Beach context is presentation-only outside the available footprint.
    b.box('main_patioslab',(12900,4800,90),(550,-2450,-55),CONCRETE,'site','Observed ocean-side patio; extent estimated')
    b.box('east_patioslab',(3900,7300,90),(6900,550,-65),CONCRETE,'site','Observed side patio; extent estimated')
    for i in range(5):
        b.box(f'patio_joint_{i}',(11,4680,8),(-4950+i*2460,-2460,-4),SOIL,'site','Construction joint in concrete')
    # Neutral seam-free far landform silhouettes, no photographic sky/terrain backdrop.
    _height_mesh(b,'inland_landform',np.linspace(-500000,500000,170),np.linspace(110000,480000,30),lambda x,y:-150+17000*(.25+.75*math.sin(x/58000)**2)*((y-110000)/370000)**1.4,SOIL,'Unsurveyed procedural distant landform context')


def _neighbor(b:Builder):
    b.box('neighbor_lower_mass',(7700,19500,2580),(-10400,8500,1290),NEIGHBOR,'neighbor','West neighbor context volume; photo estimate',provenance='reference-context')
    b.box('neighbor_upper_mass',(7700,19500,2670),(-10400,8500,3915),NEIGHBOR,'neighbor','West neighbor upper volume; photo estimate',provenance='reference-context')
    b.box('neighbor_roof',(8050,19850,145),(-10400,8500,5325),ROOF,'neighbor','Secondary flat roof',provenance='reference-context')
    for i,x in enumerate((-12600,-9200)):
        b.window(f'neighbor_window_{i}',x,1850,3060,5050,-1270,group='neighbor')
    b.box('neighbor_red_door',(760,60,1990),(-13600,-1285,3650),RED_DOOR,'neighbor','Observed muted red door',provenance='reference-context')


def _fence_and_patio_objects(b:Builder):
    y=b.cfg.bluff_edge_y+920;x0,x1=-15200,10200
    for i,x in enumerate(np.arange(x0,x1+1,1830)):
        b.cylinder(f'bluff_fence_post_{i}',(x,y,0),(x,y,1220),22,METAL,'fence','Fence post',sides=12)
    b.cylinder('bluff_fence_top',(x0,y,1160),(x1,y,1160),15,METAL,'fence','Galvanized top tube',sides=12)
    # Explicit 90-mm diamonds instead of enormous diagonal proxy wires.
    for direction in (-1,1):
        for i,offset in enumerate(np.arange(x0-1250,x1+1250,100)):
            ax=max(x0,offset);cx=min(x1,offset+direction*1120) if direction>0 else max(x0,offset-1120)
            if direction<0:ax=min(x1,offset)
            if abs(cx-ax)<10:continue
            za=80+abs(ax-offset);zc=80+abs(cx-offset)
            if max(za,zc)>1220:continue
            b.cylinder(f'chainlink_{direction}_{i}',(ax,y,za),(cx,y,zc),1.8,METAL,'fence','Actual fine chain-link diagonal',sides=5)
    # Reference table dimensions inferred from standard table, not used as a survey datum.
    b.box('pingpong_top',(2500,1390,36),(8070,-1550,755),TABLE,'patio_objects','Reference-visible outdoor table')
    for i,(x,yy) in enumerate(((7070,-2000),(9070,-2000),(7070,-1100),(9070,-1100))):
        b.box(f'pingpong_leg_{i}',(44,44,700),(x,yy,368),FRAME,'patio_objects','Table leg')
    b.box('patio_bench_seat',(1750,380,70),(8800,2650,440),WOOD,'patio_objects','Reference bench')
    for i,x in enumerate((8180,9410)):b.box(f'patio_bench_leg_{i}',(75,310,430),(x,2650,216),WOOD_DARK,'patio_objects','Bench leg')
    # Campus-facing boundary fence/hedge is visible in both references.
    for i,y in enumerate(np.arange(-2600,34000,1850)):
        x=9400-.38*max(0,y)
        b.box(f'east_boundary_post_{i}',(80,80,1740),(x,y,870),WHITE,'fence','Observed light side-boundary fence; alignment estimate')
        b.box(f'east_boundary_panel_{i}',(40,1780,1470),(x,y+925,825),WHITE,'fence','Side-boundary infill; photo-estimated extent')


def _vegetation(b:Builder):
    rng=b.rng
    for i in range(100):
        x=rng.uniform(-32000,16000);y=rng.uniform(-8700,-4700);z=terrain_z(x,y,b.cfg)
        if z < -4200:continue
        b.leaf_cluster(f'coastal_shrub_{i}',(x,y,z+330),(rng.uniform(400,850),rng.uniform(420,760),rng.uniform(320,630)),GREEN_LIGHT if i%5==0 else GREEN_DARK,'vegetation','Explicit coastal scrub leaves',count=450,leaf_length=(90,210),leaf_width=(35,85))
    for i in range(420):
        x=-29000+(i%42)*1030+rng.uniform(-380,380);y=-10900+(i//42)*610+rng.uniform(-190,190);z=terrain_z(x,y,b.cfg)
        b.leaf_cluster(f'bluff_groundcover_{i}',(x,y,z+90),(740,670,160),GREEN_LIGHT if i%6==0 else GREEN_DARK,'vegetation','Continuous irregular iceplant-like coastal groundcover',count=190,leaf_length=(110,230),leaf_width=(40,85))
    for i,y in enumerate(np.arange(-200,34000,1100)):
        x=9200-.38*max(y,0)
        b.leaf_cluster(f'east_hedge_{i}',(x,y,900),(780,850,920),GREEN_DARK,'vegetation','Explicit side hedge',count=850,leaf_length=(100,220),leaf_width=(40,85))
    for i,x in enumerate(np.arange(-24000,-6200,1100)):
        b.leaf_cluster(f'west_hedge_{i}',(x,-3400,780),(800,880,950),GREEN_LIGHT if i%4==0 else GREEN_DARK,'vegetation','West coastal hedge',count=700,leaf_length=(100,220),leaf_width=(35,80))
    # Dense coastal park beds break the gravel context into reference-visible planting patches.
    for i in range(120):
        x=rng.uniform(14000,71000);y=rng.uniform(-3500,40500)
        if 26500<x<45500 and -1500<y<11200:continue
        radius=rng.uniform(800,1900)
        b.leaf_cluster(f'park_coastal_bed_{i}',(x,y,90),(radius,radius*.7,240),GREEN_DARK,'vegetation','Reference-context low coastal planting',count=500,leaf_length=(160,290),leaf_width=(50,105))
        if i%3==0:
            b.leaf_cluster(f'park_flower_bed_{i}',(x,y,270),(radius*.75,radius*.58,120),FLOWER,'vegetation','Explicit yellow flower petals; estimated park planting',count=160,leaf_length=(25,50),leaf_width=(18,32))
    # A bent trunk is a continuous tapered ring mesh with bark ribs.
    base=np.array([7350.,-130.,0]);tip=np.array([8200.,100.,5900.]);vv=[];rings=34;sides=22
    for i in range(rings):
        t=i/(rings-1);p=base*(1-t)+tip*t+np.array([180*math.sin(t*math.pi),0,0]);radius=225*(1-.43*t)
        for j in range(sides):
            a=TAU*j/sides;r=radius*(1+.06*math.sin(7*a+27*t));vv.append(p+[r*math.cos(a),r*math.sin(a),0])
    ff=[]
    for i in range(rings-1):
        for j in range(sides):
            a=i*sides+j;c=i*sides+(j+1)%sides;d=a+sides;e=c+sides;ff.extend([(a,c,e),(a,e,d)])
    vv=np.asarray(vv);ff=np.asarray(ff);b.add(_part('yucca_trunk',vv,ff,TRUNK,'vegetation','Continuous bent/tapered trunk',_smooth_normals(vv,ff),'procedural-site'))
    for i in range(64):
        a=TAU*i/64;length=rng.uniform(1100,1900);down=rng.uniform(-1050,500);width=rng.uniform(52,105);verts=[]
        for j in range(11):
            t=j/10;centre=tip+np.array([math.cos(a)*length*t,math.sin(a)*length*t,480*math.sin(math.pi*t)+down*t]);side=np.array([-math.sin(a),math.cos(a),0])*width*math.sin(math.pi*t)*.5
            verts.extend([centre-side,centre+side])
        faces=[]
        for j in range(10):faces.extend([(2*j,2*j+1,2*j+3),(2*j,2*j+3,2*j+2)])
        v=np.asarray(verts);f=np.asarray(faces);b.add(_part(f'yucca_leaf_{i}',v,f,LEAF,'vegetation','Curved non-card yucca leaf',_smooth_normals(v,f),'procedural-site'))
    tree=np.array([5100.,12000.,0]);fork=tree+[0,0,4700]
    b.cylinder('mature_tree_trunk',tree,fork,230,TRUNK,'vegetation','Reference-visible context tree trunk',sides=20,provenance='procedural-site')
    for i in range(22):
        a=TAU*i/22;end=fork+np.array([math.cos(a)*rng.uniform(1100,3600),math.sin(a)*rng.uniform(1100,3400),rng.uniform(1700,3400)])
        b.cylinder(f'context_branch_{i}',fork,end,65 if i%3 else 105,TRUNK,'vegetation','Authored tree branch',sides=10,provenance='procedural-site')
        b.leaf_cluster(f'context_canopy_{i}',end,(1500,1250,850),GREEN_LIGHT if i%5==0 else GREEN_DARK,'vegetation','Dense curved leaves in irregular crown',count=1600,leaf_length=(100,240),leaf_width=(30,75))
    # Thin grass blades clustered rather than shrub ellipsoids.
    v=[];f=[]
    for i in range(2200):
        x=rng.uniform(-33000,18000);y=rng.uniform(-6400,-4500);z=terrain_z(x,y,b.cfg);height=rng.uniform(110,480);a=rng.uniform(0,TAU);w=rng.uniform(4,10);side=np.array([math.cos(a)*w,math.sin(a)*w,0]);root=np.array([x,y,z]);tip=root+[rng.uniform(-85,85),rng.uniform(-85,85),height];k=len(v);v.extend([root-side,root+side,tip]);f.append([k,k+1,k+2])
    v=np.asarray(v);f=np.asarray(f);b.add(_part('coastal_grass_blades',v,f,DRY_GRASS,'vegetation','Explicit fine coastal grass blades',_smooth_normals(v,f),'procedural-site'))


def build(config:HouseConfig|None=None)->Assembly:
    cfg=config or HouseConfig();cfg.check();b=Builder(cfg)
    _site(b);_neighbor(b);_architecture(b);_decks_and_rails(b);_fence_and_patio_objects(b);_vegetation(b)
    views={'hero':View(az=318,el=18,scale=17000,target=(0,-400,2700),floor=False,projection='perspective'), 'deck':View(az=326,el=14,scale=9000,target=(0,-900,3100),floor=False,projection='perspective'), 'context':View(az=40,el=25,scale=33000,target=(-5000,12000,1800),floor=False,projection='perspective')}
    constraints=site_constraints()
    metadata={'title':'6503 Del Playa — map-constrained, reference-authored reconstruction','subject':'6503 Del Playa Drive, Isla Vista, California','source_repository':'cybrdelic/cybr-geo','geometry_method':'Named CYBR GEO Part/Assembly; map outline plus authored construction detail','reference_urls':['https://www.zillow.com/homedetails/6503-Del-Playa-Dr-2-Goleta-CA-93117/2079669030_zpid/',constraints['footprint']['source_url'],constraints['parcel']['source_url'],'https://www.californiabeaches.com/beach/depressions-beach/'],'site_constraints':constraints,'no_image_generation':True,'no_photogrammetry':True,'no_scan_geometry':True,'no_scan_textures':True,'no_photo_textures':True,'units':'mm','axis':'Z up; X follows mapped ocean facade, negative Y oceanwards','config':cfg.__dict__,'uncertainty':{'survey_status':'No survey or architectural drawings. County assessment parcel and 2022 community-mapped OSM footprint are constraints, not legal/metrological truth.','dimensions':'Mapped plan envelope fixed; vertical heights, window sizes, deck and fence dimensions manually reference-estimated. Manual sparse camera alignment has no SfM, triangulated point cloud or reconstructed photographic mesh.','footprint':'Mapped polygon may combine exterior walls, recesses and balcony envelopes; not every jog has independent elevation confirmation.','hidden_surfaces':'Only neutral shallow recess geometry behind glass, tagged unobserved-presentation-proxy. No claim to an accurate interior.','terrain':'Finer authored coastal shape; no DEM or elevation survey is incorporated. Bluff profile, water level and beach slope remain estimated.','temporal':'Photo capture dates unknown; target is photographed appearance, not current structural condition.','context':'West neighbor, park plaza/furniture/planting and distant landform are reference/context estimates. Only the named park path centerline is map-constrained; its width and elevation are not. Upper ocean facade left corner is manually estimated separately from the mapped lower-floor envelope.'}}
    return Assembly('del_playa_6503_reference_reconstruction',b.parts,MATERIALS,views,metadata)


if __name__=='__main__':
    a=build();print(a.name);print('parts',len(a.parts));print('triangles',sum(len(p.faces) for p in a.parts));print('bounds_mm',a.bounds.tolist())
