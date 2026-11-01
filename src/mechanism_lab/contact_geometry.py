"""Reusable collision envelopes that retain declared openings.

Adapted from the annular-sector and strip algorithms in cybrdelic/cybr-arx6-belt
at aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2, integration/
build_p5_collision_proxies.py and build_p5_model.py. These are conservative
outer approximations, not a claim that every proxy lies inside native material.
False contacts near curved edges require native BREP refinement.
"""
from __future__ import annotations

from copy import deepcopy
import math

import numpy as np
from scipy.spatial.transform import Rotation


def _number(value, label, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float, np.number)):
        raise ValueError(f'{label} must be a finite number')
    value = float(value)
    if not math.isfinite(value) or (positive and value <= 0):
        raise ValueError(f'{label} must be finite' + (' and positive' if positive else ''))
    return value


def _vector(value, label, length=3):
    if not isinstance(value, (list, tuple, np.ndarray)) or len(value) != length:
        raise ValueError(f'{label} must have {length} coordinates')
    return np.asarray([_number(v, label) for v in value], dtype=float)


def _rotation(value):
    matrix = np.eye(3) if value is None else np.asarray(value, dtype=float)
    if (matrix.shape != (3, 3) or not np.isfinite(matrix).all()
            or not np.allclose(matrix.T @ matrix, np.eye(3), rtol=0, atol=1e-10)
            or not math.isclose(np.linalg.det(matrix), 1., rel_tol=0, abs_tol=1e-10)):
        raise ValueError('rotation must be a proper orthonormal 3 by 3 matrix')
    return matrix


def rotation_z(degrees):
    """Rotate about canonical +Z; angles are degrees, matching CAD frames."""
    angle = math.radians(_number(degrees, 'angle'))
    return np.array([[math.cos(angle), -math.sin(angle), 0.],
                     [math.sin(angle), math.cos(angle), 0.], [0., 0., 1.]])


def align_z(axis):
    """The source's shortest rotation from canonical +Z to a declared axis."""
    axis = _vector(axis, 'axis')
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        raise ValueError('axis must be nonzero')
    axis /= norm
    cross = np.cross([0, 0, 1], axis)
    length = np.linalg.norm(cross)
    if length < 1e-10:
        return np.eye(3) if axis[2] > 0 else np.diag([1., -1., -1.])
    return Rotation.from_rotvec(cross / length * math.acos(np.clip(axis[2], -1, 1))).as_matrix()


def _coverage(classification='declared_primitive_envelope', **bounds):
    return dict(classification=classification, material_containment_claimed=False,
                bounds_relative_to='declared_analytic_region',
                native_brep_equivalence='not_proven',
                complete_native_part_coverage_claimed=False,
                native_material_overcoverage_bound_mm=None, **bounds)


class ProxyBuilder:
    """Build body-local primitive records without CAD, file I/O, or global state.

    ``rotation`` and ``origin_mm`` map canonical coordinates into the owning
    body's coordinates. Box sizes are half extents, as used by MuJoCo. The
    builder owns its returned records; input specifications are never mutated.
    """

    def __init__(self, prefix=''):
        self.prefix = str(prefix)
        self.records = []
        self._names = set()

    def _append(self, name, body, kind, native_part, approximation, coverage, **geometry):
        if not isinstance(name, str) or not name or not isinstance(body, str) or not body:
            raise ValueError('Collision records require a name and owning body')
        name = self.prefix + name
        if name in self._names:
            raise ValueError(f'Duplicate collision record {name!r}')
        self._names.add(name)
        row = dict(name=name, body=body, type=kind, **geometry,
                   native_part=native_part or name.removeprefix(self.prefix),
                   approximation=approximation, coverage=deepcopy(coverage or _coverage()))
        self.records.append(row)
        return row

    def box(self, name, body, center_mm, half_size_mm, rotation=None,
            origin_mm=(0, 0, 0), native_part=None,
            approximation='Declared rectangular envelope', *, coverage=None):
        matrix = _rotation(rotation)
        center = _vector(origin_mm, 'origin') + matrix @ _vector(center_mm, 'center')
        half = _vector(half_size_mm, 'half size')
        if np.any(half <= 0):
            raise ValueError('Box half sizes must be positive')
        quat = Rotation.from_matrix(matrix).as_quat()
        return self._append(name, body, 'box', native_part, approximation, coverage,
                            center_mm=center.tolist(), half_size_mm=half.tolist(),
                            quat_wxyz=[float(quat[3]), *quat[:3].tolist()])

    def cylinder(self, name, body, start_mm, end_mm, radius_mm, rotation=None,
                 origin_mm=(0, 0, 0), native_part=None, *, coverage=None):
        matrix, origin = _rotation(rotation), _vector(origin_mm, 'origin')
        points = [_vector(p, 'cylinder endpoint') for p in (start_mm, end_mm)]
        if np.linalg.norm(points[1] - points[0]) < 1e-12:
            raise ValueError('Cylinder endpoints must differ')
        return self._append(name, body, 'cylinder', native_part,
                            'Exact cylindrical envelope', coverage,
                            fromto_mm=[(origin + matrix @ p).tolist() for p in points],
                            radius_mm=_number(radius_mm, 'radius', positive=True))

    def annulus(self, name, body, inner_radius_mm, outer_radius_mm, z_min_mm,
                z_max_mm, rotation=None, origin_mm=(0, 0, 0), center_xy_mm=(0, 0),
                sectors=24, native_part=None):
        """Cover an annulus with overlapping boxes while retaining a bounded bore.

        Each box spans radial [ri*cos(pi/n), ro] and tangential +/-ro*sin(pi/n).
        Thus every annular point is covered; a circle of radius ri*cos(pi/n)
        remains clear. The source's ro*(sec(pi/n)-1) is a conservative bound on
        outer radial excess, not an exact native-surface match.
        """
        ri, ro = _number(inner_radius_mm, 'inner radius'), _number(outer_radius_mm, 'outer radius')
        z0, z1 = _number(z_min_mm, 'z minimum'), _number(z_max_mm, 'z maximum')
        if ri < 0 or ro <= ri or z1 <= z0:
            raise ValueError('Annulus requires 0 <= inner < outer and z minimum < maximum')
        if isinstance(sectors, bool) or not isinstance(sectors, int) or not 3 <= sectors <= 4096:
            raise ValueError('Annulus requires 3 to 4096 sectors')
        matrix, origin = _rotation(rotation), _vector(origin_mm, 'origin')
        xy = _vector(center_xy_mm, 'annulus center', 2)
        delta = math.pi / sectors
        inner, outer = ri * math.cos(delta), ro
        radius = (inner + outer) / 2
        excess = ro * (1 / math.cos(delta) - 1)
        coverage = _coverage('conservative_outer_approximation', geometry='annulus',
                             sectors=sectors, inner_radius_mm=ri, outer_radius_mm=ro,
                             guaranteed_clear_radius_mm=inner,
                             max_inward_radial_overcoverage_mm=ri - inner,
                             max_outward_radial_overcoverage_mm=excess)
        start = len(self.records)
        for index in range(sectors):
            local = rotation_z(360 * index / sectors)
            center = np.array([*xy, (z0 + z1) / 2]) + local @ np.array([radius, 0, 0])
            row = self.box(f'{name}_rim_{index}', body, center,
                           [(outer - inner) / 2, ro * math.sin(delta), (z1 - z0) / 2],
                           matrix @ local, origin, native_part or name,
                           'Overlapping circumscribed annular sector; max radial excess %.4f mm' % excess,
                           coverage=coverage)
            # The source center is already in canonical parent coordinates;
            # only its orientation, not the center, receives the sector rotation.
            row['center_mm'] = (origin + matrix @ center).tolist()
        return self.records[start:]

    def capsule_plate_strips(self, name, body, radius_mm, center_distance_mm,
                             z_min_mm, z_max_mm, holes, *, strip_width_mm=4.,
                             rotation=None, origin_mm=(0, 0, 0), native_part=None):
        """Outer-envelope strips of an X-aligned capsule minus circular openings.

        Opening widths use the farthest strip edge; they never enlarge a true
        hole. Narrowing and curved outer-edge overcoverage can cause contacts
        absent from exact CAD. Small unspecified holes are not represented.
        """
        radius = _number(radius_mm, 'capsule radius', positive=True)
        distance = _number(center_distance_mm, 'capsule center distance')
        width = _number(strip_width_mm, 'strip width', positive=True)
        z0, z1 = _number(z_min_mm, 'z minimum'), _number(z_max_mm, 'z maximum')
        if distance < 0 or z1 <= z0 or math.ceil(2 * radius / width) > 100000:
            raise ValueError('Unsupported capsule dimensions or strip count')
        parsed = [(_vector(h['center_xy_mm'], 'hole center', 2),
                   _number(h['radius_mm'], 'hole radius', positive=True)) for h in holes]
        start = len(self.records)
        for index, y0 in enumerate(np.arange(-radius, radius, width)):
            y1 = min(radius, y0 + width)
            near = 0 if y0 <= 0 <= y1 else min(abs(y0), abs(y1))
            far = max(abs(y0), abs(y1))
            outer = math.sqrt(max(0., radius * radius - near * near))
            intervals = [[-outer, distance + outer]]
            for (cx, cy), hole_radius in parsed:
                hole_far = max(abs(y0 - cy), abs(y1 - cy))
                if hole_far >= hole_radius:
                    continue
                dx = math.sqrt(hole_radius * hole_radius - hole_far * hole_far)
                lo, hi, remaining = cx - dx, cx + dx, []
                for a, b in intervals:
                    if hi <= a or lo >= b:
                        remaining.append([a, b])
                    else:
                        if a < lo:
                            remaining.append([a, lo])
                        if hi < b:
                            remaining.append([hi, b])
                intervals = remaining
            coverage = _coverage('conservative_outer_approximation', geometry='capsule_plate_strips',
                                 max_strip_width_mm=width,
                                 max_outward_radial_overcoverage_mm=math.hypot(outer, far) - radius,
                                 hole_clearance_bounds=[dict(center_xy_mm=xy.tolist(),
                                      native_radius_mm=hr, guaranteed_clear_radius_mm=max(0., hr - width))
                                      for xy, hr in parsed])
            for piece, (a, b) in enumerate(intervals):
                if b - a > 1e-8:
                    self.box(f'{name}_{index}_{piece}', body,
                             [(a + b) / 2, (y0 + y1) / 2, (z0 + z1) / 2],
                             [(b - a) / 2, (y1 - y0) / 2, (z1 - z0) / 2],
                             rotation, origin_mm, native_part or name,
                             f'Conservative {width:g} mm strip of capsule minus circular bores',
                             coverage=coverage)
        return self.records[start:]


def tabletop_tiles(config, *, strip_width_mm=1., body='world', first_name='floor',
                   name_prefix='table_material_'):
    """Tile a Y-up rectangular plate with separated circular XZ openings.

    ``extent_xz_mm`` contains half extents, matching the recovered source model.
    Holes must have disjoint X intervals and fit within the plate. The returned
    records are conservative; the configuration is not changed.
    """
    half_x, half_z = _vector(config['extent_xz_mm'], 'table half extents', 2)
    thickness = _number(config['thickness_mm'], 'table thickness', positive=True)
    top = _number(config['top_y_mm'], 'table top')
    width = _number(strip_width_mm, 'strip width', positive=True)
    if min(half_x, half_z) <= 0:
        raise ValueError('Table half extents must be positive')
    holes = sorted([(_vector(h['center_xz_mm'], 'hole center', 2),
                     _number(h['radius_mm'], 'hole radius', positive=True))
                    for h in config['holes']], key=lambda h: h[0][0])
    left = -half_x
    for (cx, cz), radius in holes:
        if (cx - radius < left - 1e-8 or cx + radius > half_x
                or abs(cz) + radius > half_z or math.ceil(2 * radius / width) > 100000):
            raise ValueError('Table tiling requires contained openings with separated X intervals')
        left = cx + radius
    builder = ProxyBuilder()
    coverage = _coverage('conservative_outer_approximation', geometry='perforated_tabletop',
                         max_strip_width_mm=width,
                         hole_clearance_bounds=[dict(center_xz_mm=xy.tolist(), native_radius_mm=radius,
                              guaranteed_clear_radius_mm=max(0., radius - width)) for xy, radius in holes])

    def tile(x0, x1, z0, z1):
        if x1 - x0 < 1e-8 or z1 - z0 < 1e-8:
            return
        name = first_name if not builder.records else f'{name_prefix}{len(builder.records):03d}'
        builder.box(name, body, [(x0 + x1) / 2, top - thickness / 2, (z0 + z1) / 2],
                    [(x1 - x0) / 2, thickness / 2, (z1 - z0) / 2], native_part='tabletop',
                    approximation='Conservative plate strips; aperture lies within each circular opening',
                    coverage=coverage)

    left = -half_x
    for (cx, cz), radius in holes:
        tile(left, cx - radius, -half_z, half_z)
        edges = np.linspace(cx - radius, cx + radius, int(np.ceil(2 * radius / width)) + 1)
        for x0, x1 in zip(edges[:-1], edges[1:]):
            x = max(abs(x0 - cx), abs(x1 - cx))
            opening = float(np.sqrt(max(0., radius * radius - x * x)))
            tile(x0, x1, -half_z, cz - opening)
            tile(x0, x1, cz + opening, half_z)
        left = cx + radius
    tile(left, half_x, -half_z, half_z)
    return builder.records
