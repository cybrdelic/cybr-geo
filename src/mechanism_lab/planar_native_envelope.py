"""Bind one planar closure source to full-range, native STEP motion screening.

This adapter contains no example-specific geometry or fitted physical constants.
Every body has an explicit construction role and every checked part is imported
from a verified body-local STEP. The analytic bound covers the ideal rigid
one-degree-of-freedom branch; actual MuJoCo departures, intentional contacts,
part variation and fabrication evidence require their own treatment.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
from itertools import combinations
import json
import math
from pathlib import Path

import numpy as np

from .native_pose_envelope import certify_parametric_path, load_placed_native_parts
from .motion_envelope import _box, _brep_fingerprint
from .planar_linkage import PlanarFourBar


_DEFAULT_ANCHOR = dict(crank='input_pivot_mm', rocker='output_pivot_mm',
                       coupler='crank_pin_mm')
_ANGLE_KEY = dict(crank='crank_angle_rad', rocker='rocker_angle_rad',
                  coupler='coupler_angle_rad')
_ANCHORS = frozenset(('input_pivot_mm', 'output_pivot_mm', 'crank_pin_mm',
                      'rocker_pin_mm'))
_IMPLEMENTATION_FILES = ('planar_native_envelope.py', 'planar_linkage.py',
                         'native_pose_envelope.py', 'motion_envelope.py',
                         'native_construction.py', 'design_envelope.py', 'core.py')


def _finite(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{label} must be finite')
    return float(value)


def _rotation(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s], [s, c]])


@dataclass(frozen=True)
class PlanarBody:
    """One rigid body on a source-bounded planar closure branch.

    `world_rotation_rad` repeats a complete base linkage about world origin;
    `yaw_offset_rad` changes only this body's local CAD coordinate basis.
    A reversed connector, attached to the rocker pin rather than the crank
    pin, uses `anchor='rocker_pin_mm'` and a pi yaw offset.
    """

    role: str
    linkage: PlanarFourBar | None = None
    world_rotation_rad: float = 0.
    yaw_offset_rad: float = 0.
    anchor: str | None = None
    z_mm: float = 0.

    def __post_init__(self):
        if self.role == 'ground':
            if self.linkage is not None or self.anchor is not None:
                raise ValueError('A grounded body has no planar linkage or moving anchor')
        elif self.role not in _DEFAULT_ANCHOR or not isinstance(self.linkage, PlanarFourBar):
            raise ValueError('A moving body needs a crank, coupler or rocker linkage')
        if self.anchor is not None and self.anchor not in _ANCHORS:
            raise ValueError('Unknown planar body attachment anchor')
        _finite(self.world_rotation_rad, 'World rotation')
        _finite(self.yaw_offset_rad, 'Body coordinate yaw offset')
        _finite(self.z_mm, 'Axial body placement')

    @property
    def effective_anchor(self):
        return self.anchor or _DEFAULT_ANCHOR.get(self.role)

    def source(self):
        return dict(role=self.role, linkage=None if self.linkage is None else self.linkage.source(),
                    world_rotation_rad=self.world_rotation_rad,
                    yaw_offset_rad=self.yaw_offset_rad, anchor=self.effective_anchor,
                    z_mm=self.z_mm)


class PlanarNativePath:
    """Complete named body-pose and point-speed source for one input sweep."""

    def __init__(self, bodies, input_start_rad, input_end_rad, *, source_identity):
        if (not isinstance(bodies, dict) or len(bodies) < 2
                or any(not isinstance(name, str) or not name or not isinstance(value, PlanarBody)
                       for name, value in bodies.items())
                or not any(value.role != 'ground' for value in bodies.values())):
            raise ValueError('Planar path requires named ground/moving body bindings')
        if not isinstance(source_identity, str) or not source_identity.strip():
            raise ValueError('Planar path requires an authored source identity')
        start = _finite(input_start_rad, 'Input start')
        end = _finite(input_end_rad, 'Input end')
        if start == end:
            raise ValueError('Planar path input interval must move')
        lower, upper = sorted((start, end))
        # This fails before any CAD query when the continuous branch reaches a
        # toggle or unreachable configuration, including hidden mid-intervals.
        bounds = {}
        for value in bodies.values():
            if value.linkage is not None:
                bounds[value.linkage.source_sha256] = value.linkage.angular_rate_bounds(lower, upper)
        self.bodies = dict(bodies)
        self.input_start_rad, self.input_end_rad = start, end
        self.source_identity = source_identity
        self.angular_bounds = bounds
        self._body_sources = {name: body.source() for name, body in sorted(bodies.items())}
        directory = Path(__file__).resolve().parent
        # A certificate from older closure, pose, CAD or distance arithmetic
        # must not share identity with a certificate from revised code.
        self.implementation_sha256 = {name: hashlib.sha256(
            (directory / name).read_bytes()).hexdigest() for name in _IMPLEMENTATION_FILES}
        self.source_sha256 = hashlib.sha256(json.dumps(dict(schema='cybrgeo-planar-native-path/1',
            source_identity=source_identity, input_interval_rad=(start, end),
            bodies=self._body_sources,
            implementation_sha256=self.implementation_sha256),
            sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()

    def angle_at(self, u):
        u = _finite(u, 'Normalized input parameter')
        if not 0 <= u <= 1:
            raise ValueError('Normalized input parameter must be within [0,1]')
        return self.input_start_rad + (self.input_end_rad-self.input_start_rad)*u

    def pose_at(self, u):
        angle = self.angle_at(u)
        answers = {}
        solved = {}
        for name, body in self.bodies.items():
            if body.role == 'ground':
                answers[name] = np.eye(4)
                continue
            key = body.linkage.source_sha256
            if key not in solved:
                solved[key] = body.linkage.solve(angle)
            row = solved[key]
            point = np.asarray(row[body.effective_anchor], dtype=float)
            world = _rotation(body.world_rotation_rad)
            point = world @ point
            yaw = row[_ANGLE_KEY[body.role]] + body.yaw_offset_rad + body.world_rotation_rad
            pose = np.eye(4)
            pose[:2, :2] = _rotation(yaw)
            pose[:2, 3] = point
            pose[2, 3] = body.z_mm
            answers[name] = pose
        return answers

    def point_speed_bound(self, body_name, body_local_radius_mm, lo=0., hi=1.):
        """Conservative mm per normalized input unit for every CAD point."""
        if body_name not in self.bodies:
            raise ValueError('Unknown native body')
        radius = _finite(body_local_radius_mm, 'Body-local CAD corner radius')
        if radius < 0:
            raise ValueError('Body-local CAD corner radius must be nonnegative')
        lo, hi = _finite(lo, 'Interval start'), _finite(hi, 'Interval end')
        if not 0 <= lo <= hi <= 1:
            raise ValueError('Normalized interval must lie within [0,1]')
        body = self.bodies[body_name]
        if body.role == 'ground':
            return 0.
        start, end = sorted((self.angle_at(lo), self.angle_at(hi)))
        bounds = body.linkage.angular_rate_bounds(start, end)
        a, c = body.linkage.crank_length_mm, body.linkage.rocker_length_mm
        rocker_rate = bounds['maximum_rocker_angular_rate_rad_per_input_rad']
        coupler_rate = bounds['maximum_coupler_angular_rate_rad_per_input_rad']
        translation = dict(input_pivot_mm=0., output_pivot_mm=0.,
                           crank_pin_mm=a, rocker_pin_mm=c*rocker_rate)[body.effective_anchor]
        rotation = dict(crank=1., rocker=rocker_rate, coupler=coupler_rate)[body.role]
        return abs(self.input_end_rad-self.input_start_rad)*(translation+radius*rotation)

    def _assert_planar_source_unchanged(self):
        """Fail closed when a constructed path or its proof functions have changed."""
        if (type(self) is not PlanarNativePath
                or any(type(body) is not PlanarBody for body in self.bodies.values())
                or any(name in self.__dict__ or getattr(type(self), name) is not original
                       for name, original in _BASE_PLANAR_METHODS.items())):
            raise ValueError('Native planar proof requires unmodified pose and speed construction')
        body_sources = {name: body.source() for name, body in sorted(self.bodies.items())}
        if body_sources != self._body_sources:
            raise ValueError('Planar body source changed after the path was constructed')
        directory = Path(__file__).resolve().parent
        current = {name: hashlib.sha256((directory / name).read_bytes()).hexdigest()
                   for name in _IMPLEMENTATION_FILES}
        if current != self.implementation_sha256:
            raise ValueError('Planar clearance implementation changed after path construction')
        identity = dict(schema='cybrgeo-planar-native-path/1',
            source_identity=self.source_identity,
            input_interval_rad=(self.input_start_rad, self.input_end_rad),
            bodies=body_sources, implementation_sha256=current)
        actual = hashlib.sha256(json.dumps(identity, sort_keys=True,
            separators=(',', ':'), allow_nan=False).encode()).hexdigest()
        if actual != self.source_sha256:
            raise ValueError('Planar path identity changed after construction')
        return current

    def certify_native(self, native_directory, verified_receipt, *, clearance_pairs=None,
                       excluded_pairs=None, required_clearance_mm=0.,
                       distance_margin_mm=1e-4, max_depth=12,
                       max_pair_intervals=4095, max_total_intervals=1000000):
        """Check imported STEP for all or explicitly selected cross-body pairs.

        Selected subsets record every unexamined pair. Intentional bearing,
        drive and contact pairs must be named with a reason; they are never
        silently treated as noninterfering or as qualified fits.
        """
        self._assert_planar_source_unchanged()
        hashes = verified_receipt.get('files_sha256') if isinstance(verified_receipt, dict) else None
        if not isinstance(hashes, dict) or not hashes:
            raise ValueError('A verified native construction receipt is required')
        parts, imported = load_placed_native_parts(native_directory,
            expected_file_hashes=hashes)
        if {part['body'] for part in parts.values()} != set(self.bodies):
            raise ValueError('Planar path body set differs from verified native CAD')
        radii = imported['body_local_point_radius_mm']
        def bound(part_name, lo, hi):
            return self.point_speed_bound(parts[part_name]['body'], radii[part_name], lo, hi)
        report = certify_parametric_path(parts, self.pose_at, bound,
            source_identity=f'{self.source_sha256}|native-files:{imported["native_file_set_sha256"]}',
            clearance_pairs=clearance_pairs, excluded_pairs=excluded_pairs,
            required_clearance_mm=required_clearance_mm,
            distance_margin_mm=distance_margin_mm, max_depth=max_depth,
            max_pair_intervals=max_pair_intervals,
            max_total_intervals=max_total_intervals)
        report['native_step_import'] = imported
        report['planar_kinematics'] = dict(source_sha256=self.source_sha256,
            input_interval_rad=[self.input_start_rad, self.input_end_rad],
            implementation_sha256=self.implementation_sha256,
            branch_bounds=list(self.angular_bounds.values()),
            scope='Source-derived ideal rigid closure and conservative body point-speed bounds; no MuJoCo-between-step or physical qualification')
        self._assert_planar_source_unchanged()
        return report

    def axial_clearance_broadphase(self, native_directory, verified_receipt, *,
                                   required_clearance_mm=0., distance_margin_mm=1e-4):
        """Prove only pairs separated by invariant axial slabs for the full path.

        This bound needs no sampling or B-rep distance query. ``pose_at`` is
        constructed solely from XY translation and Z-axis rotation, with a
        constant per-body Z offset. A body's local native STEP Z interval is
        therefore invariant throughout the complete input interval. If two
        expanded intervals are disjoint, their 3D solids cannot collide at
        any input angle. Overlapping/tight intervals remain *unresolved*;
        this method never treats them as contacts or as clearances.
        """
        current = self._assert_planar_source_unchanged()
        for label, value in (('Required service clearance', required_clearance_mm),
                             ('Numerical distance margin', distance_margin_mm)):
            _finite(value, label)
        if required_clearance_mm < 0 or distance_margin_mm <= 0:
            raise ValueError('Required clearance must be nonnegative and numerical margin positive')
        hashes = verified_receipt.get('files_sha256') if isinstance(verified_receipt, dict) else None
        if not isinstance(hashes, dict) or not hashes:
            raise ValueError('A verified native construction receipt is required')
        parts, imported = load_placed_native_parts(native_directory,
            expected_file_hashes=hashes)
        if (not imported['complete_native_inventory_selected']
                or {part['body'] for part in parts.values()} != set(self.bodies)):
            raise ValueError('Planar path must bind the complete verified native body inventory')
        manifest_path = Path(native_directory) / 'placed-native-parts.json'
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        step_paths = {f"{row['body']}__{row['name']}": row['step_file']
                      for row in manifest['records']}
        if (set(step_paths) != set(parts) or hashlib.sha256(manifest_path.read_bytes()).hexdigest()
                != imported['native_files_sha256']['placed-native-parts.json']):
            raise ValueError('Native placement manifest changed during axial screening')
        intervals = {}
        for name, row in sorted(parts.items()):
            box = _box(row['shape'])
            body = self.bodies[row['body']]
            # The ground pose is identity; moving body poses are XY rotations
            # with the source-declared constant z_mm translation.
            offset = 0. if body.role == 'ground' else body.z_mm
            scale = max(abs(float(box[0, 2])), abs(float(box[1, 2])), abs(offset), 1.)
            # Expand the kernel's bounding interval and all additions outward.
            # This is numerical allowance, not manufacturing part variation.
            outward = 1e-9 + 64 * np.finfo(float).eps * scale
            low = math.nextafter(float(box[0, 2]) + offset - outward, -math.inf)
            high = math.nextafter(float(box[1, 2]) + offset + outward, math.inf)
            if not math.isfinite(low) or not math.isfinite(high) or low >= high:
                raise ValueError(f'{name} has an invalid axial native CAD bound')
            intervals[name] = dict(body=row['body'], z_interval_mm=[low, high],
                native_brep_sha256=_brep_fingerprint(row['shape']),
                step_sha256=imported['native_files_sha256'][step_paths[name]])
        separated, unresolved = [], []
        threshold = float(required_clearance_mm) + float(distance_margin_mm)
        for first, second in combinations(sorted(parts), 2):
            a, b = intervals[first], intervals[second]
            if a['body'] == b['body']:
                continue
            lower = max(0., a['z_interval_mm'][0] - b['z_interval_mm'][1],
                        b['z_interval_mm'][0] - a['z_interval_mm'][1])
            row = dict(parts=[first, second], bodies=[a['body'], b['body']],
                axial_gap_lower_bound_mm=lower)
            if lower > threshold:
                row['certified_clearance_lower_bound_mm'] = lower - float(distance_margin_mm)
                separated.append(row)
            else:
                row['reason'] = 'Axial slabs overlap or do not exceed required clearance plus numerical margin'
                unresolved.append(row)
        inventory = len(separated) + len(unresolved)
        identity = dict(planar_source_sha256=self.source_sha256,
            native_file_set_sha256=imported['native_file_set_sha256'],
            native_build_sha256=hashlib.sha256(json.dumps(verified_receipt,
                sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest(),
            implementation_sha256=current, required_clearance_mm=float(required_clearance_mm),
            distance_margin_mm=float(distance_margin_mm))
        self._assert_planar_source_unchanged()
        if (hashlib.sha256(manifest_path.read_bytes()).hexdigest()
                   != imported['native_files_sha256']['placed-native-parts.json']
                or any(hashlib.sha256((Path(native_directory) / relative).read_bytes()).hexdigest()
                       != imported['native_files_sha256'][relative]
                       for relative in step_paths.values())):
            raise ValueError('Planar source, implementation or native STEP changed during axial screening')
        return dict(schema='cybrgeo-planar-native-axial-broadphase/1',
            screened=True, status=('all_cross_body_pairs_axially_separated' if not unresolved
                                    else 'unresolved_cross_body_pairs_remain'),
            full_native_noncontact_pair_coverage=not unresolved,
            full_native_inventory_selected=True,
            input_interval_rad=[self.input_start_rad, self.input_end_rad],
            source_identity=self.source_identity,
            branch_bounds=list(self.angular_bounds.values()),
            source_binding=identity,
            input_sha256=hashlib.sha256(json.dumps(identity, sort_keys=True,
                separators=(',', ':'), allow_nan=False).encode()).hexdigest(),
            native_step_import=imported, part_axial_intervals=intervals,
            cross_body_pairs=inventory, axially_separated_pairs=separated,
            unresolved_pairs=unresolved,
            axially_separated_pair_count=len(separated),
            unresolved_pair_count=len(unresolved),
            required_clearance_mm=float(required_clearance_mm),
            distance_margin_mm=float(distance_margin_mm),
            scope='Exact-source ideal rigid planar path: verified native STEP Z slabs remain invariant under XY translation/Z rotation for the full interval. Only disjoint slabs are certified clear.',
            limitations=['Overlapping/tight slabs are unresolved and need exact swept B-rep or intentional-mate evidence.',
                         'No part variation, tolerance stack, MuJoCo departure, deformation, contact dynamics or hardware qualification.'])

    def gravity_budget(self, native_directory, verified_receipt,
                       gravity_m_s2, plane_normal):
        """Bound only CAD-mass, quasi-static gravity demand at the input.

        The native mass/COM document must be in the verified construction
        receipt and cover every bound body. The COM point-speed proof is the
        same analytic closure used by the native clearance path.
        """
        self._assert_planar_source_unchanged()
        from .design_envelope import planar_gravity_torque_bound

        hashes = verified_receipt.get('files_sha256') if isinstance(verified_receipt, dict) else None
        relative = 'mass_properties.json'
        if not isinstance(hashes, dict) or relative not in hashes:
            raise ValueError('Verified native mass properties are required')
        root = Path(native_directory).resolve()
        payload = (root / relative).read_bytes()
        actual_hash = hashlib.sha256(payload).hexdigest()
        if actual_hash != hashes[relative]:
            raise ValueError('Native CAD mass properties differ from the verified receipt')
        masses = json.loads(payload)
        if not isinstance(masses, dict) or set(masses) != set(self.bodies):
            raise ValueError('Native CAD mass body inventory differs from planar construction')
        cad_masses, com_speeds = {}, {}
        input_span = abs(self.input_end_rad-self.input_start_rad)
        for body, record in masses.items():
            if not isinstance(record, dict):
                raise ValueError(f'Native CAD body {body} lacks mass properties')
            mass, com = record.get('mass_kg'), record.get('com_m')
            if (isinstance(mass, bool) or not isinstance(mass, (int, float))
                    or not math.isfinite(mass) or mass <= 0
                    or not isinstance(com, list) or len(com) != 3
                    or any(isinstance(value, bool) or not isinstance(value, (int, float))
                           or not math.isfinite(value) for value in com)):
                raise ValueError(f'Native CAD body {body} has invalid mass or COM')
            cad_masses[body] = float(mass)
            radius_mm = math.hypot(float(com[0]), float(com[1]))*1000.
            com_speeds[body] = self.point_speed_bound(body, radius_mm)/input_span
        if hashlib.sha256((root / relative).read_bytes()).hexdigest() != actual_hash:
            raise ValueError('Native CAD mass properties changed during gravity screening')
        source_sha256 = hashlib.sha256((self.source_sha256+actual_hash).encode()).hexdigest()
        result = planar_gravity_torque_bound(cad_masses, com_speeds,
            gravity_m_s2, plane_normal, source_sha256=source_sha256)
        result['native_mass_properties_sha256'] = actual_hash
        result['planar_source_sha256'] = self.source_sha256
        result['implementation_sha256'] = self.implementation_sha256
        self._assert_planar_source_unchanged()
        return result


_BASE_PLANAR_METHODS = dict(angle_at=PlanarNativePath.angle_at,
    pose_at=PlanarNativePath.pose_at,
    point_speed_bound=PlanarNativePath.point_speed_bound)
