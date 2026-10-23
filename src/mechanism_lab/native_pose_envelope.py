"""Continuous CAD clearance for an explicitly specified rigid-pose playback.

This service measures native B-reps, not collision proxies. ``hold`` covers
every state of :class:`physical_replay.RecordedMotion` (the MP4 capture path).
``shortest_arc_slerp`` covers linear translation and shortest-arc quaternion
slerp between adjacent recorded poses. The latter is an explicit interpolation
contract, not a claim about every glTF viewer or MuJoCo's unsampled dynamics.

The analytic adapter accepts a source-derived pose function and a conservative
per-part point-speed bound. Its result is conditional on that bound; the source
of the bound is named and hashed by the caller, never inferred from samples.
"""
from __future__ import annotations

from collections import Counter, OrderedDict
from itertools import combinations
import hashlib
import json
import math
from pathlib import Path
import re
import time

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from .core import pose_cad
from .motion_envelope import _box, _box_gap, _brep_fingerprint, _corners, _distance
SCHEMA = 'cybrgeo-native-pose-envelope/1'


def _implementation_identity():
    """Bind native queries, pose placement and this interval algorithm."""
    directory = Path(__file__).resolve().parent
    sources = ('native_pose_envelope.py', 'motion_envelope.py', 'core.py')
    return {name: hashlib.sha256((directory / name).read_bytes()).hexdigest()
            for name in sources}


def _native_shape(value):
    """Keep every imported STEP object; ``Workplane.val()`` drops siblings."""
    if hasattr(value, 'vals'):
        import cadquery as cq
        values = value.vals()
        if not values:
            raise ValueError('Native CAD workplane contains no shapes')
        return values[0] if len(values) == 1 else cq.Compound.makeCompound(values)
    return value


def group_native_parts_by_body(parts):
    """Make one conservative native compound per rigid body.

    Cross-body compound clearance is the minimum over every member pair, so
    grouping cannot hide a collision. A declared excluded *body* pair omits
    every member pairing between those bodies; mating/fit checks for those
    members need independent evidence. The returned manifest retains names,
    member B-rep identities and exact potential pair counts for that audit.
    """
    import cadquery as cq

    if not isinstance(parts, dict) or not parts:
        raise ValueError('Named native parts are required for body grouping')
    bodies = {}
    for name, row in parts.items():
        if (not isinstance(name, str) or not name or not isinstance(row, dict)
                or set(row) != {'body', 'shape'} or not isinstance(row['body'], str)
                or not row['body']):
            raise ValueError('Each grouped native part needs a name, body and CAD shape')
        shape = _native_shape(row['shape'])
        if not shape.isValid() or not shape.Solids() or not math.isfinite(shape.Volume()) or shape.Volume() <= 0:
            raise ValueError(f'{name} needs a valid positive-volume native solid')
        bodies.setdefault(row['body'], []).append((name, shape))
    if len(bodies) < 2:
        raise ValueError('Body-level clearance requires at least two distinct rigid bodies')
    grouped, members = {}, {}
    for body, entries in sorted(bodies.items()):
        # Each member may itself be a compound; OpenCascade's compound minimum
        # distance still examines all contained solids in the other body.
        grouped[body] = dict(body=body, shape=(entries[0][1] if len(entries) == 1
                           else cq.Compound.makeCompound([shape for _, shape in entries])))
        members[body] = [dict(part=name, native_brep_sha256=_brep_fingerprint(shape))
                         for name, shape in sorted(entries)]
    potential = [dict(bodies=[a, b], member_pairs=len(members[a])*len(members[b]))
                 for a, b in combinations(sorted(members), 2)]
    identity = hashlib.sha256(json.dumps(dict(members=members, potential_pairs=potential),
        sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    return grouped, dict(schema='cybrgeo-body-compound-members/1', bodies=members,
                         potential_body_pairs=potential,
                         cross_body_member_pairs=sum(row['member_pairs'] for row in potential),
                         member_manifest_sha256=identity,
                         exclusion_scope='Excluding one body pair omits all of its member pairs; '
                                         'individual mating fits require independent checks.')


def _positive(value, label, *, zero=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value < 0 or (not zero and value == 0)):
        raise ValueError(f'{label} must be finite and {"nonnegative" if zero else "positive"}')
    return float(value)


def _matrix(value, label):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], rtol=0, atol=1e-10)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), rtol=0, atol=1e-8)
            or not math.isclose(float(np.linalg.det(matrix[:3, :3])), 1., rel_tol=0, abs_tol=1e-8)):
        raise ValueError(f'{label} must be a finite proper 4x4 rigid pose')
    return matrix


def _pair(pair, names, label):
    if (not isinstance(pair, (tuple, list)) or len(pair) != 2 or pair[0] == pair[1]
            or any(not isinstance(name, str) or name not in names for name in pair)):
        raise ValueError(f'{label} must name two distinct native parts')
    return tuple(sorted(pair))


def _prepare(parts, *, clearance_pairs, excluded_pairs, required_clearance_mm,
             distance_margin_mm, max_depth, max_pair_intervals, max_total_intervals):
    if not isinstance(parts, dict) or len(parts) < 2:
        raise ValueError('At least two named native parts are required')
    clean, geometry = {}, {}
    for name, row in parts.items():
        if (not isinstance(name, str) or not name or not isinstance(row, dict)
                or set(row) != {'body', 'shape'} or not isinstance(row['body'], str)
                or not row['body']):
            raise ValueError('Each part needs a unique name, body and native CAD shape')
        shape = _native_shape(row['shape'])
        if not shape.isValid() or not shape.Solids() or not math.isfinite(shape.Volume()) or shape.Volume() <= 0:
            raise ValueError(f'{name} needs a valid positive-volume native solid')
        box = _box(shape)
        clean[name] = dict(body=row['body'], shape=shape, corners=_corners(box),
                           radius_mm=float(np.max(np.linalg.norm(_corners(box), axis=1))))
        geometry[name] = _brep_fingerprint(shape)
    all_cross = {_pair(p, clean, 'Part pair') for p in combinations(clean, 2)
                 if clean[p[0]]['body'] != clean[p[1]]['body']}
    selected = all_cross if clearance_pairs is None else [_pair(p, clean, 'Clearance pair') for p in clearance_pairs]
    selected = set(selected)
    if selected - all_cross:
        raise ValueError('Clearance pairs must belong to different rigid bodies')
    excluded_pairs = excluded_pairs or {}
    if not isinstance(excluded_pairs, dict):
        raise ValueError('Excluded pairs require a reason keyed by part pair')
    excluded = {}
    for pair, reason in excluded_pairs.items():
        key = _pair(pair, clean, 'Excluded pair')
        if key not in all_cross or not isinstance(reason, str) or not reason.strip() or key in excluded:
            raise ValueError('Excluded cross-body pair needs one nonempty contact/mating reason')
        excluded[key] = reason
    if clearance_pairs is None:
        selected -= excluded.keys()
    elif selected & excluded.keys():
        raise ValueError('A pair cannot be both clearance-certified and excluded')
    if not selected:
        raise ValueError('No noncontact moving pair remains to certify')
    if (not isinstance(max_depth, int) or isinstance(max_depth, bool) or not 0 <= max_depth <= 30
            or not isinstance(max_pair_intervals, int) or isinstance(max_pair_intervals, bool)
            or max_pair_intervals < 1 or not isinstance(max_total_intervals, int)
            or isinstance(max_total_intervals, bool) or max_total_intervals < 1):
        raise ValueError('Invalid interval budgets')
    _positive(distance_margin_mm, 'CAD distance margin')
    if isinstance(required_clearance_mm, dict):
        requirements = {_pair(p, clean, 'Required-clearance pair'): _positive(v, 'Required clearance', zero=True)
                        for p, v in required_clearance_mm.items()}
        if set(requirements) != selected:
            raise ValueError('Per-pair required clearances must cover every selected pair exactly')
    else:
        required = _positive(required_clearance_mm, 'Required clearance', zero=True)
        requirements = {pair: required for pair in selected}
    return clean, geometry, sorted(selected), excluded, requirements, len(all_cross)


def _recorded_identity(geometry, times, poses, metadata):
    h = hashlib.sha256(json.dumps(dict(geometry=geometry, metadata=metadata),
        sort_keys=True, separators=(',', ':'), allow_nan=False).encode())
    h.update(np.asarray(times, dtype='<f8').tobytes())
    for body in sorted(poses):
        h.update(body.encode() + b'\0')
        h.update(np.asarray(poses[body], dtype='<f8').tobytes())
    return h.hexdigest()


def _check_segment(clean, pairs, requirements, pose_at, speed_bound,
                   segment_index, max_depth, max_pair_intervals, max_total_intervals,
                   margin, statistics, reports, failures):
    poses = OrderedDict()
    boxes = OrderedDict()
    shapes = OrderedDict()

    def state(u):
        if u not in poses:
            values = pose_at(u)
            if not isinstance(values, dict) or set(values) != {row['body'] for row in clean.values()}:
                raise ValueError('Pose callback must return every bound rigid body exactly')
            poses[u] = {body: _matrix(matrix, body) for body, matrix in values.items()}
            if len(poses) > 128:
                poses.popitem(last=False)
        return poses[u]

    def box(name, u):
        key = (name, u)
        if key not in boxes:
            transform = state(u)[clean[name]['body']]
            points = clean[name]['corners'] @ transform[:3, :3].T + transform[:3, 3]
            # Enclose floating point matrix products as well as the source
            # CAD bounding box; a narrow gap must go to the native B-rep query.
            outward = 64 * np.finfo(float).eps * (1 + np.max(np.abs(points)))
            boxes[key] = np.array([points.min(axis=0)-outward,
                                   points.max(axis=0)+outward])
            if len(boxes) > 512:
                boxes.popitem(last=False)
        return boxes[key]

    def shape(name, u):
        key = (name, u)
        if key not in shapes:
            shapes[key] = pose_cad(clean[name]['shape'], state(u)[clean[name]['body']])
            if len(shapes) > 128:
                shapes.popitem(last=False)
        return shapes[key]

    # Validate both endpoints even when broad-phase bounds skip exact CAD.
    state(0.); state(1.)
    for a, b in pairs:
        visited = 0
        pending = [(0., 1., 0)]
        row = reports[(a, b)]
        while pending:
            lo, hi, depth = pending.pop()
            visited += 1
            statistics['visited_intervals'] += 1
            if visited > max_pair_intervals or statistics['visited_intervals'] > max_total_intervals:
                failures.append(dict(segment=segment_index, parts=[a, b], interval=[lo, hi],
                                     reason='Interval budget exhausted; continuous clearance remains unresolved'))
                return
            mid = (lo + hi) / 2.
            speeds = (speed_bound(a, lo, hi), speed_bound(b, lo, hi))
            speeds = [_positive(value, 'Rigid-point speed bound', zero=True) for value in speeds]
            travel = (hi - lo) / 2. * sum(speeds)
            # Round outward for floating arithmetic; not a production tolerance.
            travel = travel * (1 + 1e-12) + 1e-12
            threshold = requirements[(a, b)] + margin + travel
            lower = _box_gap(box(a, mid), box(b, mid))
            method = 'aabb_bound'
            if lower <= threshold:
                lower = _distance(shape(a, mid), shape(b, mid))
                statistics['brep_distance_queries'] += 1
                method = 'brep_distance_bound'
            if lower > threshold:
                row['certified_intervals'] += 1
                row['minimum_clearance_lower_bound_mm'] = min(
                    row['minimum_clearance_lower_bound_mm'], lower-travel-margin)
                row['deepest_subdivision'] = max(row['deepest_subdivision'], depth)
                statistics[method] += 1
                continue
            if depth >= max_depth:
                failures.append(dict(segment=segment_index, parts=[a, b], interval=[lo, hi],
                    measured_midpoint_distance_mm=lower,
                    required_clearance_mm=requirements[(a, b)],
                    reason='Unresolved or colliding native CAD interval'))
                return
            pending.extend(((mid, hi, depth+1), (lo, mid, depth+1)))
        row['segments_checked'] += 1


def _certify(clean, geometry, pairs, excluded, requirements, cross_count, segments,
             *, interpolation, input_sha256, distance_margin_mm, max_depth,
             max_pair_intervals, max_total_intervals, provenance, limitations,
             implementation_sha256):
    started = time.monotonic()
    stats = Counter()
    reports = {pair: dict(parts=list(pair), required_clearance_mm=requirements[pair],
                          segments_checked=0, certified_intervals=0,
                          deepest_subdivision=0, minimum_clearance_lower_bound_mm=math.inf)
               for pair in pairs}
    failures = []
    for index, (pose_at, speed_bound) in enumerate(segments):
        _check_segment(clean, pairs, requirements, pose_at, speed_bound, index,
                       max_depth, max_pair_intervals, max_total_intervals,
                       distance_margin_mm, stats, reports, failures)
        if failures:
            break
    rows = list(reports.values())
    for row in rows:
        if not math.isfinite(row['minimum_clearance_lower_bound_mm']):
            row['minimum_clearance_lower_bound_mm'] = None
    if _implementation_identity() != implementation_sha256:
        raise RuntimeError('Native clearance implementation changed during certification')
    return dict(schema=SCHEMA, passed=not failures, interpolation=interpolation,
                input_sha256=input_sha256, native_brep_sha256=geometry,
                implementation_sha256=implementation_sha256,
                body_local_point_radius_mm={name: clean[name]['radius_mm'] for name in sorted(clean)},
                source=provenance, segments_requested=len(segments),
                segments_completed=len(segments) if not failures else failures[0]['segment'],
                cross_body_pairs=cross_count, clearance_pairs=rows,
                excluded_pairs=[dict(parts=list(pair), reason=reason) for pair, reason in sorted(excluded.items())],
                unexamined_cross_body_pairs=cross_count-len(pairs)-len(excluded),
                complete_pair_accounting=(cross_count == len(pairs)+len(excluded)),
                failures=failures, statistics=dict(stats),
                distance_margin_mm=distance_margin_mm,
                runtime_seconds=round(time.monotonic()-started, 3),
                scope=('All times of the explicit rigid-pose interpolation for the listed noncontact pairs; '
                       'excluded mating/contact pairs are not certified clear.'),
                limitations=limitations)


def certify_recorded_path(parts, times_s, body_poses_mm, *, interpolation='hold',
                          clearance_pairs=None, excluded_pairs=None,
                          required_clearance_mm=0., distance_margin_mm=1e-4,
                          max_depth=12, max_pair_intervals=4095,
                          max_total_intervals=1000000, source_identity=None):
    """Certify a held recording or explicit shortest-arc-slerp playback.

    ``parts`` maps names to ``{'body': name, 'shape': native_cad_shape}``.
    ``body_poses_mm`` maps every bound body to one rigid 4x4 matrix per time.
    Exclusions require explicit pair/reason entries. Passing a selected subset
    records the remaining pair count and never claims full assembly coverage.
    """
    if interpolation not in ('hold', 'shortest_arc_slerp'):
        raise ValueError('Only held states or explicit shortest-arc slerp are supported')
    implementation = _implementation_identity()
    clean, geometry, pairs, excluded, requirements, cross_count = _prepare(
        parts, clearance_pairs=clearance_pairs, excluded_pairs=excluded_pairs,
        required_clearance_mm=required_clearance_mm,
        distance_margin_mm=distance_margin_mm, max_depth=max_depth,
        max_pair_intervals=max_pair_intervals, max_total_intervals=max_total_intervals)
    times = np.asarray(times_s, dtype=float)
    if (times.ndim != 1 or len(times) < 2 or not np.isfinite(times).all()
            or times[0] < 0 or np.any(np.diff(times) <= 0)):
        raise ValueError('Recorded times must increase strictly from a nonnegative origin')
    bodies = {row['body'] for row in clean.values()}
    if not isinstance(body_poses_mm, dict) or set(body_poses_mm) != bodies:
        raise ValueError('Recorded poses must cover every bound body exactly')
    poses = {}
    for body, rows in body_poses_mm.items():
        value = np.asarray(rows, dtype=float)
        if value.shape != (len(times), 4, 4):
            raise ValueError(f'{body} needs one rigid pose per recorded time')
        poses[body] = np.asarray([_matrix(row, body) for row in value])
    metadata = dict(interpolation=interpolation,
                    implementation_sha256=implementation,
                    part_bodies={name: clean[name]['body'] for name in sorted(clean)},
                    clearance_pairs=[list(pair) for pair in pairs],
                    excluded_pairs=[dict(parts=list(pair), reason=reason) for pair, reason in sorted(excluded.items())],
                    required_clearance_mm={','.join(pair): requirements[pair] for pair in pairs},
                    margin_mm=distance_margin_mm, source_identity=source_identity)
    identity = _recorded_identity(geometry, times, poses, metadata)
    segments = []
    if interpolation == 'hold':
        for index in range(len(times)):
            frame = {body: rows[index] for body, rows in poses.items()}
            segments.append((lambda _u, frame=frame: frame,
                             lambda _name, _lo, _hi: 0.))
        depth = 0
        limitations = ['Every held recorded state is checked. MuJoCo trajectories between integration steps are not bounded.',
                       'Excluded contact/mating pairs, manufacturing tolerances, deformation and hardware are not certified.']
    else:
        for index in range(len(times)-1):
            starts = {body: rows[index] for body, rows in poses.items()}
            ends = {body: rows[index+1] for body, rows in poses.items()}
            slerps = {body: Slerp([0., 1.], Rotation.from_matrix([starts[body][:3, :3], ends[body][:3, :3]]))
                      for body in bodies}
            speeds = {}
            for name, part in clean.items():
                body = part['body']
                translation = float(np.linalg.norm(ends[body][:3, 3] - starts[body][:3, 3]))
                angle = float((Rotation.from_matrix(starts[body][:3, :3]).inv()
                               * Rotation.from_matrix(ends[body][:3, :3])).magnitude())
                speeds[name] = (translation + angle * part['radius_mm']) * (1+1e-12) + 1e-12
            def pose_at(u, starts=starts, ends=ends, slerps=slerps):
                result = {}
                for body in bodies:
                    matrix = np.eye(4)
                    matrix[:3, :3] = slerps[body]([float(u)]).as_matrix()[0]
                    matrix[:3, 3] = starts[body][:3, 3] + u * (ends[body][:3, 3]-starts[body][:3, 3])
                    result[body] = matrix
                return result
            segments.append((pose_at, lambda name, _lo, _hi, speeds=speeds: speeds[name]))
        depth = max_depth
        limitations = ['Only explicitly shortest-arc-slerped rotations and linearly interpolated translations are covered.',
                       'glTF 2.0 recommends but does not require slerp for LINEAR rotation; arbitrary viewers and MuJoCo dynamics are not covered.',
                       'Excluded contact/mating pairs, manufacturing tolerances, deformation and hardware are not certified.']
    report = _certify(clean, geometry, pairs, excluded, requirements, cross_count,
                      segments, interpolation=interpolation, input_sha256=identity,
                      distance_margin_mm=distance_margin_mm, max_depth=depth,
                      max_pair_intervals=max_pair_intervals,
                      max_total_intervals=max_total_intervals,
                      provenance=source_identity, limitations=limitations,
                      implementation_sha256=implementation)
    report['recorded_frame_count'] = len(times)
    report['recorded_time_span_s'] = [float(times[0]), float(times[-1])]
    return report


def certify_parametric_path(parts, pose_at, point_speed_bound, *, source_identity,
                            clearance_pairs=None, excluded_pairs=None,
                            required_clearance_mm=0., distance_margin_mm=1e-4,
                            max_depth=12, max_pair_intervals=4095,
                            max_total_intervals=1000000):
    """Certify one analytic [0,1] segment conditional on source-derived bounds.

    ``point_speed_bound(part_name, lo, hi)`` must bound every point's travel in
    millimetres per unit parameter throughout the interval. A sampled maximum
    is insufficient. The certificate names the supplied source identity but
    cannot prove the callback's mathematical bound on its own.
    """
    if not isinstance(source_identity, str) or not source_identity.strip():
        raise ValueError('An analytic path needs a source/bound identity')
    if not callable(pose_at) or not callable(point_speed_bound):
        raise ValueError('Analytic path requires pose and conservative speed callbacks')
    implementation = _implementation_identity()
    clean, geometry, pairs, excluded, requirements, cross_count = _prepare(
        parts, clearance_pairs=clearance_pairs, excluded_pairs=excluded_pairs,
        required_clearance_mm=required_clearance_mm,
        distance_margin_mm=distance_margin_mm, max_depth=max_depth,
        max_pair_intervals=max_pair_intervals, max_total_intervals=max_total_intervals)
    identity = hashlib.sha256(json.dumps(dict(geometry=geometry,
        implementation_sha256=implementation,
        part_bodies={name: clean[name]['body'] for name in sorted(clean)},
        source_identity=source_identity, pairs=pairs,
        excluded=[(pair, reason) for pair, reason in sorted(excluded.items())],
        requirements=[(pair, value) for pair, value in sorted(requirements.items())],
        margin_mm=distance_margin_mm), sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    return _certify(clean, geometry, pairs, excluded, requirements, cross_count,
                    [(pose_at, point_speed_bound)], interpolation='source_bounded_parametric_path',
                    input_sha256=identity, distance_margin_mm=distance_margin_mm,
                    max_depth=max_depth, max_pair_intervals=max_pair_intervals,
                    max_total_intervals=max_total_intervals, provenance=source_identity,
                    implementation_sha256=implementation,
                    limitations=['The analytic source must prove the supplied point-speed bound; samples alone are insufficient.',
                                 'This is geometric clearance for its parameterized rigid path, not integrated dynamics.',
                                 'Excluded contact/mating pairs, manufacturing tolerances, deformation and hardware are not certified.'])


def load_placed_native_parts(native_directory, *, expected_file_hashes=None,
                             selected_names=None):
    """Import actual body-local placed STEP solids from a native-build folder.

    ``expected_file_hashes`` may be the ``files_sha256`` map from a separately
    verified native-build receipt. Without it, file identities are measured
    and returned but their construction lineage is not independently verified.
    The body pivot is the local origin used by the native placement export.
    A selected subset still hashes the complete placement manifest and names
    every omitted part. It never confers whole-assembly pair coverage.
    """
    import cadquery as cq

    root = Path(native_directory).resolve()
    manifest = root / 'placed-native-parts.json'
    payload = manifest.read_bytes()
    document = json.loads(payload.decode('utf-8'))
    if (not isinstance(document, dict) or document.get('units') != 'mm'
            or not isinstance(document.get('records'), list) or not document['records']):
        raise ValueError('Native placement manifest needs nonempty mm records')
    if expected_file_hashes is not None and not isinstance(expected_file_hashes, dict):
        raise ValueError('Expected native file hashes must be a receipt mapping')
    if selected_names is not None:
        if (not isinstance(selected_names, (list, tuple, set, frozenset))
                or not selected_names or any(not isinstance(name, str) or not name
                                           for name in selected_names)
                or len(set(selected_names)) != len(selected_names)):
            raise ValueError('Selected native part names must be a nonempty unique inventory')
        selected_names = set(selected_names)
    files = {'placed-native-parts.json': hashlib.sha256(payload).hexdigest()}
    if (expected_file_hashes is not None
            and expected_file_hashes.get('placed-native-parts.json') != files['placed-native-parts.json']):
        raise ValueError('Native placement manifest differs from the verified receipt')
    parts = {}
    radii = {}
    complete_names = set()
    complete_paths = set()
    for row in document['records']:
        if not isinstance(row, dict):
            raise ValueError('Native placement record must be an object')
        body, label, relative = (row.get('body'), row.get('name'), row.get('step_file'))
        if (any(not isinstance(item, str) or not item or item in ('.', '..')
                or not re.fullmatch(r'[A-Za-z0-9_. -]+', item)
                for item in (body, label))
                or not isinstance(relative, str) or not relative.endswith('.step')
                or Path(relative).is_absolute()):
            raise ValueError('Native placement needs safe body, name and STEP path')
        name = f'{body}__{label}'
        if name in complete_names:
            raise ValueError(f'Duplicate native placement identity: {name}')
        complete_names.add(name)
        path = (root / relative).resolve()
        if not path.is_relative_to(root):
            raise ValueError(f'Native STEP path escapes the build: {relative}')
        canonical = path.relative_to(root).as_posix()
        if canonical != relative.replace('\\', '/') or canonical in complete_paths:
            raise ValueError(f'Native STEP path is noncanonical or repeated: {relative}')
        complete_paths.add(canonical)
        if selected_names is not None and name not in selected_names:
            continue
        file_hash = hashlib.sha256(path.read_bytes()).hexdigest()
        files[canonical] = file_hash
        if expected_file_hashes is not None and expected_file_hashes.get(canonical) != file_hash:
            raise ValueError(f'Native STEP differs from the verified receipt: {relative}')
        shape = _native_shape(cq.importers.importStep(str(path)))
        if not shape.isValid() or not shape.Solids() or not math.isfinite(shape.Volume()) or shape.Volume() <= 0:
            raise ValueError(f'Native STEP is not a valid positive-volume solid: {relative}')
        if hashlib.sha256(path.read_bytes()).hexdigest() != file_hash:
            raise ValueError(f'Native STEP changed during import: {relative}')
        parts[name] = dict(body=body, shape=shape)
        radii[name] = float(np.max(np.linalg.norm(_corners(_box(shape)), axis=1)))
    if selected_names is not None and selected_names != set(parts):
        raise ValueError('Selected native part is missing from the placement manifest')
    if hashlib.sha256(manifest.read_bytes()).hexdigest() != files['placed-native-parts.json']:
        raise ValueError('Native placement manifest changed during import')
    identity = hashlib.sha256(json.dumps(dict(files=files,
        part_bodies={name: parts[name]['body'] for name in sorted(parts)}),
        sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    return parts, dict(native_files_sha256=files, native_file_set_sha256=identity,
                       body_local_point_radius_mm=radii,
                       native_inventory_count=len(complete_names),
                       selected_native_part_names=sorted(parts),
                       omitted_native_part_names=sorted(complete_names-set(parts)),
                       complete_native_inventory_selected=len(parts) == len(complete_names),
                       externally_verified=expected_file_hashes is not None)


def certify_native_parametric_path(native_directory, pose_at, point_speed_bound,
                                   *, source_identity, expected_file_hashes=None,
                                   selected_names=None, **kwargs):
    """Import placed native STEP and check an analytically bounded rigid path."""
    if not isinstance(source_identity, str) or not source_identity.strip():
        raise ValueError('An analytic path needs a source/bound identity')
    parts, imported = load_placed_native_parts(native_directory,
        expected_file_hashes=expected_file_hashes, selected_names=selected_names)
    report = certify_parametric_path(parts, pose_at, point_speed_bound,
        source_identity=f'{source_identity}|native-files:{imported["native_file_set_sha256"]}',
        **kwargs)
    report['native_step_import'] = imported
    # complete_pair_accounting in the generic interval report describes only
    # imported parts. Never allow a selected subset to masquerade as full
    # native assembly coverage when a caller reads the top-level result.
    report['full_native_inventory_selected'] = imported['complete_native_inventory_selected']
    report['full_native_noncontact_pair_coverage'] = bool(
        report['passed'] and imported['complete_native_inventory_selected']
        and report['unexamined_cross_body_pairs'] == 0
        and not report['excluded_pairs'])
    return report


def verify_implementation_identity(report):
    """Reject a report made with different clearance/placement source code."""
    if not isinstance(report, dict) or report.get('schema') != SCHEMA:
        raise ValueError('Expected a native pose-envelope certificate')
    if report.get('implementation_sha256') != _implementation_identity():
        raise ValueError('Native clearance implementation changed after certification')
    return True


def verify_recorded_identity(report, parts, times_s, body_poses_mm, *, source_identity=None):
    """Reject a stale recording/shape against its certificate input identity."""
    if report.get('schema') != SCHEMA or report.get('passed') is not True:
        raise ValueError('A passing native pose certificate is required')
    verify_implementation_identity(report)
    implementation = report['implementation_sha256']
    clean, geometry, pairs, excluded, requirements, _ = _prepare(parts,
        clearance_pairs=[row['parts'] for row in report['clearance_pairs']],
        excluded_pairs={tuple(row['parts']):row['reason'] for row in report['excluded_pairs']},
        required_clearance_mm={tuple(row['parts']):row['required_clearance_mm'] for row in report['clearance_pairs']},
        distance_margin_mm=report['distance_margin_mm'], max_depth=0,
        max_pair_intervals=1, max_total_intervals=1)
    times = np.asarray(times_s, dtype=float)
    poses = {body: np.asarray(rows, dtype=float) for body, rows in body_poses_mm.items()}
    if (times.ndim != 1 or len(times) < 2 or not np.isfinite(times).all()
            or times[0] < 0 or np.any(np.diff(times) <= 0)
            or any(rows.shape != (len(times),4,4) for rows in poses.values())
            or set(poses) != {row['body'] for row in clean.values()}):
        raise ValueError('Recorded identity dimensions differ')
    for body, rows in poses.items():
        for row in rows:
            _matrix(row, body)
    metadata = dict(interpolation=report['interpolation'],
                    implementation_sha256=implementation,
                    part_bodies={name: clean[name]['body'] for name in sorted(clean)},
                    clearance_pairs=[list(pair) for pair in pairs],
                    excluded_pairs=[dict(parts=list(pair), reason=reason) for pair, reason in sorted(excluded.items())],
                    required_clearance_mm={','.join(pair): requirements[pair] for pair in pairs},
                    margin_mm=report['distance_margin_mm'], source_identity=source_identity)
    if _recorded_identity(geometry, times, poses, metadata) != report['input_sha256']:
        raise ValueError('Native CAD, recorded poses or source identity changed after certification')
    return True
