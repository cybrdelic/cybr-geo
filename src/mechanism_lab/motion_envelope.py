"""Conservative continuous clearance for a commanded joint-space polyline.

This is a geometric certificate over a *path*, not the Cartesian product of
joint limits. At an interval midpoint, CAD distance minus a rigid-point motion
bound lower-bounds clearance everywhere in that interval. Unresolved intervals
are rejected. Exact shared ancestors cancel; axisymmetry is established from
the constructive primitives, never from part names or a parent-pair exemption.

The interval inequality is conservative in real arithmetic. OpenCascade B-rep
queries remain numerical; the explicit distance margin is not a manufacturing
tolerance, contact simulation, or a formal verified-arithmetic kernel proof.
"""
from __future__ import annotations

from collections import Counter, OrderedDict
from itertools import combinations, product
import hashlib
from io import BytesIO
import json
import math
import time

import numpy as np

from .constructed import KinematicGraph
from .core import pose_cad


SCHEMA = 'cybrgeo-continuous-path-clearance/1'


def plan_identity(plan):
    """Bind the whole input, including geometry, joints and mechanical specs."""
    return hashlib.sha256(json.dumps(plan, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def _box(shape):
    b = shape.BoundingBox()
    return np.array([[b.xmin, b.ymin, b.zmin], [b.xmax, b.ymax, b.zmax]])


def _corners(box):
    return np.array(list(product(*zip(box[0], box[1]))))


def _box_gap(a, b):
    return float(np.linalg.norm(np.maximum(0., np.maximum(a[0] - b[1], b[0] - a[1]))))


def _distance(a, b):
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    q = BRepExtrema_DistShapeShape(a.wrapped, b.wrapped)
    q.Perform()
    if not q.IsDone():
        raise RuntimeError('OpenCascade minimum-distance query failed')
    result = float(q.Value())
    if not math.isfinite(result) or result < 0:
        raise RuntimeError('OpenCascade returned an invalid distance')
    return result


def _moving_chain(graph, part):
    chain = []
    while part in graph.by_child:
        joint = graph.by_child[part]
        if joint['type'] != 'fixed':
            chain.append(joint['name'])
        part = joint['parent']
    return tuple(reversed(chain))


def _residual_chains(a, b):
    shared = 0
    while shared < min(len(a), len(b)) and a[shared] == b[shared]:
        shared += 1
    return a[shared:], b[shared:]


def point_motion_bounds(corners, chain, graph, start, end, errors=None):
    """Return (speed mm / segment parameter, tracking-error travel mm).

    For a joint j, the downstream product can displace a rest point by D.
    Its distance to j's axis is therefore <= rest axis distance + D. Reverse
    induction bounds D with min(2, |angle|)*radius (or prismatic travel).
    Summing joint angular speeds times those radii bounds the product's speed.
    Both endpoint maps and errors must contain resolved followers, not only
    controls. Inflating downstream joint ranges by their errors also bounds
    the intermediate products in the nominal-to-actual telescoping difference.
    """
    errors = errors or {}
    downstream = speed = error_displacement = 0.
    for name in reversed(chain):
        joint = graph.by_name[name]
        lo, hi = start[name], end[name]
        error = errors.get(name, 0.)
        if joint['type'] == 'prismatic':
            speed += abs(hi - lo)
            error_displacement += error
            downstream += max(abs(lo), abs(hi)) + error
        elif joint['type'] == 'revolute':
            offset = corners - np.asarray(joint['origin'])
            axis = np.asarray(joint['axis'])
            perpendicular = offset - np.outer(offset @ axis, axis)
            radius = float(np.max(np.linalg.norm(perpendicular, axis=1))) + downstream
            speed += math.radians(abs(hi - lo)) * radius
            error_displacement += math.radians(error) * radius
            downstream += min(2., math.radians(max(abs(lo), abs(hi)) + error)) * radius
    # Outward allowance for arithmetic in the bound, not a geometry tolerance.
    outward = lambda value: value * (1 + 1e-12) + (1e-12 if value else 0.)
    return outward(speed), outward(error_displacement)


def point_speed_bound(corners, chain, graph, start, end):
    """Compatibility helper for the zero-error continuous-path bound."""
    return point_motion_bounds(corners, chain, graph, start, end)[0]


def _error_bounds(graph, bounds, drive_bounds=None):
    bounds = {} if bounds is None else bounds
    if not isinstance(bounds, dict) or set(bounds) - set(graph.controls):
        raise ValueError('Joint-error bounds must name independent controls only')
    controls = {name: bounds.get(name, 0.) for name in graph.controls}
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v < 0 for v in controls.values()):
        raise ValueError('Joint-error bounds must be finite and nonnegative')
    drive_bounds = {} if drive_bounds is None else drive_bounds
    followers = {n for n, j in graph.by_name.items() if j.get('drive')}
    if not isinstance(drive_bounds, dict) or set(drive_bounds) - followers:
        raise ValueError('Drive-error bounds must name driven joints only')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v < 0 for v in drive_bounds.values()):
        raise ValueError('Drive-error bounds must be finite and nonnegative')
    resolved = {}
    for name in graph.drive_order:
        joint = graph.by_name[name]
        drive = joint.get('drive')
        resolved[name] = (abs(drive['ratio']) * resolved[drive['joint']] + drive_bounds.get(name, 0.)) if drive else controls.get(name, 0.)
    return controls, resolved


def _brep_fingerprint(shape):
    from OCP.BRepTools import BRepTools
    from OCP.TopTools import TopTools_FormatVersion
    from OCP.TopoDS import TopoDS_Iterator
    # Meshing/validity queries update TopoDS's transient Checked cache flag.
    # Normalize that flag on an independent copy; keep actual geometric and
    # topological data intact and never mutate the measured solid.
    canonical = shape.copy(mesh=False)
    def clear_checked(node):
        node.Checked(False)
        iterator = TopoDS_Iterator(node)
        while iterator.More():
            clear_checked(iterator.Value())
            iterator.Next()
    clear_checked(canonical.wrapped)
    stream = BytesIO()
    BRepTools.Write_s(canonical.wrapped, stream, False, False, TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3)
    return hashlib.sha256(stream.getvalue()).hexdigest()


def _verified_primitive_source(shape, spec):
    """A stale or edited actual B-rep cannot inherit recipe symmetry claims."""
    from .constructed import primitive
    reconstructed = primitive(spec['body'])
    for operand in spec['unions']:
        reconstructed = reconstructed.fuse(primitive(operand))
    for operand in spec['cuts']:
        reconstructed = reconstructed.cut(primitive(operand))
    if spec.get('fillet', 0):
        import cadquery as cq
        reconstructed = cq.Workplane(obj=reconstructed).edges().fillet(spec['fillet']).val()
    original, rebuilt = _brep_fingerprint(shape), _brep_fingerprint(reconstructed)
    return original if original == rebuilt else None


def _cardinal(vector):
    """Recognize exact cardinal datums only; no nearly-coaxial exemption."""
    vector = tuple(float(v) for v in vector)
    return vector if sum(v != 0 for v in vector) == 1 and max(map(abs, vector)) == 1 else None


def _primitive_axis(spec):
    # Exact multiples of ninety degrees have exact integer rotation matrices
    # in the authored model. Avoid accepting a near-coaxial numerical fit.
    turns = []
    for angle in spec['rotation']:
        if not float(angle / 90).is_integer():
            return None
        turns.append(int(angle / 90) % 4)
    axis = np.array([0, 0, 1], dtype=int)
    for component, count in enumerate(turns):
        for _ in range(count):
            x, y, z = axis
            axis = np.array(([x, -z, y], [z, y, -x], [-y, x, z])[component])
    return tuple(int(v) for v in axis)


def _axisymmetric(spec, origin, axis):
    """Boolean combinations of coaxial cylinders/spheres remain invariant."""
    cardinal = _cardinal(axis)
    if cardinal is None or spec.get('fillet', 0):
        return False
    along = next(i for i, value in enumerate(cardinal) if value)
    for primitive in [spec['body'], *spec['unions'], *spec['cuts']]:
        if any(primitive['position'][i] != origin[i] for i in range(3) if i != along):
            return False
        if primitive['kind'] == 'sphere':
            continue
        if primitive['kind'] != 'cylinder':
            return False
        direction = _primitive_axis(primitive)
        if direction is None or any(direction[i] != 0 for i in range(3) if i != along):
            return False
    return True


def _coaxial_invariance(a, b, chain_a, chain_b, specs, graph):
    """Only cancel a residual rotation against an actually axisymmetric solid."""
    for stationary, stationary_chain, moving_chain in ((a, chain_a, chain_b), (b, chain_b, chain_a)):
        if stationary_chain or not moving_chain:
            continue
        first = graph.by_name[moving_chain[0]]
        axis = _cardinal(first['axis'])
        if first['type'] != 'revolute' or axis is None:
            continue
        along = next(i for i, value in enumerate(axis) if value)
        coaxial = all(graph.by_name[n]['type'] == 'revolute'
                      and _cardinal(graph.by_name[n]['axis']) in (axis, tuple(-v for v in axis))
                      and all(graph.by_name[n]['origin'][i] == first['origin'][i] for i in range(3) if i != along)
                      for n in moving_chain)
        if coaxial and _axisymmetric(specs[stationary], first['origin'], axis):
            return {'method': 'constructive_coaxial_rotational_invariance',
                    'axisymmetric_part': stationary, 'rotating_joints': list(moving_chain),
                    'origin_mm': first['origin'], 'axis': list(axis)}
    return None


def _path(graph, waypoints):
    if waypoints is None:
        waypoints = graph.motion_task.waypoints
    if not isinstance(waypoints, (list, tuple)) or len(waypoints) < 2:
        raise ValueError('A path needs at least two waypoints')
    clean = []
    for waypoint in waypoints:
        if not isinstance(waypoint, dict) or set(waypoint) != set(graph.controls):
            raise ValueError('Each waypoint must explicitly specify every independent control')
        resolved = graph.resolve(waypoint)
        clean.append({name: resolved[name] for name in graph.controls})
    return clean


def certify_path(assembly, waypoints=None, *, distance_margin_mm=1e-4,
                 invariant_volume_tolerance_mm3=1e-7, max_depth=16,
                 max_pair_intervals=16383, joint_error_bounds=None,
                 drive_error_bounds=None, progress=None):
    """Certify every point of a joint-space polyline or return passed=False.

    Positive clearance is mandatory for non-invariant moving pairs. Invariant
    pairs may touch, but must pass an actual B-rep intersection-volume query.
    All unordered part pairs participate; no names/materials/parents exempted.
    """
    if not (math.isfinite(distance_margin_mm) and distance_margin_mm > 0
            and math.isfinite(invariant_volume_tolerance_mm3) and invariant_volume_tolerance_mm3 >= 0
            and isinstance(max_depth, int) and 0 <= max_depth <= 30
            and isinstance(max_pair_intervals, int) and max_pair_intervals > 0):
        raise ValueError('Invalid continuous clearance limits')
    plan = assembly.metadata.get('construction_recipe')
    if not plan:
        raise ValueError('Continuous clearance requires the authored construction recipe')
    graph = KinematicGraph([p.name for p in assembly.parts], plan['joints'], (plan.get('construction') or {}).get('motion'))
    waypoints = _path(graph, waypoints)
    control_errors, resolved_errors = _error_bounds(graph, joint_error_bounds, drive_error_bounds)
    parts = {p.name: p for p in assembly.parts}
    if any(part.cad is None for part in parts.values()):
        raise ValueError('Continuous clearance requires actual B-reps for all parts')
    if any(not p.cad.isValid() or not p.cad.Solids() or not math.isfinite(p.cad.Volume()) or p.cad.Volume() <= 0
           for p in parts.values()):
        raise ValueError('Continuous clearance requires valid positive-volume solids')
    specs = {p['name']: p for p in plan['parts']}
    if set(specs) != set(parts):
        raise ValueError('Recipe parts differ from the measured assembly')
    measured_hashes = {}
    for name, part in parts.items():
        measured_hashes[name] = _verified_primitive_source(part.cad, specs[name])
        if measured_hashes[name] is None:
            raise ValueError(f'{name}: actual B-rep differs from its rebuilt source recipe; regenerate before certifying')
    corners = {n: _corners(_box(p.cad)) for n, p in parts.items()}
    chains = {n: _moving_chain(graph, n) for n in parts}
    statistics = Counter()
    axisym_sources = measured_hashes
    segments, failures = [], []
    started = time.monotonic()
    for index, (start, end) in enumerate(zip(waypoints, waypoints[1:])):
        resolved_start, resolved_end = graph.resolve(start), graph.resolve(end)
        transform_cache, shape_cache = OrderedDict(), OrderedDict()

        def pose_at(u):
            if u not in transform_cache:
                transforms = graph.transforms({n: start[n] + u * (end[n] - start[n]) for n in graph.controls})
                boxes = {}
                for name in parts:
                    moved = corners[name] @ transforms[name][:3, :3].T + transforms[name][:3, 3]
                    boxes[name] = np.array([moved.min(axis=0), moved.max(axis=0)])
                transform_cache[u] = (transforms, boxes)
                if len(transform_cache) > 128:
                    transform_cache.popitem(last=False)
            return transform_cache[u]

        def shape_at(name, u):
            key = (name, u)
            if key not in shape_cache:
                shape_cache[key] = pose_cad(parts[name].cad, pose_at(u)[0][name])
                if len(shape_cache) > 512:
                    shape_cache.popitem(last=False)
            return shape_cache[key]

        rows = []
        for pair_index, (a, b) in enumerate(combinations(parts, 2)):
            residual_a, residual_b = _residual_chains(chains[a], chains[b])
            speed_a, error_a = point_motion_bounds(corners[a], residual_a, graph, resolved_start, resolved_end, resolved_errors)
            speed_b, error_b = point_motion_bounds(corners[b], residual_b, graph, resolved_start, resolved_end, resolved_errors)
            invariance = _coaxial_invariance(a, b, residual_a, residual_b, specs, graph)
            if invariance:
                stationary = invariance['axisymmetric_part']
                if stationary not in axisym_sources:
                    axisym_sources[stationary] = _verified_primitive_source(parts[stationary].cad, specs[stationary])
                if axisym_sources[stationary] is None:
                    invariance = None
                else:
                    invariance['verified_brep_sha256'] = axisym_sources[stationary]
            speed = 0. if invariance else speed_a + speed_b
            error_displacement = 0. if invariance else error_a + error_b
            if speed == error_displacement == 0 and invariance is None:
                invariance = {'method': 'constant_relative_rigid_transform',
                              'residual_joints': [list(residual_a), list(residual_b)]}
            row = {'parts': [a, b], 'relative_point_speed_bound_mm': speed,
                   'joint_error_displacement_bound_mm': error_displacement,
                   'certified_intervals': 0, 'deepest_subdivision': 0}
            if invariance:
                row['invariance'] = invariance
            pending = [(0., 1., 0)]
            minimum_bound = math.inf
            visited = 0
            try:
                while pending:
                    lo, hi, depth = pending.pop()
                    visited += 1
                    if visited > max_pair_intervals:
                        raise RuntimeError('Interval budget exhausted; path remains uncertified')
                    midpoint = (lo + hi) / 2
                    halfwidth = (hi - lo) / 2
                    row['deepest_subdivision'] = max(depth, row['deepest_subdivision'])
                    boxes = pose_at(midpoint)[1]
                    lower = _box_gap(boxes[a], boxes[b]) - halfwidth * speed - error_displacement - distance_margin_mm
                    method = 'aabb_interval_bound'
                    if lower <= 0:
                        shape_a, shape_b = shape_at(a, midpoint), shape_at(b, midpoint)
                        separation = _distance(shape_a, shape_b)
                        statistics['distance_queries'] += 1
                        lower = separation - halfwidth * speed - error_displacement - distance_margin_mm
                        method = 'brep_distance_interval_bound'
                        if speed == error_displacement == 0 and separation <= distance_margin_mm:
                            overlap = max(0., float(shape_a.intersect(shape_b).Volume()))
                            statistics['intersection_queries'] += 1
                            row['invariant_intersection_volume_mm3'] = overlap
                            if not math.isfinite(overlap) or overlap > invariant_volume_tolerance_mm3:
                                raise RuntimeError(f'Invariant pair overlaps by {overlap:g} mm^3')
                            method = 'invariant_contact_brep_volume'
                            lower = 0.
                        elif lower <= 0:
                            if depth >= max_depth:
                                raise RuntimeError(f'Unresolved clearance at segment parameter [{lo:g}, {hi:g}]; center distance {separation:g} mm')
                            pending.append((midpoint, hi, depth + 1))
                            pending.append((lo, midpoint, depth + 1))
                            continue
                    row['certified_intervals'] += 1
                    minimum_bound = min(minimum_bound, lower)
                    statistics[method] += 1
                row['passed'] = True
                row['minimum_clearance_lower_bound_mm'] = minimum_bound
            except Exception as exc:
                row['passed'] = False
                row['reason'] = str(exc)
                failures.append({'segment': index, **row})
            statistics['visited_pair_intervals'] += visited
            rows.append(row)
            if progress and (pair_index % 250 == 0 or not row['passed']):
                progress({'segment': index, 'pair': pair_index + 1, 'total_pairs': len(parts)*(len(parts)-1)//2,
                          'failures': len(failures), 'distance_queries': statistics['distance_queries'],
                          'latest': row if not row['passed'] else None})
        segments.append({'index': index, 'passed': all(row['passed'] for row in rows), 'pairs': rows})
    return {'schema': SCHEMA, 'passed': not failures, 'plan_sha256': plan_identity(plan),
            'compiled_geometry_sha256': (plan.get('construction') or {}).get('recipe_sha256'),
            'mechanism_sha256': (plan.get('construction') or {}).get('mechanism_sha256'),
            'model_sha256': (plan.get('construction') or {}).get('model_sha256'),
            'part_brep_sha256': measured_hashes,
            'measured_geometry_sha256': hashlib.sha256(json.dumps(measured_hashes, sort_keys=True).encode()).hexdigest(),
            'waypoints': waypoints, 'interpolation': 'piecewise_linear_in_independent_joint_coordinates',
            'joint_error_bounds': control_errors,
            'drive_error_bounds': dict(drive_error_bounds or {}),
            'resolved_joint_error_bounds': {n: resolved_errors[n] for n in graph.by_name if graph.by_name[n]['type'] != 'fixed'},
            'drive_error_semantics': 'Additional absolute follower error relative to its actual source value times ratio plus offset; propagated in drive order.',
            'joint_error_units': {name: 'degrees' if graph.by_name[name]['type'] == 'revolute' else 'mm' for name in graph.controls},
            'distance_margin_mm': distance_margin_mm,
            'invariant_volume_tolerance_mm3': invariant_volume_tolerance_mm3,
            'max_depth': max_depth, 'max_pair_intervals': max_pair_intervals,
            'segments': segments, 'failures': failures, 'statistics': dict(statistics),
            'runtime_seconds': round(time.monotonic()-started, 3),
            'scope': 'Entire continuous commanded polyline and the declared independent-joint absolute-error tube, including reverse traversal; not all independent joint-limit combinations.',
            'proof': 'For each pair and each closed interval, CAD distance or enclosing AABB gap at its midpoint exceeds conservative rigid-point interval travel plus joint-error displacement plus the numerical margin. Exact common ancestors cancel. Constructive axisymmetry is bound to the actual B-rep by a rebuilt, triangle-free serialization hash. Invariant-contact pairs have an explicit B-rep volume check.',
            'limitations': ['OpenCascade distance and boolean queries are numerical; this is not a formally verified exact-arithmetic kernel.',
                            'Invariant contact permits only the recorded numerical volume tolerance; it does not establish contact force or fit.',
                            'No tolerance stack, deformation, dynamics, external obstacles or hardware qualification.',
                            'An integrator trajectory is covered only if its entire state trajectory remains in the certified tube; discrete membership samples alone do not prove that.']}


def authorized_command(plan, certificate, values):
    """Reject off-path commands and return an exact point of the certified path.

    The small input comparison tolerance never expands the execution envelope:
    the caller must execute the returned canonical projection, not its input.
    Certificates are internal evidence records, not untrusted signed documents.
    """
    if (certificate.get('schema') != SCHEMA or certificate.get('passed') is not True
            or certificate.get('plan_sha256') != plan_identity(plan)
            or certificate.get('interpolation') != 'piecewise_linear_in_independent_joint_coordinates'):
        raise ValueError('A successful continuous path certificate for this exact plan is required')
    graph = KinematicGraph([p['name'] for p in plan['parts']], plan['joints'], (plan.get('construction') or {}).get('motion'))
    if set(values) != set(graph.controls):
        raise ValueError('Commands must explicitly specify every independent control')
    graph.resolve(values)
    points = _path(graph, certificate['waypoints'])
    names = graph.controls
    requested = np.array([values[n] for n in names], dtype=float)
    for a, b in zip(points, points[1:]):
        start = np.array([a[n] for n in names])
        delta = np.array([b[n] - a[n] for n in names])
        length = float(delta @ delta)
        u = 0. if length == 0 else float((requested - start) @ delta / length)
        if -1e-10 <= u <= 1 + 1e-10:
            canonical = start + min(1., max(0., u)) * delta
            if np.max(np.abs(requested-canonical), initial=0.) <= 1e-9:
                return dict(zip(names, map(float, canonical)))
    raise ValueError('Command is outside the certified path; certify the requested transition before execution')


def command_in_certified_tube(plan, certificate, values):
    """Check one independent-control state, assuming ideal driven constraints.

    Intersect the parameter intervals allowed by each coordinate's error bound.
    Unlike sampling nearby path points, this exactly tests membership in the
    union of segment-plus-box tubes (subject to numerical arithmetic). Actual
    finite-compliance followers require ``joint_state_in_certified_tube``.
    """
    if (certificate.get('schema') != SCHEMA or certificate.get('passed') is not True
            or certificate.get('plan_sha256') != plan_identity(plan)
            or certificate.get('interpolation') != 'piecewise_linear_in_independent_joint_coordinates'):
        raise ValueError('A successful continuous path certificate for this exact plan is required')
    graph = KinematicGraph([p['name'] for p in plan['parts']], plan['joints'], (plan.get('construction') or {}).get('motion'))
    if set(values) != set(graph.controls):
        raise ValueError('Commands must explicitly specify every independent control')
    graph.resolve(values)
    points = _path(graph, certificate['waypoints'])
    errors, _ = _error_bounds(graph, certificate.get('joint_error_bounds'))
    for a, b in zip(points, points[1:]):
        low, high = 0., 1.
        for name in graph.controls:
            delta, offset = b[name] - a[name], values[name] - a[name]
            if delta == 0:
                if abs(offset) > errors[name]:
                    low, high = 1., 0.
                    break
            else:
                first, last = (offset-errors[name])/delta, (offset+errors[name])/delta
                low, high = max(low, min(first, last)), min(high, max(first, last))
        if low <= high:
            return True
    return False


def joint_state_in_certified_tube(plan, certificate, values):
    """Test all actual movable joint values, including compliant followers.

    This checks one state only. It does not establish tracking bounds between
    solver output times or make a claim about an unobserved integrator path.
    Soft-limit violations are reported independently: the geometric error tube
    can contain them even though commanded values must obey graph joint limits.
    """
    checker = CertifiedTube(plan, certificate)
    return bool(checker.contains_states(list(values), [list(values.values())])[0])


class CertifiedTube:
    """Prepare once, then test batches of actual integration states cheaply.

    This does not authorize commands. Actual-state limit residuals remain
    independent evidence, while ``authorized_command`` enforces command limits.
    """

    def __init__(self, plan, certificate):
        if (certificate.get('schema') != SCHEMA or certificate.get('passed') is not True
                or certificate.get('plan_sha256') != plan_identity(plan)
                or certificate.get('interpolation') != 'piecewise_linear_in_independent_joint_coordinates'):
            raise ValueError('A successful continuous path certificate for this exact plan is required')
        graph = KinematicGraph([p['name'] for p in plan['parts']], plan['joints'], (plan.get('construction') or {}).get('motion'))
        self.joint_names = tuple(n for n, j in graph.by_name.items() if j['type'] != 'fixed')
        points = [graph.resolve(p) for p in _path(graph, certificate['waypoints'])]
        self.points = np.array([[p[n] for n in self.joint_names] for p in points])
        _, errors = _error_bounds(graph, certificate.get('joint_error_bounds'), certificate.get('drive_error_bounds'))
        self.errors = np.array([errors[n] for n in self.joint_names])

    def contains_states(self, joint_names, values):
        if len(set(joint_names)) != len(joint_names) or set(joint_names) != set(self.joint_names):
            raise ValueError('Actual joint state must include every movable joint, including followers')
        raw = np.asarray(values)
        if raw.dtype.kind not in 'fiu' or raw.ndim != 2 or raw.shape[1] != len(joint_names):
            raise ValueError('Actual joint state matrix must have finite numeric values')
        actual = raw.astype(float, copy=False)
        if not np.isfinite(actual).all():
            raise ValueError('Actual joint state must be finite')
        actual = actual[:, [joint_names.index(name) for name in self.joint_names]]
        accepted = np.zeros(len(actual), dtype=bool)
        for start, end in zip(self.points, self.points[1:]):
            low, high = np.zeros(len(actual)), np.ones(len(actual))
            for column, delta in enumerate(end-start):
                offset = actual[:, column] - start[column]
                if delta == 0:
                    low[np.abs(offset) > self.errors[column]] = 2.
                else:
                    first = (offset-self.errors[column])/delta
                    last = (offset+self.errors[column])/delta
                    low = np.maximum(low, np.minimum(first, last))
                    high = np.minimum(high, np.maximum(first, last))
            accepted |= low <= high
        return accepted
