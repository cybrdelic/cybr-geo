"""Sample actual constructed B-reps for interference; never a dynamics certificate.

Run from a checkout with ``PYTHONPATH=src python tools/audit_constructed.py``.
No part pair is exempt merely because it shares a parent or material. Explicit
intentional interference declarations remain visible in the result, with their
measured volume and reason. Touching faces are not a volume collision.
"""
from __future__ import annotations

import argparse
import hashlib
from itertools import combinations
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

from mechanism_lab.constructed import build
from mechanism_lab.core import pose_cad


def bounds(shape):
    box = shape.BoundingBox()
    return np.array([[box.xmin, box.ymin, box.zmin], [box.xmax, box.ymax, box.zmax]])


def separated(a, b, tolerance=1e-6):
    """Reject pairs with no positive-volume AABB overlap (including mere touch)."""
    return bool(np.any(np.minimum(a[1], b[1]) - np.maximum(a[0], b[0]) <= tolerance))


def distance(a, b):
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    query = BRepExtrema_DistShapeShape(a.wrapped, b.wrapped)
    query.Perform()
    if not query.IsDone():
        raise RuntimeError('OpenCascade distance query failed')
    return float(query.Value())


def interface_parts(record):
    pair = record.get('parts')
    if isinstance(pair, (list, tuple)) and len(pair) == 2:
        return pair
    for left, right in (('male', 'female'), ('shaft', 'bushing'), ('part_a', 'part_b')):
        if isinstance(record.get(left), str) and isinstance(record.get(right), str):
            return [record[left], record[right]]
    return None


def audit(assembly, times=(0., 1., 2., 3., 4.), volume_tolerance=1e-5):
    if not times or any(not math.isfinite(t) for t in times):
        raise ValueError('Provide finite nonempty sample times')
    if not math.isfinite(volume_tolerance) or volume_tolerance <= 0:
        raise ValueError('Volume tolerance must be positive and finite')
    if any(part.cad is None for part in assembly.parts):
        raise ValueError('Interference audit requires actual B-reps for every part')
    construction = assembly.metadata.get('construction', {})
    exemptions = {}
    for record in construction.get('intentional_interferences', []):
        pair = record.get('parts')
        if not isinstance(pair, list) or len(pair) != 2 or not record.get('reason'):
            raise ValueError('Intentional interference needs an exact named pair and reason')
        ceiling = record.get('maximum_volume_mm3')
        if not isinstance(ceiling, (int, float)) or not math.isfinite(ceiling) or ceiling <= 0:
            raise ValueError('Intentional interference needs a finite positive volume ceiling')
        exemptions[frozenset(pair)] = record
    part_names = {part.name for part in assembly.parts}
    if any(set(pair) - part_names for pair in exemptions):
        raise ValueError('Intentional interference references an unknown part')
    interfaces = construction.get('interfaces', [])
    if isinstance(interfaces, dict):
        interfaces = [{'id': key, **value} for key, value in interfaces.items()]
    probes = construction.get('capture_probes', [])
    for probe in probes:
        direction = np.asarray(probe.get('direction'), dtype=float)
        travel = probe.get('distance_mm')
        if (probe.get('moving_part') not in part_names or probe.get('blocker') not in part_names
                or direction.shape != (3,) or not np.isfinite(direction).all() or np.linalg.norm(direction) < 1e-10
                or not isinstance(travel, (int, float)) or not math.isfinite(travel) or travel <= 0):
            raise ValueError('Capture probes need named moving/blocking parts, direction, and finite positive distance')
    started = time.monotonic()
    samples = []
    # Exact rigid-body invariance permits reusing a measured pair when its
    # relative pose is unchanged. This never exempts an attachment from its
    # first actual B-rep query. Rounding only removes floating-point noise far
    # below the kernel's modelling tolerance, in millimetres/radians.
    intersection_cache = {}
    distance_cache = {}
    capture_cache = {}
    for t in times:
        transforms = {part.name: assembly.pose(part, t, 0) for part in assembly.parts}
        inverse = {name: np.linalg.inv(transform) for name, transform in transforms.items()}
        shapes = {part.name: pose_cad(part.cad, transforms[part.name]) for part in assembly.parts}
        boxes = {name: bounds(shape) for name, shape in shapes.items()}
        collisions, declared, errors = [], [], []
        candidate_count = 0
        kernel_queries = 0
        reused_pairs = 0
        for a, b in combinations(shapes, 2):
            if separated(boxes[a], boxes[b]):
                continue
            candidate_count += 1
            try:
                relative = np.round(inverse[a] @ transforms[b], 9)
                relative[relative == 0] = 0  # canonicalize signed zero
                key = (a, b, relative.tobytes())
                if key in intersection_cache:
                    volume = intersection_cache[key]
                    reused_pairs += 1
                else:
                    overlap = shapes[a].intersect(shapes[b])
                    volume = max(0., float(overlap.Volume()))
                    intersection_cache[key] = volume
                    kernel_queries += 1
            except Exception as exc:
                errors.append({'parts': [a, b], 'error': str(exc)})
                continue
            if volume <= volume_tolerance:
                continue
            row = {'parts': [a, b], 'intersection_volume_mm3': round(volume, 9)}
            permission = exemptions.get(frozenset((a, b)))
            if permission and volume <= permission['maximum_volume_mm3'] + volume_tolerance:
                row['reason'] = permission['reason']
                row['maximum_volume_mm3'] = permission['maximum_volume_mm3']
                declared.append(row)
            else:
                if permission:
                    row['exceeded_declared_maximum_mm3'] = permission['maximum_volume_mm3']
                collisions.append(row)
        measured_interfaces = []
        for index, interface in enumerate(interfaces):
            pair = interface_parts(interface)
            row = {'id': interface.get('id', interface.get('name', str(index))), 'declared': interface}
            if pair is None:
                row['status'] = 'not_measurable_without_named_pair'
            elif set(pair) - part_names:
                row['status'] = 'invalid_part_reference'
                errors.append({'interface': row['id'], 'error': 'Unknown interface part'})
            else:
                row['parts'] = pair
                try:
                    relative = np.round(inverse[pair[0]] @ transforms[pair[1]], 9)
                    relative[relative == 0] = 0
                    key = (*pair, relative.tobytes())
                    if key not in distance_cache:
                        distance_cache[key] = distance(shapes[pair[0]], shapes[pair[1]])
                    row['minimum_surface_distance_mm'] = round(distance_cache[key], 9)
                    row['status'] = 'measured'
                except Exception as exc:
                    row['status'] = 'distance_query_failed'
                    errors.append({'interface': row['id'], 'error': str(exc)})
            measured_interfaces.append(row)
        captures = []
        for probe in probes:
            moving, blocker = probe['moving_part'], probe['blocker']
            relative = np.round(inverse[moving] @ transforms[blocker], 9)
            relative[relative == 0] = 0
            key = (json.dumps(probe, sort_keys=True), relative.tobytes())
            row = dict(probe)
            try:
                if key not in capture_cache:
                    direction = np.asarray(probe['direction'], dtype=float)
                    local_delta = direction / np.linalg.norm(direction) * probe['distance_mm']
                    world_delta = transforms[moving][:3, :3] @ local_delta
                    displaced = shapes[moving].translate(tuple(world_delta))
                    capture_cache[key] = max(0., float(displaced.intersect(shapes[blocker]).Volume()))
                row['virtual_displacement_interference_mm3'] = round(capture_cache[key], 9)
                row['blocked'] = capture_cache[key] > volume_tolerance
            except Exception as exc:
                row['blocked'] = False
                errors.append({'capture_probe': probe.get('name'), 'error': str(exc)})
            captures.append(row)
        samples.append({'time_seconds': t, 'all_part_pairs': len(shapes) * (len(shapes) - 1) // 2,
                        'positive_aabb_candidates': candidate_count,
                        'intersection_kernel_queries': kernel_queries, 'unchanged_relative_pose_reuses': reused_pairs,
                        'unexpected_interferences': collisions,
                        'declared_interferences': declared, 'interfaces': measured_interfaces,
                        'capture_probes': captures, 'errors': errors})
        failures = [row for row in captures if not row['blocked']]
        print(f"{assembly.name} t={t:g}: {candidate_count} B-rep pairs, {len(collisions)} unexpected intersections, {len(failures)} unblocked capture probes, {len(errors)} query errors", flush=True, file=sys.stderr)
        for collision in collisions[:12]:
            print(json.dumps({'time_seconds': t, **collision}), flush=True, file=sys.stderr)
        for failure in failures:
            print(json.dumps({'time_seconds': t, **failure}), flush=True, file=sys.stderr)
    recipe = assembly.metadata.get('construction_recipe')
    recipe_sha256 = hashlib.sha256(json.dumps(recipe, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest() if recipe else None
    return {'schema': 'cybrgeo-sampled-cad-audit/1', 'model': assembly.name,
            'passed': all(not item['unexpected_interferences'] and not item['errors']
                          and all(probe['blocked'] for probe in item['capture_probes']) for item in samples),
            'construction_recipe_sha256': recipe_sha256,
            'compiled_geometry_sha256': construction.get('recipe_sha256'),
            'parts': len(assembly.parts), 'units': 'mm', 'volume_tolerance_mm3': volume_tolerance,
            'part_brep_volumes_mm3': {part.name: round(float(part.cad.Volume()), 9) for part in assembly.parts},
            'runtime_seconds': round(time.monotonic() - started, 3), 'samples': samples,
            'scope': 'Actual analytic B-rep intersection volume and minimum surface distance at the listed prescribed poses only. No broad parent-pair exemption.',
            'query_reuse': 'Measured intersections/distances reused only when the same named pair has the same relative rigid transform rounded to 1e-9.',
            'capture_probe_scope': 'Virtual axial displacement of a captured part must intersect its named stop. This checks geometric retention, not clamp force, thread strength, or simulated contact.',
            'aabb_overlap_tolerance_mm': 1e-6,
            'limitations': ['Sampled poses do not prove continuous-path or full joint-range collision freedom.',
                            'Minimum surface distance is not a tolerance stack, bearing fit, or proof of mating-surface contact.',
                            'No loads, contact dynamics, motor sizing, thermal behavior, fatigue, or hardware qualification.']}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('architecture', nargs='?', default='compact_arm', choices=['compact_arm', 'belt_arm', 'inspection_head'])
    parser.add_argument('--parameters', type=Path)
    parser.add_argument('--times', nargs='+', type=float, default=[0., 1., 2., 3., 4.])
    parser.add_argument('--out', type=Path, default=Path('outputs/mechanisms/audit.json'))
    args = parser.parse_args(argv)
    # A failed rerun cannot leave an earlier success receipt at this destination.
    args.out.unlink(missing_ok=True)
    parameters = json.loads(args.parameters.read_text(encoding='utf-8')) if args.parameters else None
    report = audit(build(args.architecture, parameters), args.times)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'model': report['model'], 'passed': report['passed'], 'report': str(args.out),
                      'unexpected_interferences': sum(len(row['unexpected_interferences']) for row in report['samples']),
                      'unblocked_capture_probes': sum(not probe['blocked'] for row in report['samples'] for probe in row['capture_probes'])}, indent=2))
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
