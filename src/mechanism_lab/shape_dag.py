"""Evaluate a reusable, millimetre CSG DAG as analytic OpenCascade B-reps.

The tree interchange semantics were inspected in ARX-6's ``capture_csg.py``,
``build_csg_shader.py`` and ``csg_symmetry.py`` at the pinned source below. This
module implements those operations, not the ARX assembly or its generator.
The generic ``helix`` operation follows the compact source's native sweep,
identified separately in OPERATION_REFERENCES; it starts at Z=0, not -height/2.
``box`` arguments are half extents; cylinders and polygon extrusions use half
height about local Z. Matrices are row-major and multiply column points.

An exact B-rep evaluation of *encoded* CSG is not an exact reconstruction of
the native source: the initial capture omits fillets and dynamic springs and
supplies no authoritative mass properties. The retained ARX pinion profile has
a later sampled 2D contour refinement; this does not restore all native CAD
features. Provenance is retained, never promoted into
mechanical, dynamics or hardware qualification. This module is GPL-2.0-only,
like CYBR GEO; no source design dimensions or executable generator are bundled.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from numbers import Real
from typing import Mapping

import numpy as np


SCHEMA = 'cybrgeo-shape-dag/1'
# SHA256 of the inspected files after universal-newline UTF-8 decoding, tied to
# their repository commit (Windows checkout byte hashes differ with CRLF).
SCHEMA_REFERENCE = {
    'source': 'ARX-6 belt full source',
    'commit': 'aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2',
    'hash_convention': 'sha256(UTF-8 text with LF newlines)',
    'files': {
        'ARX-6-P5-Shader/tools/capture_csg.py': '9cecc0c3845f8f738ba9a4921f3c6c69a72afc5410932d7f924b8627fa4e3846',
        'ARX-6-P5-Shader/tools/build_csg_shader.py': '4d5da4c3a95ac3d6cafae47ec1d7e236cb04b2bfdc0d00125db1fcd919ee29cf',
        'ARX-6-P5-Shader/tools/csg_symmetry.py': 'e14ecfc0ba2bed48c99c025cd4e0fac197d3ae5c711b469a0e554cca148c2a10',
        'ARX-6-P5-Engineering/integration/generate_cad_p2.py': '5b43ee72fcb4b5358d0918e9da3fda88d2f6c50b26c2960a122347dc7ed6da72',
    },
    'capture_limitations': ['initial capture omits fillets; retained pinion has sampled 2D contour refinement',
                            'dynamic springs omitted',
                            'captured mass is not authoritative native CAD mass'],
}
OPERATION_REFERENCES = {
    'helix': {
        'repository': 'https://github.com/cybrdelic/cybr-arx6-compact',
        'commit': '0003382bd6a2e96f3bed065003efec35001982f7',
        'file': 'source/build_meshes.py',
        'sha256': '1b2e4f087634f8ff41ad450195a2a35ef3ef6445d1bc25efbad9c797f7fb2e50',
        'semantics': 'radius, wire radius, pitch, turns; circular XZ profile swept with Frenet frame; path Z=0..pitch*turns',
    },
}
_ARITY = {name: 0 for name in ('box', 'cylinder', 'sphere', 'cone', 'polygon',
                             'radial_cylinders', 'linear_polygon', 'belt_teeth', 'helix')}
_ARITY.update(transform=1, union=2, cut=2, intersection=2)
_MAX_NODES = 100_000
_MAX_DEPTH = 256
_MAX_ARRAY = 4096


def _number(value, label, *, positive=False, nonnegative=False):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f'{label} must be a finite number')
    value = float(value)
    if positive and value <= 0 or nonnegative and value < 0:
        raise ValueError(f'{label} must be {"positive" if positive else "nonnegative"}')
    return value if value else 0.0


def _count(value):
    value = _number(value, 'array count', positive=True)
    if not value.is_integer() or value > _MAX_ARRAY:
        raise ValueError(f'array count must be an integer in 1..{_MAX_ARRAY}')
    return int(value)


def _matrix(value):
    # Permit a flat captured matrix or an explicit nested 4x4 body frame.
    try:
        raw = np.asarray(value, dtype=object)
        if raw.shape not in ((16,), (4, 4)):
            raise ValueError('shape')
        matrix = np.array([_number(v, 'transform') for v in raw.flat]).reshape(4, 4)
    except (TypeError, ValueError) as exc:
        raise ValueError('transform must be a finite row-major 4x4 matrix') from exc
    if not np.array_equal(matrix[3], [0, 0, 0, 1]):
        raise ValueError('transform must be affine with last row [0, 0, 0, 1]')
    linear = matrix[:3, :3]
    if not np.allclose(linear.T @ linear, np.eye(3), rtol=0, atol=1e-10):
        raise ValueError('transform must be rigid or a reflection; scale/shear is unsupported')
    # Capture rounds rigid matrices to 12 decimal places. OpenCascade restores
    # orthogonality at its transform boundary; no general affine approximation.
    return matrix


def _polygon(value):
    if not isinstance(value, (list, tuple)) or not 3 <= len(value) <= 100_000:
        raise ValueError('polygon must contain 3..100000 XY vertices')
    points = []
    for row in value:
        if not isinstance(row, (list, tuple)) or len(row) != 2:
            raise ValueError('polygon vertices must have two coordinates')
        points.append([_number(v, 'polygon vertex') for v in row])
    if points[0] == points[-1]:
        points.pop()
    from shapely.geometry import Polygon
    if len(points) < 3 or len({tuple(p) for p in points}) != len(points):
        raise ValueError('polygon vertices must be distinct')
    polygon = Polygon(points)
    if not polygon.is_valid or polygon.area <= 0:
        raise ValueError('polygon must be simple and have positive area')
    return points


def _normalize_node(node, children):
    if not isinstance(node, Mapping) or set(node) - {'op', 'args', 'children'}:
        raise ValueError('shape node must contain only op, args and children')
    op, args = node.get('op'), node.get('args', [])
    if not isinstance(op, str) or op not in _ARITY:
        raise ValueError(f'unsupported shape operation: {op!r}')
    if len(children) != _ARITY[op]:
        raise ValueError(f'{op} needs {_ARITY[op]} children')
    if not isinstance(args, (list, tuple)):
        raise ValueError(f'{op} args must be a list')
    expected = {'box': 3, 'cylinder': 2, 'sphere': 1, 'cone': 3,
                'polygon': 2, 'transform': 16, 'radial_cylinders': 5,
                'linear_polygon': 5, 'belt_teeth': 7, 'helix': 4}.get(op, 0)
    if len(args) != expected:
        raise ValueError(f'{op} needs {expected} arguments')
    if op == 'transform':
        args = _matrix(args).ravel().tolist()
    elif op in ('polygon', 'linear_polygon'):
        args = [_polygon(args[0]), _number(args[1], 'half height', positive=True)] + [
            _number(v, op) for v in args[2:]]
        if op == 'linear_polygon':
            args[4] = _count(args[4])
    else:
        args = [_number(v, op) for v in args]
        if op in ('box', 'cylinder', 'sphere') and any(v <= 0 for v in args):
            raise ValueError(f'{op} dimensions must be positive')
        if op == 'cone' and (min(args[:2]) < 0 or max(args[:2]) <= 0 or args[2] <= 0):
            raise ValueError('cone radii must be nonnegative, not both zero, and half height positive')
        if op == 'helix' and (min(args) <= 0 or args[1] >= args[0] or args[3] > 1024):
            raise ValueError('helix requires positive radius, wire radius, pitch and turns; wire radius < coil radius and turns <= 1024')
        if op == 'radial_cylinders':
            if min(args[:2]) <= 0 or args[2] < 0:
                raise ValueError('radial cylinders need positive dimensions and nonnegative center radius')
            args[4] = _count(args[4])
        if op == 'belt_teeth':
            if min(args[:3]) <= 0 or args[3] < 0 or args[4] <= 0 or not 0 <= args[5] < args[2]:
                raise ValueError('invalid belt tooth or pitch path dimensions')
            args[6] = _count(args[6])
            # This opcode follows a single lap, as in the captured generator.
            if (args[6] - 1) * args[4] >= 2 * math.pi * args[2] + 2 * args[3]:
                raise ValueError('belt tooth centers exceed one pitch-path lap')
    return dict(op=op, args=args, children=list(children))


def _node_id(node):
    return hashlib.sha256(json.dumps(node, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def from_trees(trees: Mapping[str, dict], *, provenance=None):
    """Intern a named forest of captured trees into one serializable shared DAG.

    Identical subtrees get the same content ID, across all named roots. Source
    provenance is retained verbatim as JSON data; it is not a validation claim.
    """
    if not isinstance(trees, Mapping) or not trees or any(not isinstance(k, str) or not k for k in trees):
        raise ValueError('trees must be a nonempty map of root names to trees')
    nodes, roots, active, memo = {}, {}, set(), {}
    visits = 0

    def visit(tree, depth):
        nonlocal visits
        if depth > _MAX_DEPTH:
            raise ValueError(f'shape tree exceeds depth {_MAX_DEPTH}')
        if not isinstance(tree, Mapping):
            raise ValueError('shape child must be a node')
        key = id(tree)
        if key in active:
            raise ValueError('cycle in shape tree')
        if key in memo:
            return memo[key]
        visits += 1
        if visits > _MAX_NODES:
            raise ValueError(f'shape tree exceeds {_MAX_NODES} input nodes')
        children = tree.get('children', [])
        if not isinstance(children, (list, tuple)):
            raise ValueError('shape children must be a list')
        active.add(key)
        node = _normalize_node(tree, [visit(c, depth + 1) for c in children])
        active.remove(key)
        identifier = _node_id(node)
        nodes[identifier] = node
        memo[key] = identifier
        return identifier

    for name, tree in trees.items():
        roots[name] = visit(tree, 1)
    result = dict(schema=SCHEMA, units='mm', nodes=nodes, roots=roots)
    if provenance is not None:
        result['provenance'] = json.loads(json.dumps(provenance, allow_nan=False))
    return result


def from_tree(tree, *, provenance=None):
    """Lower one captured tree; its root name is ``shape``."""
    return from_trees({'shape': tree}, provenance=provenance)


def _transformed(shape, matrix):
    import cadquery as cq
    from OCP.gp import gp_Trsf
    # Constructing cq.Matrix directly from entries leaves gp_GTrsf's form as
    # 'other', causing Trsf() to reject even exact mirrors. gp_Trsf retains
    # analytic surface types and handles determinant -1 without meshing.
    transform = gp_Trsf()
    transform.SetValues(*matrix[:3, :].ravel().tolist())
    return shape.transformShape(cq.Matrix(transform))


def _belt_point(distance, radius, span, offset):
    half_arc, r = math.pi * radius, radius - offset
    if distance < half_arc:
        angle = math.pi + distance / radius
        return r * math.cos(angle), r * math.sin(angle)
    distance -= half_arc
    if distance < span:
        return r, distance
    distance -= span
    if distance < half_arc:
        angle = distance / radius
        return r * math.cos(angle), span + r * math.sin(angle)
    return -r, span - (distance - half_arc)


class ShapeEvaluator:
    """Validated, memoized analytic evaluation; reusable for many part roots.

    ``evaluate(root, frame=body_to_world)`` applies the frame AFTER the root's
    own transforms. With no frame, the result stays in its authored body-local
    coordinates. Returned shapes are copies so caller edits cannot poison the
    memoized geometry. Empty intersections and disconnected compounds are valid
    CSG results; assembly engineering policy must assess their suitability.
    """

    def __init__(self, dag):
        if not isinstance(dag, Mapping) or dag.get('schema') != SCHEMA or dag.get('units') != 'mm':
            raise ValueError('unsupported shape DAG schema or units')
        self.dag = deepcopy(dag)
        nodes, roots = self.dag.get('nodes'), self.dag.get('roots')
        if not isinstance(nodes, dict) or not 1 <= len(nodes) <= _MAX_NODES:
            raise ValueError('shape DAG requires a bounded nonempty node map')
        if not isinstance(roots, dict) or not roots or any(not isinstance(k, str) or not k for k in roots):
            raise ValueError('shape DAG requires named roots')
        for key, node in nodes.items():
            if not isinstance(key, str) or not isinstance(node, Mapping):
                raise ValueError('invalid shape DAG node')
            children = node.get('children', [])
            if not isinstance(children, list) or any(not isinstance(c, str) or c not in nodes for c in children):
                raise ValueError('unknown child in shape DAG')
            normalized = _normalize_node(node, children)
            if _node_id(normalized) != key:
                raise ValueError('shape DAG node content does not match its SHA256 ID')
            nodes[key] = normalized
        if any(not isinstance(v, str) or v not in nodes for v in roots.values()):
            raise ValueError('unknown root in shape DAG')
        marks, depths = {}, {}
        # Compute descendant depth, not traversal depth: nodes serialized in
        # child-first order must not evade the limit through a visited cache.
        # The validator itself is iterative even for an adversarial long DAG.
        for key in nodes:
            stack = [(key, False)]
            while stack:
                current, exiting = stack.pop()
                if exiting:
                    depths[current] = 1 + max((depths[c] for c in nodes[current]['children']), default=0)
                    if depths[current] > _MAX_DEPTH:
                        raise ValueError('shape DAG exceeds maximum depth')
                    marks[current] = 2
                    continue
                if marks.get(current) == 2:
                    continue
                if marks.get(current) == 1:
                    raise ValueError('cycle in shape DAG')
                marks[current] = 1
                stack.append((current, True))
                stack.extend((child, False) for child in nodes[current]['children'])
        self._cache = {}

    def evaluate(self, root=None, *, frame=None):
        """Evaluate a named root (or node ID); omit root for a single-root DAG."""
        roots = self.dag['roots']
        if root is None:
            if len(roots) != 1:
                raise ValueError('choose a root for a multi-root shape DAG')
            root = next(iter(roots.values()))
        else:
            root = roots.get(root, root)
        if root not in self.dag['nodes']:
            raise ValueError(f'unknown shape root: {root}')
        matrix = _matrix(frame) if frame is not None else None
        shape = self._evaluate(root)
        return _transformed(shape, matrix) if matrix is not None else shape.copy()

    def _evaluate(self, key):
        if key in self._cache:
            return self._cache[key]
        import cadquery as cq
        node = self.dag['nodes'][key]
        op, args = node['op'], node['args']
        children = [self._evaluate(c) for c in node['children']]
        try:
            if op == 'box':
                shape = cq.Workplane('XY').box(*(2 * x for x in args)).val()
            elif op == 'cylinder':
                shape = cq.Solid.makeCylinder(args[0], 2 * args[1], (0, 0, -args[1]))
            elif op == 'sphere':
                shape = cq.Workplane('XY').sphere(args[0]).val()
            elif op == 'cone':
                shape = (cq.Solid.makeCylinder(args[0], 2 * args[2], (0, 0, -args[2]))
                         if args[0] == args[1] else
                         cq.Solid.makeCone(args[0], args[1], 2 * args[2], (0, 0, -args[2])))
            elif op == 'polygon':
                shape = cq.Workplane('XY').polyline(args[0]).close().extrude(2 * args[1]).val().translate((0, 0, -args[1]))
            elif op == 'helix':
                radius, wire_radius, pitch, turns = args
                path = cq.Wire.makeHelix(pitch, pitch * turns, radius)
                profile = cq.Workplane('XZ').center(radius, 0).circle(wire_radius).val()
                shape = cq.Solid.sweep(profile, [], path, makeSolid=True, isFrenet=True)
            elif op == 'transform':
                shape = _transformed(children[0], np.array(args).reshape(4, 4))
            elif op in ('union', 'cut', 'intersection'):
                shape = getattr(children[0], {'union': 'fuse', 'cut': 'cut', 'intersection': 'intersect'}[op])(children[1])
            else:
                if op == 'radial_cylinders':
                    r, h, radius, phase, count = args
                    leaf = cq.Solid.makeCylinder(r, 2 * h, (0, 0, -h))
                    points = [(radius * math.cos(phase + 2 * math.pi * k / count),
                               radius * math.sin(phase + 2 * math.pi * k / count)) for k in range(count)]
                elif op == 'linear_polygon':
                    points_xy, h, start, step, count = args
                    leaf = cq.Workplane('XY').polyline(points_xy).close().extrude(2 * h).val().translate((0, 0, -h))
                    points = [(start + step * k, 0) for k in range(count)]
                else:  # belt_teeth, validated before any CAD operation
                    r, h, radius, span, pitch, offset, count = args
                    leaf = cq.Solid.makeCylinder(r, 2 * h, (0, 0, -h))
                    points = [_belt_point(k * pitch, radius, span, offset) for k in range(count)]
                solids = [leaf.translate((x, y, 0)) for x, y in points]
                shape = solids[0].fuse(*solids[1:]) if len(solids) > 1 else solids[0]
            if shape.isNull() or not shape.isValid():
                raise ValueError('kernel returned an invalid shape')
        except Exception as exc:
            raise ValueError(f'CAD evaluation failed at {op} node {key[:12]}: {exc}') from exc
        self._cache[key] = shape
        return shape


def evaluate_instance(record, *, body_frames=None):
    """Evaluate a captured ``{body, shape, ...}`` record, optionally in world.

    An explicit frame map must contain this body's frame; missing frames fail
    instead of placing moving bodies at an invented world origin. No material,
    mass, joints or capture validation claims are inferred by this function.
    """
    if not isinstance(record, Mapping) or 'shape' not in record:
        raise ValueError('instance requires a captured shape tree')
    frame = None
    if body_frames is not None:
        body = record.get('body')
        if not isinstance(body_frames, Mapping) or body not in body_frames:
            raise ValueError(f'missing body frame: {body!r}')
        frame = body_frames[body]
    return ShapeEvaluator(from_tree(record['shape'])).evaluate(frame=frame)


def flat_plan_part_to_dag(spec):
    """Lower legacy body/unions/cuts to the same DAG, in original order.

    The legacy primitive's size is full box dimensions, cylinder diameter/height
    or sphere diameter. Rotations are degrees, applied X then Y then Z before
    translation. Part fillet is deliberately left for the caller to apply once
    after the complete Boolean result, preserving existing plan behavior.
    """
    def primitive(row):
        if not isinstance(row, Mapping):
            raise ValueError('flat primitive must be a mapping')
        size = row.get('size')
        if not isinstance(size, (list, tuple)) or len(size) != 3:
            raise ValueError('flat primitive size must have three coordinates')
        size = [_number(v, 'size', positive=True) for v in size]
        kind = row.get('kind')
        if kind == 'box':
            args = [v / 2 for v in size]
        elif kind == 'cylinder':
            args = [size[0] / 2, size[2] / 2]
        elif kind == 'sphere':
            args = [size[0] / 2]
        else:
            raise ValueError(f'unsupported flat primitive: {kind!r}')
        tree = dict(op=kind, args=args, children=[])
        rotation, position = row.get('rotation', [0, 0, 0]), row.get('position', [0, 0, 0])
        for name, vector in [('rotation', rotation), ('position', position)]:
            if not isinstance(vector, (list, tuple)) or len(vector) != 3:
                raise ValueError(f'{name} must have three coordinates')
        matrix = np.eye(4)
        for axis, angle in enumerate(rotation):
            angle = math.radians(_number(angle, 'rotation'))
            transform = np.eye(4)
            i, j = (axis + 1) % 3, (axis + 2) % 3
            transform[i, i] = transform[j, j] = math.cos(angle)
            transform[i, j], transform[j, i] = -math.sin(angle), math.sin(angle)
            matrix = transform @ matrix
        matrix[:3, 3] = [_number(v, 'position') for v in position]
        return dict(op='transform', args=matrix.ravel().tolist(), children=[tree])

    if not isinstance(spec, Mapping) or 'body' not in spec:
        raise ValueError('flat part needs a body primitive')
    tree = primitive(spec['body'])
    for field, op in [('unions', 'union'), ('cuts', 'cut')]:
        rows = spec.get(field, [])
        if not isinstance(rows, list):
            raise ValueError(f'{field} must be a list')
        for row in rows:
            tree = dict(op=op, args=[], children=[tree, primitive(row)])
    return from_tree(tree)
