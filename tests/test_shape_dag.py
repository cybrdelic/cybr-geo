"""Analytic parity, frame order and fail-closed CSG interchange tests.

Small clamp/mirror fixtures below reflect the pinned ARX captured records; their
source identity is SCHEMA_REFERENCE. They test captured CSG semantics only,
not native ARX fillet, mass, dynamics or hardware fidelity.
"""
from copy import deepcopy
import hashlib
import json
import math

import numpy as np
import pytest

from mechanism_lab.shape_dag import (
    SCHEMA_REFERENCE, ShapeEvaluator, evaluate_instance, flat_plan_part_to_dag,
    from_tree, from_trees,
)


def node(op, args=(), *children):
    return dict(op=op, args=list(args), children=list(children))


def shifted(tree, xyz):
    matrix = np.eye(4)
    matrix[:3, 3] = xyz
    return node('transform', matrix.ravel().tolist(), tree)


def evaluate(tree):
    return ShapeEvaluator(from_tree(tree)).evaluate()


def assert_same_solid(actual, expected, tolerance=1e-7):
    assert actual.isValid() and expected.isValid()
    assert actual.Volume() == pytest.approx(expected.Volume(), abs=tolerance)
    assert actual.cut(expected).Volume() < tolerance
    assert expected.cut(actual).Volume() < tolerance


@pytest.mark.parametrize('tree,volume', [
    (node('box', [1, 2, 3]), 48),
    (node('cylinder', [2, 3]), 24 * math.pi),
    (node('sphere', [2]), 32 * math.pi / 3),
    (node('cone', [2, 1, 3]), 14 * math.pi),
    (node('cone', [2, 2, 3]), 24 * math.pi),
    (node('cone', [0, 2, 3]), 8 * math.pi),
    (node('polygon', [[[-2, -1], [2, -1], [2, 1], [-2, 1]], 3]), 48),
])
def test_centered_analytic_primitives(tree, volume):
    shape = evaluate(tree)
    assert shape.isValid()
    assert shape.Volume() == pytest.approx(volume, rel=1e-10)
    box = shape.BoundingBox()
    assert box.zmin == pytest.approx(-tree['args'][-1] if tree['op'] != 'sphere' else -2)
    assert box.zmax == pytest.approx(tree['args'][-1] if tree['op'] != 'sphere' else 2)


def test_actual_captured_clamp_intersection_matches_independent_native_cad():
    import cadquery as cq
    # csg-recipe.json: tool/gripper_cable_clamp_base_17. The source's XZ
    # workplane has a -Y normal, represented by this captured matrix.
    xz = [1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1]
    clamp = node('cut', [], node('box', [8, 3.5, 3.5]),
                 node('transform', xz, node('cylinder', [2.5, 5])))
    for x in (-5.5, 5.5):
        clamp = node('cut', [], clamp, shifted(node('cylinder', [1.1, 5]), (x, 0, 0)))
    clamp = node('intersection', [], clamp, shifted(node('box', [15, 6, 3.5]), (0, 0, -3.5)))
    record = dict(body='tool', name='gripper_cable_clamp_base_17',
                  shape=shifted(clamp, (0, 191, -23)))
    native = cq.Workplane('XY').box(16, 7, 7).val()
    native = native.cut(cq.Workplane('XZ').circle(2.5).extrude(5, both=True).val())
    for x in (-5.5, 5.5):
        native = native.cut(cq.Workplane('XY').center(x, 0).circle(1.1).extrude(5, both=True).val())
    native = native.intersect(cq.Workplane('XY').box(30, 12, 7).translate((0, 0, -3.5)).val())
    native = native.translate((0, 191, -23))
    assert_same_solid(evaluate_instance(record), native)
    assert SCHEMA_REFERENCE['commit'] == 'aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2'


def test_actual_captured_reflection_and_body_local_frame_order():
    # csg-recipe.json: jaw_right/left_pad. Frame is explicit, not guessed from
    # the named body or the pad's captured local bounds.
    mirror = [-1, 0, 0, -.9, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    record = dict(body='jaw_right', shape=node('transform', mirror, node('box', [.9, 12, 9.2])))
    local = evaluate_instance(record)
    assert local.Volume() == pytest.approx(1.8 * 24 * 18.4)
    assert local.BoundingBox().xmin == pytest.approx(-1.8)
    assert local.BoundingBox().xmax == pytest.approx(0)
    frame = [[0, -1, 0, 10], [1, 0, 0, 20], [0, 0, 1, 30], [0, 0, 0, 1]]
    world = evaluate_instance(record, body_frames={'jaw_right': frame})
    assert world.Center().toTuple() == pytest.approx((10, 19.1, 30))
    assert world.BoundingBox().ymin == pytest.approx(18.2)
    with pytest.raises(ValueError, match='missing body frame'):
        evaluate_instance(record, body_frames={'unrelated': np.eye(4)})


def test_nested_noncommuting_transforms_preserve_analytic_surface():
    tree = shifted(node('cylinder', [1, 2]), (3, 0, 0))
    rotate = [0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    tree = shifted(node('transform', rotate, tree), (10, 0, 0))
    shape = evaluate(tree)
    assert shape.Center().toTuple() == pytest.approx((10, 3, 0))
    assert any(f.geomType() == 'CYLINDER' for f in shape.Faces())


def test_shared_forest_provenance_and_returned_shapes_are_isolated():
    import cadquery as cq
    leaf = node('box', [1, 2, 3])
    provenance = {'source': 'test source', 'limitations': ['captured fillets omitted']}
    dag = from_trees({'left': shifted(leaf, (-10, 0, 0)),
                      'right': shifted(deepcopy(leaf), (10, 0, 0))}, provenance=provenance)
    assert len(dag['nodes']) == 3
    evaluator = ShapeEvaluator(dag)
    provenance['limitations'].append('changed outside')
    assert evaluator.dag['provenance']['limitations'] == ['captured fillets omitted']
    returned = evaluator.evaluate('left')
    returned.move(cq.Location((100, 0, 0)))
    assert evaluator.evaluate('left').Center().x == pytest.approx(-10)
    assert evaluator.evaluate('right').Center().x == pytest.approx(10)
    with pytest.raises(ValueError, match='choose a root'):
        evaluator.evaluate()


def test_empty_intersection_and_disjoint_union_are_valid_csg_results():
    a, b = node('box', [1, 1, 1]), shifted(node('box', [1, 1, 1]), (5, 0, 0))
    empty = evaluate(node('intersection', [], a, b))
    assert empty.Volume() == 0 and not empty.Solids()
    compound = evaluate(node('union', [], a, b))
    assert compound.Volume() == pytest.approx(16)
    assert len(compound.Solids()) == 2


@pytest.mark.parametrize('tree', [
    node('fillet', [2], node('box', [1, 1, 1])),
    node('box', [1, -2, 3]),
    node('box', [1, True, 3]),
    node('sphere', [float('nan')]),
    node('cone', [0, 0, 1]),
    node('cut', [], node('box', [1, 1, 1])),
    node('cylinder', [1, 2], node('sphere', [1])),
    node('polygon', [[[0, 0], [2, 2], [0, 2], [2, 0]], 1]),
    node('polygon', [[[0, 0], [1, 0], [1, 0], [0, 1]], 1]),
    node('transform', [2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1], node('sphere', [1])),
    node('transform', [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, .1, 1], node('sphere', [1])),
    node('radial_cylinders', [1, 1, 5, 0, 3.5]),
    node('linear_polygon', [[[0, 0], [1, 0], [0, 1]], 1, 0, 2, 5000]),
    node('belt_teeth', [1, 1, 2, 5, 100, 0, 3]),
    node('helix', [2, 2, 1, 4]),
    node('helix', [2, .2, 0, 4]),
    node('helix', [2, .2, 1, 1025]),
])
def test_invalid_or_unsupported_input_rejected_before_cad(tree):
    with pytest.raises(ValueError):
        from_tree(tree)


def test_cycles_and_tampered_node_identity_fail_closed():
    cyclic = node('union')
    cyclic['children'] = [cyclic, node('sphere', [1])]
    with pytest.raises(ValueError, match='cycle'):
        from_tree(cyclic)
    dag = from_tree(node('box', [1, 1, 1]))
    next(iter(dag['nodes'].values()))['args'][0] = 2
    with pytest.raises(ValueError, match='SHA256'):
        ShapeEvaluator(dag)


@pytest.mark.parametrize('reverse_order', [False, True])
def test_direct_dag_depth_limit_is_independent_of_serialized_node_order(reverse_order):
    dag = from_tree(node('box', [1, 1, 1]))
    child = dag['roots']['shape']
    for _ in range(1100):
        parent = dict(op='transform', args=np.eye(4).ravel().tolist(), children=[child])
        key = hashlib.sha256(json.dumps(parent, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
        dag['nodes'][key] = parent
        child = key
    dag['roots']['shape'] = child
    if reverse_order:
        dag['nodes'] = dict(reversed(dag['nodes'].items()))
    with pytest.raises(ValueError, match='maximum depth'):
        ShapeEvaluator(dag)


@pytest.mark.parametrize('kind', ['radial_cylinders', 'linear_polygon', 'belt_teeth'])
def test_optimized_array_operations_match_explicit_finite_unions(kind):
    if kind == 'radial_cylinders':
        tree = node(kind, [.4, 1, 3, math.pi / 6, 6])
        leaf = node('cylinder', [.4, 1])
        points = [(3 * math.cos(math.pi / 6 + k * math.pi / 3),
                   3 * math.sin(math.pi / 6 + k * math.pi / 3)) for k in range(6)]
    elif kind == 'linear_polygon':
        polygon = [[0, 0], [1, 0], [1, 1], [0, 1]]
        tree = node(kind, [polygon, .5, -4, 3, 4])
        leaf, points = node('polygon', [polygon, .5]), [(-4 + k * 3, 0) for k in range(4)]
    else:
        # Two semicircles of arc length 2, joined by straight spans of length 2.
        r = 2 / math.pi
        tree = node(kind, [.1, .5, r, 2, 1, 0, 8])
        leaf = node('cylinder', [.1, .5])
        points = [(-r, 0), (0, -r), (r, 0), (r, 1), (r, 2), (0, 2 + r), (-r, 2), (-r, 1)]
    explicit = shifted(leaf, (*points[0], 0))
    for point in points[1:]:
        explicit = node('union', [], explicit, shifted(leaf, (*point, 0)))
    assert_same_solid(evaluate(tree), evaluate(explicit))


def test_native_helix_starts_at_zero_and_keeps_fractional_turns_and_rigid_volume():
    radius, wire, pitch, turns = 4., .4, 1.2, 2.25
    tree = node('helix', [radius, wire, pitch, turns])
    shape = evaluate(tree)
    assert shape.isValid() and len(shape.Solids()) == 1
    box = shape.BoundingBox()
    assert box.zmin == pytest.approx(-wire, abs=.002)
    assert box.zmax == pytest.approx(pitch * turns + wire, abs=.002)
    # The source profile starts in XZ, slightly oblique to the helix tangent.
    # This is a finite Frenet sweep, not a centered cylinder or a torus.
    centerline_length = turns * math.sqrt((2 * math.pi * radius) ** 2 + pitch ** 2)
    assert shape.Volume() == pytest.approx(math.pi * wire ** 2 * centerline_length, rel=.002)
    mirrored = node('transform', [-1, 0, 0, 10, 0, 1, 0, 0, 0, 0, 1, 3, 0, 0, 0, 1], tree)
    moved = evaluate(mirrored)
    assert moved.Volume() == pytest.approx(shape.Volume(), rel=1e-9)
    assert moved.BoundingBox().zmin == pytest.approx(box.zmin + 3, abs=1e-8)


def legacy_primitive(row):
    """Independent pre-DAG native reference; do not route through the new API."""
    import cadquery as cq
    x, y, z = row['size']
    if row['kind'] == 'box':
        shape = cq.Workplane('XY').box(x, y, z).val()
    elif row['kind'] == 'cylinder':
        shape = cq.Workplane('XY').circle(x / 2).extrude(z).translate((0, 0, -z / 2)).val()
    else:
        shape = cq.Workplane('XY').sphere(x / 2).val()
    for angle, axis in zip(row['rotation'], [(1, 0, 0), (0, 1, 0), (0, 0, 1)]):
        shape = shape.rotate((0, 0, 0), axis, angle)
    return shape.translate(row['position'])


@pytest.mark.parametrize('kind', ['box', 'cylinder', 'sphere'])
def test_flat_plan_lowering_preserves_noncommuting_rotations_and_dimensions(kind):
    row = dict(kind=kind, size=[7, 5, 3], position=[4, 8, -2], rotation=[23, -31, 47])
    actual = ShapeEvaluator(flat_plan_part_to_dag(dict(body=row))).evaluate()
    assert_same_solid(actual, legacy_primitive(row))


def test_flat_plan_boolean_order_and_fillet_are_preserved():
    def primitive(size, position):
        return dict(kind='box', size=size, position=position, rotation=[0, 0, 0])
    spec = dict(body=primitive([4, 4, 4], [0, 0, 0]),
                unions=[primitive([4, 4, 4], [2, 0, 0])],
                cuts=[primitive([2, 2, 10], [2, 0, 0])], fillet=.5)
    expected = legacy_primitive(spec['body']).fuse(legacy_primitive(spec['unions'][0]))
    expected = expected.cut(legacy_primitive(spec['cuts'][0]))
    dag = flat_plan_part_to_dag(spec)
    assert_same_solid(ShapeEvaluator(dag).evaluate(), expected)
    spec['fillet'] = 0
    assert dag == flat_plan_part_to_dag(spec)


def test_repeated_subtrees_keep_strict_source_identity_after_cad_lowering():
    from mechanism_lab.constructed import part_shape
    from mechanism_lab.construction import compile_plan
    from mechanism_lab.motion_envelope import _verified_primitive_source
    # Shared cylinders in these real component parts serialize differently
    # from independent per-operand reconstruction, despite identical volume.
    specs = {p['name']: p for p in compile_plan()['parts']}
    for name in ('J1_carrier', 'tool_palm'):
        spec = specs[name]
        shape = part_shape(spec)
        assert _verified_primitive_source(shape, spec) is not None
        assert _verified_primitive_source(shape.translate((0, 0, .01)), spec) is None
