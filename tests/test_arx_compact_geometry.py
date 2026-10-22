"""Actual compact source geometry exercises the shared DAG and explicit frames."""
import ast
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

cq = pytest.importorskip('cadquery')
from OCP.gp import gp_Trsf
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform

from examples.engineering.arx_compact.design import (
    COMMIT, SOURCE, SOURCE_TO_CAD, body_frames, bounds, geometry_dag, load_source,
)
from mechanism_lab.shape_dag import ShapeEvaluator


def test_entire_source_lowers_without_a_second_generator():
    design, pose = load_source()
    dag = geometry_dag(design)
    assert len(dag['roots']) == len(design['parts']) == 278
    assert len(dag['nodes']) < 2000
    assert sum(node['op'] == 'helix' for node in dag['nodes'].values()) == 2
    assert set(p['body'] for p in design['parts']) <= body_frames(pose).keys()


def test_source_si_pose_and_local_millimetres_are_composed_in_order():
    design, pose = load_source()
    frame = body_frames(pose)['upper']
    row = np.asarray(pose['body_poses'][pose['body_names'].index('upper')])
    local_point = np.array([15., 23., -7.])
    expected_source = row[3:].reshape(3, 3) @ local_point + 1000 * row[:3]
    assert np.allclose(frame @ np.r_[local_point, 1], SOURCE_TO_CAD @ np.r_[expected_source, 1])
    evaluator = ShapeEvaluator(geometry_dag(design))
    source = evaluator.evaluate('0')
    world = evaluator.evaluate('0', frame=body_frames(pose)['fixed'])
    assert world.Volume() == pytest.approx(source.Volume(), rel=1e-10)


@pytest.mark.parametrize('invalid', ['missing', 'duplicate', 'nonrigid', 'nan'])
def test_invalid_body_frames_fail_closed(invalid):
    _, pose = load_source()
    pose = copy.deepcopy(pose)
    if invalid == 'missing':
        pose['body_names'][0] = 'unrecognized_fixed'
    elif invalid == 'duplicate':
        pose['body_names'][0] = pose['body_names'][1]
    elif invalid == 'nonrigid':
        pose['body_poses'][0][3] = 2
    else:
        pose['body_poses'][0][0] = float('nan')
    with pytest.raises(ValueError):
        body_frames(pose)


@pytest.fixture(scope='module')
def original_native_build():
    """Run only the hash-pinned original builder, without its file-writing loop."""
    fixture = Path(__file__).parent / 'fixtures/arx_compact/build_meshes.py'
    raw = fixture.read_bytes()
    assert COMMIT == '0003382bd6a2e96f3bed065003efec35001982f7'
    expected_sha256 = '1b2e4f087634f8ff41ad450195a2a35ef3ef6445d1bc25efbad9c797f7fb2e50'
    assert hashlib.sha256(raw).hexdigest() == expected_sha256 == SOURCE['native_builder_sha256']
    original = ast.parse(raw, filename=str(fixture))
    functions = [node for node in original.body if isinstance(node, ast.FunctionDef) and node.name == 'build']
    assert len(functions) == 1
    assert (functions[0].lineno, functions[0].end_lineno) == (10, 27)
    namespace = dict(json=json, np=np, cq=cq, gp_Trsf=gp_Trsf,
                     BRepBuilderAPI_Transform=BRepBuilderAPI_Transform, cache={})
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(fixture), 'exec'), namespace)
    return namespace['build']


@pytest.mark.parametrize('part_id,z_range', [
    ('263', (33., 92.)),
    ('269', (-67., -33.)),
])
def test_actual_springs_preserve_independently_measured_source_defects(part_id, z_range, original_native_build):
    # The independent, unchanged source and shared DAG run on the same kernel.
    # OCP 7.8/7.9 sweep integration differs, so a previous Windows measurement
    # is evidence for that run, not a portable mathematical volume constant.
    design, _ = load_source()
    row = next(p for p in design['parts'] if str(p['id']) == part_id)
    reference = original_native_build(copy.deepcopy(row['shape']))
    shape = ShapeEvaluator(geometry_dag(design)).evaluate(part_id)
    assert shape.isValid() and reference.isValid()
    assert len(shape.Solids()) == len(reference.Solids()) == 2
    assert shape.Volume() == pytest.approx(reference.Volume(), rel=1e-8)
    assert np.asarray(bounds(shape)) == pytest.approx(np.asarray(bounds(reference)), abs=1e-5)
    for actual_solid, source_solid in zip(sorted(shape.Solids(), key=lambda s: s.Volume()),
                                         sorted(reference.Solids(), key=lambda s: s.Volume())):
        assert actual_solid.isValid() and source_solid.isValid()
        assert actual_solid.Volume() == pytest.approx(source_solid.Volume(), rel=1e-8)
        assert actual_solid.Center().toTuple() == pytest.approx(source_solid.Center().toTuple(), abs=1e-5)
        assert np.asarray(bounds(actual_solid)) == pytest.approx(np.asarray(bounds(source_solid)), abs=1e-5)
    bb = bounds(shape)
    assert (bb[0][2], bb[1][2]) == pytest.approx(z_range, abs=1e-5)
    if part_id == '263':
        assert bb[1][2] - row['bounds_mm'][1][2] == pytest.approx(18., abs=1e-5)
