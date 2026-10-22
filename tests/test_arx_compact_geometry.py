"""Actual compact source geometry exercises the shared DAG and explicit frames."""
import copy

import numpy as np
import pytest

pytest.importorskip('cadquery')

from examples.engineering.arx_compact.design import (
    SOURCE_TO_CAD, body_frames, bounds, geometry_dag, load_source,
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


@pytest.mark.parametrize('part_id,volume,z_range', [
    ('263', 18777.547518409418, (33., 92.)),
    ('269', 5021.352834691703, (-67., -33.)),
])
def test_actual_springs_preserve_independently_measured_source_defects(part_id, volume, z_range):
    # Reference measured by AST-extracting only pinned source build_meshes.py's
    # native build() function. It independently has two solids for both rows.
    design, _ = load_source()
    shape = ShapeEvaluator(geometry_dag(design)).evaluate(part_id)
    assert shape.isValid()
    assert len(shape.Solids()) == 2
    assert shape.Volume() == pytest.approx(volume, rel=1e-8)
    bb = bounds(shape)
    assert (bb[0][2], bb[1][2]) == pytest.approx(z_range, abs=1e-5)
    if part_id == '263':
        row = next(p for p in design['parts'] if str(p['id']) == part_id)
        assert bb[1][2] - row['bounds_mm'][1][2] == pytest.approx(18., abs=1e-5)
