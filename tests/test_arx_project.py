"""The P5 client binds one source task, explicit posture policy and clock."""
from copy import deepcopy
import sys

import numpy as np
import pytest

from mechanism_lab.engineering_source import load_project
from mechanism_lab.reference_clock import PhaseClock


@pytest.fixture(scope='module')
def compiled():
    project, identity = load_project('arx_belt')
    physical, control, parameters = project.compile_physics(project.specification)
    return project, identity, physical, control, parameters


def test_default_clock_binds_authored_posture_and_derivatives(compiled):
    project, identity, physical, control, parameters = compiled
    clock = PhaseClock.from_source(parameters['reference_clock'])
    source_reference = sys.modules[project.reference.__module__].source_reference
    assert project.duration_s == clock.physical_period_s == 18.05
    assert parameters['reference_clock_validation']['passed']
    assert parameters['operating_policy'] == {'lateral_tilt_scale': 0.0}
    assert 'data/default-task.json' in identity
    assert clock.physical_time(3.2) == pytest.approx(6.1)
    assert clock.physical_time(7.2) == pytest.approx(12.1)
    assert clock.physical_time(6.0) == pytest.approx(9.7)
    assert clock.physical_time(6.65) == pytest.approx(11.0)
    for source_time in (.5, 2., 3.2, 4.9, 7.2, 8.2, 9.9):
        time = clock.physical_time(source_time)
        expected = source_reference(source_time, parameters, lateral_tilt_scale=0.0)
        actual = project.reference(time, parameters)
        np.testing.assert_allclose(actual['q'], expected['q'], atol=1e-12, rtol=0)
        np.testing.assert_allclose(actual['qd'], np.asarray(expected['qd'])*clock.rate(time), atol=1e-8, rtol=0)
        np.testing.assert_allclose(actual['qdd'], np.asarray(expected['qdd'])*clock.rate(time)**2, atol=1e-6, rtol=0)
        assert actual['gap'] == expected['gap'] and actual['stage'] == expected['stage']


def test_policy_preserves_tcp_targets_and_source_roll_in_actual_fk(compiled):
    """Independent MJCF kinematics: same TCP, changed tool direction, same roll."""
    mj = pytest.importorskip('mujoco')
    project, _, physical, _, parameters = compiled
    source_reference = sys.modules[project.reference.__module__].source_reference
    model = mj.MjModel.from_xml_string(physical.xml())
    data = mj.MjData(model)
    addresses = [int(model.joint(f'j{index}').qposadr[0]) for index in range(1, 7)]
    clock = PhaseClock.from_source(parameters['reference_clock'])
    limits = np.asarray(parameters['integration']['joint_soft_limits_rad'])
    observed = []
    for source_time in np.linspace(0, 10, 501):
        authored = project.reference(clock.physical_time(float(source_time)), parameters)
        original = source_reference(float(source_time), parameters)
        mj.mj_resetDataKeyframe(model, data, 0)
        data.qpos[addresses] = original['q']
        mj.mj_forward(model, data)
        original_tcp = data.site('tcp').xpos.copy()
        data.qpos[addresses] = authored['q']
        mj.mj_forward(model, data)
        np.testing.assert_allclose(data.site('tcp').xpos, original_tcp, rtol=0, atol=2e-10)
        observed.append(authored['q'])
        direction = data.body('tool').xmat.reshape(3, 3)[:, 1]
        assert abs(direction[2]) < 2e-10
        assert direction[1] < -.98  # Preserved forward tilt, no sideways tilt.
        phase = source_time % 5
        progress = np.clip((phase-1.65)/1.1, 0, 1)
        travel = 6*progress**5-15*progress**4+10*progress**3
        hump = 16*travel**2*(1-travel)**2
        np.testing.assert_allclose(direction[0]/-direction[1], .15*hump, atol=2e-10, rtol=0)
        reference_x = np.array([1., 0., 0.])-direction*direction[0]
        reference_x /= np.linalg.norm(reference_x)
        source_roll = (.5 if source_time % 10 < 5 else -.5)*hump
        expected_x = np.cos(source_roll)*reference_x+np.sin(source_roll)*np.cross(direction, reference_x)
        np.testing.assert_allclose(data.body('tool').xmat.reshape(3, 3)[:, 0], expected_x, atol=2e-10, rtol=0)
    observed = np.asarray(observed)
    assert np.all(observed > limits[:, 0]) and np.all(observed < limits[:, 1])
    for source_time, target in ((.8, 'A'), (3.5, 'B'), (5.8, 'B'), (8.5, 'A')):
        data.qpos[addresses] = project.reference(clock.physical_time(source_time), parameters)['q']
        mj.mj_forward(model, data)
        np.testing.assert_allclose(data.site('tcp').xpos, parameters['workpiece_targets_m'][target], atol=2e-10, rtol=0)
    midpoint = (np.asarray(parameters['workpiece_targets_m']['A'])+parameters['workpiece_targets_m']['B'])/2
    midpoint[1] = 1.08*parameters['scene_units_to_meters']
    for source_time in (2.2, 7.2):
        data.qpos[addresses] = project.reference(clock.physical_time(source_time), parameters)['q']
        mj.mj_forward(model, data)
        np.testing.assert_allclose(data.site('tcp').xpos, midpoint, atol=2e-10, rtol=0)


def test_missing_policy_preserves_source_default_and_policy_enters_identity():
    project, _ = load_project('arx_belt')
    spec = deepcopy(project.specification)
    spec['task_timing'].pop('operating_policy')
    _, _, parameters = project.compile_physics(spec)
    assert parameters['operating_policy'] == {'lateral_tilt_scale': 1.0}
    module = sys.modules[project.reference.__module__]
    clock = PhaseClock.from_source(parameters['reference_clock'])
    for source_time in (2.2, 7.2):
        expected = module.source_reference(source_time, parameters)
        np.testing.assert_allclose(project.reference(clock.physical_time(source_time), parameters)['q'], expected['q'], atol=1e-12, rtol=0)
    parameters['operating_policy']['lateral_tilt_scale'] = 0.0
    with pytest.raises(ValueError, match='unchanged bound'):
        project.reference(clock.physical_time(7.2), parameters)


@pytest.mark.parametrize('coefficient', [True, -.01, 1.01, float('nan'), float('inf'), '0'])
def test_reference_rejects_invalid_policy(compiled, coefficient):
    project, _, _, _, parameters = compiled
    source_reference = sys.modules[project.reference.__module__].source_reference
    with pytest.raises(ValueError, match='lateral_tilt_scale'):
        source_reference(2.2, parameters, lateral_tilt_scale=coefficient)


def test_project_rejects_unknown_policy_fields():
    project, _ = load_project('arx_belt')
    spec = deepcopy(project.specification)
    spec['task_timing']['operating_policy']['unapplied_roll'] = 0
    with pytest.raises(ValueError, match='only lateral_tilt_scale'):
        project.compile_physics(spec)


def test_bound_reference_rejects_replaced_or_mutated_parameters(compiled):
    project, _, _, _, parameters = compiled
    with pytest.raises(ValueError, match='unchanged bound'):
        project.reference(6.1, deepcopy(parameters))
    original = parameters['tool_length_m']
    try:
        parameters['tool_length_m'] *= 2
        with pytest.raises(ValueError, match='unchanged bound'):
            project.reference(6.1, parameters)
    finally:
        parameters['tool_length_m'] = original
    assert project.reference(6.1, parameters)['stage'] == 4
