"""One source-bound motion path across periods, renamed actuators and backends."""
import copy

import pytest

from mechanism_lab.construction import compile_mechanism, model_digest, recipe_digest
from mechanism_lab.mechanism import load_example
from mechanism_lab.motion import MotionTask


def explicit_source(period=12):
    source = load_example('inspection_head')
    points = [{name: 0. for name in source['actuators']} for _ in range(2)]
    points[0]['J2_motor'], points[1]['J2_motor'] = -2., 4.
    points[0]['left_linear'], points[1]['left_linear'] = 1., 5.
    source['motion'] = dict(period_s=period, waypoints=points)
    return source


def test_default_task_preserves_reference_eight_second_cycle():
    plan = compile_mechanism(load_example())
    task = MotionTask.from_dict(plan['construction']['motion'])
    legacy = MotionTask.from_joints(plan['joints'])
    assert task.period_s == 8
    assert task.sample(0)['values'] == {name: 0 for name in task.waypoints[0]}
    assert task.sample(4)['values']['J1_output'] == pytest.approx(8.4)
    assert task.sample(4)['values']['J2_output'] == pytest.approx(-8.4)
    assert task.sample(4)['values']['jaw_left_travel'] == pytest.approx(7)
    assert task.sample(8)['values'] == pytest.approx(task.sample(0)['values'])
    for time in (0, .37, 2, 4, 7.81, 9.2):
        assert task.sample(time) == legacy.sample(time)


def test_explicit_waypoints_resolve_source_actuators_and_preserve_nonzero_start():
    plan = compile_mechanism(explicit_source())
    task = MotionTask.from_dict(plan['construction']['motion'])
    assert plan['construction']['motion']['actuator_bindings']['J2_motor'] == 'J2_output'
    assert task.sample(0)['values']['J2_output'] == -2
    assert task.sample(6)['values']['J2_output'] == 4
    assert task.sample(12)['values']['J2_output'] == pytest.approx(-2)
    assert task.sample(6)['values']['jaw_left_travel'] == 5


def test_period_changes_time_parameterization_without_changing_path_or_geometry():
    fast = compile_mechanism(explicit_source(12))
    slow = compile_mechanism(explicit_source(24))
    a, b = [MotionTask.from_dict(plan['construction']['motion']) for plan in (fast, slow)]
    assert a.waypoints == b.waypoints
    assert recipe_digest(fast) == recipe_digest(slow)
    assert model_digest(fast) != model_digest(slow)
    for time in (.3, 2, 4.7, 8.9):
        x, y = a.sample(time), b.sample(2*time)
        assert x['values'] == pytest.approx(y['values'])
        assert x['velocities'] == pytest.approx({k: 2*v for k, v in y['velocities'].items()})
        assert x['accelerations'] == pytest.approx({k: 4*v for k, v in y['accelerations'].items()})
    # A consumer stopping this run after 4s has not completed the 12s task.
    assert a.sample(4)['values']['J2_output'] != pytest.approx(a.sample(0)['values']['J2_output'])


def test_analytic_derivatives_match_position_and_velocity_differences():
    plan = compile_mechanism(explicit_source())
    task = MotionTask.from_dict(plan['construction']['motion'])
    time, step = 2.37, 1e-4
    left, middle, right = [task.sample(t) for t in (time-step, time, time+step)]
    for name in middle['values']:
        velocity = (right['values'][name]-left['values'][name])/(2*step)
        acceleration = (right['velocities'][name]-left['velocities'][name])/(2*step)
        assert middle['velocities'][name] == pytest.approx(velocity, abs=1e-8)
        assert middle['accelerations'][name] == pytest.approx(acceleration, abs=1e-8)
    rates = task.peak_rates()
    assert rates['J2_output']['speed'] == pytest.approx(abs(task.sample(3)['velocities']['J2_output']))
    assert rates['J2_output']['acceleration'] == pytest.approx(abs(task.sample(0)['accelerations']['J2_output']))


@pytest.mark.parametrize('fault', ['missing_control', 'extra_control', 'too_many_points', 'joint_limit', 'period', 'rate'])
def test_invalid_or_unachievable_motion_is_rejected_before_cad(fault):
    source = explicit_source()
    if fault == 'missing_control':
        source['motion']['waypoints'][0].pop('J2_motor')
    elif fault == 'extra_control':
        source['motion']['waypoints'][0]['a_generated_joint_name'] = 0
    elif fault == 'too_many_points':
        source['motion']['waypoints'].append(copy.deepcopy(source['motion']['waypoints'][0]))
    elif fault == 'joint_limit':
        source['motion']['waypoints'][1]['J2_motor'] = 30
    elif fault == 'period':
        source['motion']['period_s'] = True
    else:
        source['motion']['period_s'] = .2
    with pytest.raises(ValueError):
        compile_mechanism(source)


def test_omitted_motion_uses_the_same_normalized_source_default():
    source = load_example()
    explicit = compile_mechanism(source)
    source.pop('motion')
    omitted = compile_mechanism(source)
    assert omitted['construction']['mechanism'] == explicit['construction']['mechanism']
    assert model_digest(omitted) == model_digest(explicit)
