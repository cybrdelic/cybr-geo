"""Cheap orchestration tests; CAD and MuJoCo have their own native tests."""
from copy import deepcopy
import gzip
import json
from types import SimpleNamespace

import numpy as np
import pytest

from mechanism_lab.engineering_pipeline import evaluate_plan
from mechanism_lab.motion import MotionTask
from mechanism_lab.motion_envelope import SCHEMA, plan_identity


def plan_fixture():
    def joint(name, child, drive=None):
        return dict(name=name, child=child, parent=None, type='prismatic', origin=[0, 0, 0], axis=[1, 0, 0],
                    min=0, max=12 if drive else 6, value=0, drive=drive)
    return dict(name='pipeline_fixture', units='mm', parts=[{'name': 'source'}, {'name': 'follower'}],
                joints=[joint('motor', 'source'), joint('driven', 'follower', {'joint': 'motor', 'ratio': 2, 'offset': 0})],
                construction=dict(recipe_sha256='geometry', mechanism_sha256='source', model_sha256='model',
                    motion=dict(schema='cybrgeo-motion/1', period_s=2., interpolation='cosine_out_and_back',
                                waypoints=[{'motor': 0.}, {'motor': 4.}], actuator_bindings={'actuator': 'motor'})))


def assembly_fixture(plan):
    return SimpleNamespace(parts=[SimpleNamespace(name=p['name'], cad=object()) for p in plan['parts']],
                           metadata={'construction_recipe': deepcopy(plan), 'motion_clearance': {'stale': True}})


def simulation_fixture(plan, *, duration=2.):
    task = MotionTask.from_dict(plan['construction']['motion'])
    times = [0., duration/2, duration]
    requested = [task.sample(t)['values']['motor'] for t in times]
    error = [-.001, .002, -.001]
    residual = [.003, -.004, .003]
    actual = [[q+e, 2*(q+e)+r] for q, e, r in zip(requested, error, residual)]
    limits = {name: max(0., max(-row[i] for row in actual), max(row[i]-12 for row in actual))
              for i, name in enumerate(('motor', 'driven'))}
    return dict(backend='MuJoCo', recipe_sha256='geometry', mechanism_sha256='source', model_sha256='model',
                integration_steps=2, seconds=duration, timestep_s=duration/2,
                tracking_passed=True, operating_limits_passed=True,
                metrics={'motor': {'unit': 'mm', 'max_tracking_error': max(abs(v) for v in error)}},
                maximum_joint_limit_violation_native=max(limits.values()), joint_limit_violations_native=limits,
                maximum_drive_constraint_error_native=max(abs(v) for v in residual), drive_constraint_errors_native={'driven': .004},
                integration_state_trace=dict(joint_names=['motor', 'driven'], control_names=['motor'], times_s=times,
                    joint_values=actual, requested_control_values=[[q] for q in requested],
                    sampling='every_integration_step_including_initial_and_final'),
                samples=[dict(time_s=t, joint_values=dict(zip(('motor', 'driven'), row))) for t, row in zip(times, actual)])


def mocks(monkeypatch, plan, simulation, *, clearance_passed=True):
    import mechanism_lab.constructed as constructed
    import mechanism_lab.dynamics as dynamics
    import mechanism_lab.motion_envelope as envelope
    calls = []
    assembly = assembly_fixture(plan)

    def build(actual_plan):
        calls.append(('build', actual_plan))
        return assembly

    def simulate(actual_assembly, **kwargs):
        calls.append(('simulate', kwargs))
        assert actual_assembly is assembly
        return deepcopy(simulation)

    def certify(actual_assembly, **kwargs):
        calls.append(('certify', kwargs))
        assert actual_assembly is assembly
        return dict(schema=SCHEMA, passed=clearance_passed, plan_sha256=plan_identity(plan),
                    interpolation='piecewise_linear_in_independent_joint_coordinates',
                    waypoints=kwargs['waypoints'], joint_error_bounds=kwargs['joint_error_bounds'],
                    drive_error_bounds=kwargs['drive_error_bounds'])

    monkeypatch.setattr(constructed, 'build_from_plan', build)
    monkeypatch.setattr(dynamics, 'simulate', simulate)
    monkeypatch.setattr(dynamics, 'plot_simulation', lambda *args: calls.append(('plot', None)))
    monkeypatch.setattr(envelope, 'certify_path', certify)
    return calls, assembly


def test_simulates_then_certifies_padded_tracking_and_drive_tube_once(monkeypatch, tmp_path):
    plan = plan_fixture()
    calls, assembly = mocks(monkeypatch, plan, simulation_fixture(plan))
    returned, report = evaluate_plan(plan, tmp_path)
    assert returned is assembly
    assert [name for name, _ in calls] == ['build', 'simulate', 'plot', 'certify']
    kwargs = next(value for name, value in calls if name == 'certify')
    assert kwargs['waypoints'] == plan['construction']['motion']['waypoints']
    assert kwargs['joint_error_bounds']['motor'] > .002*1.05+1e-6
    assert kwargs['drive_error_bounds']['driven'] > .004*1.05+1e-6
    assert report['passed'] and report['motion_authorization']['allowed']
    assert report['simulation_clearance']['integration_states_checked'] == 3
    assert report['simulation_clearance']['output_states_checked'] == 3
    assert report['simulation_clearance']['between_step_containment'] == 'not_established'
    assert 'integration_state_trace' not in report['simulation']
    assert 'integration_state_trace' not in assembly.metadata['simulation_summary']
    assert assembly.metadata['motion_clearance'] is report['clearance']
    assert assembly.metadata['qualification']['dynamics'] == 'actual MuJoCo run passed'
    assert assembly.metadata['qualification']['hardware'] == 'not qualified'
    assert 'joint_values' not in json.dumps(json.loads((tmp_path/'engineering.json').read_text()))
    with gzip.open(tmp_path/report['integration_trace_artifact']['path'], 'rt') as stream:
        trace = json.load(stream)
    assert len(trace['times_s']) == 3
    assert report['simulation']['integration_trace_artifact']['path'] == 'integration-state-trace.json.gz'


@pytest.mark.parametrize('flag,code', [('tracking_passed', 'tracking_limit'), ('operating_limits_passed', 'operating_limits')])
def test_dynamics_failure_cannot_authorize_a_clear_path(monkeypatch, tmp_path, flag, code):
    plan = plan_fixture()
    simulation = simulation_fixture(plan)
    simulation[flag] = False
    calls, assembly = mocks(monkeypatch, plan, simulation)
    _, report = evaluate_plan(plan, tmp_path)
    assert report['clearance']['passed']
    assert not report['passed'] and not report['motion_authorization']['allowed']
    assert any(row.get('code') == code for row in report['errors'])
    assert 'motion_clearance' not in assembly.metadata
    assert not assembly.metadata['qualification']['dynamics_run_passed']
    assert sum(name == 'certify' for name, _ in calls) == 1


@pytest.mark.parametrize('corruption', ['missing_trace', 'tracking_maximum', 'drive_maximum', 'missing_step', 'wrong_period', 'wrong_identity'])
def test_incomplete_or_inconsistent_simulation_evidence_is_not_certified(monkeypatch, tmp_path, corruption):
    plan = plan_fixture()
    simulation = simulation_fixture(plan)
    if corruption == 'missing_trace':
        simulation.pop('integration_state_trace')
    elif corruption == 'tracking_maximum':
        simulation['metrics']['motor']['max_tracking_error'] = .0001
    elif corruption == 'drive_maximum':
        simulation['drive_constraint_errors_native']['driven'] = .0001
    elif corruption == 'missing_step':
        simulation['integration_state_trace']['times_s'].pop()
    elif corruption == 'wrong_period':
        simulation['integration_state_trace']['requested_control_values'][1] = [2.]
    else:
        simulation['model_sha256'] = 'other-source'
    calls, assembly = mocks(monkeypatch, plan, simulation)
    _, report = evaluate_plan(plan, tmp_path)
    assert not report['passed']
    assert report['clearance'] is None
    assert not any(name == 'certify' for name, _ in calls)
    assert 'motion_clearance' not in assembly.metadata
    assert any(row.get('code') == 'invalid_simulation_evidence' for row in report['errors'])


def test_clearance_failure_retains_simulation_but_denies_motion(monkeypatch, tmp_path):
    plan = plan_fixture()
    calls, assembly = mocks(monkeypatch, plan, simulation_fixture(plan), clearance_passed=False)
    _, report = evaluate_plan(plan, tmp_path)
    assert report['simulation'] is not None and not report['clearance']['passed']
    assert report['simulation_clearance'] is None
    assert not report['passed'] and 'motion_clearance' not in assembly.metadata


def test_reuses_matching_analytic_assembly_and_rejects_stale_or_mesh_only(monkeypatch, tmp_path):
    plan = plan_fixture()
    calls, assembly = mocks(monkeypatch, plan, simulation_fixture(plan))
    _, report = evaluate_plan(plan, tmp_path, assembly=assembly)
    assert report['passed'] and not any(name == 'build' for name, _ in calls)
    assembly.metadata['construction_recipe']['name'] = 'changed'
    with pytest.raises(ValueError, match='exact engineering plan'):
        evaluate_plan(plan, tmp_path, assembly=assembly)
    assembly.metadata['construction_recipe'] = deepcopy(plan)
    assembly.parts[0].cad = None
    with pytest.raises(ValueError, match='mesh-only'):
        evaluate_plan(plan, tmp_path, assembly=assembly)


def test_run_duration_does_not_change_the_source_motion_period(monkeypatch, tmp_path):
    plan = plan_fixture()
    calls, _ = mocks(monkeypatch, plan, simulation_fixture(plan, duration=1.))
    _, report = evaluate_plan(plan, tmp_path, seconds=1.)
    assert report['passed']
    assert next(kwargs for name, kwargs in calls if name == 'simulate')['seconds'] == 1.
    assert next(kwargs for name, kwargs in calls if name == 'certify')['waypoints'] == plan['construction']['motion']['waypoints']


def test_output_sample_must_match_an_actual_integration_state(monkeypatch, tmp_path):
    plan = plan_fixture()
    simulation = simulation_fixture(plan)
    simulation['samples'][1]['joint_values']['driven'] = 7
    _, assembly = mocks(monkeypatch, plan, simulation)
    _, report = evaluate_plan(plan, tmp_path)
    assert not report['passed']
    assert any('Output sample disagrees' in row['message'] for row in report['errors'])
    assert 'motion_clearance' not in assembly.metadata


def test_batch_membership_covers_full_trace_without_changing_command_policy():
    from mechanism_lab.motion_envelope import CertifiedTube, authorized_command
    plan = plan_fixture()
    certificate = dict(schema=SCHEMA, passed=True, plan_sha256=plan_identity(plan),
        interpolation='piecewise_linear_in_independent_joint_coordinates',
        waypoints=plan['construction']['motion']['waypoints'], joint_error_bounds={'motor': .01}, drive_error_bounds={'driven': .02})
    states = np.array([[-.005, -.01], [2, 4.01], [2, 5]])
    assert CertifiedTube(plan, certificate).contains_states(['motor', 'driven'], states).tolist() == [True, True, False]
    with pytest.raises(ValueError, match='outside'):
        authorized_command(plan, certificate, {'motor': -.005})


def test_legacy_alias_and_v1_sources_roundtrip_through_the_canonical_mechanism():
    from mechanism_lab.construction import compile_mechanism
    from mechanism_lab.engineering_pipeline import compile_source
    plan, name = compile_source('compact_arm')
    assert name == 'compact_arm'
    canonical = compile_mechanism(plan['construction']['mechanism'])
    assert plan_identity(plan) == plan_identity(canonical)
    legacy, legacy_name = compile_source({'schema': 'cybrgeo-mechanism/1', 'name': 'legacy-example',
                                          'architecture': 'compact_arm', 'parameters': {}})
    assert legacy_name == 'legacy-example'
    assert plan_identity(legacy) == plan_identity(canonical)
