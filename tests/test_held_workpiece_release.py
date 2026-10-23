"""Release integration exercises recorded fault frames, not claimed pass flags."""
from copy import deepcopy
import gzip
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest

from mechanism_lab.held_workpiece_fault import held_free_workpieces
from mechanism_lab.physical_model import digest
from mechanism_lab.qualification import (
    QualificationError, _fault_trace_observation,
    _verify_held_workpiece_fault_cases,
)


SOURCE = Path(__file__).resolve().parents[1] / 'examples/engineering/arx_belt/data/P2-baseline.xml'
SOURCE_FILES = {'data/P2-baseline.xml': 'f' * 64}  # Test-only source digest.
MODEL_SHA = 'a' * 64
CONTROL_SHA = 'b' * 64
CASE_ID = 'puck_power_loss'
RUNTIME = {'test_only': True}
_NOT_GIVEN = object()


def _fixture(tmp_path):
    root = ET.parse(SOURCE).getroot()
    held = held_free_workpieces(root)['puck']
    case = dict(workpiece_body='puck', workpiece_mass_kg=held['workpiece_mass_kg'],
                contact_pairs=held['contact_pairs'], actuators_off=held['actuators_off'],
                gravity_vector_m_s2=[0., -9.80665, 0.],
                maximum_mass_deviation_kg=.001, minimum_observation_s=.2,
                maximum_downward_drop_m=.001,
                maximum_residual_current_A=.001,
                maximum_residual_torque_Nm=.001)
    profile = dict(schema='cybrgeo-held-workpiece-fault-profile/1',
                   source_sha256=digest(SOURCE_FILES), model_sha256=MODEL_SHA,
                   cases={CASE_ID: case})
    project, run, evidence, authority = [tmp_path/name for name in
                                          ('project', 'run', 'evidence', 'authority')]
    for path in (project, run, evidence, authority):
        path.mkdir()
    profile_path = authority/'motor-off-profile.json'
    profile_path.write_text(json.dumps(profile), encoding='utf-8')
    policy_path = authority/'trust.json'
    policy_path.write_text(json.dumps(dict(schema='cybrgeo-trust-policy/1',
        issuers={'test_only': dict(public_key_base64='TEST-ONLY',
                                   allowed_kinds=['fault_test'])})), encoding='utf-8')
    control = dict(drives=[dict(actuator=f'motor{index}') for index in range(1, 7)],
                   rack=dict(actuator='gripper_motor'))
    report = dict(model_sha256=MODEL_SHA, control_sha256=CONTROL_SHA,
                  simulation=dict(runtime_implementation=RUNTIME))
    return (SimpleNamespace(root=root), control, run, report,
            {'motor_off_fault_cases': deepcopy(profile['cases'])},
            policy_path, profile_path, project, evidence)


def _frame(time_s, *, pre=False, height=.1, powered=False, other_support=False):
    contacts = ([dict(geoms=['pad_left', 'puck'], normal_N=2.),
                 dict(geoms=['pad_right', 'puck'], normal_N=2.)]
                if pre else [])
    if other_support:
        contacts.append(dict(geoms=['puck', 'nest'], normal_N=2.))
    state = dict(current_A=.1 if powered else 0., torque_Nm=.1 if powered else 0.)
    return dict(time_s=time_s, mode='RUN' if pre else 'EMERGENCY',
                reason='' if pre else 'power-loss', contacts=contacts,
                bodies={'puck': dict(position_m=[0., height, 0.])},
                motors=[deepcopy(state) for _ in range(6)], rack_motor=deepcopy(state))


def _trace():
    return dict(schema='cybrgeo-causal-simulation/1', model_sha256=MODEL_SHA,
                control_sha256=CONTROL_SHA, runtime_implementation=RUNTIME,
                reference_identity='explicit source record',
                reference_source=dict(source_files=SOURCE_FILES),
                options=dict(power=True, motorOffTime=.3), final_options=dict(power=False),
                start_time_s=0., timestep_s=.001,
                events=[dict(t=.3, mode='EMERGENCY', reason='power-loss')],
                frames=[_frame(0., pre=True), _frame(.21, pre=True),
                        _frame(.25, pre=True), _frame(.31), _frame(.4), _frame(.5)],
                metrics=dict(free_bodies={'puck': dict(max_imposed_generalized_force=0.,
                                                       max_applied_wrench_component=0.)}))


def _check(inputs, *, fault_profile=_NOT_GIVEN):
    model, control, run, report, plan, policy, profile_path, project, evidence = inputs
    return _verify_held_workpiece_fault_cases(model, control, run, report,
        SOURCE_FILES, plan, policy,
        profile_path if fault_profile is _NOT_GIVEN else fault_profile,
        project_directory=project, evidence_directory=evidence)


def _write_trace(inputs, trace):
    run = inputs[2]
    path = run/'faults'/CASE_ID/'trace.json.gz'
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(gzip.compress(json.dumps(trace).encode('utf-8'), mtime=0))
    return path


def test_arx_free_puck_needs_external_profile_and_actual_recorded_fault(tmp_path):
    inputs = _fixture(tmp_path)
    with pytest.raises(QualificationError, match='acceptance profile'):
        _check(inputs, fault_profile=None)
    with pytest.raises(QualificationError, match='recorded simulation trace'):
        _check(inputs)
    path = _write_trace(inputs, _trace())
    verified = _check(inputs)
    assert verified['trace_sha256'] == {CASE_ID: hashlib.sha256(path.read_bytes()).hexdigest()}
    assert verified['requirements']['cases'][CASE_ID]['workpiece_mass_kg'] == 0.0819390195909


@pytest.mark.parametrize('fault, expected', [
    ('stale_source_profile', 'stale for source/model'),
    ('stale_model_profile', 'stale for source/model'),
    ('missing_plan', 'Source-planned motor-off cases must be an object'),
    ('profile_under_source', 'supplied outside source'),
    ('stale_model_trace', 'current source, model or runtime'),
    ('no_fault_event', 'matching power-loss event'),
    ('powered_drive', 'still commands or carries motor output'),
    ('omitted_motor', 'lacks every motor state'),
    ('supported_by_nest', 'not initially held only by source contacts'),
    ('post_fault_nest', 'unplanned workpiece support contact'),
    ('second_workpiece_geom', 'not initially held only by source contacts'),
    ('unnamed_workpiece_geom', 'unnamed or absent workpiece geometry'),
    ('no_pad_contact', 'not initially held only by source contacts'),
    ('short_observation', 'does not cover its declared observation'),
    ('excess_drop', 'exceeds source-bound limit'),
])
def test_release_trace_cannot_substitute_claimed_or_stale_fault(fault, expected, tmp_path):
    inputs = _fixture(tmp_path)
    model, control, run, report, plan, policy, profile_path, project, evidence = inputs
    profile = json.loads(profile_path.read_text(encoding='utf-8'))
    trace = _trace()
    if fault == 'stale_source_profile': profile['source_sha256'] = 'c'*64
    elif fault == 'stale_model_profile': profile['model_sha256'] = 'c'*64
    elif fault == 'missing_plan': plan.pop('motor_off_fault_cases')
    elif fault == 'profile_under_source':
        profile_path = project/'motor-off-profile.json'
        inputs = (model, control, run, report, plan, policy, profile_path, project, evidence)
    elif fault == 'stale_model_trace': trace['model_sha256'] = 'c'*64
    elif fault == 'no_fault_event': trace['events'] = []
    elif fault == 'powered_drive': trace['frames'][-1]['rack_motor']['current_A'] = .1
    elif fault == 'omitted_motor': trace['frames'][-1]['motors'].pop()
    elif fault == 'supported_by_nest': trace['frames'][2]['contacts'].append(
        dict(geoms=['puck', 'nest'], normal_N=2.))
    elif fault == 'post_fault_nest': trace['frames'][-1]['contacts'].append(
        dict(geoms=['puck', 'nest'], normal_N=2.))
    elif fault == 'second_workpiece_geom':
        ET.SubElement(model.root.find(".//body[@name='puck']"), 'geom',
                      name='puck_side', type='sphere', size='.005')
        trace['frames'][2]['contacts'].append(
            dict(geoms=['puck_side', 'nest'], normal_N=2.))
    elif fault == 'unnamed_workpiece_geom':
        ET.SubElement(model.root.find(".//body[@name='puck']"), 'geom',
                      type='sphere', size='.005')
    elif fault == 'no_pad_contact': trace['frames'][2]['contacts'] = []
    elif fault == 'short_observation': trace['frames'][-1]['time_s'] = .45
    elif fault == 'excess_drop': trace['frames'][-1]['bodies']['puck']['position_m'][1] = .08
    profile_path.write_text(json.dumps(profile), encoding='utf-8')
    _write_trace(inputs, trace)
    with pytest.raises(QualificationError, match=expected):
        _check(inputs)


def test_missing_physical_body_tree_cannot_bypass_fault_gate(tmp_path):
    inputs = list(_fixture(tmp_path))
    inputs[0] = SimpleNamespace(mass_records={})
    with pytest.raises(QualificationError, match='parsed physical body'):
        _check(tuple(inputs))


def test_motor_off_case_directory_is_portable(tmp_path):
    inputs = _fixture(tmp_path)
    plan, profile_path = inputs[4], inputs[6]
    profile = json.loads(profile_path.read_text(encoding='utf-8'))
    profile['cases']['CON'] = profile['cases'].pop(CASE_ID)
    plan['motor_off_fault_cases']['CON'] = plan['motor_off_fault_cases'].pop(CASE_ID)
    profile_path.write_text(json.dumps(profile), encoding='utf-8')
    with pytest.raises(QualificationError, match='portable output directory'):
        _check(inputs)
