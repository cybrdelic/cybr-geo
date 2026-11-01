"""Motor-off contract fixtures use test-only limits, never measured ARX capacity."""
from copy import deepcopy
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from mechanism_lab.held_workpiece_fault import (
    HeldWorkpieceFaultError, held_free_workpieces, require_motor_off_fault_cases,
    validate_motor_off_fault_observations, validate_motor_off_fault_simulations,
)


SOURCE = Path(__file__).resolve().parents[1] / 'examples/engineering/arx_belt/data/P2-baseline.xml'
SOURCE_SHA = 'a' * 64  # Test-only source digest; release computes this from source files.
MODEL_SHA = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
CASE_ID = 'arx_test_only_motor_off_puck'


def _root():
    return ET.parse(SOURCE).getroot()


def _inputs(root=None):
    root = _root() if root is None else root
    detected = held_free_workpieces(root)['puck']
    case = dict(workpiece_body='puck', contact_pairs=detected['contact_pairs'],
                actuators_off=detected['actuators_off'],
                workpiece_mass_kg=detected['workpiece_mass_kg'],
                gravity_vector_m_s2=[0., -9.80665, 0.],
                maximum_mass_deviation_kg=.001, minimum_observation_s=1.,
                maximum_downward_drop_m=.002,
                maximum_residual_current_A=.01,
                maximum_residual_torque_Nm=.005)
    profile = dict(schema='cybrgeo-held-workpiece-fault-profile/1',
                   source_sha256=SOURCE_SHA, model_sha256=MODEL_SHA,
                   cases={CASE_ID: case})
    policy = dict(schema='cybrgeo-trust-policy/1', issuers={
        'test-issuer': dict(public_key_base64='TEST-ONLY-VERIFIED-BY-RELEASE',
                            allowed_kinds=['fault_test'])})
    return root, profile, policy, deepcopy(profile['cases'])


def _requirements():
    root, profile, policy, planned = _inputs()
    return require_motor_off_fault_cases(root, source_sha256=SOURCE_SHA,
        model_sha256=MODEL_SHA, external_profile=profile,
        trust_policy=policy, planned_cases=planned)


def _observation(requirements):
    case = requirements['cases'][CASE_ID]
    return dict(case_id=CASE_ID, source_sha256=SOURCE_SHA, model_sha256=MODEL_SHA,
                workpiece_body='puck', assembled_serial='TEST-UNIT-1',
                part_serials={'jaw_left__pad': 'TEST-L', 'jaw_right__pad': 'TEST-R'},
                workpiece_mass_kg=case['workpiece_mass_kg'], power_removed=True,
                actuators_off=case['actuators_off'], commanded_current_A=0.,
                gravity_vector_m_s2=case['gravity_vector_m_s2'],
                initial_contact_pairs=case['contact_pairs'],
                initially_suspended_only_by_holder_contacts=True,
                other_supports_absent=True,
                commanded_torque_Nm=0., maximum_measured_current_A=0.,
                maximum_measured_torque_Nm=0., observation_s=1.,
                maximum_downward_drop_m=0.)


def _validate(requirements, observations):
    return validate_motor_off_fault_observations(requirements,
        source_sha256=SOURCE_SHA, model_sha256=MODEL_SHA,
        assembled_serial='TEST-UNIT-1',
        part_serials={'jaw_left__pad': 'TEST-L', 'jaw_right__pad': 'TEST-R'},
        observations=observations)


def test_actual_arx_free_puck_has_explicit_contacts_with_two_moving_jaws():
    detected = held_free_workpieces(_root())
    assert set(detected) == {'puck'}
    assert detected['puck']['contact_pairs'] == [['pad_left', 'puck'], ['pad_right', 'puck']]
    assert detected['puck']['holder_bodies'] == ['jaw_left', 'jaw_right']
    assert detected['puck']['workpiece_mass_kg'] == 0.0819390195909
    assert 'gripper_motor' in detected['puck']['actuators_off']
    assert len(detected['puck']['actuators_off']) == 7


def test_names_do_not_define_the_detection_gate():
    xml = SOURCE.read_text(encoding='utf-8')
    for old, new in [('pad_left', 'contact_A'), ('pad_right', 'contact_B'),
                     ('puck', 'loose_body')]:
        xml = xml.replace(old, new)
    detected = held_free_workpieces(ET.fromstring(xml))
    assert set(detected) == {'loose_body'}
    assert detected['loose_body']['contact_pairs'] == [
        ['contact_A', 'loose_body'], ['contact_B', 'loose_body']]


def test_removing_one_pair_cannot_skip_motor_off_gate():
    root = _root()
    root.find('./contact').remove(root.findall('./contact/pair')[0])
    assert set(held_free_workpieces(root)) == {'puck'}
    with pytest.raises(HeldWorkpieceFaultError, match='contacts differ'):
        _, profile, policy, planned = _inputs()
        require_motor_off_fault_cases(root, source_sha256=SOURCE_SHA,
            model_sha256=MODEL_SHA, external_profile=profile,
            trust_policy=policy, planned_cases=planned)


def test_removing_all_pairs_cannot_skip_motor_off_gate():
    root = _root()
    for pair in list(root.findall('./contact/pair')):
        root.find('./contact').remove(pair)
    with pytest.raises(HeldWorkpieceFaultError, match='no explicit moving-holder contact'):
        held_free_workpieces(root)


def test_missing_source_revision_and_boolean_gravity_cannot_enter_case_contract():
    root, profile, policy, planned = _inputs()
    with pytest.raises(HeldWorkpieceFaultError, match='Source revision'):
        require_motor_off_fault_cases(root, source_sha256=None,
            model_sha256=MODEL_SHA, external_profile=profile,
            trust_policy=policy, planned_cases=planned)
    profile['cases'][CASE_ID]['gravity_vector_m_s2'][0] = False
    with pytest.raises(HeldWorkpieceFaultError, match='finite gravity 3-vector'):
        require_motor_off_fault_cases(root, source_sha256=SOURCE_SHA,
            model_sha256=MODEL_SHA, external_profile=profile,
            trust_policy=policy, planned_cases=planned)


@pytest.mark.parametrize('fault, expected', [
    ('missing_profile', 'acceptance profile'),
    ('missing_policy', 'External trust policy'),
    ('wrong_kind', 'fault_test issuer'),
    ('missing_plan', 'must be an object'),
    ('missing_case', 'exactly the external'),
    ('stale_source', 'stale for source/model'),
    ('stale_model', 'stale for source/model'),
    ('wrong_gravity', 'gravity differs from source'),
    ('understated_mass', 'mass differs from source'),
    ('missing_actuator', 'every source actuator'),
    ('weaker_plan', 'differs from external'),
])
def test_profile_and_source_plan_fail_closed(fault, expected):
    root, profile, policy, planned = _inputs()
    if fault == 'missing_profile': profile = None
    elif fault == 'missing_policy': policy = None
    elif fault == 'wrong_kind': policy['issuers']['test-issuer']['allowed_kinds'] = ['assembly_test']
    elif fault == 'missing_plan': planned = None
    elif fault == 'missing_case': planned = {}
    elif fault == 'stale_source': profile['source_sha256'] = 'b' * 64
    elif fault == 'stale_model': profile['model_sha256'] = 'b' * 64
    elif fault == 'wrong_gravity': profile['cases'][CASE_ID]['gravity_vector_m_s2'] = [0., 0., -9.80665]
    elif fault == 'understated_mass': profile['cases'][CASE_ID]['workpiece_mass_kg'] = .07
    elif fault == 'missing_actuator': profile['cases'][CASE_ID]['actuators_off'].pop()
    elif fault == 'weaker_plan': planned[CASE_ID]['maximum_downward_drop_m'] = .01
    with pytest.raises(HeldWorkpieceFaultError, match=expected):
        require_motor_off_fault_cases(root, source_sha256=SOURCE_SHA,
            model_sha256=MODEL_SHA, external_profile=profile,
            trust_policy=policy, planned_cases=planned)


@pytest.mark.parametrize('fault, expected', [
    ('missing_evidence', 'cover exact required'),
    ('stale_source', 'another case/source/model'),
    ('stale_model', 'another case/source/model'),
    ('wrong_serial', 'inspected assembled unit'),
    ('wrong_bom', 'inspected assembled unit'),
    ('wrong_mass', 'outside source-bound range'),
    ('powered', 'did not remove power'),
    ('one_drive_on', 'did not remove power'),
    ('no_initial_contact', 'not observed initially suspended'),
    ('supported_by_table', 'not observed initially suspended'),
    ('not_suspended', 'not observed initially suspended'),
    ('different_gravity', 'not observed initially suspended'),
    ('nonzero_current_command', 'nonzero motor-off command'),
    ('nonzero_torque_command', 'nonzero motor-off command'),
    ('residual_current', 'exceeds source-bound'),
    ('residual_torque', 'exceeds source-bound'),
    ('short_duration', 'too short'),
    ('excess_drop', 'exceeds source-bound limit'),
    ('missing_measurement', 'fields must be exactly'),
])
def test_signed_measurement_fields_fail_closed(fault, expected):
    requirements = _requirements()
    row = _observation(requirements)
    if fault == 'stale_source': row['source_sha256'] = 'b' * 64
    elif fault == 'stale_model': row['model_sha256'] = 'b' * 64
    elif fault == 'wrong_serial': row['assembled_serial'] = 'OTHER'
    elif fault == 'wrong_bom': row['part_serials']['jaw_left__pad'] = 'OTHER'
    elif fault == 'wrong_mass': row['workpiece_mass_kg'] += .002
    elif fault == 'powered': row['power_removed'] = False
    elif fault == 'one_drive_on': row['actuators_off'] = ['gripper_motor']
    elif fault == 'no_initial_contact': row['initial_contact_pairs'] = []
    elif fault == 'supported_by_table': row['other_supports_absent'] = False
    elif fault == 'not_suspended': row['initially_suspended_only_by_holder_contacts'] = False
    elif fault == 'different_gravity': row['gravity_vector_m_s2'] = [0., 0., -9.80665]
    elif fault == 'nonzero_current_command': row['commanded_current_A'] = .1
    elif fault == 'nonzero_torque_command': row['commanded_torque_Nm'] = .1
    elif fault == 'residual_current': row['maximum_measured_current_A'] = .02
    elif fault == 'residual_torque': row['maximum_measured_torque_Nm'] = .006
    elif fault == 'short_duration': row['observation_s'] = .9
    elif fault == 'excess_drop': row['maximum_downward_drop_m'] = .003
    elif fault == 'missing_measurement': row.pop('maximum_downward_drop_m')
    rows = {} if fault == 'missing_evidence' else {CASE_ID: row}
    with pytest.raises(HeldWorkpieceFaultError, match=expected):
        _validate(requirements, rows)


def test_complete_test_only_case_fields_are_accepted_without_authenticity_claim():
    requirements = _requirements()
    row = _observation(requirements)
    assert _validate(requirements, {CASE_ID: row}) == {CASE_ID: row}
    with pytest.raises(HeldWorkpieceFaultError, match='stale for source/model'):
        validate_motor_off_fault_observations(requirements,
            source_sha256='b' * 64, model_sha256=MODEL_SHA,
            assembled_serial='TEST-UNIT-1', part_serials=row['part_serials'],
            observations={CASE_ID: row})


@pytest.mark.parametrize('fault, expected', [
    ('missing_result', 'exact required case IDs'),
    ('missing_trace_hash', 'fields must be exactly'),
    ('stale_source', 'another case/source/model'),
    ('no_initial_contact', 'not observed initially suspended'),
    ('supported_by_nest', 'not observed initially suspended'),
    ('too_short', 'too short'),
    ('excess_drop', 'exceeds source-bound limit'),
])
def test_simulation_receipt_fields_fail_closed(fault, expected):
    requirements = _requirements()
    row = _observation(requirements)
    row.pop('assembled_serial')
    row.pop('part_serials')
    row['trace_sha256'] = 'c' * 64  # Test-only; caller must hash/replay actual trace bytes.
    if fault == 'missing_trace_hash': row.pop('trace_sha256')
    elif fault == 'stale_source': row['source_sha256'] = 'b' * 64
    elif fault == 'no_initial_contact': row['initial_contact_pairs'] = []
    elif fault == 'supported_by_nest': row['other_supports_absent'] = False
    elif fault == 'too_short': row['observation_s'] = .9
    elif fault == 'excess_drop': row['maximum_downward_drop_m'] = .003
    results = {} if fault == 'missing_result' else {CASE_ID: row}
    with pytest.raises(HeldWorkpieceFaultError, match=expected):
        validate_motor_off_fault_simulations(requirements,
            source_sha256=SOURCE_SHA, model_sha256=MODEL_SHA, results=results)


def test_simulation_receipt_syntax_pass_is_not_a_replay_or_hardware_test():
    requirements = _requirements()
    row = _observation(requirements)
    row.pop('assembled_serial')
    row.pop('part_serials')
    row['trace_sha256'] = 'c' * 64
    assert validate_motor_off_fault_simulations(requirements,
        source_sha256=SOURCE_SHA, model_sha256=MODEL_SHA,
        results={CASE_ID: row}) == {CASE_ID: row}
