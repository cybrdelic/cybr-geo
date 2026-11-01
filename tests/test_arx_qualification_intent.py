"""The ARX diagnostic construction intent must stay bound to its source limits."""
import json
from copy import deepcopy
from pathlib import Path

import pytest

from mechanism_lab.qualification import QualificationIntentPlan
from mechanism_lab.spring_anchor import tension_anchor_demand


DATA = Path(__file__).resolve().parents[1] / 'examples' / 'engineering' / 'arx_belt' / 'data'


def read(name):
    return json.loads((DATA / name).read_text(encoding='utf-8'))


def test_source_spring_pins_and_drive_limits_are_not_recast_as_capacity():
    spec = read('spec.json')
    intent = QualificationIntentPlan(read('qualification-intent.json')).document
    assert len(intent['parts']) == 380
    assert len(intent['interfaces']) == 8
    for group, source_key in (('shoulder', 'counterbalance'),
                              ('elbow', 'elbow_counterbalance')):
        spring = spec[source_key]
        demand = tension_anchor_demand(
            spring, spec['joint_hard_limits_rad'][spring['joint_index']],
            pin_diameter_mm=4.)['required_capacity_per_separate_pin_N']
        for member in range(spring['spring_count']):
            for end in ('fixed', 'moving'):
                row = intent['interfaces'][f'{group}_spring_member_{member}_{end}']
                assert row['role'] == 'retained_contact'
                assert row['force_demand_N'] == pytest.approx(demand)
                assert row['required_service_gap_mm'] is None
                assert row['first_variation_mm'] is None
                assert row['second_variation_mm'] is None
                assert intent['parts'][row['parts'][0]]['material'] == 'steel'
                assert intent['parts'][row['parts'][1]]['material'] == 'aluminum'

    belt = spec['folded_transmission']['rating_screen']
    expected = [belt['large']['operating_screen_limit_Nm']] * 3 + [
        belt['small']['operating_screen_limit_Nm']] * 3
    for index, limit in enumerate(expected, 1):
        assert intent['loads'][f'motor{index}']['source_demand_limit_Nm'] == limit
    assert intent['loads']['rack_motor']['source_demand_limit_Nm'] == read(
        'P2-baseline-parameters.json')['gripper']['output_torque_limit_Nm']
    assert {item['kind'] for item in intent['unresolved_requirements']} >= {
        'installed_interface_graph', 'power_loss_payload_retention',
        'material_and_process_capability', 'service_and_capacity_evidence',
        'retained_spring_anchor_test'}


def test_native_builder_rejects_stale_intent_drive_source_before_cad():
    pytest.importorskip('cadquery')
    from examples.engineering.arx_belt.design import qualification_intent

    source = read('spec.json')
    assert len(qualification_intent(source).document['parts']) == 380
    changed = deepcopy(source)
    changed['folded_transmission']['rating_screen']['large']['operating_screen_limit_Nm'] += .1
    with pytest.raises(ValueError, match='source limit changed'):
        qualification_intent(changed)
