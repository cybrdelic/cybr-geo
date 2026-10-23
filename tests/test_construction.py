"""Construction contract tests, independent of optional CAD/render runtimes."""
import copy
import json
import math

import pytest

from mechanism_lab.construction import ARCHITECTURES, compile_plan, recipe_digest


@pytest.mark.parametrize('architecture', ARCHITECTURES)
def test_every_part_has_a_connected_rigid_or_moving_parent(architecture):
    plan = compile_plan(architecture)
    names = {part['name'] for part in plan['parts']}
    joints = {joint['child']: joint for joint in plan['joints']}
    assert len(names) == len(plan['parts']) == len(joints)
    assert set(joints) == names
    roots = []
    for name in names:
        visited = set()
        node = name
        while node is not None:
            assert node in names and node not in visited
            visited.add(node)
            node = joints[node]['parent']
        if joints[name]['parent'] is None:
            roots.append(name)
    assert roots == ['J1_carrier']
    assert len(plan['parts']) >= 34
    # The complete plan remains usable without Python-specific serialized objects.
    assert json.loads(json.dumps(plan)) == plan


@pytest.mark.parametrize('architecture, axes', [('compact_arm', 6), ('belt_arm', 6), ('inspection_head', 3)])
def test_interfaces_are_derived_from_actual_primitive_dimensions(architecture, axes):
    plan = compile_plan(architecture)
    parts = {part['name']: part for part in plan['parts']}
    interfaces = plan['construction']['interfaces']
    journals = [row for row in interfaces if row['kind'] == 'journal']
    assert len(journals) == 2*axes
    for row in journals:
        shaft, bushing = (parts[name] for name in row['parts'])
        assert shaft['body']['size'][0]/2 == row['shaft_radius_mm']
        assert bushing['cuts'][0]['size'][0]/2 == row['bore_radius_mm']
        assert row['bore_radius_mm'] - row['shaft_radius_mm'] == pytest.approx(row['radial_clearance_mm'])
    assert all(row['radial_clearance_mm'] > 0 for row in interfaces if 'radial_clearance_mm' in row)
    assert {name for row in interfaces for name in row['parts']} <= set(parts)


def test_belt_plan_constructs_distinct_transmission_and_derives_follower_limits():
    plan = compile_plan('belt_arm')
    by_name = {j['name']: j for j in plan['joints']}
    stages = [row for row in plan['construction']['actuators'] if row['type'] == 'equal_pulley_belt_drive']
    assert len(stages) == 2
    for stage in stages:
        source, follower = by_name[stage['joint']], by_name[stage['follower']]
        assert follower['drive'] == dict(joint=source['name'], ratio=1., offset=0.)
        assert [source['min'], source['max']] == [follower['min'], follower['max']]
    belts = [p for p in plan['parts'] if p['name'].endswith('_belt')]
    assert len(belts) == 2
    # Each loop is constructed as a full capsule minus a smaller full capsule,
    # not a disconnected set of visual cable segments.
    for part in belts:
        assert part['body']['kind'] == 'box'
        assert len(part['unions']) == 2 and len(part['cuts']) == 3
        assert all(p['kind'] == 'cylinder' for p in part['unions'])
    assert not any(j['drive'] for j in compile_plan('compact_arm')['joints'])


def test_jaws_have_independent_actual_guide_clearance_and_stroke():
    plan = compile_plan(parameters={'jaw_travel_mm': 14.})
    jaws = [j for j in plan['joints'] if j['type'] == 'prismatic']
    assert len(jaws) == 2
    assert [j['axis'][1] for j in jaws] == [-1, 1]
    assert all(j['min'] == 0 and j['max'] == 14 and j['drive'] is None for j in jaws)
    for side in ('left', 'right'):
        interface = next(row for row in plan['construction']['interfaces'] if row['name'] == f'jaw_{side}_guidance')
        assert interface['minimum_guide_engagement_mm'] == interface['carriage_length_mm']
        # The entire carriage remains inside support faces at the max stroke.
        assert 17 + 6 + interface['travel_mm'] < 43-4


@pytest.mark.parametrize('parameters', [
    {'scale': 0}, {'scale': True}, {'scale': float('nan')}, {'unknown': 4},
    {'radial_clearance_mm': -.1}, {'joint_limit_deg': 180},
    {'jaw_travel_mm': float('inf')}, {'scale': .5, 'jaw_travel_mm': 14},
])
def test_bad_design_parameters_fail_before_geometry(parameters):
    with pytest.raises(ValueError):
        compile_plan(parameters=parameters)


@pytest.mark.parametrize('scale', [.5, 1., 2.])
def test_scale_preserves_requested_absolute_clearance_and_feasible_default_stroke(scale):
    plan = compile_plan(parameters={'scale': scale, 'radial_clearance_mm': .3})
    interfaces = plan['construction']['interfaces']
    assert all(row['radial_clearance_mm'] == pytest.approx(.3)
               for row in interfaces if row['kind'] == 'journal')
    assert all(j['max'] <= 14*scale for j in plan['joints'] if j['type'] == 'prismatic')
    assert plan['parts'][0]['body']['size'][0] == 36*scale


def test_identity_covers_geometry_and_motion_but_is_not_a_certification():
    plan = compile_plan()
    original = recipe_digest(plan)
    assert original == plan['construction']['recipe_sha256']
    plan['generation'] = {'planner': 'another transport'}
    assert recipe_digest(plan) == original
    altered = copy.deepcopy(plan)
    altered['parts'][0]['body']['size'][0] += .001
    assert recipe_digest(altered) != original
    altered = copy.deepcopy(plan)
    altered['joints'][1]['axis'] = [1., 0., 0.]
    assert recipe_digest(altered) != original
    assert plan['construction']['qualification']['dynamics'] == 'not simulated'
    assert plan['construction']['qualification']['hardware'] == 'not qualified'


def test_unknown_architecture_is_not_silently_replaced():
    with pytest.raises(ValueError, match='Unknown architecture'):
        compile_plan('iris09')


def test_front_inserted_shaft_has_removable_rear_retention():
    plan = compile_plan()
    parts = {part['name']: part for part in plan['parts']}
    for index in range(1, 7):
        # The shaft has only the front axial shoulder plus its key. A rear
        # integral collar would make insertion through both bushings impossible.
        shaft = parts[f'J{index}_shaft']
        assert len(shaft['unions']) == 2
        assert parts[f'J{index}_retainer_a']['cuts'][0]['size'][0] > shaft['body']['size'][0]
        assert parts[f'J{index}_retainer_b']['cuts'][0]['size'][0] > shaft['body']['size'][0]
        for suffix in ('bolt_0', 'bolt_1', 'nut_0', 'nut_1'):
            assert f'J{index}_retainer_{suffix}' in parts
    assert len([row for row in plan['construction']['interfaces']
                if row['kind'] == 'removable_axial_retainer']) == 6


def test_returned_recipe_mutation_does_not_change_future_defaults():
    first = compile_plan()
    identity = recipe_digest(first)
    first['parts'][0]['color'][0] = 1.
    assert recipe_digest(compile_plan()) == identity


@pytest.mark.parametrize('architecture, count', [('compact_arm', 12), ('belt_arm', 20), ('inspection_head', 6)])
def test_every_output_flange_has_two_geometric_axial_capture_probes(architecture, count):
    plan = compile_plan(architecture)
    parts = {part['name']: part for part in plan['parts']}
    probes = plan['construction']['capture_probes']
    assert len(probes) == count
    assert all(probe['moving_part'] in parts and probe['blocker'] in parts for probe in probes)
    assert all(probe['distance_mm'] == 1. for probe in probes)
    for index in range(1, 4 if architecture == 'inspection_head' else 7):
        assert f'J{index}_output_cap' in parts
        assert f'J{index}_output_cap_screw' in parts
        shaft = parts[f'J{index}_shaft']
        assert any(cut['kind'] == 'cylinder' and cut['size'][0] == 4.4 for cut in shaft['cuts'])


def test_belt_keys_are_separate_installable_parts_with_matching_slots():
    plan = compile_plan('belt_arm')
    parts = {part['name']: part for part in plan['parts']}
    keys = [row for row in plan['construction']['interfaces'] if row['kind'] == 'removable_keyed_pulley']
    assert len(keys) == 4
    for row in keys:
        key = parts[row['key']]
        shaft, pulley = [parts[name] for name in row['parts']]
        assert key['body']['kind'] == 'box'
        assert any(cut['kind'] == 'box' for cut in shaft['cuts'])
        assert any(cut['kind'] == 'box' for cut in pulley['cuts'])
        assert key['body'] not in shaft['unions']


@pytest.mark.parametrize('scale', [.5, 1., 2.])
def test_tool_palm_departure_stays_outside_the_output_cap(scale):
    plan = compile_plan(parameters={'scale': scale})
    parts = {part['name']: part for part in plan['parts']}
    cap = parts['J6_output_cap']['body']
    lug = parts['tool_palm']['unions'][1]
    # Both cylinders have X axes. Their axial intervals overlap, so their
    # radial envelopes must be separated. The old radius8 lug cut into the cap.
    radial_distance = math.dist(cap['position'][1:], lug['position'][1:])
    radial_gap = radial_distance - cap['size'][0]/2 - lug['size'][0]/2
    assert radial_gap == pytest.approx(.5*scale)
