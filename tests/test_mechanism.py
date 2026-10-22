"""Behavioral tests for the canonical mechanical source, without CAD runtimes."""
import copy
import json

import pytest

from mechanism_lab.construction import (
    compile_mechanism, model_digest, recipe_digest, validate_constructed_plan,
)
from mechanism_lab.mechanism import load_example, mechanism_digest, validate_mechanism


def test_contact_scene_is_part_of_the_same_source_and_physical_identity():
    source = load_example('inspection_head')
    original = compile_mechanism(source)
    source['contacts']['free_bodies'] = [dict(
        name='contact_probe', position_mm=[0, 200, 200], velocity_mm_s=[0, 0, 0],
        angular_velocity_rad_s=[0, 0, 0], mass_kg=.1,
        shape=dict(kind='sphere', size=[20, 20, 20], position=[0, 0, 0], rotation=[0, 0, 0]))]
    with_probe = compile_mechanism(source)
    assert with_probe['construction']['dynamics']['contacts'] == source['contacts']
    assert recipe_digest(with_probe) == recipe_digest(original)
    assert model_digest(with_probe) != model_digest(original)
    assert validate_constructed_plan(with_probe)
    source['contacts']['free_bodies'][0]['mass_kg'] = -.1
    with pytest.raises(ValueError, match='mass_kg'):
        compile_mechanism(source)


def test_cut_parts_are_never_silently_replaced_by_solid_contact_hulls():
    plan = compile_mechanism(load_example())
    metadata = plan['construction']['dynamics']
    parts_with_cuts = {part['name'] for part in plan['parts'] if part['cuts']}
    proxy_parts = {proxy['part'] for proxy in metadata['collision_proxies']}
    assert parts_with_cuts.isdisjoint(proxy_parts)
    assert set(metadata['contact_policy']['omitted_parts']) == parts_with_cuts
    assert metadata['contacts']['enable_component_proxies'] is False
from mechanism_lab.component_catalog import register_component


def short_composition():
    """Compose existing interfaces under new names, without a preset selector."""
    source = load_example('compact_arm')
    base, wrist, tool = (copy.deepcopy(source['components'][i]) for i in (0, 5, 6))
    base['id'] = 'Pedestal'
    wrist.update(id='Wrist', parent='Pedestal')
    wrist['parameters']['origin_mm'] = [150., 0., 155.]
    tool.update(id='Clamp', parent='Wrist')
    source.update(name='Customer Fixture 42', components=[base, wrist, tool])
    used = {base['drive']['actuator'], wrist['drive']['actuator'],
            tool['drive']['left'], tool['drive']['right']}
    source['actuators'] = {name: row for name, row in source['actuators'].items() if name in used}
    source['loads']['payload']['component'] = 'Clamp'
    return source


@pytest.mark.parametrize('name', ['compact_arm', 'belt_arm', 'inspection_head'])
def test_examples_are_canonical_data_and_compile_through_one_contract(name):
    source = load_example(name)
    original = copy.deepcopy(source)
    assert source['schema'] == 'cybrgeo-mechanism/2'
    assert json.loads(json.dumps(source)) == source
    plan = compile_mechanism(source)
    assert source == original  # Lowering must not mutate the caller's design.
    metadata = plan['construction']
    assert metadata['architecture'] == 'component_graph'
    assert metadata['mechanism'] == validate_mechanism(source)
    assert metadata['mechanism_sha256'] == mechanism_digest(metadata['mechanism'])
    assert validate_constructed_plan(plan)
    assert metadata['qualification']['dynamics'] == 'not simulated'
    assert metadata['qualification']['hardware'] == 'not qualified'


def test_renamed_short_composition_needs_no_architecture_specific_path():
    source = short_composition()
    plan = compile_mechanism(source)
    assert plan['name'] == 'Customer Fixture 42'
    components = plan['construction']['components']
    assert [row['id'] for row in components] == ['Pedestal', 'Wrist', 'Clamp']
    parts = {part['name'] for part in plan['parts']}
    assert all(row['input_part'] in parts and row['output_part'] in parts for row in components)
    by_child = {joint['child']: joint for joint in plan['joints']}
    assert len(by_child) == len(parts)
    for part in parts:
        visited, cursor = set(), part
        while cursor is not None:
            assert cursor in parts and cursor not in visited
            visited.add(cursor)
            cursor = by_child[cursor]['parent']
    moving = [joint for joint in plan['joints'] if joint['type'] != 'fixed']
    assert len([joint for joint in moving if joint['type'] == 'revolute']) == 2
    assert len([joint for joint in moving if joint['type'] == 'prismatic']) == 2
    assert {row['joint'] for row in plan['construction']['dynamics']['actuators']} == {j['name'] for j in moving}
    assert validate_constructed_plan(plan)
    # A new user design remains independent of both the example and caller data.
    source['materials']['shaft_steel']['density_kg_m3'] = 100.
    assert plan['construction']['mechanism']['materials']['shaft_steel']['density_kg_m3'] == 7850


def test_source_list_order_is_normalized_by_parent_dependencies():
    source = short_composition()
    forward = compile_mechanism(source)
    source['components'].reverse()
    reverse = compile_mechanism(source)
    assert reverse['parts'] == forward['parts']
    assert reverse['joints'] == forward['joints']
    assert reverse['construction']['mechanism_sha256'] == forward['construction']['mechanism_sha256']
    assert model_digest(reverse) == model_digest(forward)


@pytest.mark.parametrize('fit_name', ['journal', 'guide'])
def test_worst_case_fit_stack_rejects_nominally_positive_but_closed_clearance(fit_name):
    source = short_composition()
    source['tolerances'][fit_name].update(radial_clearance_mm=.12,
        shaft_radius_tolerance_mm=.06, bore_radius_tolerance_mm=.06,
        minimum_clearance_mm=.01)
    with pytest.raises(ValueError, match='tolerance stack closes'):
        compile_mechanism(source)


@pytest.mark.parametrize('axis', [[-1, 0, 0], [0, .6, .8], [0, 0, 0]])
def test_unsupported_axis_is_rejected_before_lowering(axis):
    source = short_composition()
    source['components'][1]['parameters']['axis'] = axis
    with pytest.raises(ValueError, match='positive cardinal'):
        compile_mechanism(source)


def test_branch_is_rejected_as_unsupported_instead_of_silently_serialized():
    source = short_composition()
    branch = copy.deepcopy(source['components'][1])
    branch['id'] = 'OtherWrist'
    source['components'].append(branch)
    with pytest.raises(ValueError, match='branch interface component'):
        compile_mechanism(source)


def test_unresolved_material_does_not_get_an_invented_density():
    source = short_composition()
    del source['materials'][source['material_roles']['shaft']]
    with pytest.raises(ValueError, match='material role'):
        compile_mechanism(source)


def test_underpowered_actuator_is_rejected_before_cad():
    source = short_composition()
    source['actuators']['J1_motor']['effort_limit_Nmm'] = 1.
    with pytest.raises(ValueError, match='effort domain exceeded before CAD'):
        compile_mechanism(source)


def test_payload_that_exceeds_actuation_screen_is_rejected():
    source = short_composition()
    source['loads']['payload']['mass_kg'] = 10.
    with pytest.raises(ValueError, match='effort domain exceeded before CAD'):
        compile_mechanism(source)


def test_arbitrarily_increasing_motor_effort_does_not_evade_shaft_domain():
    source = short_composition()
    source['actuators']['J1_motor']['effort_limit_Nmm'] = 1e6
    with pytest.raises(ValueError, match='nominal shaft torsion domain'):
        compile_mechanism(source)


@pytest.mark.parametrize('assumption', ['density', 'gravity', 'friction', 'manufacturing'])
def test_physical_assumptions_change_full_identity_even_when_geometry_is_identical(assumption):
    source = short_composition()
    original = compile_mechanism(source)
    if assumption == 'density':
        source['materials']['shaft_steel']['density_kg_m3'] += 1.
    elif assumption == 'gravity':
        source['loads']['gravity_mm_s2'][2] = -9800.
    elif assumption == 'friction':
        source['contacts']['friction'][0] = .7
    else:
        source['manufacturing']['process'] += '; changed stock assumption'
    changed = compile_mechanism(source)
    assert recipe_digest(original) == recipe_digest(changed)
    assert original['construction']['mechanism_sha256'] != changed['construction']['mechanism_sha256']
    assert model_digest(original) != model_digest(changed)
    assert validate_constructed_plan(changed)


@pytest.mark.parametrize('target', ['material', 'dynamics', 'contact', 'qualification'])
def test_existing_identity_cannot_be_reused_after_editing_physical_assumptions(target):
    plan = compile_mechanism(short_composition())
    if target == 'material':
        plan['construction']['mechanism']['materials']['shaft_steel']['density_kg_m3'] += 1
    elif target == 'dynamics':
        plan['construction']['dynamics']['gravity_mm_s2'][2] = 0
    elif target == 'contact':
        plan['construction']['dynamics']['contact_policy']['enabled'] = True
    else:
        plan['construction']['qualification']['hardware'] = 'qualified'
    with pytest.raises(ValueError, match='identity mismatch'):
        validate_constructed_plan(plan)


def test_external_static_component_compiles_without_central_dispatch_changes():
    def validate_fixture(component, context):
        sizes = component['parameters'].get('size_mm')
        if (not isinstance(sizes, list) or len(sizes) != 3
                or not all(isinstance(value, (int, float)) and not isinstance(value, bool)
                           and 1 <= value <= 100 for value in sizes)):
            raise ValueError('Fixture sizes must be within its authored component domain')
        if component['parent'] is not None or component['drive']:
            raise ValueError('This fixture is a grounded static component')
        return []

    def expand_fixture(builder, component, previous):
        assert previous is None
        name = component['id'] + '_plate'
        sizes = [value / builder.scale for value in component['parameters']['size_mm']]
        builder.part(name, builder.box(sizes, [0, 0, sizes[2] / 2]),
                     'External test component: grounded solid fixture plate')
        builder.fixed(name, None)
        return dict(input_part=name, output_part=name,
                    output_datum=([0, 0, 0], [0, 0, 1], name),
                    actuators={}, capture_probes=[], capture_parent=False)

    registration = dict(validator=validate_fixture, expander=expand_fixture,
                        revision='test-1', provenance='Test-authored static fixture', root_allowed=True)
    register_component('test_static_fixture', **registration)
    source = load_example('compact_arm')
    source.update(name='External Grounded Fixture', actuators={}, components=[dict(
        id='Fixture', type='test_static_fixture', parent=None,
        parameters={'size_mm': [60, 40, 8]}, drive={})])
    source['loads']['payload']['component'] = 'Fixture'
    plan = compile_mechanism(source)
    assert [part['name'] for part in plan['parts']] == ['Fixture_plate']
    assert plan['parts'][0]['body']['size'] == [60, 40, 8]
    assert plan['joints'][0]['parent'] is None and plan['joints'][0]['type'] == 'fixed'
    metadata = plan['construction']
    assert metadata['components'][0]['type'] == 'test_static_fixture'
    assert metadata['component_definitions']['test_static_fixture']['revision'] == 'test-1'
    assert metadata['dynamics']['actuators'] == []
    assert metadata['dynamics']['payload']['part'] == 'Fixture_plate'
    assert validate_constructed_plan(plan)
    with pytest.raises(ValueError, match='already registered'):
        register_component('test_static_fixture', **registration)
    source['components'][0]['parameters']['size_mm'][0] = -1
    with pytest.raises(ValueError, match='authored component domain'):
        compile_mechanism(source)
