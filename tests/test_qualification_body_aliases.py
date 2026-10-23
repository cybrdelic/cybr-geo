"""Native CAD body aliases cannot change physical connector or spring ownership."""
import math
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest

from mechanism_lab.qualification import (
    QualificationError, QualificationIntentPlan, QualificationPlan,
    QualifiedNativeConstruction, _require_connector_demand_binding,
    _require_spring_anchor_demand_binding,
)
from mechanism_lab.native_construction import NativeConstruction
from mechanism_lab.physical_model import digest
from mechanism_lab.spring_anchor import member_anchor_positions_mm, tension_anchor_demand


def _plan(parts, interfaces, *, spring_paths=None, connectors=None):
    material = dict(grade='TEST', lot='TEST', density_kg_m3=2700.,
                    allowable_stress_MPa=100., temperature_limit_C=80.)
    process = dict(method='TEST', facility='TEST', revision='1',
                   allowed_materials=['alloy'], minimum_feature_mm=1.,
                   capability_tolerance_mm=.1, maximum_part_volume_mm3=1000.)
    part = dict(material='alloy', process='mill', nominal_minimum_wall_mm=2.,
                required_dimensional_tolerance_mm=.1, maximum_volume_mm3=500.)
    row = dict(schema='cybrgeo-qualification-plan/1',
        materials={'alloy': material}, processes={'mill': process},
        parts={name: dict(part) for name in parts}, interfaces=interfaces,
        loads={'motor': dict(part=parts[0], demand_Nm=1., design_capacity_Nm=2.,
                             service_temperature_C=50.)},
        assembly_requirements=dict(minimum_endurance_cycles=1000,
                                   minimum_thermal_test_C=60.,
                                   maximum_measured_backlash_mm=.3),
        solver_connectors=connectors or {}, spring_load_paths=spring_paths or {})
    return QualificationPlan(row)


def _clearance(first, second):
    return dict(parts=[first, second], nominal_clearance_mm=.5,
                first_tolerance_mm=.1, second_tolerance_mm=.1,
                required_clearance_mm=.2, demand_N=5., design_capacity_N=10.)


def test_connector_reaction_requires_bound_physical_endpoints_not_raw_native_names():
    # Physical a-b; CAD a-b maps to physical b-c, whereas CAD c-a maps to a-b.
    parts = ['a__one', 'b__two', 'c__three']
    interfaces = {'ab': _clearance('a__one', 'b__two'),
                  'ca': _clearance('c__three', 'a__one')}
    aliases = {'a': 'b', 'b': 'c', 'c': 'a'}
    root = ET.Element('mujoco')
    ET.SubElement(ET.SubElement(root, 'equality'), 'connect',
                  name='coupler', body1='a', body2='b')
    model = SimpleNamespace(root=root)
    simulation = dict(steps=2, metrics={'constraints': {'coupler': dict(
        type='connect', active_steps=2, missing_row_steps=0,
        incomplete_reaction_row_steps=0,
        reaction_vector_norms={'anchor_force': dict(samples=2,
            peak_absolute=dict(value=4.))})}})
    wrong = _plan(parts, interfaces, connectors={'coupler': 'ab'})
    with pytest.raises(QualificationError, match='unrelated native interface bodies'):
        _require_connector_demand_binding(wrong, model, simulation,
                                          body_mapping=aliases)
    correct = _plan(parts, interfaces, connectors={'coupler': 'ca'})
    modeled = {}
    _require_connector_demand_binding(correct, model, simulation, modeled,
                                      body_mapping=aliases)
    assert modeled == {'ab': 0., 'ca': 4.}
    with pytest.raises(QualificationError, match='unrelated native interface bodies'):
        _require_connector_demand_binding(correct, model, simulation)


def _spring_fixture():
    parent, child = 'native_parent', 'native_child'
    aliases = {parent: 'physical_parent', child: 'physical_child'}
    parts = [f'{parent}__pin', f'{parent}__clevis',
             f'{child}__pin', f'{child}__clevis']
    interfaces = {f'{role}_anchor': dict(
        role='tested_retention_contact', parts=[f'{body}__pin', f'{body}__clevis'],
        nominal_maximum_distance_mm=.1, first_tolerance_mm=.1,
        second_tolerance_mm=.1, maximum_service_play_mm=.31,
        demand_N=5., design_capacity_N=10.)
        for role, body in (('fixed', parent), ('moving', child))}
    spring_paths = {'spring_0': dict(joint='j2', maximum_service_scale=1.2,
        uncertainty_factor=1.1, anchors={'member_0_fixed': 'fixed_anchor',
                                         'member_0_moving': 'moving_anchor'})}
    plan = _plan(parts, interfaces, spring_paths=spring_paths)
    config = dict(parent_anchor_m=[0., 0., 2.], child_anchor_m=[1., 0., 0.],
                  spring_count=1, free_eye_length_m=0., initial_tension_N=0.,
                  stiffness_N_m=1.)
    spring = dict(joint='j2', parameters=config, axis=[0., 1., 0.])
    control = {'springs': [spring]}
    root = ET.Element('mujoco')
    ET.SubElement(root, 'compiler', angle='radian', autolimits='true')
    parent_body = ET.SubElement(ET.SubElement(root, 'worldbody'), 'body',
                                name='physical_parent')
    child_body = ET.SubElement(parent_body, 'body', name='physical_child')
    ET.SubElement(child_body, 'joint', name='j2', type='hinge', axis='0 1 0',
                  range=f'0 {math.pi/2}')
    demand = tension_anchor_demand(config, [0., math.pi/2],
                                   pin_diameter_mm=4., axis=[0., 1., 0.])
    positions = member_anchor_positions_mm(config, None,
        child_body_pos_m=[0., 0., 0.], child_body_quat_wxyz=[1., 0., 0., 0.],
        joint_pos_m=[0., 0., 0.], axis=[0., 1., 0.])
    anchors, bindings, placements = [], [], []
    for role, body in (('fixed', parent), ('moving', child)):
        position = positions[0][role]
        name = f'{role}_eye'
        anchors.append(dict(name=name, source_spring_index=0,
            member_index=0, role=role, body=body,
            body_local_anchor_mm=position, pin_part='pin', clevis_part='clevis'))
        bindings.append(dict(anchor=name, source_spring_index=0,
            member_index=0, role=role, body=body,
            body_local_anchor_mm=position, pin_native_part='pin',
            clevis_native_part='clevis', clevis_placed_part='clevis',
            required_capacity_N=demand['required_capacity_per_separate_pin_N'],
            pin_diameter_mm=4., pin_length_mm=16.,
            clevis_bore_diameter_mm=4.2))
        placements.extend((dict(body=body, name='pin'),
                           dict(body=body, name='clevis')))
    group = dict(source_spring_index=0, control_joint='j2',
        source_parameters_sha256=digest(config), parent_body=parent,
        child_body=child, demand=demand, anchors=anchors)
    native_plan = dict(schema='cybrgeo-spring-anchor-plan/1', groups=[group],
        nominal_pin_diameter_mm=4., nominal_pin_length_mm=16.,
        nominal_clevis_bore_diameter_mm=4.2)
    native_plan['plan_sha256'] = digest(native_plan)
    native = dict(schema='cybrgeo-native-spring-anchor-load-path/1',
                  plan=native_plan, native_anchor_bindings=bindings)
    simulation = dict(options={'springScale': 1.1, 'enforceLimits': True})
    return plan, SimpleNamespace(root=root), control, simulation, native, \
        {'records': placements}, aliases


def test_spring_anchors_use_bound_native_to_physical_body_ownership():
    plan, model, control, simulation, native, placements, aliases = _spring_fixture()
    modeled = {}
    _require_spring_anchor_demand_binding(plan, model, control, simulation,
        native, placements, modeled, body_mapping=aliases)
    assert modeled == pytest.approx({'fixed_anchor': 3.96, 'moving_anchor': 3.96})
    swapped = {'native_parent': 'physical_child',
               'native_child': 'physical_parent'}
    with pytest.raises(QualificationError, match='native demand differs'):
        _require_spring_anchor_demand_binding(plan, model, control, simulation,
            native, placements, body_mapping=swapped)
    with pytest.raises(QualificationError, match='native demand differs'):
        _require_spring_anchor_demand_binding(plan, model, control, simulation,
            native, placements)


def test_design_intent_issues_block_qualified_promotion_before_native_construction(monkeypatch):
    parts = ['a__one', 'b__two']
    plan = _plan(parts, {'ab': _clearance(*parts)})
    intent = dict(schema='cybrgeo-qualification-intent/1',
        parts={name: dict(material='alloy', process='mill',
                          nominal_minimum_wall_mm=2.,
                          required_dimensional_tolerance_mm=.1,
                          maximum_volume_mm3=500.) for name in parts},
        interfaces={'ab': dict(parts=parts, role='clearance',
                               required_service_gap_mm=.2,
                               first_variation_mm=.1, second_variation_mm=.1,
                               force_demand_N=5.)},
        loads={'motor': dict(part=parts[0], source_demand_limit_Nm=2.)},
        unresolved_requirements=[
            dict(category='design', kind='retention', subject='power-loss jaw hold',
                 reason='No passive retention design'),
            dict(category='external', kind='process_test', subject='lot fit',
                 reason='Measured fabrication capability needed')])
    source_intent = QualificationIntentPlan(intent)
    with pytest.raises(QualificationError, match='retention:power-loss jaw hold'):
        source_intent.require_resolved_by(plan)
    entered = []
    monkeypatch.setattr(NativeConstruction, '__init__',
                        lambda *args, **kwargs: entered.append(True))
    poses = {body: [[1., 0., 0., 0.], [0., 1., 0., 0.],
                    [0., 0., 1., 0.], [0., 0., 0., 1.]] for body in ('a', 'b')}
    with pytest.raises(QualificationError, match='retention:power-loss jaw hold'):
        QualifiedNativeConstruction('unused', {'alloy': 2700.},
            qualification_plan=plan, qualification_intent=source_intent,
            initial_body_poses_mm=poses)
    assert entered == []
    intent['unresolved_requirements'] = intent['unresolved_requirements'][1:]
    assert QualificationIntentPlan(intent).require_resolved_by(plan) == plan.document
    # A generic strict plan without source intent keeps its existing entry point.
    QualifiedNativeConstruction('unused', {'alloy': 2700.},
        qualification_plan=plan, initial_body_poses_mm=poses)
    assert entered == [True]
