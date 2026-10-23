"""Synthetic cryptographic fixtures test gates; they are not hardware evidence."""
import base64
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from cryptography.hazmat.primitives import serialization

from mechanism_lab.qualification import (
    QualificationError, QualificationPlan, QualifiedNativeConstruction,
    _check_placed_interface,
    _qualified_body_poses, _qualified_body_mapping, _qualified_body_mapping_receipt,
    _qualified_world_shape, _require_initial_pose_binding,
    _physical_startup_poses_mm,
    _recheck_qualified_native_steps, _require_physical_mass_binding,
    _require_connector_demand_binding, _require_spring_anchor_demand_binding,
    _require_native_spring_anchor_steps,
    _simulated_loads, _verify_evidence,
    release_project,
)
from mechanism_lab.physical_model import digest
from mechanism_lab.spring_anchor import tension_anchor_demand


def _plan():
    return dict(schema='cybrgeo-qualification-plan/1',
                materials={'alloy': dict(grade='TEST-ALLOY', lot='TEST-LOT', density_kg_m3=2700.,
                                         allowable_stress_MPa=100., temperature_limit_C=80.)},
                processes={'mill': dict(method='TEST-MILL', facility='TEST-FACILITY', revision='TEST-1',
                                        allowed_materials=['alloy'], minimum_feature_mm=1.,
                                        capability_tolerance_mm=.1, maximum_part_volume_mm3=1000.)},
                parts={name: dict(material='alloy', process='mill', nominal_minimum_wall_mm=2.,
                                  required_dimensional_tolerance_mm=.1, maximum_volume_mm3=500.)
                       for name in ('fixed__a', 'moving__b')},
                interfaces={'pin_fit': dict(parts=['fixed__a', 'moving__b'], nominal_clearance_mm=.5,
                                            first_tolerance_mm=.1, second_tolerance_mm=.1,
                                            required_clearance_mm=.2, demand_N=5., design_capacity_N=10.)},
                loads={'output': dict(part='moving__b', demand_Nm=1., design_capacity_Nm=2.,
                                      service_temperature_C=50.)},
                assembly_requirements=dict(minimum_endurance_cycles=1000,
                                           minimum_thermal_test_C=60.,
                                           maximum_measured_backlash_mm=.3))


def _initial_poses():
    identity = [[1., 0., 0., 0.], [0., 1., 0., 0.],
                [0., 0., 1., 0.], [0., 0., 0., 1.]]
    return {'fixed': deepcopy(identity), 'moving': deepcopy(identity)}


def test_qualified_initial_pose_contract_rejects_missing_nonrigid_and_stale_frames():
    parts = _plan()['parts']
    valid = _initial_poses()
    assert _qualified_body_poses(valid, parts) == valid
    for fault in ('missing', 'extra', 'reflection', 'scale', 'nonfinite', 'boolean'):
        altered = deepcopy(valid)
        if fault == 'missing':
            del altered['moving']
        elif fault == 'extra':
            altered['unused'] = deepcopy(altered['fixed'])
        elif fault == 'reflection':
            altered['moving'][0][0] = -1.
        elif fault == 'scale':
            altered['moving'][0][0] = 2.
        elif fault == 'nonfinite':
            altered['moving'][0][3] = math.inf
        else:
            altered['moving'][0][0] = True
        with pytest.raises(QualificationError, match='initial pose'):
            _qualified_body_poses(altered, parts)
    record = dict(initial_body_poses_mm=valid, initial_body_pose_sha256=digest(valid))
    assert _require_initial_pose_binding(record, valid, parts) == valid
    moved = deepcopy(valid)
    moved['moving'][0][3] = 10.
    with pytest.raises(QualificationError, match='differs from the physical model'):
        _require_initial_pose_binding(record, moved, parts)
    record['initial_body_poses_mm'] = moved
    with pytest.raises(QualificationError, match='construction receipt'):
        _require_initial_pose_binding(record, valid, parts)


def test_qualified_physical_body_aliases_are_explicit_complete_and_one_to_one():
    parts = _plan()['parts']
    identity = {'fixed': 'fixed', 'moving': 'moving'}
    assert _qualified_body_mapping(None, parts) == identity
    aliases = {'fixed': 'fixed_hardware', 'moving': 'jaw_left'}
    assert _qualified_body_mapping(aliases, parts) == aliases
    record = dict(physical_body_mapping=aliases,
                  physical_body_mapping_sha256=digest(aliases))
    assert _qualified_body_mapping_receipt(record, parts) == aliases
    for invalid in ({'fixed': 'fixed_hardware'},
                    {'fixed': 'jaw_left', 'moving': 'jaw_left'},
                    {'fixed': 'fixed_hardware', 'moving': 'jaw_left', 'extra': 'extra'}):
        with pytest.raises(QualificationError, match='physical body mapping'):
            _qualified_body_mapping(invalid, parts)
    record['physical_body_mapping']['moving'] = 'jaw_right'
    with pytest.raises(QualificationError, match='construction receipt'):
        _qualified_body_mapping_receipt(record, parts)


def test_qualified_pair_uses_common_world_frame_without_moving_local_solid(monkeypatch):
    class PointSolid:
        def __init__(self, x):
            self.x = x

        def transformShape(self, matrix):
            return PointSolid(self.x + matrix[0][3])

        def distance(self, other):
            return abs(self.x - other.x)

    monkeypatch.setitem(sys.modules, 'cadquery', SimpleNamespace(Matrix=lambda value: value))
    poses = _initial_poses()
    poses['moving'][0][3] = -99.5
    fixed, moving = PointSolid(0.), PointSolid(100.)
    world_fixed = _qualified_world_shape(fixed, 'fixed', poses)
    world_moving = _qualified_world_shape(moving, 'moving', poses)
    assert fixed.distance(moving) == 100.
    assert world_fixed.distance(world_moving) == pytest.approx(.5)
    assert moving.x == 100.  # Native STEP and body-local inertia stay untouched.


def test_qualified_initial_pose_matches_actual_mujoco_startup_keyframe():
    pytest.importorskip('mujoco')
    from mechanism_lab.physical_model import PhysicalModel

    physical = PhysicalModel.from_xml('''<mujoco><worldbody>
        <body name="fixed"/><body name="moving" pos="0.01 0 0">
          <joint name="hinge" type="hinge" axis="0 0 1"/>
          <geom type="box" size=".001 .001 .001" mass=".001"/>
        </body></worldbody><keyframe>
        <key name="rotated" qpos="1.5707963267948966"/>
        </keyframe></mujoco>''')
    parts = _plan()['parts']
    default = _physical_startup_poses_mm(physical, {}, parts, None)
    rotated = _physical_startup_poses_mm(physical, {'initial_keyframe': 'rotated'}, parts, None)
    assert default['moving'][0][3] == pytest.approx(10.)
    assert default['moving'][0][0] == pytest.approx(1.)
    assert rotated['moving'][0][0] == pytest.approx(0., abs=1e-12)
    assert rotated['moving'][1][0] == pytest.approx(1.)
    record = dict(initial_body_poses_mm=default,
                  initial_body_pose_sha256=digest(default))
    with pytest.raises(QualificationError, match='differs from the physical model'):
        _require_initial_pose_binding(record, rotated, parts)
    with pytest.raises(QualificationError, match='keyframe is missing'):
        _physical_startup_poses_mm(physical, {'initial_keyframe': 'unknown'}, parts, None)
    alias = {'fixed': 'fixed', 'moving': 'jaw_left'}
    aliased = PhysicalModel.from_xml(physical.xml().replace('name="moving"', 'name="jaw_left"'))
    assert _physical_startup_poses_mm(aliased, {'initial_keyframe': 'rotated'},
                                      parts, alias) == rotated


def test_qualified_static_hardware_keeps_native_mass_without_fake_solver_inertia():
    from mechanism_lab.physical_model import PhysicalModel

    row = dict(mass_kg=1., com_m=[0., 0., 0.],
               inertia_tensor_kg_m2=[[.1, 0., 0.], [0., .1, 0.], [0., 0., .1]])
    model = PhysicalModel.from_xml('''<mujoco><worldbody>
        <body name="base"/><body name="arm"><joint name="pivot" type="hinge"/>
        <inertial mass="1" pos="0 0 0" diaginertia=".1 .1 .1"/>
        </body></worldbody></mujoco>''')
    native = {'fixed': row, 'moving': row}
    aliases = {'fixed': 'base', 'moving': 'arm'}
    model.mass_records = {'arm': deepcopy(row)}
    _require_physical_mass_binding(model, native, aliases)
    model.mass_records.clear()
    with pytest.raises(QualificationError, match='Moving physical body arm'):
        _require_physical_mass_binding(model, native, aliases)
    model.mass_records = {'arm': deepcopy(row)}
    ET.SubElement(model.bodies['base'], 'inertial', mass='1', pos='0 0 0', diaginertia='.1 .1 .1')
    with pytest.raises(QualificationError, match='unbound inertial'):
        _require_physical_mass_binding(model, native, aliases)
    model.bodies['base'].remove(model.bodies['base'].find('inertial'))
    ET.SubElement(model.bodies['base'], 'joint', name='unexpected', type='hinge')
    with pytest.raises(QualificationError, match='Moving physical body base'):
        _require_physical_mass_binding(model, native, aliases)


def _retained_contact_plan():
    plan = _plan()
    plan['interfaces']['pin_fit'] = dict(role='tested_retention_contact',
        parts=['fixed__a', 'moving__b'], nominal_maximum_distance_mm=0.,
        first_tolerance_mm=.1, second_tolerance_mm=.1,
        maximum_service_play_mm=.2, demand_N=5., design_capacity_N=10.)
    return plan


def test_qualified_connector_mapping_needs_known_interface_and_covers_model_demand():
    from types import SimpleNamespace

    plan = _plan()
    plan['solver_connectors'] = {'bearing_joint': 'unknown'}
    with pytest.raises(QualificationError, match='planned physical interface'):
        QualificationPlan(plan)
    plan['solver_connectors']['bearing_joint'] = 'pin_fit'
    qualified = QualificationPlan(plan)
    root = ET.Element('mujoco')
    ET.SubElement(ET.SubElement(root, 'equality'), 'connect',
                  name='bearing_joint', body1='fixed', body2='moving')
    simulation = dict(steps=2, metrics={'constraints': {'bearing_joint': dict(
        type='connect', active_steps=2, missing_row_steps=0,
        incomplete_reaction_row_steps=0,
        reaction_vector_norms={'anchor_force': dict(samples=2,
            peak_absolute=dict(value=4.))})}})
    assert _require_connector_demand_binding(qualified, SimpleNamespace(root=root),
                                             simulation) is None


def _spring_binding_fixture():
    """One non-default-axis source spring with two placed native ends."""
    plan = _plan()
    for name in ('fixed__pin', 'fixed__clevis', 'moving__pin', 'moving__clevis'):
        plan['parts'][name] = deepcopy(plan['parts']['fixed__a'])
    anchors = {}
    native_anchors = []
    bindings = []
    placements = []
    for role, body, position in (('fixed', 'fixed', [0., 0., 2000.]),
                                 ('moving', 'moving', [1000., 0., 0.])):
        interface = f'{role}_anchor'
        plan['interfaces'][interface] = dict(parts=[f'{body}__pin', f'{body}__clevis'],
            role='tested_retention_contact', nominal_maximum_distance_mm=.1,
            first_tolerance_mm=.1, second_tolerance_mm=.1,
            maximum_service_play_mm=.31, demand_N=4.2, design_capacity_N=5.)
        anchors[f'member_0_{role}'] = interface
        native_anchors.append(dict(name=f'eye_{role}', source_spring_index=0,
            member_index=0, role=role, body=body, body_local_anchor_mm=position,
            pin_part='pin', clevis_part='clevis'))
        bindings.append(dict(anchor=f'eye_{role}', source_spring_index=0,
            member_index=0, role=role, body=body, body_local_anchor_mm=position,
            pin_native_part='pin', clevis_native_part='clevis',
            clevis_placed_part='clevis', required_capacity_N=3.,
            pin_diameter_mm=4., pin_length_mm=16., clevis_bore_diameter_mm=4.2,
            measured_nominal_radial_gap_mm=.1, intersection_volume_mm3=0.))
        placements.extend((dict(body=body, name='pin'), dict(body=body, name='clevis')))
    plan['spring_load_paths'] = {'spring_0': dict(joint='j2',
        maximum_service_scale=1.2, uncertainty_factor=1.1, anchors=anchors)}
    config = dict(parent_anchor_m=[0., 0., 2.], child_anchor_m=[1., 0., 0.],
        spring_count=1, free_eye_length_m=0., initial_tension_N=0., stiffness_N_m=1.)
    control = {'springs': [dict(joint='j2', parameters=config, axis=[0., 1., 0.])]}
    root = ET.Element('mujoco')
    ET.SubElement(root, 'compiler', angle='radian', autolimits='true')
    fixed = ET.SubElement(ET.SubElement(root, 'worldbody'), 'body', name='fixed')
    moving = ET.SubElement(fixed, 'body', name='moving')
    ET.SubElement(moving, 'joint', name='j2', type='hinge', axis='0 1 0',
                  range=f'0 {math.pi/2}')
    demand = tension_anchor_demand(config, [0., math.pi/2], pin_diameter_mm=4.,
                                    axis=[0., 1., 0.])
    assert demand['required_capacity_per_separate_pin_N'] == pytest.approx(3.)
    group = dict(source_spring_index=0, control_joint='j2',
        source_parameters_sha256=digest(config), parent_body='fixed',
        child_body='moving', demand=demand, anchors=native_anchors)
    native_plan = dict(schema='cybrgeo-spring-anchor-plan/1', groups=[group],
        nominal_pin_diameter_mm=4., nominal_pin_length_mm=16.,
        nominal_clevis_bore_diameter_mm=4.2)
    native_plan['plan_sha256'] = digest(native_plan)
    native = dict(schema='cybrgeo-native-spring-anchor-load-path/1',
        plan=native_plan, native_anchor_bindings=bindings)
    simulation = dict(options={'springScale': 1.1, 'enforceLimits': True})
    return plan, SimpleNamespace(root=root), control, simulation, native, {'records': placements}


def test_spring_native_load_path_combines_with_connector_and_requires_service_margin():
    plan, physical, control, simulation, native, placements = _spring_binding_fixture()
    qualified = QualificationPlan(plan)
    modeled = {'fixed_anchor': .2}
    _require_spring_anchor_demand_binding(qualified, physical, control, simulation,
                                          native, placements, modeled)
    assert modeled == pytest.approx({'fixed_anchor': 4.16, 'moving_anchor': 3.96})
    with pytest.raises(QualificationError, match='combined modeled force'):
        _require_spring_anchor_demand_binding(qualified, physical, control,
            simulation, native, placements, {'fixed_anchor': .3})


@pytest.mark.parametrize('fault, expected', [
    ('missing_plan', 'Every controlled spring'),
    ('missing_native', 'source-bound native anchor receipt'),
    ('extra_control', 'Every controlled spring'),
    ('wrong_joint_body', 'native pin or clevis is unbound'),
    ('wrong_both_positions', 'differs from its physical joint frame'),
    ('changed_body_frame', 'differs from its physical joint frame'),
    ('unsupported_body_orientation', 'invalid physical force inputs'),
    ('wrong_physical_axis', 'invalid physical force inputs'),
    ('extra_joint', 'one physical hinge degree of freedom'),
    ('degree_joint_range', 'hinge ranges in radians'),
    ('unlimited_hinge', 'invalid physical force inputs'),
    ('limits_disabled', 'requires physical joint limits'),
    ('wrong_pair', 'unrelated native interface'),
    ('clearance_without_retention', 'requires tested retention and play'),
    ('wrong_position', 'native pin or clevis is unbound'),
    ('wrong_plan_pin_diameter', 'native pin diameter differs'),
    ('wrong_plan_pin_length', 'native pin or clevis is unbound'),
    ('wrong_plan_bore', 'native pin or clevis is unbound'),
    ('missing_member_end', 'both native ends'),
    ('over_service_scale', 'exceeds its declared service scale'),
    ('stale_source', 'native demand differs'),
    ('bool_source_index', 'native demand differs'),
])
def test_spring_release_rejects_unbound_or_understated_native_force(fault, expected):
    plan, physical, control, simulation, native, placements = _spring_binding_fixture()
    if fault == 'missing_plan':
        plan['spring_load_paths'].clear()
    elif fault == 'missing_native':
        native = None
    elif fault == 'extra_control':
        control['springs'].append(deepcopy(control['springs'][0]))
    elif fault == 'wrong_joint_body':
        native['plan']['groups'][0]['anchors'][0]['body'] = 'moving'
        native['plan']['plan_sha256'] = digest({k: v for k, v in native['plan'].items()
                                              if k != 'plan_sha256'})
    elif fault == 'wrong_both_positions':
        native['plan']['groups'][0]['anchors'][0]['body_local_anchor_mm'] = [1., 0., 2000.]
        native['native_anchor_bindings'][0]['body_local_anchor_mm'] = [1., 0., 2000.]
        native['plan']['plan_sha256'] = digest({k: v for k, v in native['plan'].items()
                                              if k != 'plan_sha256'})
    elif fault == 'changed_body_frame':
        physical.root.find('.//body/body').set('pos', '.001 0 0')
    elif fault == 'unsupported_body_orientation':
        physical.root.find('.//body/body').set('euler', '0 0 90')
    elif fault == 'wrong_physical_axis':
        physical.root.find('.//joint').set('axis', '0 0 1')
    elif fault == 'extra_joint':
        ET.SubElement(physical.root.find('.//body/body'), 'joint', name='other')
    elif fault == 'degree_joint_range':
        physical.root.find('./compiler').set('angle', 'degree')
    elif fault == 'unlimited_hinge':
        physical.root.find('.//joint').set('limited', 'false')
    elif fault == 'limits_disabled':
        simulation['options']['enforceLimits'] = False
    elif fault == 'wrong_pair':
        plan['interfaces']['fixed_anchor']['parts'] = ['fixed__pin', 'fixed__a']
    elif fault == 'clearance_without_retention':
        plan['interfaces']['fixed_anchor'] = dict(parts=['fixed__pin', 'fixed__clevis'],
            nominal_clearance_mm=.5, first_tolerance_mm=.1, second_tolerance_mm=.1,
            required_clearance_mm=.2, demand_N=4.2, design_capacity_N=5.)
    elif fault == 'wrong_position':
        native['native_anchor_bindings'][0]['body_local_anchor_mm'] = [0., 0., 0.]
    elif fault in ('wrong_plan_pin_diameter', 'wrong_plan_pin_length', 'wrong_plan_bore'):
        field = {'wrong_plan_pin_diameter': 'nominal_pin_diameter_mm',
                 'wrong_plan_pin_length': 'nominal_pin_length_mm',
                 'wrong_plan_bore': 'nominal_clevis_bore_diameter_mm'}[fault]
        native['plan'][field] += .1
        native['plan']['plan_sha256'] = digest({k: v for k, v in native['plan'].items()
                                              if k != 'plan_sha256'})
    elif fault == 'missing_member_end':
        native['plan']['groups'][0]['anchors'].pop()
        native['plan']['plan_sha256'] = digest({k: v for k, v in native['plan'].items()
                                              if k != 'plan_sha256'})
    elif fault == 'over_service_scale':
        simulation['options']['springScale'] = 1.3
    elif fault == 'stale_source':
        control['springs'][0]['parameters']['stiffness_N_m'] = 2.
    elif fault == 'bool_source_index':
        native['plan']['groups'][0]['source_spring_index'] = False
        native['plan']['plan_sha256'] = digest({k: v for k, v in native['plan'].items()
                                              if k != 'plan_sha256'})
    qualified = QualificationPlan(plan)
    with pytest.raises(QualificationError, match=expected):
        _require_spring_anchor_demand_binding(qualified, physical, control,
                                              simulation, native, placements, {})


def test_spring_plan_rejects_missing_member_side_or_invalid_service_bound():
    plan, _, _, _, _, _ = _spring_binding_fixture()
    QualificationPlan(plan)
    for value in (0., .99, float('inf'), float('nan')):
        for field in ('maximum_service_scale', 'uncertainty_factor'):
            altered = deepcopy(plan)
            altered['spring_load_paths']['spring_0'][field] = value
            with pytest.raises(QualificationError):
                QualificationPlan(altered)
    altered = deepcopy(plan)
    del altered['spring_load_paths']['spring_0']['anchors']['member_0_moving']
    with pytest.raises(QualificationError, match='both ends'):
        QualificationPlan(altered)


def test_source_spring_force_never_substitutes_for_tested_pin_capacity(tmp_path):
    package, policy, _, _, _ = _signed_fixture(tmp_path)
    plan, physical, control, simulation, native, placements = _spring_binding_fixture()
    _require_spring_anchor_demand_binding(QualificationPlan(plan), physical,
        control, simulation, native, placements, {})
    # The synthetic hardware records cover neither spring-end part serials nor
    # capacity tests. A valid native force model cannot supply those records.
    with pytest.raises(QualificationError, match='cover every required subject'):
        _verify_evidence(package, policy, 'a'*64, plan,
                         {part: 'b'*64 for part in plan['parts']})


def test_native_spring_anchor_rechecks_placed_brep_size_and_bore():
    import cadquery as cq

    pin = cq.Workplane('XY').circle(2.).extrude(16.).translate((0, 0, -8)).val()
    clevis = cq.Workplane('XY').circle(6.).circle(2.1).extrude(4.).translate((0, 0, -2)).val()
    solids = {'fixed__pin': pin, 'fixed__clevis': clevis}
    row = dict(body='fixed', pin_native_part='pin', clevis_placed_part='clevis',
               body_local_anchor_mm=[0., 0., 0.], pin_diameter_mm=4.,
               pin_length_mm=16., clevis_bore_diameter_mm=4.2,
               measured_nominal_radial_gap_mm=.1, intersection_volume_mm3=0.)
    _require_native_spring_anchor_steps(solids, {'native_anchor_bindings': [row]})
    wrong_diameter = deepcopy(row)
    wrong_diameter['pin_diameter_mm'] = 3.9
    with pytest.raises(QualificationError, match='STEP fails its source dimensions'):
        _require_native_spring_anchor_steps(solids,
                                            {'native_anchor_bindings': [wrong_diameter]})
    wrong_gap = deepcopy(row)
    wrong_gap['measured_nominal_radial_gap_mm'] = .2
    with pytest.raises(QualificationError, match='receipt differs'):
        _require_native_spring_anchor_steps(solids,
                                            {'native_anchor_bindings': [wrong_gap]})


def _signed_fixture(tmp_path, *, basis='a'*64, step_hashes=None):
    package, authority = tmp_path/'evidence', tmp_path/'authority'
    package.mkdir()
    authority.mkdir()
    private = Ed25519PrivateKey.generate()
    public = private.public_key().public_bytes(serialization.Encoding.Raw, serialization.PublicFormat.Raw)
    policy = authority/'trusted-issuers.json'
    policy.write_text(json.dumps(dict(schema='cybrgeo-trust-policy/1', issuers={
        'test-only': dict(public_key_base64=base64.b64encode(public).decode(),
                          allowed_kinds=['system_service_envelope', 'material', 'process', 'part_inspection', 'interface_test',
                                         'load_test', 'assembly_test'])})), encoding='utf-8')
    plan = _plan()
    step_hashes = step_hashes or {'fixed__a': 'b'*64, 'moving__b': 'c'*64}
    rows = [
        ('system_service_envelope', 'complete_assembly', dict(load_paths_reviewed=True,
            mechanical_load_paths={'output': {'parts': ['moving__b', 'fixed__a'],
                                              'interfaces': ['pin_fit']}},
            actuator_demand_Nm={'output': 1.}, interface_demand_N={'pin_fit': 5.},
            required_thermal_test_C=60., required_endurance_cycles=1000,
            maximum_backlash_mm=.3)),
        ('material', 'alloy', dict(grade='TEST-ALLOY', lot='TEST-LOT', density_kg_m3=2700.,
                                   allowable_stress_MPa=100., temperature_limit_C=80.)),
        ('process', 'mill', dict(method='TEST-MILL', facility='TEST-FACILITY', revision='TEST-1',
                                 minimum_feature_mm=1., capability_tolerance_mm=.1,
                                 maximum_part_volume_mm3=1000.)),
        ('part_inspection', 'fixed__a', dict(material_lot='TEST-LOT', process_revision='TEST-1',
                                             process_facility='TEST-FACILITY', process_method='TEST-MILL',
                                             fabricated_serial='TEST-A', measured_minimum_wall_mm=2.,
                                             maximum_dimensional_error_mm=.05, step_sha256=step_hashes['fixed__a'])),
        ('part_inspection', 'moving__b', dict(material_lot='TEST-LOT', process_revision='TEST-1',
                                              process_facility='TEST-FACILITY', process_method='TEST-MILL',
                                              fabricated_serial='TEST-B', measured_minimum_wall_mm=2.,
                                              maximum_dimensional_error_mm=.05, step_sha256=step_hashes['moving__b'])),
        ('interface_test', 'pin_fit', dict(assembled_serial='TEST-ASSEMBLY',
                                           part_serials={'fixed__a': 'TEST-A', 'moving__b': 'TEST-B'},
                                           measured_minimum_clearance_mm=.3, tested_capacity_N=10.)),
        ('load_test', 'output', dict(assembled_serial='TEST-ASSEMBLY', part_serial='TEST-B',
                                      tested_capacity_Nm=2., tested_temperature_C=60.)),
        ('assembly_test', 'complete_assembly', dict(assembled_serial='TEST-ASSEMBLY', fit_passed=True,
                                                    part_serials={'fixed__a': 'TEST-A', 'moving__b': 'TEST-B'},
                                                    motion_passed=True, thermal_passed=True,
                                                    tested_cycles=1000., tested_temperature_C=70.,
                                                    measured_backlash_mm=.1)),
    ]
    evidence = []
    for index, (kind, subject, measurements) in enumerate(rows):
        name = f'test-record-{index}.txt'
        payload = f'TEST ONLY: {kind}/{subject}\n'.encode()
        (package/name).write_bytes(payload)
        row = dict(schema='cybrgeo-signed-evidence/1', kind=kind, subject=subject,
                   basis_sha256=basis, result='observed_pass', measurements=measurements,
                   document=name, document_sha256=hashlib.sha256(payload).hexdigest(), issuer='test-only')
        serialized = json.dumps(row, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
        row['signature_base64'] = base64.b64encode(private.sign(serialized)).decode()
        evidence.append(row)
    (package/'evidence.json').write_text(json.dumps(dict(schema='cybrgeo-signed-evidence-set/1',
                                                         records=evidence)), encoding='utf-8')
    return package, policy, plan, evidence, private


def _resign(package, records, private):
    for row in records:
        signed = {name: value for name, value in row.items() if name != 'signature_base64'}
        row['signature_base64'] = base64.b64encode(private.sign(
            json.dumps(signed, sort_keys=True, separators=(',', ':'), allow_nan=False).encode())).decode()
    (package/'evidence.json').write_text(json.dumps(dict(schema='cybrgeo-signed-evidence-set/1',
                                                         records=records)), encoding='utf-8')


def _check(package, policy, plan):
    return _verify_evidence(package, policy, 'a'*64, plan, {'fixed__a': 'b'*64, 'moving__b': 'c'*64})


def test_plan_rejects_unsupported_material_process_fit_and_load():
    QualificationPlan(_plan())
    mutations = [
        ('unsupported material/process', lambda p: p['processes']['mill']['allowed_materials'].clear()),
        ('wall below process limit', lambda p: p['parts']['fixed__a'].update(nominal_minimum_wall_mm=.5)),
        ('required tolerance below process capability', lambda p: p['parts']['fixed__a'].update(required_dimensional_tolerance_mm=.05)),
        ('closed worst-case fit', lambda p: p['interfaces']['pin_fit'].update(first_tolerance_mm=.4)),
        ('part variation exceeds fit allowance', lambda p: p['parts']['fixed__a'].update(required_dimensional_tolerance_mm=.15)),
        ('interface capacity below demand', lambda p: p['interfaces']['pin_fit'].update(design_capacity_N=4.)),
        ('load capacity below demand', lambda p: p['loads']['output'].update(design_capacity_Nm=.5)),
        ('service temperature above material limit', lambda p: p['loads']['output'].update(service_temperature_C=90.)),
    ]
    for _, mutate in mutations:
        plan = _plan()
        mutate(plan)
        with pytest.raises(QualificationError):
            QualificationPlan(plan)


def test_retained_contact_is_distinct_from_clearance_and_bounds_part_variation():
    plan = _retained_contact_plan()
    QualificationPlan(plan)
    fit = _check_placed_interface('pin_fit', plan['interfaces']['pin_fit'], 0., 0.)
    assert fit == dict(role='tested_retention_contact', worst_case_closest_gap_mm=.2)
    with pytest.raises(QualificationError, match='contact proximity/play'):
        _check_placed_interface('pin_fit', plan['interfaces']['pin_fit'], .01, 0.)
    with pytest.raises(QualificationError, match='overlap'):
        _check_placed_interface('pin_fit', plan['interfaces']['pin_fit'], 0., .01)
    changed = deepcopy(plan)
    changed['interfaces']['pin_fit']['maximum_service_play_mm'] = .19
    with pytest.raises(QualificationError, match='tolerance or load'):
        QualificationPlan(changed)
    changed = deepcopy(plan)
    changed['interfaces']['pin_fit'].pop('role')
    with pytest.raises(QualificationError, match='fields must be exactly'):
        QualificationPlan(changed)
    # The legacy clearance mode still rejects nominal zero with real positive
    # part variation; it cannot borrow the contact role's physical-test path.
    changed = _plan()
    changed['interfaces']['pin_fit'].update(nominal_clearance_mm=0., required_clearance_mm=0.)
    with pytest.raises(QualificationError, match='tolerance or load'):
        QualificationPlan(changed)


@pytest.mark.parametrize('fault', ('none', 'missing_retention', 'failed_retention',
                                   'missing_play', 'excess_play', 'weak_capacity',
                                   'wrong_serial', 'stale_basis'))
def test_retained_contact_needs_serial_bound_physical_retention_and_play(tmp_path, fault):
    package, policy, _old_plan, records, private = _signed_fixture(tmp_path)
    plan = _retained_contact_plan()
    contact = records[5]['measurements']
    contact.pop('measured_minimum_clearance_mm')
    contact.update(retention_passed_under_load=True, measured_maximum_play_mm=.1)
    if fault == 'missing_retention': contact.pop('retention_passed_under_load')
    elif fault == 'failed_retention': contact['retention_passed_under_load'] = False
    elif fault == 'missing_play': contact.pop('measured_maximum_play_mm')
    elif fault == 'excess_play': contact['measured_maximum_play_mm'] = .3
    elif fault == 'weak_capacity': contact['tested_capacity_N'] = 9.
    elif fault == 'wrong_serial': contact['part_serials']['fixed__a'] = 'OTHER'
    elif fault == 'stale_basis': records[5]['basis_sha256'] = '0'*64
    _resign(package, records, private)
    if fault == 'none':
        assert _check(package, policy, plan)['evidence_count'] == 8
    else:
        with pytest.raises(QualificationError):
            _check(package, policy, plan)


def test_plan_rejects_cross_platform_step_name_collisions():
    for alias in ('fixed__A', 'CON__spacer', 'fixed__NUL', 'fixed__a.', 'fixed__a__rev'):
        plan = _plan()
        plan['parts'][alias] = deepcopy(plan['parts']['fixed__a'])
        with pytest.raises(QualificationError):
            QualificationPlan(plan)


def test_only_active_rack_motor_is_a_load_case():
    assert set(_simulated_loads({'actuators': {'output': {'peak_output_torque_Nm': 1.}},
                                 'rack_motor': None})) == {'output'}
    assert set(_simulated_loads({'actuators': {'output': {'peak_output_torque_Nm': 1.}},
                                 'rack_motor': {'peak_output_torque_Nm': 2.}})) == {'output', 'rack_motor'}
    with pytest.raises(QualificationError):
        _simulated_loads({'actuators': {}, 'rack_motor': 'unsupported'})


def test_plain_diagnostic_builder_cannot_mint_qualification_receipt(tmp_path):
    from mechanism_lab.native_construction import NativeConstruction
    plain = NativeConstruction(tmp_path/'diagnostic', {'alloy': 2700.})
    forged = {'schema': 'cybrgeo-qualification-construction/1', 'plan_sha256': '0'*64}
    with pytest.raises(ValueError, match='reserved for the qualified builder'):
        plain.write_json('qualification-construction.json', forged)
    with pytest.raises(ValueError, match='requires the qualified builder'):
        plain.finish('diagnostic only', extra_records={'qualification-construction.json': forged})
    assert not (plain.output/'qualification-construction.json').exists()
    assert not (plain.output/'native-build.json').exists()


def test_policy_blocks_wrong_part_before_native_export(tmp_path, monkeypatch):
    class Solid:
        def isValid(self): return True
        def Solids(self): return [self]
        def Volume(self): return 10.
    monkeypatch.setattr('mechanism_lab.qualification._shape', lambda shape: shape)
    reached = []
    monkeypatch.setattr('mechanism_lab.native_construction.NativeConstruction.place',
                        lambda *args, **kwargs: reached.append(args))
    constructor = QualifiedNativeConstruction(tmp_path/'native', {'alloy': 2700.},
                                               qualification_plan=_plan(),
                                               initial_body_poses_mm=_initial_poses())
    with pytest.raises(QualificationError, match='lacks its exact'):
        constructor.place('fixed', 'a', Solid(), 'wrong-material')
    with pytest.raises(QualificationError, match='lacks its exact'):
        constructor.place('fixed', 'unknown', Solid(), 'alloy')
    with pytest.raises(QualificationError, match='portable body__part'):
        constructor.place('fixed', 'a__subpart', Solid(), 'alloy')
    with pytest.raises(QualificationError, match='portable body__part'):
        constructor.place('fixed__a', 'subpart', Solid(), 'alloy')
    with pytest.raises(QualificationError, match='unbound CAD templates'):
        constructor.addpart('unbound', Solid(), 'alloy')
    with pytest.raises(QualificationError, match='unplaced or overridden'):
        constructor.add_mass_record('fixed', {})
    assert reached == []
    constructor.place('fixed', 'a', Solid(), 'alloy')
    with pytest.raises(QualificationError, match='already exported'):
        constructor.place('fixed', 'a', Solid(), 'alloy')
    assert len(reached) == 1


def test_signed_evidence_requires_external_authority_and_full_coverage(tmp_path):
    package, policy, plan, records, private = _signed_fixture(tmp_path)
    assert _check(package, policy, plan)['evidence_count'] == 8
    (package/'evidence.json').write_text(json.dumps(dict(schema='cybrgeo-signed-evidence-set/1',
                                                         records=records[:-1])), encoding='utf-8')
    with pytest.raises(QualificationError, match='cover every required'):
        _check(package, policy, plan)
    _resign(package, records, private)
    wrong_policy = tmp_path/'authority'/'untrusted.json'
    wrong_policy.write_text(json.dumps(dict(schema='cybrgeo-trust-policy/1', issuers={})), encoding='utf-8')
    with pytest.raises(QualificationError):
        _check(package, wrong_policy, plan)


def test_signed_service_envelope_cannot_understate_combined_source_force(tmp_path):
    package, policy, plan, _, _ = _signed_fixture(tmp_path)
    step_hashes = {'fixed__a': 'b'*64, 'moving__b': 'c'*64}
    assert _verify_evidence(package, policy, 'a'*64, plan, step_hashes,
                            {'pin_fit': 5.})['evidence_count'] == 8
    with pytest.raises(QualificationError, match='understates modeled interface force'):
        _verify_evidence(package, policy, 'a'*64, plan, step_hashes,
                         {'pin_fit': 5.01})


@pytest.mark.parametrize('fault', ['stale_basis', 'forged_signature', 'material_lot',
                                   'process_revision', 'process_facility', 'process_method',
                                   'bad_fit', 'weak_interface_capacity', 'low_load', 'wrong_step', 'low_cycles',
                                   'wrong_assembly_bom', 'wrong_interface_unit',
                                   'wrong_interface_part', 'wrong_load_unit', 'wrong_load_part',
                                   'understated_service_torque', 'unreviewed_load_path',
                                   'missing_mechanical_path', 'disconnected_mechanical_path'])
def test_signed_evidence_rejects_stale_forged_or_failed_measurements(tmp_path, fault):
    package, policy, plan, records, private = _signed_fixture(tmp_path)
    modified = deepcopy(records)
    if fault == 'stale_basis': modified[0]['basis_sha256'] = '0'*64
    elif fault == 'forged_signature': modified[0]['signature_base64'] = records[1]['signature_base64']
    elif fault == 'material_lot': modified[1]['measurements']['lot'] = 'OTHER'
    elif fault == 'process_revision': modified[2]['measurements']['revision'] = 'OTHER'
    elif fault == 'process_facility': modified[3]['measurements']['process_facility'] = 'OTHER'
    elif fault == 'process_method': modified[4]['measurements']['process_method'] = 'OTHER'
    elif fault == 'bad_fit': modified[5]['measurements']['measured_minimum_clearance_mm'] = .1
    elif fault == 'weak_interface_capacity': modified[5]['measurements']['tested_capacity_N'] = 6.
    elif fault == 'low_load': modified[6]['measurements']['tested_capacity_Nm'] = 1.5
    elif fault == 'wrong_step': modified[3]['measurements']['step_sha256'] = '0'*64
    elif fault == 'low_cycles': modified[7]['measurements']['tested_cycles'] = 999
    elif fault == 'wrong_assembly_bom': modified[7]['measurements']['part_serials']['moving__b'] = 'OTHER'
    elif fault == 'wrong_interface_unit': modified[5]['measurements']['assembled_serial'] = 'OTHER'
    elif fault == 'wrong_interface_part': modified[5]['measurements']['part_serials']['moving__b'] = 'OTHER'
    elif fault == 'wrong_load_unit': modified[6]['measurements']['assembled_serial'] = 'OTHER'
    elif fault == 'wrong_load_part': modified[6]['measurements']['part_serial'] = 'OTHER'
    elif fault == 'understated_service_torque': modified[0]['measurements']['actuator_demand_Nm']['output'] = 1.5
    elif fault == 'unreviewed_load_path': modified[0]['measurements']['load_paths_reviewed'] = False
    elif fault == 'missing_mechanical_path': modified[0]['measurements']['mechanical_load_paths'] = {}
    elif fault == 'disconnected_mechanical_path': modified[0]['measurements']['mechanical_load_paths']['output']['parts'].reverse()
    if fault != 'forged_signature': _resign(package, modified, private)
    else: (package/'evidence.json').write_text(json.dumps(dict(schema='cybrgeo-signed-evidence-set/1',
                                                               records=modified)), encoding='utf-8')
    with pytest.raises(QualificationError):
        _check(package, policy, plan)


def test_examples_cannot_release_without_preconstruction_policy(tmp_path):
    # Current authored examples intentionally lack supplier lot, process and
    # fabricated-hardware evidence. They must not create a product release.
    from mechanism_lab.engineering_source import project_directory
    source = project_directory('fourbar_lift')
    with pytest.raises(QualificationError, match='qualification construction plan'):
        release_project(source, tmp_path/'run', tmp_path/'evidence', tmp_path/'trust.json',
                        tmp_path/'product')
    assert not (tmp_path/'product').exists()


@pytest.mark.parametrize('case,limit,duration,expected', [
    ('drive', 'belt', .001, 'demanded torque beyond its belt model limit'),
    ('drive', 'gear', .001, 'demanded torque beyond its gear model limit'),
    ('drive', 'belt', None, 'demand-limit'),
    ('drive', 'gear', None, 'demand-limit'),
    ('rack', 'output', .001, 'source output model limit'),
    ('rack', 'output', None, 'output demand-limit telemetry'),
    ('rack', 'output', 0., None),
    ('unmapped_connector', 'belt', 0., 'lack exact planned interface mapping'),
    ('missing_connector', 'belt', 0., 'inventory differs'),
    ('zero_step_connector', 'belt', 0., 'Modeled connector demands require integration-step count'),
    ('incomplete_connector', 'belt', 0., 'reaction coverage is incomplete'),
    ('understated_connector', 'belt', 0., 'understates modeled connector force'),
    ('unrelated_connector', 'belt', 0., 'unrelated native interface bodies'),
    ('missing_body_endpoint_connector', 'belt', 0., 'requires two distinct named body endpoints'),
    ('retained_contact', 'belt', 0., None),
    ('forged_contact_fit', 'belt', 0., 'fails its exact part-pair plan'),
])
def test_release_rejects_unbounded_drivetrain_or_connector_demand(
        tmp_path, monkeypatch, case, limit, duration, expected):
    """Clipped drivetrain demand or incomplete connector demand cannot release.

    Only unrelated upstream CAD/source checks are stubbed. The release method
    reads persisted metrics; a clean rack reaches downstream release basis
    hashing without needing rotary belt/gear fields. No product is created.
    """
    from types import SimpleNamespace
    from mechanism_lab import engineering_source, physical_model, qualification
    from mechanism_lab.causal_runtime import _runtime_identity
    from mechanism_lab.physical_model import digest

    plan = (_retained_contact_plan() if case in ('retained_contact', 'forged_contact_fit')
            else _plan())
    connector_case = case.endswith('_connector')
    if connector_case and case != 'unmapped_connector':
        plan['solver_connectors'] = {'bearing_joint': 'pin_fit'}
    if case == 'rack':
        plan['loads']['rack_motor'] = dict(part='moving__b', demand_Nm=1.,
                                          design_capacity_Nm=2., service_temperature_C=50.)
    source = tmp_path/'project'
    (source/'data').mkdir(parents=True)
    plan_path = source/'data'/'qualification-plan.json'
    plan_path.write_text(json.dumps(plan), encoding='utf-8')
    source_files = {'data/qualification-plan.json': hashlib.sha256(plan_path.read_bytes()).hexdigest()}
    native = tmp_path/'native'
    native.mkdir()
    pair_fit = dict(parts=['fixed__a', 'moving__b'], intersection_volume_mm3=0.,
                    minimum_distance_mm=.5, worst_case_clearance_mm=.3)
    if case in ('retained_contact', 'forged_contact_fit'):
        pair_fit = dict(parts=['fixed__a', 'moving__b'], intersection_volume_mm3=0.,
                        minimum_distance_mm=0., role='tested_retention_contact',
                        worst_case_closest_gap_mm=.2 if case == 'retained_contact' else .3)
    qualified_record = dict(schema='cybrgeo-qualification-construction/2',
                            plan_sha256=QualificationPlan(plan).sha256,
                            qualification_service_sha256=qualification._sha(qualification.__file__),
                            placements=['fixed__a', 'moving__b'], interface_names=['pin_fit'],
                            initial_body_poses_mm=_initial_poses(),
                            initial_body_pose_sha256=digest(_initial_poses()),
                            physical_body_mapping={'fixed': 'fixed', 'moving': 'moving'},
                            physical_body_mapping_sha256=digest({'fixed': 'fixed', 'moving': 'moving'}),
                            exact_placed_interface_checks={'pin_fit': pair_fit})
    record_path = native/'qualification-construction.json'
    record_path.write_text(json.dumps(qualified_record), encoding='utf-8')
    receipt = dict(files_sha256={'qualification-construction.json': hashlib.sha256(
                       record_path.read_bytes()).hexdigest()},
                   material_densities_kg_m3={'alloy': 2700.})
    (native/'placed-native-parts.json').write_text(json.dumps(dict(units='mm', records=[
        dict(body='fixed', name='a'), dict(body='moving', name='b')])), encoding='utf-8')
    (native/'native-interface-requirements.json').write_text(json.dumps(dict(
        passed=True, checks=[dict(name='pin_fit')])), encoding='utf-8')
    binding = {'schema': 'test-only-source-binding'}
    run = tmp_path/'run'
    (run/'simulation').mkdir(parents=True)
    model = {'schema': 'test-only-physical', 'mass_records': {}}
    control = {'schema': 'test-only-controller'}
    metrics = {'peak_output_torque_Nm': .5,
               'belt_demand_limit_s': 0., 'gear_demand_limit_s': 0.}
    rack_metrics = None
    if case == 'rack':
        rack_metrics = {'peak_output_torque_Nm': .2}
        if duration is not None:
            rack_metrics['output_demand_limit_s'] = duration
    elif case == 'drive' and duration is None:
        del metrics[f'{limit}_demand_limit_s']
    elif case == 'drive':
        metrics[f'{limit}_demand_limit_s'] = duration
    reaction = dict(type='connect', active_steps=2, missing_row_steps=0,
        incomplete_reaction_row_steps=0,
        reaction_vector_norms={'anchor_force': dict(samples=2,
            peak_absolute=dict(value=6. if case == 'understated_connector' else 4.))})
    constraints = ({} if case == 'missing_connector' else
                   {'bearing_joint': reaction} if connector_case else {})
    if case == 'incomplete_connector':
        reaction['missing_row_steps'] = 1
    root = ET.Element('mujoco')
    world = ET.SubElement(root, 'worldbody')
    fixed_body = ET.SubElement(world, 'body', name='fixed')
    moving_body = ET.SubElement(world, 'body', name='moving')
    ET.SubElement(moving_body, 'joint', name='motion', type='hinge')
    if connector_case:
        attributes = dict(name='bearing_joint', body1=(
            'other' if case == 'unrelated_connector' else 'fixed'))
        if case != 'missing_body_endpoint_connector':
            attributes['body2'] = 'moving'
        ET.SubElement(ET.SubElement(root, 'equality'), 'connect', **attributes)
    simulation = dict(schema='cybrgeo-causal-simulation/1',
                      model_sha256=digest(model), control_sha256=digest(control),
                      runtime_implementation=_runtime_identity(), mujoco_version='test-only',
                      options={}, steps=2, metrics={'actuators': {'output': metrics},
                                                    'rack_motor': rack_metrics,
                                                    'constraints': constraints})
    if case == 'zero_step_connector':
        simulation['steps'] = 0
    (run/'physical-source.json').write_text(json.dumps(model), encoding='utf-8')
    (run/'control-source.json').write_text(json.dumps(control), encoding='utf-8')
    (run/'simulation'/'simulation.json').write_text(json.dumps(simulation), encoding='utf-8')
    report = dict(schema='cybrgeo-engineering-result/2', name='synthetic', passed=True,
                  source_files=source_files, native={'directory': str(native)},
                  native_build_sha256=digest(receipt),
                  native_source_binding_sha256=digest(binding),
                  model_sha256=digest(model), control_sha256=digest(control),
                  simulation=simulation)
    (run/'engineering.json').write_text(json.dumps(report), encoding='utf-8')

    monkeypatch.setattr(engineering_source, 'project_directory', lambda _: source)
    monkeypatch.setattr(engineering_source, 'load_project', lambda _: (
        SimpleNamespace(name='synthetic', specification={}), source_files))
    monkeypatch.setattr(engineering_source, 'verify_native_build',
                        lambda *args, **kwargs: receipt)
    monkeypatch.setattr(engineering_source, 'verify_native_source_binding',
                        lambda *args, **kwargs: binding)
    # The release now binds exact native mass to moving physical bodies before
    # checking demand telemetry. Give this synthetic downstream-load fixture a
    # valid CAD/physical mass pair; never stub out the production mass gate.
    matched_mass = dict(mass_kg=1., com_m=[0., 0., 0.],
                        inertia_tensor_kg_m2=[[.1, 0., 0.], [0., .1, 0.], [0., 0., .1]])
    monkeypatch.setattr(qualification, '_recheck_qualified_native_steps',
                        lambda *args, **kwargs: {'fixed': deepcopy(matched_mass),
                                                  'moving': deepcopy(matched_mass)})
    monkeypatch.setattr(qualification, '_physical_startup_poses_mm',
                        lambda model, control, parts, mapping: _initial_poses())
    monkeypatch.setattr(physical_model.PhysicalModel, 'from_source',
                        lambda *_: SimpleNamespace(mass_records={'moving': deepcopy(matched_mass)},
                                                   bodies={'fixed': fixed_body, 'moving': moving_body},
                                                   root=root))
    class LoadGatePassed(Exception):
        pass
    original_sha = qualification._sha
    def observe_after_load_gate(path):
        if Path(path).name == 'trace.json.gz':
            raise LoadGatePassed
        return original_sha(path)
    monkeypatch.setattr(qualification, '_sha', observe_after_load_gate)
    monkeypatch.setattr(qualification, '_verify_evidence',
                        lambda *args, **kwargs: pytest.fail('Load gate was bypassed'))
    product = tmp_path/'product'
    with pytest.raises(LoadGatePassed if expected is None else QualificationError,
                       match=expected):
        release_project(source, run, tmp_path/'evidence', tmp_path/'trusted-policy.json', product)
    assert not product.exists()
    assert not list(tmp_path.glob('.product-stage-*'))


def test_signed_release_copies_real_qualified_native_steps_atomically(tmp_path, monkeypatch):
    """Synthetic signed reports exercise the real native constructor and release copy.

    The fake engineering run is a unit-test stand-in, never a qualified robot.
    It retains real CadQuery STEP and completion-file hashes while mocking only
    the causal simulation/source-loader services covered elsewhere.
    """
    import cadquery as cq
    import mechanism_lab.qualification as qualification
    import mechanism_lab.engineering_source as engineering_source
    from mechanism_lab.native_construction import NativeInterfaceRequirements
    from mechanism_lab.physical_model import digest
    from mechanism_lab.causal_runtime import _runtime_identity
    from types import SimpleNamespace

    plan = _plan()
    project_dir = tmp_path/'project'
    (project_dir/'data').mkdir(parents=True)
    plan_file = project_dir/'data'/'qualification-plan.json'
    plan_file.write_text(json.dumps(plan), encoding='utf-8')
    source_files = {'data/qualification-plan.json': hashlib.sha256(plan_file.read_bytes()).hexdigest()}
    native = tmp_path/'native'
    builder = QualifiedNativeConstruction(native, {'alloy': 2700.},
        qualification_plan=plan, initial_body_poses_mm=_initial_poses(),
        provenance={'input_spec_sha256': digest({'test': True})})
    first = cq.Workplane('XY').box(4, 4, 4).translate((-2.25, 0, 0))
    second = cq.Workplane('XY').box(4, 4, 4).translate((2.25, 0, 0))
    builder.place('fixed', 'a', first, 'alloy')
    builder.place('moving', 'b', second, 'alloy')
    placed_step = native/'cad'/'placed'/'fixed__a.step'
    original_step = placed_step.read_bytes()
    cq.exporters.export(first.translate((20, 0, 0)), str(placed_step))
    with pytest.raises(QualificationError, match='differs from constructed CAD'):
        builder.finish('test-only native mass')
    placed_step.write_bytes(original_step)
    spoofed = NativeInterfaceRequirements()
    spoofed.clearance('pin_fit', first, second.translate((100, 0, 0)), minimum_distance_mm=.5)
    with pytest.raises(QualificationError, match='derives interface measurements'):
        builder.finish('test-only native mass', native_requirements=spoofed)
    builder.placed_records.append(deepcopy(builder.placed_records[0]))
    with pytest.raises(QualificationError, match='inventory is ambiguous'):
        builder.finish('test-only native mass')
    builder.placed_records.pop()
    assert not (native/'qualification-construction.json').exists()
    builder.finish('test-only native mass')
    receipt = json.loads((native/'native-build.json').read_text(encoding='utf-8'))
    placed_document = json.loads((native/'placed-native-parts.json').read_text(encoding='utf-8'))
    fit_document = json.loads((native/'native-interface-requirements.json').read_text(encoding='utf-8'))
    construction_record = json.loads((native/'qualification-construction.json').read_text(encoding='utf-8'))
    bad_material_placements = deepcopy(placed_document)
    bad_material_placements['records'][0]['material'] = 'other-material'
    with pytest.raises(QualificationError, match='material differs from plan'):
        _recheck_qualified_native_steps(native, bad_material_placements, fit_document,
                                        receipt, plan, construction_record,
                                        startup_poses=_initial_poses())
    forged_pair_receipt = deepcopy(construction_record)
    forged_pair_receipt['exact_placed_interface_checks']['pin_fit']['minimum_distance_mm'] = 5.
    with pytest.raises(QualificationError, match='independently measured fit'):
        _recheck_qualified_native_steps(native, placed_document, fit_document,
                                        receipt, plan, forged_pair_receipt,
                                        startup_poses=_initial_poses())
    real_mass = json.loads((native/'mass_properties.json').read_text(encoding='utf-8'))
    lighter_aggregate = deepcopy(real_mass)
    lighter_aggregate['moving']['mass_kg'] *= .5
    original_mass_bytes = (native/'mass_properties.json').read_bytes()
    (native/'mass_properties.json').write_text(json.dumps(lighter_aggregate), encoding='utf-8')
    changed_receipt = deepcopy(receipt)
    changed_receipt['files_sha256']['mass_properties.json'] = hashlib.sha256(
        (native/'mass_properties.json').read_bytes()).hexdigest()
    with pytest.raises(QualificationError, match='aggregate mass_kg differs'):
        _recheck_qualified_native_steps(native, placed_document, fit_document,
                                        changed_receipt, plan, construction_record,
                                        startup_poses=_initial_poses())
    (native/'mass_properties.json').write_bytes(original_mass_bytes)
    fake_model = SimpleNamespace(mass_records=deepcopy(real_mass))
    _require_physical_mass_binding(fake_model, real_mass)
    fake_model.mass_records['moving']['mass_kg'] *= .5
    with pytest.raises(QualificationError, match='inertia differs'):
        _require_physical_mass_binding(fake_model, real_mass)
    binding = {'schema': 'test-only binding', 'source_files': source_files}
    step_hashes = {f"{row['body']}__{row['name']}": hashlib.sha256((native/row['step_file']).read_bytes()).hexdigest()
                   for row in json.loads((native/'placed-native-parts.json').read_text())['records']}

    run = tmp_path/'run'
    (run/'simulation').mkdir(parents=True)
    model = {'schema': 'test-only-physical', 'mass_records': real_mass}
    control = {'schema': 'test-only-controller'}
    runtime_identity = _runtime_identity()
    simulation = dict(schema='cybrgeo-causal-simulation/1', model_sha256=digest(model),
                      control_sha256=digest(control), runtime_implementation=runtime_identity,
                      mujoco_version='test-only', options={},
                      metrics={'actuators': {'output': {
                          'peak_output_torque_Nm': .5,
                          'belt_demand_limit_s': 0., 'gear_demand_limit_s': 0.}}})
    (run/'physical-source.json').write_text(json.dumps(model), encoding='utf-8')
    (run/'control-source.json').write_text(json.dumps(control), encoding='utf-8')
    (run/'simulation'/'simulation.json').write_text(json.dumps(simulation), encoding='utf-8')
    (run/'simulation'/'trace.json.gz').write_bytes(b'TEST-ONLY-TRACE')
    report = dict(schema='cybrgeo-engineering-result/2', name='synthetic', passed=True,
                  source_files=source_files, native={'directory': str(native)},
                  native_build_sha256=digest(receipt), native_source_binding_sha256=digest(binding),
                  model_sha256=digest(model), control_sha256=digest(control), simulation=simulation)
    (run/'engineering.json').write_text(json.dumps(report), encoding='utf-8')
    basis = dict(source_sha256=digest(source_files), plan_sha256=digest(plan),
                 native_build_sha256=digest(receipt), native_source_binding_sha256=digest(binding),
                 model_sha256=digest(model), control_sha256=digest(control),
                 engineering_report_sha256=hashlib.sha256((run/'engineering.json').read_bytes()).hexdigest(),
                 simulation_sha256=hashlib.sha256((run/'simulation'/'simulation.json').read_bytes()).hexdigest(),
                 integration_trace_sha256=hashlib.sha256((run/'simulation'/'trace.json.gz').read_bytes()).hexdigest(),
                 runtime_sha256=digest(dict(runtime=runtime_identity, mujoco_version='test-only', options={})))
    package, trust, _, _, _ = _signed_fixture(tmp_path, basis=digest(basis), step_hashes=step_hashes)

    monkeypatch.setattr(engineering_source, 'project_directory', lambda source: project_dir)
    monkeypatch.setattr(engineering_source, 'load_project',
                        lambda source: (SimpleNamespace(name='synthetic', specification={'test': True}), source_files))
    def verify_native(directory, specification, **kwargs):
        actual = json.loads((native/'native-build.json').read_text(encoding='utf-8'))
        for relative, expected in actual['files_sha256'].items():
            assert hashlib.sha256((native/relative).read_bytes()).hexdigest() == expected
        return actual
    monkeypatch.setattr(engineering_source, 'verify_native_build', verify_native)
    monkeypatch.setattr(engineering_source, 'verify_native_source_binding', lambda *args, **kwargs: binding)
    monkeypatch.setattr(engineering_source, 'assert_project_source_identity',
                        lambda source, expected: expected if expected == source_files else (_ for _ in ()).throw(ValueError()))
    from mechanism_lab import physical_model
    monkeypatch.setattr(physical_model.PhysicalModel, 'from_source',
                        lambda source: SimpleNamespace(mass_records=source['mass_records']))
    monkeypatch.setattr(qualification, '_physical_startup_poses_mm',
                        lambda model, control, parts, mapping: _initial_poses())

    product = tmp_path/'product'
    manifest = release_project(project_dir, run, package, trust, product)
    assert manifest['status'] == 'qualified_for_declared_envelope_under_external_evidence'
    assert (product/'qualified-release.json').is_file()
    assert len(list((product/'parts').glob('*.step'))) == 2
    assert len(list((product/'evidence').glob('test-record-*.txt'))) == 8

    original_copy = qualification.shutil.copyfile
    def corrupted_step_copy(source, target, *args, **kwargs):
        result = original_copy(source, target, *args, **kwargs)
        if Path(source).suffix == '.step':
            Path(target).write_bytes(b'CORRUPTED-STEP')
        return result
    monkeypatch.setattr(qualification.shutil, 'copyfile', corrupted_step_copy)
    failed = tmp_path/'product-failed'
    with pytest.raises(QualificationError, match='changed while copying'):
        release_project(project_dir, run, package, trust, failed)
    assert not failed.exists()
    assert not list(tmp_path.glob('.product-failed-stage-*'))
    monkeypatch.setattr(qualification.shutil, 'copyfile', original_copy)
    original_sha = qualification._sha
    service_reads = [0]
    def changing_release_policy(path):
        if Path(path).resolve() == Path(qualification.__file__).resolve():
            service_reads[0] += 1
            if service_reads[0] > 1:
                return '0'*64
        return original_sha(path)
    monkeypatch.setattr(qualification, '_sha', changing_release_policy)
    changed_policy_target = tmp_path/'product-changed-policy'
    with pytest.raises(QualificationError, match='changed during release copy'):
        release_project(project_dir, run, package, trust, changed_policy_target)
    assert service_reads[0] >= 2
    assert not changed_policy_target.exists()
    assert not list(tmp_path.glob('.product-changed-policy-stage-*'))
    monkeypatch.setattr(qualification, '_sha', original_sha)
    from mechanism_lab import causal_runtime
    actual_runtime = causal_runtime._runtime_identity
    def changed_runtime():
        identity = deepcopy(actual_runtime())
        identity['source_files']['actuation.py'] = '0'*64
        return identity
    monkeypatch.setattr(causal_runtime, '_runtime_identity', changed_runtime)
    changed_runtime_target = tmp_path/'product-changed-runtime'
    with pytest.raises(QualificationError, match='Causal runtime source'):
        release_project(project_dir, run, package, trust, changed_runtime_target)
    assert not changed_runtime_target.exists()
    monkeypatch.setattr(causal_runtime, '_runtime_identity', actual_runtime)
    def change_runtime_during_copy(source, target, *args, **kwargs):
        result = original_copy(source, target, *args, **kwargs)
        if Path(source).suffix == '.step':
            monkeypatch.setattr(causal_runtime, '_runtime_identity', changed_runtime)
        return result
    monkeypatch.setattr(qualification.shutil, 'copyfile', change_runtime_during_copy)
    late_runtime_target = tmp_path/'product-late-runtime'
    with pytest.raises(QualificationError, match='Causal runtime source'):
        release_project(project_dir, run, package, trust, late_runtime_target)
    assert not late_runtime_target.exists()
    assert not list(tmp_path.glob('.product-late-runtime-stage-*'))
