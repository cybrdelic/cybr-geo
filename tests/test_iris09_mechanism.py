"""The actual IRIS-derived client's CAD masses and passive mechanical closure."""
from copy import deepcopy
import math

import numpy as np
import pytest

from mechanism_lab.causal_runtime import CausalSimulation
from mechanism_lab.engineering_source import load_project
from mechanism_lab.native_construction import aggregate_mass_properties, shape_mass_properties


@pytest.fixture(scope='module')
def client():
    project, _ = load_project('iris09')
    import sys
    module = sys.modules[project.construct.__module__]
    rows, layout, _ = module.moving_geometry(project.specification)
    records = {}
    for body, name, shape, material in rows:
        mass, com, inertia = shape_mass_properties(shape, module.DENSITIES[material])
        records.setdefault(body, []).append(dict(name=name, mass_kg=mass, com_m=com.tolist(),
            inertia_at_com_kg_m2=inertia.tolist(), provenance='Actual moving native CAD fixture'))
    masses = {body: aggregate_mass_properties(value) for body, value in records.items()}
    return project, module, rows, layout, masses


def simulation(client, options=None, removed=False):
    project, module, rows, layout, masses = client
    physical, control, parameters = module.physical_from_records(project.specification, masses)
    if removed:
        equality = physical.root.find('equality')
        equality.remove(equality.find("connect[@name='rod_00_closure']"))
        physical.provenance['adverse_case'] = 'First constraint disabled numerically; native geometry unchanged, no alternate hardware assembly claim'
    runtime = CausalSimulation(physical, control, lambda t: module.reference(t, parameters), options=options,
                              reference_source=dict(task_parameters=parameters, test_scope='Moving native CAD fixture'))
    return runtime, parameters


def test_same_pin_frames_define_native_bores_and_physical_closed_loops(client):
    project, module, rows, layout, masses = client
    assert project.duration_s == 8.
    assert layout['ring_delta_rad'] == pytest.approx(-.28327195585996)
    assert len(layout['links']) == 9
    for link in layout['links']:
        assert link.length_mm == pytest.approx(20.)
        assert link.native_shape().isValid()
    assert all(shape.isValid() and shape.Volume() > 0 for _, _, shape, _ in rows)
    assert len(masses) == 19
    runtime, _ = simulation(client)
    assert runtime.model.nu == 1 and runtime.model.njnt == 19 and runtime.model.neq == 9
    for index in range(9):
        assert runtime.model.joint(f'leaf_{index:02}_hinge').id not in runtime.joint_ids
    assert runtime.control['inertial_feedforward'] is False


def test_real_single_motor_moves_all_nine_passive_leaves_and_closes_loops(client):
    runtime, parameters = simulation(client)
    result = runtime.run(8., output_hz=40)
    checks = client[1].requirements(result, parameters)
    assert checks['passed'], checks
    assert result['metrics']['actuators']['annular_motor']['copper_heat_J'] > 0
    assert result['metrics']['monitoring_interval']['integration_steps'] == 40000
    for index in range(9):
        measured = result['metrics']['joints'][f'leaf_{index:02}_hinge']
        assert measured['maximum_position']['value'] > .89
        assert measured['direct_motor_actuator'] is None
    incomplete = deepcopy(result)
    incomplete['frames'] = [frame for frame in result['frames'] if abs(frame['time_s']-4.) > .1]
    assert not client[1].requirements(incomplete, parameters)['passed']
    incomplete = deepcopy(result)
    del incomplete['metrics']['constraints']['rod_00_closure']
    assert not client[1].requirements(incomplete, parameters)['passed']


def test_power_off_and_removed_link_falsify_the_mechanical_task(client):
    off, parameters = simulation(client, {'power': False})
    off_result = off.run(4.05, output_hz=40)
    assert not client[1].requirements(off_result, parameters)['passed']
    for index in range(9):
        assert abs(off.data.qpos[off.model.joint(f'leaf_{index:02}_hinge').qposadr[0]]) < 1e-10
    assert off.motors[0].state.current_A == 0
    broken, parameters = simulation(client, removed=True)
    broken_result = broken.run(4.05, output_hz=40)
    first = broken.model.joint('leaf_00_hinge').qposadr[0]
    intact = broken.model.joint('leaf_01_hinge').qposadr[0]
    assert abs(broken.data.qpos[first]) < 1e-10
    assert broken.data.qpos[intact] > .89
    assert not client[1].requirements(broken_result, parameters)['passed']
    assert broken_result['model_sha256'] != off_result['model_sha256']


def test_source_geometry_and_unreachable_link_edits_are_rejected(client):
    project, module, *_ = client
    changed = deepcopy(project.specification)
    changed['source_geometry']['leaf']['hinge_radius'] = .1
    with pytest.raises(ValueError, match='exact bound source'):
        module.layout(changed)
    changed = deepcopy(project.specification)
    changed['rod_length_mm'] = .1
    with pytest.raises(ValueError, match='authored radial'):
        module.layout(changed)


def test_axial_stack_derives_pin_tip_clearance_and_rejects_unattached_edits(client):
    project, module, *_ = client
    spec = project.specification
    stack = module.axial_stack(spec)
    assert stack['output_pin_top_mm'] == pytest.approx(32.6)
    assert stack['lower_lip_mm'] == pytest.approx([32.8, 33.1])
    assert stack['rotor_mm'][0] == pytest.approx(33.3)
    assert stack['lower_lip_mm'][0]-stack['output_pin_top_mm'] == pytest.approx(.2)
    # The repaired lip clears the whole swept disk, including outward crank
    # positions that the previous initial-pose check missed.
    envelope = module._cylinder(67., stack['rod_bottom_mm'], stack['moving_upper_envelope_mm'])
    lip = module._annulus(63., 66.8, *stack['lower_lip_mm'])
    assert envelope.intersect(lip).Volume() == pytest.approx(0., abs=1e-9)
    assert envelope.distance(lip) == pytest.approx(.2)
    changed = deepcopy(spec)
    changed['rotor_pin_radius_mm'] = 45.
    with pytest.raises(ValueError, match='authored radial'):
        module.layout(changed)
    changed = deepcopy(spec)
    changed['rod_height_mm'] = 25.
    with pytest.raises(ValueError, match='support envelope'):
        module.layout(changed)
    for thickness in (1.2, 1.4):
        changed = deepcopy(spec)
        changed['rod_thickness_mm'] = thickness
        with pytest.raises(ValueError, match='radial-bearing clearance'):
            module.layout(changed)
    for group, key, value in ((None, 'rod_height_mm', 32.),
                              ('axial_stack', 'keeper_thickness_mm', .4),
                              ('axial_stack', 'rod_keeper_clearance_mm', .4)):
        changed = deepcopy(spec)
        (changed[group] if group else changed)[key] = value
        with pytest.raises(ValueError, match='radial-bearing clearance'):
            module.layout(changed)
    # Independent BREP regression at maximum outward crank reach: the
    # default keeper clears the radial-bearing face by its declared gap.
    from mechanism_lab.core import pose_cad
    xy = (20*math.cos(.45), 20*math.sin(.45))
    keeper = module._split_keeper(.82, 1.6, *stack['output_keeper_mm'], xy)
    posed = pose_cad(keeper, module.design.leaf_frame(0, .45, 50.))
    radial = module._annulus(64.05, 66.8, *stack['radial_bearing_mm'])
    assert posed.intersect(radial).Volume() == pytest.approx(0., abs=1e-9)
    assert posed.distance(radial) == pytest.approx(.1)
    changed = deepcopy(spec)
    changed['rod_height_mm'] = 31.2
    changed['axial_stack'].update(rod_keeper_clearance_mm=.1, keeper_thickness_mm=.1,
        pin_tip_extension_mm=.03, moving_pin_to_fixed_lip_clearance_mm=.01,
        capture_lip_thickness_mm=.05)
    with pytest.raises(ValueError, match='must overlap'):
        module.layout(changed)
