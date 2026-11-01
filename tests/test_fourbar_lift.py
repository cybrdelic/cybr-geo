"""A distinct loaded mechanism exercising the same engineering consumer path."""
from copy import deepcopy
import gzip
import json
import math
import sys

import numpy as np
import pytest

from mechanism_lab.engineering_pipeline import generate_project
from mechanism_lab.engineering_source import load_project
from mechanism_lab.exporters import export_step


@pytest.fixture(scope='module')
def built(tmp_path_factory):
    output = tmp_path_factory.mktemp('fourbar')
    assembly, report = generate_project('fourbar_lift',output=output)
    assert report['passed'], report['requirements']
    export_step(assembly,output/'fourbar_lift.step')
    project, _ = load_project('fourbar_lift')
    module = sys.modules[project.construct.__module__]
    with gzip.open(output/'simulation'/'trace.json.gz','rt',encoding='utf-8') as stream:
        trace = json.load(stream)
    return output,assembly,report,project,module,trace


def test_single_electrical_motor_drives_cad_mass_loaded_passive_output(built):
    output,assembly,report,project,module,trace = built
    assert len(assembly.parts) == 15
    assert len(assembly.metadata['recorded_times_s'])>900
    assert trace['steps']==16000
    joints = trace['metrics']['joints']
    assert set(joints)=={'crank_hinge','rocker_hinge','coupler_hinge'}
    assert joints['crank_hinge']['direct_motor_actuator']=='crank_motor'
    assert joints['rocker_hinge']['direct_motor_actuator'] is None
    assert joints['coupler_hinge']['direct_motor_actuator'] is None
    assert trace['metrics']['actuators']['crank_motor']['electrical_energy_J']>0
    assert trace['metrics']['actuators']['crank_motor']['copper_heat_J']>0
    assert abs(trace['frames'][-1]['qpos'][0]-math.tau)<project.specification['tracking_limit_rad']
    assert trace['metrics']['max_applied_generalized_force']==0
    assert trace['metrics']['max_external_body_force_N']==0
    design = report['design_envelope']
    assert design['nominal_fit']['status'] == 'declared_stack_passed'
    assert design['nominal_fit']['complete_declared_stack']
    assert {row['name'] for row in design['nominal_fit']['fits']} == {
        'input_pin_bore', 'output_pin_bore'}
    assert all(row['variation_source'] == 'native_authored_design'
               and row['declared_stack_screen_passed']
               for row in design['nominal_fit']['fits'])
    assert not design['nominal_fit']['manufacturing_fit_qualified']
    assert design['planar_kinematics']['full_interval_branch_bounds']
    assert design['planar_kinematics']['quasi_static_gravity']['total_gravity_torque_bound_Nm'] > 0
    assert design['connector_demands']['complete_solver_coverage']
    assert design['connector_demands']['connectors']['coupler_closure']['peak_modeled_anchor_force_N'] > 0
    assert not design['manufacturing_qualified']
    assert json.loads((output/'design-envelope.json').read_text()) == design
    masses = json.loads((output/'native'/'mass_properties.json').read_text(encoding='utf-8'))
    source = trace['physical_source']['mass_records']
    assert source=={name:masses[name] for name in ('crank','rocker','coupler')}
    assert masses['rocker']['mass_kg']>masses['crank']['mass_kg']
    # The analytic circle relation is only an independent observed-state check.
    # It is never sent to the controller or replay path.
    initial = module.layout(project.specification)['phi']
    observed = []
    for frame in trace['frames']:
        expected = module.rocker_angle(frame['qpos'][0]+project.specification['initial_crank_angle_rad'],project.specification)
        delta = expected-initial-frame['qpos'][2]
        observed.append(abs(math.atan2(math.sin(delta),math.cos(delta))))
    assert max(observed)<1e-4


def test_saved_native_exports_and_actual_motion_have_no_sampled_interpenetration(built):
    from examples.engineering.fourbar_lift.verify import verify
    output, *_ = built
    receipt = verify(output,sample_count=65)
    assert receipt['passed']
    assert receipt['aggregate_step_solids']==receipt['glb_nodes']==15
    assert len(receipt['native_step_mass_checks'])==15
    audit = receipt['sampled_clearance']
    assert len(audit['frame_indices'])==65
    assert audit['exact_native_pair_checks']>100
    assert audit['minimum_exact_gap_mm']>0.04
    assert all(row['maximum_overlap_mm3']<=1e-6 for row in audit['pairs'])


def test_power_and_physical_pin_connection_are_causally_necessary(built):
    from examples.engineering.fourbar_lift.verify import causal_controls
    output,assembly,report,project,module,nominal = built
    receipt = causal_controls(output)
    assert receipt['passed'] and len(receipt['cases'])==2
    cases = {row['name']:row for row in receipt['cases']}
    for row in cases.values():
        assert row['steps']==nominal['steps']
        assert row['observed_passed'] is False
    for name in cases:
        with gzip.open(output/'controls'/name/'trace.json.gz','rt',encoding='utf-8') as stream:
            changed = json.load(stream)
        assert changed['options']==cases[name]['options']
        assert changed['physical_source']['provenance']['native_build_sha256']==report['native_build_sha256']
        assert changed['physical_source']['provenance']['native_source_binding_sha256']==report['native_source_binding_sha256']
        if name=='power-disabled':
            assert changed['metrics']['actuators']['crank_motor']['peak_current_A']==0
            assert abs(changed['frames'][-1]['qpos'][0]-math.tau)>1.
        else:
            assert changed['model_sha256']!=nominal['model_sha256']
            assert 'coupler_closure' not in changed['metrics']['constraints']
            actual = np.array([frame['qpos'][2] for frame in changed['frames']])
            target = np.array([frame['qpos'][2] for frame in nominal['frames']])
            assert np.sqrt(np.mean((actual-target)**2))>.1


def test_missing_joint_or_closure_observations_cannot_satisfy_mechanism(built):
    *_,project,module,trace = built
    for category,name in (('joints','rocker_hinge'),('constraints','coupler_closure')):
        changed = deepcopy(trace)
        del changed['metrics'][category][name]
        assert not module.requirements(changed,changed['task_parameters'])['passed']


def test_same_topology_but_changed_exported_model_cannot_pass_readback(built):
    from examples.engineering.fourbar_lift.verify import verify
    output,*_ = built
    path = output/'model.xml'
    original = path.read_bytes()
    try:
        path.write_bytes(original.replace(b'gravity="0 -9.81 0"',b'gravity="0 -1 0"'))
        assert path.read_bytes()!=original
        with pytest.raises(AssertionError):
            verify(output,sample_count=2)
    finally:
        path.write_bytes(original)


@pytest.mark.parametrize('key,value,match',[
    ('crank_length_mm',70.,'Grashof'),
    ('shaft_diameter_mm',8.,'bearing and keeper'),
    ('link_width_mm',7.,'minimum nominal wall'),
    ('load_position_mm',60.,'load lobe'),
    ('load_thickness_mm',6.,'backplate clearance'),
    ('coupler_length_mm',float('nan'),'finite and positive'),
])
def test_unreachable_or_unmated_geometry_is_rejected_before_construction(key,value,match):
    project,_ = load_project('fourbar_lift')
    module = sys.modules[project.construct.__module__]
    spec = deepcopy(project.specification)
    spec[key] = value
    with pytest.raises(ValueError,match=match):
        module.layout(spec)


def test_large_load_edit_is_rejected_by_constructive_swept_bound():
    project,_ = load_project('fourbar_lift')
    module = sys.modules[project.construct.__module__]
    spec = deepcopy(project.specification)
    spec.update(load_position_mm=300.,load_radius_mm=232.)
    with pytest.raises(ValueError,match='swept envelope'):
        module.layout(spec)
