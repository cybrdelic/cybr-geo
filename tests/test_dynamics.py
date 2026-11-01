"""Actual MuJoCo dynamics, compared to analytic mechanics and force failures."""
import json
import math
from copy import deepcopy

import numpy as np
import pytest

from mechanism_lab.constructed import build_from_plan
from mechanism_lab.dynamics import export_mjcf, geometry_integrals, load_model, plot_simulation, simulate, simulated_assembly

import mujoco


def pendulum():
    # 1kg, one-metre rectangular rod, horizontal at q=0; pivot at left end.
    part = dict(name='rod', body=dict(kind='box', size=[1000, 10, 10], position=[500, 0, 0], rotation=[0, 0, 0]),
                cuts=[], unions=[], color=[.4, .5, .6], metalness=.7, roughness=.4, fillet=0)
    plan = dict(name='pendulum', units='mm', parts=[part], joints=[dict(name='hinge', child='rod', parent=None,
        type='revolute', origin=[0, 0, 0], axis=[0, 1, 0], min=-175, max=175, value=0)])
    return build_from_plan(plan)


def settings(limit=20000, damping=0):
    return dict(default_density_kg_m3=10000, gravity_mm_s2=[0, 0, -9810],
                actuators=[dict(joint='hinge', effort_limit_Nmm=limit, viscous_damping=damping)])


def test_cad_inertia_and_actual_pendulum_gravity_acceleration(tmp_path):
    assembly = pendulum()
    integral = geometry_integrals(assembly)['rod']
    assert integral['volume_mm3'] == pytest.approx(100000)
    assert integral['com_mm'] == pytest.approx([500, 0, 0])
    assert integral['inertia_mm5'][1][1] == pytest.approx(100000 * (1000**2 + 10**2) / 12)
    info = export_mjcf(assembly, tmp_path, settings=settings())
    model, _ = load_model(tmp_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    inertia_about_pivot = (1 + .01**2) / 12 + .5**2
    assert info['mass_properties']['rod']['mass'] == pytest.approx(1)
    assert data.qacc[0] == pytest.approx(9.81 * .5 / inertia_about_pivot, rel=1e-9)
    # Zero velocity gravity bias is the torque needed to hold horizontal.
    assert data.qfrc_bias[0] == pytest.approx(-4.905, rel=1e-9)


def test_unpowered_pendulum_conserves_energy_and_moves_under_gravity(tmp_path):
    result = simulate(pendulum(), seconds=.5, timestep=.0002, output_hz=100,
                      settings=settings(), controller=False, output=tmp_path)
    assert result['samples'][-1]['simulated_values']['hinge'] > 60
    energy = np.array([sum(sample['energy_J']) for sample in result['samples']])
    # Symplectic integration has bounded first-order energy error.
    assert np.ptp(energy) < .015
    assert result['backend'] == 'MuJoCo'


def test_torque_saturation_causes_measured_tracking_failure(tmp_path):
    assembly = pendulum()
    nominal = simulate(assembly, seconds=2, timestep=.002, settings=settings(), output=tmp_path / 'nominal')
    weak = simulate(assembly, seconds=2, timestep=.002, settings=settings(), effort_scale=.005, output=tmp_path / 'weak')
    assert nominal['tracking_passed']
    assert nominal['metrics']['hinge']['max_tracking_error'] < .2
    assert not weak['tracking_passed']
    assert weak['metrics']['hinge']['max_tracking_error'] > 30
    assert weak['metrics']['hinge']['saturation_fraction'] > .9
    assert weak['metrics']['hinge']['peak_applied_effort_SI'] <= .1 + 1e-12
    assert weak['metrics']['hinge']['declared_effort_limit_SI'] == 20
    assert weak['model']['motor_limits_SI']['hinge'] == 20
    assert weak['run_settings']['effort_scale'] == .005
    assert 'Experimental effort_scale=' in weak['assumptions'][-1]
    posed = simulated_assembly(assembly, weak)
    sample = weak['samples'][10]
    assert np.allclose(posed.pose(assembly.parts[0], sample['time_s']), sample['body_poses_mm']['rod'])
    plot_simulation(weak, tmp_path / 'tracking.svg')
    assert '<polyline' in (tmp_path / 'tracking.svg').read_text(encoding='utf-8')
    assert 'MuJoCo' in (tmp_path / 'tracking.svg').read_text(encoding='utf-8')


def test_coupled_body_mass_and_drive_constraint_are_simulated(tmp_path):
    assembly = pendulum()
    recipe = deepcopy(assembly.metadata['construction_recipe'])
    recipe['parts'].append({**deepcopy(recipe['parts'][0]), 'name': 'follower'})
    recipe['parts'][1]['body']['position'] = [1500, 0, 0]
    recipe['joints'].append(dict(name='follow', child='follower', parent='rod', type='revolute',
        origin=[1000, 0, 0], axis=[0, 1, 0], min=-175, max=175, value=0,
        drive=dict(joint='hinge', ratio=.5, offset=0)))
    doubled = build_from_plan(recipe)
    export_mjcf(doubled, tmp_path, settings=settings(100000))
    model, _ = load_model(tmp_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    matrix = np.zeros((2, 2))
    mujoco.mj_fullM(model, data, matrix)
    # Two rods have nonzero cross inertia; independent scalar pendulums fail.
    assert matrix[0, 1] == pytest.approx(1 / 3 + .5 + .01**2 / 12, rel=1e-9)
    result = simulate(doubled, seconds=.5, timestep=.001, settings=settings(100000), output=tmp_path / 'trace')
    assert result['maximum_drive_constraint_error_native'] < .05
    assert result['metrics']['hinge']['peak_applied_effort_SI'] > 4.905


def test_explicit_contact_environment_falls_and_rests_on_plane(tmp_path):
    from mechanism_lab.construction import compile_mechanism
    from mechanism_lab.mechanism import load_example
    source = load_example('inspection_head')
    source['name'] = 'Canonical contact probe'
    source['components'] = source['components'][:1]
    source['actuators'] = {'J1_motor': source['actuators']['J1_motor']}
    source['loads']['payload'] = dict(component='J1', mass_kg=0, com_offset_mm=[0, 0, 0])
    source['contacts']['free_bodies'] = [dict(name='dropped_sphere',
        position_mm=[0, 200, 200], mass_kg=.1,
        velocity_mm_s=[0, 0, 0], angular_velocity_rad_s=[0, 0, 0],
        shape=dict(kind='sphere', size=[20, 20, 20], position=[0, 0, 0], rotation=[0, 0, 0]))]
    plan = compile_mechanism(source)
    assembly = build_from_plan(plan)
    result = simulate(assembly, seconds=1, timestep=.001, output=tmp_path)
    assert result['model_sha256'] == plan['construction']['model_sha256']
    assert result['model']['settings']['contacts'] == source['contacts']
    heights = [s['body_poses_mm']['dropped_sphere'][2][3] for s in result['samples']]
    assert heights[0] == pytest.approx(200)
    # Before impact free fall is causal: z(t)=z0-g*t^2/2, within dt error.
    sample = min(result['samples'], key=lambda s: abs(s['time_s'] - .1))
    assert sample['body_poses_mm']['dropped_sphere'][2][3] == pytest.approx(200 - 4905 * .1**2, abs=.6)
    assert heights[-1] == pytest.approx(10, abs=.1)
    assert result['maximum_contacts'] > 0
    assert result['samples'][-1]['summed_normal_contact_force_N'] == pytest.approx(.981, rel=.01)


def test_missing_motor_limits_and_mutated_mjcf_fail_closed(tmp_path):
    assembly = pendulum()
    with pytest.raises(ValueError, match='explicit effort_limit'):
        export_mjcf(assembly, tmp_path)
    export_mjcf(assembly, tmp_path, settings=settings())
    model_path = tmp_path / 'model.xml'
    model_path.write_text(model_path.read_text().replace('9.81', '1.62'))
    with pytest.raises(ValueError, match='hash mismatch'):
        load_model(tmp_path)


def test_contact_friction_changes_free_body_motion(tmp_path):
    assembly = pendulum()
    def trial(friction, target):
        config = settings()
        config['contacts'] = dict(ground=True, friction=[friction, .001, .0001], free_bodies=[dict(name='slider', position_mm=[0, 200, 10],
            velocity_mm_s=[300, 0, 0], mass_kg=.1,
            shape=dict(kind='box', size=[20, 20, 20], friction=[friction, .001, .0001]))])
        return simulate(assembly, seconds=.4, timestep=.001, settings=config, output=target)
    slippery, gripping = trial(.01, tmp_path / 'slip'), trial(.8, tmp_path / 'grip')
    displacement = lambda r: r['samples'][-1]['body_poses_mm']['slider'][0][3]
    # MuJoCo combines geom friction using max: override ground consistently
    # through export policy, so this fixture tests two explicit material pairs.
    assert gripping['maximum_contacts'] > 0
    assert displacement(gripping) < 20
    assert displacement(slippery) > 80


def test_cache_preserves_exact_inertia_and_rejects_mutated_mesh_integrals(tmp_path):
    from mechanism_lab.core import save_cache, load_cache
    assembly = pendulum()
    before = geometry_integrals(assembly)
    save_cache(assembly, tmp_path)
    cached = load_cache(tmp_path)
    assert geometry_integrals(cached) == before
    cached.parts[0].vertices *= 2
    changed = geometry_integrals(cached)['rod']
    assert changed['source'] == 'tessellated watertight mesh integrals'
    assert changed['volume_mm3'] == pytest.approx(before['rod']['volume_mm3'] * 8)


def test_canonical_physical_metadata_is_checked_before_export(tmp_path):
    from types import SimpleNamespace
    from mechanism_lab.construction import compile_plan
    recipe = compile_plan('inspection_head')
    recipe['construction']['dynamics']['payload']['mass_kg'] += 1
    assembly = SimpleNamespace(metadata={'construction_recipe': recipe},
        parts=[SimpleNamespace(name=p['name']) for p in recipe['parts']])
    with pytest.raises(ValueError, match='mechanical model identity'):
        export_mjcf(assembly, tmp_path)


def test_canonical_settings_cannot_be_overridden_outside_source(tmp_path):
    from types import SimpleNamespace
    from mechanism_lab.construction import compile_plan
    recipe = compile_plan('inspection_head')
    assembly = SimpleNamespace(metadata={'construction_recipe': recipe},
        parts=[SimpleNamespace(name=p['name']) for p in recipe['parts']])
    for override in ({'gravity_mm_s2': [0, 0, 0]}, {'contacts': {'ground': False}},
                     {'default_density_kg_m3': 1}, {'payload': {'mass_kg': 0}}):
        with pytest.raises(ValueError, match='mechanism source, not backend overrides'):
            export_mjcf(assembly, tmp_path, settings=override)
    assert not tmp_path.joinpath('model.xml').exists()


def test_low_inertia_drive_constraint_converges_under_step_halving(tmp_path):
    # A light motor rotor drives a much heavier rod on a separate body branch.
    # This exposes soft-constraint regularization hidden by equally heavy links.
    original = pendulum().metadata['construction_recipe']
    recipe = deepcopy(original)
    recipe['parts'][0]['name'] = 'rotor'
    recipe['parts'][0]['body'].update(size=[10, 10, 10], position=[0, 0, 0])
    recipe['parts'].append(deepcopy(original['parts'][0]))
    recipe['joints'][0].update(child='rotor', min=-5, max=5)
    recipe['joints'].append(dict(name='driven', child='rod', parent=None, type='revolute',
        origin=[0, 0, 0], axis=[0, 1, 0], min=-5, max=5, value=0,
        drive=dict(joint='hinge', ratio=1, offset=0)))
    assembly = build_from_plan(recipe)
    coarse = simulate(assembly, seconds=.3, timestep=.002, settings=settings(), output=tmp_path / 'coarse')
    fine = simulate(assembly, seconds=.3, timestep=.00005, settings=settings(), output=tmp_path / 'fine')
    assert coarse['model']['solver_settings']['effective_timestep_s'] == .0001
    assert coarse['model']['motor_limits_SI'] == fine['model']['motor_limits_SI']
    assert coarse['model']['mass_properties'] == fine['model']['mass_properties']
    assert coarse['maximum_drive_constraint_error_native'] > fine['maximum_drive_constraint_error_native'] * 3
    assert fine['metrics']['hinge']['max_tracking_error'] < coarse['metrics']['hinge']['max_tracking_error']
    trace = coarse['integration_state_trace']
    assert len(trace['joint_values']) == coarse['integration_steps'] + 1
    assert trace['joint_names'] == ['hinge', 'driven']


def test_source_motion_period_is_shared_with_native_pose_and_not_run_duration(tmp_path):
    from mechanism_lab.motion import MotionTask
    from mechanism_lab.construction import recipe_digest
    plan = deepcopy(pendulum().metadata['construction_recipe'])
    task = MotionTask.from_joints(plan['joints'], period_s=12)
    plan['construction'] = dict(schema='cybrgeo-construction/1', recipe_sha256=recipe_digest(plan), motion=task.to_dict())
    assembly = build_from_plan(plan)
    short = simulate(assembly, seconds=1, timestep=.002, settings=settings(), output=tmp_path / 'short')
    long = simulate(assembly, seconds=2, timestep=.002, settings=settings(), output=tmp_path / 'long')
    one_second = next(s for s in long['samples'] if s['time_s'] == 1)
    assert one_second['requested_values'] == short['samples'][-1]['requested_values'] == task.sample(1)['values']
    assert one_second['simulated_values'] == pytest.approx(short['samples'][-1]['simulated_values'])
    requested_angle = math.radians(task.sample(1)['values']['hinge'])
    assert assembly.pose(assembly.parts[0], 1)[0, 0] == pytest.approx(math.cos(requested_angle))
