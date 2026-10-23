"""Actual native integration on a one-axis mechanism with an unrelated free body.

The free body's coordinates precede the motor coordinate deliberately: the host
must resolve names, apply motor forces, and leave free-body integration alone.
"""
from copy import deepcopy
import gzip
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pytest

from mechanism_lab.causal_runtime import CausalSimulation
from mechanism_lab.physical_model import PhysicalModel


XML = '''<mujoco model="one_hinge_and_free_workpiece">
  <compiler angle="radian"/><option timestep="0.001" gravity="0 0 -9.81" integrator="implicitfast"/>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 .1" friction=".8 .01 .001"/>
    <body name="workpiece" pos="0 0 .021"><freejoint name="free_workpiece"/>
      <geom name="block" type="box" size=".02 .02 .02" mass=".1" friction=".8 .01 .001"/>
    </body>
    <body name="link" pos="2 0 1.5"><joint name="swing" axis="0 1 0" frictionloss=".01" damping=".01"/>
      <inertial pos=".5 0 0" mass="1" diaginertia=".001 .084 .084"/>
      <geom type="capsule" fromto="0 0 0 1 0 0" size=".02" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <contact><pair geom1="floor" geom2="block" friction=".8 .8 .01 .001 .001"/></contact>
  <actuator><motor name="motor" joint="swing"/></actuator>
  <keyframe><key name="start" qpos="0 0 .021 1 0 0 0 .3" qvel=".25 0 0 0 0 0 0"/></keyframe>
</mujoco>'''


def control():
    motor = dict(gear_ratio=50., gear_efficiency=.9, torque_constant_Nm_A=.02,
                 resistance_ohm=2., inductance_H=.01, bus_voltage_V=24., peak_current_A=10.,
                 gear_efficiency_transition_motor_rad_s=3., thermal_resistance_K_W=10.,
                 thermal_capacity_J_K=20., gear_output_torque_limit_Nm=10.)
    return dict(drives=[dict(joint='swing', actuator='motor', motor=motor, kp=20., kd=4.,
                            ki=0., integrator_limit=.1, soft_limits=[-3., 3.],
                            brake=dict(delay_s=.02, ramp_s=.01, holding_torque_Nm=10.,
                                       screening_dynamic_torque_Nm=8.))],
                initial_keyframe='start', inertial_feedforward=True,
                gravity_and_coriolis_feedforward=True)


def make(options=None, ctrl=None, ref=None, xml=XML):
    return CausalSimulation(PhysicalModel.from_xml(xml), ctrl or control(),
                            ref or (lambda t: dict(q=[.3], qd=[0.], qdd=[0.])), options=options)


def test_named_coordinate_motor_gravity_and_thermal_causality():
    powered, failed = make(), make({'power': False, 'brakes': False})
    assert powered.qadr.tolist() == [7]
    assert powered.vadr.tolist() == [6]
    result = powered.run(.3, output_hz=20)
    failed.run(.3, output_hz=20)
    assert abs(powered.data.qpos[7]-.3) < .01
    assert failed.data.qpos[7] > .7
    assert powered.motors[0].state.copper_heat_J > 0
    assert powered.motors[0].state.temperature_C > 25
    assert failed.motors[0].state.current_A == 0
    assert failed.safety.reason == 'power-loss'
    assert np.all(failed.data.qfrc_applied[:6] == 0)
    assert np.all(failed.data.xfrc_applied == 0)
    assert result['metrics']['actuators']['motor']['rms_current_A'] > 0
    assert result['metrics']['free_bodies']['workpiece']['max_imposed_generalized_force'] == 0
    assert result['reference_identity'] == 'unbound Python callback'


def test_free_workpiece_contact_friction_has_causal_effect_without_pose_commands():
    nominal, slippery = make(), make({'frictionScale': .001})
    a, b = nominal.run(.3, output_hz=100), slippery.run(.3, output_hz=100)
    assert any(c['normal_N'] > .1 for f in a['frames'] for c in f['contacts'])
    assert b['frames'][-1]['qpos'][0] - a['frames'][-1]['qpos'][0] > .03
    for sim in (nominal, slippery):
        assert .018 < sim.data.qpos[2] < .022
        assert np.all(sim.data.qfrc_applied[:6] == 0)
        assert np.all(sim.data.xfrc_applied == 0)
    assert nominal.model.jnt_type[nominal.model.joint('free_workpiece').id] == nominal.mj.mjtJoint.mjJNT_FREE


@pytest.mark.parametrize('explicit_pair', [False, True])
def test_friction_intervention_scales_native_contact_sources_and_changes_free_motion(explicit_pair):
    xml = XML if explicit_pair else XML.replace(
        '<contact><pair geom1="floor" geom2="block" friction=".8 .8 .01 .001 .001"/></contact>', '')
    nominal = make(xml=xml)
    slippery = make({'frictionScale': .001}, xml=xml)
    assert nominal.model.npair == int(explicit_pair)
    np.testing.assert_array_equal(slippery.model.geom_friction, nominal.model.geom_friction*.001)
    np.testing.assert_array_equal(slippery.model.pair_friction, nominal.model.pair_friction*.001)
    nominal.run(.3, output_hz=10)
    slippery.run(.3, output_hz=10)
    assert slippery.data.qpos[0]-nominal.data.qpos[0] > .03
    assert np.all(slippery.data.qfrc_applied[:6] == 0)
    assert np.all(slippery.data.xfrc_applied == 0)


@pytest.mark.parametrize('explicit_pair', [False, True])
def test_scheduled_friction_change_uses_original_coefficients_without_compounding(explicit_pair):
    xml = XML if explicit_pair else XML.replace(
        '<contact><pair geom1="floor" geom2="block" friction=".8 .8 .01 .001 .001"/></contact>', '')
    initial = make({'frictionScale': .5}, xml=xml)
    changed = make({'frictionScale': .5, 'frictionChangeTime': .002, 'frictionAfterScale': .001}, xml=xml)
    for _ in range(2):
        initial.step()
        changed.step()
        np.testing.assert_array_equal(changed.data.qpos, initial.data.qpos)
    changed.step()
    np.testing.assert_array_equal(changed.model.geom_friction, changed.base_geom_friction*.001)
    np.testing.assert_array_equal(changed.model.pair_friction, changed.base_pair_friction*.001)
    changed.step()
    np.testing.assert_array_equal(changed.model.geom_friction, changed.base_geom_friction*.001)
    np.testing.assert_array_equal(changed.model.pair_friction, changed.base_pair_friction*.001)
    assert changed.friction_changed


def test_fault_delayed_brake_native_constraint_and_watchdog():
    sim = make({'estopTime': .02})
    out = sim.run(.12, output_hz=1000)
    assert sim.safety.reason == 'emergency-stop'
    assert sim.safety.fault_time_s == pytest.approx(.02)
    assert all(f['brake_capacity_Nm'] == [0.] for f in out['frames'] if f['time_s'] <= .04)
    assert max(f['brake_capacity_Nm'][0] for f in out['frames']) >= 8
    assert any(abs(f['brake_torque_Nm'][0]) > .1 for f in out['frames'])
    assert sim.brake_energy[0] > 0
    assert sim.motors[0].state.current_A == 0
    watchdog = make({'heartbeat': False})
    watchdog.run(.13)
    assert watchdog.safety.reason == 'command-watchdog'


def test_frame_refreshes_poststep_pose_without_recomputing_dynamics():
    sim = make({'power': False, 'brakes': False})
    for _ in range(30):
        sim.step()
    before = sim.data.qfrc_bias.copy()
    state = sim.data.qpos.copy()
    mass_before, mass_after = np.zeros((sim.model.nv, sim.model.nv)), np.zeros((sim.model.nv, sim.model.nv))
    sim.mj.mj_fullM(sim.model, sim.data, mass_before)
    frame = sim.frame()
    theta = sim.data.qpos[7]
    expected = [[math.cos(theta), 0, math.sin(theta)], [0, 1, 0], [-math.sin(theta), 0, math.cos(theta)]]
    assert np.allclose(np.reshape(frame['bodies']['link']['rotation_row_major'], (3, 3)), expected)
    assert np.array_equal(before, sim.data.qfrc_bias)
    assert np.array_equal(state, sim.data.qpos)
    sim.mj.mj_fullM(sim.model, sim.data, mass_after)
    assert np.array_equal(mass_before, mass_after)
    assert frame['bodies']['workpiece']['position_m'] == pytest.approx(frame['qpos'][:3])


def test_repeated_run_records_final_frame_and_initial_options(tmp_path):
    sim = make({'motorOffTime': .003})
    first = sim.run(.005, output_hz=120)
    second = sim.run(.004, output_hz=120, output=tmp_path)
    assert [f['time_s'] for f in first['frames']] == pytest.approx([0, .005])
    assert [f['time_s'] for f in second['frames']] == pytest.approx([.005, .009])
    assert second['options']['power'] is True
    assert second['final_options']['power'] is False
    assert second['start_time_s'] == pytest.approx(.005)
    saved = json.loads(gzip.decompress((tmp_path / 'trace.json.gz').read_bytes()))
    assert saved == second


@pytest.mark.parametrize('options', [{'frictionScale': -1}, {'torqueScale': math.nan},
                                   {'power': 0}, {'payloadScale': 2}, {'brakeDelayScale': 0}])
def test_invalid_or_unimplemented_perturbations_are_not_silently_accepted(options):
    with pytest.raises(ValueError):
        make(options)


@pytest.mark.parametrize('seconds,hz', [(0, 120), (.0001, 120), (.1, math.inf), (math.nan, 120)])
def test_invalid_run_arguments(seconds, hz):
    with pytest.raises(ValueError):
        make().run(seconds, output_hz=hz)


def test_incompatible_transmission_and_reference_fail_before_integration():
    with pytest.raises(ValueError, match='unit-gain direct'):
        make(xml=XML.replace('name="motor" joint="swing"', 'name="motor" joint="swing" gear="2"'))
    bad = control()
    bad['drives'] *= 2
    with pytest.raises(ValueError, match='unique'):
        make(ctrl=bad)
    sim = make(ref=lambda t: dict(q=[math.nan], qd=[0.], qdd=[0.]))
    before = sim.data.qpos.copy()
    with pytest.raises(ValueError, match='Reference q'):
        sim.step()
    assert sim.steps == 0
    assert np.array_equal(before, sim.data.qpos)


def test_source_identity_is_snapshot_of_compiled_model_control_and_reference():
    physical, cfg, source = PhysicalModel.from_xml(XML), control(), {'task': 'hold', 'q': [.3]}
    sim = CausalSimulation(physical, cfg, lambda t: dict(q=[.3], qd=[0.], qdd=[0.]), reference_source=source)
    expected_model = sim.model_sha256
    expected_reference = sim.reference_sha256
    physical.root.set('model', 'edited_after_compilation')
    cfg['drives'][0]['motor']['peak_current_A'] = 999
    source['q'][0] = 7
    report = sim.run(.002)
    assert report['model_sha256'] == expected_model
    assert report['reference_sha256'] == expected_reference
    assert report['reference_source']['q'] == [.3]
    assert sim.control['drives'][0]['motor']['peak_current_A'] == 10


def passive_model(equality='<connect name="closure" site1="rod_tip" site2="passive_tip"/>',
                  *, extra='', key='.2 .01 .2 -.2', velocity='0 0 0 0', armature=0., damping=.01):
    # Passive hinge and spring-loaded slide deliberately precede the driven DOF.
    # A parallelogram four-bar couples the hinges through a passive rod and
    # actual connect forces. No passive position target or actuator is supplied.
    return f'''<mujoco><compiler angle="radian"/>
      <option timestep=".001" gravity="0 0 0" integrator="implicitfast"/>
      <worldbody>
        <body name="passive" pos=".2 0 0"><joint name="follower" axis="0 0 1"/>
          <inertial pos="0 0 0" mass="1" diaginertia=".01 .01 .01"/>
          <site name="passive_tip" pos=".1 0 0"/>
        </body>
        <body name="slider" pos="2 0 0"><joint name="translation" type="slide" axis="1 0 0"
            stiffness="10000" damping=".1"/>
          <inertial pos="0 0 0" mass="1" diaginertia=".01 .01 .01"/>
        </body>
        <body name="input"><joint name="swing" axis="0 0 1" armature="{armature}" damping="{damping}"/>
          <inertial pos="0 0 0" mass="1" diaginertia=".01 .01 .01"/>
          <site name="input_tip" pos=".1 0 0"/>
          <body name="rod" pos=".1 0 0"><joint name="rod_hinge" axis="0 0 1"/>
            <inertial pos=".1 0 0" mass=".1" diaginertia=".0001 .0004 .0004"/>
            <site name="rod_tip" pos=".2 0 0"/>
          </body>
        </body>
      </worldbody>{extra}<equality>{equality}</equality>
      <actuator><motor name="motor" joint="swing"/></actuator>
      <keyframe><key name="start" qpos="{key}" qvel="{velocity}"/></keyframe>
    </mujoco>'''


def passive_sim(xml=None, **kwargs):
    cfg = control()
    cfg.update(inertial_feedforward=False, gravity_and_coriolis_feedforward=False)
    cfg['drives'][0].update(kp=5., kd=.3)
    cfg['drives'][0].pop('brake')
    return make(xml=xml or passive_model(), ctrl=cfg,
                ref=lambda t: dict(q=[.6], qd=[0.], qdd=[0.]), **kwargs)


def test_undeclared_position_actuator_cannot_secretly_drive_a_passive_coordinate():
    xml = passive_model().replace('</actuator>', '<position name="hidden_servo" joint="follower" kp="10"/></actuator>')
    with pytest.raises(ValueError, match='undeclared.*hidden_servo'):
        passive_sim(xml)


@pytest.mark.parametrize('output_limit_Nm,clipped', [(.1, False), (.0001, True)])
def test_declared_rack_reports_output_clipping_and_complete_actuator_coverage(
        output_limit_Nm, clipped):
    cfg = control()
    cfg.pop('initial_keyframe')
    cfg['rack'] = dict(joints=['jaw_left', 'jaw_right'], actuator='rack_motor',
        motor=deepcopy(cfg['drives'][0]['motor']), pinion_pitch_radius_m=.01,
        max_jaw_velocity_m_s=.1, position_to_velocity_gain_per_s=10., velocity_force_gain_N_s_m=1.,
        output_torque_limit_Nm=output_limit_Nm,
        output_torque_per_current_Nm_A=.5, no_load_current_A=.01,
        current_budget=dict(continuous_current_A=1., peak_current_A=2., peak_budget_s=.1))
    xml = passive_model('').split('<keyframe>')[0] + '</mujoco>'
    jaws = ''.join(f'<body name="{name}"><joint name="{name}" type="slide" axis="1 0 0"/>'
                   '<inertial pos="0 0 0" mass="1" diaginertia=".01 .01 .01"/></body>'
                   for name in cfg['rack']['joints'])
    xml = xml.replace('</worldbody>', jaws + '</worldbody><tendon><fixed name="rack">'
        '<joint joint="jaw_left" coef="50"/><joint joint="jaw_right" coef="50"/></fixed></tendon>')
    xml = xml.replace('</actuator>', '<motor name="rack_motor" tendon="rack"/></actuator>')
    sim = make(ctrl=cfg, xml=xml, ref=lambda t: dict(q=[0.], qd=[0.], qdd=[0.], gap=.01))
    result = sim.run(.002)
    assert sim.model.nu == 2
    assert result['metrics']['joints']['jaw_left']['position_unit'] == 'm'
    assert sim.data.qfrc_actuator[sim.rack_vadr[0]] > 0
    rack = result['metrics']['rack_motor']
    assert rack['samples'] == 2 and rack['peak_current_A'] > 0
    assert rack['peak_current_A'] == pytest.approx(abs(rack['peak_absolute_current']['value']))
    assert rack['peak_output_torque_Nm'] > 0
    assert rack['source_output_torque_limit_Nm'] == output_limit_Nm
    assert rack['peak_requested_output_torque_Nm'] > 0
    assert (rack['output_demand_limit_s'] > 0) is clipped


def rotating_pinion_fixture(*, brake=True):
    """A small physical hinge/rack/coil-loss fixture, separate from ARX CAD."""
    xml = '''<mujoco model="rotating_pinion_fixture">
      <compiler angle="radian"/><option timestep=".0005" gravity="0 0 0"/>
      <worldbody>
        <body name="drive"><joint name="swing" type="hinge" axis="0 0 1"/>
          <inertial pos="0 0 0" mass=".02" diaginertia=".00001 .00001 .00001"/></body>
        <body name="pinion"><joint name="gripper_pinion" type="hinge" axis="0 0 1"/>
          <inertial pos="0 0 0" mass=".01" diaginertia=".000003 .000003 .000003"/></body>
        <body name="jaw_left"><joint name="jaw_left" type="slide" axis="1 0 0"
            ref=".03" range=".01 .04" stiffness="100" springref=".01"/>
          <inertial pos="0 0 0" mass=".02" diaginertia=".00001 .00001 .00001"/></body>
        <body name="jaw_right"><joint name="jaw_right" type="slide" axis="-1 0 0"
            ref=".03" range=".01 .04" stiffness="100" springref=".01"/>
          <inertial pos="0 0 0" mass=".02" diaginertia=".00001 .00001 .00001"/></body>
      </worldbody>
      <equality>
        <joint name="mesh_left" joint1="jaw_left" joint2="gripper_pinion" polycoef="0 .01 0 0 0"
          solref=".0005 1" solimp=".9999 .99999 .000001"/>
        <joint name="mesh_right" joint1="jaw_right" joint2="gripper_pinion" polycoef="0 .01 0 0 0"
          solref=".0005 1" solimp=".9999 .99999 .000001"/>
      </equality>
      <actuator><motor name="motor" joint="swing"/>
        <motor name="rack_motor" joint="gripper_pinion"/></actuator>
    </mujoco>'''
    cfg = control()
    cfg.pop('initial_keyframe')
    cfg.update(inertial_feedforward=False, gravity_and_coriolis_feedforward=False)
    cfg['drives'][0].pop('brake')
    rack = dict(joints=['jaw_left', 'jaw_right'], actuator='rack_motor',
        pinion_joint='gripper_pinion', pinion_body='pinion',
        motor=deepcopy(cfg['drives'][0]['motor']), pinion_pitch_radius_m=.01,
        maximum_coupling_error_m=.0001,
        max_jaw_velocity_m_s=.1, position_to_velocity_gain_per_s=10.,
        velocity_force_gain_N_s_m=1., output_torque_limit_Nm=.05,
        output_torque_per_current_Nm_A=.5, no_load_current_A=.01,
        current_budget=dict(continuous_current_A=1., peak_current_A=2., peak_budget_s=.1))
    if brake:
        rack['pinion_brake'] = dict(delay_s=.002, ramp_s=.003,
            holding_torque_Nm=.03, screening_dynamic_torque_Nm=.025)
    cfg['rack'] = rack
    return xml, cfg


def test_rotating_pinion_brake_opposes_a_native_spring_without_motor_power():
    xml, cfg = rotating_pinion_fixture()
    reference = lambda t: dict(q=[0.], qd=[0.], qdd=[0.], gap=.03)
    held = make({'power': False}, ctrl=cfg, xml=xml, ref=reference)
    free = make({'power': False, 'brakes': False}, ctrl=cfg, xml=xml, ref=reference)
    a, b = held.run(.05, output_hz=100), free.run(.05, output_hz=100)
    assert held.rack_pinion_joint == held.model.joint('gripper_pinion').id
    assert held.safety.reason == free.safety.reason == 'power-loss'
    assert held.rack_motor.state.current_A == free.rack_motor.state.current_A == 0
    assert .025 <= held.rack_brake_capacity <= .03
    assert held.rack_brake_energy > 0
    assert any(abs(frame['rack_brake']['native_constraint_torque_Nm']) > 0
               for frame in a['frames'])
    assert all(frame['rack_brake']['capacity_Nm'] == 0 for frame in b['frames'])
    assert a['metrics']['rack_brake']['engaged_steps'] > 0
    assert a['frames'][-1]['qpos'][held.rack_qadr[0]] > b['frames'][-1]['qpos'][free.rack_qadr[0]]


def test_rotating_pinion_motor_moves_both_physically_coupled_jaws():
    xml, cfg = rotating_pinion_fixture(brake=False)
    xml = xml.replace('stiffness="100"', 'stiffness="0"')
    cfg['rack']['velocity_force_gain_N_s_m'] = 50.
    sim = make(ctrl=cfg, xml=xml,
        ref=lambda t: dict(q=[0.], qd=[0.], qdd=[0.], gap=.035))
    initial = sim.data.qpos.copy()
    result = sim.run(.04, output_hz=50)
    assert sim.rack_motor.state.current_A != 0
    assert sim.data.qpos[sim.rack_qadr[0]] > initial[sim.rack_qadr[0]]
    assert sim.data.qpos[sim.rack_qadr[1]] > initial[sim.rack_qadr[1]]
    for jaw in sim.rack_qadr:
        assert sim.data.qpos[jaw] - sim.model.qpos0[jaw] == pytest.approx(
            .01 * (sim.data.qpos[sim.rack_pinion_qadr] - sim.model.qpos0[sim.rack_pinion_qadr]),
            abs=1e-5)
    assert result['metrics']['rack_brake'] is None
    assert result['metrics']['rack_coupling']['peak_absolute_error_m'] < .0001


def test_rotating_pinion_soft_mesh_cannot_report_coherent_motion():
    xml, cfg = rotating_pinion_fixture(brake=False)
    xml = xml.replace('solref=".0005 1" solimp=".9999 .99999 .000001"',
                      'solref=".02 1" solimp=".9 .95 .00001"')
    xml = xml.replace('stiffness="100"', 'stiffness="0"')
    cfg['rack']['velocity_force_gain_N_s_m'] = 50.
    sim = make(ctrl=cfg, xml=xml,
        ref=lambda t: dict(q=[0.], qd=[0.], qdd=[0.], gap=.035))
    with pytest.raises(RuntimeError, match='mesh exceeds source-authored dynamic coupling limit'):
        sim.run(.04)
    assert sim.rack_coupling_monitor['peak_absolute_error_m'] > .0001


@pytest.mark.parametrize('corruption,problem', [
    ('no_second_mesh', 'Each jaw must be actively coupled'),
    ('inactive_mesh', 'Each jaw must be actively coupled'),
    ('wrong_radius', 'pitch radius'),
    ('wrong_body', 'separate inertial hinge body'),
    ('brake_without_hinge', 'separately rotating pinion hinge'),
    ('wrong_start', 'Initial keyframe violates'),
])
def test_rotating_pinion_brake_rejects_unconnected_or_forged_transmission(corruption, problem):
    xml, cfg = rotating_pinion_fixture()
    if corruption == 'no_second_mesh':
        xml = xml.replace('name="mesh_right" joint1="jaw_right" joint2="gripper_pinion"',
                          'name="mesh_right" joint1="jaw_right" joint2="jaw_left"')
    elif corruption == 'inactive_mesh':
        xml = xml.replace('name="mesh_right" joint1="jaw_right"',
                          'name="mesh_right" active="false" joint1="jaw_right"')
    elif corruption == 'wrong_radius':
        xml = xml.replace('polycoef="0 .01 0 0 0"', 'polycoef="0 .02 0 0 0"', 1)
    elif corruption == 'wrong_body':
        cfg['rack']['pinion_body'] = 'drive'
    elif corruption == 'brake_without_hinge':
        cfg['rack'].pop('pinion_joint')
        cfg['rack'].pop('pinion_body')
    elif corruption == 'wrong_start':
        xml = xml.replace('</mujoco>',
            '<keyframe><key name="bad" qpos="0 0 .035 .03"/></keyframe></mujoco>')
        cfg['initial_keyframe'] = 'bad'
    with pytest.raises(ValueError, match=problem):
        make(ctrl=cfg, xml=xml, ref=lambda t: dict(q=[0.], qd=[0.], qdd=[0.], gap=.03))


def test_all_scalar_joint_extrema_cover_unsaved_steps_and_repeated_runs():
    oracle, sparse = passive_sim(), passive_sim()
    positions = [oracle.data.qpos.copy()]
    velocities = [oracle.data.qvel.copy()]
    for _ in range(120):
        oracle.step()
        positions.append(oracle.data.qpos.copy())
        velocities.append(oracle.data.qvel.copy())
    positions, velocities = np.array(positions), np.array(velocities)
    first = sparse.run(.06, output_hz=1)
    result = sparse.run(.06, output_hz=1)
    metrics = result['metrics']
    assert len(first['frames']) == len(result['frames']) == 2
    assert metrics['monitoring_interval'] == dict(start_time_s=0., end_time_s=.12, integration_steps=120)
    assert set(metrics['joints']) == {'follower', 'translation', 'swing', 'rod_hinge'}
    for name, qadr in [('follower', 0), ('translation', 1), ('swing', 2), ('rod_hinge', 3)]:
        record = metrics['joints'][name]
        assert record['samples'] == 121
        for event_name, index in [('minimum_position', np.argmin(positions[:, qadr])),
                                  ('maximum_position', np.argmax(positions[:, qadr]))]:
            assert record[event_name] == pytest.approx(dict(value=positions[index, qadr], time_s=index*.001))
        speed_index = np.argmax(abs(velocities[:, qadr]))
        assert record['peak_absolute_speed'] == pytest.approx(
            dict(value=velocities[speed_index, qadr], time_s=speed_index*.001))
    assert metrics['joints']['follower']['direct_motor_actuator'] is None
    assert metrics['joints']['swing']['direct_motor_actuator'] == 'motor'
    assert metrics['joints']['translation']['position_unit'] == 'm'
    assert metrics['joints']['translation']['speed_unit'] == 'm/s'
    assert metrics['joints']['follower']['position_unit'] == 'rad'
    peak = metrics['joints']['translation']['peak_absolute_speed']
    assert peak['time_s'] not in [f['time_s'] for f in first['frames'] + result['frames']]
    assert abs(peak['value']) > .9
    assert np.array_equal(sparse.data.qpos, oracle.data.qpos)
    assert sparse.data.qpos[0] > .3  # The motor moves the passive follower through the rod.
    assert sparse.data.qfrc_actuator[0] == sparse.data.qfrc_applied[0] == 0


def test_connect_residual_is_actual_solver_gap_and_inactive_rows_are_not_zero():
    sim = passive_sim(passive_model(key='.22 .01 .2 -.2'))
    gaps, forces = [], []
    for _ in range(25):
        sim.step()
        rows = sim.data.efc_type == sim.mj.mjtConstraint.mjCNSTR_EQUALITY
        gaps.append(sim.data.efc_pos[rows].copy())
        forces.append(sim.data.efc_force[rows].copy())
    # The logged vector is the solver's actual constraint-space force: its
    # Jacobian transpose recreates generalized constraint force in this model.
    generalized = np.zeros(sim.model.nv)
    sim.mj.mj_mulJacTVec(sim.model, sim.data, generalized,
                         sim.data.efc_force[:sim.data.nefc])
    np.testing.assert_allclose(generalized, sim.data.qfrc_constraint, rtol=1e-10, atol=1e-12)
    frozen = deepcopy(sim.constraint_monitor['closure'])
    sim.data.eq_active[:] = 0
    result = sim.run(.005, output_hz=1)
    record = result['metrics']['constraints']['closure']
    assert record['active_steps'] == record['sampled_steps'] == 25
    assert record['inactive_steps'] == record['missing_row_steps'] == 5
    gaps, forces = np.array(gaps), np.array(forces)
    for axis, row in enumerate(record['components']):
        index = np.argmax(abs(gaps[:, axis]))
        assert row['unit'] == 'm'
        assert row['peak_absolute']['value'] == pytest.approx(gaps[index, axis])
        # These are pre-integration solver evaluations, not end-of-step qpos.
        assert row['peak_absolute']['time_s'] == pytest.approx(index*.001)
        assert row['samples'] == 25
        reaction = row['solver_reaction']
        peak = np.argmax(abs(forces[:, axis]))
        assert reaction['unit'] == 'N' and reaction['samples'] == 25
        assert reaction['minimum']['value'] == pytest.approx(min(forces[:, axis]))
        assert reaction['maximum']['value'] == pytest.approx(max(forces[:, axis]))
        assert reaction['peak_absolute']['value'] == pytest.approx(forces[peak, axis])
        assert reaction['peak_absolute']['time_s'] == pytest.approx(peak*.001)
    norm = np.linalg.norm(gaps, axis=1)
    assert record['vector_norms']['anchor_delta']['peak_absolute']['value'] == pytest.approx(max(norm))
    reactions = np.linalg.norm(forces, axis=1)
    observed = record['reaction_vector_norms']['anchor_force']
    assert observed['unit'] == 'N' and observed['samples'] == 25
    assert observed['peak_absolute']['value'] == pytest.approx(max(reactions))
    assert observed['peak_absolute']['time_s'] == pytest.approx(np.argmax(reactions)*.001)
    assert record['incomplete_reaction_row_steps'] == 0
    assert record['vector_norms'] == frozen['vector_norms']
    assert record['reaction_vector_norms'] == frozen['reaction_vector_norms']
    assert record['components'] == frozen['components']


def test_never_active_constraint_has_no_fabricated_zero_residual():
    sim = passive_sim(passive_model().replace('name="closure"', 'name="closure" active="false"'))
    record = sim.run(.003)['metrics']['constraints']['closure']
    assert record['sampled_steps'] == record['active_steps'] == 0
    assert record['missing_row_steps'] == record['inactive_steps'] == 3
    assert record['components'] == [] and record['vector_norms'] == {}
    assert record['reaction_vector_norms'] == {}


def test_unnamed_passive_joint_does_not_overwrite_a_named_joint_monitor():
    xml = passive_model().replace('name="follower"', '').replace('name="translation"', 'name="joint_0"')
    result = passive_sim(xml).run(.002)
    joints = result['metrics']['joints']
    assert len(joints) == 4
    assert joints['joint_0']['joint_id'] == 1 and joints['joint_0']['position_unit'] == 'm'
    assert joints['_joint_0']['joint_id'] == 0 and joints['_joint_0']['position_unit'] == 'rad'


def test_rk4_constraint_event_reports_stage_interval_without_false_timestamp():
    sim = passive_sim(passive_model().replace('integrator="implicitfast"', 'integrator="RK4"'))
    record = sim.run(.003)['metrics']['constraints']['closure']
    event = record['vector_norms']['anchor_delta']['peak_absolute']
    assert event['time_s'] is None
    assert 0 <= event['interval_start_s'] < event['interval_end_s'] <= .003


@pytest.mark.parametrize('equality,extra,expected', [
    ('<joint name="closure" joint1="follower" joint2="swing"/>', '', ['rad']),
    ('<joint name="closure" joint1="translation" joint2="swing" polycoef="0 .01 0 0 0"/>', '', ['m']),
    ('<weld name="closure" body1="passive" body2="input" torquescale=".2"/>', '', ['m']*6),
    ('<tendon name="closure" tendon1="mixed"/>',
     '<tendon><fixed name="mixed"><joint joint="follower" coef=".2"/><joint joint="translation" coef="1"/></fixed></tendon>',
     ['tendon_coordinate']),
    ('<tendon name="closure" tendon1="length"/>',
     '<tendon><spatial name="length"><site site="input_tip"/><site site="passive_tip"/></spatial></tendon>', ['m']),
])
def test_equality_residual_units_follow_native_coordinate_type(equality, extra, expected):
    sim = passive_sim(passive_model(equality, extra=extra, key='.22 .01 .2 -.2'))
    result = sim.run(.002, output_hz=1)
    record = result['metrics']['constraints']['closure']
    assert [row['unit'] for row in record['components']] == expected
    dual = [row['solver_reaction']['unit'] for row in record['components']]
    assert dual == (['N']*3 if record['type'] == 'connect' else
                    ['N']*6 if record['type'] == 'weld' else
                    ['Nm'] if expected == ['rad'] else
                    ['N'] if expected == ['m'] else ['dual_to_tendon_coordinate'])
    assert record['sampled_steps'] == 2
    if record['type'] == 'weld':
        angular = record['components'][5]
        assert angular['torquescale_m'] == .2
        assert angular['label'] == 'torquescale_scaled_quaternion_z'
        assert abs(angular['peak_absolute']['value']) == pytest.approx(.2*math.sin(.02/2))
        assert 'not an angle' in angular['interpretation']


@pytest.mark.parametrize('powered', [True, False])
def test_transmission_estimate_includes_measured_rotor_acceleration_and_deceleration(powered):
    sim = passive_sim(passive_model('', armature=.02, damping=1., velocity='0 0 2 -2'), options={'power': powered})
    expected = []
    for _ in range(40):
        before = float(sim.data.qvel[2])
        sim.step()
        acceleration = (float(sim.data.qvel[2]) - before)/.001
        motor = sim.motors[0].state.torque_Nm
        expected.append((motor-.02*acceleration, motor, acceleration))
    record = sim.transmission_monitor[0]
    assert record['samples'] == 40
    for name, index in [('minimum', np.argmin([v[0] for v in expected])),
                        ('maximum', np.argmax([v[0] for v in expected])),
                        ('peak_absolute', np.argmax([abs(v[0]) for v in expected]))]:
        value, motor, acceleration = expected[index]
        event = record[name]
        assert event == pytest.approx(dict(value=value, time_s=(index+1)*.001, interval_start_s=index*.001,
            motor_output_torque_Nm=motor, armature_inertia_kg_m2=.02, acceleration_rad_s2=acceleration))
    if not powered:
        assert all(v[1] == 0 and v[2] < 0 and v[0] > 0 for v in expected)
    # A later run must preserve prior events and output an independent snapshot.
    report = sim.run(.001)['metrics']['actuators']['motor']['transmitted_torque_estimate']
    snapshot = deepcopy(report)
    sim.step()
    assert report == snapshot


def test_observers_do_not_recompute_or_change_integration_dynamics(monkeypatch):
    observed, original = passive_sim(), passive_sim()
    for name in ('_observe_joints', '_observe_constraints', '_observe_transmissions', '_observe_contacts', '_observe_rack'):
        monkeypatch.setattr(original, name, lambda *args: None)
    for _ in range(100):
        observed.step()
        original.step()
    for field in ('qpos', 'qvel', 'qacc', 'qfrc_bias', 'qfrc_constraint', 'efc_pos', 'efc_force'):
        assert np.array_equal(getattr(observed.data, field), getattr(original.data, field))
    matrices = [np.zeros((sim.model.nv, sim.model.nv)) for sim in (observed, original)]
    for sim, matrix in zip((observed, original), matrices):
        sim.mj.mj_fullM(sim.model, sim.data, matrix)
    assert np.array_equal(*matrices)
    assert observed.motors[0].state == original.motors[0].state


def test_contact_penetration_covers_initialization_and_every_unsaved_solver_step():
    xml = XML.replace('qpos="0 0 .021', 'qpos="0 0 .019')
    oracle, sparse = make(xml=xml), make(xml=xml)
    distances = [float(oracle.data.contact[i].dist) for i in range(oracle.data.ncon)]
    initial_contacts = len(distances)
    assert initial_contacts > 0
    steps_with_contacts = 0
    for _ in range(30):
        oracle.step()
        distances += [float(oracle.data.contact[i].dist) for i in range(oracle.data.ncon)]
        steps_with_contacts += int(oracle.data.ncon > 0)
    sparse.run(.015, output_hz=1)
    result = sparse.run(.015, output_hz=1)
    contacts = result['metrics']['contacts']
    assert contacts['samples'] == 30 and contacts['initial_state_sampled']
    assert len(contacts['pairs']) == 1
    pair = contacts['pairs'][0]
    assert pair['geoms'] == ['floor', 'block']
    assert pair['initial_contacts'] == initial_contacts
    assert pair['contact_samples'] == len(distances)
    assert pair['steps_with_contacts'] == steps_with_contacts
    assert pair['maximum_penetration_m'] == pytest.approx(max(0., -min(distances)))
    assert pair['maximum_penetration_m'] >= .001-1e-12
    assert pair['peak_event']['signed_distance_m'] == pytest.approx(min(distances))
    assert result['metrics']['rack_motor'] is None


def test_unnamed_contact_geom_has_a_distinct_label_from_existing_fallback_name():
    xml = XML.replace(
        '<contact><pair geom1="floor" geom2="block" friction=".8 .8 .01 .001 .001"/></contact>', ''
    ).replace('name="floor"', '').replace('name="block"', 'name="geom_0"').replace(
        'qpos="0 0 .021', 'qpos="0 0 .019')
    sim = make(xml=xml)
    assert sim.geom_names[:2] == ['_geom_0', 'geom_0']
    result = sim.run(.001, output_hz=1000)
    assert result['frames'][0]['contacts']
    for frame in result['frames']:
        assert all(contact['geoms'] == ['_geom_0', 'geom_0'] for contact in frame['contacts'])
    pair = result['metrics']['contacts']['pairs'][0]
    assert pair['geom_ids'] == [0, 1]
    assert pair['geoms'] == ['_geom_0', 'geom_0']


def test_contact_observation_preserves_actual_contact_integration(monkeypatch):
    observed, original = make(), make()
    monkeypatch.setattr(original, '_observe_contacts', lambda *args: None)
    for _ in range(100):
        observed.step()
        original.step()
    for field in ('qpos', 'qvel', 'qacc', 'qfrc_constraint', 'efc_force', 'efc_pos'):
        assert np.array_equal(getattr(observed.data, field), getattr(original.data, field))
    assert observed.motors[0].state == original.motors[0].state


def test_runtime_identity_captures_imported_source_and_is_snapshot_per_run(monkeypatch):
    import mechanism_lab.causal_runtime as runtime
    sim = make()
    identity = deepcopy(sim.implementation_identity)
    for name, expected in identity['source_files'].items():
        assert hashlib.sha256(Path(runtime.__file__).with_name(name).read_bytes()).hexdigest() == expected
    assert identity['packages']['mujoco'] == sim.mj.__version__
    assert identity['packages']['numpy'] == np.__version__
    monkeypatch.setattr(runtime, '_IMPLEMENTATION_IDENTITY', {'mutated_after_initialization': True})
    result = sim.run(.002)
    assert result['runtime_implementation'] == identity
    sim.implementation_identity['source_files']['actuation.py'] = 'changed_after_run'
    assert result['runtime_implementation'] == identity
