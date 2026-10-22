"""Actual native integration on a one-axis mechanism with an unrelated free body.

The free body's coordinates precede the motor coordinate deliberately: the host
must resolve names, apply motor forces, and leave free-body integration alone.
"""
from copy import deepcopy
import gzip
import json
import math

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
