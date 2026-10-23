"""Numeric parity with original ARX JavaScript plus independent causal checks."""
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess

import pytest

from mechanism_lab.actuation import (
    CurrentBudget, DelayedBrake, GearedDCMotor, MotorState, SafetyState,
    SOURCE_PROVENANCE, TensionSpring,
)

FIXTURES = Path(__file__).parent / 'fixtures' / 'arx_actuation'
SOURCE = json.loads((FIXTURES / 'parameters.json').read_text(encoding='utf-8'))


@pytest.fixture(scope='module')
def oracle():
    node = shutil.which('node')
    if not node:
        pytest.skip('Node is required to execute the original ARX JavaScript parity oracle')
    def run(payload):
        result = subprocess.run([node, str(FIXTURES / 'oracle.mjs')], input=json.dumps(payload),
                                text=True, capture_output=True, check=True, timeout=30)
        return json.loads(result.stdout)
    return run


def motor_config(index):
    return {key: values[index] for key, values in SOURCE['motors'].items() if isinstance(values, list)}


def test_original_source_extract_identity():
    expected = {
        'source-safety.mjs': SOURCE_PROVENANCE['safety.mjs_sha256'],
        'source-motor-block.js': '05b23453004cb11fc04ddaa966eb2cf6c19f2cca437d34c5becd0f47da348ab4',
        'source-gripper-block.js': '2e9b803e4fcd33b482d59648fa44cf6aca4a935620f2420f7569066f1ff4bbcd',
        'source-current-budget-block.js': '7db649651306e40c29af75603e43b786a3c0c1f15c0766d25bec8112c4f0147a',
        'parameters.json': '0d4aff6a35eb3335d9867752ce6d82184a357c48ae469247866bf0dfd810dbfa',
    }
    assert SOURCE['source_commit'] == SOURCE_PROVENANCE['repository_commit']
    for name, digest in expected.items():
        assert hashlib.sha256((FIXTURES / name).read_bytes()).hexdigest() == digest


@pytest.mark.parametrize('index', range(6))
def test_original_arm_motor_electrical_thermal_and_energy_parity(oracle, index):
    config = motor_config(index)
    steps = [dict(demand=amplitude*math.sin(k*.13), speed=speed*math.cos(k*.11),
                  dt=1/2400, power=k % 71 != 70, scale=scale)
             for k in range(240)
             for amplitude, speed, scale in [(80, 3, 1), (30, -8, .55), (10, 20, 0.)]]
    expected = oracle(dict(kind='motor', config=config, current=.4, temperature=33., steps=steps))
    motor = GearedDCMotor(config, MotorState(current_A=.4, temperature_C=33.))
    fields = {'current_A':'current', 'temperature_C':'temperature', 'voltage_V':'voltage',
              'torque_Nm':'torque', 'electrical_energy_J':'electricalEnergy',
              'copper_heat_J':'copperHeat', 'mechanical_energy_J':'mechanicalEnergy', 'gear_heat_J':'gearHeat'}
    for step, original in zip(steps, expected):
        result = motor.step(step['demand'], step['speed'], step['dt'], power=step['power'], torque_scale=step['scale'])
        for key, source_key in fields.items():
            assert result[key] == pytest.approx(original[source_key][0], rel=2e-12, abs=2e-12)
        assert result['belt_demand_limit_s'] == pytest.approx(original['monitor']['beltDemandLimitSeconds'][0])
        assert result['gear_demand_limit_s'] == pytest.approx(original['monitor']['gearDemandLimitSeconds'][0])
    assert result['gear_heat_J'] >= 0


def test_original_gripper_current_clamp_and_back_emf_parity(oracle):
    config = SOURCE['gripper']
    steps = [dict(demand=.5*math.sin(k*.2), speed=2*math.cos(k*.17), dt=1/2400,
                  power=k % 80 != 79, scale=1 if k < 130 else .5,
                  ceiling=.35 if k < 100 else .1875) for k in range(250)]
    expected = oracle(dict(kind='gripper', config=config, current=.2, temperature=29., steps=steps))
    motor = GearedDCMotor(config, MotorState(current_A=.2, temperature_C=29.))
    fields = {'current_A':'gripperCurrent', 'temperature_C':'gripperTemperature',
              'voltage_V':'gripperVoltage', 'torque_Nm':'gripperTorque',
              'current_squared_integral_A2_s':'gripperI2Integral'}
    for step, original in zip(steps, expected):
        result = motor.step(step['demand'], step['speed'], step['dt'], power=step['power'],
                            torque_scale=step['scale'], current_ceiling_A=step['ceiling'], clamp_actual_current=True)
        for key, source_key in fields.items():
            assert result[key] == pytest.approx(original[source_key], rel=2e-12, abs=2e-12)
        assert abs(result['current_A']) <= step['ceiling']*step['scale']+1e-12


def test_rl_response_voltage_saturation_and_power_loss_are_causal():
    config = dict(gear_ratio=1, gear_efficiency=1, torque_constant_Nm_A=1,
        resistance_ohm=1, inductance_H=2, bus_voltage_V=2, peak_current_A=10,
        gear_efficiency_transition_motor_rad_s=3, thermal_resistance_K_W=5,
        thermal_capacity_J_K=10)
    motor = GearedDCMotor(config)
    record = motor.step(1, 0, .1)
    assert record['current_A'] == pytest.approx(1-math.exp(-.05))
    assert record['torque_Nm'] == record['current_A']
    assert record['temperature_C'] > 25
    # Back EMF greater than the supply reverses current despite positive demand.
    saturated = GearedDCMotor(config).step(1, 10, 1)
    assert saturated['voltage_V'] == 2
    assert saturated['current_A'] < 0
    assert saturated['electrical_energy_J'] < 0
    assert saturated['torque_Nm'] < 0
    loss = motor.step(1, 0, .1, power=False)
    assert loss['current_A'] == loss['torque_Nm'] == loss['voltage_V'] == 0
    assert loss['temperature_C'] < record['temperature_C']


def test_original_spring_torque_energy_parity_and_arbitrary_axis(oracle):
    cases = [dict(q=q, strength=strength, index=index) for index in (1, 2)
             for strength in (0, .35, 1) for q in (-.2, 0, .6, 1.5, 2.4)]
    expected = oracle(dict(kind='spring', spec=SOURCE['spec'], cases=cases))
    for case, original in zip(cases, expected):
        config = SOURCE['spec']['counterbalance' if case['index'] == 1 else 'elbow_counterbalance']
        result = TensionSpring(config).state(case['q'], case['strength'])
        for key in original:
            assert result[key] == pytest.approx(original[key], rel=2e-12, abs=2e-12)
    config = SOURCE['spec']['elbow_counterbalance']
    spring = TensionSpring(config)
    q, epsilon = 1.5, 1e-6
    derivative = (spring.state(q+epsilon)['potential_energy_J']-spring.state(q-epsilon)['potential_energy_J'])/(2*epsilon)
    assert spring.state(q)['torque_Nm'] == pytest.approx(-derivative, rel=1e-8)
    # Proper rotation [x,y,z] -> [z,x,y] preserves the physical spring torque.
    rotated = {**config, **{key:[config[key][2], config[key][0], config[key][1]]
                           for key in ('parent_anchor_m', 'child_anchor_m')}}
    arbitrary = TensionSpring(rotated, axis=(-1, 0, 0)).state(q)
    assert arbitrary == pytest.approx(spring.state(q))


def test_original_brake_delay_ramp_and_current_budget_parity(oracle):
    config = SOURCE['spec']['brakes']['large']
    cases = [dict(time=t, fault=fault, scale=scale) for fault in (None, .1)
             for scale in (.5, 1, 2) for t in (0, .1, .115, .125, .16)]
    expected = oracle(dict(kind='brake', config=config, cases=cases))
    brake = DelayedBrake(config)
    for case, original in zip(cases, expected):
        assert brake.fraction(case['time'], case['fault'], case['scale']) == pytest.approx(original)
    assert brake.capacity(.12, .1, .02) == pytest.approx(0, abs=1e-12)
    assert brake.capacity(.2, .1, .02) == config['screening_dynamic_torque_Nm']
    assert brake.capacity(.2, .1, 0) == config['holding_torque_Nm']
    budget_config = SOURCE['spec']['gripper_driver']
    steps = [dict(current=current, dt=.01) for current in ([.3]*30+[.16]*8+[.1]*25)]
    expected = oracle(dict(kind='budget', config=budget_config, steps=steps))
    budget = CurrentBudget(budget_config)
    for step, original in zip(steps, expected):
        assert budget.step(step['current'], step['dt']) == pytest.approx(original['ceiling'])
        assert budget.credit_s == pytest.approx(original['credit'])


@pytest.mark.parametrize('fault_options,reason', [({'estop':True},'emergency-stop'),
    ({'power':False},'power-loss'), ({'feedbackValid':False},'feedback-invalid')])
def test_original_fault_latching_and_controlled_stop_parity(oracle, fault_options, reason):
    limits = [[-2, 2], [-1, 1], [0, 3]]
    q, qd = [.3, -.4, 1.4], [.5, -.2, .3]
    normal = dict(q=q, qd=qd, qdd=[0]*3, gap=.025, stage=2)
    operations = [dict(action='command', time=0), dict(action='stop', time=0, q=q, qd=qd, gap=.025)]
    operations += [dict(action='command', time=t) for t in (.1, .2, .35)]
    operations += [dict(action='update', time=.35, q=q, qd=[0]*3),
                   dict(action='update', time=.36, q=q, qd=[0]*3, options=fault_options),
                   dict(action='update', time=.37, q=q, qd=[0]*3, options={'power':False})]
    expected = oracle(dict(kind='safety', limits=limits, normal=normal, operations=operations))
    safety = SafetyState(limits)
    for op, original in zip(operations, expected):
        if op['action'] == 'command': safety.command(op['time'])
        if op['action'] == 'stop': safety.normal_stop(op['time'], op['q'], op['qd'], gap=op['gap'])
        if op['action'] == 'update':
            options = {('feedback_valid' if k == 'feedbackValid' else k):v for k, v in op.get('options', {}).items()}
            safety.update(op['time'], op['q'], op['qd'], **options)
        assert safety.mode == original['mode']
        assert safety.reason == original['reason']
        assert safety.fault_time_s == original['fault_time_s']
        assert safety.events == original['events']
        result = safety.reference(op['time'], normal)
        for key, value in original['reference'].items():
            assert result[key] == pytest.approx(value)
    assert safety.reason == reason


def test_watchdog_limit_stop_envelope_and_invalid_feedback():
    safety = SafetyState([[-1, 1]])
    safety.update(.1, [0], [0])
    assert safety.mode == 'RUN'
    safety.update(.10001, [0], [0])
    assert safety.reason == 'command-watchdog'
    safety = SafetyState([[-1, 1]])
    safety.normal_stop(0, [.95], [1])
    assert safety.reason == 'controlled-stop-envelope'
    safety = SafetyState([[-1, 1]])
    safety.update(0, [1.003], [0])
    assert safety.reason == 'software-travel-limit'
    safety = SafetyState([[-1, 1]])
    safety.update(0, [0], [float('nan')])
    assert safety.reason == 'feedback-invalid'


def test_native_brake_work_is_attribution_not_an_extra_force():
    record = DelayedBrake.constraint_work(-10, 12, 3, 2, 1, .01)
    assert record == pytest.approx(dict(torque_Nm=-8, energy_increment_J=.12))
    assert DelayedBrake.constraint_work(10, 12, 3, 2, 1, .01)['energy_increment_J'] == 0
    assert DelayedBrake.constraint_work(-10, 0, 0, 2, 1, .01) == dict(torque_Nm=0, energy_increment_J=0)


@pytest.mark.parametrize('key,value', [('resistance_ohm',0),('inductance_H',-1),
    ('gear_efficiency',1.01),('torque_constant_Nm_A',float('nan')),('max_pwm_duty',2)])
def test_invalid_physical_parameters_fail_before_integration(key, value):
    with pytest.raises(ValueError):
        GearedDCMotor({**motor_config(0),key:value})
