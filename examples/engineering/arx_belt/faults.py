"""Source-derived ARX scenario requirements, using GEO's shared causal runtime.

Adapted from cybrdelic/cybr-arx6-belt aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2,
ARX-6-P5-Engineering/integration/validate_p5.mjs. Original case definitions,
numeric thresholds and provenance are in data/source-scenarios.json.
See NOTICE-SOURCES.md; this adaptation follows CYBR GEO's GPL-2.0-only license.

These are numerical model requirements, not a hardware safety qualification.
Ordinary emergency cases deliberately do not claim fail-safe payload retention.
No motor, brake, spring, contact or integration equations are implemented here.
"""
from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path

import numpy as np

from mechanism_lab.causal_runtime import CausalSimulation
from mechanism_lab.physical_model import PhysicalModel, digest


def source_scenarios():
    return json.loads((Path(__file__).parent / 'data/source-scenarios.json').read_text(encoding='utf-8'))


def clock_from_parameters(parameters):
    """The same authored clock schedules task landmarks and fault phases."""
    from mechanism_lab.reference_clock import PhaseClock
    receipt = parameters.get('reference_clock')
    if receipt is None:
        return PhaseClock([0., 10.], [0., 10.])
    return PhaseClock.from_source(receipt)


def scheduled_cases(parameters, names=None, *, clocks=('original-clock', 'matched-phase')):
    """Retain original event times and separately exercise the same task phases.

    Brake engagement, brake ramp, stop and watchdog durations remain physical
    seconds. In particular, a matched watchdog maps the trip phase then backs
    out its unchanged delay; it does not stretch the watchdog timer.
    """
    document = source_scenarios()
    known = {row['name'] for row in document['scenarios']}
    if names is not None and (not names or set(names) - known):
        raise ValueError('Unknown or empty source scenario selection')
    if not clocks or set(clocks) - {'original-clock', 'matched-phase'}:
        raise ValueError('Select original-clock and/or matched-phase')
    clock = clock_from_parameters(parameters)
    watchdog = float(parameters.get('watchdog_s', .1))
    rows = []
    for original in document['scenarios']:
        if names is not None and original['name'] not in names:
            continue
        timed = [key for key in original['options'] if key.endswith('Time') or key == 'heartbeatUntil']
        for scope in clocks if timed else ('same-clock-independent',):
            row = deepcopy(original)
            row.update(source_seconds=original['seconds'], source_options=deepcopy(original['options']),
                       clock_scope=scope, id=original['name'] + '--' + scope)
            # Full nominal cycles and negative contact tests retain the source
            # task-phase coverage, including a three-cycle endurance case.
            if original['kind'] == 'nominal' or original['name'] in {'friction_loss', 'weak_grip'}:
                row['seconds'] = clock.physical_time(original['seconds'])
            if scope == 'matched-phase':
                for key in timed:
                    if key == 'heartbeatUntil':
                        trip_phase = original['options'][key] + watchdog
                        row['options'][key] = clock.physical_time(trip_phase) - watchdog
                        row['seconds'] = clock.physical_time(trip_phase) + original['seconds'] - trip_phase
                    else:
                        row['options'][key] = clock.physical_time(original['options'][key])
                        if original['name'] != 'friction_loss':
                            row['seconds'] = row['options'][key] + original['seconds'] - original['options'][key]
            rows.append(row)
    return rows


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _finite_tree(value):
    if isinstance(value, dict):
        return all(_finite_tree(item) for item in value.values())
    if isinstance(value, (list, tuple)):
        return all(_finite_tree(item) for item in value)
    return not isinstance(value, (int, float)) or isinstance(value, bool) or math.isfinite(value)


def _check(name, passed, measured=None, limit=None, **detail):
    return dict(name=name, passed=bool(passed), measured=measured, limit=limit, **detail)


def _measurement_coverage(result):
    metrics, steps = result.get('metrics', {}), result.get('steps', 0)
    interval = metrics.get('monitoring_interval', {})
    if not (steps > 0 and result.get('start_time_s') == 0 and interval.get('start_time_s') == 0
            and interval.get('integration_steps') == steps and result.get('frames')):
        return False
    contacts, rack = metrics.get('contacts') or {}, metrics.get('rack_motor') or {}
    if contacts.get('samples') != steps or not contacts.get('initial_state_sampled') or rack.get('samples') != steps:
        return False
    for index in range(6):
        joint = metrics.get('joints', {}).get(f'j{index+1}', {})
        transmission = metrics.get('actuators', {}).get(f'motor{index+1}', {}).get('transmitted_torque_estimate', {})
        if joint.get('samples') != steps+1 or transmission.get('samples') != steps:
            return False
    return True


def ordinary_checks(result, parameters):
    """Unchanged common source thresholds; missing measurements fail closed.

    Electrical, transmission and joint extrema must cover every solver step.
    Contacts come from the shared solver-stage observer, plus the initial saved
    contact state, rather than interpolation of the 120 Hz display frames.
    """
    limits = source_scenarios()['thresholds']
    frames, metrics = result.get('frames', []), result.get('metrics', {})
    steps = result.get('steps', 0)
    monitoring = metrics.get('monitoring_interval', {})
    covered = (steps > 0 and result.get('start_time_s') == 0
               and monitoring.get('start_time_s') == 0 and monitoring.get('integration_steps') == steps)
    checks = [_check('all saved exported states finite', bool(frames) and _finite_tree(frames),
                     'All saved exported states', 'finite')]
    joints = metrics.get('joints', {})
    margins = []
    for index, bounds in enumerate(parameters['integration']['joint_hard_limits_rad']):
        row = joints.get(f'j{index+1}', {})
        low = (row.get('minimum_position') or {}).get('value')
        high = (row.get('maximum_position') or {}).get('value')
        if row.get('samples') == steps + 1 and row.get('position_unit') == 'rad' and _finite(low) and _finite(high):
            margins.extend([low-bounds[0], bounds[1]-high])
    margin = min(margins) if len(margins) == 12 else None
    checks.append(_check('hard travel bounds respected', covered and margin is not None and margin >= limits['hard_limit_margin_rad'], margin, limits['hard_limit_margin_rad']))
    contacts = metrics.get('contacts') or {}
    contact_valid = covered and contacts.get('samples') == steps and isinstance(contacts.get('pairs'), list)
    penetration = 0.
    for row in contacts.get('pairs', []):
        names, value = row.get('geoms', []), row.get('maximum_penetration_m')
        contact_valid = contact_valid and len(names) == 2 and _finite(value) and value >= 0
        if 'puck' not in names and _finite(value):
            penetration = max(penetration, value)
    if frames:
        for row in frames[0].get('contacts', []):
            value = row.get('distance_m')
            contact_valid = contact_valid and _finite(value) and len(row.get('geoms', [])) == 2
            if 'puck' not in row.get('geoms', []) and _finite(value):
                penetration = max(penetration, -value)
    checks.append(_check('no structural proxy penetration over 0.1 mm', contact_valid and penetration < limits['structural_penetration_m'],
                         penetration if contact_valid else None, limits['structural_penetration_m'], coverage='initial saved state and every solver step'))
    puck = metrics.get('free_bodies', {}).get('puck', {})
    free_forces = [puck.get('max_imposed_generalized_force'), puck.get('max_applied_wrench_component')]
    checks.append(_check('no imposed force on independent free workpiece', covered and free_forces == [0., 0.], free_forces, [0., 0.]))
    motors = parameters['motors']
    records = [metrics.get('actuators', {}).get(f'motor{i+1}', {}) for i in range(6)]
    specifications = [
        ('motor-generated output torque within belt service allowance', 'peak_output_torque_Nm', motors['belt_allowable_output_torque_Nm'], limits['belt_torque_tolerance_Nm']),
        ('configured arm peak current ceilings', 'peak_current_A', motors['peak_current_A'], limits['current_tolerance_A']),
        ('motor input speed ratings', 'peak_motor_speed_rad_s', motors['gear_max_input_speed_rad_s'], 0.),
        ('voltage ceilings', 'peak_voltage_V', (np.asarray(motors['bus_voltage_V'])*motors['max_pwm_duty']).tolist(), limits['voltage_tolerance_V']),
    ]
    for name, key, bounds, tolerance in specifications:
        values = [row.get(key) for row in records]
        valid = covered and all(_finite(value) and value >= 0 for value in values)
        checks.append(_check(name, valid and all(value <= bound+tolerance for value, bound in zip(values, bounds)), values, bounds, tolerance=tolerance))
    transmission, valid = [], covered
    for row in records:
        record = row.get('transmitted_torque_estimate') or {}
        value = (record.get('peak_absolute') or {}).get('value')
        valid = valid and record.get('samples') == steps and record.get('unit') == 'Nm' and _finite(value)
        transmission.append(abs(value) if _finite(value) else None)
    allowances = motors['belt_allowable_output_torque_Nm']
    checks.append(_check('estimated belt torque including rotor deceleration within service allowance',
        valid and all(value <= allowance+limits['belt_torque_tolerance_Nm'] for value, allowance in zip(transmission, allowances)), transmission, allowances,
        method='Shared runtime: motor torque minus upstream armature times actual step acceleration'))
    rack = metrics.get('rack_motor') or {}
    peak = rack.get('peak_current_A')
    checks.append(_check('configured gripper peak current ceiling', covered and rack.get('samples') == steps and _finite(peak)
        and peak <= limits['gripper_peak_current_A']+limits['gripper_current_tolerance_A'], peak, limits['gripper_peak_current_A']))
    speed = [row.get('peak_motor_speed_rad_s') for row in records]
    rpm = [value/ratio*60/(2*math.pi) for value, ratio in zip(speed, motors['gear_ratio'])] if all(_finite(value) for value in speed) else None
    checks.append(_check('belt pulley speed within handbook low-speed screen', covered and rpm is not None and max(rpm) <= limits['belt_pulley_rpm'], rpm, limits['belt_pulley_rpm']))
    return checks


def nominal_checks(result, parameters):
    """Operating demand checks for default generation; task placement is separate."""
    checks = ordinary_checks(result, parameters)
    values = [result.get('metrics', {}).get('actuators', {}).get(f'motor{i+1}', {}).get('rms_current_A') for i in range(6)]
    bounds = parameters['motors']['continuous_current_A']
    checks.append(_check('arm RMS currents below continuous ratings', all(_finite(value) and 0 <= value <= bound for value, bound in zip(values, bounds)), values, bounds))
    frames = result.get('frames', [])
    checks.append(_check('completed without safety trip', bool(frames) and frames[-1].get('mode') == 'RUN' and not result.get('events'), result.get('events'), 'RUN'))
    return checks


def evaluate_case(result, parameters, case):
    limits = source_scenarios()['thresholds']
    clock = clock_from_parameters(parameters)
    frames = result['frames']
    checks = nominal_checks(result, parameters) if case['kind'] == 'nominal' else ordinary_checks(result, parameters)
    if case['kind'] == 'nominal':
        for source_time, target in ((4.9, 'B'), (case['source_seconds']-.1, 'A')):
            time_s = clock.physical_time(source_time)
            frame = min(frames, key=lambda row: abs(row['time_s']-time_s))
            error = float(np.linalg.norm(np.asarray(frame['bodies']['puck']['position_m'])-parameters['workpiece_targets_m'][target]))
            checks.append(_check(f'contact-driven placement {target}', result['duration_s'] >= time_s and error < limits['placement_error_m'], error, limits['placement_error_m'], time_s=frame['time_s']))
    # The source general validator did not apply nominal placement requirements
    # to these controls. Retain that numeric verdict and separately ask whether
    # the same contact-driven transfer task succeeds after the perturbation.
    task_checks = []
    if case['name'] in {'friction_loss', 'weak_grip'}:
        time_s = clock.physical_time(4.9)
        frame = min(frames, key=lambda row: abs(row['time_s']-time_s))
        error = float(np.linalg.norm(np.asarray(frame['bodies']['puck']['position_m'])-parameters['workpiece_targets_m']['B']))
        task_checks.append(_check('adverse-control contact-driven placement B', result['duration_s'] >= time_s and error < limits['placement_error_m'],
            error, limits['placement_error_m'], time_s=frame['time_s'],
            scope='Separately applied source nominal task requirement; not an original general-validator verdict'))
    observation = result.get('source_fault_observation') or {}
    fault_time = observation.get('time_s')
    drop, drift, settled = 0., [0.]*6, [0.]*6
    if fault_time is not None:
        qadr, vadr = observation['qpos_addresses'], observation['dof_addresses']
        for frame in frames:
            if frame['time_s'] < fault_time:
                continue
            drift = np.maximum(drift, abs(np.asarray(frame['qpos'])[qadr]-observation['q_before'])).tolist()
            drop = max(drop, observation['puck_position_before_m'][1]-frame['bodies']['puck']['position_m'][1])
            if frame['time_s'] > fault_time+limits['settling_delay_s']:
                settled = np.maximum(settled, abs(np.asarray(frame['qvel'])[vadr])).tolist()
        if case['options'].get('brakes', True):
            currents = [row['current_A'] for row in frames[-1]['motors']]
            grip = frames[-1]['rack_motor']['current_A']
            checks.append(_check('all drives disabled after emergency', all(abs(value) < limits['drive_disabled_current_A'] for value in currents) and grip == 0, currents+[grip], 0.))
            checks.append(_check('all joints settle below 0.002 rad/s', max(settled) < limits['settling_speed_rad_s'], settled, limits['settling_speed_rad_s'], sampling='source-compatible 120 Hz after fault + 0.5 s'))
            energies = frames[-1]['brake_energy_J']
            bounds = [parameters['integration']['brakes']['large' if i < 3 else 'small']['max_single_stop_energy_J'] for i in range(6)]
            checks.append(_check('model brake work below catalog event-energy ceiling', all(value < bound for value, bound in zip(energies, bounds)), energies, bounds))
    if case['kind'] == 'normal_stop':
        held = [frame['bodies']['puck']['position_m'] for frame in frames if frame['time_s'] > case['options']['normalStopTime']+.6]
        ranges = (np.max(held, axis=0)-np.min(held, axis=0)).tolist() if held else None
        grip = frames[-1]['rack_motor']['current_A']
        checks.append(_check('normal stop retains grip power', frames[-1]['mode'] == 'HOLD' and abs(grip) > limits['hold_current_A'], [frames[-1]['mode'], grip], ['HOLD', limits['hold_current_A']]))
        checks.append(_check('held payload drift below 1 mm', ranges is not None and np.linalg.norm(ranges) < limits['held_payload_drift_m'], ranges, limits['held_payload_drift_m']))
    options = case['options']
    reason = expected_time = None
    for key, label in [('motorOffTime', 'power-loss'), ('estopTime', 'emergency-stop'), ('invalidFeedbackTime', 'feedback-invalid')]:
        if key in options:
            reason, expected_time = label, options[key]
    if 'heartbeatUntil' in options:
        reason, expected_time = 'command-watchdog', options['heartbeatUntil'] + float(parameters.get('watchdog_s', .1))
    events = result['events']
    evidence_valid = _measurement_coverage(result)
    valid = evidence_valid and result['duration_s'] >= case['seconds']-result['timestep_s']*.5-1e-9
    if reason:
        valid = valid and len(events) == 1 and events[0]['reason'] == reason and fault_time is not None and abs(fault_time-expected_time) <= result['timestep_s']+1e-8
    elif 'frictionChangeTime' in options:
        valid = valid and not any(event['t'] < options['frictionChangeTime']-1e-9 for event in events)
    elif not case['deliberate_negative_control']:
        if case['kind'] == 'normal_stop':
            valid = valid and fault_time is None and len(events) == 2 and frames[-1]['mode'] == 'HOLD'
        else:
            valid = valid and not events
    numeric = all(row['passed'] for row in checks)
    task_passed = all(row['passed'] for row in task_checks)
    passed = numeric and valid and task_passed
    return dict(schema='cybrgeo-arx-source-case-result/1', case=deepcopy(case), checks=checks,
        numeric_acceptance_passed=numeric, original_source_numeric_acceptance_passed=numeric,
        task_outcome_checks=task_checks, task_outcomes_passed=task_passed,
        measurement_coverage_valid=evidence_valid, scenario_valid=bool(valid), passed=bool(passed),
        expected_outcome_observed=(evidence_valid and valid and (not task_passed if task_checks else not numeric)) if case['deliberate_negative_control'] else bool(passed),
        events=events, expected_fault_time_s=expected_time, actual_fault_time_s=fault_time,
        payload=dict(max_observed_fault_drop_m=drop, required_by_ordinary_source_fault_checks=False,
                     scope='Observed free-payload drop; no fail-safe retention claim'),
        max_fault_travel_deg=(np.asarray(drift)*180/math.pi).tolist(), max_speed_after_fault_deadline_rad_s=settled,
        model_sha256=result['model_sha256'], control_sha256=result['control_sha256'], reference_sha256=result['reference_sha256'],
        reference_source=result.get('reference_source'), timestep_s=result['timestep_s'], duration_s=result['duration_s'],
        source_scenarios_sha256=digest(source_scenarios()), qualification='Numerical model requirements only; no hardware qualification')


def run_case(physical_source, control, parameters, reference, case, *, reference_source):
    """Run the shared engine; observe the actual first emergency onset only."""
    physical = PhysicalModel.from_source(physical_source)
    if 'timestep_s' in case:
        physical.root.find('option').set('timestep', format(case['timestep_s'], '.17g'))
    simulation = CausalSimulation(physical, control, reference, options=case['options'], reference_source=reference_source)
    observation = {}
    original_step = simulation.step
    puck = simulation.model.body('puck').id
    joint = int(simulation.model.body_jntadr[puck])
    address = int(simulation.model.jnt_qposadr[joint])
    def observed_step():
        before = simulation.data.qpos[simulation.qadr].copy()
        puck_before = simulation.data.qpos[address:address+3].copy()
        original_step()
        if not observation and simulation.safety.fault_time_s is not None:
            observation.update(time_s=simulation.safety.fault_time_s,
                qpos_addresses=simulation.qadr.tolist(), dof_addresses=simulation.vadr.tolist(),
                q_before=before.tolist(), puck_position_before_m=puck_before.tolist())
    simulation.step = observed_step
    result = simulation.run(case['seconds'], output_hz=120)
    result['source_fault_observation'] = observation
    result['source_case'] = deepcopy(case)
    result['source_case_requirements'] = evaluate_case(result, parameters, case)
    return result
