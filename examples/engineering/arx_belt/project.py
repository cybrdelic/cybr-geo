"""Example registration. All execution engines are shared GEO services."""
from copy import deepcopy
from collections.abc import Mapping

import numpy as np

from mechanism_lab.engineering_source import EngineeringProject
from mechanism_lab.reference_clock import PhaseClock
from .design import build
from .model import compile_model as compile_source_model, load
from .task import reference as source_reference


def clock_from_source(source):
    return PhaseClock(source_times_s=source['source_times_s'],
                      physical_times_s=source['physical_times_s'])


def authored_reference(time_s, parameters):
    """One source-derived task with its explicit, identity-bound policy."""
    return source_reference(time_s, parameters,
        lateral_tilt_scale=parameters['operating_policy']['lateral_tilt_scale'])


def requirements(result, parameters):
    """Task outcomes, separate from manufacturing or hardware qualification."""
    frames = result['frames']
    clock = PhaseClock.from_source(parameters['reference_clock'])
    period = clock.source()['physical_period_s']
    checks = [dict(name='complete nominal transfer cycle recorded', passed=result['duration_s'] >= period - 1e-9,
                   duration_s=result['duration_s'], required_s=period)]
    for source_time_s, target in ((4.9, 'B'), (9.9, 'A')):
        time_s = clock.physical_time(source_time_s)
        if result['duration_s'] < time_s:
            continue
        frame = min(frames, key=lambda row: abs(row['time_s'] - time_s))
        actual = np.asarray(frame['bodies']['puck']['position_m'])
        wanted = np.asarray(parameters['workpiece_targets_m'][target])
        error = float(np.linalg.norm(actual - wanted))
        checks.append(dict(name=f'contact-driven placement {target}', passed=error <= .0015,
                           error_m=error, limit_m=.0015, time_s=frame['time_s'], source_time_s=source_time_s))
    checks.append(dict(name='no externally applied body wrench',
        passed=result['metrics']['max_external_body_force_N'] == 0,
        value=result['metrics']['max_external_body_force_N']))
    checks.append(dict(name='no unexpected fault', passed=not result['events'], events=result['events']))
    from .faults import nominal_checks
    checks.extend(nominal_checks(result, parameters))
    return dict(passed=all(row['passed'] for row in checks), checks=checks,
        scope='Nominal contact task and unchanged source operating-demand checks. Fault scenarios are separately executed; power-loss payload retention and hardware are not qualified.')


def replay(directory, result):
    from .geometry import build_assembly
    return build_assembly(directory, result)


def create():
    specification = load('spec.json')
    specification['task_timing'] = load('default-task.json')
    clock = clock_from_source(specification['task_timing'])
    bound_reference = active_parameters = parameter_snapshot = None

    def compile_model(spec, *, native_directory=None):
        nonlocal bound_reference, active_parameters, parameter_snapshot
        policy = spec['task_timing'].get('operating_policy', {'lateral_tilt_scale': 1.0})
        if not isinstance(policy, Mapping) or set(policy) != {'lateral_tilt_scale'}:
            raise ValueError('Operating policy must contain only lateral_tilt_scale')
        physical, control, parameters = compile_source_model(spec, native_directory=native_directory)
        authored_clock = clock_from_source(spec['task_timing'])
        parameters['operating_policy'] = deepcopy(dict(policy))
        parameters['reference_clock'] = authored_clock.source()
        bound_reference = authored_clock.bind(authored_reference, parameters,
                                              **spec['task_timing']['join_validation'])
        parameters['reference_clock_validation'] = bound_reference.validation_report
        active_parameters = parameters
        parameter_snapshot = deepcopy(parameters)
        return physical, control, parameters

    def reference(time_s, parameters):
        if bound_reference is None or parameters is not active_parameters or parameters != parameter_snapshot:
            raise ValueError('Compile this project before using unchanged bound reference parameters')
        return bound_reference(time_s)

    return EngineeringProject('arx_belt', specification, build, compile_model,
                              reference, replay, requirements,
                              duration_s=clock.source()['physical_period_s'])
