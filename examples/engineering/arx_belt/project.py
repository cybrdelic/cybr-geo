"""Example registration. All execution engines are shared GEO services."""
import numpy as np

from mechanism_lab.engineering_source import EngineeringProject
from .design import build
from .model import compile_model, load
from .task import reference


def requirements(result, parameters):
    """Task outcomes, separate from manufacturing or hardware qualification."""
    frames = result['frames']
    checks = [dict(name='complete nominal transfer cycle recorded', passed=result['duration_s'] >= 10. - 1e-9,
                   duration_s=result['duration_s'], required_s=10.)]
    for time_s, target in ((4.9, 'B'), (9.9, 'A')):
        if result['duration_s'] < time_s:
            continue
        frame = min(frames, key=lambda row: abs(row['time_s'] - time_s))
        actual = np.asarray(frame['bodies']['puck']['position_m'])
        wanted = np.asarray(parameters['workpiece_targets_m'][target])
        error = float(np.linalg.norm(actual - wanted))
        checks.append(dict(name=f'contact-driven placement {target}', passed=error <= .0015,
                           error_m=error, limit_m=.0015, time_s=frame['time_s']))
    checks.append(dict(name='no externally applied body wrench',
        passed=result['metrics']['max_external_body_force_N'] == 0,
        value=result['metrics']['max_external_body_force_N']))
    checks.append(dict(name='no unexpected fault', passed=not result['events'], events=result['events']))
    return dict(passed=all(row['passed'] for row in checks), checks=checks,
        scope='Nominal task outcomes only; known adverse cases remain failures. No hardware qualification.',
        known_original_failures=['power_loss_3.2', 'weak_spring_double_delay_3.2', 'weak_spring_double_delay_7.2'])


def replay(directory, result):
    from .geometry import build_assembly
    return build_assembly(directory, result)


def create():
    return EngineeringProject('arx_belt', load('spec.json'), build, compile_model,
                              reference, replay, requirements)
