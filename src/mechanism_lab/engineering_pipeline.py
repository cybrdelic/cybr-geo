"""One mechanical source drives construction, admissible motion and simulation.

This is orchestration over the existing Assembly boundary, not another geometry
or physics model. A failed analysis never becomes an authorized motion record.
"""
from __future__ import annotations

from copy import deepcopy
import gzip
import hashlib
import json
import math
from pathlib import Path
import tempfile

import numpy as np


def compile_source(source='compact_arm', parameters=None):
    from .construction import compile_mechanism, compile_plan
    if isinstance(source, dict):
        document = deepcopy(source)
        name = document.get('name', 'mechanism')
    elif str(source).endswith('.json'):
        path = Path(source)
        document = json.loads(path.read_text(encoding='utf-8'))
        name = path.stem
    else:
        adapted = compile_plan(str(source), parameters)
        return compile_mechanism(adapted['construction']['mechanism']), str(source)
    if document.get('schema') == 'cybrgeo-mechanism/2':
        if parameters:
            raise ValueError('Edit the engineering document directly; legacy parameter overrides do not apply to v2 sources')
        from .importers import safe_name
        return compile_mechanism(document), safe_name(name)
    if document.get('schema') == 'cybrgeo-mechanism/1':
        adapted = compile_plan(document['architecture'], document.get('parameters'))
        return compile_mechanism(adapted['construction']['mechanism']), name
    raise ValueError('Expected a cybrgeo-mechanism/2 engineering document')


def _finite(value, label, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0 or (positive and value == 0):
        raise ValueError(f'{label} must be finite and ' + ('positive' if positive else 'nonnegative'))
    return float(value)


def _matrix(value, rows, columns, label):
    raw = np.asarray(value)
    if raw.dtype.kind not in 'fiu' or raw.shape != (rows, columns):
        raise ValueError(f'{label} has an invalid numeric matrix shape')
    result = raw.astype(float, copy=False)
    if not np.isfinite(result).all():
        raise ValueError(f'{label} contains nonfinite values')
    return result


def _trace_evidence(plan, simulation, graph, motion_task):
    """Cross-check the maxima against every recorded integration state."""
    if simulation.get('backend') != 'MuJoCo':
        raise ValueError('The engineering pipeline requires actual MuJoCo simulation evidence')
    construction = plan.get('construction') or {}
    for field in ('recipe_sha256', 'mechanism_sha256', 'model_sha256'):
        if simulation.get(field) != construction.get(field):
            raise ValueError(f'Simulation {field} differs from the engineering source')
    trace = simulation.get('integration_state_trace')
    if not isinstance(trace, dict) or trace.get('sampling') != 'every_integration_step_including_initial_and_final':
        raise ValueError('Simulation must retain every integration state, including initial and final states')
    steps = simulation.get('integration_steps')
    if isinstance(steps, bool) or not isinstance(steps, int) or steps < 1:
        raise ValueError('Simulation integration step count is invalid')
    count = steps + 1
    names, controls = trace.get('joint_names', []), trace.get('control_names', [])
    movable = {name for name, joint in graph.by_name.items() if joint['type'] != 'fixed'}
    if (not isinstance(names, list) or not isinstance(controls, list)
            or len(set(names)) != len(names) or set(names) != movable
            or len(set(controls)) != len(controls) or set(controls) != set(graph.controls)):
        raise ValueError('Integration trace must name every movable joint and independent control exactly once')
    values = _matrix(trace.get('joint_values'), count, len(names), 'Integration states')
    requested = _matrix(trace.get('requested_control_values'), count, len(controls), 'Requested controls')
    times = np.asarray(trace.get('times_s'), dtype=float)
    duration = _finite(simulation.get('seconds'), 'Simulation duration', positive=True)
    timestep = _finite(simulation.get('timestep_s'), 'Simulation timestep', positive=True)
    if (times.shape != (count,) or not np.isfinite(times).all() or abs(times[0]) > 1e-12
            or not np.all(np.diff(times) > 0) or not np.allclose(np.diff(times), timestep, rtol=1e-8, atol=1e-12)
            or not math.isclose(times[-1], duration, rel_tol=1e-10, abs_tol=1e-12)):
        raise ValueError('Integration trace times do not cover every declared solver step')
    expected_requests = []
    for timestamp in times:
        target = motion_task.sample(float(timestamp))['values']
        expected_requests.append([target[name] for name in controls])
    expected_requests = np.array(expected_requests)
    if not np.allclose(requested, expected_requests, rtol=1e-10, atol=1e-10):
        raise ValueError('Requested integration targets disagree with the canonical motion task and its period')
    columns = {name: index for index, name in enumerate(names)}
    metrics = simulation.get('metrics') or {}
    control_maxima, drive_maxima, limit_maxima = {}, {}, {}
    for index, name in enumerate(controls):
        measured = float(np.max(np.abs(values[:, columns[name]] - requested[:, index])))
        recorded = _finite((metrics.get(name) or {}).get('max_tracking_error'), f'{name} tracking maximum')
        expected_unit = 'deg' if graph.by_name[name]['type'] == 'revolute' else 'mm'
        if (metrics.get(name) or {}).get('unit') != expected_unit:
            raise ValueError(f'{name}: recorded tracking units disagree with the joint type')
        if not math.isclose(measured, recorded, rel_tol=1e-8, abs_tol=1e-10):
            raise ValueError(f'{name}: recorded tracking maximum disagrees with the complete integration trace')
        control_maxima[name] = max(measured, recorded)
    for name in names:
        joint = graph.by_name[name]
        actual = values[:, columns[name]]
        limit_maxima[name] = max(0., float(np.max(joint['min']-actual)), float(np.max(actual-joint['max'])))
        recorded_limit = _finite((simulation.get('joint_limit_violations_native') or {}).get(name), f'{name} limit residual')
        if not math.isclose(limit_maxima[name], recorded_limit, rel_tol=1e-8, abs_tol=1e-10):
            raise ValueError(f'{name}: recorded joint-limit residual disagrees with the complete integration trace')
        drive = joint.get('drive')
        if drive:
            expected = values[:, columns[drive['joint']]] * drive['ratio'] + drive['offset']
            measured = float(np.max(np.abs(actual-expected)))
            recorded = _finite((simulation.get('drive_constraint_errors_native') or {}).get(name), f'{name} drive residual')
            if not math.isclose(measured, recorded, rel_tol=1e-8, abs_tol=1e-10):
                raise ValueError(f'{name}: recorded drive residual disagrees with the complete integration trace')
            drive_maxima[name] = max(measured, recorded)
    for field, observed in (('maximum_joint_limit_violation_native', max(limit_maxima.values(), default=0.)),
                            ('maximum_drive_constraint_error_native', max(drive_maxima.values(), default=0.))):
        recorded = _finite(simulation.get(field), field)
        if not math.isclose(observed, recorded, rel_tol=1e-8, abs_tol=1e-10):
            raise ValueError(f'{field} disagrees with per-joint integration evidence')
    return dict(trace=trace, names=names, controls=controls, values=values, requested=requested, times=times,
                control_maxima=control_maxima, drive_maxima=drive_maxima, limit_maxima=limit_maxima)


def _padded(bounds):
    # This is an outward numerical margin around one observed run, not a
    # prediction that tracking stays this accurate under future disturbances.
    return {name: math.nextafter(value * 1.05 + 1e-6, math.inf) for name, value in bounds.items()}


def _requested_path_mask(evidence, waypoints):
    actual, controls = evidence['requested'], evidence['controls']
    accepted = np.zeros(len(actual), dtype=bool)
    for first, last in zip(waypoints, waypoints[1:]):
        start = np.array([first[n] for n in controls])
        delta = np.array([last[n]-first[n] for n in controls])
        length = float(delta @ delta)
        parameter = np.zeros(len(actual)) if length == 0 else (actual-start) @ delta / length
        projected = start + np.clip(parameter, 0., 1.)[:, None] * delta
        accepted |= np.max(np.abs(actual-projected), axis=1, initial=0.) <= 1e-9
    return accepted


def _membership(plan, certificate, simulation, evidence):
    from .motion_envelope import CertifiedTube
    checker = CertifiedTube(plan, certificate)
    membership = checker.contains_states(evidence['names'], evidence['values'])
    on_path = _requested_path_mask(evidence, certificate['waypoints'])
    samples = simulation.get('samples')
    if not isinstance(samples, list) or not samples:
        raise ValueError('Simulation output samples are missing')
    sample_rows, times = [], evidence['times']
    for sample in samples:
        values = sample.get('joint_values')
        if not isinstance(values, dict) or set(values) != set(evidence['names']):
            raise ValueError('Each simulation output sample must contain all actual movable joint values')
        timestamp = _finite(sample.get('time_s'), 'Sample time')
        nearest = int(np.argmin(np.abs(times-timestamp)))
        row = [values[n] for n in evidence['names']]
        if abs(times[nearest]-timestamp) > 1e-9 or not np.allclose(row, evidence['values'][nearest], rtol=1e-10, atol=1e-10):
            raise ValueError('Output sample disagrees with its recorded integration state')
        sample_rows.append(row)
    sampled = checker.contains_states(evidence['names'], sample_rows)
    outside = np.flatnonzero(~membership)
    off_path = np.flatnonzero(~on_path)
    return dict(passed=bool(membership.all() and sampled.all() and on_path.all()),
                integration_states_checked=len(membership), output_states_checked=len(sampled),
                integration_states_outside_tube=int(len(outside)), output_states_outside_tube=int((~sampled).sum()),
                requested_states_outside_path=int(len(off_path)),
                first_outside_times_s=evidence['times'][outside[:16]].tolist(),
                first_off_path_request_times_s=evidence['times'][off_path[:16]].tolist(),
                tracking_and_residual_maxima_match_complete_trace=True,
                between_step_containment='not_established',
                scope='Every recorded integration state and output joint state is checked against the continuous geometric tube. This does not prove containment between integration states or of interpolated body-pose replay.')


def _persist_trace(directory, trace, artifact=None):
    target = directory/'simulation'/'integration-state-trace.json.gz'
    target.parent.mkdir(parents=True, exist_ok=True)
    if artifact:
        if (artifact.get('path') != target.name or not target.is_file()
                or hashlib.sha256(target.read_bytes()).hexdigest() != artifact.get('sha256')):
            raise ValueError('The saved integration trace artifact is missing or has a different digest')
        with gzip.open(target, 'rt', encoding='utf-8') as stream:
            if json.load(stream) != trace:
                raise ValueError('The saved integration trace differs from measured in-memory states')
    else:
        with gzip.open(target, 'wt', encoding='utf-8') as stream:
            json.dump(trace, stream, separators=(',', ':'), allow_nan=False)
    return dict(path='simulation/integration-state-trace.json.gz', sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
                states=len(trace['times_s']), sampling=trace['sampling'], joint_names=trace['joint_names'])


def evaluate_plan(plan, output=None, seconds=None, timestep=.002, *, assembly=None):
    """Simulate once, certify its padded error tube once, then authorize motion.

    The continuous geometry proof covers the whole requested path and declared
    joint-state tube. Simulation membership is checked at every integrator state;
    containment between those states is explicitly not established. Supplying an
    analytic Assembly reuses geometry after checking exact plan identity.
    """
    from .constructed import KinematicGraph, build_from_plan
    from .motion import MotionTask
    from .motion_envelope import certify_path, plan_identity
    from .dynamics import simulate, plot_simulation
    plan = deepcopy(plan)
    graph = KinematicGraph([p['name'] for p in plan['parts']], plan['joints'], (plan.get('construction') or {}).get('motion'))
    lowered_motion = (plan.get('construction') or {}).get('motion')
    motion_task = MotionTask.from_dict(lowered_motion) if lowered_motion else MotionTask.from_joints(plan['joints'])
    waypoints = motion_task.waypoints
    for point in waypoints:
        graph.resolve(point)
    duration = _finite(seconds if seconds is not None else motion_task.period_s, 'Simulation duration', positive=True)
    if assembly is None:
        assembly = build_from_plan(plan)
    elif plan_identity(assembly.metadata.get('construction_recipe')) != plan_identity(plan):
        raise ValueError('The reused analytic Assembly does not match the exact engineering plan')
    if any(getattr(part, 'cad', None) is None for part in assembly.parts):
        raise ValueError('Engineering evaluation requires actual analytic CAD; mesh-only cached geometry cannot be certified')
    if {part.name for part in assembly.parts} != {part['name'] for part in plan['parts']}:
        raise ValueError('Reused Assembly part names differ from its engineering plan')
    for key in ('engineering_run', 'motion_clearance', 'simulation_summary'):
        assembly.metadata.pop(key, None)
    directory = Path(output) if output is not None else Path(tempfile.mkdtemp(prefix='cybr-geo-engineering-'))
    directory.mkdir(parents=True, exist_ok=True)
    for filename in ('motion-envelope.json', 'engineering.json'):
        (directory/filename).unlink(missing_ok=True)
    report = dict(plan=plan, clearance=None, simulation=None, simulation_clearance=None,
                  passed=False, errors=[], stage='construction',
                  motion_authorization=dict(allowed=False, mode='exact_certified_path_only',
                                            api='mechanism_lab.motion_envelope.authorized_command'))
    simulation = evidence = None
    try:
        simulation = simulate(assembly, seconds=duration, timestep=timestep, output_hz=20, output=directory/'simulation')
        report['simulation'] = {key: value for key, value in simulation.items() if key != 'integration_state_trace'}
        if simulation.get('tracking_passed') is not True:
            report['errors'].append(dict(stage='simulation', code='tracking_limit', message='Integrated joint tracking exceeded the configured tolerance.'))
        if simulation.get('operating_limits_passed') is not True:
            report['errors'].append(dict(stage='simulation', code='operating_limits', message='Integrated speed or acceleration exceeded the declared operating limits.'))
        evidence = _trace_evidence(plan, simulation, graph, motion_task)
        report['integration_trace_artifact'] = _persist_trace(directory, evidence['trace'], simulation.get('integration_trace_artifact'))
        report['simulation']['integration_trace_artifact'] = {**report['integration_trace_artifact'], 'path': 'integration-state-trace.json.gz'}
        if max(evidence['limit_maxima'].values(), default=0.) > .05:
            report['errors'].append(dict(stage='simulation', code='joint_limit_residual', message='The compliant joint-limit residual exceeded 0.05 native joint units.'))
        if max(evidence['drive_maxima'].values(), default=0.) > .05:
            report['errors'].append(dict(stage='simulation', code='drive_residual', message='The driven-joint constraint residual exceeded 0.05 native joint units.'))
        report['error_bound_derivation'] = dict(independent_tracking_maxima=evidence['control_maxima'],
                                               drive_residual_maxima=evidence['drive_maxima'],
                                               padding='maximum * 1.05 + 1e-6 native units, rounded outward',
                                               evidence='All integration states including initial and final; reported maxima cross-checked against the full trace.',
                                               scope='Bounds describe this observed run with numerical padding; future tracking accuracy is not guaranteed.')
    except (ValueError, RuntimeError) as error:
        report['errors'].append(dict(stage='simulation', code='invalid_simulation_evidence', message=str(error)))
        evidence = None
    report['stage'] = 'simulation'
    if simulation is not None:
        try:
            plot_simulation(simulation, directory/'simulation-tracking.svg')
        except (ValueError, RuntimeError) as error:
            report['errors'].append(dict(stage='simulation_plot', message=str(error)))
    if evidence is not None:
        try:
            clearance = certify_path(assembly, waypoints=waypoints,
                                     joint_error_bounds=_padded(evidence['control_maxima']),
                                     drive_error_bounds=_padded(evidence['drive_maxima']))
            report['clearance'] = clearance
            (directory/'motion-envelope.json').write_text(json.dumps(clearance, indent=2)+'\n', encoding='utf-8')
            if not clearance['passed']:
                report['errors'].append(dict(stage='clearance', message='The complete requested path and observed-error tube could not be certified; motion is not authorized.'))
            else:
                report['simulation_clearance'] = _membership(plan, clearance, simulation, evidence)
                if not report['simulation_clearance']['passed']:
                    report['errors'].append(dict(stage='simulation_clearance', message='An actual integration/output state escaped the certified tube, or a request departed from the certified path.'))
        except (ValueError, RuntimeError) as error:
            report['errors'].append(dict(stage='clearance', message=str(error)))
    report['stage'] = 'complete'
    report['passed'] = bool(not report['errors'] and report['clearance'] and report['clearance']['passed']
                            and report['simulation_clearance'] and report['simulation_clearance']['passed'])
    report['motion_authorization'].update(allowed=report['passed'], plan_sha256=plan_identity(plan),
                                          execute='Only the canonical values returned by authorized_command; arbitrary independent commands remain unverified.')
    report['scope'] = ('One engineering source compiled to actual CAD, MuJoCo dynamics, and a continuous geometric clearance tube around its requested path. '
                       'Every recorded integration state is checked; between-step containment and interpolated body-pose replay are not certified. '
                       'Contact dynamics is limited to declared proxies. No hardware qualification.')
    dynamics_passed = bool(simulation is not None and evidence is not None
                           and not any(row['stage'] == 'simulation' for row in report['errors']))
    report['qualification'] = dict(
        kinematics='continuous CAD path and joint-error tube certified' if report['clearance'] and report['clearance']['passed'] else 'continuous CAD motion not certified',
        dynamics=('actual MuJoCo run passed' if dynamics_passed else 'actual MuJoCo run failed engineering checks')
                  if simulation is not None else 'MuJoCo simulation failed before completed evidence',
        dynamics_run_passed=dynamics_passed, hardware='not qualified')
    assembly.metadata['qualification'] = dict(report['qualification'])
    assembly.metadata['engineering_run'] = {key: value for key, value in report.items() if key not in ('plan', 'simulation')}
    if report['passed']:
        assembly.metadata['motion_clearance'] = report['clearance']
    if report['simulation']:
        assembly.metadata['simulation_summary'] = {key: value for key, value in report['simulation'].items()
                                                   if key not in ('samples', 'model', 'integration_state_trace')}
    (directory/'engineering.json').write_text(json.dumps({key: value for key, value in report.items() if key != 'simulation'}, indent=2)+'\n', encoding='utf-8')
    return assembly, report
