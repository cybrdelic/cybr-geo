"""Replay pinned ARX source requirements through the current shared GEO core.

Example (after a fresh source-bound native build):
    python tools/check_arx_faults.py --native outputs/arx/native --output outputs/arx-faults

The 28 source cases include four deliberately adverse controls. Timed cases run
at both their original clock times and equivalent authored task phases. The
default three independent workers never construct CAD. Use --source-mass-only
explicitly for a cached-source exploration; it is not fresh native evidence.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import gzip
import hashlib
import importlib
import inspect
import json
from pathlib import Path
import sys

from mechanism_lab.engineering_source import (load_project, assert_project_source_identity,
    verify_native_build, verify_native_source_binding)
from mechanism_lab.physical_model import digest

_WORKER = None
_RUNNER_SHA256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def _write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n', encoding='utf-8', newline='\n')


def _engine_identity():
    rows = {}
    for name in ('causal_runtime', 'actuation', 'physical_model', 'reference_clock'):
        module = importlib.import_module('mechanism_lab.'+name)
        rows[name+'.py'] = hashlib.sha256(Path(inspect.getfile(module)).read_bytes()).hexdigest()
    return rows


def _fault_module(project):
    # This command intentionally evaluates the ARX example's source requirements.
    # Its shared runtime accepts any EngineeringProject; these task thresholds
    # are not silently imposed on other mechanisms.
    return importlib.import_module(project.construct.__module__.rsplit('.', 1)[0]+'.faults')


def _initialize(snapshot):
    global _WORKER
    project, files = load_project(snapshot['project'])
    if (files != snapshot['source_files'] or _engine_identity() != snapshot['engine_files_sha256']
            or snapshot['runner_sha256'] != _RUNNER_SHA256):
        raise ValueError('Project or runtime changed before scenario worker initialization')
    # A project may bind its reference to the exact parameters returned by its
    # compile operation. Each spawned process must establish that binding, not
    # substitute a deserialized lookalike or remove the project's guard.
    physical, control, parameters = project.compile_physics(project.specification,
        native_directory=snapshot['native_directory'])
    physical.provenance['design_source_files'] = files
    if snapshot['native_build_sha256'] is not None:
        physical.provenance['native_build_sha256'] = snapshot['native_build_sha256']
        physical.provenance['native_source_binding_sha256'] = snapshot['native_source_binding_sha256']
    if (digest(physical.to_source()) != digest(snapshot['physical_source'])
            or digest(control) != digest(snapshot['control'])
            or digest(parameters) != digest(snapshot['task_parameters'])):
        raise ValueError('Worker compilation differs from the suite input snapshot')
    _WORKER = snapshot, project, _fault_module(project), parameters


def _run_one(case):
    snapshot, project, faults, parameters = _WORKER
    assert_project_source_identity(snapshot['project'], snapshot['source_files'])
    if _engine_identity() != snapshot['engine_files_sha256']:
        raise ValueError('Runtime changed during the scenario suite')
    result = faults.run_case(snapshot['physical_source'], snapshot['control'], parameters,
        lambda time: project.reference(time, parameters), case,
        reference_source=dict(source_files=snapshot['source_files'], task_parameters=parameters))
    imported = result.get('runtime_implementation', {}).get('source_files', {})
    if any(imported.get(name) != snapshot['engine_files_sha256'][name] for name in ('causal_runtime.py', 'actuation.py')):
        raise ValueError('Imported runtime implementation differs from the suite source snapshot')
    assert_project_source_identity(snapshot['project'], snapshot['source_files'])
    if _engine_identity() != snapshot['engine_files_sha256']:
        raise ValueError('Runtime changed during scenario integration')
    result['fault_suite_provenance'] = {key: snapshot[key] for key in
        ('source_files', 'engine_files_sha256', 'runner_sha256', 'native_build_sha256', 'native_source_binding_sha256', 'mass_scope')}
    directory = Path(snapshot['output'])/case['id']
    directory.mkdir(parents=True, exist_ok=False)
    trace = gzip.compress(json.dumps(result, separators=(',', ':'), allow_nan=False).encode(), mtime=0)
    (directory/'trace.json.gz').write_bytes(trace)
    summary = result['source_case_requirements']
    summary.update(trace_sha256=hashlib.sha256(trace).hexdigest(),
        fault_suite_provenance=result['fault_suite_provenance'],
        runtime_implementation=result.get('runtime_implementation'),
        peak_belt_Nm=[abs(result['metrics']['actuators'][f'motor{i+1}']['transmitted_torque_estimate']['peak_absolute']['value']) for i in range(6)])
    _write(directory/'checks.json', summary)
    return summary


def _friction_prefixes(output, rows):
    """Compare actual unperturbed states, not fabricated/replayed poses."""
    nominal = next((row for row in rows if row['case']['name'] == 'nominal'), None)
    controls = [row for row in rows if row['case']['name'] == 'friction_loss']
    if nominal is None or not controls:
        return []
    def read(row):
        raw = (Path(output)/row['case']['id']/'trace.json.gz').read_bytes()
        if hashlib.sha256(raw).hexdigest() != row['trace_sha256']:
            raise ValueError('A recorded scenario trace changed before prefix comparison')
        return json.loads(gzip.decompress(raw))
    normal = read(nominal)
    normal_frames = {frame['time_s']: frame for frame in normal['frames']}
    fields = ('qpos', 'qvel', 'motors', 'rack_motor', 'desired_q', 'bodies')
    comparisons = []
    for row in controls:
        adverse = read(row)
        cutoff = row['case']['options']['frictionChangeTime']
        frames = [frame for frame in adverse['frames'] if frame['time_s'] < cutoff-1e-9]
        same_identity = all(normal[key] == adverse[key] for key in ('model_sha256', 'control_sha256', 'reference_sha256', 'timestep_s'))
        missing = [frame['time_s'] for frame in frames if frame['time_s'] not in normal_frames]
        same = bool(frames) and not missing and same_identity
        maximum_state_difference = 0.
        for frame in frames:
            other = normal_frames.get(frame['time_s'])
            if other is None:
                continue
            same = same and all(frame[field] == other[field] for field in fields)
            maximum_state_difference = max(maximum_state_difference,
                max(abs(a-b) for a,b in zip(frame['qpos']+frame['qvel'], other['qpos']+other['qvel'])))
        comparisons.append(dict(case=row['case']['id'], nominal_case=nominal['case']['id'],
            before_time_s=cutoff, samples=len(frames), same_model_control_reference_and_timestep=same_identity,
            compared_fields=list(fields), missing_sample_times_s=missing,
            maximum_qpos_qvel_absolute_difference=maximum_state_difference, passed=bool(same),
            scope='Exact numeric equality of recorded states before the declared friction change; not an altered-state replay'))
    return comparisons


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', default='arx_belt')
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--native', type=Path)
    source.add_argument('--source-mass-only', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--case', action='append', help='Repeat to select source scenario names; default all 28')
    parser.add_argument('--clock', action='append', choices=['original-clock', 'matched-phase'])
    parser.add_argument('--workers', type=int, choices=range(1, 5), default=3)
    parser.add_argument('--timestep-scale', type=float, choices=(1., .5), default=1.,
        help='Explicit integration refinement; keeps physical parameters and thresholds unchanged')
    args = parser.parse_args()
    project, files = load_project(args.project)
    faults = _fault_module(project)
    if project.name != 'arx_belt':
        raise ValueError('The pinned source requirements in this command belong to arx_belt')
    receipt = verify_native_build(args.native, project.specification, source_files=files) if args.native else None
    binding = verify_native_source_binding(args.native, project.specification, files, receipt=receipt) if receipt else None
    physical, control, parameters = project.compile_physics(project.specification, native_directory=args.native)
    physical.provenance['design_source_files'] = files
    if receipt:
        physical.provenance['native_build_sha256'] = digest(receipt)
        physical.provenance['native_source_binding_sha256'] = digest(binding)
    cases = faults.scheduled_cases(parameters, args.case, clocks=args.clock or ('original-clock', 'matched-phase'))
    if args.timestep_scale != 1.:
        base_timestep = float(physical.root.find('option').get('timestep'))
        for case in cases:
            original_timestep = case.get('timestep_s', base_timestep)
            case.update(source_timestep_s=original_timestep, timestep_scale=args.timestep_scale,
                timestep_s=original_timestep*args.timestep_scale, id=case['id']+'--dt-'+format(args.timestep_scale, 'g'))
    snapshot = dict(project=args.project, source_files=files, physical_source=physical.to_source(),
        control=control, task_parameters=parameters, engine_files_sha256=_engine_identity(),
        native_build_sha256=digest(receipt) if receipt else None,
        native_source_binding_sha256=digest(binding) if binding else None,
        mass_scope='verified fresh native build' if receipt else 'explicit shipped-source mass exploration',
        source_scenarios=faults.source_scenarios(), cases=cases, output=str(args.output.resolve()), runner_sha256=_RUNNER_SHA256,
        native_directory=str(args.native.resolve()) if args.native else None)
    assert_project_source_identity(args.project, files)
    args.output.mkdir(parents=True, exist_ok=False)
    _write(args.output/'inputs.json', snapshot)
    rows, errors = [], []
    if args.workers == 1:
        _initialize(snapshot)
        for case in cases:
            row = _run_one(case)
            rows.append(row)
            print(json.dumps(dict(case=case['id'], passed=row['passed'], expected_outcome_observed=row['expected_outcome_observed'],
                failed=[item['name'] for item in row['checks']+row['task_outcome_checks'] if not item['passed']])), flush=True)
    else:
        with ProcessPoolExecutor(max_workers=args.workers, initializer=_initialize, initargs=(snapshot,)) as pool:
            jobs = {pool.submit(_run_one, case): case for case in cases}
            for future in as_completed(jobs):
                case = jobs[future]
                try:
                    row = future.result()
                    rows.append(row)
                    print(json.dumps(dict(case=case['id'], passed=row['passed'], expected_outcome_observed=row['expected_outcome_observed'],
                        failed=[item['name'] for item in row['checks']+row['task_outcome_checks'] if not item['passed']])), flush=True)
                except Exception as error:
                    errors.append(dict(case=case['id'], error=str(error)))
                    print(json.dumps(errors[-1]), flush=True)
                _write(args.output/'progress.json', dict(completed=len(rows), total=len(cases), errors=errors))
    assert_project_source_identity(args.project, files)
    if (_engine_identity() != snapshot['engine_files_sha256']
            or hashlib.sha256(Path(__file__).read_bytes()).hexdigest() != _RUNNER_SHA256):
        raise ValueError('Runtime changed during the suite; no final acceptance is published')
    if receipt:
        verified = verify_native_build(args.native, project.specification, source_files=files)
        if digest(verified) != digest(receipt) or digest(verify_native_source_binding(
                args.native, project.specification, files, receipt=verified)) != digest(binding):
            raise ValueError('Native source/artifacts changed during the suite')
    rows.sort(key=lambda row: row['case']['id'])
    prefixes = _friction_prefixes(args.output, rows)
    passed = (not errors and len(rows) == len(cases) and all(row['expected_outcome_observed'] for row in rows)
              and all(row['passed'] for row in prefixes))
    summary = dict(schema='cybrgeo-arx-source-fault-suite/1', complete=not errors and len(rows) == len(cases),
        passed=passed, requested_cases=[row['id'] for row in cases], results=rows, errors=errors,
        input_sha256=hashlib.sha256((args.output/'inputs.json').read_bytes()).hexdigest(),
        source_files=files, engine_files_sha256=snapshot['engine_files_sha256'], runner_sha256=_RUNNER_SHA256,
        native_build_sha256=snapshot['native_build_sha256'], native_source_binding_sha256=snapshot['native_source_binding_sha256'],
        mass_scope=snapshot['mass_scope'], friction_prefix_comparisons=prefixes,
        qualification='Source-derived bounded simulation cases; ordinary power-loss checks do not require payload retention; no hardware qualification')
    _write(args.output/'summary.json', summary)
    print(json.dumps(dict(complete=summary['complete'], passed=passed, scenarios=len(rows), errors=len(errors))), flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
