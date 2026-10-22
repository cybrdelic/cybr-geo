"""Run the actual ARX example through GEO's reusable causal runtime.

Without --native this explicitly uses the shipped source mass records. With
--native it verifies regenerated native artifacts before using their inertias.
Friction loss changes declared contact friction at source phase 2 s, mapped
through the project's execution clock; it never moves the puck.
"""
import argparse
import gzip
import json
from pathlib import Path

from mechanism_lab.causal_runtime import CausalSimulation
from mechanism_lab.engineering_source import (load_project, verify_native_build,
    verify_native_source_binding, assert_project_source_identity)
from mechanism_lab.physical_model import digest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', default='arx_belt')
    parser.add_argument('--native', type=Path)
    parser.add_argument('--case', choices=('nominal', 'friction_loss'), default='nominal')
    parser.add_argument('--seconds', type=float, help='Defaults to the complete authored project cycle')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    project, source_files = load_project(args.project)
    seconds = project.duration_s if args.seconds is None else args.seconds
    receipt = verify_native_build(args.native, project.specification, source_files=source_files) if args.native else None
    binding = verify_native_source_binding(args.native, project.specification, source_files, receipt=receipt) if receipt is not None else None
    physical, control, parameters = project.compile_physics(project.specification, native_directory=args.native)
    physical.provenance['design_source_files'] = source_files
    if receipt is not None:
        physical.provenance['native_build_sha256'] = digest(receipt)
        physical.provenance['native_source_binding_sha256'] = digest(binding)
    physical.write(args.output)
    (args.output/'control-source.json').write_text(json.dumps(control, indent=2), encoding='utf-8')
    from mechanism_lab.reference_clock import PhaseClock
    clock = PhaseClock.from_source(parameters['reference_clock'])
    options = dict(frictionChangeTime=clock.physical_time(2.), frictionAfterScale=.01) if args.case == 'friction_loss' else {}
    simulation = CausalSimulation(physical, control, lambda t: project.reference(t, parameters), options=options,
        reference_source=dict(source_files=source_files, task_parameters=parameters))
    print(f'Integrating {args.case}: {seconds:g}s, {simulation.model.nbody} bodies, {simulation.model.ngeom} contact/display geoms', flush=True)
    result = simulation.run(seconds, output=args.output)
    result['physical_source'], result['task_parameters'] = physical.to_source(), parameters
    result['requirements'] = project.requirements(result, parameters)
    assert_project_source_identity(args.project, source_files)
    if receipt is not None:
        verified = verify_native_build(args.native, project.specification, source_files=source_files)
        if digest(verified) != digest(receipt) or digest(verify_native_source_binding(
                args.native, project.specification, source_files, receipt=verified)) != digest(binding):
            raise ValueError('Native construction identity changed during causal simulation')
    (args.output/'trace.json.gz').write_bytes(gzip.compress(json.dumps(result, separators=(',', ':'), allow_nan=False).encode(), mtime=0))
    summary = {key: value for key, value in result.items() if key not in ('frames', 'physical_source', 'task_parameters')}
    summary['frames'] = len(result['frames'])
    (args.output/'simulation.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print(json.dumps({'output': str(args.output), 'requirements': result['requirements'], 'metrics': result['metrics']}, indent=2), flush=True)


if __name__ == '__main__':
    main()
