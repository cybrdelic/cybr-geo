"""Run the actual ARX example through GEO's reusable causal runtime.

Without --native this explicitly uses the shipped source mass records. With
--native it verifies regenerated native artifacts before using their inertias.
Friction loss changes declared contact friction at 2 s; it never moves the puck.
"""
import argparse
import gzip
import json
from pathlib import Path

from mechanism_lab.causal_runtime import CausalSimulation
from mechanism_lab.engineering_source import load_project, verify_native_build
from mechanism_lab.physical_model import digest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', default='arx_belt')
    parser.add_argument('--native', type=Path)
    parser.add_argument('--case', choices=('nominal', 'friction_loss'), default='nominal')
    parser.add_argument('--seconds', type=float, default=10.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    project, source_files = load_project(args.project)
    receipt = verify_native_build(args.native, project.specification, recipe_sha256=source_files.get('design.py')) if args.native else None
    physical, control, parameters = project.compile_physics(project.specification, native_directory=args.native)
    physical.provenance['design_source_files'] = source_files
    if receipt is not None:
        physical.provenance['native_build_sha256'] = digest(receipt)
    physical.write(args.output)
    (args.output/'control-source.json').write_text(json.dumps(control, indent=2), encoding='utf-8')
    options = dict(frictionChangeTime=2., frictionAfterScale=.01) if args.case == 'friction_loss' else {}
    simulation = CausalSimulation(physical, control, lambda t: project.reference(t, parameters), options=options,
        reference_source=dict(source_files=source_files, task_parameters=parameters))
    print(f'Integrating {args.case}: {args.seconds:g}s, {simulation.model.nbody} bodies, {simulation.model.ngeom} contact/display geoms', flush=True)
    result = simulation.run(args.seconds, output=args.output)
    result['physical_source'], result['task_parameters'] = physical.to_source(), parameters
    result['requirements'] = project.requirements(result, parameters)
    (args.output/'trace.json.gz').write_bytes(gzip.compress(json.dumps(result, separators=(',', ':'), allow_nan=False).encode(), mtime=0))
    summary = {key: value for key, value in result.items() if key not in ('frames', 'physical_source', 'task_parameters')}
    summary['frames'] = len(result['frames'])
    (args.output/'simulation.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print(json.dumps({'output': str(args.output), 'requirements': result['requirements'], 'metrics': result['metrics']}, indent=2), flush=True)


if __name__ == '__main__':
    main()
