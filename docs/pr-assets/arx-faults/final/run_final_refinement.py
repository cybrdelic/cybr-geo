"""Portable reproduction launcher, not the wrapper used for archived results.

Use a Python environment with the repository's simulation dependencies installed.
This launches the current checkout against an existing source-bound native build;
the runner rejects stale source/artifact bindings. It does not claim a rerun of
the historical receipt merely because the same scenario selection is used.
"""
import argparse
import json
import os
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[4])
    parser.add_argument('--native', type=Path, default=Path('outputs/arx_belt/native'))
    parser.add_argument('--output', type=Path, default=Path('outputs/arx-faults-halfstep'))
    parser.add_argument('--workers', type=int, choices=range(1, 5), default=3)
    parser.add_argument('--dry-run', action='store_true', help='Show the invocation without starting a simulation')
    args = parser.parse_args()
    repo = args.repo.resolve()
    runner = repo / 'tools/check_arx_faults.py'
    if not runner.is_file():
        parser.error('--repo must identify a CYBR GEO checkout containing tools/check_arx_faults.py')
    native = (repo / args.native).resolve()
    output = (repo / args.output).resolve()
    if not args.dry_run:
        if not (native / 'native-build.json').is_file() or not (native / 'native-source-binding.json').is_file():
            parser.error('--native must contain a completed source-bound native build')
        if output.exists():
            parser.error('--output must be a new directory')
    command = [sys.executable, '-B', str(runner), '--native', str(native),
        '--output', str(output), '--workers', str(args.workers), '--clock', 'matched-phase']
    for case in ('nominal', 'power_loss_3.2', 'power_loss_7.2',
            'weak_spring_double_delay_3.2', 'weak_spring_double_delay_7.2'):
        command.extend(('--case', case))
    command.extend(('--timestep-scale', '.5'))
    overrides = {'PYTHONDONTWRITEBYTECODE': '1', 'OPENBLAS_NUM_THREADS': '1',
        'OMP_NUM_THREADS': '2', 'LP_NUM_THREADS': '2'}
    environment = os.environ.copy()
    environment.update(overrides)
    environment['PYTHONPATH'] = str(repo / 'src') + (
        os.pathsep + environment['PYTHONPATH'] if environment.get('PYTHONPATH') else '')
    if args.dry_run:
        print(json.dumps(dict(command=command, cwd=str(repo), explicit_environment=overrides,
            pythonpath_policy='Prepend checkout/src with os.pathsep; preserve any inherited PYTHONPATH.',
            scope='Invocation preview only; no simulation executed and no historical receipt replaced.'), indent=2))
        return
    os.chdir(repo)
    os.execvpe(sys.executable, command, environment)


if __name__ == '__main__':
    main()
