"""Stage the ROAM recipes in an isolated output tree and run their CAD gates."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
MACHINES = ('table', 'drill', 'bend', 'vise', 'measure', 'press', 'electronics')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('example', choices=(*MACHINES, 'spine'))
    parser.add_argument('--out', type=Path, default=REPO / '.build/roam')
    parser.add_argument('--stage-only', action='store_true', help='Copy source without claiming a build or check')
    args = parser.parse_args()
    root = args.out.resolve()
    # Recipes retain their shared workspace-relative layout; nothing depends on
    # the original author directory or Windows virtual environment.
    source = HERE / ('spine' if args.example == 'spine' else 'workshop')
    target = root / 'outputs' / ('roam-spine' if args.example == 'spine' else 'roam-workshop/cadsrc')
    if root == HERE or HERE in root.parents:
        parser.error('Choose an output directory outside the example source')
    target.mkdir(parents=True, exist_ok=True)
    for path in source.iterdir():
        if path.is_file():
            shutil.copy2(path, target / path.name)
    if args.stage_only:
        print(target)
        return
    env = dict(os.environ, PYTHONUTF8='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    env['PYTHONPATH'] = str(REPO / 'src') + os.pathsep + env.get('PYTHONPATH', '')
    commands = ([['build_checked.py'], ['engineering.py']] if args.example == 'spine'
                else [[args.example + '.py'], ['build_checked.py', args.example]])
    for command in commands:
        subprocess.run([sys.executable, str(target / command[0]), *command[1:]],
                       cwd=root, env=env, check=True)
    print('CAD gates completed. Physical operation and fabrication remain unqualified.')

if __name__ == '__main__':
    main()
