"""Emit native solver expectations for the separate JavaScript rig acceptance.

Run with CYBR GEO installed, or from this checkout. No CAD is regenerated here:
the inputs are the saved recipes which produced the WASM artifact assemblies.
"""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src'))
from mechanism_lab.constructed import KinematicGraph
from mechanism_lab.motion import MotionTask


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recipes', type=Path, help='Directory containing architecture.json recipes')
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    models = []
    for path in sorted(args.recipes.glob('*.json')):
        plan = json.loads(path.read_text(encoding='utf-8'))
        if not isinstance(plan, dict) or not plan.get('parts') or not plan.get('construction'):
            continue
        architecture = path.stem
        graph = KinematicGraph([part['name'] for part in plan['parts']], plan['joints'])
        motion = MotionTask.from_dict(plan['construction']['motion']) if plan['construction'].get('motion') else MotionTask.from_joints(plan['joints'])
        samples = []
        for index in range(9):
            time = index * motion.period_s / 8
            command = motion.sample(time)
            samples.append({'time_seconds': time, **command, 'transforms': {
                name: matrix.tolist() for name, matrix in graph.transforms(command['values']).items()
            }})
        models.append({'architecture': architecture, **{key: plan['construction'].get(key) for key in ('recipe_sha256', 'mechanism_sha256', 'model_sha256')}, 'motion': motion.to_dict(), 'peak_rates': motion.peak_rates(), 'samples': samples})
    if not models:
        raise ValueError('No compiled construction recipes found')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({'models': models}))


if __name__ == '__main__':
    main()
