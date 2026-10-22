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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recipes', type=Path, help='Directory containing architecture.json recipes')
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    models = []
    for architecture in ('compact_arm', 'belt_arm', 'inspection_head'):
        plan = json.loads((args.recipes / (architecture + '.json')).read_text())
        graph = KinematicGraph([part['name'] for part in plan['parts']], plan['joints'])
        samples = []
        for time in range(9):
            values = graph.demo_values(time)
            samples.append({'time_seconds': time, 'values': values, 'transforms': {
                name: matrix.tolist() for name, matrix in graph.transforms(values).items()
            }})
        models.append({'architecture': architecture, 'recipe_sha256': plan['construction']['recipe_sha256'], 'samples': samples})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({'models': models}))


if __name__ == '__main__':
    main()
