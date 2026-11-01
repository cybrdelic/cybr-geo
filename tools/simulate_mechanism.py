"""Run the shared native geometry -> MuJoCo backend and save integrated motion.

Example: PYTHONPATH=src python tools/simulate_mechanism.py examples/custom.json --out outputs/custom-dynamics
The positional input accepts a canonical mechanism JSON or a named construction
example, all compiled through the same mechanical source pipeline.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from mechanism_lab.dynamics import plot_simulation, simulate, simulated_assembly
from mechanism_lab.exporters import export_animated_glb
from mechanism_lab.constructed import build_from_plan
from mechanism_lab.engineering_pipeline import compile_source


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mechanism', nargs='?', default='compact_arm')
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=8)
    parser.add_argument('--timestep', type=float, default=.002)
    parser.add_argument('--effort-scale', type=float, default=1)
    args = parser.parse_args()
    plan, name = compile_source(args.mechanism)
    assembly = build_from_plan(plan)
    assembly.name = name
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / 'recipe.json').write_text(json.dumps(plan, indent=2) + '\n', encoding='utf-8')
    (args.out / 'mechanism.json').write_text(json.dumps(plan['construction']['mechanism'], indent=2) + '\n', encoding='utf-8')
    result = simulate(assembly, seconds=args.seconds, timestep=args.timestep,
                      effort_scale=args.effort_scale, output=args.out)
    plot_simulation(result, args.out / 'tracking.svg')
    export_animated_glb(simulated_assembly(assembly, result), args.out / 'simulated-motion.glb',
                        duration=result['seconds'], fps=20)
    summary = {key: value for key, value in result.items() if key not in ('samples', 'model', 'integration_state_trace')}
    print(json.dumps(summary, indent=2))
    return 0 if result['tracking_passed'] and result['operating_limits_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
