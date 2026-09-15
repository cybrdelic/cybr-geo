"""Nominal static audit of all workshop recipes; does not replace checked builds."""
import importlib
import json
import os
from pathlib import Path
import sys
import traceback

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE / 'workshop'), str(HERE.parents[1] / 'src')]
from build_checked import audit

def main():
    for name in ('table', 'drill', 'bend', 'vise', 'measure', 'press', 'electronics'):
        machine = importlib.import_module(name).build()
        parts = {n: p['shape'] for n, p in machine.parts.items()}
        assert all(p.isValid() and len(p.Solids()) == 1 for p in parts.values())
        threads = {frozenset((t['a'], t['b'])): t for t in machine.threads}
        report = audit(parts, machine.parts, machine.dofs, threads, {})
        assert not report['problems'], report['problems']
        for connection in machine.connections:
            assert parts[connection['a']].distance(parts[connection['b']]) <= connection['max_gap_mm'] + .001
        print(json.dumps({'machine': name, 'parts': len(parts), 'pairs': report['pairs'],
                          'exact': report['exact'], 'static_passed': True}), flush=True)

if __name__ == '__main__':
    try:
        main()
    except BaseException:
        traceback.print_exc()
        sys.stderr.flush()
        os._exit(1)
    sys.stdout.flush()
    os._exit(0)
