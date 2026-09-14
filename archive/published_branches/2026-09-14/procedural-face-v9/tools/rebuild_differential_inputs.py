"""Rebuild differential inputs from retained source when original arrays are absent.

Existing arrays are never overwritten unless --force is explicit. Recreated
geometry is marked regenerated; this does not assert binary identity with the
historical delivered archives, whose dependency versions may differ.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--force', action='store_true')
    args = parser.parse_args()
    target = ROOT / 'assets/differential_v3/geometry'
    target.mkdir(parents=True, exist_ok=True)
    names = ('reference_parts', 'kinematic_core_parts', 'working_variant_parts')
    complete = all((target / (name + suffix)).is_file()
                   for name in names for suffix in ('.npz', '.json'))
    if complete and not args.force:
        print('Existing differential arrays retained. Use --force to regenerate.')
        return
    if any(target.glob('*.npz')) and not args.force:
        raise RuntimeError('Partial existing inputs: back them up before using --force.')
    base = ROOT / 'archive/reference_v2'
    subprocess.run([sys.executable, str(base / 'src/build_geometry.py')], check=True, cwd=ROOT)
    geom = ROOT / 'examples/differential/geometry'
    geom.mkdir(parents=True, exist_ok=True)
    shutil.copy2(base / 'geometry/mesh_arrays.npz', geom / 'reference_mesh_arrays.npz')
    shutil.copy2(base / 'geometry/manifest.json', geom / 'reference_manifest.json')
    subprocess.run([sys.executable, str(ROOT / 'examples/differential/src/model.py')], check=True, cwd=ROOT)
    for name in names:
        for suffix in ('.npz', '.json'):
            shutil.copy2(geom / (name + suffix), target / (name + suffix))
    shutil.copy2(geom / 'reference_manifest.json', target / 'reference_manifest.json')
    receipt = {
        'origin': 'Regenerated from retained procedural source; not a byte-identical archive restoration.',
        'files': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in sorted(target.iterdir()) if p.suffix in ('.json', '.npz')},
    }
    (target / 'regeneration_receipt.json').write_text(json.dumps(receipt, indent=2))
    print('Differential inputs rebuilt for both toolkit APIs:', target)

if __name__ == '__main__':
    main()
