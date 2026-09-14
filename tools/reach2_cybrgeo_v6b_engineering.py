"""Engineering gate wrapper for authoritative native CYBR GEO v6b recipe."""
from __future__ import annotations
import argparse, importlib.util, json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools'/'reach2_cybrgeo_v6_engineering.py'
spec=importlib.util.spec_from_file_location('v6_engineering_base',BASE)
_base=importlib.util.module_from_spec(spec);assert spec.loader is not None;spec.loader.exec_module(_base)
_base.RECIPE=ROOT/'examples'/'reach2_cybrgeo_v6b.py'
qualify=_base.qualify

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path);ap.add_argument('--require-pass',action='store_true');args=ap.parse_args();r=qualify();text=json.dumps(r,indent=2);print(text)
    if args.out:args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(text+'\n')
    if args.require_pass and not r['qualified']:raise SystemExit('failed: '+', '.join(c['name'] for c in r['checks'] if not c['passed']))
if __name__=='__main__':main()
