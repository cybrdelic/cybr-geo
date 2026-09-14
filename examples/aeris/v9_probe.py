"""Isolate AERIS geometry that Mitsuba refuses to load.

Each invocation loads only one requested AERIS group or part into the generic
V9 scene at tiny resolution. Run each probe in a separate process: a native
Mitsuba/Dr.Jit segfault then identifies the offending subset without killing the
parent diagnostic loop.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src'))
sys.path.insert(0,str(Path(__file__).resolve().parent))
os.environ.setdefault('MECHANISM_LAB_ROOT',str(ROOT))

from dataclasses import replace
from recipe import coupled_pose
from mechanism_lab.core import load_cache
from mechanism_lab.photoreal import prepare_view_geometry,resolve_studio
from mechanism_lab.v9 import ensure_assets
from mechanism_lab.v9_dispatch import _large_scene_dict,_scalar_mitsuba


def load(out:Path):
    assembly=load_cache(out/'cache')
    assembly.motion_function=coupled_pose
    return assembly


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--out',type=Path,default=ROOT/'build/aeris')
    selection=parser.add_mutually_exclusive_group()
    selection.add_argument('--group')
    selection.add_argument('--part')
    parser.add_argument('--list-groups',action='store_true')
    parser.add_argument('--list-parts',action='store_true')
    parser.add_argument('--render',action='store_true',help='Also trace one sample after scene construction')
    args=parser.parse_args()

    assembly=load(args.out)
    if args.list_groups:
        for name in sorted({p.group for p in assembly.parts}):print(name)
        return
    if args.list_parts:
        for part in assembly.parts:print(part.name)
        return

    if args.group is not None:
        parts=[p for p in assembly.parts if p.group==args.group]
        label=f'group:{args.group}'
    elif args.part is not None:
        parts=[p for p in assembly.parts if p.name==args.part]
        label=f'part:{args.part}'
    else:
        parts=list(assembly.parts);label='all'
    if not parts:raise SystemExit(f'No geometry selected for {label}')
    assembly=replace(assembly,name='aeris_v9_probe',parts=parts)

    view=resolve_studio(assembly,assembly.views['hero'])
    subset,view=prepare_view_geometry(assembly,view)
    assets=ensure_assets()
    mi,variant=_scalar_mitsuba()
    probe_dir=args.out/'v9_probe'/label.replace(':','_').replace('/','_')
    probe_dir.mkdir(parents=True,exist_ok=True)
    scene,triangles,camera,shapes,mode=_large_scene_dict(
        mi,subset,view,(96,72),1,2,assets,probe_dir,
    )
    print('PROBE_PRELOAD',label,'parts',len(parts),'triangles',triangles,'shapes',shapes,'variant',variant,flush=True)
    loaded=mi.load_dict(scene)
    print('PROBE_LOADED',label,flush=True)
    if args.render:
        image=mi.render(loaded,spp=1)
        print('PROBE_RENDERED',label,'shape',getattr(image,'shape',None),flush=True)


if __name__=='__main__':main()
