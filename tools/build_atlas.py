#!/usr/bin/env python3
"""Reproduce ATLAS geometry, verification, exports, whiteprint, and photographs.

Run from the repository with PYTHONPATH=src. No downloaded models or generated
images are needed. All render jobs use mechanism_lab.photoreal.
"""
from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path

from mechanism_lab.core import validate
from mechanism_lab.exporters import (
    export_animated_glb, export_bom, export_glb, export_step, export_stls,
)
from mechanism_lab.registry import load


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['build','stills','film','drawing','all'])
    parser.add_argument('--out',type=Path,default=Path('outputs/atlas_fixture'))
    parser.add_argument('--size',default='1800x1200')
    parser.add_argument('--spp',type=int,default=512)
    parser.add_argument('--threads',type=int,default=8)
    parser.add_argument('--views',nargs='+',default=['hero','cutaway','exploded','materials_macro'])
    parser.add_argument('--fps',type=int,default=24)
    args=parser.parse_args()
    args.out.mkdir(parents=True,exist_ok=True)
    analytic=args.action in ('build','drawing','all')
    assembly=load('atlas_fixture',analytic=analytic)
    if args.action in ('build','all'):
        result=validate(assembly,expensive=True)
        (args.out/'validation.json').write_text(json.dumps(result,indent=2)+'\n')
        export_glb(assembly,args.out/'atlas_fixture.glb')
        export_step(assembly,args.out/'atlas_fixture_analytic.step',individual=False)
        export_stls(assembly,args.out/'stl_parts')
        export_bom(assembly,args.out)
        export_animated_glb(assembly,args.out/'atlas_fixture_motion.glb',duration=8,fps=24)
        export_animated_glb(assembly,args.out/'atlas_fixture_exploded.glb',duration=8,fps=24,mode='explode')
        (args.out/'capabilities.json').write_text(json.dumps({
            'techniques':assembly.metadata['geometry_techniques'],
            'parts':[{ 'name':p.name,'techniques':list(p.tags),'role':p.role,
                       'analytic_cad':p.cad is not None} for p in assembly.parts],
        },indent=2)+'\n')
        print('Geometry and portable exports complete',flush=True)
    if args.action in ('drawing','all'):
        from mechanism_lab.whiteprint import whiteprint
        names=['AT_001_Fixture_base','AT_020_Freeform_housing','AT_060_Precision_guide_face']
        names += [f'AT_071_Freeform_jaw_{i+1}' for i in range(3)]
        whiteprint(assembly,args.out/'atlas_whiteprint',part_names=names,
                   title='ATLAS / NOMINAL INTERFACE STUDY',scale=.5)
        print('Analytic drawing complete',flush=True)
    if args.action in ('stills','all'):
        from mechanism_lab.photoreal import render_photoreal
        size=tuple(map(int,args.size.split('x')))
        for view in args.views:
            print('Rendering',view,flush=True)
            result=render_photoreal(assembly,args.out/f'atlas_{view}.png',view,size=size,
                                    spp=args.spp,threads=args.threads,depth=14)
            print(view,result['seconds'],'seconds',flush=True)
    if args.action in ('film','all'):
        from mechanism_lab.media import Shot,make_gif
        from mechanism_lab.photoreal import render_photoreal_video
        size=tuple(map(int,args.size.split('x')))
        shots=[Shot('hero',8,'orbit',18),Shot('cutaway',8,'motion'),Shot('exploded',8,'explode')]
        render_photoreal_video(assembly,args.out/'atlas_film.mp4',shots,size=size,
                               fps=args.fps,spp=args.spp,threads=args.threads,depth=12,
                               shutter_samples=3,shutter_angle=180)
        make_gif(args.out/'atlas_film.mp4',args.out/'atlas_motion.gif')


if __name__=='__main__':
    main()
