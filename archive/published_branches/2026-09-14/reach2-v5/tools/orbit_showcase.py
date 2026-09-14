"""Reproduce the ORBIT model, measured geometry checks and native rendered media."""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path

import numpy as np

from mechanism_lab.core import pose_cad, validate
from mechanism_lab.exporters import export_animated_glb, export_bom, export_glb, export_step
from mechanism_lab.registry import load

ROOT=Path(__file__).resolve().parents[1]
RECIPE=ROOT/'examples/orbit_inspection_wrist.py'


def check_interfaces(assembly):
    """Measure actual BRep intersections, in addition to mesh/transform checks.

    This is a selected interface audit, explicitly not whole-device collision
    certification.  Returning sampled volumes preserves a reproducible record.
    """
    lookup={p.name:p for p in assembly.parts}
    pairs=[
        ('G01_72T_Involute_wrist_gear','G02_20T_Involute_input_pinion'),
        ('L02_Opposed_helical_lead_screw','J03_1_Helically_cut_bronze_nut'),
        ('L02_Opposed_helical_lead_screw','J03_-1_Helically_cut_bronze_nut'),
        ('J04_1_Lofted_windowed_finger','P01_Hollow_sculpted_palm'),
        ('J04_-1_Lofted_windowed_finger','P01_Hollow_sculpted_palm'),
        ('J04_1_Lofted_windowed_finger','L01_0_Ground_guide_rod'),
        ('J04_1_Lofted_windowed_finger','L01_1_Ground_guide_rod'),
        ('J04_1_Lofted_windowed_finger','J06_1_Grooved_elastomer_pad'),
        ('R01_Rear_Outer_race','R01_Rear_Ball_00'),
        ('R01_Rear_Inner_race','R01_Rear_Ball_00'),
        ('W01_Revolved_hollow_shaft_and_flange','G01_72T_Involute_wrist_gear'),
        ('H01_Removable_gear_shell','G01_72T_Involute_wrist_gear'),
    ]
    rows=[]
    for t in (0., 1., 2., 4.):
        for an,bn in pairs:
            # Stationary-to-stationary fits need only one sample.
            if t and (an.startswith('R01') or bn.endswith('elastomer_pad')):
                continue
            a,b=lookup[an],lookup[bn]
            a=pose_cad(a.cad,assembly.pose(a,t,0))
            b=pose_cad(b.cad,assembly.pose(b,t,0))
            common=a.intersect(b)
            volume=sum(abs(s.Volume()) for s in common.Solids())
            rows.append({'time_seconds':t,'parts':[an,bn],'overlap_mm3':volume})
            print('INTERFACE',t,an,bn,round(volume,8),flush=True)
    limit=1e-4
    return {'method':'OpenCascade BRep intersection volume at four prescribed poses',
            'sampled_pairs':len(pairs),'volume_tolerance_mm3':limit,
            'passed':all(row['overlap_mm3']<=limit for row in rows),'measurements':rows,
            'scope':'Selected listed interfaces only; no load, friction or global swept-volume certification.'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['build','render','film','drawing'])
    parser.add_argument('--out',type=Path,default=ROOT/'outputs/orbit_showcase')
    parser.add_argument('--views',nargs='+',default=['hero','internal','rear','exploded','gripper'])
    parser.add_argument('--size',default='1920x1440')
    parser.add_argument('--spp',type=int,default=768)
    parser.add_argument('--threads',type=int,default=8)
    parser.add_argument('--interfaces',action='store_true')
    parser.add_argument('--drawing',action='store_true')
    args=parser.parse_args();out=args.out;out.mkdir(parents=True,exist_ok=True)
    a=load(str(RECIPE),analytic=args.mode in ('build','drawing'))
    if args.mode=='build':
        report=validate(a,expensive=True)
        (out/'geometry-validation.json').write_text(json.dumps(report,indent=2)+'\n')
        export_step(a,out/'CYBR_ORBIT_analytic.step',individual=False)
        export_glb(a,out/'CYBR_ORBIT.glb')
        export_glb(a,out/'CYBR_ORBIT_exploded.glb',explode=1)
        export_animated_glb(a,out/'CYBR_ORBIT_motion.glb',duration=8,fps=30)
        export_bom(a,out/'model')
        (out/'capabilities.json').write_text(json.dumps(a.metadata,indent=2)+'\n')
        if args.interfaces:
            checks=check_interfaces(a)
            (out/'interface-validation.json').write_text(json.dumps(checks,indent=2)+'\n')
            if not checks['passed']:
                raise RuntimeError('Selected interface collision check failed; inspect measurements.')
        if args.drawing:
            from mechanism_lab.whiteprint import whiteprint
            names=['B01_Mounting_shoe','B03_Rear_bearing_pedestal','H02_1_Split_front_bearing_cover','H02_-1_Split_front_bearing_cover',
                   'P01_Hollow_sculpted_palm','J04_1_Lofted_windowed_finger','J04_-1_Lofted_windowed_finger']
            whiteprint(a,out/'ORBIT_nominal_whiteprint',part_names=names,title='ORBIT / NOMINAL STRUCTURAL ENVELOPE')
    elif args.mode=='render':
        from mechanism_lab.photoreal import render_photoreal
        size=tuple(map(int,args.size.split('x')))
        for view in args.views:
            print('RENDER',view,flush=True)
            r=render_photoreal(a,out/f'ORBIT_{view}.png',view,size,args.spp,args.threads,14)
            print('DONE',view,r['seconds'],flush=True)
    elif args.mode=='film':
        from mechanism_lab.photoreal import render_photoreal_video
        from mechanism_lab.media import Shot
        size=tuple(map(int,args.size.split('x')))
        shots=[Shot('internal',4.,'motion')]
        r=render_photoreal_video(a,out/'ORBIT_pathtraced_motion.mp4',shots,size,24,args.spp,args.threads,10,
                                 shutter_angle=180,shutter_samples=3)
        print(json.dumps(r,indent=2))
    elif args.mode=='drawing':
        from mechanism_lab.whiteprint import whiteprint
        # Keep the hidden-line drawing legible: an explicit analytic structural
        # subset, with threaded internals available in the full STEP assembly.
        names=['B01_Mounting_shoe','B03_Rear_bearing_pedestal','H02_1_Split_front_bearing_cover','H02_-1_Split_front_bearing_cover',
               'P01_Hollow_sculpted_palm','J04_1_Lofted_windowed_finger','J04_-1_Lofted_windowed_finger']
        whiteprint(a,out/'ORBIT_nominal_whiteprint',part_names=names,title='ORBIT / NOMINAL STRUCTURAL ENVELOPE')


if __name__=='__main__':
    main()
