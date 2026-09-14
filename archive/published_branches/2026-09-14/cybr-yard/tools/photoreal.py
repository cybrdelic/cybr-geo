#!/usr/bin/env python3
"""Small explicit CLI for final-quality stills and films."""
from __future__ import annotations
import argparse,json,os
from pathlib import Path


def resolution(text):
    w,h=(int(x) for x in text.lower().split('x'));return w,h


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    sub=p.add_subparsers(dest='mode',required=True)
    s=sub.add_parser('still');s.add_argument('recipe');s.add_argument('--view',default='hero');s.add_argument('--size',type=resolution,default=(1920,1080));s.add_argument('--spp',type=int,default=512);s.add_argument('--depth',type=int,default=14);s.add_argument('--threads',type=int,default=4);s.add_argument('--f-stop',type=float);s.add_argument('--focus-distance',type=float);s.add_argument('--out',type=Path,required=True)
    f=sub.add_parser('film');f.add_argument('recipe');f.add_argument('--shots',type=Path,required=True);f.add_argument('--size',type=resolution,default=(1920,1080));f.add_argument('--fps',type=int,default=24);f.add_argument('--spp',type=int,default=144);f.add_argument('--depth',type=int,default=12);f.add_argument('--threads',type=int,default=4);f.add_argument('--shutter-angle',type=float,default=180.);f.add_argument('--shutter-samples',type=int,default=3);f.add_argument('--out',type=Path,required=True)
    a=p.parse_args();os.environ['MECHANISM_LAB_ROOT']=str(a.root.resolve())
    from mechanism_lab.registry import load
    assembly=load(a.recipe,rebuild=True)
    if a.mode=='still':
        from mechanism_lab.photoreal import render_photoreal
        r=render_photoreal(assembly,a.out,a.view,a.size,a.spp,a.threads,a.depth,'auto',False,0.,a.f_stop,a.focus_distance,True)
    else:
        from mechanism_lab.media import Shot
        from mechanism_lab.photoreal import render_photoreal_video
        shots=[Shot(**x) for x in json.loads(a.shots.read_text())]
        r=render_photoreal_video(assembly,a.out,shots,a.size,a.fps,a.spp,a.threads,a.depth,a.shutter_angle,a.shutter_samples)
    print(json.dumps(r,indent=2))


if __name__=='__main__':main()
