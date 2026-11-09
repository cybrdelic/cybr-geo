#!/usr/bin/env python3
"""Render verified planetary-actuator films from the built-in recipe.

`preview` uses the fast PBR renderer for shot development. `proof` and `final`
use the thin-lens path tracer; every encoded frame is freshly rendered geometry.
"""
from __future__ import annotations
import argparse
from pathlib import Path

from mechanism_lab.registry import load
from mechanism_lab.media import Shot,render_video
from mechanism_lab.photoreal import render_photoreal_video


def shots(preset):
    if preset=='proof':
        return [
            Shot('hero',1.25,'orbit',orbit_degrees=16,title='PLANETARY ACTUATOR / INTERNAL HERO'),
            Shot('kinematics',1.50,'motion',orbit_degrees=0,title='EXACT 5:1 PLANETARY KINEMATICS'),
        ]
    return [
        Shot('exterior',3.0,'orbit',orbit_degrees=18,title='COMPACT PLANETARY ACTUATOR'),
        Shot('exploded',3.0,'explode',orbit_degrees=0,title='ASSEMBLY / INSPECTION REVEAL'),
        Shot('kinematics',4.0,'motion',orbit_degrees=0,title='18T SUN / 27T PLANETS / 72T FIXED RING'),
        Shot('gear_macro',3.0,'motion',orbit_degrees=0,title='INVOLUTE MESH / MACRO'),
        Shot('encoder_macro',2.5,'still',orbit_degrees=0,title='INPUT SUPPORT / ENCODER INTERFACE'),
        Shot('hero',3.0,'orbit',orbit_degrees=14,title='5:1 CARRIER OUTPUT'),
    ]


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--preset',choices=['preview','proof','final'],default='proof')
    p.add_argument('--out',type=Path)
    p.add_argument('--threads',type=int,default=4)
    args=p.parse_args()
    assembly=load('planetary_actuator',rebuild=True)
    if args.preset=='preview':
        out=args.out or Path('outputs/planetary_actuator/videos/planetary_preview.mp4')
        render_video(assembly,out,shots('final'),size=(1280,720),fps=24)
    elif args.preset=='proof':
        out=args.out or Path('outputs/planetary_actuator/films/planetary_photoreal_proof.mp4')
        render_photoreal_video(
            assembly,out,shots('proof'),size=(960,540),fps=24,spp=48,threads=args.threads,depth=12,
            shutter_angle=180,shutter_samples=3,intent='concept',allow_estimates=False,
        )
    else:
        out=args.out or Path('outputs/planetary_actuator/films/planetary_photoreal_final.mp4')
        render_photoreal_video(
            assembly,out,shots('final'),size=(1920,1080),fps=24,spp=192,threads=args.threads,depth=14,
            shutter_angle=180,shutter_samples=3,intent='concept',allow_estimates=False,
        )
    print(out)

if __name__=='__main__':
    main()
