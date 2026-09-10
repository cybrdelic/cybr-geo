#!/usr/bin/env python3
"""Render each real mechanism subassembly in isolation, then the full assembly.

Uses the same VTK/EGL PBR renderer and antialiasing modes as `lab video`.
No generated imagery or still-image panning is involved.
"""
from __future__ import annotations
import argparse, hashlib, json, math, subprocess, time
from pathlib import Path
import numpy as np

from mechanism_lab.registry import load
from mechanism_lab.render import Studio, labelled, antialias_description
from mechanism_lab.media import probe


def resolution(text):
    w,h=(int(x) for x in text.lower().split('x'));return w,h


def bounds_for(assembly, parts):
    points=[]
    for p in parts:
        T=assembly.pose(p,0,0)
        points.append(p.vertices@T[:3,:3].T+T[:3,3])
    q=np.concatenate(points)
    return q.min(0),q.max(0)


def render(recipe, output, size=(1280,720), fps=18, seconds=10.0, aa='ssaa2'):
    assembly=load(recipe,rebuild=True)
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    partial=output.with_name(output.stem+'.partial.mp4')
    command=['ffmpeg','-y','-v','error','-threads','2','-f','rawvideo','-pix_fmt','rgb24','-s',f'{size[0]}x{size[1]}','-r',str(fps),'-i','-',
             '-an','-c:v','libx264','-threads','2','-preset','medium','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(partial)]
    proc=subprocess.Popen(command,stdin=subprocess.PIPE)
    groups=[]
    for p in assembly.parts:
        if p.group not in groups:groups.append(p.group)
    hero=assembly.views['hero']; aa_text=antialias_description(aa)
    inspect_seconds=seconds*.78;final_seconds=seconds-inspect_seconds
    per=max(.45,inspect_seconds/max(1,len(groups)))
    rows=[];frame=0;start=time.time()
    studio=Studio(assembly,size=size,aa=aa)
    try:
        for gi,group in enumerate(groups):
            selected=[p for p in assembly.parts if p.group==group]
            lo,hi=bounds_for(assembly,selected);target=(lo+hi)/2
            scale=max(7.0,float(np.linalg.norm(hi-lo))*.62)
            n=max(1,round(per*fps))
            studio.visible(lambda p,g=group:p.group==g)
            studio.pose(0,0)
            for f in range(n):
                u=f/max(1,n-1);az=35+44*(u-.5)
                studio.set_camera(az,26,scale,target)
                im=studio.render();digest=hashlib.sha256(im.tobytes()).hexdigest()
                footer=f'SUBASSEMBLY {gi+1}/{len(groups)}  |  {group.upper()}  |  {aa_text}  |  actual geometry'
                im=labelled(im,assembly.name.upper()+' / '+group.upper(),f'{len(selected)} named components',footer)
                proc.stdin.write(np.ascontiguousarray(im,dtype=np.uint8).tobytes())
                rows.append(dict(frame=frame,phase='subassembly',group=group,pixel_sha256=digest));frame+=1
        studio.visible(lambda p:p.group not in hero.hide)
        n=max(1,round(final_seconds*fps))
        for f in range(n):
            u=f/max(1,n-1);az=hero.az+52*(u-.5)
            studio.pose(f/fps,0)
            studio.set_camera(az,hero.el,hero.scale,hero.target)
            im=studio.render();digest=hashlib.sha256(im.tobytes()).hexdigest()
            im=labelled(im,hero.title or assembly.name.upper(),hero.note,f'COMPLETE ASSEMBLY  |  {aa_text}  |  actual geometry')
            proc.stdin.write(np.ascontiguousarray(im,dtype=np.uint8).tobytes())
            rows.append(dict(frame=frame,phase='complete',pixel_sha256=digest));frame+=1
        proc.stdin.close();code=proc.wait()
        if code:raise RuntimeError(f'FFmpeg failed: {code}')
        partial.replace(output)
    finally:
        studio.close()
    report={'model':assembly.name,'groups':groups,'frames':frame,'unique_frames':len({r['pixel_sha256'] for r in rows}),
            'resolution':list(size),'fps':fps,'antialiasing':aa_text,'seconds_to_render':time.time()-start,'probe':probe(output)}
    output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def main():
    p=argparse.ArgumentParser();p.add_argument('recipe');p.add_argument('--out',type=Path,required=True)
    p.add_argument('--size',type=resolution,default=(1280,720));p.add_argument('--fps',type=int,default=18)
    p.add_argument('--seconds',type=float,default=10.0);p.add_argument('--aa',default='ssaa2')
    a=p.parse_args();print(json.dumps(render(a.recipe,a.out,a.size,a.fps,a.seconds,a.aa),indent=2))

if __name__=='__main__':main()
