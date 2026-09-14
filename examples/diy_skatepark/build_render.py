"""Build, audit, export and natively path trace CYBR YARD.

Run from a CYBR GEO checkout with PYTHONPATH=src. No image synthesis or assets.
"""
from __future__ import annotations
import argparse, csv, json, time, sys, hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src'))
sys.path.insert(0,str(Path(__file__).parent))
from recipe import build
from mechanism_lab.core import validate, save_cache, load_cache
from mechanism_lab.exporters import export_glb, export_step, export_bom
from mechanism_lab.photoreal import render_photoreal, render_photoreal_video
from mechanism_lab.media import Shot

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,default=ROOT/'build/diy_skatepark')
    p.add_argument('--views',nargs='*',default=['hero','mini','street','coping','structure'])
    p.add_argument('--width',type=int,default=1920);p.add_argument('--height',type=int,default=1280)
    p.add_argument('--spp',type=int,default=192);p.add_argument('--depth',type=int,default=12)
    p.add_argument('--threads',type=int,default=4);p.add_argument('--cache',action='store_true')
    p.add_argument('--step',action='store_true');p.add_argument('--video',action='store_true')
    p.add_argument('--video-seconds',type=float,default=4);p.add_argument('--video-spp',type=int,default=48)
    args=p.parse_args();out=args.out.resolve();out.mkdir(parents=True,exist_ok=True)
    start=time.time()
    if args.cache:
        a=load_cache(out/'geometry');print('Loaded cached geometry',len(a.parts),flush=True)
        # Camera-only edits remain available without rebuilding CAD.
    else:
        print('Building named analytic CAD and mesh assembly',flush=True);a=build()
        print('Geometry built',len(a.parts),'parts',sum(len(q.faces) for q in a.parts),'triangles',round(time.time()-start,2),'seconds',flush=True)
        report=validate(a,expensive=True)
        (out/'geometry_validation.json').write_text(json.dumps(report,indent=2)+'\n')
        save_cache(a,out/'geometry');export_bom(a,out/'geometry')
        export_glb(a,out/'CYBR_YARD.glb')
        (out/'design.json').write_text(json.dumps(a.metadata,indent=2)+'\n')
        cutlist=a.metadata['cutlist']
        with (out/'nominal_parts.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=['name','group','material','stock','nominal_bounds_mm']);w.writeheader();w.writerows(cutlist)
        if args.step:
            print('Exporting analytic STEP (visual leaf / steel-entry meshes explicitly omitted)',flush=True)
            export_step(a,out/'CYBR_YARD_analytic.step',individual=False)
        print('Export and geometry checks complete',round(time.time()-start,2),'seconds',flush=True)
    results=[]
    for view in args.views:
        output=out/'renders'/f'{view}.png'
        print('Path tracing',view,args.width,args.height,args.spp,'spp',flush=True)
        report=render_photoreal(a,output,view_name=view,size=(args.width,args.height),spp=args.spp,depth=args.depth,threads=args.threads,intent='concept')
        results.append(report);print('Completed',view,round(report['seconds'],2),'seconds',flush=True)
    if args.video:
        print('Path tracing orbit film',flush=True)
        render_photoreal_video(a,out/'CYBR_YARD_orbit.mp4',[Shot('hero',args.video_seconds,'orbit',22.)],
             size=(960,640),fps=24,spp=args.video_spp,threads=args.threads,depth=10,shutter_angle=0,shutter_samples=1,intent='concept')
    if results:
        (out/'render_session.json').write_text(json.dumps(dict(elapsed=time.time()-start,renders=results),indent=2)+'\n')
    print('DONE',round(time.time()-start,2),'seconds',flush=True)
if __name__=='__main__':main()
