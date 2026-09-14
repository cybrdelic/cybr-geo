"""Supplementary inspection videos, honestly labelled VTK PBR (not path traced)."""
from pathlib import Path
from dataclasses import replace
import argparse,sys,os,json
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(Path(__file__).resolve().parent))
os.environ.setdefault('MECHANISM_LAB_ROOT',str(ROOT))
from render_assets import load
from mechanism_lab.media import render_video,Shot,make_gif

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,default=ROOT/'build/aeris');ap.add_argument('--fps',type=int,default=24);args=ap.parse_args()
    folder=args.out/'video';folder.mkdir(exist_ok=True)
    a=load(args.out);c=load(args.out,True)
    # Framing remains the same genuine model; captions explicitly identify PBR.
    a.views['hero']=replace(a.views['hero'],scale=208,note='VTK PBR inspection / actual geometry / concept only')
    a.views['exploded']=replace(a.views['exploded'],scale=245,note='VTK PBR inspection / axial disassembly / no force simulation')
    c.views['hero']=replace(c.views['hero'],scale=190,note='VTK PBR inspection / real CAD section / prescribed 15 rpm')
    clips=[('aeris_service',a,[Shot('hero',3.,'orbit',28,'AERIS / ASSEMBLED'),Shot('exploded',5.,'explode',0,'AERIS / SERVICE EXPLOSION')]),
           ('aeris_cutaway_motion',c,[Shot('hero',5.,'motion',0,'AERIS / ROTOR & FLOW-PATH GEOMETRY')])]
    for name,assembly,shots in clips:
        report=render_video(assembly,folder/f'{name}.mp4',shots,size=(960,624),fps=args.fps)
        assert report['unique_frames']>report['frames']*.8
        make_gif(folder/f'{name}.mp4',folder/f'{name}.gif',width=600,fps=10,seconds=5,start=3 if name=='aeris_service' else 0)

if __name__=='__main__':main()
