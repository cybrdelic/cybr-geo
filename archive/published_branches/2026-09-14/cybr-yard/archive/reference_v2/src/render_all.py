"""Reproduce the final stills, logs, and render manifest from the mesh."""
from __future__ import annotations
import argparse,hashlib,json,os,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def sha(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for b in iter(lambda:f.read(1<<20),b''):h.update(b)
    return h.hexdigest()

def main():
    p=argparse.ArgumentParser();p.add_argument('--quick',action='store_true');p.add_argument('--threads',type=int,default=5);p.add_argument('--only');a=p.parse_args()
    renderer=ROOT/'src/pathtrace';mesh=ROOT/'geometry/scene.meshbin'
    if not renderer.exists():
        subprocess.run(['g++','-O3','-march=native','-fno-math-errno','-fopenmp','-std=c++17',str(ROOT/'src/pathtrace.cpp'),'-o',str(renderer)],check=True)
    jobs=[('hero','hero',1600,1400,96),('rear','rear',1200,1040,48),('front','front',1000,1000,48),('side','side',1200,900,48),('internals','core',1200,900,48)]
    if a.only:jobs=[j for j in jobs if j[0] in a.only.split(',')]
    (ROOT/'renders').mkdir(exist_ok=True);(ROOT/'logs').mkdir(exist_ok=True)
    manifest={'geometry_sha256':sha(mesh),'renderer_source_sha256':sha(ROOT/'src/pathtrace.cpp'),'pipeline':'triangle path trace -> geometry-guided non-neural variance filtering -> ACES fit -> sRGB','renders':[]}
    for name,view,w,h,spp in jobs:
        if a.quick:w//=2;h//=2;spp=24
        out=ROOT/'renders'/f'{name}.ppm';log=ROOT/'logs'/f'{name}.log';t=time.time()
        cmd=[str(renderer),str(mesh),str(out),'--w',str(w),'--h',str(h),'--spp',str(spp),'--threads',str(a.threads),'--view',view]
        print('Rendering',name,w,h,spp,flush=True)
        with open(log,'w') as f:subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,check=True)
        env=dict(os.environ,OPENBLAS_NUM_THREADS='1',NUMBA_NUM_THREADS=str(min(a.threads,4)))
        subprocess.run(['python',str(ROOT/'src/finish_render.py'),str(out),'--passes','3'],env=env,check=True)
        item=dict(name=name,width=w,height=h,samples_per_pixel=spp,seconds=time.time()-t,raw_png_sha256=sha(out.with_stem(out.stem+'_raw').with_suffix('.png')),final_png_sha256=sha(out.with_suffix('.png')))
        manifest['renders'].append(item)
        (ROOT/'render_manifest.json').write_text(json.dumps(manifest,indent=2))
        print('Complete',name,round(item['seconds'],1),'seconds',flush=True)
if __name__=='__main__':main()
