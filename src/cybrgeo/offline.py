"""Reusable CPU path tracing of arbitrary named triangle assemblies.

The existing C++ renderer is retained. A material-palette-specific build is
cached by source hash, not hand-rewritten per object. Both raw and geometric
edge-aware filtered frames are exported. Video PBR is separately labeled.
"""
from pathlib import Path
import hashlib,json,subprocess,time,os
import numpy as np
from PIL import Image
from .core import Assembly

def render(assembly:Assembly, path, width=1400,height=1000,samples=64,depth=7,
           camera=(235,24,70,(-14,0,0)),threads=4,poses=None):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    cache=path.parent/'.cache';cache.mkdir(exist_ok=True)
    source=Path(__file__).with_name('native')/'pathtrace.cpp'
    cpp=source.read_text()
    palette=[]
    for m in assembly.materials:
        rgb=','.join(f'{x:.8f}f' for x in m.color)
        palette.append('{{'+rgb+'},'+f'{m.metal:.8f}f,{m.rough:.8f}f,0'+'}')
    # The baseline's optional floor reserves slot8. We disable the floor here.
    while len(palette)<9:palette.append('{{.037f,.041f,.044f},.1f,.46f,0}')
    start=cpp.index('Material mats[]={');end=cpp.index('\n};',start)+3
    cpp=cpp[:start]+'Material mats[]={\n'+',\n'.join(palette)+'\n};'+cpp[end:]
    key=hashlib.sha256(cpp.encode()).hexdigest()[:16];exe=cache/f'pathtrace_{key}'
    if not exe.exists():
        build=cache/f'pathtrace_{key}.cpp';build.write_text(cpp)
        subprocess.run(['g++','-O3','-std=c++17','-fopenmp',str(build),'-o',str(exe)],check=True)
    meshpath=cache/(path.stem+'.meshbin');recs=[]
    for p in assembly.parts:
        t=np.eye(4) if poses is None else poses.get(p.name,np.eye(4))
        v=p.vertices@t[:3,:3].T+t[:3,3];n=p.normals@t[:3,:3].T
        recs.append(np.c_[v[p.faces].reshape(-1,9),n[p.faces].reshape(-1,9),
                          np.full(len(p.faces),p.material),np.full(len(p.faces),10)].astype('<f4'))
    with meshpath.open('wb') as f:
        np.array([sum(len(r) for r in recs)],dtype='<u4').tofile(f)
        for r in recs:r.tofile(f)
    az,el,scale,target=camera;ppm=cache/(path.stem+'.ppm')
    cmd=[str(exe),str(meshpath),str(ppm),'--w',str(width),'--h',str(height),'--spp',str(samples),
         '--depth',str(depth),'--threads',str(threads),'--ortho','--az',str(az),'--el',str(el),
         '--scale',str(2*scale),'--tx',str(target[0]),'--ty',str(target[1]),'--tz',str(target[2]),
         '--studio-scale','1.3','--no-floor']
    t=time.monotonic()
    with (cache/(path.stem+'.log')).open('w') as log:subprocess.run(cmd,check=True,stdout=log,stderr=log)
    from .filter import read_pfm,tonemap,atrous
    image=read_pfm(str(ppm)+'.pfm');Image.fromarray(tonemap(image)).save(path.with_name(path.stem+'_raw.png'))
    with open(str(ppm)+'.guides','rb') as f:
        wh=np.fromfile(f,'<u4',2);guide=np.fromfile(f,'<f4').reshape(int(wh[1]),int(wh[0]),9)
    var=guide[:,:,7].copy();filtered=image.copy()
    for i in range(3):filtered,var=atrous(filtered,guide,var,2**i,i)
    Image.fromarray(tonemap(filtered)).save(path)
    meta=dict(renderer='C++ Monte Carlo path tracer',width=width,height=height,samples=samples,depth=depth,
              camera=camera,seconds=time.monotonic()-t,parts=len(assembly.parts),
              source_hash=hashlib.sha256(cpp.encode()).hexdigest(),
              filtering='Three geometric guide-aware atrous passes; raw image retained',
              limitation='Material appearance approximated, not measured optical properties')
    path.with_suffix('.json').write_text(json.dumps(meta,indent=2));return meta
