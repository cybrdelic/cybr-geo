"""Shared offline rendering. No network fetches or external rendering runtime."""
from __future__ import annotations
import hashlib
import errno
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import numpy as np
from PIL import Image


def compile_renderer():
    source=Path(__file__).parent/'native'
    compiler=shutil.which(os.environ.get('CXX','g++'))
    if not compiler:raise RuntimeError('Install a C++17 compiler with OpenMP (Ubuntu/WSL: sudo apt install g++)')
    version=subprocess.check_output([compiler,'--version'])
    files=[source/'src/main.cpp',*sorted((source/'include').rglob('*.hpp'))]
    digest=hashlib.sha256(version+b''.join(p.read_bytes() for p in files)).hexdigest()
    cache=Path(os.environ.get('CYBR_LIGHT_CACHE',str(Path(tempfile.gettempdir())/'cybr-light')))/digest[:24]
    cache.mkdir(parents=True,exist_ok=True);exe=cache/'cybr-light'
    if not exe.is_file():
        temporary=cache/f'cybr-light.{os.getpid()}.tmp'
        log=cache/'compile.log'
        with log.open('w') as stream:
            result=subprocess.run([compiler,'-std=c++17','-O3','-fopenmp','-I'+str(source/'include'),str(source/'src/main.cpp'),'-ldl','-o',str(temporary)],stdout=stream,stderr=stream)
        if result.returncode:raise RuntimeError(f'CYBR LIGHT compilation failed; see {log}')
        temporary.replace(exe)
    return exe,digest


def publish(source,destination):
    """Publish one closed, verified file atomically across output mounts."""
    source=Path(source);destination=Path(destination);temporary=None
    try:
        fd,name=tempfile.mkstemp(prefix=destination.name+'.',suffix='.partial',dir=destination.parent)
        os.close(fd);temporary=Path(name)
        try:os.replace(source,temporary)
        except OSError as exc:
            if exc.errno!=errno.EXDEV:raise
            shutil.copyfile(source,temporary)
        with temporary.open('rb') as stream:os.fsync(stream.fileno())
        temporary.replace(destination)
    finally:
        if temporary is not None:temporary.unlink(missing_ok=True)
        source.unlink(missing_ok=True)


class Meshlets:
    """CLM1: <=64 full-attribute vertices / <=124 triangles per meshlet.

    Deduplication includes normals, UVs and tint, so UV seams and sharp edges
    remain intact. Material/component IDs stay per triangle. Memory is bounded
    by each supplied chunk. The CPU BVH intersects indices into shared meshlet vertex blocks.
    """
    def __init__(self,path):
        self.path=Path(path);self.path.parent.mkdir(parents=True,exist_ok=True)
        # Pack off the watched output mount, then publish once. Streaming
        # millions of tiny writes there can repeatedly synchronize huge files.
        self.stream=tempfile.NamedTemporaryFile(prefix='cybr-meshlets-',suffix='.clm',delete=False)
        self.temporary=Path(self.stream.name);self.stream.write(struct.pack('<4sQI',b'CLM1',0,0))
        self.triangles=0;self.meshlets=0;self.vertices=0

    def add(self,records):
        a=np.asarray(records,dtype='<f4')
        if a.ndim!=2 or a.shape[1]!=36 or not np.isfinite(a).all():raise ValueError('Mesh records must be finite Nx36')
        if (a[:,18:20]<0).any() or (a[:,18:20]>16777215).any() or not np.equal(a[:,18:20],np.floor(a[:,18:20])).all():raise ValueError('Invalid material/component IDs')
        if (a[:,26:35]<0).any():raise ValueError('Negative vertex tint')
        for start in range(0,len(a),124):self._packet(a[start:start+124])

    def _packet(self,a):
        if not len(a):return
        corners=np.concatenate((a[:,:9].reshape(-1,3),a[:,9:18].reshape(-1,3),a[:,20:26].reshape(-1,2),a[:,26:35].reshape(-1,3)),axis=1)
        vertices,indices=np.unique(corners,axis=0,return_inverse=True)
        if len(vertices)>64:
            middle=len(a)//2;self._packet(a[:middle]);self._packet(a[middle:]);return
        self.stream.write(struct.pack('<HH',len(vertices),len(a)))
        self.stream.write(vertices.astype('<f4').tobytes())
        dtype=np.dtype([('indices','u1',(3,)),('material','<u4'),('object','<u4')])
        triangles=np.empty(len(a),dtype=dtype);triangles['indices']=indices.reshape(-1,3)
        triangles['material']=a[:,18].astype('<u4');triangles['object']=a[:,19].astype('<u4')
        self.stream.write(triangles.tobytes());self.triangles+=len(a);self.meshlets+=1;self.vertices+=len(vertices)

    def __enter__(self):return self
    def __exit__(self,kind,error,tb):
        completed=False
        try:
            if kind is None:
                if not self.triangles:raise ValueError('Empty mesh')
                self.stream.seek(0);self.stream.write(struct.pack('<4sQI',b'CLM1',self.triangles,self.meshlets))
                self.stream.flush();os.fsync(self.stream.fileno())
            self.stream.close();completed=kind is None
        finally:
            try:
                if not self.stream.closed:self.stream.close()
            finally:
                if not completed:self.temporary.unlink(missing_ok=True)
        if completed:
            publish(self.temporary,self.path)


class Scene:
    def __init__(self):self.lines=[];self.materials=0
    def emit(self,command,*values):
        def flatten(values):
            for value in values:
                if isinstance(value,(tuple,list,np.ndarray)):yield from flatten(value)
                elif isinstance(value,str):yield value
                else:yield format(float(value),'.10g')
        self.lines.append(command+' '+' '.join(flatten(values)))
    def material(self,kind='diffuse',color=(.5,.5,.5),rough=.5,ior=1.5,dispersion=0,eta=(.25,.6,1.2),k=(3.5,2.8,2),absorption=(0,0,0),emission=0,kelvin=6500,anisotropy=0,texture=None):
        index=self.materials;self.materials+=1
        self.emit('material',kind,color,rough,ior,dispersion,eta,k,absorption,emission,kelvin)
        if anisotropy:
            alpha=max(.001,rough*rough);aspect=np.sqrt(1-.8*min(.99,anisotropy))
            self.emit('anisotropy',index,alpha/aspect,alpha*aspect)
        if texture:self.emit('texture',index,json.dumps(str(Path(texture).resolve())),1,1,1)
        return index
    def settings(self,width,height,spp,depth,threads,bands=8,seed=2026,exposure=1):
        if min(width,height,spp,depth,threads,bands)<1 or bands>128 or width*height>8000000:raise ValueError('Invalid render settings (maximum 8 million pixels)')
        self.emit('settings',width,height,spp,bands,depth,min(5,depth),threads,seed,exposure,0,0,-1,1,1)
        self.emit('film_format','openexr');self.emit('render_options','volpath',1,'tent',1)
    def camera(self,origin,target,up=(0,0,1),fov=40,aperture=0,focus=None,orthographic=False,scale=4):
        origin=np.asarray(origin);target=np.asarray(target)
        if not np.isfinite([*origin,*target,*up,fov,aperture]).all() or np.linalg.norm(target-origin)<1e-9 or not 0<fov<179 or aperture<0:raise ValueError('Invalid camera')
        self.emit('camera',origin,target,up,fov,aperture,focus or np.linalg.norm(target-origin),0,0,orthographic,scale)
    def mesh(self,path):self.emit('meshlets',json.dumps(str(Path(path).resolve())))
    def save(self,path):
        path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
        lines=[]
        for line in self.lines:
            # Mesh and texture assets are portable relative to the scene file.
            import re
            line=re.sub(r'"([^"]+)"',lambda match:json.dumps(os.path.relpath(match.group(1),path.parent.resolve())) if Path(match.group(1)).is_absolute() else match.group(0),line)
            lines.append(line)
        path.write_text('# CYBR LIGHT / indexed meshlets\n'+'\n'.join(lines)+'\n');return path


def read_pfm(path):
    with Path(path).open('rb') as stream:
        kind=stream.readline().strip();w,h=map(int,stream.readline().split());scale=float(stream.readline())
        if kind!=b'PF' or min(w,h)<1 or not np.isfinite(scale) or not scale:raise ValueError('Invalid PFM')
        data=np.fromfile(stream,dtype='<f4' if scale<0 else '>f4')
    if len(data)!=w*h*3:raise ValueError('PFM length mismatch')
    return np.flipud(data.reshape(h,w,3)).copy()*abs(scale)


def render(scene,stem,runner=None,metadata=None):
    stem=Path(stem);stem.parent.mkdir(parents=True,exist_ok=True)
    # A previous success report cannot survive a failed replacement.
    stem.with_suffix('.json').unlink(missing_ok=True)
    executable,digest=compile_renderer();path=scene.save(stem.with_suffix('.cys'))
    # Native outputs are staged separately. The public report is written once
    # after finishing, so a synchronized earlier native report cannot replace it.
    with tempfile.TemporaryDirectory(prefix='cybr-light-film-') as directory:
        native=Path(directory)/'film';command=[str(executable),'--scene',str(path),'--out',str(native)]
        if runner:runner(command)
        else:
            with stem.with_suffix('.log').open('w') as log:
                result=subprocess.run(command,stdout=log,stderr=log)
            if result.returncode:raise RuntimeError(f'CYBR LIGHT render failed; see {stem.with_suffix(".log")}')
        report=json.loads(native.with_suffix('.json').read_text());film=read_pfm(native.with_suffix('.pfm'))
        if film.shape!=(report['height'],report['width'],3) or not np.isfinite(film).all() or report['invalid_path_samples']:raise RuntimeError('Invalid CYBR LIGHT film')
        for source in sorted(Path(directory).glob('film*')):
            if source.suffix!='.json':publish(source,stem.with_name(stem.name+source.name[4:]))
        finish(stem,film)
    report.update(denoising_used=True,denoiser='three camera-footprint, geometry/object/variance guided atrous display passes',engine_source_sha256=digest,mesh_input='CLM1 indexed meshlets: 64 vertices / 124 triangles maximum',raw_film_preserved=True)
    if metadata:report.update(metadata)
    temporary=None
    try:
        with tempfile.NamedTemporaryFile(mode='w',prefix='cybr-light-report-',suffix='.json',delete=False) as stream:
            temporary=Path(stream.name);json.dump(report,stream,indent=2);stream.write('\n');stream.flush();os.fsync(stream.fileno())
        publish(temporary,stem.with_suffix('.json'))
    finally:
        if temporary is not None:temporary.unlink(missing_ok=True)
    return report


def finish(stem,film=None,exposure=None):
    """Filter display output only; preserve all original radiance and guides."""
    from .finishing import atrous,tonemap,save_png
    stem=Path(stem)
    if film is None:film=read_pfm(stem.with_suffix('.pfm'))
    if exposure is None:
        settings=next(l.split() for l in stem.with_suffix('.cys').read_text().splitlines() if l.startswith('settings '))
        exposure=float(settings[9])
    guide=np.zeros((*film.shape[:2],9),np.float32)
    guide[:,:,:3]=read_pfm(stem.with_name(stem.name+'_normal.pfm'))*2-1
    guide[:,:,3:6]=read_pfm(stem.with_name(stem.name+'_albedo.pfm'))
    guide[:,:,6]=read_pfm(stem.with_name(stem.name+'_depth.pfm'))[:,:,0]
    guide[:,:,7]=read_pfm(stem.with_name(stem.name+'_stderr.pfm'))[:,:,0]**2
    guide[:,:,8]=read_pfm(stem.with_name(stem.name+'_object.pfm'))[:,:,0]
    if not np.isfinite(guide).all():raise RuntimeError('Invalid CYBR LIGHT diagnostic passes')
    guide[guide[:,:,6]==0,8]=-1
    save_png(Image.fromarray(tonemap(film,exposure,'aces')),stem.with_name(stem.name+'_unfiltered.png'))
    filtered=film.copy();variance=guide[:,:,7].copy()
    # Express depth in projected pixel footprints. This makes filtering
    # invariant to scene units and scales from a flange to a vaulted room.
    camera=next(l.split() for l in stem.with_suffix('.cys').read_text().splitlines() if l.startswith('camera '))
    depths=guide[:,:,6];positive=depths[depths>0]
    if positive.size:
        footprint=(float(camera[16]) if float(camera[15]) else 2*np.tan(np.radians(float(camera[10]))/2)*float(np.median(positive)))/film.shape[0]
        guide[:,:,6]/=max(footprint,1e-12)
    for iteration in range(3):filtered,variance=atrous(filtered,guide,variance,2**iteration,iteration,-2)
    save_png(Image.fromarray(tonemap(filtered,exposure,'aces')),stem.with_suffix('.png'))
