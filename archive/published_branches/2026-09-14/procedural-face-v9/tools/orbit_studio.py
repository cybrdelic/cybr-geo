"""ORBIT revision 2: resumable native photographic stills and geometry films.

Use --held with the inspection-cache for the sample-holding configuration.
Every distinct film pose is path traced. Exploded return frames reuse exactly
the same authored pose. No generated images or interpolated motion frames.
"""
from __future__ import annotations
import argparse
from dataclasses import asdict,replace
import hashlib
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import time
import uuid
import numpy as np
from PIL import Image
from mechanism_lab.core import load_cache,save_cache,validate,pose_cad
from mechanism_lab.finish_render import atrous,read_pfm,tonemap
from mechanism_lab.photoreal import _invoke,compile_renderer,_camera_distance
from mechanism_lab.media import probe
from mechanism_lab.truth import assert_renderable,write_truth_report

ROOT=Path(__file__).resolve().parents[1]
RECIPE=ROOT/'examples/orbit_inspection_wrist.py'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save_png(im,path):
    """Publish a complete, checked PNG in one rename on hosted filesystems."""
    path=Path(path);encoded=io.BytesIO();im.save(encoded,format='PNG')
    payload=encoded.getvalue()
    with Image.open(io.BytesIO(payload)) as check:check.verify()
    temporary=path.with_name(f'{path.stem}.{uuid.uuid4().hex}.partial.png')
    with temporary.open('xb') as stream:
        stream.write(payload);stream.flush();os.fsync(stream.fileno())
    if temporary.stat().st_size!=len(payload):raise IOError(f'Incomplete PNG write: {temporary}')
    with Image.open(temporary) as check:check.verify()
    temporary.replace(path)


def build_deliverables(recipe,out,interfaces=False,drawing=False):
    from mechanism_lab.exporters import export_step,export_glb,export_animated_glb,export_bom
    a=recipe.build()
    report=validate(a,expensive=True)
    if any(not p['watertight'] or not p['consistent_winding'] or p['analytic_valid'] is False for p in report['checks']):
        raise ValueError('Invalid model geometry; inspect the build before rendering')
    (out/'geometry-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    save_cache(a,out/'cache');export_bom(a,out/'model')
    (out/'capabilities.json').write_text(json.dumps(a.metadata,indent=2)+'\n')
    export_step(a,out/'CYBR_ORBIT_analytic.step',individual=False)
    export_glb(a,out/'CYBR_ORBIT.glb');export_glb(a,out/'CYBR_ORBIT_exploded.glb',explode=1)
    export_animated_glb(a,out/'CYBR_ORBIT_motion.glb',duration=8,fps=30)
    b=recipe.with_inspection_coupon(a);save_cache(b,out/'inspection-cache')
    export_glb(b,out/'CYBR_ORBIT_inspection.glb')
    export_animated_glb(b,out/'CYBR_ORBIT_inspection_motion.glb',duration=8,fps=30)
    checks=[]
    for p in b.parts:
        if p.group!='workpiece':continue
        for q in b.parts:
            if 'Grooved_elastomer_pad' not in q.name:continue
            ps=pose_cad(p.cad,b.pose(p));qs=pose_cad(q.cad,b.pose(q))
            overlap=sum(abs(s.Volume()) for s in ps.intersect(qs).Solids())
            checks.append(dict(sample=p.name,pad=q.name,intersection_mm3=overlap,distance_mm=ps.distance(qs)))
            if overlap>1e-5:raise ValueError(f'Sample/pad intersection: {checks[-1]}')
    (out/'inspection-fit.json').write_text(json.dumps(checks,indent=2)+'\n')
    if interfaces:
        from orbit_showcase import check_interfaces
        report=check_interfaces(a)
        (out/'interface-validation.json').write_text(json.dumps(report,indent=2)+'\n')
        if not report['passed']:raise ValueError('Selected interface check failed')
    if drawing:
        from mechanism_lab.whiteprint import whiteprint
        names=['B01_Mounting_shoe','B03_Rear_bearing_pedestal','H02_1_Split_front_bearing_cover','H02_-1_Split_front_bearing_cover',
               'P01_Hollow_sculpted_palm','J04_1_Lofted_windowed_finger','J04_-1_Lofted_windowed_finger']
        whiteprint(a,out/'ORBIT_nominal_whiteprint',part_names=names,title='ORBIT / NOMINAL STRUCTURAL ENVELOPE')
    print(f'BUILD COMPLETE {len(a.parts)} unloaded parts / {len(b.parts)} with sample',flush=True)


def read_buffer(path,channels):
    with Path(path).open('rb') as stream:
        width,height=np.fromfile(stream,'<u4',2)
        return np.fromfile(stream,'<f4').reshape(height,width,channels)


def filtered(radiance,guide,passes):
    variance=guide[:,:,7].copy()
    for i in range(passes):
        radiance,variance=atrous(radiance,guide,variance,2**i,i,floor_id=-2)
    return radiance


def checkpoint(directory,settings):
    signature=hashlib.sha256(json.dumps(settings,sort_keys=True).encode()).hexdigest()
    file=directory/'settings.json'
    if file.exists() and json.loads(file.read_text())['signature']!=signature:
        raise ValueError(f'Incompatible checkpoint: {directory}; choose a new --work directory')
    directory.mkdir(parents=True,exist_ok=True)
    file.write_text(json.dumps(dict(signature=signature,settings=settings),indent=2)+'\n')


def native_frame(exe,assembly,view,work,frame,size,spp,threads,depth,t,explosion,seed):
    """Retain HDR/geometry evidence; use distinct native filenames per pose."""
    buffers=work/f'{frame:06d}.npz';record=work/f'{frame:06d}.json'
    if buffers.exists() and record.exists():
        row=json.loads(record.read_text())
        if digest(buffers)!=row['buffers_sha256']:raise ValueError(f'Corrupt checkpoint {buffers}')
        if (row['model_time_seconds'],row['explosion'],row['seed'])!=(t,explosion,seed):
            raise ValueError(f'Pose or seed differs from checkpoint: {buffers}; choose a new --work directory')
        return row
    mesh=work/f'native_{frame:06d}.meshbin';ppm=work/f'native_{frame:06d}.ppm'
    start=time.monotonic()
    _invoke(exe,assembly,view,mesh,ppm,size,spp,threads,depth,seed,t,explosion,log=work/f'{frame:06d}.render.log')
    radiance=read_pfm(str(ppm)+'.pfm');guides=read_buffer(str(ppm)+'.guides',9)
    surfaces=read_buffer(str(ppm)+'.surfaces',4)
    if not np.isfinite(radiance).all() or np.min(radiance)<0:raise ValueError('Invalid native radiance')
    poses=np.asarray([assembly.pose(p,t,explosion) for p in assembly.parts])
    np.savez_compressed(buffers,radiance=radiance,guides=guides,surfaces=surfaces,poses=poses)
    row=dict(frame=frame,model_time_seconds=t,explosion=explosion,
             render_seconds=time.monotonic()-start,buffers_sha256=digest(buffers),seed=seed)
    record.write_text(json.dumps(row,indent=2)+'\n')
    for p in (mesh,mesh.with_suffix('.materials'),ppm,Path(str(ppm)+'.pfm'),
              Path(str(ppm)+'.guides'),Path(str(ppm)+'.surfaces')):p.unlink(missing_ok=True)
    return row


def render_still(assembly,exe,out,work,name,args):
    view=assembly.views[name]
    subset=replace(assembly,parts=[p for p in assembly.parts if p.group not in view.hide])
    truth=assert_renderable(assembly,'concept',False)
    directory=work/name
    settings=dict(view=asdict(view),resolution=list(args.size),spp=args.spp,depth=args.depth,
                  recipe_sha256=digest(RECIPE),renderer_sha256=digest(exe),
                  mesh_cache_sha256=digest(args.cache/'meshes.npz'),held=args.held,seed=20260913)
    checkpoint(directory,settings)
    row=native_frame(exe,subset,view,directory,0,args.size,args.spp,args.threads,args.depth,
                     args.time,view.explode,20260913)
    with np.load(directory/'000000.npz') as data:
        radiance=data['radiance'];guides=data['guides']
    save_png(Image.fromarray(tonemap(radiance,view.exposure)),directory/'native_unfiltered.png')
    radiance=filtered(radiance,guides,args.passes)
    im=Image.fromarray(tonemap(radiance,view.exposure))
    if args.deliver_size is not None:im=im.resize(args.deliver_size,Image.Resampling.LANCZOS)
    path=out/f'ORBIT_{name}.png';save_png(im,path)
    report=dict(file=path.name,renderer='CYBR GEO native photographic path tracer, revision 2',
                configuration=assembly.metadata.get('configuration','unloaded_open_jaws'),
                resolution=list(im.size),native_resolution=list(args.size),spp=args.spp,
                bounce_limit=args.depth,camera=asdict(view),camera_distance_mm=_camera_distance(view,args.size),
                filtering=f'{args.passes} normal/depth/part/variance-guided a-trous passes',
                supersampled=args.deliver_size is not None,record=row,settings=settings,truth=truth)
    path.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    write_truth_report(truth,path.with_suffix('.truth.json'))
    print(f'COMPLETE {path.name} {row["render_seconds"]:.1f}s',flush=True)


def render_film(assembly,exe,out,work,name,args):
    view=assembly.views['internal' if name=='motion' else 'exploded']
    subset=replace(assembly,parts=[p for p in assembly.parts if p.group not in view.hide])
    truth=assert_renderable(assembly,'concept',False)
    total=round(args.duration*args.fps)
    if total%2:raise ValueError('Use an even number of film frames')
    directory=work/name
    settings=dict(view=asdict(view),resolution=list(args.size),spp=args.spp,depth=args.depth,
                  fps=args.fps,frames=total,recipe_sha256=digest(RECIPE),renderer_sha256=digest(exe),
                  mesh_cache_sha256=digest(args.cache/'meshes.npz'),held=args.held,
                  filtering_passes=args.passes,seed=20260913,temporal_filter=args.temporal,
                  cycle_phase_seconds=-2. if name=='motion' and args.held else 0.)
    checkpoint(directory,settings)
    # With closed jaws, every part depends only on the wrist angle. Starting at
    # -18 degrees makes the return visit exactly the same poses as the outward
    # sweep. Reuse those poses, just as for the symmetric exploded sequence.
    symmetric=name=='exploded' or args.held
    unique=total//2+1 if symmetric else total
    rows=[];start=time.monotonic()
    for frame in range(unique):
        u=frame/total;t=(8*u+settings['cycle_phase_seconds']) if name=='motion' else 0.
        explosion=.5-.5*math.cos(math.tau*u) if name=='exploded' else view.explode
        row=native_frame(exe,subset,view,directory,frame,args.size,args.spp,args.threads,args.depth,
                         t,explosion,20260913+frame*7919)
        rows.append(row)
        print(f'{name} native {frame+1}/{unique}; {row["render_seconds"]:.1f}s; elapsed {time.monotonic()-start:.1f}s',flush=True)
    temporal_stats=[]
    for frame in range(unique):
        with np.load(directory/f'{frame:06d}.npz') as data:
            radiance=data['radiance'];guides=data['guides']
            if args.temporal:
                from mechanism_lab.film_filter import temporal_radiance
                neighbors=[]
                for other in (frame-1,frame+1):
                    if not symmetric:other%=total
                    if 0<=other<unique:
                        with np.load(directory/f'{other:06d}.npz') as n:
                            neighbors.append({k:n[k] for k in ('radiance','guides','surfaces','poses')})
                radiance,stats=temporal_radiance({k:data[k] for k in ('radiance','guides','surfaces','poses')},neighbors,view)
                temporal_stats.append(stats)
        radiance=filtered(radiance,guides,args.passes)
        pixels=tonemap(radiance,view.exposure);png=directory/f'{frame:06d}.png'
        save_png(Image.fromarray(pixels),png)
        rows[frame]['png_sha256']=digest(png)
        if frame%12==0:print(f'{name} finish {frame+1}/{unique}',flush=True)
    for frame in range(unique,total):
        mirror=total-frame
        if name=='motion':
            model_t=8*frame/total+settings['cycle_phase_seconds']
            error=max(float(np.max(abs(subset.pose(p,model_t,view.explode)-subset.pose(p,rows[mirror]['model_time_seconds'],view.explode)))) for p in subset.parts)
            if error>1e-8:raise ValueError('Attempted reuse of a different mechanism pose')
        else:error=0.
        shutil.copyfile(directory/f'{mirror:06d}.png',directory/f'{frame:06d}.png')
        rows.append(dict(frame=frame,reused_identical_pose_frame=mirror,max_pose_difference=error,png_sha256=rows[mirror]['png_sha256']))
    destination=out/f'ORBIT_{name}.mp4';partial=out/f'ORBIT_{name}.partial.mp4'
    subprocess.run(['ffmpeg','-y','-v','error','-xerror','-framerate',str(args.fps),'-i',str(directory/'%06d.png'),
                    '-frames:v',str(total),'-an','-c:v','libx264','-threads','2','-preset','slow','-crf','15',
                    '-pix_fmt','yuv420p','-movflags','+faststart',str(partial)],check=True)
    partial.replace(destination);encoded=probe(destination)
    if int(encoded['streams'][0]['nb_read_frames'])!=total:raise ValueError('Encoded frame count mismatch')
    report=dict(file=destination.name,settings=settings,configuration=assembly.metadata.get('configuration','unloaded_open_jaws'),
                renderer='CYBR GEO native photographic path tracer, revision 2',
                prescribed_cycle_playback_speed=8/args.duration if name=='motion' else None,
                unique_native_frames=unique,reused_identical_pose_frames=total-unique,
                interpolation=False,motion_blur=False,
                temporal_filter='Conservative adjacent-frame radiance reuse with exact rigid geometry reprojection' if args.temporal else None,
                temporal_stats=temporal_stats,probe=encoded,frames_log=rows,truth=truth)
    destination.with_suffix('.video.json').write_text(json.dumps(report,indent=2)+'\n')
    write_truth_report(truth,destination.with_suffix('.video.truth.json'))
    print(f'COMPLETE {destination}',flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['build','stills','film'])
    parser.add_argument('--cache',type=Path)
    parser.add_argument('--out',required=True,type=Path)
    parser.add_argument('--work',type=Path)
    parser.add_argument('--interfaces',action='store_true')
    parser.add_argument('--drawing',action='store_true')
    parser.add_argument('--held',action='store_true')
    parser.add_argument('--views',nargs='+',default=['hero'])
    parser.add_argument('--clips',nargs='+',choices=['motion','exploded'],default=['motion'])
    parser.add_argument('--size',default='1920x1440')
    parser.add_argument('--deliver-size')
    parser.add_argument('--spp',type=int,default=256)
    parser.add_argument('--passes',type=int,default=3)
    parser.add_argument('--depth',type=int,default=12)
    parser.add_argument('--threads',type=int,default=8)
    parser.add_argument('--fps',type=int,default=24)
    parser.add_argument('--duration',type=float,default=4.)
    parser.add_argument('--time',type=float,default=0.)
    parser.add_argument('--temporal',action='store_true')
    args=parser.parse_args()
    args.size=tuple(map(int,args.size.split('x')))
    args.deliver_size=tuple(map(int,args.deliver_size.split('x'))) if args.deliver_size else None
    spec=importlib.util.spec_from_file_location('orbit_recipe',RECIPE)
    recipe=importlib.util.module_from_spec(spec);spec.loader.exec_module(recipe)
    args.out.mkdir(parents=True,exist_ok=True)
    if args.mode=='build':
        build_deliverables(recipe,args.out,args.interfaces,args.drawing)
        return
    if args.cache is None or args.work is None:raise ValueError('Rendering requires --cache and --work')
    assembly=load_cache(args.cache)
    held=assembly.metadata.get('configuration')=='inspection_sample_held'
    if args.held!=held:raise ValueError('--held must match the cache configuration')
    assembly.motion_function=recipe.inspection_pose if held else recipe.bind_service_motion(assembly)
    args.out.mkdir(parents=True,exist_ok=True);exe=compile_renderer()
    if args.mode=='stills':
        for view in args.views:render_still(assembly,exe,args.out,args.work,view,args)
    else:
        for clip in args.clips:render_film(assembly,exe,args.out,args.work,clip,args)


if __name__=='__main__':main()
