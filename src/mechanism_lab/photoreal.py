"""Photographic Mechanism Lab rendering: thin-lens path tracing and final-film frames."""
from __future__ import annotations
from dataclasses import replace
from pathlib import Path
import json,math,subprocess,tempfile,time
import numpy as np
from PIL import Image
from .core import project_root
from .exporters import export_meshbin


def compile_renderer():
    root=project_root();build=root/'.build/native';exe=build/'mechanism_photoreal'
    subprocess.run(['cmake','-S',str(root/'native'),'-B',str(build),'-DCMAKE_BUILD_TYPE=Release'],check=True)
    subprocess.run(['cmake','--build',str(build),'--target','mechanism_photoreal','--parallel','4'],check=True)
    return exe


def _camera_distance(view,size):
    if view.camera_distance_mm is not None:return float(view.camera_distance_mm)
    aspect=size[0]/size[1];sensor_h=float(view.sensor_width_mm)/aspect
    vfov=2*math.atan(sensor_h/(2*float(view.focal_length_mm)))
    return float(view.scale)/max(1e-6,math.tan(vfov/2))


def _render_scratch_paths(mesh,ppm):
    """All large native-render scratch files produced for one temporal sample."""
    mesh=Path(mesh);ppm=Path(ppm)
    return (
        mesh,
        mesh.with_suffix('.materials'),
        ppm,
        Path(str(ppm)+'.pfm'),
        Path(str(ppm)+'.guides'),
    )


def _cleanup_render_scratch(mesh,ppm):
    """Delete one sample's scratch immediately after its HDR contribution is read.

    A detailed mechanical assembly can produce a meshbin hundreds of megabytes in
    size. Keeping one unique mesh/PFM/guide set per shutter sample makes film disk
    use grow with frame count even though those files are never needed again. Film
    rendering is therefore intentionally streaming: only the accumulated HDR frame
    and compressed PNG frame sequence survive each sample.
    """
    for path in _render_scratch_paths(mesh,ppm):
        try:path.unlink()
        except FileNotFoundError:pass


def _invoke(exe,assembly,view,mesh,ppm,size,spp,threads,depth,seed,time_seconds=0.,explode=None,
            f_stop=None,focus_distance=None):
    export_meshbin(assembly,mesh,time_seconds=time_seconds,explode=view.explode if explode is None else explode)
    distance=_camera_distance(view,size)
    fstop=float(f_stop if f_stop is not None else view.f_stop)
    focus=float(focus_distance if focus_distance is not None else (view.focus_distance_mm or distance))
    cmd=[str(exe),str(mesh),str(ppm),'--materials',str(mesh.with_suffix('.materials')),
         '--w',str(size[0]),'--h',str(size[1]),'--spp',str(spp),'--depth',str(depth),'--threads',str(threads),'--seed',str(seed),
         '--az',str(view.az),'--el',str(view.el),'--tx',str(view.target[0]),'--ty',str(view.target[1]),'--tz',str(view.target[2]),
         '--focal-length',str(view.focal_length_mm),'--sensor-width',str(view.sensor_width_mm),'--camera-distance',str(distance),
         '--fstop',str(fstop),'--focus-distance',str(focus),
         '--env-strength',str(view.environment_strength),'--background-strength',str(view.background_strength),
         '--light-size',str(view.light_size),'--light-intensity',str(view.light_intensity),
         '--floor-gap',str(view.floor_gap_mm),'--floor-roughness',str(view.floor_roughness),'--exposure',str(view.exposure)]
    if view.projection=='orthographic':cmd.extend(['--ortho','--scale',str(2*view.scale)])
    if not view.floor:cmd.append('--no-floor')
    subprocess.run(cmd,check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)


def render_photoreal(assembly,output,view_name='hero',size=(1920,1080),spp=512,threads=4,depth=14,
                     intent='auto',allow_estimates=False,time_seconds=0.,f_stop=None,focus_distance=None,captions=False):
    from .truth import assert_renderable,write_truth_report
    from . import finish_render as filt
    from .render import labelled
    truth=assert_renderable(assembly,intent,allow_estimates);view=assembly.views[view_name]
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True);exe=compile_renderer();start=time.time()
    parts=[p for p in assembly.parts if p.group not in view.hide];subset=replace(assembly,parts=parts)
    with tempfile.TemporaryDirectory(prefix='mechanism_photo_') as td:
        td=Path(td);mesh=td/'scene.meshbin';ppm=td/'render.ppm'
        _invoke(exe,subset,view,mesh,ppm,size,spp,threads,depth,2026,time_seconds,f_stop=f_stop,focus_distance=focus_distance)
        arr=filt.read_pfm(str(ppm)+'.pfm')
        raw=Image.fromarray(filt.tonemap(arr,exposure=view.exposure));raw.save(output.with_name(output.stem+'_linear-tonemapped.png'))
        # Preview sample counts receive exactly one conservative edge-aware pass.
        # >=384 spp is left completely native so material-scale highlight detail is
        # never replaced by a denoiser's guess.
        if spp<384:
            with open(str(ppm)+'.guides','rb') as f:
                w,h=np.fromfile(f,'<u4',2);guides=np.fromfile(f,'<f4').reshape(h,w,9)
            variance=guides[:,:,7].copy();arr,_=filt.atrous(arr,guides,variance,1,0)
        clean=Image.fromarray(filt.tonemap(arr,exposure=view.exposure));clean.save(output)
        if captions:
            counts=truth['tier_counts'];line=f"{truth['resolved_intent'].upper()} / "+', '.join(f'{k}:{v}' for k,v in counts.items())
            labelled(clean,view.title or assembly.name.upper(),view.note,
                     f'{spp} spp / {depth} bounces / thin-lens f/{f_stop or view.f_stop:g} / {line}',
                     tag='CYBR MECHANISM LAB / PHOTOGRAPHIC PATH TRACE').save(output.with_name(output.stem+'_card.png'))
    report={'file':output.name,'model':assembly.name,'view':view_name,'resolution':list(size),'spp':spp,'bounce_limit':depth,
            'projection':view.projection,'focal_length_mm':view.focal_length_mm,'sensor_width_mm':view.sensor_width_mm,
            'camera_distance_mm':_camera_distance(view,size),'f_stop':f_stop or view.f_stop,
            'focus_distance_mm':focus_distance or view.focus_distance_mm or _camera_distance(view,size),
            'environment_strength':view.environment_strength,'light_size':view.light_size,'floor_roughness':view.floor_roughness,
            'seconds':time.time()-start,'renderer':'native thin-lens BVH/GGX/MIS path tracer','truth':truth}
    output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n');write_truth_report(truth,output.with_suffix('.truth.json'));return report


def render_photoreal_video(assembly,output,shots,size=(1920,1080),fps=24,spp=144,threads=4,depth=12,
                           shutter_angle=180.,shutter_samples=3,intent='auto',allow_estimates=False):
    """Slow final-film renderer. Every encoded frame is freshly path traced.

    Each temporal subframe samples the actual authored state at a point inside the
    shutter interval: mechanism pose for motion/orbit shots, camera azimuth for
    orbit shots, and explosion amount for exploded shots. Thin-lens DOF is sampled
    inside every native subframe. The final pixel is the linear-HDR temporal mean,
    so motion blur is neither screen-space nor a post-process velocity blur.

    Native scratch is streamed per shutter sample. This keeps temporary storage
    bounded even for multi-million-triangle assemblies instead of retaining every
    exported mesh, PFM and guide buffer for the entire movie.
    """
    from .truth import assert_renderable,write_truth_report
    from .media import probe
    from . import finish_render as filt
    truth=assert_renderable(assembly,intent,allow_estimates)
    if fps<=0 or shutter_samples<1 or spp<shutter_samples:raise ValueError('Invalid film sampling settings')
    if not 0<shutter_angle<=360:raise ValueError('shutter_angle must be in (0,360]')
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True);exe=compile_renderer();start=time.time();frames=[];global_frame=0
    with tempfile.TemporaryDirectory(prefix='mechanism_film_') as td:
        td=Path(td);mesh=td/'sample.meshbin';ppm=td/'sample.ppm'
        for si,shot in enumerate(shots):
            view=assembly.views[shot.view];parts=[p for p in assembly.parts if p.group not in view.hide];subset=replace(assembly,parts=parts)
            n=round(shot.duration*fps);frame_dt=1/fps;shutter_dt=frame_dt*shutter_angle/360
            shot_span=max(frame_dt,(n-1)*frame_dt)
            for f in range(n):
                base_t=global_frame/fps;accum=None
                for ss in range(shutter_samples):
                    offset=((ss+.5)/shutter_samples-.5)*shutter_dt
                    local_t=min(shot_span,max(0.,f*frame_dt+offset));u=local_t/shot_span
                    sample_time=max(0.,base_t+offset) if shot.action in ('motion','orbit') else 0.
                    angle=view.az+shot.orbit_degrees*(u-.5) if shot.action=='orbit' else view.az
                    explosion=(.5-.5*math.cos(math.tau*u)) if shot.action=='explode' else view.explode
                    sample_view=replace(view,az=angle)
                    try:
                        _invoke(exe,subset,sample_view,mesh,ppm,size,max(1,spp//shutter_samples),threads,depth,
                                2026+global_frame*17+ss,sample_time,explosion)
                        sample=filt.read_pfm(str(ppm)+'.pfm')
                        if accum is None:accum=sample.astype(np.float64,copy=True)
                        else:accum+=sample
                    finally:
                        _cleanup_render_scratch(mesh,ppm)
                if accum is None:raise RuntimeError('No temporal samples rendered for film frame')
                arr=accum/shutter_samples;im=Image.fromarray(filt.tonemap(arr,exposure=view.exposure));path=td/f'{global_frame:06}.png';im.save(path)
                frames.append(path);global_frame+=1
        if not frames:raise ValueError('Film shot list produced no frames')
        listfile=td/'frames.txt';listfile.write_text(''.join(f"file '{p.as_posix()}'\nduration {1/fps}\n" for p in frames)+f"file '{frames[-1].as_posix()}'\n")
        subprocess.run(['ffmpeg','-y','-v','error','-f','concat','-safe','0','-i',str(listfile),'-r',str(fps),'-an','-c:v','libx264','-preset','slow','-crf','16','-pix_fmt','yuv420p','-movflags','+faststart',str(output)],check=True)
    report={'model':assembly.name,'frames':global_frame,'duration':global_frame/fps,'resolution':list(size),'fps':fps,'spp_per_frame':spp,
            'depth':depth,'shutter_angle':shutter_angle,'shutter_samples':shutter_samples,'seconds_to_render':time.time()-start,
            'method':'thin-lens path tracing + streamed temporal geometry/camera/explode supersampling','probe':probe(output),'truth':truth}
    output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n');write_truth_report(truth,output.with_suffix('.truth.json'));return report
