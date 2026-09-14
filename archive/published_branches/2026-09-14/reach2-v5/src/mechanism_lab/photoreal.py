"""Photographic Mechanism Lab rendering: thin-lens path tracing and final-film frames."""
from __future__ import annotations
from dataclasses import replace
from pathlib import Path
import json,math,subprocess,tempfile,time,hashlib,os
import numpy as np
from PIL import Image
from .core import project_root
from .exporters import export_meshbin


def compile_renderer():
    # Works from an installed wheel as well as a checkout. Hash every included
    # source; renderer upgrades cannot accidentally reuse an old executable.
    source=Path(__file__).parent/'native'
    embree=os.environ.get('CYBR_GEO_EMBREE_ROOT','')
    flags=[]
    if embree:
        prefix=Path(embree).resolve()
        if not (prefix/'include/embree4/rtcore.h').is_file():raise ValueError('CYBR_GEO_EMBREE_ROOT needs Embree 4 headers and lib/')
        flags=['-DCYBR_EMBREE4','-I'+str(prefix/'include'),'-L'+str(prefix/'lib'),'-Wl,-rpath,'+str(prefix/'lib'),'-lembree4']
    key=hashlib.sha256(b''.join(p.read_bytes() for p in sorted(source.glob('*')) if p.suffix in ('.h','.cpp'))+str(flags).encode()).hexdigest()[:20]
    build=Path(os.environ.get('CYBR_GEO_NATIVE_CACHE',str(Path(tempfile.gettempdir())/'cybrgeo-native')))/key
    build.mkdir(parents=True,exist_ok=True);exe=build/'mechanism_photoreal'
    if not exe.exists():
        temporary=build/f'mechanism_photoreal.{os.getpid()}.tmp'
        subprocess.run(['g++','-O3','-std=c++17','-fopenmp',str(source/'photoreal.cpp'),*flags,'-o',str(temporary)],check=True)
        temporary.replace(exe)
    return exe


def resolve_studio(assembly,view):
    """Lock lights and ground in model coordinates before changing any poses."""
    bounds=assembly.bounds
    return replace(view,
        studio_target=view.studio_target or tuple(bounds.mean(axis=0)),
        studio_scale=view.studio_scale if view.studio_scale is not None else max(1e-5,float(np.max(bounds[1]-bounds[0]))/200.),
        studio_az=view.az if view.studio_az is None else view.studio_az,
        studio_el=view.el if view.studio_el is None else view.studio_el,
        floor_z_mm=float(bounds[0,2]-view.floor_gap_mm) if view.floor_z_mm is None else view.floor_z_mm)


def _camera_distance(view,size):
    if view.camera_distance_mm is not None:return float(view.camera_distance_mm)
    aspect=size[0]/size[1];sensor_h=float(view.sensor_width_mm)/aspect
    vfov=2*math.atan(sensor_h/(2*float(view.focal_length_mm)))
    return float(view.scale)/max(1e-6,math.tan(vfov/2))


def prepare_view_geometry(assembly,view):
    """Apply visibility and genuine capped mesh sections before path tracing."""
    parts=[p for p in assembly.parts if p.group not in view.hide]
    if view.section is not None:
        import vtk
        from vtk.util.numpy_support import vtk_to_numpy
        from .render import clip_closed,polydata
        clipped=[]
        for part in parts:
            section=clip_closed(polydata(part),normal=view.section)
            triangle=vtk.vtkTriangleFilter();triangle.SetInputData(section);triangle.Update()
            section=triangle.GetOutput()
            if not section.GetNumberOfPoints():continue
            clipped.append(replace(part,vertices=vtk_to_numpy(section.GetPoints().GetData()).copy(),
                faces=vtk_to_numpy(section.GetPolys().GetConnectivityArray()).reshape(-1,3).copy(),
                normals=vtk_to_numpy(section.GetPointData().GetNormals()).copy(),cad=None))
        parts=clipped;view=replace(view,section=None)
    if not parts:raise ValueError('The view contains no visible geometry')
    return replace(assembly,parts=parts),view


def _invoke(exe,assembly,view,mesh,ppm,size,spp,threads,depth,seed,time_seconds=0.,explode=None,
            f_stop=None,focus_distance=None,log=None):
    view=resolve_studio(assembly,view)
    if view.section is not None:
        raise ValueError('Photographic sections require a pre-sectioned geometry scene; an uncut image will not be substituted')
    export_meshbin(assembly,mesh,time_seconds=time_seconds,explode=view.explode if explode is None else explode,photographic=True)
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
    cmd.extend(['--studio',view.studio_style,'--floor-color',*map(str,view.floor_color),
                '--background-color',*map(str,view.background_color)])
    cmd.extend(['--studio-target',*map(str,view.studio_target),'--studio-scale',str(view.studio_scale),
                '--studio-az',str(view.studio_az),'--studio-el',str(view.studio_el),'--floor-z',str(view.floor_z_mm)])
    if view.projection=='orthographic':cmd.extend(['--ortho','--scale',str(2*view.scale)])
    if not view.floor:cmd.append('--no-floor')
    if log is None:
        subprocess.run(cmd,check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    else:
        with Path(log).open('w') as stream:
            subprocess.run(cmd,check=True,stdout=stream,stderr=stream)


def render_photoreal(assembly,output,view_name='hero',size=(1920,1080),spp=512,threads=4,depth=14,
                     intent='auto',allow_estimates=False,time_seconds=0.,f_stop=None,focus_distance=None,captions=False):
    from .truth import assert_renderable,write_truth_report
    from . import finish_render as filt
    from .render import labelled
    truth=assert_renderable(assembly,intent,allow_estimates);view=resolve_studio(assembly,assembly.views[view_name])
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True);exe=compile_renderer();start=time.time()
    subset,view=prepare_view_geometry(assembly,view)
    with tempfile.TemporaryDirectory(prefix='mechanism_photo_') as td:
        td=Path(td);mesh=td/'scene.meshbin';ppm=td/'render.ppm'
        _invoke(exe,subset,view,mesh,ppm,size,spp,threads,depth,2026,time_seconds,f_stop=f_stop,focus_distance=focus_distance)
        arr=filt.read_pfm(str(ppm)+'.pfm')
        raw=Image.fromarray(filt.tonemap(arr,exposure=view.exposure,operator=view.tone_mapping));filt.save_png(raw,output.with_name(output.stem+'_linear-tonemapped.png'))
        with open(str(ppm)+'.guides','rb') as f:
            w,h=np.fromfile(f,'<u4',2);guides=np.fromfile(f,'<f4').reshape(h,w,9)
        clean=filt.finish_frame(arr,guides,view.exposure,2,view.tone_mapping);filt.save_png(clean,output)
        if captions:
            counts=truth['tier_counts'];line=f"{truth['resolved_intent'].upper()} / "+', '.join(f'{k}:{v}' for k,v in counts.items())
            labelled(clean,view.title or assembly.name.upper(),view.note,
                     f'{spp} spp / {depth} bounces / thin-lens f/{f_stop or view.f_stop:g} / {line}',
                     tag='CYBR MECHANISM LAB / PHOTOGRAPHIC PATH TRACE').save(output.with_name(output.stem+'_card.png'))
    report={'file':output.name,'model':assembly.name,'view':view_name,'resolution':list(size),'spp':spp,'bounce_limit':depth,
            'section_normal':assembly.views[view_name].section,
            'projection':view.projection,'focal_length_mm':view.focal_length_mm,'sensor_width_mm':view.sensor_width_mm,
            'camera_distance_mm':_camera_distance(view,size),'f_stop':f_stop or view.f_stop,
            'focus_distance_mm':focus_distance or view.focus_distance_mm or _camera_distance(view,size),
            'environment_strength':view.environment_strength,'light_size':view.light_size,'floor_roughness':view.floor_roughness,
            'tone_mapping':view.tone_mapping,'filtering':'2 geometry-guided linear-light atrous passes',
            'studio_target':view.studio_target,'studio_scale':view.studio_scale,'floor_z_mm':view.floor_z_mm,
            'seconds':time.time()-start,'renderer':'native thin-lens BVH/GGX/MIS path tracer','truth':truth}
    output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n');write_truth_report(truth,output.with_suffix('.truth.json'));return report


def render_photoreal_video(assembly,output,shots,size=(1920,1080),fps=24,spp=144,threads=4,depth=12,
                           shutter_angle=180.,shutter_samples=3,intent='auto',allow_estimates=False):
    """Slow final-film renderer. Every encoded frame is freshly path traced.

    Motion blur is real temporal supersampling of geometry poses across the shutter
    interval; thin-lens DOF is sampled inside each native subframe.
    """
    from .truth import assert_renderable,write_truth_report
    from .media import probe
    from . import finish_render as filt
    truth=assert_renderable(assembly,intent,allow_estimates)
    if fps<=0 or shutter_samples<1 or spp<shutter_samples:raise ValueError('Invalid film sampling settings')
    if any(s.duration<=0 or round(s.duration*fps)<1 or s.view not in assembly.views or s.action not in ('motion','orbit','explode','still') for s in shots):
        raise ValueError('Invalid film shot')
    if any(n<=0 or n%2 for n in size):raise ValueError('H.264 frame dimensions must be positive and even')
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True);exe=compile_renderer();start=time.time();frames=[];global_frame=0;temporal_stats=[]
    with tempfile.TemporaryDirectory(prefix='mechanism_film_') as td:
        td=Path(td)
        for si,shot in enumerate(shots):
            previous=None
            view=resolve_studio(assembly,assembly.views[shot.view]);subset,view=prepare_view_geometry(assembly,view)
            n=round(shot.duration*fps);frame_dt=1/fps;shutter_dt=frame_dt*shutter_angle/360
            for f in range(n):
                u=f/max(1,n-1);base_t=global_frame/fps
                angle=view.az+shot.orbit_degrees*(u-.5) if shot.action=='orbit' else view.az
                explosion=(.5-.5*math.cos(math.tau*u)) if shot.action=='explode' else view.explode
                v=replace(view,az=angle)
                hdr=[]
                center_guides=None
                center_surfaces=None;center_poses=None
                for ss in range(shutter_samples):
                    offset=((ss+.5)/shutter_samples-.5)*shutter_dt
                    t=max(0.,base_t+offset) if shot.action in ('motion','orbit') else 0.
                    mesh=td/f'mesh_{global_frame}_{ss}.meshbin';ppm=td/f'frame_{global_frame}_{ss}.ppm'
                    sub_samples=spp//shutter_samples+int(ss<spp%shutter_samples)
                    _invoke(exe,subset,v,mesh,ppm,size,sub_samples,threads,depth,2026+global_frame*17+ss,t,explosion)
                    hdr.append(filt.read_pfm(str(ppm)+'.pfm'))
                    if ss==shutter_samples//2:
                        with open(str(ppm)+'.guides','rb') as stream:
                            w,h=np.fromfile(stream,'<u4',2);center_guides=np.fromfile(stream,'<f4').reshape(h,w,9)
                        with open(str(ppm)+'.surfaces','rb') as stream:
                            w,h=np.fromfile(stream,'<u4',2);center_surfaces=np.fromfile(stream,'<f4').reshape(h,w,4)
                        center_poses=np.array([subset.pose(p,t,explosion) for p in subset.parts])
                    for item in (mesh,mesh.with_suffix('.materials'),ppm,Path(str(ppm)+'.pfm'),Path(str(ppm)+'.guides'),Path(str(ppm)+'.surfaces')):
                        item.unlink(missing_ok=True)
                arr=np.mean(hdr,axis=0)
                current=dict(radiance=arr,guides=center_guides,surfaces=center_surfaces,poses=center_poses)
                if previous is not None and shot.action!='orbit' and v.projection=='perspective':
                    from .film_filter import temporal_radiance
                    arr,stats=temporal_radiance(current,[previous],v)
                    temporal_stats.append(dict(frame=global_frame,neighbors=stats))
                previous=current
                im=filt.finish_frame(arr,center_guides,v.exposure,2,v.tone_mapping)
                path=td/f'{global_frame:06}.png';filt.save_png(im,path)
                frames.append(path);global_frame+=1
                if f%12==0 or f==n-1:print(f'{output.name}: shot {si+1}/{len(shots)}, frame {f+1}/{n}',flush=True)
        if not frames:raise ValueError('No frames in film')
        subprocess.run(['ffmpeg','-y','-v','error','-xerror','-framerate',str(fps),'-i',str(td/'%06d.png'),'-frames:v',str(len(frames)),'-an','-c:v','libx264','-preset','slow','-crf','15','-pix_fmt','yuv420p','-movflags','+faststart',str(output)],check=True)
        subprocess.run(['ffmpeg','-v','error','-xerror','-i',str(output),'-f','null','-'],check=True)
    encoded=probe(output)
    if int(encoded['streams'][0]['nb_read_frames'])!=global_frame:raise ValueError('Encoded film frame count mismatch')
    report={'model':assembly.name,'frames':global_frame,'duration':global_frame/fps,'resolution':list(size),'fps':fps,'spp_per_frame':spp,
            'depth':depth,'shutter_angle':shutter_angle,'shutter_samples':shutter_samples,'seconds_to_render':time.time()-start,
            'method':'thin-lens path tracing + temporal geometry supersampling','probe':encoded,'truth':truth}
    report.update(tone_mapping='per-view neutral by default',filtering='shared geometry-guided linear-light finishing',studio='fixed in model coordinates per shot',
                  temporal_filter='one previous native frame reprojected using part transforms; current retains >=80% weight; camera-orbit shots skip reuse',temporal_stats=temporal_stats)
    output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n');write_truth_report(truth,output.with_suffix('.truth.json'));return report
