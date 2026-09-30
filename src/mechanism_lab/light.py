"""Assembly-to-CYBR LIGHT adapter: actual poses, smooth meshes and optics."""
from __future__ import annotations
from dataclasses import replace
from pathlib import Path
import json
import math
import subprocess
import tempfile
import numpy as np
from cybr_light import Scene, Meshlets, render, read_pfm
from .photoreal import resolve_studio, prepare_view_geometry, _camera_distance
from .truth import assert_renderable, write_truth_report


def scene_for(assembly,view,mesh,size,spp,threads,depth,bands=8,f_stop=None,focus_distance=None,seed=2026):
    scene=Scene();scene.settings(*size,spp,depth,threads,bands,seed,view.exposure)
    a,e=math.radians(view.az),math.radians(view.el)
    distance=_camera_distance(view,size);target=np.asarray(view.target)*.001
    direction=np.array([math.cos(e)*math.cos(a),math.cos(e)*math.sin(a),math.sin(e)])
    fstop=view.f_stop if f_stop is None else f_stop
    if fstop<=0:raise ValueError('f-stop must be positive')
    vfov=math.degrees(2*math.atan(view.sensor_width_mm/(size[0]/size[1])/(2*view.focal_length_mm)))
    scene.camera(target+direction*distance*.001,target,fov=vfov,
                 aperture=view.focal_length_mm*.001/(2*fstop),focus=(focus_distance or view.focus_distance_mm or distance)*.001,
                 orthographic=view.projection=='orthographic',scale=2*view.scale*.001)
    for material in assembly.materials:
        if material.opacity<1:
            # Existing GEO transparent surfaces are glass concepts, not measured
            # transmission spectra. Their RGB color controls absorption.
            absorption=np.maximum(0,1-np.asarray(material.color))*2
            scene.material('roughglass' if material.rough>.04 else 'glass',color=(1,1,1),rough=material.rough,
                           ior=material.ior,absorption=absorption)
        else:
            scene.material('metal' if material.metal>=.5 else 'plastic',color=material.color,rough=material.rough,
                           ior=material.ior,anisotropy=material.anisotropy,
                           eta=(1,1,1),k=np.sqrt(4*np.clip(material.color,.001,.995)/(1-np.clip(material.color,.001,.995))))
    center=np.asarray(view.studio_target)*.001
    span=max(.01,view.studio_scale*.2)
    if view.floor:
        floor=scene.material('plastic',view.floor_color,view.floor_roughness)
        scene.emit('quad',floor,16777000,[center[0]-span*12,center[1]-span*12,view.floor_z_mm*.001],
                   [span*24,0,0],[0,span*24,0],[0,0,0])
    scene.emit('environment',view.background_color,max(.025,view.environment_strength));scene.emit('environment_flat')
    # Three fixed softboxes create broad key highlights, a cool fill and rim.
    # Lights are locked to the assembly's studio, independently of camera orbit.
    for az,el,power,kelvin,w,h in [(view.studio_az-40,55,7,5600,1.6,1.4),
                                (view.studio_az+85,35,3,7500,1.2,2.2),
                                (view.studio_az+170,65,9,6200,1.8,.45)]:
        aa,ee=map(math.radians,(az,el));axis=np.array([math.cos(ee)*math.cos(aa),math.cos(ee)*math.sin(aa),math.sin(ee)])
        position=center+axis*span*2.4
        normal=-axis;right=np.cross([0,0,1],normal);right/=np.linalg.norm(right);up=np.cross(normal,right)
        u=right*span*w*view.light_size;v=up*span*h*view.light_size
        emitter=scene.material('emitter',(1,1,1),emission=power*view.light_intensity,kelvin=kelvin)
        scene.emit('quad',emitter,16777001,position-(u+v)/2,u,v,[0,0,0])
    scene.mesh(mesh);return scene


def write_geometry(assembly,path,time_seconds=0,explode=0):
    with Meshlets(path) as writer:
        for index,part in enumerate(assembly.parts):
            matrix=assembly.pose(part,time_seconds,explode)
            vertices=(part.vertices@matrix[:3,:3].T+matrix[:3,3])*.001
            normals=part.normals@matrix[:3,:3].T
            for start in range(0,len(part.faces),24000):
                faces=part.faces[start:start+24000];a=np.zeros((len(faces),36),'<f4')
                a[:,:9]=vertices[faces].reshape(-1,9);a[:,9:18]=normals[faces].reshape(-1,9)
                a[:,18]=part.material;a[:,19]=index+1;a[:,26:35]=1
                writer.add(a)
    return {'triangles':writer.triangles,'meshlets':writer.meshlets,'indexed_vertices':writer.vertices,
            'bytes':Path(path).stat().st_size,'unindexed_bytes':16+writer.triangles*144}


def render_light(assembly,output,view_name='hero',size=(1100,825),spp=96,threads=4,depth=14,
                 intent='auto',allow_estimates=False,time_seconds=0,f_stop=None,focus_distance=None,bands=8,
                 explode=None,seed=2026):
    truth=assert_renderable(assembly,intent,allow_estimates)
    view=resolve_studio(assembly,assembly.views[view_name]);subset,view=prepare_view_geometry(assembly,view)
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    if output.suffix.lower()!='.png':raise ValueError('CYBR LIGHT still output must end in .png')
    mesh=output.with_suffix('.clm')
    geometry=write_geometry(subset,mesh,time_seconds,view.explode if explode is None else explode)
    scene=scene_for(subset,view,mesh,size,spp,threads,depth,bands,f_stop,focus_distance,seed)
    report=render(scene,output.with_suffix(''))
    report.update(model=assembly.name,view=view_name,resolution=list(size),spp=spp,geometry=geometry,
                  render_profile='light',truth=truth,material_mapping='GEO RGB authoring controls, metal/plastic/glass; metalness threshold 0.5; coat uses plastic Fresnel')
    output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    write_truth_report(truth,output.with_suffix('.truth.json'));return report


def render_light_video(assembly,output,shots,size=(1280,720),fps=24,spp=64,threads=4,depth=12,
                       shutter_angle=180,shutter_samples=3,intent='auto',allow_estimates=False,bands=8):
    from PIL import Image
    from .finish_render import tonemap
    from .media import probe
    truth=assert_renderable(assembly,intent,allow_estimates)
    if fps<1 or shutter_samples<1 or spp<shutter_samples or not 0<=shutter_angle<=360:raise ValueError('Invalid film sampling')
    if any(v<1 or v%2 for v in size):raise ValueError('H.264 dimensions must be positive and even')
    if not shots or any(s.duration<=0 or round(s.duration*fps)<1 or s.view not in assembly.views or s.action not in ('motion','orbit','explode','still') for s in shots):raise ValueError('Invalid shot')
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True);output.with_suffix('.json').unlink(missing_ok=True)
    count=0
    with tempfile.TemporaryDirectory(prefix='cybr-light-film-') as directory:
        directory=Path(directory)
        for shot in shots:
            fixed=resolve_studio(assembly,assembly.views[shot.view])
            for frame in range(round(shot.duration*fps)):
                u=frame/max(1,round(shot.duration*fps)-1)
                view=replace(fixed,az=fixed.az+shot.orbit_degrees*(u-.5) if shot.action=='orbit' else fixed.az)
                posed=replace(assembly,views={**assembly.views,shot.view:view})
                explosion=.5-.5*math.cos(math.tau*u) if shot.action=='explode' else fixed.explode
                films=[]
                for sample in range(shutter_samples):
                    time=max(0,count/fps+((sample+.5)/shutter_samples-.5)*shutter_angle/(360*fps)) if shot.action=='motion' else 0
                    path=directory/'sample.png'
                    render_light(posed,path,shot.view,size,spp//shutter_samples+int(sample<spp%shutter_samples),threads,depth,
                                 intent,allow_estimates,time,bands=bands,explode=explosion,seed=2026+count*17+sample)
                    films.append(read_pfm(path.with_suffix('.pfm')))
                Image.fromarray(tonemap(np.mean(films,axis=0),view.exposure,'aces')).save(directory/f'{count:06}.png');count+=1
        subprocess.run(['ffmpeg','-y','-v','error','-xerror','-framerate',str(fps),'-i',str(directory/'%06d.png'),'-frames:v',str(count),'-an','-c:v','libx264','-crf','16','-pix_fmt','yuv420p','-movflags','+faststart',str(output)],check=True)
        subprocess.run(['ffmpeg','-v','error','-xerror','-i',str(output),'-f','null','-'],check=True)
    encoded=probe(output)
    if int(encoded['streams'][0]['nb_read_frames'])!=count:raise RuntimeError('Encoded frame count mismatch')
    report={'renderer':'CYBR LIGHT 0.2','frames':count,'fps':fps,'spp_per_frame':spp,'wavelengths_per_packet':bands,
            'shutter_angle':shutter_angle,'shutter_samples':shutter_samples,'probe':encoded,'truth':truth,'image_generation_used':False}
    output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n');write_truth_report(truth,output.with_suffix('.truth.json'));return report
