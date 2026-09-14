"""Lossless adapter from the original cybrgeo API to shared photographic backends.

`render` and `film` are V9 by default. The previous in-house BVH/GGX renderer
remains available as `render_native` / `film_native` for compatibility.
"""
from dataclasses import asdict
import numpy as np
from mechanism_lab.core import Assembly,Part,Material,View
from mechanism_lab.render_profiles import V9
from mechanism_lab.v9 import render_v9,render_v9_video
from mechanism_lab.photoreal import render_photoreal,render_photoreal_video


def adapt(scene,view=None,poses=None):
    bounds=scene.bounds
    if view is None:
        view=View(az=35,el=24,scale=float(np.max(bounds[1]-bounds[0]))*.52,
                  target=tuple(bounds.mean(axis=0)),focal_length_mm=72.,f_stop=16.)
    parts=[]
    for p in scene.parts:
        provenance=p.metadata.get('provenance','unknown')
        parts.append(Part(p.name,p.vertices,p.faces,p.normals,p.material,p.group,p.motion,
                          p.center,p.explode,p.role,provenance,scene.cad.get(p.name),
                          finish_axis=tuple(p.metadata.get('finish_axis',(1,0,0))),
                          finish_origin=tuple(p.metadata['finish_origin']) if 'finish_origin' in p.metadata else None))
    def pose(p,t,e):
        matrix=np.eye(4) if poses is None else np.asarray(poses.get(p.name,np.eye(4))).copy()
        matrix[:3,3]+=np.asarray(p.explode)*e
        return matrix
    meta=dict(scene.metadata);meta.setdefault('truth_intent','inspection')
    return Assembly(scene.name,parts,[Material(**asdict(m)) for m in scene.materials],{'hero':view},meta,pose)


def render(scene,path,width=V9.still_size[0],height=V9.still_size[1],samples=V9.still_spp,depth=V9.still_depth,
           camera=None,threads=4,poses=None,view=None):
    if camera is not None:
        az,el,scale,target=camera
        view=View(az,el,scale,tuple(target),focal_length_mm=72.,f_stop=16.)
    return render_v9(adapt(scene,view,poses),path,size=(width,height),spp=samples,depth=depth,intent='inspection')


def film(scene,path,seconds=6.,fps=24,mode='orbit',size=V9.video_size,samples=V9.video_spp,threads=4,depth=V9.video_depth):
    from mechanism_lab.media import Shot
    if mode not in ('orbit','explode'):raise ValueError('Expected orbit or explode')
    return render_v9_video(adapt(scene),path,[Shot('hero',seconds,mode)],size,fps,samples,depth,intent='inspection')


def render_native(scene,path,width=1920,height=1440,samples=512,depth=14,
                  camera=None,threads=4,poses=None,view=None):
    if camera is not None:
        az,el,scale,target=camera
        view=View(az,el,scale,tuple(target))
    return render_photoreal(adapt(scene,view,poses),path,size=(width,height),spp=samples,
                           depth=depth,threads=threads,intent='inspection')


def film_native(scene,path,seconds=6.,fps=24,mode='orbit',size=(1280,720),samples=128,threads=4,depth=12):
    from mechanism_lab.media import Shot
    if mode not in ('orbit','explode'):raise ValueError('Expected orbit or explode')
    return render_photoreal_video(adapt(scene),path,[Shot('hero',seconds,mode)],
                                  size,fps,samples,threads,depth,intent='inspection')
