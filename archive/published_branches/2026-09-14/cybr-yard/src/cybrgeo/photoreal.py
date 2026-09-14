"""Lossless scene adapter to CYBR GEO's single photographic renderer.

Both public assembly APIs share the native executable, material implementation,
camera, fixed studio, color transform, and linear-light filtering.
"""
from dataclasses import asdict
import numpy as np
from mechanism_lab.core import Assembly,Part,Material,View
from mechanism_lab.photoreal import render_photoreal,render_photoreal_video


def adapt(scene,view=None,poses=None):
    bounds=scene.bounds
    if view is None:
        view=View(az=35,el=24,scale=float(np.max(bounds[1]-bounds[0]))*.52,
                  target=tuple(bounds.mean(axis=0)),f_stop=11.)
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
    # An adapter cannot promote unknown provenance to a verified reference.
    meta=dict(scene.metadata);meta.setdefault('truth_intent','inspection')
    return Assembly(scene.name,parts,[Material(**asdict(m)) for m in scene.materials],{'hero':view},meta,pose)


def render(scene,path,width=1920,height=1440,samples=512,depth=14,
           camera=None,threads=4,poses=None,view=None):
    if camera is not None:
        az,el,scale,target=camera
        view=View(az,el,scale,tuple(target))
    return render_photoreal(adapt(scene,view,poses),path,size=(width,height),spp=samples,
                           depth=depth,threads=threads,intent='inspection')


def film(scene,path,seconds=6.,fps=24,mode='orbit',size=(1280,720),samples=128,threads=4):
    from mechanism_lab.media import Shot
    if mode not in ('orbit','explode'):raise ValueError('Expected orbit or explode')
    return render_photoreal_video(adapt(scene),path,[Shot('hero',seconds,mode)],
                                  size,fps,samples,threads,intent='inspection')
