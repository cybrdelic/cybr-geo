"""Runtime dispatch for the true V9 renderer.

The approved ORBIT V9 artifact stays on its exact Mitsuba PLY/``llvm_ad_rgb``
path. Other CYBR GEO assemblies use the *same* V9 camera, HDRI, physical bench,
principled materials, path integrator, AOVs, OIDN and ACES/sRGB pipeline, but
their already-world-transformed geometry is packed into material/variation OBJ
meshes and loaded with Mitsuba ``scalar_rgb``. This avoids reproducible Mitsuba
3.7.1 PLY-loader crashes seen on otherwise validated AERIS meshes. OBJ here is
only a renderer interchange format: no geometry is simplified, decimated,
moved, replaced, or synthesized.
"""
from __future__ import annotations

from pathlib import Path
import numpy as np

from . import v9 as _core
from .render_profiles import V9

ORBIT_REFERENCE_MODEL='cybr_orbit_inspection_wrist'


def _scalar_mitsuba():
    try:
        import mitsuba as mi
    except ImportError as error:
        raise RuntimeError(
            "V9 requires Mitsuba 3. Install cybr-geo with its current dependencies."
        ) from error
    variants=mi.variants()
    if 'scalar_rgb' not in variants:
        raise RuntimeError(f'Mitsuba scalar_rgb is unavailable; variants={variants}')
    try:
        mi.set_variant('scalar_rgb')
    except Exception:
        if mi.variant()!='scalar_rgb':
            raise
    return mi,'scalar_rgb'


def _write_obj(path:Path,vertices,normals,faces):
    """Write exact indexed geometry as OBJ with one normal per source vertex."""
    path.parent.mkdir(parents=True,exist_ok=True)
    vertices=np.asarray(vertices,dtype=np.float64)
    normals=np.asarray(normals,dtype=np.float64)
    faces=np.asarray(faces,dtype=np.int64)
    if vertices.ndim!=2 or vertices.shape[1]!=3 or normals.shape!=vertices.shape:
        raise ValueError('Invalid V9 OBJ vertex/normal arrays')
    if faces.ndim!=2 or faces.shape[1]!=3:
        raise ValueError('Invalid V9 OBJ triangle array')
    if not np.isfinite(vertices).all() or not np.isfinite(normals).all():
        raise ValueError('Non-finite V9 OBJ geometry')
    if len(faces) and (faces.min()<0 or faces.max()>=len(vertices)):
        raise ValueError('Out-of-range V9 OBJ index')
    with path.open('w',encoding='ascii',newline='\n') as stream:
        stream.write('# CYBR GEO V9 exact render mesh; units are scene millimetres\n')
        for x,y,z in vertices:
            stream.write(f'v {x:.9g} {y:.9g} {z:.9g}\n')
        for x,y,z in normals:
            stream.write(f'vn {x:.9g} {y:.9g} {z:.9g}\n')
        for a,b,c in faces+1:
            stream.write(f'f {a}//{a} {b}//{b} {c}//{c}\n')


def _generic_shapes(assembly,time_seconds=0.0,explode=0.0):
    """Batch exact transformed triangles by material and finish variation."""
    groups={}
    for part in assembly.parts:
        bucket=_core._variation_bucket(part.name)
        key=(int(part.material),bucket)
        record=groups.setdefault(key,{
            'vertices':[],'normals':[],'faces':[],'count':0,'offset':0
        })
        vertices,normals,faces=_core.transformed_mesh(
            assembly,part,time_seconds,explode)
        record['vertices'].append(vertices)
        record['normals'].append(normals)
        record['faces'].append(faces+record['offset'])
        record['offset']+=len(vertices)
        record['count']+=1

    shapes=[]
    for (material,bucket),record in sorted(groups.items()):
        label=f'material_{material:02d}_variation_{bucket}'
        shapes.append({
            'label':label,
            'material':material,
            'variation_key':label,
            'vertices':np.concatenate(record['vertices'],axis=0),
            'normals':np.concatenate(record['normals'],axis=0),
            'faces':np.concatenate(record['faces'],axis=0).astype('<i4',copy=False),
            'source_parts':record['count'],
        })
    return shapes


def _large_scene_dict(mi,assembly,view,size,spp,depth,assets,mesh_dir,
                      time_seconds=0.0,explode=0.0,azimuth=None,
                      f_stop=None,focus_distance=None):
    """V9 scene builder differing from reference only in mesh interchange/variant."""
    origin,target,distance,hfov=_core._camera(view,size,azimuth)
    focal=float(view.focal_length_mm)
    fstop=float(f_stop or view.f_stop or V9.reference_f_stop)
    focus=float(focus_distance or view.focus_distance_mm or distance)

    bench_rough={
        'type':'bitmap','filename':str(assets['roughness']),'raw':True,
        'filter_type':'bilinear','wrap_mode':'repeat',
        'to_uv':mi.ScalarTransform3f.scale([2.2,1.7]),
    }
    bench_bsdf={
        'type':'principled',
        'base_color':{'type':'rgb','value':[.105,.100,.094]},
        'metallic':0.0,'roughness':bench_rough,'specular':.34,'clearcoat':0.0,
    }
    scene={
        'type':'scene',
        'integrator':{
            'type':'aov','aovs':'albedo:albedo,normal:sh_normal',
            'beauty':{'type':'path','max_depth':depth,'rr_depth':5},
        },
        'sensor':{
            'type':'thinlens','fov':hfov,'fov_axis':'x',
            'to_world':mi.ScalarTransform4f.look_at(
                origin=origin.tolist(),target=target.tolist(),up=[0,0,1]),
            'focus_distance':focus,'aperture_radius':focal/(2*fstop),
            'sampler':{'type':'independent','sample_count':spp},
            'film':{
                'type':'hdrfilm','width':size[0],'height':size[1],
                'component_format':'float32',
                'rfilter':{'type':'gaussian','stddev':V9.reconstruction_filter_stddev},
            },
        },
        'environment':{
            'type':'envmap','filename':str(assets['hdri']),
            'scale':V9.environment_scale,
            'to_world':mi.ScalarTransform4f.rotate(
                [0,0,1],V9.environment_rotation_degrees),
            'mis_compensation':True,
        },
    }

    bounds=assembly.bounds
    center=bounds.mean(axis=0)
    span=bounds[1]-bounds[0]
    if view.floor:
        floor_z=float(
            view.floor_z_mm if view.floor_z_mm is not None
            else bounds[0,2]-view.floor_gap_mm
        )-.04
        scene['bench']={
            'type':'rectangle',
            'to_world':mi.ScalarTransform4f.translate([
                float(center[0]),float(center[1]),floor_z
            ]) @ mi.ScalarTransform4f.scale([
                max(185.0,float(span[0])*.90),
                max(135.0,float(span[1])*.90),1]),
            'bsdf':bench_bsdf,
        }

    render_shapes=_generic_shapes(
        assembly,time_seconds=time_seconds,explode=explode)
    triangle_count=0
    source_parts=0
    for index,record in enumerate(render_shapes):
        triangle_count+=len(record['faces'])
        source_parts+=record['source_parts']
        path=mesh_dir/f"{index:04d}_{_core._safe_name(record['label'])}.obj"
        _write_obj(path,record['vertices'],record['normals'],record['faces'])
        scene[f'part_{index:04d}']={
            'type':'obj','filename':str(path),
            'bsdf':_core.principled(
                assembly.materials[record['material']],record['variation_key']),
        }
    if source_parts!=len(assembly.parts):
        raise RuntimeError('V9 generic serialization lost source parts')

    camera={
        'origin':origin.tolist(),'target':target.tolist(),'distance':distance,
        'focal_length_mm':focal,'f_stop':fstop,
        'horizontal_fov_degrees':hfov,
    }
    return (
        scene,triangle_count,camera,len(render_shapes),
        'material-variation-batches-obj-generic',
    )


def _dispatch(function,assembly,*args,**kwargs):
    # Preserve the exact loader/variant used by the approved V9 artifact itself.
    # Generic CYBR GEO models get a more robust interchange path while retaining
    # every visual/transport element that defines V9.
    if assembly.name==ORBIT_REFERENCE_MODEL:
        return function(assembly,*args,**kwargs)

    original_mitsuba=_core._mitsuba
    original_scene=_core._scene_dict
    _core._mitsuba=_scalar_mitsuba
    _core._scene_dict=_large_scene_dict
    try:
        result=function(assembly,*args,**kwargs)
        if isinstance(result,dict):
            result['generic_v9_dispatch']=(
                'scalar_rgb + exact OBJ material/variation batches; '
                'V9 lighting/material/camera/AOV/OIDN/color contract unchanged')
        return result
    finally:
        _core._mitsuba=original_mitsuba
        _core._scene_dict=original_scene


def render_v9(assembly,*args,**kwargs):
    return _dispatch(_core.render_v9,assembly,*args,**kwargs)


def render_v9_video(assembly,*args,**kwargs):
    return _dispatch(_core.render_v9_video,assembly,*args,**kwargs)
