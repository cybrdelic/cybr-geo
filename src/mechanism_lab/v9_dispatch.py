"""Runtime dispatch for the true V9 renderer.

The approved ORBIT V9 artifact stays on its exact Mitsuba PLY/``llvm_ad_rgb``
path. Other CYBR GEO assemblies use the same V9 camera, HDRI, physical bench,
principled materials, path integrator, AOVs, OIDN and ACES/sRGB pipeline, but
their already-world-transformed triangle buffers are handed to Mitsuba as
procedural ``mi.Mesh`` objects instead of being reparsed through OBJ/PLY loader
plugins. This is important for large heterogeneous assemblies such as AERIS.
No geometry is simplified, decimated, remeshed, moved, replaced, or synthesized.

Mitsuba 3.7.1 has a reproducible native crash when the envmap emitter's optional
``mis_compensation`` flag is enabled, independent of image format and execution
variant. Generic V9 therefore leaves that optional sampling optimization off.
This does not alter the HDRI radiance, orientation, exposure, BSDFs or path
integral; it only uses the emitter's ordinary importance sampler instead of its
MIS-compensated sampling distribution.
"""
from __future__ import annotations

from pathlib import Path
import numpy as np

from . import v9 as _core
from .render_profiles import V9

ORBIT_REFERENCE_MODEL='cybr_orbit_inspection_wrist'


def _scalar_mitsuba():
    """Compatibility-named selector for the generic procedural V9 backend.

    Procedural meshes require dynamically sized Dr.Jit arrays, so prefer the
    same LLVM RGB variant as the approved ORBIT V9 render. The scalar variant is
    intentionally not used here: its Point3f is a single 3-vector rather than a
    dynamically sized structure-of-arrays and therefore cannot hold an entire
    mesh buffer.
    """
    try:
        import mitsuba as mi
    except ImportError as error:
        raise RuntimeError(
            "V9 requires Mitsuba 3. Install cybr-geo with its current dependencies."
        ) from error
    variants=mi.variants()
    if 'llvm_ad_rgb' not in variants:
        raise RuntimeError(
            'Generic V9 procedural meshes require Mitsuba llvm_ad_rgb; '
            f'variants={variants}'
        )
    try:
        mi.set_variant('llvm_ad_rgb')
    except Exception:
        if mi.variant()!='llvm_ad_rgb':
            raise
    return mi,'llvm_ad_rgb'


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
        vertices=np.ascontiguousarray(
            np.concatenate(record['vertices'],axis=0),dtype=np.float32)
        normals=np.ascontiguousarray(
            np.concatenate(record['normals'],axis=0),dtype=np.float32)
        faces=np.ascontiguousarray(
            np.concatenate(record['faces'],axis=0),dtype=np.uint32)
        if not np.isfinite(vertices).all() or not np.isfinite(normals).all():
            raise ValueError(f'Non-finite V9 geometry in {label}')
        if len(faces) and int(faces.max())>=len(vertices):
            raise ValueError(f'Out-of-range V9 triangle index in {label}')
        shapes.append({
            'label':label,
            'material':material,
            'variation_key':label,
            'vertices':vertices,
            'normals':normals,
            'faces':faces,
            'source_parts':record['count'],
        })
    return shapes


def _procedural_mesh(mi,record,material):
    """Create a Mitsuba mesh directly from validated CYBR GEO buffers.

    This deliberately avoids every disk mesh parser. Mitsuba's documented
    procedural Mesh API exposes flat position/normal/index buffers through
    ``mi.traverse``; filling those buffers preserves the exact submitted indexed
    triangles and authored vertex normals.
    """
    import drjit as dr

    vertices=record['vertices']
    normals=record['normals']
    faces=record['faces']

    bsdf=mi.load_dict(_core.principled(material,record['variation_key']))
    props=mi.Properties()
    props['bsdf']=bsdf
    mesh=mi.Mesh(
        record['label'],
        vertex_count=len(vertices),
        face_count=len(faces),
        props=props,
        has_vertex_normals=True,
        has_vertex_texcoords=False,
    )
    params=mi.traverse(mesh)

    vertex_pos=mi.Point3f(
        mi.Float(vertices[:,0]),mi.Float(vertices[:,1]),mi.Float(vertices[:,2]))
    vertex_nrm=mi.Normal3f(
        mi.Float(normals[:,0]),mi.Float(normals[:,1]),mi.Float(normals[:,2]))
    face_idx=mi.Vector3u(
        mi.UInt32(faces[:,0]),mi.UInt32(faces[:,1]),mi.UInt32(faces[:,2]))
    params['vertex_positions']=dr.ravel(vertex_pos)
    params['vertex_normals']=dr.ravel(vertex_nrm)
    params['faces']=dr.ravel(face_idx)
    params.update()

    if int(mesh.vertex_count())!=len(vertices) or int(mesh.face_count())!=len(faces):
        raise RuntimeError(f'Procedural V9 mesh count mismatch for {record["label"]}')
    return mesh


def _large_scene_dict(mi,assembly,view,size,spp,depth,assets,mesh_dir,
                      time_seconds=0.0,explode=0.0,azimuth=None,
                      f_stop=None,focus_distance=None):
    """Build the V9 scene with exact procedural meshes for generic assemblies."""
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
            # Mitsuba 3.7.1 segfaults with True on both HDR/EXR and both LLVM/
            # scalar variants. False changes sampling variance only, not the
            # environment's emitted radiance or the converged path integral.
            'mis_compensation':False,
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
        scene[f'part_{index:04d}']=_procedural_mesh(
            mi,record,assembly.materials[record['material']]
        )
    if source_parts!=len(assembly.parts):
        raise RuntimeError('V9 procedural serialization lost source parts')

    camera={
        'origin':origin.tolist(),'target':target.tolist(),'distance':distance,
        'focal_length_mm':focal,'f_stop':fstop,
        'horizontal_fov_degrees':hfov,
    }
    return (
        scene,triangle_count,camera,len(render_shapes),
        'material-variation-batches-procedural-mesh',
    )


def _dispatch(function,assembly,*args,**kwargs):
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
                'llvm_ad_rgb + exact in-memory procedural Mesh material/variation '
                'batches; workshop HDRI with ordinary unbiased importance sampling; '
                'V9 lighting/material/camera/AOV/OIDN/color contract unchanged')
        return result
    finally:
        _core._mitsuba=original_mitsuba
        _core._scene_dict=original_scene


def render_v9(assembly,*args,**kwargs):
    return _dispatch(_core.render_v9,assembly,*args,**kwargs)


def render_v9_video(assembly,*args,**kwargs):
    return _dispatch(_core.render_v9_video,assembly,*args,**kwargs)
