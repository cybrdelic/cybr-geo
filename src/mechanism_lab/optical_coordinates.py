"""Recover source UVs or explicitly lower authored inspection bed coordinates.

No fallback orientation: source-affine recipes stay unchanged; new bed frames
are explicit, geometry-bound, proposed/unqualified and inspection-only.
The caller owns part/source correspondence and explicit metre-unit declaration.
"""
import numpy as np

def recover_affine(vertices, loop_indices, loop_uv, *, uv_name, source_part_id,
                   source_mesh_sha256, units, tolerance=1e-5):
    if units!='m':raise ValueError('Explicit metre coordinates required')
    xyz=np.asarray(vertices,dtype=np.float64)
    ids=np.asarray(loop_indices,dtype=np.int64)
    uv=np.asarray(loop_uv,dtype=np.float64)
    if xyz.ndim!=2 or xyz.shape[1]!=3 or uv.shape!=(len(ids),2):raise ValueError('Invalid UV/mesh shape')
    if not np.isfinite(xyz).all() or not np.isfinite(uv).all():raise ValueError('Non-finite coordinates')
    if ids.size==0 or ids.min()<0 or ids.max()>=len(xyz):raise ValueError('Invalid loop indices')
    a=np.c_[xyz[ids],np.ones(len(ids))]
    fit,_,rank,_=np.linalg.lstsq(a,uv,rcond=None)
    error=float(np.abs(a@fit-uv).max())
    if rank!=4 or error>tolerance:raise ValueError(f'No verified 3D affine UV convention: rank={rank}, error={error}')
    return {'kind':'source_affine_optical_uv_v1','units':'m','uv_name':uv_name,
            'source_part_id':source_part_id,'source_mesh_sha256':source_mesh_sha256,
            'matrix':fit.tolist(),'maximum_source_uv_error':error,
            'manufacturing_print_orientation':False}

def coordinates(vertices, recipe, *, purpose='manufacturing'):
    if recipe.get('kind')=='authored_inspection_optical_uv_v1':
        bed=inspection_bed_coordinates(vertices,recipe,purpose=purpose)
        return np.column_stack((bed[:,0]/recipe['texture_u_repeat_m'],
            bed[:,2]/(recipe['nominal_layer_pitch_m']*recipe['heightmap_v_cycles'])))
    if recipe.get('kind')!='source_affine_optical_uv_v1' or recipe.get('units')!='m':raise ValueError('Explicit verified optical mapping required')
    xyz=np.asarray(vertices,dtype=np.float64);matrix=np.asarray(recipe['matrix'],dtype=np.float64)
    if xyz.ndim!=2 or xyz.shape[1]!=3 or matrix.shape!=(4,2) or not np.isfinite(xyz).all() or not np.isfinite(matrix).all():raise ValueError('Invalid coordinate mapping')
    return np.c_[xyz,np.ones(len(xyz))]@matrix

def apply_to_mesh(mesh, recipe, *, purpose='manufacturing'):
    """Create optical UV attribute only; preserve geometry and material graph."""
    uv=coordinates([tuple(v.co) for v in mesh.vertices],recipe,purpose=purpose)
    name=recipe['uv_name']
    if name in mesh.uv_layers:raise ValueError('Refusing to overwrite existing authored UVs')
    layer=mesh.uv_layers.new(name=name)
    for loop in mesh.loops:layer.data[loop.index].uv=uv[loop.vertex_index]
    return layer


def inspection_plan_recipe(plan, mesh_path, brep_path, *, purpose='manufacturing'):
    """Bind an explicitly authored bed/optical proposal to actual geometry.

    Uses the existing component inspection contract, never release authority.
    One parent-to-bed frame owns both optical height and future inspection export.
    """
    from .assembly_model import require_component_export
    from pathlib import Path
    import hashlib
    require_component_export(purpose)
    if (plan.get('status')!='newly_authored_proposed_unqualified' or
            plan.get('historical_mapping_recovered') is not False or
            plan.get('manufacturing_qualified') is not False or
            plan.get('toolpath_generated') is not False or
            not isinstance(plan.get('requires_support'),bool)):
        raise ValueError('Explicit proposed/unqualified inspection metadata required')
    for key,path in [('mesh_sha256',mesh_path),('brep_sha256',brep_path)]:
        if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=plan[key]:
            raise ValueError('Stale inspection plan geometry: '+key)
    recipe={k:plan[k] for k in ['part_id','status','historical_mapping_recovered',
        'manufacturing_qualified','toolpath_generated','requires_support','mesh_sha256',
        'brep_sha256','parent_to_bed_m','nominal_layer_pitch_m','heightmap_v_cycles',
        'texture_u_repeat_m','uv_name','rationale','limitations']}
    recipe.update(kind='authored_inspection_optical_uv_v1',units='m')
    _inspection_frame(recipe,purpose=purpose)
    return recipe


def _inspection_frame(recipe, *, purpose):
    from .assembly_model import require_component_export
    require_component_export(purpose)
    if recipe.get('status')!='newly_authored_proposed_unqualified' or recipe.get('manufacturing_qualified') is not False:
        raise ValueError('Only explicitly unqualified authored frames are supported')
    frame=np.asarray(recipe['parent_to_bed_m'],dtype=np.float64)
    if (frame.shape!=(4,4) or not np.isfinite(frame).all() or
            not np.allclose(frame[3],[0,0,0,1],atol=1e-12,rtol=0) or
            not np.allclose(frame[:3,:3]@frame[:3,:3].T,np.eye(3),atol=1e-7,rtol=0) or
            not np.isclose(np.linalg.det(frame[:3,:3]),1,atol=1e-7,rtol=0)):
        raise ValueError('Inspection bed frame must be finite, rigid and right-handed')
    for key in ['nominal_layer_pitch_m','heightmap_v_cycles','texture_u_repeat_m']:
        if not np.isfinite(recipe[key]) or recipe[key]<=0:raise ValueError('Invalid optical repeat scale')
    return frame


def inspection_bed_coordinates(vertices, recipe, *, purpose='manufacturing'):
    """Coordinate transform for future diagnostic export; does not emit toolpaths."""
    frame=_inspection_frame(recipe,purpose=purpose)
    xyz=np.asarray(vertices,dtype=np.float64)
    if xyz.ndim!=2 or xyz.shape[1]!=3 or not np.isfinite(xyz).all():raise ValueError('Invalid vertices')
    return (np.c_[xyz,np.ones(len(xyz))]@frame.T)[:,:3]
