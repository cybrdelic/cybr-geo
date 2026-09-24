"""Behavioral checks for original geometry and its physically visible apertures."""
from pathlib import Path
import builtins
import sys
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'examples'),str(ROOT/'tools')]
from procedural_human_face import build,FaceParameters,Anatomy,SKIN


@pytest.fixture(scope='module')
def closed():
    return build(FaceParameters(eyelid_closure=1.0),quality='preview',hair=False)


def test_anatomy_builds_without_any_file_reads(monkeypatch,closed):
    def no_files(*args,**kwargs):
        raise AssertionError('Procedural anatomy attempted file access')
    with monkeypatch.context() as m:
        m.setattr(builtins,'open',no_files)
        m.setattr(Path,'open',no_files)
        again=build(FaceParameters(eyelid_closure=1.0),quality='preview',hair=False)
    assert again.metadata['input_assets']==[]
    assert again.metadata['scan_used'] is False
    assert len(again.parts)==len(closed.parts)
    for a,b in zip(closed.parts,again.parts):
        assert np.array_equal(a.vertices,b.vertices)
        assert np.array_equal(a.faces,b.faces)


def test_referenced_geometry_and_ear_charts_are_nondegenerate(closed):
    for p in closed.parts:
        assert np.isfinite(p.vertices).all()
        assert np.isfinite(p.normals).all()
        assert p.faces.min()>=0 and p.faces.max()<len(p.vertices)
        triangle=p.vertices[p.faces]
        area=np.linalg.norm(np.cross(triangle[:,1]-triangle[:,0],triangle[:,2]-triangle[:,0]),axis=1)
        assert (area>1e-10).all(),p.name
        lengths=np.linalg.norm(p.normals[np.unique(p.faces)],axis=1)
        assert np.allclose(lengths,1,atol=1e-5),p.name
        if 'pinna' in p.name:
            uv=p.portrait_uv[p.faces]
            du=uv[:,1]-uv[:,0];dv=uv[:,2]-uv[:,0]
            jac=np.abs(du[:,0]*dv[:,1]-du[:,1]*dv[:,0])
            assert (jac>1e-14).all()


def test_lids_share_exact_skin_boundary_positions_and_normals(closed):
    body=next(p for p in closed.parts if p.name=='Sculpted_head_neck_and_shoulders')
    lookup={tuple(v):n for v,n in zip(body.vertices,body.normals)}
    for name in ['Left_eyelids','Right_eyelids']:
        lid=next(p for p in closed.parts if p.name==name)
        shared=[(i,lookup[tuple(v)]) for i,v in enumerate(lid.vertices) if tuple(v) in lookup]
        assert len(shared)>30
        assert all(np.array_equal(lid.normals[i],normal) for i,normal in shared)


def test_blink_changes_actual_occlusion_and_nostrils_have_depth(closed):
    import mitsuba as mi
    mi.set_variant('llvm_ad_rgb')
    from render_procedural_face import mesh_shape
    material=mi.load_dict({'type':'diffuse'})
    for closure,assembly in [(1.,closed),(0.,build(FaceParameters(eyelid_closure=0),quality='preview',hair=False))]:
        entries={'type':'scene'}
        for p in assembly.parts:entries[p.name]=mesh_shape(p,material)
        scene=mi.load_dict(entries)
        anatomy=Anatomy(FaceParameters(eyelid_closure=closure))
        for side in (-1,1):
            c=anatomy.eye(side)
            ray=mi.Ray3f(mi.Point3f(float(c[0]),-150.,float(c[2])),mi.Vector3f(0,1,0))
            hit=scene.ray_intersect(ray)
            assert bool(np.asarray(hit.is_valid())[0])
            name=hit.shape[0].id()
            assert ('eyelids' in name) if closure else ('ocular_tear_surface' in name),name
        nx=9.7+.35;nz=-16.
        hit=scene.ray_intersect(mi.Ray3f(mi.Point3f(nx,-150,nz),mi.Vector3f(0,1,0)))
        assert 'nasal_vestibule' in hit.shape[0].id()
        y=-150+float(np.asarray(hit.t)[0])
        assert y>float(anatomy.front(nx,nz))+2.


def test_no_scan_or_imported_anatomy_contract():
    assembly=build(FaceParameters(eyelid_closure=.10),quality='preview',hair=False)
    assert assembly.metadata['scan_used'] is False
    assert assembly.metadata['imported_anatomy_mesh'] is False
    assert assembly.metadata['photographic_skin_textures'] is False
    assert assembly.metadata['image_generation'] is False
    assert assembly.metadata['input_assets']==[]
    assert all('no-scan' in p.tags for p in assembly.parts if p.group=='anatomy')
