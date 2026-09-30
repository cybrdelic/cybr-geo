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
    return build(quality='preview',hair=False)


def test_anatomy_builds_without_any_file_reads(monkeypatch,closed):
    def no_files(*args,**kwargs):
        raise AssertionError('Procedural anatomy attempted file access')
    with monkeypatch.context() as m:
        m.setattr(builtins,'open',no_files)
        m.setattr(Path,'open',no_files)
        again=build(quality='preview',hair=False)
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
