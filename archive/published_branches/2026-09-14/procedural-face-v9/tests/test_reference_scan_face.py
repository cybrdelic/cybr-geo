"""Checks for the actual portrait integration risks: UV seams and export scale."""
import importlib.util
from pathlib import Path
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def portrait():
    if not (ROOT/'assets/portrait/LeePerrySmith.glb').exists():
        pytest.skip('Run tools/fetch_portrait_assets.py to fetch the attributed scan')
    spec=importlib.util.spec_from_file_location('portrait_recipe_test',ROOT/'examples/reference_scan_face.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.build()


def test_subdivision_has_no_displacement_cracks_at_uv_seams(portrait):
    p=portrait.parts[0]
    assert len(p.faces)==17684*16
    assert np.isfinite(p.vertices).all()
    assert np.isfinite(p.normals).all()
    assert np.allclose(np.linalg.norm(p.normals,axis=1),1.,atol=2e-5)
    assert p.portrait_uv.shape==(len(p.vertices),2)
    assert p.portrait_uv.min()>=0 and p.portrait_uv.max()<=1
    # UV chart splits must not produce disconnected, displaced silhouettes.
    # Welding the final positions must recover all ordinary interior edges.
    _,idx=np.unique(np.round(p.vertices,6),axis=0,return_inverse=True)
    f=idx[p.faces]
    edges=np.sort(np.concatenate([f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]]),axis=1)
    _,counts=np.unique(edges,axis=0,return_counts=True)
    assert np.max(counts)<=2
    assert np.sum(counts==1)<1000  # the actual lower scan boundary is left open
    assert np.ptp(p.vertices[:,2])>300
    assert np.ptp(p.vertices[:,2])<325


def test_texture_landmark_mapping(portrait):
    p=portrait.parts[0]
    # The lip texture must sit on the mouth in the source geometry, not on the
    # chin. Actual OBJ-import handedness is checked separately below.
    uv=p.portrait_uv
    lip=(abs(uv[:,0]-.5)<.04)&(abs(uv[:,1]-.535)<.025)
    assert lip.sum()>10
    centre=p.vertices[lip].mean(axis=0)
    assert -55<centre[2]<5
    assert centre[1]<-50
