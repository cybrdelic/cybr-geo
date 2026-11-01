import copy,hashlib
import numpy as np
import pytest
from mechanism_lab.optical_coordinates import inspection_plan_recipe,coordinates,inspection_bed_coordinates
from mechanism_lab.qualification import QualificationError

@pytest.fixture
def inputs(tmp_path):
    mesh=tmp_path/'mesh';mesh.write_bytes(b'mesh');brep=tmp_path/'brep';brep.write_bytes(b'brep')
    p=dict(part_id=532,status='newly_authored_proposed_unqualified',historical_mapping_recovered=False,
        manufacturing_qualified=False,toolpath_generated=False,requires_support=True,
        mesh_sha256=hashlib.sha256(b'mesh').hexdigest(),brep_sha256=hashlib.sha256(b'brep').hexdigest(),
        parent_to_bed_m=[[1,0,0,0],[0,0,-1,0],[0,1,0,-.1448],[0,0,0,1]],
        nominal_layer_pitch_m=.0002,heightmap_v_cycles=16,texture_u_repeat_m=.0072,
        uv_name='PrintUV',rationale='Side face, supports required',limitations=['Unqualified'])
    return p,mesh,brep

def test_same_frame_drives_height_and_bed(inputs):
    p,m,b=inputs;r=inspection_plan_recipe(p,m,b,purpose='inspection')
    v=np.array([[.0072,.145,0],[0,.1448,.01]])
    bed=inspection_bed_coordinates(v,r,purpose='inspection');uv=coordinates(v,r,purpose='inspection')
    np.testing.assert_allclose(uv[:,0],bed[:,0]/.0072)
    np.testing.assert_allclose(uv[:,1]*16,bed[:,2]/.0002)
    assert r['requires_support'] and r['historical_mapping_recovered'] is False

def test_manufacturing_cannot_use_proposal(inputs):
    p,m,b=inputs
    with pytest.raises(QualificationError):inspection_plan_recipe(p,m,b)
    r=inspection_plan_recipe(p,m,b,purpose='inspection')
    with pytest.raises(QualificationError):coordinates([[0,0,0]],r)
    with pytest.raises(QualificationError):inspection_bed_coordinates([[0,0,0]],r)

def test_stale_geometry_rejected(inputs):
    p,m,b=inputs;m.write_bytes(b'new mesh')
    with pytest.raises(ValueError,match='Stale'):inspection_plan_recipe(p,m,b,purpose='inspection')

@pytest.mark.parametrize('change',['mirror','scale','nan','qualification','support'])
def test_invalid_frame_and_metadata_rejected(inputs,change):
    p,m,b=inputs;p=copy.deepcopy(p)
    if change=='mirror':p['parent_to_bed_m'][0][0]=-1
    if change=='scale':p['parent_to_bed_m'][0][0]=2
    if change=='nan':p['parent_to_bed_m'][0][0]=float('nan')
    if change=='qualification':p['manufacturing_qualified']=True
    if change=='support':del p['requires_support']
    with pytest.raises(ValueError):inspection_plan_recipe(p,m,b,purpose='inspection')
