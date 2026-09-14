import json
import numpy as np
import pytest
from cybrgeo import Assembly,Material,Part,rotation,translation,from_shape

def sample():
 import cadquery as cq
 s=cq.Workplane('XY').box(20,30,5).val()
 return Assembly('test',[from_shape('box',s)],cad={'box':s})

def test_round_trip(tmp_path):
 a=sample();a.save(tmp_path);b=Assembly.load(tmp_path)
 assert b.parts[0].name=='box'
 assert np.allclose(a.parts[0].vertices,b.parts[0].vertices)
 assert b.validate()['part_count']==1

def test_archive_integrity(tmp_path):
 a=sample();a.save(tmp_path);p=tmp_path/'meshes.npz';p.write_bytes(p.read_bytes()+b'bad')
 with pytest.raises(ValueError,match='SHA256'):Assembly.load(tmp_path)

def test_units_in_glb(tmp_path):
 import trimesh
 a=sample();a.export_glb(tmp_path/'a.glb');s=trimesh.load(tmp_path/'a.glb',force='scene')
 assert np.allclose(s.extents,[.02,.005,.03],atol=1e-6)

def test_transform_inverse():
 p=sample().parts[0];m=translation([8,9,2])@rotation(.56)
 assert np.allclose(p.transformed(m).transformed(np.linalg.inv(m)).vertices,p.vertices)

def test_duplicate_rejected():
 p=sample().parts[0]
 with pytest.raises(ValueError):Assembly('invalid',[p,p])

def test_hlr(tmp_path):
 from cybrgeo.whiteprint import project,write_whiteprint
 a=sample();v=project(a.cad['box'])
 assert len(v.visible)>0
 out=write_whiteprint(a.cad['box'],tmp_path/'sheet',title='TEST BOX')
 assert out['units']=='mm'
 for ext in ['svg','pdf','dxf','png']:assert (tmp_path/f'sheet.{ext}').stat().st_size>100

def test_palette_failure():
 p=sample().parts[0];p.material=100
 with pytest.raises(ValueError):Assembly('bad_palette',[p])

def test_brep_roundtrip(tmp_path):
 a=sample();a.save(tmp_path);b=Assembly.load(tmp_path,with_cad=True)
 assert len(b.cad)==1
 assert abs(b.cad['box'].Volume()-3000)<1e-5

def test_profile_and_holes():
 from cybrgeo.features import involute_profile,bolt_circle,involute_spur_gear
 p=involute_profile(20,1.75,.1)
 assert np.isfinite(p).all()
 assert abs(p[:,0].max()-19.25)<1e-9
 assert len(bolt_circle(8,49.8,22.5))==8
 assert involute_spur_gear(20,1.75,10,8.05).val().isValid()
