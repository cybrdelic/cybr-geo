"""Material frames follow moving parts; legacy mesh tables remain readable."""
import cadquery as cq
import numpy as np
from mechanism_lab.core import Assembly,Material,cad_part,axis_pose
from mechanism_lab.exporters import export_meshbin


def test_photographic_table_preserves_part_frames_and_legacy_format(tmp_path):
    material=Material('turned bronze',(.6,.3,.1),1.,.3,coat=.2,anisotropy=.65,microfinish='turned')
    a=cad_part('a',cq.Workplane('XY').box(2,3,4),analytic_normals=True,
               finish_axis=(0,1,0),finish_origin=(0,2,0))
    b=a.moved((10,0,0),prefix='b')
    assembly=Assembly('test',[a,b],[material],motion_function=lambda p,t,e:axis_pose(t,explode=(0,0,5),amount=e))
    path=tmp_path/'photo.meshbin'
    export_meshbin(assembly,path,time_seconds=np.pi/2,explode=1,photographic=True)
    lines=path.with_suffix('.materials').read_text().splitlines()
    assert lines[0]=='CYBR_PHOTO_MATERIALS 2'
    rows=np.array([list(map(float,l.split())) for l in lines[1:]])
    assert rows.shape==(2,17)
    assert np.allclose(rows[:,5],8)
    assert np.allclose(rows[:,9],.65)
    assert np.allclose(rows[:,11:14],[0,0,1])
    assert np.allclose(rows[:,14:],[[0,0,7],[10,0,7]])
    with path.open('rb') as f:
        count=np.fromfile(f,'<u4',1)[0];triangles=np.fromfile(f,'<f4').reshape(count,20)
    assert set(triangles[:,18])=={0,1}
    export_meshbin(assembly,tmp_path/'legacy.meshbin')
    legacy=(tmp_path/'legacy.materials').read_text().splitlines()
    assert len(legacy)==1 and len(legacy[0].split())==6
    assert legacy[0].split()[-1]=='1'
