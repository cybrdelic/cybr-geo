import math
import json
import numpy as np
import pytest
from mechanism_lab.assembly_model import ComponentAssembly, RackPinionMotion, require_component_export


def matrix(x=0., y=0., angle=0.):
    c,s=math.cos(angle),math.sin(angle)
    return np.array([[c,-s,0,x],[s,c,0,y],[0,0,1,0],[0,0,0,1.]])


def model():
    parent=matrix(.4,.2,.7)
    poses={'base':matrix(),'tool':parent,'jaw+':parent@matrix(.019,.1692),'jaw-':parent@matrix(-.019,.1692)}
    parts=[dict(id=10,body_index='tool',group='tool'),dict(id=11,body_index='jaw+',group='tool'),dict(id=12,body_index='jaw-',group='tool')]
    motion=RackPinionMotion('tool','jaw+',(10,),.018,.3,.003,(0,.1515,0),{'jaw+':1.,'jaw-':-1.})
    return ComponentAssembly(parts,{'base':None,'tool':'base','jaw+':'tool','jaw-':'tool'}, {1:poses},motions=[motion],
        drives=[dict(parent_body='base',child_body='tool',axis=[0,1,0],origin_m=[0,0,0])],
        selections={'tool':dict(ids=[10,11,12],relative_body='tool')})


def test_render_collision_and_export_share_identical_rack_placement():
    assembly=model();motion=assembly.motions[0]
    q=.024;angle=.3-(q-.018)/.003
    rotation=matrix(angle=angle);pivot=np.array([0,.1515,0])
    rotation[:3,3]=pivot-rotation[:3,:3]@pivot
    assert np.allclose(assembly.local_pose(1,10,'tool',rack_coordinate_m=q),rotation)
    assert np.allclose(assembly.pose(1,10,rack_coordinate_m=q),assembly.body_pose(1,'tool')@rotation)
    assert np.allclose(assembly.local_pose(1,11,'tool',rack_coordinate_m=q)[:3,3],[q,.1692,0])
    assert np.allclose(assembly.local_pose(1,12,'tool',rack_coordinate_m=q)[:3,3],[-q,.1692,0])
    assert assembly.rack_coordinate(1)==pytest.approx(.019)
    assert assembly.angle(1)==pytest.approx(.3-(.019-.018)/.003)
    assert assembly.select_ids('tool')==(10,11,12)
    assert assembly.manifest()['whole_robot_qualified'] is False


def test_gravity_uses_topology_not_part_or_body_number_order():
    assembly=ComponentAssembly([dict(id=77,body_index=2)],{99:None,8:99,2:8},
       {1:{99:matrix(),8:matrix(),2:matrix(1.)}})
    delta={2:dict(delta_mass_kg=2.,delta_first_moment_kg_m=[0,0,0])}
    drives=[dict(parent_body=99,child_body=8,axis=[0,1,0],origin_m=[0,0,0])]
    assert assembly.gravity_increment(1,delta,drives)==pytest.approx([19.62])


def mass(pid,body,m,x):
    return dict(id=pid,body_index=body,mass_kg=m,com_m=[x,0,0],inertia_com_kg_m2=(np.eye(3)*.1).tolist())


def test_parallel_axis_deltas_and_new_parts_have_one_mechanical_owner():
    result=ComponentAssembly.mass_deltas([mass(1,'a',2.,1.)],[mass(1,'a',3.,2.),mass(2,'b',1.,0.)])
    assert result['body_deltas']['a']['delta_mass_kg']==1.
    assert result['body_deltas']['a']['delta_first_moment_kg_m']==[4.,0.,0.]
    assert np.allclose(result['body_deltas']['a']['delta_inertia_origin_kg_m2'],np.diag([0,10,10]))
    assert result['body_deltas']['b']['delta_mass_kg']==1.
    assert len(result['part_deltas'])==2


def test_body_reassignment_is_not_silently_counted_on_the_new_body_only():
    result=ComponentAssembly.mass_deltas([mass(1,'a',2.,1.)],[mass(1,'b',2.,1.)])
    assert result['body_deltas']['a']['delta_mass_kg']==-2.
    assert result['body_deltas']['b']['delta_mass_kg']==2.


def test_ambiguous_component_or_motion_ownership_rejected_at_construction():
    a=model()
    with pytest.raises(ValueError,match='exclusively owned'):
        ComponentAssembly(list(a.parts.values()),a.body_parents,a.frames,motions=a.motions*2)
    with pytest.raises(ValueError,match='Duplicate component'):
        ComponentAssembly([dict(id=1,body_index=0)]*2,{0:None},{1:{0:matrix()}})
    with pytest.raises(ValueError,match='cyclic'):
        ComponentAssembly([],{0:1,1:0},{1:{0:matrix(),1:matrix()}})


def test_document_roundtrip_preserves_motion_identity_and_numeric_body_ids():
    original=model()
    restored=ComponentAssembly.from_document(json.loads(json.dumps(original.manifest())))
    assert restored.select_ids()==original.select_ids()
    assert restored.drives==original.drives
    assert restored.selections==original.selections
    for pid in original.parts:
        assert np.allclose(restored.pose(1,pid,rack_coordinate_m=.031),original.pose(1,pid,rack_coordinate_m=.031))
    numeric=ComponentAssembly([dict(id=1,body_index=4)],{4:None},{1:{4:matrix()}})
    assert ComponentAssembly.from_document(json.loads(json.dumps(numeric.manifest()))).parts[1]['body_index']==4


def test_inspection_is_explicit_and_manufacturing_uses_the_existing_release_contract():
    from mechanism_lab.qualification import QualificationError
    assembly=model()
    with pytest.raises(QualificationError,match='QualifiedNativeConstruction and release_project'):
        assembly.require_export()
    with pytest.raises(QualificationError):
        require_component_export()
    assert require_component_export('inspection')['hardware_qualified'] is False
    assert assembly.require_export('inspection')['assembly_revision']==assembly.revision
    document=assembly.manifest()
    document['units']='mm'
    with pytest.raises(ValueError,match='units=m'):
        ComponentAssembly.from_document(document)
