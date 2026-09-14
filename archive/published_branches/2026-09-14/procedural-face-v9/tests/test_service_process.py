from dataclasses import replace
import cadquery as cq
import numpy as np
import pytest
from mechanism_lab.core import Assembly,Material,cad_part
from mechanism_lab.assembly_process import AssemblyProcess,Move,ToolAccess,validate_process


def fixture():
    sleeve=cq.Workplane('XY').circle(5).circle(2.1).extrude(8).val().translate((0,0,2))
    pin=cq.Solid.makeCylinder(2,12,cq.Vector(0,0,0),cq.Vector(0,0,1))
    return Assembly('slide',[cad_part('sleeve',sleeve),cad_part('pin',pin)],[Material('steel',(.5,.5,.5),1,.3)])


def test_axial_removal_and_exact_reverse():
    a=fixture();p=AssemblyProcess('good',[Move('release','Pull pin',('pin',),(0,0,14))]).bind(a)
    r=validate_process(a,p,linear_step_mm=.5)
    assert r['passed'] and r['operations'][0]['cad_tests']>0
    for t in np.linspace(0,p.duration,17):
        for name,T in p.poses(t).items():assert np.allclose(T,p.poses(p.duration-t,True)[name])
    for T in p.poses(p.duration,True).values():assert np.array_equal(T,np.eye(4))


def test_wrong_radial_explosion_hits_the_bore():
    a=fixture();p=AssemblyProcess('bad',[Move('bad','Pull sideways',('pin',),(8,0,0))])
    r=validate_process(a,p,linear_step_mm=.5)
    assert not r['passed'] and r['collisions'][0]['obstacle']=='sleeve'


def test_tools_are_checked_against_remaining_parts():
    a=fixture();tool=ToolAccess((0,0,6),(1,0,0),1.,20.,3.,10.)
    p=AssemblyProcess('blocked',[Move('release','Pull pin',('pin',),(0,0,14),tool=tool)])
    r=validate_process(a,p,linear_step_mm=2)
    assert not r['passed'] and r['tool_obstructions'][0]['part']=='sleeve'


def test_release_dependencies_fail_closed():
    with pytest.raises(ValueError,match='dependencies'):
        AssemblyProcess('bad',[Move('pull','Pull',('pin',),(0,0,14),requires=('release-screw',))]).bind(fixture())


def test_purchased_service_units_cannot_be_split_by_an_explosion():
    with pytest.raises(ValueError,match='retained service unit'):
        AssemblyProcess('bad',[Move('pull','Pull',('pin',),(0,0,14))],
                        retained_units={'cartridge':['pin','sleeve']}).bind(fixture())


def test_floor_is_an_obstacle():
    a=fixture();p=AssemblyProcess('bad',[Move('drop','Through floor',('pin',),(0,0,-20))])
    r=validate_process(a,p,linear_step_mm=2)
    assert r['floor_collisions'] and not r['passed']


def test_mesh_only_parts_are_reported_as_unchecked():
    a=fixture();a.parts[0]=replace(a.parts[0],cad=None)
    p=AssemblyProcess('unknown',[Move('pull','Pull pin',('pin',),(0,0,14))])
    r=validate_process(a,p,linear_step_mm=3)
    assert not r['passed'] and r['unchecked_mesh_parts']==['sleeve']


def test_service_motion_preserves_existing_home_transforms():
    a=fixture()
    home=np.eye(4);home[:3,3]=(13,7,4)
    a.motion_function=lambda part,t,e:home.copy()
    process=AssemblyProcess('translated',[Move('pull','Pull pin',('pin',),(0,0,14))]).bind(a)
    forward=process.assembly(a);reverse=process.assembly(a,reverse=True)
    for t in np.linspace(0,1,9):
        for part in a.parts:
            assert np.allclose(forward.pose(part,t),process.poses(t)[part.name])
            assert np.allclose(reverse.pose(part,1-t),forward.pose(part,t))
    assert np.array_equal(forward.pose(a.parts[0],1),home)
