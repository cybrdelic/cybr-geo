"""Geometry evidence for ATLAS. No assertion of force or manufacturing fitness."""
import math

import cadquery as cq
import numpy as np
import pytest

from mechanism_lab.advanced_geometry import helical_sweep
from mechanism_lab.core import pose_cad
from mechanism_lab.models.atlas_fixture import SPEC,JAW_ANGLES,build,pose


@pytest.fixture(scope='module')
def assembly():
    return build()


def test_continuous_helix_profile_is_on_centerline():
    shape=helical_sweep(x0=2,length=12,helix_radius=3,pitch=4,section_radius=.5)
    assert shape.isValid() and len(shape.Solids())==1
    expected=math.pi*.5**2*math.hypot(12,2*math.pi*3*3)
    assert shape.Volume()==pytest.approx(expected,rel=.003)
    # Every sampled face point lies near the requested 3 mm radial centerline.
    v,_=shape.tessellate(.06,.18)
    radii=np.hypot([p.y for p in v],[p.z for p in v])
    assert radii.min()>2.48 and radii.max()<3.52


def test_named_parts_and_native_geometry_are_complete(assembly):
    assert len({p.name for p in assembly.parts})==len(assembly.parts)
    assert all(p.cad.isValid() for p in assembly.parts if p.cad is not None)
    # Only field surfaces and fine conductors intentionally have no analytic BREP.
    mesh_only={p.name for p in assembly.parts if p.cad is None}
    assert mesh_only=={'AT_081_Implicit_gyroid_insert',*[f'AT_093_Conductor_{i+1}' for i in range(3)]}


def test_actual_cam_brep_contains_the_follower_clearance(assembly):
    cam=next(p for p in assembly.parts if p.name=='AT_050_Three_spiral_cam')
    # Sample follower circumference against the real cut BREP at many poses,
    # independently of the interpolated slot boundary representation.
    for t in np.linspace(0,SPEC.cycle_seconds/2,13):
        world=pose_cad(cam.cad,pose(cam,float(t),0))
        for i,a in enumerate(JAW_ANGLES):
            pin=next(p for p in assembly.parts if p.name==f'AT_073_Cam_follower_{i+1}')
            center=np.array([103.5,36*math.cos(a),36*math.sin(a)])+pose(pin,float(t),0)[:3,3]
            for phi in np.linspace(0,math.tau,32,endpoint=False):
                q=center+np.array([0,3.01*math.cos(phi),3.01*math.sin(phi)])
                assert not world.isInside(tuple(q),1e-5), (t,i,phi)


def test_jaws_remain_self_centered_and_guided(assembly):
    face=next(p for p in assembly.parts if p.name=='AT_060_Precision_guide_face')
    for t in (0,1,2,3,4):
        centers=[]
        for i,a in enumerate(JAW_ANGLES):
            slider=next(p for p in assembly.parts if p.name==f'AT_070_Jaw_slide_{i+1}')
            moved=pose_cad(slider.cad,pose(slider,t,0))
            assert moved.intersect(face.cad).Volume()<1e-5
            c=np.array([0,36*math.cos(a),36*math.sin(a)])+pose(slider,t,0)[:3,3]
            centers.append(c)
        assert np.linalg.norm(np.mean(centers,axis=0))<1e-9
        assert np.ptp([np.linalg.norm(c) for c in centers])<1e-9


def test_stationary_posts_clear_rotating_handwheel_and_cam(assembly):
    rotating=[p for p in assembly.parts if p.name in ('AT_040_Revolved_handwheel','AT_050_Three_spiral_cam')]
    posts=[p for p in assembly.parts if p.name.startswith('AT_062_')]
    for t in (0,2,4):
        for p in rotating:
            sh=pose_cad(p.cad,pose(p,t,0))
            for post in posts:
                assert sh.intersect(post.cad).Volume()<1e-5


def test_mating_involute_profiles_clear_over_the_complete_stroke():
    from cybrgeo.features import involute_profile
    from shapely.geometry import Polygon
    from shapely import affinity
    def polygon(teeth,center,phase):
        p=involute_profile(teeth,2.5,.12,12)
        return Polygon(np.c_[p[:,0]*np.cos(p[:,1]+phase)+center[0],
                             p[:,0]*np.sin(p[:,1]+phase)+center[1]])
    ring=polygon(44,(0,0),0)
    pinion=polygon(16,(75,0),math.pi/16)
    assert ring.is_valid and pinion.is_valid
    for theta in np.linspace(0,math.pi/4,181):
        a=affinity.rotate(ring,-theta,origin=(0,0),use_radians=True)
        b=affinity.rotate(pinion,theta*44/16,origin=(75,0),use_radians=True)
        assert a.intersection(b).area<1e-8


def test_gear_mount_has_a_real_clear_seat_and_keyways(assembly):
    parts={p.name:p for p in assembly.parts}
    drive=parts['AT_032_Windowed_drive_sleeve'].cad
    gear=parts['AT_033_Involute_input_gear'].cad
    key=parts['AT_033a_Drive_gear_key'].cad
    assert len(drive.Solids())==len(gear.Solids())==1
    assert drive.intersect(gear).Volume()<1e-6
    assert key.intersect(drive).Volume()<1e-6
    assert key.intersect(gear).Volume()<1e-6
