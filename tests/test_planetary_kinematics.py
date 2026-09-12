import math
from types import SimpleNamespace
import numpy as np

from mechanism_lab.gears import PlanetarySpec, planetary_angles, planet_centers, mesh_residuals
from mechanism_lab.gears.planetary import planet_initial_phases
from mechanism_lab.models.planetary_actuator import pose


def test_willis_ratio_and_planet_speed():
    s=PlanetarySpec(input_speed_rps=.45)
    assert math.isclose(s.carrier_speed_rps,.09,rel_tol=0,abs_tol=1e-12)
    assert math.isclose(s.planet_absolute_speed_rps,-.15,rel_tol=0,abs_tol=1e-12)
    for t in (0,.1,.75,1.4):
        q=planetary_angles(s,t)
        assert math.isclose(q['carrier'],q['sun']/5.0,rel_tol=0,abs_tol=1e-12)


def test_external_and_internal_mesh_phase_residuals_stay_zero():
    s=PlanetarySpec()
    for t in np.linspace(0,3.0,17):
        residuals=mesh_residuals(s,float(t))
        assert max(abs(v) for v in residuals['sun_planet']) < 2e-12
        assert max(abs(v) for v in residuals['ring_planet']) < 2e-12


def test_planet_centers_remain_equal_radius_and_spacing():
    s=PlanetarySpec()
    q=planetary_angles(s,1.137)
    centers=planet_centers(s,q['carrier'])
    angles=[]
    for y,z in centers:
        assert math.isclose(math.hypot(y,z),s.planet_center_radius,rel_tol=0,abs_tol=1e-12)
        angles.append(math.atan2(z,y)%(2*math.pi))
    angles=sorted(angles)
    gaps=[(angles[(i+1)%3]-angles[i])%(2*math.pi) for i in range(3)]
    assert all(math.isclose(g,2*math.pi/3,abs_tol=1e-12) for g in gaps)


def test_model_planet_pose_moves_bind_center_to_exact_orbit():
    s=PlanetarySpec()
    y,z=planet_centers(s,0)[1]
    dummy=SimpleNamespace(motion='planet_1',explode=np.zeros(3))
    t=.83
    T=pose(dummy,t,0,s)
    p=np.array([29.,y,z,1.])
    moved=T@p
    q=planetary_angles(s,t)
    expected=planet_centers(s,q['carrier'])[1]
    assert np.allclose(moved[1:3],expected,atol=1e-10)
    assert np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-12)


def test_initial_planet_phases_are_not_arbitrary_visual_tuning():
    s=PlanetarySpec()
    phases=planet_initial_phases(s)
    residuals=mesh_residuals(s,0)
    assert len(set(round(v,12) for v in phases)) == 3
    assert max(abs(v) for v in residuals['sun_planet']) < 1e-12
    assert max(abs(v) for v in residuals['ring_planet']) < 1e-12
