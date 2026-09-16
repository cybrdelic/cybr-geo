import math
import numpy as np

from mechanism_lab.gears import (
    PlanetarySpec,
    external_profile,
    internal_space_profile,
    external_spur_gear,
    internal_ring_gear,
    planetary_validation_report,
)


def test_default_pitch_geometry_and_ratio_are_exact():
    s=PlanetarySpec()
    s.validate()
    assert s.ring_teeth == s.sun_teeth + 2*s.planet_teeth
    assert s.sun_pitch_radius == 7.2
    assert s.planet_pitch_radius == 10.8
    assert s.ring_pitch_radius == 28.8
    assert s.planet_center_radius == 18.0
    assert s.reduction == 5.0
    report=planetary_validation_report(s)
    assert report['pitch_geometry_mm']['sun_planet_tangency_error'] < 1e-12
    assert report['pitch_geometry_mm']['ring_planet_tangency_error'] < 1e-12
    assert report['topology']['equal_spacing_phase_condition']


def test_external_profile_hits_standard_addendum_and_root():
    s=PlanetarySpec()
    p=external_profile(s.sun_teeth,s.module,s.pressure_angle_deg,s.backlash_mm)
    r=p[:,0]
    assert np.isclose(r.max(), s.sun_pitch_radius+s.module, atol=1e-9)
    assert np.isclose(r.min(), s.sun_pitch_radius-1.25*s.module, atol=1e-9)
    assert len(p) > s.sun_teeth*20


def test_internal_space_profile_has_inward_tip_and_outward_root():
    s=PlanetarySpec()
    p=internal_space_profile(s.ring_teeth,s.module,s.pressure_angle_deg,s.backlash_mm)
    r=p[:,0]
    assert np.isclose(r.min(), s.ring_pitch_radius-s.module, atol=1e-9)
    assert np.isclose(r.max(), s.ring_pitch_radius+1.25*s.module, atol=1e-9)


def test_cad_solids_are_valid_and_ring_has_real_radial_wall():
    s=PlanetarySpec()
    sun=external_spur_gear(s.sun_teeth,s.module,s.face_width_mm,origin=(0,0,0),backlash=s.backlash_mm)
    ring=internal_ring_gear(s.ring_teeth,s.module,s.face_width_mm,outer_radius=35.0,origin=(0,0,0),backlash=s.backlash_mm)
    assert sun.isValid()
    assert ring.isValid()
    assert ring.Volume() > sun.Volume()


def test_invalid_equal_spacing_and_tooth_relation_fail_closed():
    import pytest
    with pytest.raises(ValueError,match='ring_teeth'):
        PlanetarySpec(ring_teeth=71).validate()
    with pytest.raises(ValueError,match='divisible'):
        PlanetarySpec(planets=4).validate()
