"""Regression: the former 72T profile crossed neighbouring root transitions."""
import numpy as np
import pytest
from shapely.geometry import Polygon
from cybrgeo.features import involute_profile, involute_spur_gear


@pytest.mark.parametrize('teeth',[12,20,48,72,100,144])
def test_involute_root_transitions_do_not_self_intersect(teeth):
    profile=involute_profile(teeth,1.,backlash=.055,nflank=12)
    xy=np.column_stack((profile[:,0]*np.cos(profile[:,1]),profile[:,0]*np.sin(profile[:,1])))
    assert Polygon(xy).is_valid
    assert Polygon(xy).area>0


def test_72_tooth_gear_is_a_valid_closed_analytic_solid():
    gear=involute_spur_gear(72,1.,10.,bore=28.2,backlash=.055).val()
    assert gear.isValid()
    assert len(gear.Solids())==1
    assert gear.Volume()>30000
