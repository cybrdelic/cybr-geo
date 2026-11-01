"""Focused construction invariants for shared nominal hardware recipes."""
from dataclasses import replace
import pytest
from mechanism_lab.fasteners import SleevedFastenerStack

@pytest.mark.parametrize('spec',[
 SleevedFastenerStack(2,12,1.5,1.05,0,8,2.5,.3,4,1.6,-1),
 SleevedFastenerStack(4,70,4,2.15,-30,30,7,1,7,3.2),
 SleevedFastenerStack(3,40,2,1.6,-19.5,16,3.5,.5,5.5,2.4),
 SleevedFastenerStack(2.5,8,1.75,1.3,-19,-14,3,.5,5,2),
])
def test_complete_seated_native_stack(spec):
 s=spec.native_shapes()
 assert set(s)=={'sleeve','lower_washer','upper_washer','bolt','nut'}
 assert all(x.isValid() and len(x.Solids())==1 and x.Volume()>0 for x in s.values())
 assert s['sleeve'].distance(s['lower_washer'])<1e-8
 assert s['sleeve'].distance(s['upper_washer'])<1e-8
 assert spec.engagement_mm==spec.nut_height_mm

@pytest.mark.parametrize('change',[
 {'length_mm':7.9},{'bore_radius_mm':1.25},{'sleeve_outer_radius_mm':1.3},
 {'washer_radius_mm':1.5},{'direction':0},{'high_mm':-19},
 {'nut_across_flats_mm':2},{'low_mm':float('nan')},{'length_mm':True},
])
def test_inconsistent_stack_cannot_be_constructed(change):
 spec=SleevedFastenerStack(2.5,8,1.75,1.3,-19,-14,3,.5,5,2)
 with pytest.raises(ValueError):replace(spec,**change)
