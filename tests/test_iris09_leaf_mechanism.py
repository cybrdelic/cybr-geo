"""The mechanically derived IRIS crop is rigid and continuously envelope-safe."""
import math

import cadquery as cq
import numpy as np
import pytest

from examples.engineering.iris09.leaf_geometry import (
    build_leaf, swept_radius_mm, cut_fixed_shaft_keepouts,
)


@pytest.fixture(scope='module')
def leaf():
    return build_leaf()


def placed(shape, index, angle):
    azimuth = index*2*math.pi/9
    return shape.rotate((0,0,0),(0,0,1),-math.degrees(azimuth+angle)).translate(
        (43*math.cos(azimuth),-43*math.sin(azimuth),0))


def test_continuous_extremum_catches_between_endpoint_bulge(leaf):
    shape, report = leaf
    angle, radius = .45, 6.51
    x,y = radius*math.cos(angle),radius*math.sin(angle)
    assert all(math.hypot(43+x*math.cos(q)+y*math.sin(q),
                         -x*math.sin(q)+y*math.cos(q)) < 49.5 for q in (0,.9))
    assert swept_radius_mm((x,y),43) == pytest.approx(49.51)
    assert not shape.isInside((x,y,13-.008*y-.0325/2),1e-8)
    assert shape.isValid() and len(shape.Solids()) == 1
    assert 36.7 < report['fixed_volume_mm3'] < 36.8
    assert report['boundary_continuous_radius_max_mm'] <= 49.5+1e-6


def test_fixed_native_membership_matches_source_and_exact_swept_domain(leaf):
    shape, _ = leaf
    random = np.random.default_rng(916)
    for x,y in random.uniform([-30,-85],[30,20],(400,2)):
        source = math.hypot(x-34.5,y+8.5)<57.5 and math.hypot(x-12.5,y+60)<80
        expected = source and swept_radius_mm((x,y),43) < 49.5
        assert shape.isInside((x,y,13-.008*y-.0325/2),1e-8) == expected


@pytest.mark.parametrize('angle',[0.,.3,.6,.9])
def test_actual_brep_leaf_neighbors_do_not_intersect_at_declared_samples(leaf,angle):
    shape, report = leaf
    first = placed(shape,0,angle)
    for index in range(1,5):
        other = placed(shape,index,angle)
        assert first.intersect(other).Volume() < 1e-9
        assert first.distance(other) > .014
    assert first.Volume() == pytest.approx(report['fixed_volume_mm3'],rel=1e-10)


def test_leaf_vertical_bounds_leave_space_below_proposed_bearing_deck(leaf):
    shape, _ = leaf
    bounds = shape.BoundingBox()
    assert bounds.zmax < 13.7
    assert 15.-bounds.zmax > 1.3
    assert shape.isInside((0.,0.,13.-.0325/2),1e-8)


def test_hub_keepouts_cut_actual_interference_and_retain_one_attached_leaf(leaf):
    original, _ = leaf
    shape, report = cut_fixed_shaft_keepouts(original,shaft_radius_mm=1.8,clearance_mm=.1)
    assert shape.isValid() and len(shape.Solids()) == 1
    assert report['connected_solids'] == 1 and report['own_pivot_material_retained']
    assert 35.6 < shape.Volume() < 35.8
    removed_interference = 0.
    for angle in (0.,.137,.371234,.6,.9):
        actual, uncut = placed(shape,0,angle), placed(original,0,angle)
        for index in range(1,9):
            phi=index*2*math.pi/9
            shaft=cq.Solid.makeCylinder(1.8,25,(43*math.cos(phi),-43*math.sin(phi),10))
            removed_interference += uncut.intersect(shaft).Volume()
            assert actual.intersect(shaft).Volume() < 1e-9
            assert actual.distance(shaft) >= .1-2e-6
    assert removed_interference > .01
    # The nominal hub joins real material around the own pivot; it is not a
    # disconnected optical foil riding on a prescribed pose.
    hub=cq.Solid.makeCylinder(1.8,.15,(0,0,12.95))
    connected=shape.fuse(hub).clean()
    assert connected.isValid() and len(connected.Solids()) == 1


@pytest.mark.parametrize('interval',[(0,0),(.9,0),(0,math.pi),(0,float('nan'))])
def test_invalid_ranges_cannot_advertise_an_envelope(interval):
    with pytest.raises(ValueError):
        build_leaf(hinge_range_rad=interval)
