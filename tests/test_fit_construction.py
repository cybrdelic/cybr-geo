"""Constructive nominal pin/bore fits in the actual example CAD path."""
from copy import deepcopy
import math
import sys

import pytest

from mechanism_lab.engineering_source import load_project
from mechanism_lab.fit_construction import PinBoreFit
from mechanism_lab.native_construction import NativeInterfaceRequirements


@pytest.mark.parametrize(('name', 'pin_mm', 'service_mm', 'variation_mm', 'reserve_mm', 'bore_mm'), [
    ('fourbar_lift', 5., .05, .01, .005, 5.15),
    ('iris09', 2., .025, .005, .0025, 2.075),
])
def test_fit_is_derived_before_native_geometry(name, pin_mm, service_mm, variation_mm, reserve_mm, bore_mm):
    project, _ = load_project(name)
    spec = project.specification
    fit = PinBoreFit.from_document(spec['pin_diameter_mm'], spec['pin_bore_fit'])
    assert fit.pin_diameter_mm == pin_mm
    assert fit.nominal_bore_diameter_mm == pytest.approx(bore_mm)
    assert fit.worst_declared_radial_gap_mm == pytest.approx(service_mm + reserve_mm)
    assert fit.pin_declared_radial_variation_mm == variation_mm
    assert fit.bore_declared_radial_variation_mm == variation_mm
    assert fit.design_reserve_radial_gap_mm == reserve_mm
    assert spec['diametral_clearance_mm'] == pytest.approx(bore_mm-pin_mm)
    assert fit.report()['process_capability_verified'] is False

    mechanism = sys.modules[project.construct.__module__]
    info = mechanism.layout(spec)
    link = info['link'] if name == 'fourbar_lift' else info['links'][0]
    pin = mechanism.cylinder(pin_mm/2, -link.thickness_mm/2, link.thickness_mm/2) if name == 'fourbar_lift' else (
        mechanism._cylinder(pin_mm/2, -link.thickness_mm/2, link.thickness_mm/2))
    rod_shape = link.native_shape()
    assert rod_shape.distance(pin) == pytest.approx(fit.nominal_radial_gap_mm, abs=1e-6)
    assert rod_shape.isValid()
    checks = NativeInterfaceRequirements({'scope': 'Focused constructed fit test'})
    rod_check = checks.clearance('rod_input_pin', rod_shape, pin,
        minimum_distance_mm=fit.required_service_radial_gap_mm)
    assert rod_check['minimum_distance_mm'] == pytest.approx(fit.nominal_radial_gap_mm, abs=1e-6)
    assert rod_check['required_minimum_distance_mm'] == fit.required_service_radial_gap_mm
    assert rod_check['pass']

    if name == 'iris09':
        bearing = mechanism.fixed_leaf_bearing(spec, info['count'])
        hinge_mm = spec['source_geometry']['leaf']['hinge_radius'] * spec['scene_unit_mm']
        fixed_pin = mechanism._cylinder(pin_mm/2, 22., 23.5, (hinge_mm, 0.))
        assert bearing.isValid()
        assert bearing.distance(fixed_pin) == pytest.approx(fit.nominal_radial_gap_mm, abs=1e-6)
        bearing_check = checks.clearance('fixed_bearing_leaf_pin', bearing, fixed_pin,
            minimum_distance_mm=fit.required_service_radial_gap_mm)
        assert bearing_check['minimum_distance_mm'] == pytest.approx(fit.nominal_radial_gap_mm, abs=1e-6)
        assert bearing_check['pass']
    assert checks.report()['passed']


@pytest.mark.parametrize(('name', 'too_large'), [('fourbar_lift', 2.), ('iris09', .3)])
def test_incompatible_authored_fit_rejects_before_solids(name, too_large):
    project, _ = load_project(name)
    mechanism = sys.modules[project.construct.__module__]
    changed = deepcopy(project.specification)
    changed['pin_bore_fit']['pin_declared_radial_variation_mm'] = too_large
    changed.pop('diametral_clearance_mm')
    with pytest.raises(ValueError, match='bores? violate'):
        mechanism.validate(changed)


@pytest.mark.parametrize('invalid', [0, -1, math.nan, math.inf, True, '0.01'])
def test_missing_or_invalid_declared_budget_rejects(invalid):
    design = dict(required_service_radial_gap_mm=.05,
                  pin_declared_radial_variation_mm=invalid,
                  bore_declared_radial_variation_mm=.01,
                  design_reserve_radial_gap_mm=.005)
    with pytest.raises(ValueError):
        PinBoreFit.from_document(5., design)
    del design['pin_declared_radial_variation_mm']
    with pytest.raises(ValueError):
        PinBoreFit.from_document(5., design)


@pytest.mark.parametrize('invalid', [0, -1, math.nan, math.inf, True, '0.005'])
def test_missing_or_invalid_positive_design_reserve_rejects(invalid):
    design = dict(required_service_radial_gap_mm=.05,
                  pin_declared_radial_variation_mm=.01,
                  bore_declared_radial_variation_mm=.01,
                  design_reserve_radial_gap_mm=invalid)
    with pytest.raises(ValueError):
        PinBoreFit.from_document(5., design)
    del design['design_reserve_radial_gap_mm']
    with pytest.raises(ValueError):
        PinBoreFit.from_document(5., design)


@pytest.mark.parametrize('name', ['fourbar_lift', 'iris09'])
def test_derived_clearance_cannot_be_overridden_independently(name):
    project, _ = load_project(name)
    mechanism = sys.modules[project.construct.__module__]
    changed = deepcopy(project.specification)
    changed['diametral_clearance_mm'] += .01
    with pytest.raises(ValueError, match='disagrees with its fit design source'):
        mechanism.validate(changed)
