"""One native distance requirement shared by mechanism and manufacturing clients."""
import json

import cadquery as cq
import pytest

from mechanism_lab.native_construction import NativeConstruction, NativeInterfaceRequirements


def test_nonintersecting_parts_can_fail_required_clearance_before_export(tmp_path):
    first = cq.Workplane('XY').box(10, 10, 10)
    second = first.translate((10.15, 0, 0))
    requirements = NativeInterfaceRequirements({'source': 'authored 0.2 mm gap'})
    row = requirements.clearance('undersized_gap', first, second,
                                 minimum_distance_mm=.2, kernel_allowance_mm=1e-8)
    assert row['intersection_volume_mm3'] == pytest.approx(0., abs=1e-10)
    assert row['minimum_distance_mm'] == pytest.approx(.15)
    assert not row['clearance_passed'] and not row['pass']
    assert row['required_minimum_distance_mm'] == row['required_minimum_clearance_mm'] == .2
    build = NativeConstruction(tmp_path, {'alloy': 2700})
    with pytest.raises(ValueError, match='undersized_gap'):
        build.finish('Incomplete nominal interface', native_requirements=requirements)
    assert not (tmp_path / 'native-build.json').exists()
    report = json.loads((tmp_path / 'native-interface-requirements.json').read_text())
    assert not report['passed']
    assert report['checks'][0]['distance_kernel_allowance_mm'] == 1e-8


def test_numerical_allowance_is_explicit_and_does_not_replace_volume_requirement():
    first = cq.Workplane('XY').box(10, 10, 10)
    second = first.translate((10.2, 0, 0))
    strict = NativeInterfaceRequirements().clearance('strict', first, second,
                                                    minimum_distance_mm=.2000005)
    tolerant = NativeInterfaceRequirements().clearance('kernel_budget', first, second,
        minimum_distance_mm=.2000005, kernel_allowance_mm=1e-6)
    assert not strict['pass']
    assert tolerant['pass'] and tolerant['clearance_passed']
    assert tolerant['minimum_distance_mm'] == strict['minimum_distance_mm']
    # The distance criterion alone cannot make overlapping material acceptable.
    overlapping = NativeInterfaceRequirements().clearance('overlap', first, first.translate((9, 0, 0)),
                                                          minimum_distance_mm=0.)
    assert overlapping['clearance_passed']
    assert overlapping['intersection_volume_mm3'] == pytest.approx(100.)
    assert not overlapping['pass']


@pytest.mark.parametrize('keyword', ['minimum_distance_mm', 'kernel_allowance_mm'])
@pytest.mark.parametrize('value', [True, -.01, float('inf'), float('nan')])
def test_invalid_distance_requirement_does_not_append_partial_check(keyword, value):
    requirements = NativeInterfaceRequirements()
    arguments = dict(minimum_distance_mm=.2, kernel_allowance_mm=1e-8)
    arguments[keyword] = value
    with pytest.raises(ValueError, match='finite and nonnegative'):
        requirements.clearance('invalid', None, None, **arguments)
    assert requirements.checks == []

