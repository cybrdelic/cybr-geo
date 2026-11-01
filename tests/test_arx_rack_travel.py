"""The authored ARX rack must cover the whole actual opposed MJCF slide."""
from copy import deepcopy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from examples.engineering.arx_belt.rack_contract import (SOURCE_MODEL,
    arx_rack_report, rack_travel_from_source)
from mechanism_lab.rack_travel import RackPinionTravel


SPEC = json.loads(Path('examples/engineering/arx_belt/data/spec.json').read_text(encoding='utf-8'))


def test_source_rack_pitch_placement_and_every_stop_margin():
    travel = rack_travel_from_source(SPEC)
    report = travel.report()
    pitch = .3 * math.pi
    assert report['pitch_radius_m'] == pytest.approx(.003)
    assert report['toothed_length_mm'] == pytest.approx(50*pitch)
    assert report['pitch_line_flank_inset_mm'] == pytest.approx(pitch/4)
    assert report['right_rack_x_offset_mm'] == pytest.approx(2-50*pitch/2)
    assert report['left_rack_x_offset_mm'] == pytest.approx(50*pitch/2-2)
    assert report['slide_range_mm'] == [10., 40.]
    assert report['minimum_edge_margin_mm'] == pytest.approx(49.75*pitch-42.)
    assert report['geometry_margin_mm'] == pytest.approx(47.75*pitch-42.)
    assert report['full_nominal_travel_supported'] is True
    assert json.loads(json.dumps(report, allow_nan=False)) == report
    frame = arx_rack_report(travel)['arx_p5_mating_frame']
    assert frame['pitch_radius_mm'] == pytest.approx(3.)
    assert frame['right_local_y_mm'] == pytest.approx(204.5-219.2)
    assert frame['left_preoffset_y_mm'] == pytest.approx(198.5)


def test_rack_length_and_placement_edits_cannot_remove_nominal_engagement():
    for changes in ({'rack_teeth':45}, {'toothed_end_local_x_mm':8.},
                    {'required_edge_pitch_count':5.}, {'design_reserve_pitch_count':5.}):
        document = dict(SPEC['gripper_rack_travel'], **changes)
        with pytest.raises(ValueError, match='full slide travel'):
            RackPinionTravel.from_document(document, (.010, .040))
    # With e=5.2, the rack blank has 1.9239 mm of edge margin (old check
    # would pass 1.885 mm). The actual pitch-line flank is inset p/4, leaving
    # only 1.6883 mm; the constructive contract must reject that geometry.
    with pytest.raises(ValueError, match='full slide travel'):
        RackPinionTravel.from_document(dict(SPEC['gripper_rack_travel'],
                                            toothed_end_local_x_mm=5.2), (.010, .040))
    with pytest.raises(ValueError, match='full slide travel'):
        RackPinionTravel.from_document(SPEC['gripper_rack_travel'], (.010, .044))


def test_actual_mjcf_slide_and_tendon_must_match_constructed_rack():
    root = ET.fromstring(SOURCE_MODEL.read_bytes())
    root.find(".//joint[@name='jaw_left']").set('range', '.010 .044')
    root.find(".//joint[@name='jaw_right']").set('range', '.010 .044')
    with pytest.raises(ValueError, match='full slide travel'):
        rack_travel_from_source(SPEC, model_root=root)
    root = ET.fromstring(SOURCE_MODEL.read_bytes())
    root.find(".//tendon/fixed[@name='rack_pinion_angle']/joint").set('coef', '150')
    with pytest.raises(ValueError, match='pitch conversion'):
        rack_travel_from_source(SPEC, model_root=root)
    root = ET.fromstring(SOURCE_MODEL.read_bytes())
    root.find(".//joint[@name='jaw_right']").set('axis', '1 0 0')
    with pytest.raises(ValueError, match='slide axis'):
        rack_travel_from_source(SPEC, model_root=root)


def test_pinion_module_change_needs_matching_physical_tendon():
    changed = deepcopy(SPEC)
    changed['gripper_rack_travel']['module_mm'] = .31
    with pytest.raises(ValueError, match='support redesign'):
        rack_travel_from_source(changed)
    changed = deepcopy(SPEC)
    changed['gripper_rack_travel']['rack_teeth'] = 52
    with pytest.raises(ValueError, match='support redesign'):
        rack_travel_from_source(changed)


def test_unrepresentable_source_slide_limit_rejects_cleanly():
    with pytest.raises(ValueError, match='representable'):
        RackPinionTravel.from_document(SPEC['gripper_rack_travel'], (.010, 10**400))


@pytest.mark.parametrize('change', [
    {'module_mm':0.}, {'module_mm':float('nan')},
    {'pinion_teeth':True}, {'rack_teeth':2},
    {'rack_teeth':10**400},
    {'toothed_end_local_x_mm':float('inf')},
    {'toothed_end_local_x_mm':10**400},
    {'required_edge_pitch_count':0.}, {'design_reserve_pitch_count':0.},
])
def test_invalid_authoring_cannot_enter_rack_construction(change):
    with pytest.raises(ValueError):
        RackPinionTravel.from_document(dict(SPEC['gripper_rack_travel'], **change), (.010, .040))
