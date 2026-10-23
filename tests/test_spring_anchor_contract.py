"""Small causal/geometry checks for source-derived spring-anchor construction."""
from copy import deepcopy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import cadquery as cq
import pytest

from mechanism_lab.spring_anchor import tension_anchor_demand, verify_native_anchor
from mechanism_lab.engineering_source import _CONSTRUCTION_SERVICES
from examples.engineering.arx_belt.spring_anchor_contract import (
    arx_spring_anchor_plan, verify_arx_anchor_receipt,
    verify_arx_spring_joint_frames,
)


DATA = Path(__file__).resolve().parents[1] / 'examples' / 'engineering' / 'arx_belt' / 'data'


def source():
    return (json.loads((DATA / 'spec.json').read_text(encoding='utf-8')),
            json.loads((DATA / 'folded-transmissions.json').read_text(encoding='utf-8')))


def test_exact_interior_peak_and_undefined_coincident_eyes():
    spring = dict(parent_anchor_m=[2, 0, 0], child_anchor_m=[1, 0, 0],
                  spring_count=1, free_eye_length_m=0., initial_tension_N=0.,
                  stiffness_N_m=1.)
    demand = tension_anchor_demand(spring, [0, math.tau], pin_diameter_mm=4.)
    assert demand['peak_joint_angle_rad'] == pytest.approx(math.pi)
    assert demand['peak_eye_length_m'] == pytest.approx(3.)
    assert demand['required_capacity_per_separate_pin_N'] == pytest.approx(3.)
    assert demand['pin_clevis_capacity_verified'] is False
    # A midpoint singularity cannot be hidden by regular endpoint states.
    spring['parent_anchor_m'] = [1, 0, 0]
    with pytest.raises(ValueError, match='Coincident spring anchors'):
        tension_anchor_demand(spring, [-1., 1.], pin_diameter_mm=4.)


def test_placed_native_pin_requires_actual_matching_bore():
    pin = cq.Workplane('XY').circle(2).extrude(16).translate((0, 0, -8))
    clevis = cq.Workplane('XY').circle(6).circle(2.1).extrude(4).translate((0, 0, -2))
    kwargs = dict(pin_diameter_mm=4., pin_length_mm=16., nominal_bore_diameter_mm=4.2)
    binding = verify_native_anchor(pin, clevis, [0, 0, 0], **kwargs)
    assert binding['measured_nominal_radial_gap_mm'] == pytest.approx(.1)
    assert binding['intersection_volume_mm3'] == pytest.approx(0.)
    with pytest.raises(ValueError, match='position or dimensions'):
        verify_native_anchor(pin.translate((.2, 0, 0)), clevis, [0, 0, 0], **kwargs)
    with pytest.raises(ValueError, match='position or dimensions'):
        verify_native_anchor(cq.Workplane('XY').circle(2).extrude(14).translate((0, 0, -7)),
                             clevis, [0, 0, 0], **kwargs)
    undersize_bore = cq.Workplane('XY').circle(6).circle(2.05).extrude(4).translate((0, 0, -2))
    with pytest.raises(ValueError, match='does not mate'):
        verify_native_anchor(pin, undersize_bore, [0, 0, 0], **kwargs)
    with pytest.raises(ValueError, match='does not mate'):
        verify_native_anchor(pin, clevis.translate((.4, 0, 0)), [0, 0, 0], **kwargs)
    with pytest.raises(ValueError, match='does not mate'):
        verify_native_anchor(pin, cq.Workplane('XY').circle(6).extrude(4).translate((0, 0, -2)),
                             [0, 0, 0], **kwargs)
    # The inherited ARX moving clevis bore ended at the exact pin end face:
    # zero-volume contact still defeats nominal clearance. Extending the cut
    # one millimetre at both ends removes that specific defect.
    blank = cq.Workplane('XY').circle(6).extrude(24).translate((0, 0, -12))
    original_bore = cq.Workplane('XY').circle(2.1).extrude(16).translate((0, 0, -8))
    with pytest.raises(ValueError, match='does not mate'):
        verify_native_anchor(pin, blank.cut(original_bore), [0, 0, 0], **kwargs)
    relieved_bore = cq.Workplane('XY').circle(2.1).extrude(18).translate((0, 0, -9))
    assert verify_native_anchor(pin, blank.cut(relieved_bore), [0, 0, 0],
                                **kwargs)['measured_nominal_radial_gap_mm'] == pytest.approx(.1)


def test_pinned_arx_plan_binds_all_eight_pins_to_full_range_demand():
    spec, transmissions = source()
    plan = arx_spring_anchor_plan(spec, transmissions)
    assert plan['nominal_pin_diameter_mm'] == 4.
    assert plan['nominal_pin_length_mm'] == 16.
    assert plan['nominal_bore_end_relief_mm'] == 1.
    shoulder, elbow = plan['groups']
    assert shoulder['demand']['peak_joint_angle_rad'] == pytest.approx(.95)
    assert shoulder['demand']['required_capacity_per_separate_pin_N'] == pytest.approx(230.77027076724193)
    assert elbow['demand']['peak_joint_angle_rad'] == pytest.approx(1.85)
    assert elbow['demand']['required_capacity_per_separate_pin_N'] == pytest.approx(88.17746887670654)
    assert [(row['body'], row['body_local_anchor_mm']) for row in shoulder['anchors']] == [
        ('turret', [-65., 104., 60.]), ('upper', [0., 70., 60.]),
        ('turret', [-65., 104., 84.]), ('upper', [0., 70., 84.])]
    assert [(row['body'], row['body_local_anchor_mm']) for row in elbow['anchors']] == [
        ('upper', [-30., 190., -55.]), ('fore', [0., -45., -55.]),
        ('upper', [-30., 190., -79.]), ('fore', [0., -45., -79.])]
    assert all(not group['demand']['pin_clevis_capacity_verified'] for group in plan['groups'])
    assert {'spring_anchor.py', 'actuation.py'} <= set(_CONSTRUCTION_SERVICES)
    changed = deepcopy(spec)
    changed['counterbalance']['reference_force_each_N'] += 1
    with pytest.raises(ValueError, match='reference disagrees'):
        arx_spring_anchor_plan(changed, transmissions)


def test_named_native_receipt_rejects_missing_stale_or_unqualified_bindings():
    spec, transmissions = source()
    plan = arx_spring_anchor_plan(spec, transmissions)
    anchors = [row for group in plan['groups'] for row in group['anchors']]
    bindings = []
    placements = []
    placed_clevises = set()
    for index, anchor in enumerate(anchors):
        body, name = anchor['body'], f'spring_eye_pin_{index}'
        clevis_part = anchor['clevis_part']
        clevis_name = clevis_part+'_0'
        bindings.append(dict(anchor=anchor['name'], body=body, pin_native_part=name,
            clevis_native_part=clevis_part, clevis_placed_part=clevis_name,
            required_capacity_N=anchor['required_capacity_N'],
            body_local_anchor_mm=anchor['body_local_anchor_mm'],
            pin_diameter_mm=4., pin_length_mm=16., clevis_bore_diameter_mm=4.2,
            measured_nominal_radial_gap_mm=.1, intersection_volume_mm3=0.))
        placements.append(dict(body=body, name=name, material='steel',
                               step_file=f'cad/placed/{body}__{name}.step'))
        clevis_key = (body, clevis_name)
        if clevis_key not in placed_clevises:
            placements.append(dict(body=body, name=clevis_name, material='aluminum',
                                   step_file=f'cad/placed/{body}__{clevis_name}.step'))
            placed_clevises.add(clevis_key)
    receipt = dict(schema='cybrgeo-native-spring-anchor-load-path/1',
                   plan=plan, native_anchor_bindings=bindings,
                   tested_pin_clevis_capacity_available=False)
    placed = dict(units='mm', records=placements)
    parts = {'spring_eye_pin': {'material': 'steel'}}
    parts.update({row['clevis_part']: {'material': 'aluminum'} for row in anchors})
    assert verify_arx_anchor_receipt(spec, transmissions, receipt, placed, parts) == plan
    for field, bad in [('required_capacity_N', 1.), ('clevis_native_part', 'wrong'),
                       ('measured_nominal_radial_gap_mm', .2), ('pin_length_mm', 14.)]:
        altered = deepcopy(receipt)
        altered['native_anchor_bindings'][0][field] = bad
        with pytest.raises(ValueError, match='geometry or load path'):
            verify_arx_anchor_receipt(spec, transmissions, altered, placed, parts)
    altered = deepcopy(receipt)
    altered['native_anchor_bindings'].pop()
    with pytest.raises(ValueError, match='eight distinct'):
        verify_arx_anchor_receipt(spec, transmissions, altered, placed, parts)
    altered = deepcopy(receipt)
    altered['tested_pin_clevis_capacity_available'] = True
    with pytest.raises(ValueError, match='source or construction'):
        verify_arx_anchor_receipt(spec, transmissions, altered, placed, parts)
    changed_transmissions = deepcopy(transmissions)
    next(row for row in changed_transmissions if row['joint'] == 'J2')['origin_mm'][1] += 1
    with pytest.raises(ValueError, match='source or construction'):
        verify_arx_anchor_receipt(spec, changed_transmissions, receipt, placed, parts)
    changed_placed = deepcopy(placed)
    changed_placed['records'][0]['material'] = 'polymer'
    with pytest.raises(ValueError, match='geometry or load path'):
        verify_arx_anchor_receipt(spec, transmissions, receipt, changed_placed, parts)
    clevis_label = bindings[0]['clevis_placed_part']
    missing_clevis = deepcopy(placed)
    missing_clevis['records'] = [record for record in missing_clevis['records']
                                 if record['name'] != clevis_label]
    with pytest.raises(ValueError, match='geometry or load path'):
        verify_arx_anchor_receipt(spec, transmissions, receipt, missing_clevis, parts)
    wrong_body = deepcopy(placed)
    next(record for record in wrong_body['records']
         if record['name'] == clevis_label)['body'] = 'fore'
    with pytest.raises(ValueError, match='geometry or load path'):
        verify_arx_anchor_receipt(spec, transmissions, receipt, wrong_body, parts)
    wrong_clevis_label = deepcopy(receipt)
    wrong_clevis_label['native_anchor_bindings'][0]['clevis_placed_part'] = 'shoulder_spring_fixed_clevis_999'
    with pytest.raises(ValueError, match='geometry or load path'):
        verify_arx_anchor_receipt(spec, transmissions, wrong_clevis_label, placed, parts)


def test_source_derived_pivots_reach_the_actual_simulated_hinges():
    spec, transmissions = source()
    plan = arx_spring_anchor_plan(spec, transmissions)
    model = ET.parse(DATA / 'P2-baseline.xml').getroot()
    verify_arx_spring_joint_frames(model, plan)
    model.find(".//body[@name='upper']").set('pos', '0 0.145 0')
    with pytest.raises(ValueError, match='hinge frame'):
        verify_arx_spring_joint_frames(model, plan)
    model = ET.parse(DATA / 'P2-baseline.xml').getroot()
    model.find(".//joint[@name='j3']").set('axis', '0 0 1')
    with pytest.raises(ValueError, match='hinge frame'):
        verify_arx_spring_joint_frames(model, plan)
