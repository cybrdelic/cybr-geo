"""Lightweight source-bound preflight for an unbuilt passive gripper brake."""
from copy import deepcopy
import json
import math

import pytest

from examples.engineering.arx_belt.gripper_retention import (
    CANDIDATE_PARTS, CANDIDATE_REVISION, DATA, candidate_recipe,
    _capture_lip_recipe, fault_contact_eligibility, proposal,
    require_nominal_brake_placement)
from examples.engineering.arx_belt.model import compile_model, control_source


def test_current_pinion_cannot_mate_the_authored_small_brake_hub():
    design = proposal()
    assert design['vendor_envelope']['outer_diameter_mm'] == 39.5
    assert design['vendor_envelope']['holding_torque_static_Nm'] == 2.2
    assert design['source_upper_shaft_diameter_mm'] == 3.
    assert design['authored_hub_bore_diameter_mm'] == 8.02
    assert design['source_direct_hub_radial_gap_mm'] == pytest.approx(2.51)
    assert design['source_direct_hub_seat_valid'] is False
    with pytest.raises(ValueError, match='Shaft/hub nominal radial gap'):
        require_nominal_brake_placement(3., design['minimum_authored_outboard_inner_face_z_mm'],
                                        construction_input=design)


def test_outboard_survey_gap_cannot_spend_service_variation_reserve():
    design = proposal()
    assert design['surveyed_outboard_nominal_gap_mm'] == 1.5
    assert design['minimum_authored_outboard_inner_face_z_mm'] == 21.25
    with pytest.raises(ValueError, match='service gap, variation or design reserve'):
        require_nominal_brake_placement(8., 20.5, construction_input=design)
    with pytest.raises(ValueError, match='service gap, variation or design reserve'):
        require_nominal_brake_placement(8., 21.249, construction_input=design)


def test_derived_nominal_interface_remains_diagnostic_only():
    design = proposal()
    placed = require_nominal_brake_placement(
        design['proposed_upper_shaft_diameter_mm'],
        design['minimum_authored_outboard_inner_face_z_mm'],
        construction_input=design)
    assert placed['shaft_hub_seat']['nominal_radial_gap_mm'] == pytest.approx(.01)
    assert placed['nominal_puck_to_stator_gap_mm'] == pytest.approx(2.25)
    assert placed['outboard_outer_face_z_mm'] == pytest.approx(46.95)
    assert not placed['native_solids_verified']
    assert not placed['motor_off_payload_retained']
    assert not placed['manufacturing_qualified']
    assert design['added_brake_mass_fraction_of_archived_tool'] > .6
    assert design['full_brake_rating_equivalent_force_per_jaw_N'] > 300.
    assert design['maximum_model_jaw_opening_during_delay_and_ramp_mm'] == pytest.approx(.96)


def test_vendor_source_or_authored_design_cannot_drift_independently():
    spec = json.loads((DATA/'spec.json').read_text(encoding='utf-8'))
    changed = deepcopy(spec)
    changed['brakes']['small']['OD_mm'] = 62.5
    with pytest.raises(ValueError, match='small brake OD_mm changed'):
        proposal(specification=changed)
    document = json.loads((DATA/'gripper-retention-proposal.json').read_text(encoding='utf-8'))
    changed = deepcopy(document)
    changed['puck_to_stator_axial_design']['first_declared_variation_mm'] = None
    with pytest.raises(ValueError, match='finite and positive'):
        proposal(document=changed)
    changed = deepcopy(document)
    changed['candidate_nominal_seat']['maximum_nominal_radial_gap_mm'] = math.nan
    with pytest.raises(ValueError, match='finite and positive'):
        proposal(document=changed)


def test_opt_in_revision_requires_a_redesigned_native_support_path():
    spec = json.loads((DATA/'spec.json').read_text(encoding='utf-8'))
    assert candidate_recipe(spec) is None
    spec['gripper_retention_revision'] = CANDIDATE_REVISION
    candidate = candidate_recipe(spec)
    assert candidate['placement']['outboard_inner_face_z_mm'] == pytest.approx(22.75)
    assert candidate['placement']['nominal_puck_to_stator_gap_mm'] == pytest.approx(3.75)
    assert candidate['geometry']['front_bearing_outer_diameter'] == 10.
    assert candidate['geometry']['front_bearing_inner_diameter'] == 8.02
    assert candidate['geometry']['stator_standoff_centers_xy'] == [[-14., -7.5], [14., -7.5]]
    lip = candidate['capture_lip']
    assert lip['nominal_puck_face_y_mm'] == pytest.approx(238.)
    assert lip['ledge_top_y_mm'] == pytest.approx(238.8)
    assert lip['source_finger']['y_end'] == pytest.approx(232.)
    assert lip['source_nest_seat_radius_mm'] == pytest.approx(15.)
    stem, ledge = lip['boxes']['stem'], lip['boxes']['ledge']
    assert stem['center_tool_mm'][1] - stem['half_size_mm'][1] < lip['source_finger']['y_end']
    assert ledge['center_tool_mm'][0] - ledge['half_size_mm'][0] == pytest.approx(-2.5)
    assert candidate['proposal']['puck_outer_radius_mm'] - 2.5 > lip['source_nest_seat_radius_mm']
    assert len(CANDIDATE_PARTS) == 13
    with pytest.raises(ValueError, match='fresh native CAD and mass records'):
        compile_model(spec)
    bad = deepcopy(spec)
    bad['gripper_retention_revision'] = 'unknown'
    with pytest.raises(ValueError, match='Unknown ARX gripper retention revision'):
        candidate_recipe(bad)


def test_capture_lip_rejects_loss_of_nominal_drop_budget_or_nest_clearance():
    authored = json.loads((DATA/'gripper-retention-proposal.json').read_text(encoding='utf-8'))
    geom = authored['candidate_native_geometry_mm']['capture_lip']
    design = proposal()
    bad = deepcopy(geom)
    bad['nominal_puck_face_gap_mm'] = 1.1
    with pytest.raises(ValueError, match='Capture lip must join'):
        _capture_lip_recipe(bad, design)
    bad = deepcopy(geom)
    bad['inner_reach_below_pad_face_mm'] = 4.5
    with pytest.raises(ValueError, match='clear the nest seat'):
        _capture_lip_recipe(bad, design)
    bad = deepcopy(geom)
    bad['stem_start_y_mm'] = 233.
    with pytest.raises(ValueError, match='join the source finger'):
        _capture_lip_recipe(bad, design)


def test_candidate_brake_only_controls_a_separately_rotating_native_pinion():
    baseline = json.loads((DATA/'P2-baseline-parameters.json').read_text(encoding='utf-8'))
    spec = json.loads((DATA/'spec.json').read_text(encoding='utf-8'))
    spec['gripper_retention_revision'] = CANDIDATE_REVISION
    baseline['integration'] = spec
    baseline['model_revision'] = CANDIDATE_REVISION
    rack = control_source(baseline)['rack']
    assert rack['pinion_joint'] == 'gripper_pinion'
    assert rack['pinion_brake']['holding_torque_Nm'] == pytest.approx(2.2)
    assert rack['pinion_brake']['delay_s'] == pytest.approx(.012)
    assert rack['pinion_brake']['ramp_s'] == pytest.approx(.012)
    baseline['model_revision'] = 'P5-derived-rotating-pinion'
    assert 'pinion_brake' not in control_source(baseline)['rack']
    baseline['model_revision'] = 'P5'
    assert 'pinion_joint' not in control_source(baseline)['rack']


def test_motor_off_candidate_requires_suspended_puck_held_by_both_pads():
    def frame(time_s, *pairs):
        return dict(time_s=time_s,
                    contacts=[dict(geoms=list(pair)) for pair in pairs])
    held = [frame(2.16, ('pad_left', 'puck'), ('pad_right', 'puck')),
            frame(2.19, ('puck', 'pad_left'), ('puck', 'pad_right'))]
    assert fault_contact_eligibility(held, 2.2)['passed']
    on_nest = held + [frame(2.195, ('pad_left', 'puck'),
                            ('pad_right', 'puck'), ('nest_A_seat', 'puck'))]
    check = fault_contact_eligibility(on_nest, 2.2)
    assert not check['passed']
    assert not check['no_external_workpiece_support']
    assert check['nonpad_puck_contact_geoms'] == ['nest_A_seat']
    one_pad = [frame(2.19, ('pad_left', 'puck'))]
    assert not fault_contact_eligibility(one_pad, 2.2)['passed']
    with pytest.raises(ValueError, match='No sampled puck contacts'):
        fault_contact_eligibility(held, 3.2)
