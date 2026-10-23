"""Focused source-bound design budget screens, without hardware claims."""
import json
from pathlib import Path

import pytest

from mechanism_lab.design_envelope import (
    fit_budget_report,
    motor_torque_screen,
    planar_gravity_torque_bound,
    radial_clearance_mm,
    validate_load_paths,
)
from mechanism_lab.fit_construction import PinBoreFit


ROOT = Path(__file__).resolve().parents[1]


def native_gap(name, measured, required, *, kernel=0.0):
    return {
        'schema': 'cybrgeo-native-interfaces/1',
        'checks': [{
            'name': name,
            'intersection_volume_mm3': 0.0,
            'minimum_distance_mm': measured,
            'volume_tolerance_mm3': 1e-5,
            'required_minimum_distance_mm': required,
            'distance_kernel_allowance_mm': kernel,
            'pass': measured + kernel >= required,
        }],
        'passed': measured + kernel >= required,
    }


def test_iris_zero_slack_cannot_absorb_unmeasured_variation():
    # The source-authored IRIS pin-to-lip requirement is 0.20 mm; equal nominal
    # separation leaves no positive dimensional-variation budget.
    source = json.loads((ROOT / 'examples/engineering/iris09/data/mechanism.json').read_text())
    assert source['axial_stack']['moving_pin_to_fixed_lip_clearance_mm'] == 0.2
    native = native_gap('iris-pin-to-lip', 0.2, 0.2, kernel=1e-8)
    unresolved = fit_budget_report(native)
    fit = unresolved['fits'][0]
    assert fit['available_total_variation_mm'] == pytest.approx(0)
    assert fit['status'] == 'unresolved'
    assert fit['unresolved_reason'] == 'part_variation_unavailable'
    assert unresolved['status'] == 'unresolved'
    assert fit['declared_stack_screen_passed'] is None
    assert not unresolved['manufacturing_fit_qualified']
    assert not unresolved['complete_declared_stack']
    assert len(unresolved['native_interface_report_sha256']) == 64

    # Numerical B-rep allowance cannot be borrowed as a process allowance.
    declared = fit_budget_report(native, variation_allowances_mm={
        'iris-pin-to-lip': {'first_mm': 0.001, 'second_mm': 0.001}})
    assert declared['fits'][0]['worst_case_declared_gap_mm'] == pytest.approx(0.198)
    assert not declared['declared_stack_screen_passed']
    assert not declared['manufacturing_fit_qualified']


def test_fourbar_diameter_converts_to_radial_gap_and_stacks_two_parts():
    spec = json.loads((ROOT / 'examples/engineering/fourbar_lift/data/specification.json').read_text())
    fit_design = PinBoreFit.from_document(spec['pin_diameter_mm'], spec['pin_bore_fit'])
    nominal_radial = radial_clearance_mm(fit_design.nominal_diametral_clearance_mm)
    assert nominal_radial == pytest.approx(0.075)
    native = native_gap('fourbar-pin-bushing', nominal_radial,
                        fit_design.required_service_radial_gap_mm)
    screened = fit_budget_report(native, variation_allowances_mm={
        'fourbar-pin-bushing': {'first_mm': fit_design.pin_declared_radial_variation_mm,
                                'second_mm': fit_design.bore_declared_radial_variation_mm}})
    fit = screened['fits'][0]
    assert fit['available_total_variation_mm'] == pytest.approx(0.025)
    assert fit['worst_case_declared_gap_mm'] == pytest.approx(0.055)
    assert fit['worst_case_declared_gap_mm'] - fit_design.required_service_radial_gap_mm == pytest.approx(
        fit_design.design_reserve_radial_gap_mm)
    assert screened['complete_declared_stack']
    assert screened['declared_stack_screen_passed']
    assert not screened['manufacturing_fit_qualified']

    failed = fit_budget_report(native, variation_allowances_mm={
        'fourbar-pin-bushing': {'first_mm': 0.025, 'second_mm': 0.02}})
    assert failed['fits'][0]['status'] == 'declared_stack_failed'
    assert failed['fits'][0]['worst_case_declared_gap_mm'] == pytest.approx(0.03)


def test_embedded_design_variation_screens_exact_native_row_without_qualifying_process():
    native = native_gap('pin_bore', .14, .10)
    native['checks'][0]['declared_design_variations_mm'] = {
        'first_mm': .015, 'second_mm': .015}
    # An axial keeper remains unallocated; one passing pin fit cannot make the
    # complete assembly's fit stack pass.
    keeper = native_gap('axial_keeper', .12, .10)['checks'][0]
    native['checks'].append(keeper)
    screened = fit_budget_report(native)
    pin, axial = screened['fits']
    assert pin['declared_stack_screen_passed']
    assert pin['worst_case_declared_gap_mm'] == pytest.approx(.11)
    assert axial['declared_stack_screen_passed'] is None
    assert not screened['complete_declared_stack']
    assert screened['status'] == 'unresolved'
    assert not screened['manufacturing_fit_qualified']
    with pytest.raises(ValueError, match='conflict|differ|disagree'):
        fit_budget_report(native, variation_allowances_mm={
            'pin_bore': {'first_mm': .02, 'second_mm': .015}})
    matched = fit_budget_report(native, variation_allowances_mm={
        'pin_bore': {'first_mm': .015, 'second_mm': .015}})
    assert matched['fits'][0]['declared_stack_screen_passed']


def test_declared_stack_cannot_borrow_numerical_cad_allowance():
    native = native_gap('pin_bore', .1199, .10, kernel=.001)
    native['checks'][0]['declared_design_variations_mm'] = {
        'first_mm': .01, 'second_mm': .01}
    screened = fit_budget_report(native)
    fit = screened['fits'][0]
    assert fit['nominal_cad_passed']
    assert fit['worst_case_declared_gap_mm'] == pytest.approx(.0999)
    assert fit['numerical_distance_allowance_mm'] == .001
    assert not fit['declared_stack_screen_passed']
    assert screened['status'] == 'declared_stack_failed'


def test_arx_motor_two_has_near_zero_source_model_margin():
    source_motor = {
        'gear_output_torque_limit_Nm': 20.0,
        'belt_allowable_output_torque_Nm': 19.1984 / 1.5,
    }
    observed = 12.798933333298653
    result = motor_torque_screen('motor2', source_motor, observed,
                                 demand_limit_durations_s={'belt': 0., 'gear': 0.})
    assert result['governing_model_limit_Nm'] == pytest.approx(12.798933333333332)
    assert 0 <= result['model_limit_margin_Nm'] < 1e-9
    assert result['model_limit_utilization'] == pytest.approx(1.0)
    assert result['model_screen_passed']
    assert not result['hardware_capacity_qualified']
    assert len(result['source_motor_sha256']) == 64
    assert motor_torque_screen('motor2', source_motor)['status'] == 'unresolved'
    assert motor_torque_screen('motor2', source_motor, observed)['status'] == 'unresolved'
    assert not motor_torque_screen('motor2', source_motor, 12.8,
        demand_limit_durations_s={'belt': 0., 'gear': 0.})['model_screen_passed']
    clipped = motor_torque_screen('motor2', source_motor, observed,
        demand_limit_durations_s={'belt': .0983333333334, 'gear': 0.})
    assert clipped['status'] == 'source_model_demand_limited'
    assert clipped['demanded_torque_was_limited']
    assert not clipped['model_screen_passed']


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -1.0, True, '0.1'])
def test_invalid_variation_rejected(bad):
    with pytest.raises(ValueError):
        fit_budget_report(native_gap('joint', 0.2, 0.1), variation_allowances_mm={
            'joint': {'first_mm': bad, 'second_mm': 0.01}})


def test_unknown_variation_and_invalid_source_identity_rejected():
    with pytest.raises(ValueError, match='unknown'):
        fit_budget_report(native_gap('joint', 0.2, 0.1), variation_allowances_mm={
            'different': {'first_mm': 0.0, 'second_mm': 0.0}})
    with pytest.raises(ValueError, match='SHA-256'):
        motor_torque_screen('drive', {'gear_output_torque_limit_Nm': 1},
                            source_sha256='unbound')
    with pytest.raises(ValueError):
        motor_torque_screen('drive', {'gear_output_torque_limit_Nm': float('inf')}, 0.1)
    with pytest.raises(ValueError):
        radial_clearance_mm(-0.1)


def test_declared_stack_cannot_hide_other_failed_native_interface():
    native = native_gap('joint', 0.2, 0.1)
    native['checks'].append({
        'name': 'support', 'minimum_distance_mm': 0.0,
        'intersection_volume_mm3': 1.0, 'volume_tolerance_mm3': 1e-5,
        'pass': False,
    })
    native['passed'] = False
    screened = fit_budget_report(native, variation_allowances_mm={
        'joint': {'first_mm': 0.01, 'second_mm': 0.01}})
    assert screened['fits'][0]['declared_stack_screen_passed']
    assert not screened['native_interface_passed']
    assert not screened['declared_stack_screen_passed']


def test_load_path_topology_needs_connected_and_complete_interfaces():
    parts = ['rotor', 'link', 'bearing', 'housing']
    interfaces = {
        'drive': ['rotor', 'link'],
        'pin': ['link', 'bearing'],
        'reaction': ['bearing', 'housing'],
    }
    paths = {'drive_load': {'parts': parts,
                            'interfaces': ['drive', 'pin', 'reaction']}}
    report = validate_load_paths(parts, interfaces, {'drive_load': 'rotor'}, paths)
    assert report['topology_passed']
    assert not report['hardware_load_path_qualified']

    with pytest.raises(ValueError, match='omit'):
        validate_load_paths(parts, interfaces, {'drive_load': 'rotor'}, {
            'drive_load': {'parts': parts[:-1], 'interfaces': ['drive', 'pin']}})
    with pytest.raises(ValueError, match='disconnects'):
        validate_load_paths(parts, interfaces, {'drive_load': 'rotor'}, {
            'drive_load': {'parts': parts, 'interfaces': ['pin', 'drive', 'reaction']}})


def test_planar_gravity_bound_projects_gravity_and_converts_millimetres():
    masses = {'link': 2.0, 'payload': 0.5}
    speeds = {'link': 100.0, 'payload': 50.0}
    vertical = planar_gravity_torque_bound(masses, speeds, [0, 0, -9.81], [0, 2, 0])
    assert vertical['gravity_in_motion_plane_m_s2'] == pytest.approx(9.81)
    assert vertical['body_contributions_Nm']['link'] == pytest.approx(1.962)
    assert vertical['body_contributions_Nm']['payload'] == pytest.approx(0.24525)
    assert vertical['total_gravity_torque_bound_Nm'] == pytest.approx(2.20725)
    assert vertical['status'] == 'source_geometry_bound'
    assert not vertical['hardware_load_qualified']
    assert len(vertical['source_inputs_sha256']) == 64
    horizontal = planar_gravity_torque_bound(masses, speeds, [0, 0, -9.81], [0, 0, 1])
    assert horizontal['total_gravity_torque_bound_Nm'] == pytest.approx(0)
    assert horizontal['source_inputs_sha256'] != vertical['source_inputs_sha256']


@pytest.mark.parametrize('masses,speeds,gravity,normal', [
    ({'link': 1}, {}, [0, 0, -9.81], [0, 1, 0]),
    ({'link': 0}, {'link': 1}, [0, 0, -9.81], [0, 1, 0]),
    ({'link': 1}, {'link': -1}, [0, 0, -9.81], [0, 1, 0]),
    ({'link': 1}, {'link': 1}, [0, 0, float('nan')], [0, 1, 0]),
    ({'link': 1}, {'link': 1}, [0, 0, -9.81], [0, 0, 0]),
])
def test_planar_gravity_bound_rejects_missing_or_invalid_source_inputs(
        masses, speeds, gravity, normal):
    with pytest.raises(ValueError):
        planar_gravity_torque_bound(masses, speeds, gravity, normal)
