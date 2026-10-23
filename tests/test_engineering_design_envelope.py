"""The normal engineering output binds its design budgets to source artifacts."""
import hashlib
import json

import pytest

from mechanism_lab.engineering_pipeline import _native_design_screens
from mechanism_lab.design_envelope import connector_force_screens


def _screen(tmp_path, *, gap=.2, demand=.4, limit=1., variations=None):
    report = dict(schema='cybrgeo-native-interfaces/1', passed=True,
        checks=[dict(name='shaft_fit', minimum_distance_mm=gap,
            required_minimum_distance_mm=.2, distance_kernel_allowance_mm=1e-8,
            intersection_volume_mm3=0., volume_tolerance_mm3=1e-5,
            **{'pass': True})])
    if variations is not None:
        report['checks'][0]['declared_design_variations_mm'] = variations
    payload = json.dumps(report).encode()
    (tmp_path / 'native-interface-requirements.json').write_bytes(payload)
    receipt = dict(files_sha256={
        'native-interface-requirements.json': hashlib.sha256(payload).hexdigest()})
    control = dict(drives=[dict(actuator='motor', motor=dict(
        gear_output_torque_limit_Nm=limit))])
    simulation = dict(model_sha256='model', steps=1, metrics=dict(actuators=dict(
        motor=dict(peak_output_torque_Nm=demand,
                   belt_demand_limit_s=0., gear_demand_limit_s=0.))))
    return receipt, control, simulation


def test_design_budget_is_bound_and_unqualified_without_part_variation(tmp_path):
    receipt, control, simulation = _screen(tmp_path)
    screen = _native_design_screens(tmp_path, receipt, {'design.py': 'source'},
                                    control, simulation)
    fit = screen['nominal_fit']
    assert fit['fits'][0]['available_total_variation_mm'] == pytest.approx(0.)
    assert fit['status'] == 'unresolved'
    assert not fit['manufacturing_fit_qualified']
    assert screen['drives']['motor']['model_limit_margin_Nm'] == pytest.approx(.6)
    assert screen['drives']['motor']['model_screen_passed']
    assert screen['drives']['motor']['complete_demand_telemetry']
    assert not screen['drives']['motor']['hardware_capacity_qualified']
    assert not screen['manufacturing_qualified']


def test_native_interface_tamper_cannot_relabel_design_budget(tmp_path):
    receipt, control, simulation = _screen(tmp_path)
    (tmp_path / 'native-interface-requirements.json').write_text('{}')
    with pytest.raises(ValueError, match='verified construction receipt'):
        _native_design_screens(tmp_path, receipt, {'design.py': 'source'},
                               control, simulation)


def test_bound_constructive_fit_budget_enters_normal_design_screen(tmp_path):
    receipt, control, simulation = _screen(tmp_path, gap=.24,
        variations={'first_mm': .01, 'second_mm': .01})
    screen = _native_design_screens(tmp_path, receipt, {'design.py': 'source'},
                                    control, simulation)
    assert screen['nominal_fit']['fits'][0]['worst_case_declared_gap_mm'] == pytest.approx(.22)
    assert screen['nominal_fit']['declared_stack_screen_passed']
    assert not screen['nominal_fit']['manufacturing_fit_qualified']
    assert not screen['manufacturing_qualified']
    # Editing the embedded target after construction invalidates the native
    # receipt before it can be used to improve a diagnostic fit score.
    report = json.loads((tmp_path / 'native-interface-requirements.json').read_text())
    report['checks'][0]['declared_design_variations_mm']['first_mm'] = .001
    (tmp_path / 'native-interface-requirements.json').write_text(json.dumps(report))
    with pytest.raises(ValueError, match='verified construction receipt'):
        _native_design_screens(tmp_path, receipt, {'design.py': 'source'},
                               control, simulation)


def test_missing_native_interfaces_and_telemetry_stay_unresolved(tmp_path):
    _, control, simulation = _screen(tmp_path)
    simulation['metrics']['actuators'].clear()
    screen = _native_design_screens(tmp_path, dict(files_sha256={}),
                                    {'design.py': 'source'}, control, simulation)
    assert screen['nominal_fit']['status'] == 'native_interface_requirements_not_declared'
    assert screen['drives']['motor']['model_screen_passed'] is None
    assert not screen['manufacturing_qualified']


def test_missing_source_limit_telemetry_cannot_pass_model_screen(tmp_path):
    receipt, control, simulation = _screen(tmp_path)
    del simulation['metrics']['actuators']['motor']['belt_demand_limit_s']
    screen = _native_design_screens(tmp_path, receipt, {'design.py': 'source'},
                                    control, simulation)
    assert screen['drives']['motor']['status'] == 'unresolved'
    assert screen['drives']['motor']['model_screen_passed'] is None


def test_missing_solver_reaction_row_cannot_become_zero_load():
    connector = dict(type='connect', active_steps=3, missing_row_steps=0,
        incomplete_reaction_row_steps=0,
        reaction_vector_norms=dict(anchor_force=dict(samples=3,
            peak_absolute=dict(value=12.5))))
    good = connector_force_screens({'pin': connector}, 3)['connectors']['pin']
    assert good['complete_solver_coverage']
    assert good['peak_modeled_anchor_force_N'] == 12.5
    connector['missing_row_steps'] = 1
    connector['reaction_vector_norms']['anchor_force']['samples'] = 2
    incomplete = connector_force_screens({'pin': connector}, 3)['connectors']['pin']
    assert not incomplete['complete_solver_coverage']
    assert incomplete['peak_modeled_anchor_force_N'] is None
    assert incomplete['observed_peak_if_incomplete_N'] == 12.5
