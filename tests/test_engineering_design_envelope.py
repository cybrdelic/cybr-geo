"""The normal engineering output binds its design budgets to source artifacts."""
import hashlib
import json
from types import SimpleNamespace

import pytest

from mechanism_lab.engineering_pipeline import _native_design_screens
from mechanism_lab.design_envelope import connector_force_screens
from mechanism_lab.physical_model import PhysicalModel, digest
from mechanism_lab.planar_linkage import PlanarFourBar
from mechanism_lab.planar_native_envelope import PlanarBody, PlanarNativePath


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


def _planar_screen_fixture(tmp_path):
    """A three-part verified native inventory with one unresolved axial pair."""
    import cadquery as cq

    native = tmp_path / 'native'
    placed = native / 'cad' / 'placed'
    placed.mkdir(parents=True)
    rows, hashes = [], {}
    for body, part in (('fixed', 'housing'), ('crank', 'shaft'),
                       ('rocker', 'bridge')):
        relative = f'cad/placed/{body}__{part}.step'
        cq.exporters.export(cq.Workplane('XY').box(1., 1., 1.).val(),
                            str(native / relative))
        rows.append(dict(body=body, name=part, step_file=relative))
        hashes[relative] = hashlib.sha256((native / relative).read_bytes()).hexdigest()
    manifest = native / 'placed-native-parts.json'
    manifest.write_text(json.dumps(dict(units='mm', records=rows)), encoding='utf-8')
    hashes[manifest.name] = hashlib.sha256(manifest.read_bytes()).hexdigest()
    masses = native / 'mass_properties.json'
    masses.write_text(json.dumps({body: dict(mass_kg=1., com_m=[0., 0., 0.])
                                  for body in ('fixed', 'crank', 'rocker')}), encoding='utf-8')
    hashes[masses.name] = hashlib.sha256(masses.read_bytes()).hexdigest()
    linkage = PlanarFourBar((0., 0.), (4., 0.), 1., 4., 3.)
    path = PlanarNativePath(dict(fixed=PlanarBody('ground'),
        crank=PlanarBody('crank', linkage),
        rocker=PlanarBody('rocker', linkage, z_mm=5.)), 0., 1.,
        source_identity='normal-three-part-engineering-design-screen')
    project = SimpleNamespace(specification={}, planar_path=lambda _spec, _sources: path)
    simulation = dict(model_sha256='model', steps=1, metrics={},
        physical_source=PhysicalModel.from_xml(
            '<mujoco><option gravity="0 -9.81 0"/></mujoco>').to_source())
    return native, dict(files_sha256=hashes), dict(drives=[]), simulation, project


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
    assert screen['planar_kinematics'] is None


def test_planar_design_screen_runs_full_native_axial_broadphase_by_default(tmp_path):
    native, receipt, control, simulation, project = _planar_screen_fixture(tmp_path)
    screen = _native_design_screens(native, receipt, {'design.py': 'source'},
                                    control, simulation, project=project)
    planar = screen['planar_kinematics']
    axial = planar['native_axial_broadphase']
    assert axial['full_native_inventory_selected']
    assert axial['native_step_import']['externally_verified']
    assert axial['source_binding']['native_build_sha256'] == digest(receipt)
    assert axial['source_binding']['planar_source_sha256'] == planar['source_sha256']
    assert planar['axially_separated_pair_count'] == axial['axially_separated_pair_count'] == 2
    assert planar['unresolved_pair_count'] == axial['unresolved_pair_count'] == 1
    assert axial['unresolved_pairs'][0]['parts'] == ['crank__shaft', 'fixed__housing']
    assert planar['continuous_native_clearance_status'] == 'unresolved_without_pair_complete_BREP_certificate'
    assert not axial['full_native_noncontact_pair_coverage']
    assert not screen['manufacturing_qualified']
    # Media readback compares its recomputed object with the saved JSON object.
    assert json.loads(json.dumps(screen, allow_nan=False)) == screen


def test_planar_design_screen_fails_on_missing_receipt_or_changed_step(tmp_path):
    native, receipt, control, simulation, project = _planar_screen_fixture(tmp_path)
    incomplete = json.loads(json.dumps(receipt))
    del incomplete['files_sha256']['cad/placed/rocker__bridge.step']
    with pytest.raises(ValueError, match='verified receipt'):
        _native_design_screens(native, incomplete, {'design.py': 'source'},
                               control, simulation, project=project)
    (native / 'cad/placed/rocker__bridge.step').write_bytes(b'changed')
    with pytest.raises(ValueError, match='verified receipt'):
        _native_design_screens(native, receipt, {'design.py': 'source'},
                               control, simulation, project=project)


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


def test_zero_step_diagnostic_never_claims_solver_coverage():
    empty = connector_force_screens({}, 0)
    assert empty['connectors'] == {}
    assert empty['complete_solver_coverage'] is False
    # Zero matching counters and a synthetic zero peak must not look like
    # measured every-step connector demand in an unintegrated run.
    connector = dict(type='connect', active_steps=0, missing_row_steps=0,
        incomplete_reaction_row_steps=0,
        reaction_vector_norms=dict(anchor_force=dict(samples=0,
            peak_absolute=dict(value=0.))))
    record = connector_force_screens({'pin': connector}, 0)
    assert record['complete_solver_coverage'] is False
    assert record['connectors']['pin']['complete_solver_coverage'] is False
    assert record['connectors']['pin']['peak_modeled_anchor_force_N'] is None
    with pytest.raises(ValueError, match='nonnegative integration-step count'):
        connector_force_screens({}, -1)
