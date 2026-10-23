"""Synthetic cryptographic fixtures test gates; they are not hardware evidence."""
import base64
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from cryptography.hazmat.primitives import serialization

from mechanism_lab.qualification import (
    QualificationError, QualificationPlan, QualifiedNativeConstruction,
    _recheck_qualified_native_steps, _require_physical_mass_binding,
    _require_connector_demand_binding, _simulated_loads, _verify_evidence,
    release_project,
)


def _plan():
    return dict(schema='cybrgeo-qualification-plan/1',
                materials={'alloy': dict(grade='TEST-ALLOY', lot='TEST-LOT', density_kg_m3=2700.,
                                         allowable_stress_MPa=100., temperature_limit_C=80.)},
                processes={'mill': dict(method='TEST-MILL', facility='TEST-FACILITY', revision='TEST-1',
                                        allowed_materials=['alloy'], minimum_feature_mm=1.,
                                        capability_tolerance_mm=.1, maximum_part_volume_mm3=1000.)},
                parts={name: dict(material='alloy', process='mill', nominal_minimum_wall_mm=2.,
                                  required_dimensional_tolerance_mm=.1, maximum_volume_mm3=500.)
                       for name in ('fixed__a', 'moving__b')},
                interfaces={'pin_fit': dict(parts=['fixed__a', 'moving__b'], nominal_clearance_mm=.5,
                                            first_tolerance_mm=.1, second_tolerance_mm=.1,
                                            required_clearance_mm=.2, demand_N=5., design_capacity_N=10.)},
                loads={'output': dict(part='moving__b', demand_Nm=1., design_capacity_Nm=2.,
                                      service_temperature_C=50.)},
                assembly_requirements=dict(minimum_endurance_cycles=1000,
                                           minimum_thermal_test_C=60.,
                                           maximum_measured_backlash_mm=.3))


def test_qualified_connector_mapping_needs_known_interface_and_covers_model_demand():
    from types import SimpleNamespace

    plan = _plan()
    plan['solver_connectors'] = {'bearing_joint': 'unknown'}
    with pytest.raises(QualificationError, match='planned physical interface'):
        QualificationPlan(plan)
    plan['solver_connectors']['bearing_joint'] = 'pin_fit'
    qualified = QualificationPlan(plan)
    root = ET.Element('mujoco')
    ET.SubElement(ET.SubElement(root, 'equality'), 'connect',
                  name='bearing_joint', body1='fixed', body2='moving')
    simulation = dict(steps=2, metrics={'constraints': {'bearing_joint': dict(
        type='connect', active_steps=2, missing_row_steps=0,
        incomplete_reaction_row_steps=0,
        reaction_vector_norms={'anchor_force': dict(samples=2,
            peak_absolute=dict(value=4.))})}})
    assert _require_connector_demand_binding(qualified, SimpleNamespace(root=root),
                                             simulation) is None


def _signed_fixture(tmp_path, *, basis='a'*64, step_hashes=None):
    package, authority = tmp_path/'evidence', tmp_path/'authority'
    package.mkdir()
    authority.mkdir()
    private = Ed25519PrivateKey.generate()
    public = private.public_key().public_bytes(serialization.Encoding.Raw, serialization.PublicFormat.Raw)
    policy = authority/'trusted-issuers.json'
    policy.write_text(json.dumps(dict(schema='cybrgeo-trust-policy/1', issuers={
        'test-only': dict(public_key_base64=base64.b64encode(public).decode(),
                          allowed_kinds=['system_service_envelope', 'material', 'process', 'part_inspection', 'interface_test',
                                         'load_test', 'assembly_test'])})), encoding='utf-8')
    plan = _plan()
    step_hashes = step_hashes or {'fixed__a': 'b'*64, 'moving__b': 'c'*64}
    rows = [
        ('system_service_envelope', 'complete_assembly', dict(load_paths_reviewed=True,
            mechanical_load_paths={'output': {'parts': ['moving__b', 'fixed__a'],
                                              'interfaces': ['pin_fit']}},
            actuator_demand_Nm={'output': 1.}, interface_demand_N={'pin_fit': 5.},
            required_thermal_test_C=60., required_endurance_cycles=1000,
            maximum_backlash_mm=.3)),
        ('material', 'alloy', dict(grade='TEST-ALLOY', lot='TEST-LOT', density_kg_m3=2700.,
                                   allowable_stress_MPa=100., temperature_limit_C=80.)),
        ('process', 'mill', dict(method='TEST-MILL', facility='TEST-FACILITY', revision='TEST-1',
                                 minimum_feature_mm=1., capability_tolerance_mm=.1,
                                 maximum_part_volume_mm3=1000.)),
        ('part_inspection', 'fixed__a', dict(material_lot='TEST-LOT', process_revision='TEST-1',
                                             process_facility='TEST-FACILITY', process_method='TEST-MILL',
                                             fabricated_serial='TEST-A', measured_minimum_wall_mm=2.,
                                             maximum_dimensional_error_mm=.05, step_sha256=step_hashes['fixed__a'])),
        ('part_inspection', 'moving__b', dict(material_lot='TEST-LOT', process_revision='TEST-1',
                                              process_facility='TEST-FACILITY', process_method='TEST-MILL',
                                              fabricated_serial='TEST-B', measured_minimum_wall_mm=2.,
                                              maximum_dimensional_error_mm=.05, step_sha256=step_hashes['moving__b'])),
        ('interface_test', 'pin_fit', dict(assembled_serial='TEST-ASSEMBLY',
                                           part_serials={'fixed__a': 'TEST-A', 'moving__b': 'TEST-B'},
                                           measured_minimum_clearance_mm=.3, tested_capacity_N=10.)),
        ('load_test', 'output', dict(assembled_serial='TEST-ASSEMBLY', part_serial='TEST-B',
                                      tested_capacity_Nm=2., tested_temperature_C=60.)),
        ('assembly_test', 'complete_assembly', dict(assembled_serial='TEST-ASSEMBLY', fit_passed=True,
                                                    part_serials={'fixed__a': 'TEST-A', 'moving__b': 'TEST-B'},
                                                    motion_passed=True, thermal_passed=True,
                                                    tested_cycles=1000., tested_temperature_C=70.,
                                                    measured_backlash_mm=.1)),
    ]
    evidence = []
    for index, (kind, subject, measurements) in enumerate(rows):
        name = f'test-record-{index}.txt'
        payload = f'TEST ONLY: {kind}/{subject}\n'.encode()
        (package/name).write_bytes(payload)
        row = dict(schema='cybrgeo-signed-evidence/1', kind=kind, subject=subject,
                   basis_sha256=basis, result='observed_pass', measurements=measurements,
                   document=name, document_sha256=hashlib.sha256(payload).hexdigest(), issuer='test-only')
        serialized = json.dumps(row, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
        row['signature_base64'] = base64.b64encode(private.sign(serialized)).decode()
        evidence.append(row)
    (package/'evidence.json').write_text(json.dumps(dict(schema='cybrgeo-signed-evidence-set/1',
                                                         records=evidence)), encoding='utf-8')
    return package, policy, plan, evidence, private


def _resign(package, records, private):
    for row in records:
        signed = {name: value for name, value in row.items() if name != 'signature_base64'}
        row['signature_base64'] = base64.b64encode(private.sign(
            json.dumps(signed, sort_keys=True, separators=(',', ':'), allow_nan=False).encode())).decode()
    (package/'evidence.json').write_text(json.dumps(dict(schema='cybrgeo-signed-evidence-set/1',
                                                         records=records)), encoding='utf-8')


def _check(package, policy, plan):
    return _verify_evidence(package, policy, 'a'*64, plan, {'fixed__a': 'b'*64, 'moving__b': 'c'*64})


def test_plan_rejects_unsupported_material_process_fit_and_load():
    QualificationPlan(_plan())
    mutations = [
        ('unsupported material/process', lambda p: p['processes']['mill']['allowed_materials'].clear()),
        ('wall below process limit', lambda p: p['parts']['fixed__a'].update(nominal_minimum_wall_mm=.5)),
        ('required tolerance below process capability', lambda p: p['parts']['fixed__a'].update(required_dimensional_tolerance_mm=.05)),
        ('closed worst-case fit', lambda p: p['interfaces']['pin_fit'].update(first_tolerance_mm=.4)),
        ('part variation exceeds fit allowance', lambda p: p['parts']['fixed__a'].update(required_dimensional_tolerance_mm=.15)),
        ('interface capacity below demand', lambda p: p['interfaces']['pin_fit'].update(design_capacity_N=4.)),
        ('load capacity below demand', lambda p: p['loads']['output'].update(design_capacity_Nm=.5)),
        ('service temperature above material limit', lambda p: p['loads']['output'].update(service_temperature_C=90.)),
    ]
    for _, mutate in mutations:
        plan = _plan()
        mutate(plan)
        with pytest.raises(QualificationError):
            QualificationPlan(plan)


def test_plan_rejects_cross_platform_step_name_collisions():
    for alias in ('fixed__A', 'CON__spacer', 'fixed__NUL', 'fixed__a.', 'fixed__a__rev'):
        plan = _plan()
        plan['parts'][alias] = deepcopy(plan['parts']['fixed__a'])
        with pytest.raises(QualificationError):
            QualificationPlan(plan)


def test_only_active_rack_motor_is_a_load_case():
    assert set(_simulated_loads({'actuators': {'output': {'peak_output_torque_Nm': 1.}},
                                 'rack_motor': None})) == {'output'}
    assert set(_simulated_loads({'actuators': {'output': {'peak_output_torque_Nm': 1.}},
                                 'rack_motor': {'peak_output_torque_Nm': 2.}})) == {'output', 'rack_motor'}
    with pytest.raises(QualificationError):
        _simulated_loads({'actuators': {}, 'rack_motor': 'unsupported'})


def test_plain_diagnostic_builder_cannot_mint_qualification_receipt(tmp_path):
    from mechanism_lab.native_construction import NativeConstruction
    plain = NativeConstruction(tmp_path/'diagnostic', {'alloy': 2700.})
    forged = {'schema': 'cybrgeo-qualification-construction/1', 'plan_sha256': '0'*64}
    with pytest.raises(ValueError, match='reserved for the qualified builder'):
        plain.write_json('qualification-construction.json', forged)
    with pytest.raises(ValueError, match='requires the qualified builder'):
        plain.finish('diagnostic only', extra_records={'qualification-construction.json': forged})
    assert not (plain.output/'qualification-construction.json').exists()
    assert not (plain.output/'native-build.json').exists()


def test_policy_blocks_wrong_part_before_native_export(tmp_path, monkeypatch):
    class Solid:
        def isValid(self): return True
        def Solids(self): return [self]
        def Volume(self): return 10.
    monkeypatch.setattr('mechanism_lab.qualification._shape', lambda shape: shape)
    reached = []
    monkeypatch.setattr('mechanism_lab.native_construction.NativeConstruction.place',
                        lambda *args, **kwargs: reached.append(args))
    constructor = QualifiedNativeConstruction(tmp_path/'native', {'alloy': 2700.},
                                               qualification_plan=_plan())
    with pytest.raises(QualificationError, match='lacks its exact'):
        constructor.place('fixed', 'a', Solid(), 'wrong-material')
    with pytest.raises(QualificationError, match='lacks its exact'):
        constructor.place('fixed', 'unknown', Solid(), 'alloy')
    with pytest.raises(QualificationError, match='portable body__part'):
        constructor.place('fixed', 'a__subpart', Solid(), 'alloy')
    with pytest.raises(QualificationError, match='portable body__part'):
        constructor.place('fixed__a', 'subpart', Solid(), 'alloy')
    with pytest.raises(QualificationError, match='unbound CAD templates'):
        constructor.addpart('unbound', Solid(), 'alloy')
    with pytest.raises(QualificationError, match='unplaced or overridden'):
        constructor.add_mass_record('fixed', {})
    assert reached == []
    constructor.place('fixed', 'a', Solid(), 'alloy')
    with pytest.raises(QualificationError, match='already exported'):
        constructor.place('fixed', 'a', Solid(), 'alloy')
    assert len(reached) == 1


def test_signed_evidence_requires_external_authority_and_full_coverage(tmp_path):
    package, policy, plan, records, private = _signed_fixture(tmp_path)
    assert _check(package, policy, plan)['evidence_count'] == 8
    (package/'evidence.json').write_text(json.dumps(dict(schema='cybrgeo-signed-evidence-set/1',
                                                         records=records[:-1])), encoding='utf-8')
    with pytest.raises(QualificationError, match='cover every required'):
        _check(package, policy, plan)
    _resign(package, records, private)
    wrong_policy = tmp_path/'authority'/'untrusted.json'
    wrong_policy.write_text(json.dumps(dict(schema='cybrgeo-trust-policy/1', issuers={})), encoding='utf-8')
    with pytest.raises(QualificationError):
        _check(package, wrong_policy, plan)


@pytest.mark.parametrize('fault', ['stale_basis', 'forged_signature', 'material_lot',
                                   'process_revision', 'process_facility', 'process_method',
                                   'bad_fit', 'weak_interface_capacity', 'low_load', 'wrong_step', 'low_cycles',
                                   'wrong_assembly_bom', 'wrong_interface_unit',
                                   'wrong_interface_part', 'wrong_load_unit', 'wrong_load_part',
                                   'understated_service_torque', 'unreviewed_load_path',
                                   'missing_mechanical_path', 'disconnected_mechanical_path'])
def test_signed_evidence_rejects_stale_forged_or_failed_measurements(tmp_path, fault):
    package, policy, plan, records, private = _signed_fixture(tmp_path)
    modified = deepcopy(records)
    if fault == 'stale_basis': modified[0]['basis_sha256'] = '0'*64
    elif fault == 'forged_signature': modified[0]['signature_base64'] = records[1]['signature_base64']
    elif fault == 'material_lot': modified[1]['measurements']['lot'] = 'OTHER'
    elif fault == 'process_revision': modified[2]['measurements']['revision'] = 'OTHER'
    elif fault == 'process_facility': modified[3]['measurements']['process_facility'] = 'OTHER'
    elif fault == 'process_method': modified[4]['measurements']['process_method'] = 'OTHER'
    elif fault == 'bad_fit': modified[5]['measurements']['measured_minimum_clearance_mm'] = .1
    elif fault == 'weak_interface_capacity': modified[5]['measurements']['tested_capacity_N'] = 6.
    elif fault == 'low_load': modified[6]['measurements']['tested_capacity_Nm'] = 1.5
    elif fault == 'wrong_step': modified[3]['measurements']['step_sha256'] = '0'*64
    elif fault == 'low_cycles': modified[7]['measurements']['tested_cycles'] = 999
    elif fault == 'wrong_assembly_bom': modified[7]['measurements']['part_serials']['moving__b'] = 'OTHER'
    elif fault == 'wrong_interface_unit': modified[5]['measurements']['assembled_serial'] = 'OTHER'
    elif fault == 'wrong_interface_part': modified[5]['measurements']['part_serials']['moving__b'] = 'OTHER'
    elif fault == 'wrong_load_unit': modified[6]['measurements']['assembled_serial'] = 'OTHER'
    elif fault == 'wrong_load_part': modified[6]['measurements']['part_serial'] = 'OTHER'
    elif fault == 'understated_service_torque': modified[0]['measurements']['actuator_demand_Nm']['output'] = 1.5
    elif fault == 'unreviewed_load_path': modified[0]['measurements']['load_paths_reviewed'] = False
    elif fault == 'missing_mechanical_path': modified[0]['measurements']['mechanical_load_paths'] = {}
    elif fault == 'disconnected_mechanical_path': modified[0]['measurements']['mechanical_load_paths']['output']['parts'].reverse()
    if fault != 'forged_signature': _resign(package, modified, private)
    else: (package/'evidence.json').write_text(json.dumps(dict(schema='cybrgeo-signed-evidence-set/1',
                                                               records=modified)), encoding='utf-8')
    with pytest.raises(QualificationError):
        _check(package, policy, plan)


def test_examples_cannot_release_without_preconstruction_policy(tmp_path):
    # Current authored examples intentionally lack supplier lot, process and
    # fabricated-hardware evidence. They must not create a product release.
    from mechanism_lab.engineering_source import project_directory
    source = project_directory('fourbar_lift')
    with pytest.raises(QualificationError, match='qualification construction plan'):
        release_project(source, tmp_path/'run', tmp_path/'evidence', tmp_path/'trust.json',
                        tmp_path/'product')
    assert not (tmp_path/'product').exists()


@pytest.mark.parametrize('case,limit,duration,expected', [
    ('drive', 'belt', .001, 'demanded torque beyond its belt model limit'),
    ('drive', 'gear', .001, 'demanded torque beyond its gear model limit'),
    ('drive', 'belt', None, 'demand-limit'),
    ('drive', 'gear', None, 'demand-limit'),
    ('rack', 'output', .001, 'source output model limit'),
    ('rack', 'output', None, 'output demand-limit telemetry'),
    ('rack', 'output', 0., None),
    ('unmapped_connector', 'belt', 0., 'lack exact planned interface mapping'),
    ('missing_connector', 'belt', 0., 'inventory differs'),
    ('incomplete_connector', 'belt', 0., 'reaction coverage is incomplete'),
    ('understated_connector', 'belt', 0., 'understates modeled connector force'),
    ('unrelated_connector', 'belt', 0., 'unrelated native interface bodies'),
    ('missing_body_endpoint_connector', 'belt', 0., 'requires two distinct named body endpoints'),
])
def test_release_rejects_unbounded_drivetrain_or_connector_demand(
        tmp_path, monkeypatch, case, limit, duration, expected):
    """Clipped drivetrain demand or incomplete connector demand cannot release.

    Only unrelated upstream CAD/source checks are stubbed. The release method
    reads persisted metrics; a clean rack reaches downstream release basis
    hashing without needing rotary belt/gear fields. No product is created.
    """
    from types import SimpleNamespace
    from mechanism_lab import engineering_source, physical_model, qualification
    from mechanism_lab.causal_runtime import _runtime_identity
    from mechanism_lab.physical_model import digest

    plan = _plan()
    connector_case = case.endswith('_connector')
    if connector_case and case != 'unmapped_connector':
        plan['solver_connectors'] = {'bearing_joint': 'pin_fit'}
    if case == 'rack':
        plan['loads']['rack_motor'] = dict(part='moving__b', demand_Nm=1.,
                                          design_capacity_Nm=2., service_temperature_C=50.)
    source = tmp_path/'project'
    (source/'data').mkdir(parents=True)
    plan_path = source/'data'/'qualification-plan.json'
    plan_path.write_text(json.dumps(plan), encoding='utf-8')
    source_files = {'data/qualification-plan.json': hashlib.sha256(plan_path.read_bytes()).hexdigest()}
    native = tmp_path/'native'
    native.mkdir()
    qualified_record = dict(schema='cybrgeo-qualification-construction/1',
                            plan_sha256=QualificationPlan(plan).sha256,
                            qualification_service_sha256=qualification._sha(qualification.__file__),
                            placements=['fixed__a', 'moving__b'], interface_names=['pin_fit'],
                            exact_placed_interface_checks={'pin_fit': dict(
                                parts=['fixed__a', 'moving__b'], intersection_volume_mm3=0.,
                                minimum_distance_mm=.5, worst_case_clearance_mm=.3)})
    record_path = native/'qualification-construction.json'
    record_path.write_text(json.dumps(qualified_record), encoding='utf-8')
    receipt = dict(files_sha256={'qualification-construction.json': hashlib.sha256(
                       record_path.read_bytes()).hexdigest()},
                   material_densities_kg_m3={'alloy': 2700.})
    (native/'placed-native-parts.json').write_text(json.dumps(dict(units='mm', records=[
        dict(body='fixed', name='a'), dict(body='moving', name='b')])), encoding='utf-8')
    (native/'native-interface-requirements.json').write_text(json.dumps(dict(
        passed=True, checks=[dict(name='pin_fit')])), encoding='utf-8')
    binding = {'schema': 'test-only-source-binding'}
    run = tmp_path/'run'
    (run/'simulation').mkdir(parents=True)
    model = {'schema': 'test-only-physical', 'mass_records': {}}
    control = {'schema': 'test-only-controller'}
    metrics = {'peak_output_torque_Nm': .5,
               'belt_demand_limit_s': 0., 'gear_demand_limit_s': 0.}
    rack_metrics = None
    if case == 'rack':
        rack_metrics = {'peak_output_torque_Nm': .2}
        if duration is not None:
            rack_metrics['output_demand_limit_s'] = duration
    elif case == 'drive' and duration is None:
        del metrics[f'{limit}_demand_limit_s']
    elif case == 'drive':
        metrics[f'{limit}_demand_limit_s'] = duration
    reaction = dict(type='connect', active_steps=2, missing_row_steps=0,
        incomplete_reaction_row_steps=0,
        reaction_vector_norms={'anchor_force': dict(samples=2,
            peak_absolute=dict(value=6. if case == 'understated_connector' else 4.))})
    constraints = ({} if case == 'missing_connector' else
                   {'bearing_joint': reaction} if connector_case else {})
    if case == 'incomplete_connector':
        reaction['missing_row_steps'] = 1
    root = ET.Element('mujoco')
    if connector_case:
        attributes = dict(name='bearing_joint', body1=(
            'other' if case == 'unrelated_connector' else 'fixed'))
        if case != 'missing_body_endpoint_connector':
            attributes['body2'] = 'moving'
        ET.SubElement(ET.SubElement(root, 'equality'), 'connect', **attributes)
    simulation = dict(schema='cybrgeo-causal-simulation/1',
                      model_sha256=digest(model), control_sha256=digest(control),
                      runtime_implementation=_runtime_identity(), mujoco_version='test-only',
                      options={}, steps=2, metrics={'actuators': {'output': metrics},
                                                    'rack_motor': rack_metrics,
                                                    'constraints': constraints})
    (run/'physical-source.json').write_text(json.dumps(model), encoding='utf-8')
    (run/'control-source.json').write_text(json.dumps(control), encoding='utf-8')
    (run/'simulation'/'simulation.json').write_text(json.dumps(simulation), encoding='utf-8')
    report = dict(schema='cybrgeo-engineering-result/2', name='synthetic', passed=True,
                  source_files=source_files, native={'directory': str(native)},
                  native_build_sha256=digest(receipt),
                  native_source_binding_sha256=digest(binding),
                  model_sha256=digest(model), control_sha256=digest(control),
                  simulation=simulation)
    (run/'engineering.json').write_text(json.dumps(report), encoding='utf-8')

    monkeypatch.setattr(engineering_source, 'project_directory', lambda _: source)
    monkeypatch.setattr(engineering_source, 'load_project', lambda _: (
        SimpleNamespace(name='synthetic', specification={}), source_files))
    monkeypatch.setattr(engineering_source, 'verify_native_build',
                        lambda *args, **kwargs: receipt)
    monkeypatch.setattr(engineering_source, 'verify_native_source_binding',
                        lambda *args, **kwargs: binding)
    monkeypatch.setattr(qualification, '_recheck_qualified_native_steps',
                        lambda *args, **kwargs: {})
    monkeypatch.setattr(physical_model.PhysicalModel, 'from_source',
                        lambda *_: SimpleNamespace(mass_records={}, root=root))
    class LoadGatePassed(Exception):
        pass
    original_sha = qualification._sha
    def observe_after_load_gate(path):
        if Path(path).name == 'trace.json.gz':
            raise LoadGatePassed
        return original_sha(path)
    monkeypatch.setattr(qualification, '_sha', observe_after_load_gate)
    monkeypatch.setattr(qualification, '_verify_evidence',
                        lambda *args, **kwargs: pytest.fail('Load gate was bypassed'))
    product = tmp_path/'product'
    with pytest.raises(LoadGatePassed if expected is None else QualificationError,
                       match=expected):
        release_project(source, run, tmp_path/'evidence', tmp_path/'trusted-policy.json', product)
    assert not product.exists()
    assert not list(tmp_path.glob('.product-stage-*'))


def test_signed_release_copies_real_qualified_native_steps_atomically(tmp_path, monkeypatch):
    """Synthetic signed reports exercise the real native constructor and release copy.

    The fake engineering run is a unit-test stand-in, never a qualified robot.
    It retains real CadQuery STEP and completion-file hashes while mocking only
    the causal simulation/source-loader services covered elsewhere.
    """
    import cadquery as cq
    import mechanism_lab.qualification as qualification
    import mechanism_lab.engineering_source as engineering_source
    from mechanism_lab.native_construction import NativeInterfaceRequirements
    from mechanism_lab.physical_model import digest
    from mechanism_lab.causal_runtime import _runtime_identity
    from types import SimpleNamespace

    plan = _plan()
    project_dir = tmp_path/'project'
    (project_dir/'data').mkdir(parents=True)
    plan_file = project_dir/'data'/'qualification-plan.json'
    plan_file.write_text(json.dumps(plan), encoding='utf-8')
    source_files = {'data/qualification-plan.json': hashlib.sha256(plan_file.read_bytes()).hexdigest()}
    native = tmp_path/'native'
    builder = QualifiedNativeConstruction(native, {'alloy': 2700.},
        qualification_plan=plan, provenance={'input_spec_sha256': digest({'test': True})})
    first = cq.Workplane('XY').box(4, 4, 4).translate((-2.25, 0, 0))
    second = cq.Workplane('XY').box(4, 4, 4).translate((2.25, 0, 0))
    builder.place('fixed', 'a', first, 'alloy')
    builder.place('moving', 'b', second, 'alloy')
    fits = NativeInterfaceRequirements()
    fits.clearance('pin_fit', first, second, minimum_distance_mm=.5, kernel_allowance_mm=1e-8)
    placed_step = native/'cad'/'placed'/'fixed__a.step'
    original_step = placed_step.read_bytes()
    cq.exporters.export(first.translate((20, 0, 0)), str(placed_step))
    with pytest.raises(QualificationError, match='differs from constructed CAD'):
        builder.finish('test-only native mass', native_requirements=fits)
    placed_step.write_bytes(original_step)
    spoofed = NativeInterfaceRequirements()
    spoofed.clearance('pin_fit', first, second.translate((100, 0, 0)), minimum_distance_mm=.5)
    with pytest.raises(QualificationError, match='different solids'):
        builder.finish('test-only native mass', native_requirements=spoofed)
    builder.placed_records.append(deepcopy(builder.placed_records[0]))
    with pytest.raises(QualificationError, match='inventory is ambiguous'):
        builder.finish('test-only native mass', native_requirements=fits)
    builder.placed_records.pop()
    assert not (native/'qualification-construction.json').exists()
    builder.finish('test-only native mass', native_requirements=fits)
    receipt = json.loads((native/'native-build.json').read_text(encoding='utf-8'))
    placed_document = json.loads((native/'placed-native-parts.json').read_text(encoding='utf-8'))
    fit_document = json.loads((native/'native-interface-requirements.json').read_text(encoding='utf-8'))
    construction_record = json.loads((native/'qualification-construction.json').read_text(encoding='utf-8'))
    bad_material_placements = deepcopy(placed_document)
    bad_material_placements['records'][0]['material'] = 'other-material'
    with pytest.raises(QualificationError, match='material differs from plan'):
        _recheck_qualified_native_steps(native, bad_material_placements, fit_document,
                                        receipt, plan, construction_record)
    forged_pair_receipt = deepcopy(construction_record)
    forged_pair_receipt['exact_placed_interface_checks']['pin_fit']['minimum_distance_mm'] = 5.
    with pytest.raises(QualificationError, match='independently measured fit'):
        _recheck_qualified_native_steps(native, placed_document, fit_document,
                                        receipt, plan, forged_pair_receipt)
    real_mass = json.loads((native/'mass_properties.json').read_text(encoding='utf-8'))
    lighter_aggregate = deepcopy(real_mass)
    lighter_aggregate['moving']['mass_kg'] *= .5
    original_mass_bytes = (native/'mass_properties.json').read_bytes()
    (native/'mass_properties.json').write_text(json.dumps(lighter_aggregate), encoding='utf-8')
    changed_receipt = deepcopy(receipt)
    changed_receipt['files_sha256']['mass_properties.json'] = hashlib.sha256(
        (native/'mass_properties.json').read_bytes()).hexdigest()
    with pytest.raises(QualificationError, match='aggregate mass_kg differs'):
        _recheck_qualified_native_steps(native, placed_document, fit_document,
                                        changed_receipt, plan, construction_record)
    (native/'mass_properties.json').write_bytes(original_mass_bytes)
    fake_model = SimpleNamespace(mass_records=deepcopy(real_mass))
    _require_physical_mass_binding(fake_model, real_mass)
    fake_model.mass_records['moving']['mass_kg'] *= .5
    with pytest.raises(QualificationError, match='inertia differs'):
        _require_physical_mass_binding(fake_model, real_mass)
    binding = {'schema': 'test-only binding', 'source_files': source_files}
    step_hashes = {f"{row['body']}__{row['name']}": hashlib.sha256((native/row['step_file']).read_bytes()).hexdigest()
                   for row in json.loads((native/'placed-native-parts.json').read_text())['records']}

    run = tmp_path/'run'
    (run/'simulation').mkdir(parents=True)
    model = {'schema': 'test-only-physical', 'mass_records': real_mass}
    control = {'schema': 'test-only-controller'}
    runtime_identity = _runtime_identity()
    simulation = dict(schema='cybrgeo-causal-simulation/1', model_sha256=digest(model),
                      control_sha256=digest(control), runtime_implementation=runtime_identity,
                      mujoco_version='test-only', options={},
                      metrics={'actuators': {'output': {
                          'peak_output_torque_Nm': .5,
                          'belt_demand_limit_s': 0., 'gear_demand_limit_s': 0.}}})
    (run/'physical-source.json').write_text(json.dumps(model), encoding='utf-8')
    (run/'control-source.json').write_text(json.dumps(control), encoding='utf-8')
    (run/'simulation'/'simulation.json').write_text(json.dumps(simulation), encoding='utf-8')
    (run/'simulation'/'trace.json.gz').write_bytes(b'TEST-ONLY-TRACE')
    report = dict(schema='cybrgeo-engineering-result/2', name='synthetic', passed=True,
                  source_files=source_files, native={'directory': str(native)},
                  native_build_sha256=digest(receipt), native_source_binding_sha256=digest(binding),
                  model_sha256=digest(model), control_sha256=digest(control), simulation=simulation)
    (run/'engineering.json').write_text(json.dumps(report), encoding='utf-8')
    basis = dict(source_sha256=digest(source_files), plan_sha256=digest(plan),
                 native_build_sha256=digest(receipt), native_source_binding_sha256=digest(binding),
                 model_sha256=digest(model), control_sha256=digest(control),
                 engineering_report_sha256=hashlib.sha256((run/'engineering.json').read_bytes()).hexdigest(),
                 simulation_sha256=hashlib.sha256((run/'simulation'/'simulation.json').read_bytes()).hexdigest(),
                 integration_trace_sha256=hashlib.sha256((run/'simulation'/'trace.json.gz').read_bytes()).hexdigest(),
                 runtime_sha256=digest(dict(runtime=runtime_identity, mujoco_version='test-only', options={})))
    package, trust, _, _, _ = _signed_fixture(tmp_path, basis=digest(basis), step_hashes=step_hashes)

    monkeypatch.setattr(engineering_source, 'project_directory', lambda source: project_dir)
    monkeypatch.setattr(engineering_source, 'load_project',
                        lambda source: (SimpleNamespace(name='synthetic', specification={'test': True}), source_files))
    def verify_native(directory, specification, **kwargs):
        actual = json.loads((native/'native-build.json').read_text(encoding='utf-8'))
        for relative, expected in actual['files_sha256'].items():
            assert hashlib.sha256((native/relative).read_bytes()).hexdigest() == expected
        return actual
    monkeypatch.setattr(engineering_source, 'verify_native_build', verify_native)
    monkeypatch.setattr(engineering_source, 'verify_native_source_binding', lambda *args, **kwargs: binding)
    monkeypatch.setattr(engineering_source, 'assert_project_source_identity',
                        lambda source, expected: expected if expected == source_files else (_ for _ in ()).throw(ValueError()))
    from mechanism_lab import physical_model
    monkeypatch.setattr(physical_model.PhysicalModel, 'from_source',
                        lambda source: SimpleNamespace(mass_records=source['mass_records']))

    product = tmp_path/'product'
    manifest = release_project(project_dir, run, package, trust, product)
    assert manifest['status'] == 'qualified_for_declared_envelope_under_external_evidence'
    assert (product/'qualified-release.json').is_file()
    assert len(list((product/'parts').glob('*.step'))) == 2
    assert len(list((product/'evidence').glob('test-record-*.txt'))) == 8

    original_copy = qualification.shutil.copyfile
    def corrupted_step_copy(source, target, *args, **kwargs):
        result = original_copy(source, target, *args, **kwargs)
        if Path(source).suffix == '.step':
            Path(target).write_bytes(b'CORRUPTED-STEP')
        return result
    monkeypatch.setattr(qualification.shutil, 'copyfile', corrupted_step_copy)
    failed = tmp_path/'product-failed'
    with pytest.raises(QualificationError, match='changed while copying'):
        release_project(project_dir, run, package, trust, failed)
    assert not failed.exists()
    assert not list(tmp_path.glob('.product-failed-stage-*'))
    monkeypatch.setattr(qualification.shutil, 'copyfile', original_copy)
    original_sha = qualification._sha
    service_reads = [0]
    def changing_release_policy(path):
        if Path(path).resolve() == Path(qualification.__file__).resolve():
            service_reads[0] += 1
            if service_reads[0] > 1:
                return '0'*64
        return original_sha(path)
    monkeypatch.setattr(qualification, '_sha', changing_release_policy)
    changed_policy_target = tmp_path/'product-changed-policy'
    with pytest.raises(QualificationError, match='changed during release copy'):
        release_project(project_dir, run, package, trust, changed_policy_target)
    assert service_reads[0] >= 2
    assert not changed_policy_target.exists()
    assert not list(tmp_path.glob('.product-changed-policy-stage-*'))
    monkeypatch.setattr(qualification, '_sha', original_sha)
    from mechanism_lab import causal_runtime
    actual_runtime = causal_runtime._runtime_identity
    def changed_runtime():
        identity = deepcopy(actual_runtime())
        identity['source_files']['actuation.py'] = '0'*64
        return identity
    monkeypatch.setattr(causal_runtime, '_runtime_identity', changed_runtime)
    changed_runtime_target = tmp_path/'product-changed-runtime'
    with pytest.raises(QualificationError, match='Causal runtime source'):
        release_project(project_dir, run, package, trust, changed_runtime_target)
    assert not changed_runtime_target.exists()
    monkeypatch.setattr(causal_runtime, '_runtime_identity', actual_runtime)
    def change_runtime_during_copy(source, target, *args, **kwargs):
        result = original_copy(source, target, *args, **kwargs)
        if Path(source).suffix == '.step':
            monkeypatch.setattr(causal_runtime, '_runtime_identity', changed_runtime)
        return result
    monkeypatch.setattr(qualification.shutil, 'copyfile', change_runtime_during_copy)
    late_runtime_target = tmp_path/'product-late-runtime'
    with pytest.raises(QualificationError, match='Causal runtime source'):
        release_project(project_dir, run, package, trust, late_runtime_target)
    assert not late_runtime_target.exists()
    assert not list(tmp_path.glob('.product-late-runtime-stage-*'))
