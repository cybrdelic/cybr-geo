"""Fail-closed manufacturing release over the shared native construction path.

Simulation and diagnostic CAD remain useful without being hardware evidence. A
qualified build uses ``QualifiedNativeConstruction`` *before* each placement;
release later requires independently signed, revision-bound material, process,
inspection, interface, load and assembled-hardware records. No trust root is
bundled with this library or its examples.
"""
from __future__ import annotations

import base64
import binascii
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import tempfile

from .native_construction import NativeConstruction, _shape, _QUALIFICATION_RECEIPT_CAPABILITY
from .physical_model import digest


class QualificationError(ValueError):
    """A manufacturing release requirement is absent, invalid or stale."""


def _object(value, label):
    if not isinstance(value, dict) or not value:
        raise QualificationError(f'{label} must be a nonempty object')
    return value


def _text(value, label):
    if not isinstance(value, str) or not value.strip():
        raise QualificationError(f'{label} must be nonempty text')
    return value


def _number(value, label, *, positive=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value < 0 or (positive and value == 0)):
        raise QualificationError(f'{label} must be finite and {"positive" if positive else "nonnegative"}')
    return float(value)


def _keys(value, expected, label):
    if set(value) != set(expected):
        raise QualificationError(f'{label} fields must be exactly {sorted(expected)}')


def _safe_id(value, label):
    _text(value, label)
    if value in ('.', '..') or any(char in value for char in '/\\:*?"<>|'):
        raise QualificationError(f'{label} must be an export-safe identity')
    return value


def _safe_part_export_id(value):
    """Constrain native body__part stems to portable, unambiguous STEP names."""
    _safe_id(value, 'Part ID')
    tokens = value.split('__')
    reserved = {'CON', 'PRN', 'AUX', 'NUL'} | {f'{prefix}{number}' for prefix in ('COM', 'LPT') for number in range(1, 10)}
    if (len(tokens) != 2 or len(value) > 120
            or any(not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', token)
                   or token.endswith('.') or token.upper() in reserved for token in tokens)):
        raise QualificationError('Part ID must have one portable body__part export stem')
    return value


def _read_json(path):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError) as error:
        raise QualificationError(f'Missing or invalid JSON: {path}') from error


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


@dataclass(frozen=True)
class QualificationPlan:
    """Typed construction contract checked before any native placement export.

    The plan is a project ``data/qualification-plan.json`` source input. Its
    material and process numbers are design limits, never proof of capacity.
    Independent signed measurements must confirm them at release.
    """
    document: dict

    def __post_init__(self):
        row = self.document
        _keys(row, ('schema', 'parts', 'materials', 'processes', 'interfaces', 'loads',
                    'assembly_requirements'), 'Qualification plan')
        if row['schema'] != 'cybrgeo-qualification-plan/1':
            raise QualificationError('Expected cybrgeo-qualification-plan/1')
        parts = _object(row['parts'], 'Parts')
        materials = _object(row['materials'], 'Materials')
        processes = _object(row['processes'], 'Processes')
        interfaces = _object(row['interfaces'], 'Interfaces')
        loads = _object(row['loads'], 'Loads')
        for name, material in materials.items():
            _safe_id(name, 'Material ID')
            _keys(material, ('grade', 'lot', 'density_kg_m3', 'allowable_stress_MPa',
                             'temperature_limit_C'), f'Material {name}')
            _text(material['grade'], 'Material grade')
            _text(material['lot'], 'Material lot')
            _number(material['density_kg_m3'], 'Material density', positive=True)
            _number(material['allowable_stress_MPa'], 'Material allowable stress', positive=True)
            _number(material['temperature_limit_C'], 'Material temperature limit', positive=True)
        for name, process in processes.items():
            _safe_id(name, 'Process ID')
            _keys(process, ('method', 'facility', 'revision', 'allowed_materials',
                            'minimum_feature_mm', 'capability_tolerance_mm',
                            'maximum_part_volume_mm3'), f'Process {name}')
            for field in ('method', 'facility', 'revision'):
                _text(process[field], f'Process {field}')
            allowed = process['allowed_materials']
            if not isinstance(allowed, list) or not allowed or len(set(allowed)) != len(allowed) or not set(allowed) <= materials.keys():
                raise QualificationError(f'Process {name} has invalid allowed materials')
            for field in ('minimum_feature_mm', 'capability_tolerance_mm', 'maximum_part_volume_mm3'):
                _number(process[field], f'Process {field}', positive=True)
        export_stems = set()
        for name, part in parts.items():
            _safe_part_export_id(name)
            if name.casefold() in export_stems:
                raise QualificationError('Part IDs collide on case-insensitive export filesystems')
            export_stems.add(name.casefold())
            _keys(part, ('material', 'process', 'nominal_minimum_wall_mm',
                         'required_dimensional_tolerance_mm', 'maximum_volume_mm3'), f'Part {name}')
            material, process = part['material'], part['process']
            if material not in materials or process not in processes or material not in processes[process]['allowed_materials']:
                raise QualificationError(f'Part {name} has unsupported material/process combination')
            wall = _number(part['nominal_minimum_wall_mm'], 'Part wall', positive=True)
            tolerance = _number(part['required_dimensional_tolerance_mm'], 'Part tolerance', positive=True)
            volume = _number(part['maximum_volume_mm3'], 'Part volume', positive=True)
            if (wall < processes[process]['minimum_feature_mm']
                    or tolerance < processes[process]['capability_tolerance_mm']
                    or volume > processes[process]['maximum_part_volume_mm3']):
                raise QualificationError(f'Part {name} exceeds declared process capabilities')
        for name, interface in interfaces.items():
            _safe_id(name, 'Interface ID')
            _keys(interface, ('parts', 'nominal_clearance_mm', 'first_tolerance_mm',
                              'second_tolerance_mm', 'required_clearance_mm',
                              'demand_N', 'design_capacity_N'), f'Interface {name}')
            pair = interface['parts']
            if (not isinstance(pair, list) or len(pair) != 2 or pair[0] == pair[1]
                    or not set(pair) <= parts.keys()):
                raise QualificationError(f'Interface {name} must connect two declared parts')
            clearance = _number(interface['nominal_clearance_mm'], 'Interface nominal clearance')
            first = _number(interface['first_tolerance_mm'], 'Interface first tolerance')
            second = _number(interface['second_tolerance_mm'], 'Interface second tolerance')
            required = _number(interface['required_clearance_mm'], 'Interface required clearance')
            demand = _number(interface['demand_N'], 'Interface demand', positive=True)
            capacity = _number(interface['design_capacity_N'], 'Interface capacity', positive=True)
            if (first < parts[pair[0]]['required_dimensional_tolerance_mm']
                    or second < parts[pair[1]]['required_dimensional_tolerance_mm']):
                raise QualificationError(f'Interface {name} tolerances understate participating part variation')
            if clearance - first - second < required or capacity < demand:
                raise QualificationError(f'Interface {name} fails its tolerance or load requirement')
        for name, load in loads.items():
            _safe_id(name, 'Load ID')
            _keys(load, ('part', 'demand_Nm', 'design_capacity_Nm', 'service_temperature_C'), f'Load {name}')
            if load['part'] not in parts:
                raise QualificationError(f'Load {name} refers to an undeclared part')
            demand = _number(load['demand_Nm'], 'Load torque demand', positive=True)
            capacity = _number(load['design_capacity_Nm'], 'Load torque capacity', positive=True)
            temperature = _number(load['service_temperature_C'], 'Load service temperature')
            if capacity < demand or temperature > materials[parts[load['part']]['material']]['temperature_limit_C']:
                raise QualificationError(f'Load {name} exceeds a declared capacity')
        assembly = _object(row['assembly_requirements'], 'Assembly requirements')
        _keys(assembly, ('minimum_endurance_cycles', 'minimum_thermal_test_C',
                         'maximum_measured_backlash_mm'), 'Assembly requirements')
        cycles = assembly['minimum_endurance_cycles']
        if isinstance(cycles, bool) or not isinstance(cycles, int) or cycles < 2:
            raise QualificationError('Assembly endurance requirement must declare at least two cycles')
        tested_temperature = _number(assembly['minimum_thermal_test_C'], 'Assembly thermal test temperature')
        _number(assembly['maximum_measured_backlash_mm'], 'Assembly backlash limit', positive=True)
        if tested_temperature < max(load['service_temperature_C'] for load in loads.values()):
            raise QualificationError('Assembly thermal test requirement must cover the load service temperature')

    @property
    def sha256(self):
        return digest(self.document)

    def check_placement(self, body, label, material, shape, densities):
        name = f'{body}__{label}'
        _safe_part_export_id(name)
        part = self.document['parts'].get(name)
        if part is None or part['material'] != material:
            raise QualificationError(f'Placement {name} lacks its exact qualified material/process declaration')
        declared_density = self.document['materials'][material]['density_kg_m3']
        if material not in densities or not math.isclose(densities[material], declared_density, rel_tol=1e-12):
            raise QualificationError(f'Placement {name} density differs from its qualified material')
        native = _shape(shape)
        if not native.isValid() or not native.Solids():
            raise QualificationError(f'Placement {name} requires valid native CAD')
        volume = _number(float(native.Volume()), f'Placement {name} CAD volume', positive=True)
        if volume > part['maximum_volume_mm3']:
            raise QualificationError(f'Placement {name} exceeds its process volume envelope')


class QualifiedNativeConstruction(NativeConstruction):
    """Opt-in native constructor that rejects invalid declarations before CAD export.

    Plain ``NativeConstruction`` is a diagnostic tool. Only this constructor
    emits a source-bound qualification-construction receipt. Its presence is
    required, but is insufficient, for a hardware release.
    """

    def __init__(self, output, densities, body_ids=(), provenance=None, *, qualification_plan):
        document = (qualification_plan.document if isinstance(qualification_plan, QualificationPlan)
                    else qualification_plan)
        self.qualification_plan = QualificationPlan(deepcopy(document))
        self._qualified_placements = set()
        self._placing_qualified_part = False
        if set(densities) != set(self.qualification_plan.document['materials']):
            raise QualificationError('Qualified construction must declare every material density exactly')
        super().__init__(output, densities, body_ids, provenance)

    def addpart(self, *args, **kwargs):
        raise QualificationError('Qualified construction requires named direct placements; unbound CAD templates are diagnostic')

    def add_mass_record(self, body, record):
        if not self._placing_qualified_part:
            raise QualificationError('Qualified construction cannot add unplaced or overridden body mass')
        return super().add_mass_record(body, record)

    def add_assembly_mass(self, *args, **kwargs):
        raise QualificationError('Qualified construction cannot add mass-only assembly records')

    def save_auxiliary_assembly(self, *args, **kwargs):
        raise QualificationError('Qualified construction cannot export unbound auxiliary CAD')

    def place(self, body, label, shape, material='aluminum', mass=None, note='CAD density', *,
              linear_tolerance_mm=.18, angular_tolerance_rad=.22):
        if mass is not None:
            raise QualificationError('Qualified construction cannot override a placed CAD material mass')
        name = f'{body}__{label}'
        if name in self._qualified_placements:
            raise QualificationError(f'Qualified placement {name} was already exported')
        self.qualification_plan.check_placement(body, label, material, shape, self.densities)
        self._placing_qualified_part = True
        try:
            super().place(body, label, shape, material, mass, note,
                          linear_tolerance_mm=linear_tolerance_mm, angular_tolerance_rad=angular_tolerance_rad)
        finally:
            self._placing_qualified_part = False
        self._qualified_placements.add(name)

    def finish(self, body_status, extra_records=None, *, native_requirements=None):
        plan = self.qualification_plan.document
        if self._qualified_placements != set(plan['parts']):
            raise QualificationError('Qualified construction part inventory differs from placed native CAD')
        native_names = [f"{row['body']}__{row['name']}" for row in self.placed_records]
        if (len(native_names) != len(plan['parts']) or set(native_names) != set(plan['parts'])
                or len({name.casefold() for name in native_names}) != len(native_names)):
            raise QualificationError('Qualified placed STEP inventory is ambiguous or incomplete')
        if native_requirements is None:
            raise QualificationError('Qualified construction requires named native interface measurements')
        report = native_requirements.report()
        if not report.get('passed') or {check['name'] for check in report.get('checks', [])} != set(plan['interfaces']):
            raise QualificationError('Qualified construction must measure every declared interface exactly')
        import cadquery as cq
        exported = {}
        for row in self.placed_records:
            name = f"{row['body']}__{row['name']}"
            path = (self.output / row['step_file']).resolve()
            if not path.is_relative_to(self.output) or not path.is_file():
                raise QualificationError(f'Qualified placed STEP is missing or outside its native output: {name}')
            imported = _shape(cq.importers.importStep(str(path)))
            original = _shape(self.placed_shapes[row['body'], row['name']])
            if not imported.isValid() or not imported.Solids():
                raise QualificationError(f'Qualified placed STEP is invalid: {name}')
            # STEP round-trip arithmetic tolerance, independent of every
            # dimensional manufacturing allowance declared in the plan.
            difference_mm3 = float(original.cut(imported).Volume() + imported.cut(original).Volume())
            volume_mm3 = float(original.Volume())
            if (not math.isfinite(difference_mm3) or not math.isfinite(volume_mm3)
                    or difference_mm3 > max(1e-6, volume_mm3 * 1e-8)):
                raise QualificationError(f'Qualified placed STEP differs from constructed CAD: {name}')
            exported[name] = imported
        measured_interfaces = {}
        for check in report['checks']:
            name = check['name']
            interface = plan['interfaces'][name]
            first, second = (exported[part] for part in interface['parts'])
            measured = _number(float(first.distance(second)), f'Native interface {name} clearance')
            overlap = _number(float(first.intersect(second).Volume()), f'Native interface {name} overlap')
            # These are fixed CAD arithmetic allowances, never process fits.
            distance_kernel_allowance_mm = 1e-7
            volume_kernel_allowance_mm3 = 1e-6
            claimed = _number(check.get('minimum_distance_mm'), f'Native interface {name} claimed clearance')
            claimed_overlap = _number(check.get('intersection_volume_mm3'), f'Native interface {name} claimed overlap')
            if (abs(claimed - measured) > distance_kernel_allowance_mm
                    or abs(claimed_overlap - overlap) > volume_kernel_allowance_mm3):
                raise QualificationError(f'Native interface {name} receipt measured different solids than its placed part pair')
            worst = measured - interface['first_tolerance_mm'] - interface['second_tolerance_mm']
            if (overlap > volume_kernel_allowance_mm3
                    or measured + distance_kernel_allowance_mm < interface['nominal_clearance_mm']
                    or worst + distance_kernel_allowance_mm < interface['required_clearance_mm']):
                raise QualificationError(f'Placed CAD parts fail qualified interface {name} clearance/tolerance')
            measured_interfaces[name] = dict(parts=list(interface['parts']), minimum_distance_mm=measured,
                                             intersection_volume_mm3=overlap, worst_case_clearance_mm=worst,
                                             distance_kernel_allowance_mm=distance_kernel_allowance_mm,
                                             volume_kernel_allowance_mm3=volume_kernel_allowance_mm3)
        records = dict(extra_records or {})
        if 'qualification-construction.json' in records:
            raise QualificationError('Qualification construction receipt is reserved')
        records['qualification-construction.json'] = dict(
            schema='cybrgeo-qualification-construction/1', plan_sha256=self.qualification_plan.sha256,
            qualification_service_sha256=_sha(__file__),
            placements=sorted(self._qualified_placements), interface_names=sorted(plan['interfaces']),
            exact_placed_interface_checks=measured_interfaces,
            status='declared_construction_constraints_passed; actual_wall_fit_load_hardware_evidence_pending')
        return super().finish(body_status, records, native_requirements=native_requirements,
                              _receipt_capability=_QUALIFICATION_RECEIPT_CAPABILITY)


EVIDENCE_KINDS = ('system_service_envelope', 'material', 'process', 'part_inspection',
                  'interface_test', 'load_test', 'assembly_test')


def _artifact(root, relative):
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute() or '..' in Path(relative).parts:
        raise QualificationError('Evidence document must use a contained relative path')
    candidate = root / relative
    if any(path.is_symlink() for path in (candidate, *candidate.parents) if path.is_relative_to(root)):
        raise QualificationError('Evidence documents cannot be symbolic links')
    path = candidate.resolve()
    if not path.is_relative_to(root) or not path.is_file():
        raise QualificationError('Evidence document is missing, linked or outside its package')
    return path


def _verify_evidence(package, trust_policy, basis_sha256, plan, part_step_sha256):
    """Verify detached Ed25519 signatures under an externally supplied trust policy."""
    try:
        from cryptography.exceptions import InvalidSignature
        from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey
    except ImportError as error:
        raise QualificationError('Release requires the cryptography Ed25519 verifier') from error
    policy = _read_json(trust_policy)
    if policy.get('schema') != 'cybrgeo-trust-policy/1':
        raise QualificationError('Expected externally configured cybrgeo-trust-policy/1')
    issuers = _object(policy.get('issuers'), 'Trusted issuers')
    envelope = _read_json(package / 'evidence.json')
    if set(envelope) != {'schema', 'records'} or envelope['schema'] != 'cybrgeo-signed-evidence-set/1' or not isinstance(envelope['records'], list):
        raise QualificationError('Expected cybrgeo-signed-evidence-set/1')
    expected = ({('material', name) for name in plan['materials']}
                | {('process', name) for name in plan['processes']}
                | {('part_inspection', name) for name in plan['parts']}
                | {('interface_test', name) for name in plan['interfaces']}
                | {('load_test', name) for name in plan['loads']}
                | {('assembly_test', 'complete_assembly')}
                | {('system_service_envelope', 'complete_assembly')})
    seen = {}
    for row in envelope['records']:
        if not isinstance(row, dict):
            raise QualificationError('Signed evidence record must be an object')
        _keys(row, ('schema', 'kind', 'subject', 'basis_sha256', 'result', 'measurements',
                    'document', 'document_sha256', 'issuer', 'signature_base64'), 'Signed evidence')
        if row['schema'] != 'cybrgeo-signed-evidence/1' or row['kind'] not in EVIDENCE_KINDS or row['result'] != 'observed_pass':
            raise QualificationError('Evidence kind, schema or observed result is invalid')
        key = (row['kind'], row['subject'])
        if key not in expected or key in seen or row['basis_sha256'] != basis_sha256:
            raise QualificationError('Evidence subject is extra, duplicate or bound to a stale revision')
        issuer = issuers.get(row['issuer'])
        if (not isinstance(issuer, dict) or set(issuer) != {'public_key_base64', 'allowed_kinds'}
                or not isinstance(issuer['allowed_kinds'], list) or row['kind'] not in issuer['allowed_kinds']):
            raise QualificationError('Evidence issuer is not trusted for this kind of physical record')
        document = _artifact(package, row['document'])
        if _sha(document) != row['document_sha256']:
            raise QualificationError('Evidence report bytes differ from the signed digest')
        signed = {name: value for name, value in row.items() if name != 'signature_base64'}
        payload = json.dumps(signed, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
        try:
            public = Ed25519PublicKey.from_public_bytes(base64.b64decode(issuer['public_key_base64'], validate=True))
            public.verify(base64.b64decode(row['signature_base64'], validate=True), payload)
        except (ValueError, binascii.Error, InvalidSignature) as error:
            raise QualificationError('Evidence signature is invalid or uses an untrusted key') from error
        seen[key] = row['measurements']
    if set(seen) != expected:
        raise QualificationError('Manufacturing evidence does not cover every required subject')
    service = _object(seen['system_service_envelope', 'complete_assembly'], 'Signed service envelope')
    if service.get('load_paths_reviewed') is not True:
        raise QualificationError('An independent source-bound load-path review is required')
    paths = _object(service.get('mechanical_load_paths'), 'Signed mechanical load paths')
    if set(paths) != set(plan['loads']):
        raise QualificationError('Signed load paths must cover every declared active load')
    covered_interfaces = set()
    for load_name, path in paths.items():
        path = _object(path, f'{load_name} signed load path')
        _keys(path, ('parts', 'interfaces'), f'{load_name} signed load path')
        members, mates = path['parts'], path['interfaces']
        if (not isinstance(members, list) or not isinstance(mates, list)
                or len(members) < 2 or len(mates) != len(members)-1
                or len(set(members)) != len(members) or len(set(mates)) != len(mates)
                or members[0] != plan['loads'][load_name]['part']
                or not set(members) <= set(plan['parts'])
                or not set(mates) <= set(plan['interfaces'])):
            raise QualificationError(f'Signed {load_name} path is missing a distinct reaction part or has invalid members')
        for first, second, mate in zip(members, members[1:], mates):
            if set(plan['interfaces'][mate]['parts']) != {first, second}:
                raise QualificationError(f'Signed {load_name} path is disconnected at {mate}')
        covered_interfaces.update(mates)
    if covered_interfaces != set(plan['interfaces']):
        raise QualificationError('Signed load paths omit a declared load-bearing interface')
    actuator_demands = _object(service.get('actuator_demand_Nm'), 'Signed actuator demands')
    interface_demands = _object(service.get('interface_demand_N'), 'Signed interface demands')
    if set(actuator_demands) != set(plan['loads']) or set(interface_demands) != set(plan['interfaces']):
        raise QualificationError('Signed service envelope must cover every actuator and physical interface')
    for name, demand in actuator_demands.items():
        if _number(demand, f'{name} signed torque demand', positive=True) > plan['loads'][name]['demand_Nm']:
            raise QualificationError(f'Plan understates the independent torque demand for {name}')
    for name, demand in interface_demands.items():
        if _number(demand, f'{name} signed force demand', positive=True) > plan['interfaces'][name]['demand_N']:
            raise QualificationError(f'Plan understates the independent interface force demand for {name}')
    if (_number(service.get('required_thermal_test_C'), 'Signed thermal demand') > plan['assembly_requirements']['minimum_thermal_test_C']
            or _number(service.get('required_endurance_cycles'), 'Signed endurance demand', positive=True) > plan['assembly_requirements']['minimum_endurance_cycles']
            or _number(service.get('maximum_backlash_mm'), 'Signed backlash limit') < plan['assembly_requirements']['maximum_measured_backlash_mm']):
        raise QualificationError('Plan understates independent thermal, endurance or backlash requirements')
    for name, material in plan['materials'].items():
        row = _object(seen['material', name], 'Material measurements')
        if (row.get('grade') != material['grade'] or row.get('lot') != material['lot']
                or _number(row.get('density_kg_m3'), 'Measured material density', positive=True) != material['density_kg_m3']
                or _number(row.get('allowable_stress_MPa'), 'Measured stress capacity', positive=True) < material['allowable_stress_MPa']
                or _number(row.get('temperature_limit_C'), 'Measured temperature limit', positive=True) < material['temperature_limit_C']):
            raise QualificationError(f'Material {name} grade, lot or measured capacity does not meet the plan')
    for name, process in plan['processes'].items():
        row = _object(seen['process', name], 'Process measurements')
        if (row.get('facility') != process['facility'] or row.get('revision') != process['revision']
                or row.get('method') != process['method']
                or _number(row.get('minimum_feature_mm'), 'Measured process feature', positive=True) > process['minimum_feature_mm']
                or _number(row.get('capability_tolerance_mm'), 'Measured process tolerance', positive=True) > process['capability_tolerance_mm']
                or _number(row.get('maximum_part_volume_mm3'), 'Measured process volume', positive=True) < process['maximum_part_volume_mm3']):
            raise QualificationError(f'Process {name} measured capability does not meet the plan')
    assembly = _object(seen['assembly_test', 'complete_assembly'], 'Assembly measurements')
    assembled_serial = _text(assembly.get('assembled_serial'), 'Assembled unit serial')
    assembled_part_serials = _object(assembly.get('part_serials'), 'Assembled unit part serials')
    if set(assembled_part_serials) != set(plan['parts']):
        raise QualificationError('Signed assembled-unit BOM must cover every planned part')
    inspected_serials = {}
    for name, part in plan['parts'].items():
        row = _object(seen['part_inspection', name], 'Part inspection')
        material = plan['materials'][part['material']]
        process = plan['processes'][part['process']]
        if (row.get('material_lot') != material['lot'] or row.get('process_revision') != process['revision']
                or row.get('process_facility') != process['facility']
                or row.get('process_method') != process['method']
                or row.get('step_sha256') != part_step_sha256[name]
                or not isinstance(row.get('fabricated_serial'), str) or not row['fabricated_serial']
                or _number(row.get('measured_minimum_wall_mm'), 'Measured wall', positive=True) < part['nominal_minimum_wall_mm']
                or _number(row.get('maximum_dimensional_error_mm'), 'Measured part error') > part['required_dimensional_tolerance_mm']):
            raise QualificationError(f'Part {name} fabrication or inspection fails its declaration')
        inspected_serials[name] = row['fabricated_serial']
    if (len(set(inspected_serials.values())) != len(inspected_serials)
            or assembled_part_serials != inspected_serials):
        raise QualificationError('Assembled-unit BOM differs from the unique inspected part serials')
    for name, interface in plan['interfaces'].items():
        row = _object(seen['interface_test', name], 'Interface measurements')
        pair_serials = {part: inspected_serials[part] for part in interface['parts']}
        if (row.get('assembled_serial') != assembled_serial
                or row.get('part_serials') != pair_serials
                or _number(row.get('measured_minimum_clearance_mm'), 'Measured clearance') < interface['required_clearance_mm']
                or _number(row.get('tested_capacity_N'), 'Tested interface capacity', positive=True) < interface['design_capacity_N']):
            raise QualificationError(f'Interface {name} unit, part serials, clearance or capacity fail')
    for name, load in plan['loads'].items():
        row = _object(seen['load_test', name], 'Load measurements')
        if (row.get('assembled_serial') != assembled_serial
                or row.get('part_serial') != inspected_serials[load['part']]
                or _number(row.get('tested_capacity_Nm'), 'Tested load capacity', positive=True) < load['design_capacity_Nm']
                or _number(row.get('tested_temperature_C'), 'Tested load temperature') < load['service_temperature_C']):
            raise QualificationError(f'Load {name} unit, part serial, capacity or temperature fail')
    required = plan['assembly_requirements']
    if (assembly.get('assembled_serial') in (None, '') or assembly.get('fit_passed') is not True
            or assembly.get('motion_passed') is not True or assembly.get('thermal_passed') is not True
            or _number(assembly.get('tested_cycles'), 'Assembly endurance cycles', positive=True) < required['minimum_endurance_cycles']
            or _number(assembly.get('tested_temperature_C'), 'Assembly thermal test temperature') < required['minimum_thermal_test_C']
            or _number(assembly.get('measured_backlash_mm'), 'Measured assembly backlash') > required['maximum_measured_backlash_mm']):
        raise QualificationError('Actual assembled-hardware fit, motion, thermal and cycle tests are required')
    return dict(evidence_count=len(seen), document_sha256={row['document']: row['document_sha256'] for row in envelope['records']},
                trust_policy_sha256=_sha(trust_policy), evidence_set_sha256=_sha(package / 'evidence.json'))


def _simulated_loads(metrics):
    if not isinstance(metrics, dict) or not isinstance(metrics.get('actuators'), dict):
        raise QualificationError('Simulation lacks an actuator load inventory')
    actuators = dict(metrics['actuators'])
    rack = metrics.get('rack_motor')
    if rack is not None:
        if not isinstance(rack, dict) or 'rack_motor' in actuators:
            raise QualificationError('Simulation rack motor record is invalid or duplicated')
        actuators['rack_motor'] = rack
    return actuators


def _require_current_causal_runtime(recorded):
    """Invalidate evidence when causal code or solver packages have changed.

    The generating host's OS and machine in the signed record remain historical
    facts; requiring the release host to match them would break valid portable
    verification without making the recorded integration more trustworthy.
    """
    from .causal_runtime import _runtime_identity
    current = _runtime_identity()
    if (not isinstance(recorded, dict) or recorded.get('schema') != current['schema']
            or recorded.get('source_files') != current['source_files']
            or recorded.get('packages') != current['packages']):
        raise QualificationError('Causal runtime source or package versions changed since the signed run')


def _numeric_geometry_matches(actual, expected):
    if isinstance(expected, list):
        return (isinstance(actual, list) and len(actual) == len(expected)
                and all(_numeric_geometry_matches(left, right) for left, right in zip(actual, expected)))
    return (isinstance(actual, (int, float)) and not isinstance(actual, bool)
            and isinstance(expected, (int, float)) and not isinstance(expected, bool)
            and math.isfinite(actual) and math.isfinite(expected)
            and math.isclose(actual, expected, rel_tol=1e-6, abs_tol=1e-12))


def _recheck_qualified_native_steps(native, placements, interfaces, receipt, plan, record):
    """Reopen the releasable STEP, independently checking pair fits and mass."""
    import cadquery as cq
    from .native_construction import aggregate_mass_properties, shape_mass_properties

    files = receipt['files_sha256']
    required = ('placed-native-parts.json', 'native-interface-requirements.json',
                'mass_properties.json', 'cad_parts.json')
    if any(files.get(name) != _sha(native / name) for name in required):
        raise QualificationError('Qualified native placement, fit or mass manifests are not source-bound')
    if receipt.get('parts') != 0 or receipt.get('placements') != len(plan['parts']) or _read_json(native / 'cad_parts.json'):
        raise QualificationError('Qualified native release contains unbound CAD parts or placements')
    mass = _read_json(native / 'mass_properties.json')
    bodies = {row['body'] for row in placements['records']}
    if set(mass) != bodies:
        raise QualificationError('Qualified native body mass inventory differs from placed parts')
    components = {}
    for body, aggregate in mass.items():
        rows = aggregate.get('components')
        if not isinstance(rows, list):
            raise QualificationError(f'Qualified body {body} lacks component mass records')
        components[body] = {row.get('name'): row for row in rows if isinstance(row, dict)}
        if len(components[body]) != len(rows):
            raise QualificationError(f'Qualified body {body} has duplicate or malformed mass records')
    solids = {}
    for row in placements['records']:
        name = f"{row['body']}__{row['name']}"
        part = plan['parts'][name]
        relative = f'cad/placed/{name}.step'
        path = native / relative
        if (row.get('step_file') != relative or row.get('material') != part['material']
                or files.get(relative) != _sha(path)):
            raise QualificationError(f'Qualified native part {name} source or material differs from plan')
        solid = _shape(cq.importers.importStep(str(path)))
        if not solid.isValid() or not solid.Solids():
            raise QualificationError(f'Qualified native STEP {name} is invalid')
        if _number(float(solid.Volume()), f'{name} STEP volume', positive=True) > part['maximum_volume_mm3']:
            raise QualificationError(f'Qualified native STEP {name} exceeds its process volume')
        component = components[row['body']].get(row['name'])
        if component is None:
            raise QualificationError(f'Qualified native STEP {name} lacks mass provenance')
        actual_mass, center, inertia = shape_mass_properties(
            solid, plan['materials'][part['material']]['density_kg_m3'])
        if (not _numeric_geometry_matches(actual_mass, component.get('mass_kg'))
                or not _numeric_geometry_matches(center.tolist(), component.get('com_m'))
                or not _numeric_geometry_matches(inertia.tolist(), component.get('inertia_at_com_kg_m2'))):
            raise QualificationError(f'Qualified native STEP {name} mass differs from its declared material')
        solids[name] = solid
    for body, rows in components.items():
        if set(rows) != {row['name'] for row in placements['records'] if row['body'] == body}:
            raise QualificationError(f'Qualified body {body} contains unplaced mass components')
        recomputed = aggregate_mass_properties(list(rows.values()))
        for field in ('mass_kg', 'com_m', 'inertia_tensor_kg_m2', 'fullinertia_mujoco_kg_m2'):
            if not _numeric_geometry_matches(recomputed[field], mass[body].get(field)):
                raise QualificationError(f'Qualified body {body} aggregate {field} differs from verified STEP components')
    reported_checks = {row['name']: row for row in interfaces['checks']}
    if len(reported_checks) != len(interfaces['checks']):
        raise QualificationError('Qualified native interface report has duplicate checks')
    for name, interface in plan['interfaces'].items():
        first, second = (solids[part] for part in interface['parts'])
        measured = _number(float(first.distance(second)), f'{name} STEP clearance')
        overlap = _number(float(first.intersect(second).Volume()), f'{name} STEP overlap')
        worst = measured - interface['first_tolerance_mm'] - interface['second_tolerance_mm']
        prior = record['exact_placed_interface_checks'][name]
        reported = reported_checks[name]
        if (not _numeric_geometry_matches(measured, prior['minimum_distance_mm'])
                or not _numeric_geometry_matches(overlap, prior['intersection_volume_mm3'])
                or not _numeric_geometry_matches(measured, reported.get('minimum_distance_mm'))
                or not _numeric_geometry_matches(overlap, reported.get('intersection_volume_mm3'))
                or overlap > 1e-6 or measured + 1e-7 < interface['nominal_clearance_mm']
                or worst + 1e-7 < interface['required_clearance_mm']):
            raise QualificationError(f'Qualified native STEP pair {name} fails independently measured fit')
    return mass


def _require_physical_mass_binding(model, native_mass):
    """Require the actual solver inertials to match every qualified CAD body."""
    from .physical_model import inertia_tensor
    records = getattr(model, 'mass_records', None)
    if not isinstance(records, dict) or set(records) != set(native_mass):
        raise QualificationError('Physical model mass inventory differs from qualified native CAD bodies')
    for body, native in native_mass.items():
        mass, center, tensor = inertia_tensor(records[body])
        if (not _numeric_geometry_matches(mass, native['mass_kg'])
                or not _numeric_geometry_matches(center.tolist(), native['com_m'])
                or not _numeric_geometry_matches(tensor.tolist(), native['inertia_tensor_kg_m2'])):
            raise QualificationError(f'Physical model body {body} inertia differs from qualified native CAD')


def release_project(source, run_directory, evidence_package, trust_policy, output):
    """Copy qualified native STEP only after every source and physical gate passes.

    Evidence signatures prove origin under the *caller configured* trust policy;
    the library cannot establish whether that independent authority actually
    performed a physical test. This operation creates no release directory on
    rejected evidence. Existing diagnostic artifacts are never promoted.
    """
    from .engineering_source import (load_project, project_directory, verify_native_build,
                                     verify_native_source_binding)
    from .physical_model import PhysicalModel

    run = Path(run_directory).resolve()
    package = Path(evidence_package).resolve()
    trust = Path(trust_policy).resolve()
    destination = Path(output).resolve()
    project_dir = project_directory(source)
    if any(trust.is_relative_to(root) for root in (project_dir, run, package)):
        raise QualificationError('Trust policy must be provisioned outside source, run and evidence directories')
    if destination.exists():
        raise FileExistsError('Qualified release destination must be a fresh directory')
    project, source_files = load_project(source)
    plan_path = project_dir / 'data' / 'qualification-plan.json'
    if 'data/qualification-plan.json' not in source_files or not plan_path.is_file():
        raise QualificationError('Project lacks a source-bound qualification construction plan')
    plan = QualificationPlan(_read_json(plan_path))
    report = _read_json(run / 'engineering.json')
    if (report.get('schema') != 'cybrgeo-engineering-result/2' or report.get('passed') is not True
            or report.get('source_files') != source_files or report.get('name') != project.name):
        raise QualificationError('A current accepted software engineering run is required')
    native = Path(report['native']['directory']).resolve()
    receipt = verify_native_build(native, project.specification, source_files=source_files)
    binding = verify_native_source_binding(native, project.specification, source_files, receipt=receipt)
    if (report.get('native_build_sha256') != digest(receipt)
            or report.get('native_source_binding_sha256') != digest(binding)):
        raise QualificationError('Engineering result differs from current native source/bindings')
    record = _read_json(native / 'qualification-construction.json')
    if ('qualification-construction.json' not in receipt['files_sha256']
            or receipt['files_sha256']['qualification-construction.json'] != _sha(native / 'qualification-construction.json')
            or record.get('schema') != 'cybrgeo-qualification-construction/1'
            or record.get('plan_sha256') != plan.sha256
            or record.get('qualification_service_sha256') != _sha(__file__)
            or set(record.get('placements', ())) != set(plan.document['parts'])
            or set(record.get('interface_names', ())) != set(plan.document['interfaces'])):
        raise QualificationError('Native CAD was not constructed through the current qualification policy')
    exact_interfaces = record.get('exact_placed_interface_checks')
    if not isinstance(exact_interfaces, dict) or set(exact_interfaces) != set(plan.document['interfaces']):
        raise QualificationError('Qualified native receipt lacks exact placed-part interface measurements')
    for name, measured in exact_interfaces.items():
        declared = plan.document['interfaces'][name]
        if (measured.get('parts') != declared['parts']
                or _number(measured.get('intersection_volume_mm3'), 'Qualified CAD overlap') > 1e-6
                or _number(measured.get('minimum_distance_mm'), 'Qualified CAD gap') + 1e-7 < declared['nominal_clearance_mm']
                or _number(measured.get('worst_case_clearance_mm'), 'Qualified worst-case gap') + 1e-7 < declared['required_clearance_mm']):
            raise QualificationError(f'Qualified native interface {name} fails its exact part-pair plan')
    placements = _read_json(native / 'placed-native-parts.json')
    interfaces = _read_json(native / 'native-interface-requirements.json')
    placed_names = [f"{row['body']}__{row['name']}" for row in placements['records']]
    if (placements.get('units') != 'mm' or len(placed_names) != len(plan.document['parts'])
            or set(placed_names) != set(plan.document['parts'])
            or len({name.casefold() for name in placed_names}) != len(placed_names)
            or not interfaces.get('passed')
            or {row['name'] for row in interfaces.get('checks', ())} != set(plan.document['interfaces'])):
        raise QualificationError('Native CAD part/interface inventory differs from the qualification plan')
    if set(receipt.get('material_densities_kg_m3', {})) != set(plan.document['materials']):
        raise QualificationError('Native material inventory differs from the qualification plan')
    for name, material in plan.document['materials'].items():
        if receipt['material_densities_kg_m3'][name] != material['density_kg_m3']:
            raise QualificationError('Native material density differs from the qualification plan')
    native_mass = _recheck_qualified_native_steps(native, placements, interfaces, receipt, plan.document, record)
    physical_source = _read_json(run / 'physical-source.json')
    physical_model = PhysicalModel.from_source(physical_source)
    _require_physical_mass_binding(physical_model, native_mass)
    control = _read_json(run / 'control-source.json')
    simulation = _read_json(run / 'simulation' / 'simulation.json')
    if (digest(physical_source) != report.get('model_sha256')
            or digest(control) != report.get('control_sha256')
            or simulation.get('model_sha256') != report['model_sha256']
            or simulation.get('control_sha256') != report['control_sha256']
            or simulation.get('schema') != 'cybrgeo-causal-simulation/1'
            or simulation.get('runtime_implementation') != report['simulation'].get('runtime_implementation')
            or simulation.get('mujoco_version') != report['simulation'].get('mujoco_version')):
        raise QualificationError('Physical model, controller or runtime differs from the accepted run')
    for key, value in report['simulation'].items():
        if key in simulation and simulation[key] != value:
            raise QualificationError(f'Simulation {key} differs from its accepted engineering report')
    _require_current_causal_runtime(simulation['runtime_implementation'])
    simulated_loads = _simulated_loads(simulation.get('metrics'))
    if not simulated_loads or set(simulated_loads) != set(plan.document['loads']):
        raise QualificationError('Load cases must cover every active simulated actuator exactly')
    for name, metrics in simulated_loads.items():
        peak = _number(metrics.get('peak_output_torque_Nm'), f'{name} simulated peak torque')
        if plan.document['loads'][name]['demand_Nm'] < peak:
            raise QualificationError(f'Load case {name} understates simulated torque demand')
    basis = dict(source_sha256=digest(source_files), plan_sha256=plan.sha256,
                 native_build_sha256=digest(receipt), native_source_binding_sha256=digest(binding),
                 model_sha256=report['model_sha256'], control_sha256=report['control_sha256'],
                 engineering_report_sha256=_sha(run / 'engineering.json'),
                 simulation_sha256=_sha(run / 'simulation' / 'simulation.json'),
                 integration_trace_sha256=_sha(run / 'simulation' / 'trace.json.gz'),
                 runtime_sha256=digest(dict(runtime=simulation['runtime_implementation'],
                                            mujoco_version=simulation['mujoco_version'], options=simulation['options'])))
    step_hashes = {}
    for row in placements['records']:
        name = f"{row['body']}__{row['name']}"
        path = (native / row['step_file']).resolve()
        if not path.is_relative_to(native):
            raise QualificationError(f'Native STEP escapes its source directory: {name}')
        relative = path.relative_to(native).as_posix()
        if receipt['files_sha256'].get(relative) != _sha(path):
            raise QualificationError(f'Native STEP changed before release: {name}')
        step_hashes[name] = _sha(path)
    evidence = _verify_evidence(package, trust, digest(basis), plan.document, step_hashes)
    # A failed copy never leaves a partially populated product directory.
    destination.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix='.' + destination.name + '-stage-', dir=destination.parent))
    try:
        parts_dir = staging / 'parts'
        parts_dir.mkdir()
        copied = {}
        for row in placements['records']:
            name = f"{row['body']}__{row['name']}"
            path = (native / row['step_file']).resolve()
            target = parts_dir / (name + '.step')
            shutil.copyfile(path, target)
            copied[name] = _sha(target)
            if copied[name] != step_hashes[name]:
                raise QualificationError(f'Native STEP changed while copying release: {name}')
        evidence_dir = staging / 'evidence'
        evidence_dir.mkdir()
        shutil.copyfile(package / 'evidence.json', evidence_dir / 'evidence.json')
        if _sha(evidence_dir / 'evidence.json') != evidence['evidence_set_sha256']:
            raise QualificationError('Signed evidence set changed during release copy')
        for relative, expected in evidence['document_sha256'].items():
            source_document = _artifact(package, relative)
            target = evidence_dir / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source_document, target)
            if _sha(target) != expected:
                raise QualificationError('A physical evidence report changed during release copy')
        from .engineering_source import assert_project_source_identity
        assert_project_source_identity(source, source_files)
        _require_current_causal_runtime(simulation['runtime_implementation'])
        if (digest(verify_native_build(native, project.specification, source_files=source_files)) != basis['native_build_sha256']
                or _sha(run / 'engineering.json') != basis['engineering_report_sha256']
                or _sha(run / 'simulation' / 'simulation.json') != basis['simulation_sha256']
                or _sha(run / 'simulation' / 'trace.json.gz') != basis['integration_trace_sha256']
                or digest(_read_json(run / 'physical-source.json')) != basis['model_sha256']
                or digest(_read_json(run / 'control-source.json')) != basis['control_sha256']
                or _sha(trust) != evidence['trust_policy_sha256']
                or _sha(__file__) != record['qualification_service_sha256']):
            raise QualificationError('Engineering source or run changed during release copy')
        manifest = dict(schema='cybrgeo-qualified-manufacturing-release/1',
                        status='qualified_for_declared_envelope_under_external_evidence',
                        basis=basis, evidence=evidence, part_step_sha256=copied,
                        scope='External signed records support the declared material lots, process route, inspected parts, interfaces, loads and one tested assembly. No claim extends beyond their recorded service envelope or independent issuer trust.')
        (staging / 'qualified-release.json').write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n', encoding='utf-8')
        if destination.exists():
            raise FileExistsError('Qualified release destination appeared during staging')
        os.replace(staging, destination)
        return manifest
    finally:
        if staging.exists():
            if staging.parent.resolve() != destination.parent.resolve() or not staging.name.startswith('.' + destination.name + '-stage-'):
                raise QualificationError('Refusing to clean a staging directory outside its target parent')
            shutil.rmtree(staging)
