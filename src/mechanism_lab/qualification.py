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


_CLEARANCE_INTERFACE_FIELDS = {'parts', 'nominal_clearance_mm', 'first_tolerance_mm',
                               'second_tolerance_mm', 'required_clearance_mm',
                               'demand_N', 'design_capacity_N'}
_RETAINED_CONTACT_INTERFACE_FIELDS = {'role', 'parts', 'nominal_maximum_distance_mm',
                                      'first_tolerance_mm', 'second_tolerance_mm',
                                      'maximum_service_play_mm', 'demand_N',
                                      'design_capacity_N'}
_RETAINED_CONTACT_ROLE = 'tested_retention_contact'


def _interface_role(interface):
    """Keep legacy clearance strict; contact needs an explicitly distinct plan."""
    if not isinstance(interface, dict):
        raise QualificationError('Interface must be an object')
    if 'role' not in interface:
        _keys(interface, _CLEARANCE_INTERFACE_FIELDS, 'Clearance interface')
        return 'clearance'
    if interface.get('role') != _RETAINED_CONTACT_ROLE:
        raise QualificationError('Unknown qualified interface role')
    _keys(interface, _RETAINED_CONTACT_INTERFACE_FIELDS, 'Retained contact interface')
    return _RETAINED_CONTACT_ROLE


def _check_placed_interface(name, interface, measured, overlap,
                            *, distance_kernel_allowance_mm=1e-7,
                            volume_kernel_allowance_mm3=1e-6):
    """Check exact placed CAD without treating kernel allowance as a fit.

    Contact proximity does not prove retention. It is only a prerequisite for
    separately signed, serial-bound assembled play and capacity tests.
    """
    distance = _number(measured, f'{name} native distance')
    intersection = _number(overlap, f'{name} native overlap')
    if intersection > volume_kernel_allowance_mm3:
        raise QualificationError(f'Placed CAD parts fail qualified interface {name} overlap')
    first = interface['first_tolerance_mm']
    second = interface['second_tolerance_mm']
    if _interface_role(interface) == 'clearance':
        worst = distance - first - second
        if (distance + distance_kernel_allowance_mm < interface['nominal_clearance_mm']
                or worst + distance_kernel_allowance_mm < interface['required_clearance_mm']):
            raise QualificationError(f'Placed CAD parts fail qualified interface {name} clearance/tolerance')
        return dict(worst_case_clearance_mm=worst)
    largest_closest_gap = distance + first + second
    if (distance > interface['nominal_maximum_distance_mm'] + distance_kernel_allowance_mm
            or largest_closest_gap > interface['maximum_service_play_mm'] + distance_kernel_allowance_mm):
        raise QualificationError(f'Placed CAD parts fail qualified interface {name} contact proximity/play')
    return dict(role=_RETAINED_CONTACT_ROLE,
                worst_case_closest_gap_mm=largest_closest_gap)


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
        fields = {'schema', 'parts', 'materials', 'processes', 'interfaces', 'loads',
                  'assembly_requirements'}
        if not fields <= set(row) or set(row) - fields - {'solver_connectors', 'spring_load_paths'}:
            raise QualificationError('Qualification plan fields must include the construction contract and only supported load-path mappings')
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
            role = _interface_role(interface)
            pair = interface['parts']
            if (not isinstance(pair, list) or len(pair) != 2 or pair[0] == pair[1]
                    or not set(pair) <= parts.keys()):
                raise QualificationError(f'Interface {name} must connect two declared parts')
            first = _number(interface['first_tolerance_mm'], 'Interface first tolerance')
            second = _number(interface['second_tolerance_mm'], 'Interface second tolerance')
            demand = _number(interface['demand_N'], 'Interface demand', positive=True)
            capacity = _number(interface['design_capacity_N'], 'Interface capacity', positive=True)
            if (first < parts[pair[0]]['required_dimensional_tolerance_mm']
                    or second < parts[pair[1]]['required_dimensional_tolerance_mm']):
                raise QualificationError(f'Interface {name} tolerances understate participating part variation')
            if role == 'clearance':
                nominal = _number(interface['nominal_clearance_mm'], 'Interface nominal clearance')
                required = _number(interface['required_clearance_mm'], 'Interface required clearance')
                admissible = nominal - first - second >= required
            else:
                # A contacting surface is never recast as a positive clearance.
                # Bounded nominal closest approach plus both part variations
                # must fit the planned service play. Actual assembled retention
                # and play are still required for each inspected unit.
                nominal = _number(interface['nominal_maximum_distance_mm'],
                                  'Contact nominal closest-gap limit')
                service = _number(interface['maximum_service_play_mm'],
                                  'Contact maximum service play')
                admissible = nominal + first + second <= service
            if not admissible or capacity < demand:
                raise QualificationError(f'Interface {name} fails its tolerance or load requirement')
        connectors = row.get('solver_connectors', {})
        if not isinstance(connectors, dict):
            raise QualificationError('Solver connectors must map source equality names to planned interfaces')
        for name, interface_name in connectors.items():
            _safe_id(name, 'Solver connector ID')
            if interface_name not in interfaces:
                raise QualificationError(f'Solver connector {name} lacks a planned physical interface')
        spring_paths = row.get('spring_load_paths', {})
        if not isinstance(spring_paths, dict):
            raise QualificationError('Spring load paths must map each control spring to its native interfaces')
        spring_interfaces = set()
        for name, path in spring_paths.items():
            if not isinstance(name, str) or not re.fullmatch(r'spring_(0|[1-9][0-9]*)', name):
                raise QualificationError('Spring load path keys must use their control-source index')
            if not isinstance(path, dict):
                raise QualificationError(f'Spring load path {name} must be an object')
            _keys(path, ('joint', 'maximum_service_scale', 'uncertainty_factor', 'anchors'),
                  f'Spring load path {name}')
            _safe_id(path['joint'], f'Spring {name} joint')
            if (_number(path['maximum_service_scale'], f'Spring {name} maximum service scale') < 1
                    or _number(path['uncertainty_factor'], f'Spring {name} uncertainty factor') < 1):
                raise QualificationError('Spring service scale and authored uncertainty factor must be at least one')
            anchors = path['anchors']
            if not isinstance(anchors, dict) or not anchors:
                raise QualificationError(f'Spring load path {name} needs exact physical anchor interfaces')
            members = {}
            for anchor, interface_name in anchors.items():
                match = re.fullmatch(r'member_(0|[1-9][0-9]*)_(fixed|moving)', anchor) if isinstance(anchor, str) else None
                if (match is None or not isinstance(interface_name, str)
                        or interface_name not in interfaces or interface_name in spring_interfaces):
                    raise QualificationError(f'Spring anchor {anchor} lacks a distinct planned interface')
                members.setdefault(int(match[1]), set()).add(match[2])
                spring_interfaces.add(interface_name)
            if (set(members) != set(range(len(members)))
                    or any(endpoints != {'fixed', 'moving'} for endpoints in members.values())):
                raise QualificationError(f'Spring load path {name} must declare both ends of every physical member')
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
            fit = _check_placed_interface(name, interface, measured, overlap,
                distance_kernel_allowance_mm=distance_kernel_allowance_mm,
                volume_kernel_allowance_mm3=volume_kernel_allowance_mm3)
            measured_interfaces[name] = dict(parts=list(interface['parts']), minimum_distance_mm=measured,
                                             intersection_volume_mm3=overlap, **fit,
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


def _verify_evidence(package, trust_policy, basis_sha256, plan, part_step_sha256,
                     modeled_interface_demands=None):
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
        signed = _number(demand, f'{name} signed force demand', positive=True)
        if signed > plan['interfaces'][name]['demand_N']:
            raise QualificationError(f'Plan understates the independent interface force demand for {name}')
        if (modeled_interface_demands is not None
                and signed < modeled_interface_demands.get(name, 0.)):
            raise QualificationError(f'Signed service envelope understates modeled interface force for {name}')
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
                or _number(row.get('tested_capacity_N'), 'Tested interface capacity', positive=True) < interface['design_capacity_N']):
            raise QualificationError(f'Interface {name} unit, part serials, clearance or capacity fail')
        if _interface_role(interface) == 'clearance':
            if (_number(row.get('measured_minimum_clearance_mm'), 'Measured clearance')
                    < interface['required_clearance_mm']):
                raise QualificationError(f'Interface {name} measured service clearance fails')
        elif (row.get('retention_passed_under_load') is not True
              or _number(row.get('measured_maximum_play_mm'), 'Measured retained-contact play')
              > interface['maximum_service_play_mm']):
            raise QualificationError(f'Interface {name} needs tested retention and bounded assembled play')
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


def _require_connector_demand_binding(plan, physical_model, simulation,
                                      modeled_interface_demands=None):
    """Bound modeled connect/weld reactions by source-planned interface loads.

    Each source equality must be monitored on every integration step and mapped
    to a physical interface. Summing peaks that share an interface is a safe
    screening overbound, not a pin/bearing stress calculation or a hardware
    capacity measurement. Physical capacity still requires signed test records.
    """
    from .design_envelope import connector_force_screens

    root = getattr(physical_model, 'root', None)
    source_connectors = {}
    if root is not None:
        for element in root.findall('./equality/*'):
            if element.tag not in ('connect', 'weld'):
                continue
            name = element.get('name')
            if not name or name in source_connectors:
                raise QualificationError('Physical connect/weld constraints need unique source names')
            body1, body2 = element.get('body1'), element.get('body2')
            if not body1 or not body2 or body1 == body2:
                raise QualificationError(f'Physical connector {name} requires two distinct named body endpoints')
            source_connectors[name] = dict(kind=element.tag, bodies={body1, body2})
    metrics = simulation.get('metrics', {})
    constraints = metrics.get('constraints', {}) if isinstance(metrics, dict) else None
    if not isinstance(constraints, dict):
        raise QualificationError('Simulation lacks a connector reaction inventory')
    recorded = {name: row.get('type') for name, row in constraints.items()
                if isinstance(row, dict) and row.get('type') in ('connect', 'weld')}
    mapping = plan.document.get('solver_connectors', {})
    if (set(recorded) != set(source_connectors)
            or any(recorded[name] != source_connectors[name]['kind'] for name in recorded)):
        raise QualificationError('Recorded connector inventory differs from the physical model')
    if set(mapping) != set(source_connectors):
        raise QualificationError('Source connect/weld constraints lack exact planned interface mapping')
    for source_name, interface_name in mapping.items():
        planned_parts = plan.document['interfaces'][interface_name]['parts']
        part_bodies = {part.split('__', 1)[0] for part in planned_parts}
        if (len(part_bodies) != 2
                or part_bodies != source_connectors[source_name]['bodies']):
            raise QualificationError(
                f'Physical connector {source_name} maps to unrelated native interface bodies')
    if not mapping:
        return
    steps = simulation.get('steps')
    if isinstance(steps, bool) or not isinstance(steps, int) or steps < 1:
        raise QualificationError('Modeled connector demands require integration-step count')
    try:
        screen = connector_force_screens(constraints, steps)
    except (ValueError, TypeError, AttributeError, KeyError) as error:
        raise QualificationError('Connector reaction telemetry is invalid') from error
    if not screen['complete_solver_coverage'] or set(screen['connectors']) != set(mapping):
        raise QualificationError('Modeled connector reaction coverage is incomplete')
    demands = {name: 0. for name in plan.document['interfaces']}
    for name, row in screen['connectors'].items():
        demands[mapping[name]] += row['peak_modeled_anchor_force_N']
    for name, demand in demands.items():
        if demand > plan.document['interfaces'][name]['demand_N']:
            raise QualificationError(f'Interface {name} understates modeled connector force demand')
        if modeled_interface_demands is not None:
            modeled_interface_demands[name] = modeled_interface_demands.get(name, 0.) + demand


def _require_spring_anchor_demand_binding(plan, physical_model, control, simulation,
                                          native_record, placements,
                                          modeled_interface_demands=None):
    """Map every controlled spring end to an actual tested interface demand.

    The native receipt establishes which placed pin/clevis pair carries each
    spring end. The control source and physical hinge independently determine
    the ideal full-range force. Authored service scale and uncertainty only
    increase that demand; neither is a measured spring or hardware capacity.
    """
    from .actuation import TensionSpring
    from .spring_anchor import member_anchor_positions_mm, tension_anchor_demand

    springs = control.get('springs', []) if isinstance(control, dict) else None
    paths = plan.document.get('spring_load_paths', {})
    if not isinstance(springs, list) or any(not isinstance(item, dict) for item in springs):
        raise QualificationError('Control source has an invalid spring inventory')
    if not springs:
        if paths or native_record is not None:
            raise QualificationError('Spring load-path plan or native receipt has no controlled spring')
        return
    if set(paths) != {f'spring_{index}' for index in range(len(springs))}:
        raise QualificationError('Every controlled spring needs an exact planned anchor load path')
    if (not isinstance(native_record, dict)
            or native_record.get('schema') != 'cybrgeo-native-spring-anchor-load-path/1'
            or not isinstance(native_record.get('plan'), dict)
            or not isinstance(native_record.get('native_anchor_bindings'), list)
            or not isinstance(placements, dict)
            or not isinstance(placements.get('records'), list)):
        raise QualificationError('Controlled springs need a source-bound native anchor receipt')
    native_plan = native_record['plan']
    if (native_plan.get('schema') != 'cybrgeo-spring-anchor-plan/1'
            or native_plan.get('plan_sha256') != digest({
                key: value for key, value in native_plan.items() if key != 'plan_sha256'})):
        raise QualificationError('Native spring-anchor plan has invalid source identity')
    groups = native_plan.get('groups')
    if not isinstance(groups, list) or len(groups) != len(springs):
        raise QualificationError('Native anchor groups differ from controlled springs')
    placed = {}
    for item in placements['records']:
        if not isinstance(item, dict):
            raise QualificationError('Native spring-anchor placement inventory is invalid')
        key = (item.get('body'), item.get('name'))
        if (not all(isinstance(value, str) for value in key) or key in placed):
            raise QualificationError('Native spring-anchor placement identity is ambiguous')
        placed[key] = item
    bindings = {}
    for item in native_record['native_anchor_bindings']:
        if not isinstance(item, dict):
            raise QualificationError('Native spring-anchor binding is invalid')
        key = (item.get('source_spring_index'), item.get('member_index'), item.get('role'))
        if (not isinstance(key[0], int) or isinstance(key[0], bool)
                or not isinstance(key[1], int) or isinstance(key[1], bool)
                or key[2] not in ('fixed', 'moving') or key in bindings):
            raise QualificationError('Native spring-anchor binding identity is ambiguous')
        bindings[key] = item
    expected_keys = set()
    options = simulation.get('options') if isinstance(simulation, dict) else None
    if not isinstance(options, dict):
        raise QualificationError('Spring load screening needs the accepted runtime options')
    if options.get('enforceLimits') is not True:
        raise QualificationError('Spring force screening requires physical joint limits throughout the accepted run')
    root = getattr(physical_model, 'root', None)
    if root is None:
        raise QualificationError('Spring load paths need a physical joint and body tree')
    compiler = root.find('./compiler')
    if compiler is None or compiler.get('angle') != 'radian':
        raise QualificationError('Spring load screening requires physical hinge ranges in radians')
    parents = {child: parent for parent in root.iter() for child in parent}
    for index, spring in enumerate(springs):
        path = paths[f'spring_{index}']
        joint_name, config = spring.get('joint'), spring.get('parameters')
        if (not isinstance(joint_name, str) or not isinstance(config, dict)
                or path['joint'] != joint_name):
            raise QualificationError('Spring load path differs from its controlled joint or parameters')
        joint_rows = [row for row in root.findall('.//body/joint')
                      if row.get('name') == joint_name]
        if len(joint_rows) != 1 or joint_rows[0].get('type', 'hinge') != 'hinge':
            raise QualificationError(f'Spring {joint_name} requires one ranged physical hinge')
        moving_body_node = parents[joint_rows[0]]
        if (len(moving_body_node.findall('./joint')) != 1
                or moving_body_node.find('./freejoint') is not None):
            raise QualificationError(f'Spring {joint_name} requires one physical hinge degree of freedom')
        fixed_body_node = parents.get(moving_body_node)
        while fixed_body_node is not None and fixed_body_node.tag not in ('body', 'worldbody'):
            fixed_body_node = parents.get(fixed_body_node)
        fixed_body = ('world' if fixed_body_node is not None and fixed_body_node.tag == 'worldbody'
                      else fixed_body_node.get('name') if fixed_body_node is not None else None)
        moving_body = moving_body_node.get('name')
        if not fixed_body or not moving_body or fixed_body == moving_body:
            raise QualificationError(f'Spring {joint_name} lacks distinct named physical joint bodies')
        try:
            if any(key in moving_body_node.attrib for key in
                   ('euler', 'axisangle', 'xyaxes', 'zaxis')):
                raise ValueError('Unsupported body orientation for source-frame spring anchors')
            def xml_vector(element, attribute, default, size):
                values = [float(value) for value in element.get(attribute, default).split()]
                if len(values) != size or not all(math.isfinite(value) for value in values):
                    raise ValueError(f'Invalid physical {attribute} vector')
                return values

            joint_range = [float(value) for value in joint_rows[0].get('range', '').split()]
            physical_axis = [float(value) for value in joint_rows[0].get('axis', '0 0 1').split()]
            spring_axis = TensionSpring(config, spring.get('axis', [0, 0, -1])).axis
            length = math.sqrt(sum(value*value for value in physical_axis))
            limited = joint_rows[0].get('limited')
            if (limited in ('false', '0')
                    or (limited is None and compiler.get('autolimits') not in ('true', '1'))
                    or float(joint_rows[0].get('ref', '0')) != 0
                    or len(joint_range) != 2 or len(physical_axis) != 3
                    or not all(math.isfinite(value) for value in joint_range+physical_axis)
                    or length <= 0
                    or any(not math.isclose(value/length, expected, rel_tol=0, abs_tol=1e-10)
                           for value, expected in zip(physical_axis, spring_axis))):
                raise ValueError('Physical hinge axis or range disagrees with the spring')
            group = groups[index]
            if not isinstance(group, dict):
                raise ValueError('Native spring group must be an object')
            diameter = _number(group.get('demand', {}).get('nominal_pin_diameter_mm'),
                               'Native spring anchor pin diameter', positive=True)
            demand = tension_anchor_demand(config, joint_range,
                                           pin_diameter_mm=diameter, axis=spring_axis)
            positions = member_anchor_positions_mm(config,
                spring.get('member_offsets_m'),
                child_body_pos_m=xml_vector(moving_body_node, 'pos', '0 0 0', 3),
                child_body_quat_wxyz=xml_vector(moving_body_node, 'quat', '1 0 0 0', 4),
                joint_pos_m=xml_vector(joint_rows[0], 'pos', '0 0 0', 3),
                axis=spring_axis)
        except (AttributeError, KeyError, TypeError, ValueError) as error:
            raise QualificationError(f'Spring {joint_name} has invalid physical force inputs') from error
        if (type(group.get('source_spring_index')) is not int
                or group['source_spring_index'] != index
                or group.get('control_joint') != joint_name
                or group.get('source_parameters_sha256') != digest(config)
                or ('parent_body' in group and group['parent_body'] != fixed_body)
                or ('child_body' in group and group['child_body'] != moving_body)
                or group.get('demand') != demand):
            raise QualificationError(f'Spring {joint_name} native demand differs from control and physical range')
        if ('nominal_pin_diameter_mm' in native_plan
                and _number(native_plan['nominal_pin_diameter_mm'],
                            'Native spring plan pin diameter', positive=True) != diameter):
            raise QualificationError(f'Spring {joint_name} native pin diameter differs from its source plan')
        scale_key = spring.get('scale_option', 'springScale')
        if not isinstance(scale_key, str) or 'springScale' not in options:
            raise QualificationError(f'Spring {joint_name} lacks its accepted runtime scale')
        run_scale = _number(options.get(scale_key, options['springScale']),
                            f'Spring {joint_name} run scale')
        service_scale = path['maximum_service_scale']
        if run_scale > service_scale:
            raise QualificationError(f'Spring {joint_name} exceeds its declared service scale')
        force = demand['required_capacity_per_separate_pin_N'] * service_scale * path['uncertainty_factor']
        group_anchors = group.get('anchors')
        if not isinstance(group_anchors, list) or len(group_anchors) != 2*config['spring_count']:
            raise QualificationError(f'Spring {joint_name} lacks both native ends of every member')
        if set(path['anchors']) != {
                f'member_{member}_{role}' for member in range(config['spring_count'])
                for role in ('fixed', 'moving')}:
            raise QualificationError(f'Spring {joint_name} plan omits or adds an anchor interface')
        for anchor in group_anchors:
            if not isinstance(anchor, dict):
                raise QualificationError('Native spring anchor is invalid')
            member, role = anchor.get('member_index'), anchor.get('role')
            key = (index, member, role)
            name = f'member_{member}_{role}'
            binding = bindings.get(key)
            if (not isinstance(member, int) or isinstance(member, bool)
                    or member < 0 or member >= config['spring_count']
                    or role not in ('fixed', 'moving') or key in expected_keys
                    or type(anchor.get('source_spring_index')) is not int
                    or anchor['source_spring_index'] != index
                    or binding is None or path['anchors'].get(name) is None):
                raise QualificationError(f'Spring {joint_name} has a missing or duplicate physical anchor')
            expected_keys.add(key)
            body = anchor.get('body')
            expected_body = fixed_body if role == 'fixed' else moving_body
            actual_position = anchor.get('body_local_anchor_mm')
            expected_position = positions[member][role]
            if (not isinstance(actual_position, list) or len(actual_position) != 3
                    or any(isinstance(value, bool) or not isinstance(value, (int, float))
                           or not math.isfinite(value)
                           or not math.isclose(value, expected, rel_tol=0., abs_tol=1e-6)
                           for value, expected in zip(actual_position, expected_position))):
                raise QualificationError(f'Spring {joint_name} native anchor differs from its physical joint frame')
            pin, clevis = binding.get('pin_native_part'), binding.get('clevis_placed_part')
            pin_base, clevis_base = anchor.get('pin_part'), anchor.get('clevis_part')
            if (body != expected_body or not isinstance(pin, str)
                    or not isinstance(clevis, str) or binding.get('body') != body
                    or not isinstance(pin_base, str) or not isinstance(clevis_base, str)
                    or not (pin == pin_base or pin.startswith(pin_base+'_'))
                    or not (clevis == clevis_base or clevis.startswith(clevis_base+'_'))
                    or binding.get('anchor') != anchor.get('name')
                    or binding.get('body_local_anchor_mm') != anchor.get('body_local_anchor_mm')
                    or binding.get('clevis_native_part') != anchor.get('clevis_part')
                    or binding.get('pin_diameter_mm') != diameter
                    or ('nominal_pin_length_mm' in native_plan
                        and binding.get('pin_length_mm') != _number(
                            native_plan['nominal_pin_length_mm'],
                            'Native spring plan pin length', positive=True))
                    or ('nominal_clevis_bore_diameter_mm' in native_plan
                        and binding.get('clevis_bore_diameter_mm') != _number(
                            native_plan['nominal_clevis_bore_diameter_mm'],
                            'Native spring plan clevis bore', positive=True))
                    or binding.get('required_capacity_N') != demand['required_capacity_per_separate_pin_N']
                    or (body, pin) not in placed or (body, clevis) not in placed):
                raise QualificationError(f'Spring {joint_name} native pin or clevis is unbound')
            pair = {f'{body}__{pin}', f'{body}__{clevis}'}
            interface = path['anchors'][name]
            if pair != set(plan.document['interfaces'][interface]['parts']):
                raise QualificationError(f'Spring {joint_name} maps force to an unrelated native interface')
            if modeled_interface_demands is not None:
                modeled_interface_demands[interface] = modeled_interface_demands.get(interface, 0.) + force
            elif force > plan.document['interfaces'][interface]['demand_N']:
                raise QualificationError(f'Spring {joint_name} interface understates source force demand')
        if {key for key in expected_keys if key[0] == index} != {
                (index, member, role) for member in range(config['spring_count'])
                for role in ('fixed', 'moving')}:
            raise QualificationError(f'Spring {joint_name} native member coverage is incomplete')
    if set(bindings) != expected_keys:
        raise QualificationError('Native spring-anchor receipt has an extra or missing binding')
    if modeled_interface_demands is not None:
        for name, force in modeled_interface_demands.items():
            if force > plan.document['interfaces'][name]['demand_N']:
                raise QualificationError(f'Interface {name} understates combined modeled force demand')


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


def _require_native_spring_anchor_steps(solids, spring_record):
    """Recheck each source-bound pin/bore on the actual imported placed STEP."""
    from .spring_anchor import verify_native_anchor

    rows = spring_record.get('native_anchor_bindings') if isinstance(spring_record, dict) else None
    if not isinstance(rows, list) or not rows:
        raise QualificationError('Native spring-anchor receipt lacks placed pin/bore bindings')
    for row in rows:
        if not isinstance(row, dict):
            raise QualificationError('Native spring-anchor pin/bore binding is invalid')
        body, pin, clevis = (row.get('body'), row.get('pin_native_part'),
                              row.get('clevis_placed_part'))
        if not all(isinstance(value, str) and value for value in (body, pin, clevis)):
            raise QualificationError('Native spring-anchor pin/bore identity is invalid')
        try:
            measured = verify_native_anchor(solids[f'{body}__{pin}'],
                solids[f'{body}__{clevis}'], row.get('body_local_anchor_mm'),
                pin_diameter_mm=row.get('pin_diameter_mm'),
                pin_length_mm=row.get('pin_length_mm'),
                nominal_bore_diameter_mm=row.get('clevis_bore_diameter_mm'))
        except (KeyError, TypeError, ValueError) as error:
            raise QualificationError(f'Native spring-anchor {body} pin/bore STEP fails its source dimensions') from error
        if any(not _numeric_geometry_matches(value, row.get(name))
               for name, value in measured.items() if name != 'scope'):
            raise QualificationError(f'Native spring-anchor {body} pin/bore receipt differs from placed STEP')


def _recheck_qualified_native_steps(native, placements, interfaces, receipt, plan, record,
                                    spring_record=None):
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
        fit = _check_placed_interface(name, interface, measured, overlap)
        prior = record['exact_placed_interface_checks'][name]
        reported = reported_checks[name]
        if (not _numeric_geometry_matches(measured, prior['minimum_distance_mm'])
                or not _numeric_geometry_matches(overlap, prior['intersection_volume_mm3'])
                or not _numeric_geometry_matches(measured, reported.get('minimum_distance_mm'))
                or not _numeric_geometry_matches(overlap, reported.get('intersection_volume_mm3'))
                or any(not _numeric_geometry_matches(value, prior.get(key))
                       for key, value in fit.items() if key != 'role')
                or (_interface_role(interface) != 'clearance'
                    and prior.get('role') != _RETAINED_CONTACT_ROLE)):
            raise QualificationError(f'Qualified native STEP pair {name} fails independently measured fit')
    if spring_record is not None:
        _require_native_spring_anchor_steps(solids, spring_record)
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
        if measured.get('parts') != declared['parts']:
            raise QualificationError(f'Qualified native interface {name} differs from its exact part-pair plan')
        fit = _check_placed_interface(name, declared,
            _number(measured.get('minimum_distance_mm'), 'Qualified CAD distance'),
            _number(measured.get('intersection_volume_mm3'), 'Qualified CAD overlap'))
        if any((measured.get(key) != value if key == 'role' else
                not _numeric_geometry_matches(value, measured.get(key)))
               for key, value in fit.items()):
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
    spring_file = 'spring-anchor-load-path.json'
    spring_record = (_read_json(native / spring_file)
                     if spring_file in receipt['files_sha256'] else None)
    native_mass = _recheck_qualified_native_steps(native, placements, interfaces,
        receipt, plan.document, record, spring_record)
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
    modeled_interface_demands = {}
    _require_connector_demand_binding(plan, physical_model, simulation,
                                      modeled_interface_demands)
    _require_spring_anchor_demand_binding(plan, physical_model, control,
                                          simulation, spring_record, placements,
                                          modeled_interface_demands)
    simulated_loads = _simulated_loads(simulation.get('metrics'))
    if not simulated_loads or set(simulated_loads) != set(plan.document['loads']):
        raise QualificationError('Load cases must cover every active simulated actuator exactly')
    for name, metrics in simulated_loads.items():
        peak = _number(metrics.get('peak_output_torque_Nm'), f'{name} simulated peak torque')
        if plan.document['loads'][name]['demand_Nm'] < peak:
            raise QualificationError(f'Load case {name} understates simulated torque demand')
        if name == 'rack_motor':
            # The opposed rack has its own requested-output clip before the
            # gripper motor model, not rotary arm belt/gear source limits.
            if 'output_demand_limit_s' not in metrics:
                raise QualificationError('Rack motor lacks output demand-limit telemetry')
            duration = _number(metrics['output_demand_limit_s'],
                               'Rack motor output demand-limit duration')
            if duration > 0:
                raise QualificationError('Rack motor demanded torque beyond its source output model limit')
            continue
        # Delivered torque is clipped by the source drivetrain limit. A low
        # delivered peak cannot excuse a task that demanded more than its
        # declared belt/gear screen; redesign the drive or task and rerun.
        for limit in ('belt', 'gear'):
            if f'{limit}_demand_limit_s' not in metrics:
                raise QualificationError(f'Load case {name} lacks {limit} demand-limit telemetry')
            duration = _number(metrics[f'{limit}_demand_limit_s'],
                               f'{name} {limit} demand-limit duration')
            if duration > 0:
                raise QualificationError(f'Load case {name} demanded torque beyond its {limit} model limit')
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
    evidence = _verify_evidence(package, trust, digest(basis), plan.document,
                                step_hashes, modeled_interface_demands)
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
