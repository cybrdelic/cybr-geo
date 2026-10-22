"""Shared native CAD placement and rigid-body mass accounting.

The integration/aggregation equations and placement export convention are
adapted from cybrdelic/cybr-arx6-belt, commit
aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2, integration/generate_cad_p2.py.
No arm geometry, body topology or controller is built into this module.
Coordinates are body-local millimetres; mass records use kg, metres and kg m².
Mass overrides are declared envelope approximations, not measured inertias.
This module follows CYBR GEO's GPL-2.0-only licensing.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
import hashlib
import json
import math
import os
from pathlib import Path
import re
from typing import Any

import numpy as np


def read_specification(spec):
    """Load finite JSON and distinguish semantic identity from source bytes.

    The canonical digest is the shared physical-model digest of the parsed
    object, independent of JSON formatting/key order or path-versus-dict input.
    The exact original file hash is retained separately when a file was read.
    Numeric values are not rounded or otherwise relaxed for identity matching.
    """
    from .physical_model import digest

    def object_pairs(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f'Duplicate specification key: {key}')
            result[key] = value
        return result

    file_digest = None
    try:
        if isinstance(spec, (str, os.PathLike)):
            original = Path(spec).read_bytes()
            document = json.loads(original.decode('utf-8-sig'), object_pairs_hook=object_pairs)
            file_digest = hashlib.sha256(original).hexdigest()
        else:
            document = deepcopy(spec)
        if not isinstance(document, dict):
            raise ValueError('Native construction requires a specification object')
        # Isolate the JSON document and reject nonfinite/nonserializable values
        # before its identity is used or the builder writes any output.
        document = json.loads(json.dumps(document, allow_nan=False))
        canonical = digest(document)
    except (TypeError, ValueError, UnicodeError) as error:
        raise ValueError(f'Invalid native specification: {error}') from error
    identity = dict(input_spec_sha256=canonical, canonical_specification_sha256=canonical,
                    input_spec_hash_encoding='physical_model.digest: sorted-key compact UTF-8 JSON; no numeric rounding')
    if file_digest is not None:
        identity['input_spec_file_sha256'] = file_digest
    return document, identity


def _name(value):
    if not isinstance(value, str) or not value or not re.fullmatch(r"[A-Za-z0-9_. -]+", value) or value in ('.', '..'):
        raise ValueError('Native CAD identifiers must be nonempty portable basenames')
    return value


def _positive(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f'{label} must be finite and positive')
    return float(value)


def _shape(value):
    return value.val() if hasattr(value, 'val') else value


def transform_shape(shape, pos=(0, 0, 0), axis=(0, 0, 1)):
    """Orient a +Z native shape, then translate, retaining native CAD exactly."""
    position, direction = np.asarray(pos, float), np.asarray(axis, float)
    if position.shape != (3,) or direction.shape != (3,) or not np.isfinite([position, direction]).all() or np.linalg.norm(direction) <= 0:
        raise ValueError('Placement requires a finite position and nonzero axis')
    direction = direction / np.linalg.norm(direction)
    z = np.array([0., 0., 1.])
    cross = np.cross(z, direction)
    angle = math.degrees(math.acos(np.clip(z @ direction, -1, 1)))
    if np.linalg.norm(cross) > 1e-10:
        shape = shape.rotate((0, 0, 0), tuple(cross), angle)
    elif direction[2] < 0:
        shape = shape.rotate((0, 0, 0), (1, 0, 0), 180)
    return shape.translate(tuple(position))


def validate_inertia(tensor):
    """Check finite, symmetric, positive and physically realizable full inertia."""
    tensor = np.asarray(tensor, float)
    if tensor.shape != (3, 3) or not np.isfinite(tensor).all() or not np.allclose(tensor, tensor.T, rtol=1e-10, atol=1e-14):
        raise ValueError('Inertia must be a finite symmetric 3x3 tensor')
    eigenvalues = np.linalg.eigvalsh(tensor)
    if eigenvalues[0] <= 0 or eigenvalues[-1] > sum(eigenvalues[:2]) + max(1e-12, eigenvalues[-1] * 1e-9):
        raise ValueError('Inertia must be positive and satisfy the principal-moment triangle inequality')
    return tensor


def shape_mass_properties(shape, density_kg_m3, mass_kg=None):
    """Native volume integrals with an optional explicit envelope mass override."""
    import cadquery as cq
    density = _positive(density_kg_m3, 'Density')
    native = _shape(shape)
    if not native.isValid():
        raise ValueError('Native mass integration requires a valid shape')
    volume = _positive(float(native.Volume()), 'CAD volume')
    mass = volume * density * 1e-9
    center = np.asarray(native.Center().toTuple(), float) * .001
    tensor = np.asarray(cq.Shape.matrixOfInertia(native), float) * density * 1e-15
    if mass_kg is not None:
        override = _positive(mass_kg, 'Explicit mass')
        tensor *= override / mass
        mass = override
    return mass, center, validate_inertia(tensor)


def aggregate_mass_properties(records, status='Explicit component mass records; no hardware qualification'):
    """Combine inertias about each component's CoM using parallel-axis shifts."""
    if not records:
        raise ValueError('Body mass aggregation requires component records')
    records = deepcopy(records)
    for row in records:
        _positive(row['mass_kg'], 'Component mass')
        center = np.asarray(row['com_m'], float)
        if center.shape != (3,) or not np.isfinite(center).all() or not row.get('provenance'):
            raise ValueError('Component mass needs a finite center and explicit provenance')
        validate_inertia(row['inertia_at_com_kg_m2'])
    mass = sum(row['mass_kg'] for row in records)
    center = sum(row['mass_kg'] * np.asarray(row['com_m']) for row in records) / mass
    tensor = np.zeros((3, 3))
    for row in records:
        offset = np.asarray(row['com_m']) - center
        tensor += np.asarray(row['inertia_at_com_kg_m2']) + row['mass_kg'] * ((offset @ offset) * np.eye(3) - np.outer(offset, offset))
    validate_inertia(tensor)
    return dict(mass_kg=mass, com_m=center.tolist(), inertia_tensor_kg_m2=tensor.tolist(),
                fullinertia_mujoco_kg_m2=[tensor[0, 0], tensor[1, 1], tensor[2, 2], tensor[0, 1], tensor[0, 2], tensor[1, 2]],
                components=records, status=status)


class NativeInterfaceRequirements:
    """Measured nominal native-solid fit requirements for a construction.

    Touching interfaces may be intentional. Each requirement therefore declares
    an intersection-volume tolerance explicitly; this is a CAD kernel allowance,
    not a dimensional manufacturing tolerance or a motion-clearance proof.
    """
    def __init__(self, provenance=None):
        self.provenance = deepcopy(provenance or {})
        self.checks = []

    def noninterference(self, name, first, second, *, allowed_volume_mm3=1e-5):
        name = _name(name)
        if any(row['name'] == name for row in self.checks):
            raise ValueError(f'Duplicate native interface requirement: {name}')
        tolerance = allowed_volume_mm3
        if isinstance(tolerance, bool) or not isinstance(tolerance, (int, float)) or not math.isfinite(tolerance) or tolerance < 0:
            raise ValueError('Native interface volume tolerance must be finite and nonnegative')
        first, second = _shape(first), _shape(second)
        if not first.isValid() or not second.isValid():
            raise ValueError('Native interface requirements need valid shapes')
        volume = float(first.intersect(second).Volume())
        distance = float(first.distance(second))
        if not math.isfinite(volume) or not math.isfinite(distance) or volume < 0 or distance < 0:
            raise ValueError('Native interface measurement was invalid')
        row = dict(name=name, intersection_volume_mm3=volume, minimum_distance_mm=distance,
                   volume_tolerance_mm3=tolerance, **{'pass':volume <= tolerance})
        self.checks.append(row)
        return row

    def report(self):
        return dict(schema='cybrgeo-native-interfaces/1', provenance=self.provenance,
                    checks=deepcopy(self.checks), passed=bool(self.checks) and all(row['pass'] for row in self.checks),
                    scope='Declared nominal BREP interfaces only; no tolerance-stack, full-motion, fatigue, stress or hardware qualification')


@dataclass
class NativeBuild:
    output: Path
    parts: dict[str, Any]
    placed_shapes: dict[tuple[str, str], Any]
    assemblies: dict[str, Any]
    mass_properties: dict
    part_metadata: dict
    meshes: list
    placed_records: list
    files_sha256: dict
    provenance: dict
    auxiliary_assemblies: dict[str, Any] = field(default_factory=dict)


class NativeConstruction:
    """Per-build registries; importing the module has no filesystem effects."""
    def __init__(self, output, densities, body_ids=(), provenance=None):
        self.output = Path(output).resolve()
        self.cad_directory = self.output / 'cad'
        self.placed_directory = self.cad_directory / 'placed'
        self.placed_directory.mkdir(parents=True, exist_ok=True)
        # A failed replacement build cannot retain an earlier success receipt.
        (self.output / 'native-build.json').unlink(missing_ok=True)
        self.densities = {_name(k): _positive(v, 'Density') for k, v in densities.items()}
        self.parts, self.part_metadata, self.assemblies, self.placed_shapes = {}, {}, {}, {}
        self.body_records = {_name(name): [] for name in body_ids}
        self.meshes, self.placed_records, self.auxiliary_assemblies = [], [], {}
        self.provenance = deepcopy(provenance or {})
        self.provenance['native_construction_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        self._files = []

    def _export(self, shape, path, **kwargs):
        import cadquery as cq
        cq.exporters.export(shape, str(path), **kwargs)
        self._files.append(path)

    def addpart(self, name, shape, material='aluminum', note='Custom nominal prototype geometry'):
        name = _name(name)
        if name in self.parts:
            raise ValueError(f'Duplicate native part: {name}')
        native = _shape(shape)
        if not native.isValid():
            raise ValueError('Invalid solid: ' + name)
        volume = _positive(float(native.Volume()), 'CAD volume')
        density = self.densities[material]
        self.parts[name] = shape
        self.part_metadata[name] = dict(material=material, volume_mm3=volume, mass_kg=volume*density*1e-9, status=note)
        self._export(shape, self.cad_directory / (name+'.step'))
        self._export(shape, self.cad_directory / (name+'.stl'), tolerance=.1, angularTolerance=.15)
        return shape

    def place(self, body, label, shape, material='aluminum', mass=None, note='CAD density'):
        import cadquery as cq
        body, label = _name(body), _name(label)
        if (body, label) in self.placed_shapes:
            raise ValueError(f'Duplicate native placement: {body}/{label}')
        self.body_records.setdefault(body, [])
        if body not in self.assemblies:
            self.assemblies[body] = cq.Assembly(name=body)
        self.assemblies[body].add(shape, name=label, color=cq.Color(.65, .68, .71) if material == 'aluminum' else cq.Color(.25, .28, .31))
        filename = body+'__'+label+'.step'
        self._export(shape, self.placed_directory / filename)
        self.placed_shapes[(body, label)] = shape
        self.placed_records.append(dict(body=body, name=label, material=material, step_file='cad/placed/'+filename))
        vertices, triangles = _shape(shape).tessellate(.18, .22)
        self.meshes.append(dict(body=body, name=label, material=material,
                                vertices_mm=[[round(c, 5) for c in vertex.toTuple()] for vertex in vertices],
                                triangles=[list(triangle) for triangle in triangles]))
        weight, center, tensor = shape_mass_properties(shape, self.densities[material], mass)
        self.add_mass_record(body, dict(name=label, mass_kg=weight, com_m=center.tolist(),
                                       inertia_at_com_kg_m2=tensor.tolist(), provenance=note))

    def place_part(self, body, name, pos=(0, 0, 0), axis=(0, 0, 1)):
        self.place(body, name+'_'+str(len(self.body_records.setdefault(body, []))),
                   transform_shape(self.parts[name], pos, axis), self.part_metadata[name]['material'])

    def add_mass_record(self, body, record):
        aggregate_mass_properties([record])
        self.body_records.setdefault(_name(body), []).append(deepcopy(record))

    def add_assembly_mass(self, body, assembly, materials, *, offset_mm=(0, 0, 0), provenance='Native assembly component CAD density'):
        """Integrate the actual located components of a separately authored body.

        Material keys are exact assembly paths produced by CadQuery's recursive
        iterator. Locations include all parent transforms; ``offset_mm`` shifts
        the authored assembly into its declared body frame. This adds inertial
        records only and does not duplicate display placements or CAD exports.
        """
        body = _name(body)
        offset = np.asarray(offset_mm, float)
        if offset.shape != (3,) or not np.isfinite(offset).all() or not provenance:
            raise ValueError('Assembly mass needs a finite body-frame offset and provenance')
        components = list(assembly)
        if not components or set(materials) != {name for _, name, _, _ in components}:
            raise ValueError('Assembly mass requires one explicit material for every component path')
        records = []
        for shape, name, location, _ in components:
            material = materials[name]
            placed = shape.moved(location).translate(tuple(offset))
            mass, center, tensor = shape_mass_properties(placed, self.densities[material])
            records.append(dict(name=name, material=material, mass_kg=mass, com_m=center.tolist(),
                inertia_at_com_kg_m2=tensor.tolist(), provenance=provenance))
        for record in records:
            self.add_mass_record(body, record)

    def save_auxiliary_assembly(self, name, assembly):
        name = _name(name)
        self.auxiliary_assemblies[name] = assembly
        path = self.cad_directory / (name+'_subassembly.step')
        assembly.save(str(path))
        self._files.append(path)

    def write_json(self, filename, document):
        path = self.output / _name(filename)
        def scalar(value):
            return value.item() if hasattr(value, 'item') else list(value)
        path.write_text(json.dumps(document, indent=2, default=scalar, allow_nan=False)+'\n', encoding='utf-8')
        self._files.append(path)

    def finish(self, body_status, extra_records=None, *, native_requirements=None):
        if native_requirements is not None:
            if not isinstance(native_requirements, NativeInterfaceRequirements):
                raise TypeError('Expected native interface requirements')
            report = native_requirements.report()
            self.write_json('native-interface-requirements.json', report)
            if not report['passed']:
                failed = [row['name'] for row in report['checks'] if not row['pass']]
                raise ValueError('Native construction interface requirements failed: ' + ', '.join(failed or ['no requirements supplied']))
        properties = {body: aggregate_mass_properties(rows, body_status) for body, rows in self.body_records.items() if rows}
        for body, assembly in self.assemblies.items():
            path = self.cad_directory / (body+'_subassembly.step')
            assembly.save(str(path))
            self._files.append(path)
        self.write_json('mass_properties.json', properties)
        self.write_json('cad_parts.json', self.part_metadata)
        self.write_json('geometry-meshes.json', dict(units='mm', meshes=self.meshes))
        self.write_json('placed-native-parts.json', dict(units='mm', records=self.placed_records))
        for filename, document in (extra_records or {}).items():
            self.write_json(filename, document)
        hashes = {str(path.relative_to(self.output)).replace('\\', '/'): hashlib.sha256(path.read_bytes()).hexdigest() for path in self._files}
        self.write_json('native-build.json', dict(schema='cybrgeo-native-build/1', provenance=self.provenance,
                        units='CAD mm; mass kg; body-local CoM m; inertia kg m2',
                        parts=len(self.parts), placements=len(self.placed_records), body_ids=list(properties),
                        files_sha256=hashes, scope='Native geometry and explicit mass accounting; no dynamics or hardware qualification'))
        return NativeBuild(self.output, self.parts, self.placed_shapes, self.assemblies, properties,
                           self.part_metadata, self.meshes, self.placed_records, hashes,
                           self.provenance, self.auxiliary_assemblies)
