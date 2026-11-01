"""Replay verified native geometry using a bound run's integrated body states.

Examples declare mappings, materials, a world basis and optional scene geometry.
They do not need another CAD loader, pose evaluator or native identity verifier.
This module does not simulate motion or certify an imported run's claims.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from .engineering_source import verify_native_build, verify_native_source_binding
from .physical_model import PhysicalModel, digest
from .physical_replay import build_assembly as bind_meshes


def _identity(row):
    if not isinstance(row, dict):
        raise ValueError('Native and scene records must be objects')
    for key in ('body', 'name'):
        value = row.get(key)
        if (not isinstance(value, str) or not value or value in ('.', '..')
                or any(char in value for char in '/\\:*?"<>|')):
            raise ValueError(f'Replay record requires an export-safe {key}')
    return f"{row['body']}__{row['name']}"


def _indexed(records, label):
    if not isinstance(records, (list, tuple)):
        raise ValueError(f'{label} requires a list of records')
    result = {}
    for row in records:
        key = _identity(row)
        if key in result:
            raise ValueError(f'Duplicate {label} identity: {key}')
        result[key] = row
    return result


def _artifact(directory, receipt, relative):
    if not isinstance(relative, str) or Path(relative).is_absolute():
        raise ValueError('Native artifact must be a relative path')
    path = (directory / relative).resolve()
    if not path.is_relative_to(directory):
        raise ValueError('Native artifact escapes the native output directory')
    expected = receipt['files_sha256'].get(path.relative_to(directory).as_posix())
    if expected is None:
        raise ValueError(f'Native artifact is absent from the completion receipt: {relative}')
    payload = path.read_bytes()
    if hashlib.sha256(payload).hexdigest() != expected:
        raise ValueError(f'Native artifact differs from the completion receipt: {relative}')
    return path, payload


def _analytic_shape(value, label):
    shape = value.val() if hasattr(value, 'val') else value
    try:
        volume = float(shape.Volume())
        valid = shape.isValid() and bool(shape.Solids())
    except (AttributeError, TypeError) as error:
        raise ValueError(f'{label} requires an analytic solid') from error
    if not valid or not math.isfinite(volume) or volume <= 0:
        raise ValueError(f'{label} requires valid positive-volume analytic CAD')
    return shape


def mesh_record(shape, *, body, name, material='default', tolerance=.15, angular_tolerance=.20):
    """Tessellate caller-declared body-local mm scene CAD without changing it."""
    row = dict(body=body, name=name, material=material, provenance='imported-analytic-cad')
    _identity(row)
    if any(isinstance(x, bool) or not math.isfinite(x) or x <= 0
           for x in (tolerance, angular_tolerance)):
        raise ValueError('Scene tessellation tolerances must be finite and positive')
    shape = _analytic_shape(shape, 'Scene geometry')
    vertices, faces = shape.tessellate(tolerance, angular_tolerance)
    row.update(vertices_mm=[v.toTuple() for v in vertices], triangles=faces,
               tessellation={'linear_mm': tolerance, 'angular_rad': angular_tolerance})
    return row


def build_assembly(native_directory, simulation_result, *, specification,
                   body_mapping, mass_mapping, materials, world_rotation=None,
                   name='Integrated native replay', analytic=True,
                   scene_records=(), scene_shapes=None, metadata=None):
    """Load verified native meshes/STEP and bind actual recorded rigid poses.

    ``body_mapping`` maps every display body to a recorded physical body (or
    ``world``). ``mass_mapping`` maps every physical CAD-inertia body to its
    exact native mass record; no mirrored or inferred mass fallback is used.
    ``specification`` is the native build's parsed specification, independent
    of where an example stores it inside its bound task parameters.

    Native and added scene geometry must be body-local millimetres in the
    simulation's local coordinate basis. ``world_rotation`` maps the simulation
    world to core Z-up exactly once. STEP remains in the same local coordinates
    as its display mesh; the existing exporter applies the recorded world pose.

    ``scene_records`` uses the physical_replay mesh schema; ``scene_shapes``
    maps each ``body__name`` to its BRep. Supplied scene CAD is the geometric
    authority: its record can contain only body/name/material, or a mesh made
    with ``mesh_record``; a supplied mesh must match that CAD's tessellation.
    Callers derive these from their bound
    run's declared scene, not from an archived baseline. With ``analytic=True``
    every displayed part requires CAD. Imported success flags are not promoted
    to qualification by this loader.
    """
    directory = Path(native_directory).resolve()
    if simulation_result.get('schema') != 'cybrgeo-causal-simulation/1':
        raise ValueError('Native replay requires cybrgeo-causal-simulation/1')
    parameters = simulation_result.get('task_parameters')
    reference = simulation_result.get('reference_source')
    if (not isinstance(parameters, dict) or not isinstance(reference, dict)
            or digest(reference) != simulation_result.get('reference_sha256')
            or digest(reference.get('task_parameters')) != digest(parameters)):
        raise ValueError('Replay task parameters must match the bound reference source')
    source_files = reference.get('source_files')
    if not isinstance(source_files, dict) or not source_files:
        raise ValueError('Native replay requires the complete authored project source identity')
    source = simulation_result.get('physical_source')
    if not isinstance(source, dict) or digest(source) != simulation_result.get('model_sha256'):
        raise ValueError('Replay physical source does not match simulation model identity')
    physical = PhysicalModel.from_source(source)
    receipt = verify_native_build(directory, specification)
    binding = verify_native_source_binding(directory, specification, source_files, receipt=receipt)
    if physical.provenance.get('native_build_sha256') != digest(receipt):
        raise ValueError('Native build receipt is not bound to the simulation physical source')
    if physical.provenance.get('native_source_binding_sha256') != digest(binding):
        raise ValueError('Native construction source binding is not bound to the simulation physical source')

    def read_artifact(filename):
        return json.loads(_artifact(directory, receipt, filename)[1].decode('utf-8'))

    masses = read_artifact('mass_properties.json')
    if not isinstance(mass_mapping, dict) or set(mass_mapping) != set(physical.mass_records):
        raise ValueError('Mass mapping must cover every physical CAD mass record exactly')
    if any(not isinstance(value, str) or value not in masses for value in mass_mapping.values()):
        raise ValueError('Mass mapping refers to a missing native mass record')
    if digest({body: masses[key] for body, key in mass_mapping.items()}) != digest(physical.mass_records):
        raise ValueError('Mapped native mass records differ from the simulation source')

    document = read_artifact('geometry-meshes.json')
    placements = read_artifact('placed-native-parts.json')
    if document.get('units') != 'mm' or placements.get('units') != 'mm':
        raise ValueError('Native meshes and analytic placements must explicitly use mm')
    native = _indexed(document.get('meshes'), 'native mesh')
    placed = _indexed(placements.get('records'), 'native placement')
    if not native or set(native) != set(placed):
        raise ValueError('Placed analytic CAD identities do not match native display meshes')
    for key, row in placed.items():
        if row.get('material', 'default') != native[key].get('material', 'default'):
            raise ValueError(f'Native material binding differs for {key}')
    scenes = _indexed(deepcopy(scene_records), 'scene mesh')
    if native.keys() & scenes.keys():
        raise ValueError('Scene geometry cannot replace a native part')
    cads = dict(scene_shapes or {})
    if set(cads) - scenes.keys() or (analytic and set(cads) != set(scenes)):
        raise ValueError('Scene analytic CAD identities must match scene display records')
    cads = {key: _analytic_shape(shape, key) for key, shape in cads.items()}
    for key, shape in cads.items():
        row = scenes[key]
        tolerance = row.get('tessellation', {'linear_mm': .15, 'angular_rad': .20})
        if not isinstance(tolerance, dict) or set(tolerance) != {'linear_mm', 'angular_rad'}:
            raise ValueError('Scene tessellation requires linear_mm and angular_rad')
        canonical = mesh_record(shape, body=row['body'], name=row['name'],
            material=row.get('material', 'default'), tolerance=tolerance['linear_mm'],
            angular_tolerance=tolerance['angular_rad'])
        for field in ('vertices_mm', 'triangles'):
            if field in row and not np.array_equal(np.asarray(row[field]), np.asarray(canonical[field])):
                raise ValueError(f'Scene display mesh differs from its analytic CAD: {key}')
        row.update(canonical)
    document['meshes'].extend(scenes.values())
    bodies = {row['body'] for row in document['meshes']}
    if not isinstance(body_mapping, dict) or set(body_mapping) != bodies:
        raise ValueError('Body mapping must cover every display body exactly')
    known_bodies = set(physical.bodies) | {'world'}
    if any(not isinstance(value, str) or value not in known_bodies for value in body_mapping.values()):
        raise ValueError('Display body mapping must refer to a body in the physical source')
    for body in {row['body'] for row in native.values()}:
        target = body_mapping[body]
        mass_targets = {physical_body for physical_body, native_body in mass_mapping.items()
                        if native_body == body}
        if ((mass_targets and target not in mass_targets)
                or (target in mass_mapping and mass_mapping[target] != body)):
            raise ValueError(f'Native display body mapping disagrees with its CAD inertia binding: {body}')
    # Validate all referenced placements even for mesh-only inspection, and
    # require their bytes in the verified completion receipt before importing.
    for key, row in placed.items():
        path, _ = _artifact(directory, receipt, row.get('step_file'))
        if analytic:
            import cadquery as cq
            cads[key] = _analytic_shape(cq.importers.importStep(str(path)).val(), key)

    details = dict(metadata or {})
    details.update(native_geometry_file_sha256=receipt['files_sha256']['geometry-meshes.json'],
                   native_source=deepcopy(receipt['provenance']),
                   native_build_sha256=digest(receipt),
                   native_source_binding_sha256=digest(binding),
                   reference_sha256=simulation_result['reference_sha256'],
                   native_mass_mapping=dict(mass_mapping), analytic_replay=bool(analytic))
    assembly = bind_meshes(document, body_mapping, simulation_result, name=name,
                           materials=materials, world_rotation=world_rotation, metadata=details)
    for part in assembly.parts:
        if part.name in cads:
            part.cad = cads[part.name]
            part.provenance = 'imported-analytic-cad'
    if analytic and any(part.cad is None for part in assembly.parts):
        raise ValueError('Analytic replay must retain a BRep for every display part')
    return assembly
