"""Resolve complete body-local parts from shared CSG or verified native BREP.

The replacement boundary follows ARX compact Refined's actual
``source/export_refined_native.py`` at commit
85bd6d2ce5b7c567d08b3ffccf2a9be46005aabf: a BREP replaces the complete
evaluated local part, not a primitive inside its CSG tree. Unlike the source's
file-existence selection, callers declare the exact replacement set and hashes.
No upstream generator, renderer, or module with import side effects executes.
This module uses CYBR GEO's existing GPL-2.0-only license.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
from io import BytesIO
import json
from pathlib import Path, PurePosixPath
import re
from collections.abc import Mapping

import numpy as np

from .physical_model import digest
from .shape_dag import ShapeEvaluator, from_trees
from . import native_construction, shape_dag


CONSTRUCTION_CODE_PATHS = {
    'src/mechanism_lab/part_geometry.py': Path(__file__).resolve(),
    'src/mechanism_lab/shape_dag.py': Path(shape_dag.__file__).resolve(),
    'src/mechanism_lab/native_construction.py': Path(native_construction.__file__).resolve(),
}


def construction_code_hashes():
    return {name: hashlib.sha256(path.read_bytes()).hexdigest()
            for name, path in CONSTRUCTION_CODE_PATHS.items()}


_LOADED_CODE_HASHES = construction_code_hashes()


def require_current_construction(expected=None):
    """Reject drift from imported code and from the caller's build snapshot."""
    current = construction_code_hashes()
    if current != _LOADED_CODE_HASHES or expected is not None and current != expected:
        raise ValueError('Construction code changed; reload the resolver and rebuild before accepting output')
    return dict(current)


def _json_copy(value):
    return json.loads(json.dumps(value, allow_nan=False))


def place_local_shape(shape, frame):
    """Apply one proper body-to-world millimetre frame to a complete local part."""
    import cadquery as cq
    from OCP.gp import gp_Trsf
    matrix = np.asarray(frame, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.array_equal(matrix[3], [0, 0, 0, 1])
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-10, rtol=0)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-10, rtol=0)):
        raise ValueError('Part placement requires a finite proper rigid body-to-world mm frame')
    transform = gp_Trsf()
    transform.SetValues(*matrix[:3].ravel().tolist())
    return shape.transformShape(cq.Matrix(transform))


def native_properties(shape, *, density_kg_m3):
    """Measure one explicit native shape in its mm basis with a declared density."""
    import cadquery as cq
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    identity = require_current_construction()
    shape = shape.val() if hasattr(shape, 'val') else shape
    mass, center, tensor = native_construction.shape_mass_properties(shape, density_kg_m3)
    box = Bnd_Box()
    # Display tessellation must not change physical geometry measurements.
    BRepBndLib.AddOptimal_s(shape.wrapped, box, False, False)
    bounds = cq.BoundBox(box)
    require_current_construction(identity)
    return dict(volume_mm3=float(shape.Volume()), density_kg_m3=float(density_kg_m3),
                mass_kg=float(mass), com_mm=(center * 1000).tolist(),
                inertia_kg_m2=tensor.tolist(),
                bounds_mm=[[getattr(bounds, axis + edge) for axis in ('x', 'y', 'z')]
                           for edge in ('min', 'max')],
                solid_count=len(shape.Solids()), valid=bool(shape.isValid()))


class PartGeometryResolver:
    """One geometry authority per part, bound to its complete authored record.

    ``overrides`` is an explicit list of records with ``part``,
    ``record_sha256``, ``path``, ``sha256``, ``units='mm'`` and
    ``frame='body-local'``. Optional ``provenance`` is retained as data.
    The part record hash includes ownership/material/shape, not just its ID.
    Paths are portable relative POSIX paths beneath ``asset_root``. All BREP
    bytes are verified before any CAD evaluation, then imported from those
    same in-memory bytes. Missing or corrupt declared replacements fail closed.
    A shape returned to callers is a copy; cache edits cannot change later parts.
    """

    def __init__(self, dag, part_records, *, overrides=(), asset_root=None, source=None):
        self._code_identity = require_current_construction()
        self.evaluator = ShapeEvaluator(dag)
        if (not isinstance(part_records, Mapping)
                or set(part_records) != set(self.evaluator.dag['roots'])):
            raise ValueError('Part records must cover the shape DAG roots exactly')
        self.records = _json_copy(dict(part_records))
        if any(not isinstance(record, dict) for record in self.records.values()):
            raise ValueError('Every part record must be an object')
        if (any('shape' not in record for record in self.records.values())
                or from_trees({key: record['shape'] for key, record in self.records.items()})['roots']
                != self.evaluator.dag['roots']):
            raise ValueError('Part record shapes must match the supplied DAG roots')
        self.record_sha256 = {key: digest(value) for key, value in self.records.items()}
        self.source = _json_copy(source or {})
        if (not isinstance(overrides, (list, tuple))
                or any(not isinstance(row, Mapping) for row in overrides)):
            raise ValueError('BREP overrides must be an explicit list of records')
        if overrides and (not isinstance(self.source, dict) or any(
                not isinstance(self.source.get(key), str) or not self.source[key].strip()
                for key in ('repository', 'commit'))):
            raise ValueError('BREP overrides require explicit source repository and commit')
        if overrides and asset_root is None:
            raise ValueError('BREP overrides require an explicit asset root')
        root = Path(asset_root).resolve() if asset_root is not None else None
        self.overrides, self._bytes, self._cache = {}, {}, {}
        required = {'part', 'record_sha256', 'path', 'sha256', 'units', 'frame'}
        for original in overrides:
            row = _json_copy(dict(original))
            if set(row) - (required | {'provenance'}) or not required <= row.keys():
                raise ValueError('Invalid BREP override fields')
            key = row['part']
            if not isinstance(key, str) or key not in self.records or key in self.overrides:
                raise ValueError(f'Unknown or duplicate BREP part: {key!r}')
            if row['record_sha256'] != self.record_sha256[key]:
                raise ValueError(f'BREP part record identity mismatch: {key}')
            if row['units'] != 'mm' or row['frame'] != 'body-local':
                raise ValueError('BREP replacements must declare body-local millimetres')
            relative, expected = row['path'], row['sha256']
            if (not isinstance(relative, str) or not relative
                    or '\\' in relative or ':' in relative
                    or PurePosixPath(relative).is_absolute()
                    or any(part in ('', '.', '..') for part in relative.split('/'))
                    or PurePosixPath(relative).suffix.lower() != '.brep'):
                raise ValueError('BREP path must be a confined relative POSIX .brep path')
            if not isinstance(expected, str) or not re.fullmatch('[0-9a-f]{64}', expected):
                raise ValueError('BREP SHA256 must be a lowercase hexadecimal digest')
            path = root.joinpath(*PurePosixPath(relative).parts)
            if (not path.resolve().is_relative_to(root)
                    or any(parent.is_symlink() for parent in (path, *path.parents) if parent != root)):
                raise ValueError('BREP paths must remain under the asset root without symlinks')
            payload = path.read_bytes()
            if hashlib.sha256(payload).hexdigest() != expected:
                raise ValueError(f'BREP file identity mismatch: {relative}')
            self.overrides[key] = row
            self._bytes[expected] = payload

    def resolve(self, part, *, frame=None):
        require_current_construction(self._code_identity)
        if part not in self.records:
            raise ValueError(f'Unknown part: {part!r}')
        if part in self.overrides:
            import cadquery as cq
            identity = self.overrides[part]['sha256']
            if identity not in self._cache:
                shape = cq.Shape.importBrep(BytesIO(self._bytes[identity]))
                if (shape.isNull() or not shape.isValid() or not shape.Solids()
                        or not np.isfinite(shape.Volume()) or shape.Volume() <= 0):
                    raise ValueError(f'BREP part is not valid positive-volume solid geometry: {part}')
                self._cache[identity] = shape
            shape = self._cache[identity].copy()
        else:
            shape = self.evaluator.evaluate(part)
        result = place_local_shape(shape, frame) if frame is not None else shape
        require_current_construction(self._code_identity)
        return result

    def describe(self, part):
        if part not in self.records:
            raise ValueError(f'Unknown part: {part!r}')
        result = dict(part=part, record_sha256=self.record_sha256[part],
                      csg_root_sha256=self.evaluator.dag['roots'][part],
                      kind='brep' if part in self.overrides else 'csg',
                      units='mm', frame='body-local')
        if part in self.overrides:
            result['override'] = deepcopy(self.overrides[part])
        return result

    def report(self):
        require_current_construction(self._code_identity)
        return dict(schema='cybrgeo-part-geometry/1', source=deepcopy(self.source),
                    implementation_sha256=self._code_identity['src/mechanism_lab/part_geometry.py'],
                    shape_evaluator_sha256=self._code_identity['src/mechanism_lab/shape_dag.py'],
                    construction_code_sha256=dict(self._code_identity),
                    parts=[self.describe(key) for key in self.records],
                    scope='Selected native geometry only; no inherited dynamics or hardware acceptance')

    def mass_properties(self, part, *, density_kg_m3):
        """Integrate selected geometry with an explicit homogeneous density.

        No source mass override, display finish, supplier property or silent
        material fallback is interpreted here. CoM/tensor are in the local
        part's body basis and about its CoM, before any world placement.
        """
        return native_properties(self.resolve(part), density_kg_m3=density_kg_m3)
