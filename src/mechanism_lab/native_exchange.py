"""Native shape-DAG construction for non-Python geometry consumers.

This is a transport adapter, not another geometry kernel or recipe library.
All primitives, placements and booleans execute through ``ShapeEvaluator``.
Coordinates remain in the authored millimetre frame, with no up-axis change.

``geometry.mesh`` is little endian: four uint32 values (magic 0x47454f4d,
version 1, vertex count, index count), vertex-count XYZ float32 positions,
then index-count uint32 triangle indices. Counts include all named roots in
source order. Roots are compounded, not silently fused with one another.
Native STEP is authoritative; the mesh is its explicitly sampled derivative.
This diagnostic interchange does not create a manufacturing release.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import struct

import numpy as np

from .native_construction import read_specification
from .shape_dag import ShapeEvaluator, from_trees


SCHEMA = 'cybrgeo-native-exchange/1'
MESH_MAGIC = 0x47454f4d
MESH_VERSION = 1


def _json(path, document):
    path.write_text(json.dumps(document, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def _bounds(shape):
    from .manufacturing_joints import native_bounds
    # Use GEO's analytic BREP bounds, not CadQuery's default cached-mesh
    # bounds, whose padding changes after display tessellation.
    box = native_bounds(shape)
    return dict(min=[box.xmin, box.ymin, box.zmin], max=[box.xmax, box.ymax, box.zmax])


def _positive(value, label):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f'{label} must be finite and positive')
    return float(value)


def construct(source, output, *, linear_tolerance_mm=.05, angular_tolerance_rad=.1, purpose='manufacturing'):
    """Build native roots once and write STEP plus their sampled mesh.

    ``source`` is the source JSON path or its parsed object. The output must
    not exist. Validation and native evaluation happen before creating it;
    ``result.json`` is written only after all geometry exports succeed.
    No materials, measured properties or engineering acceptance are inferred.
    """
    from .assembly_model import require_component_export
    require_component_export(purpose)
    output = Path(output)
    if output.exists():
        raise FileExistsError(f'Native exchange output must be a fresh directory: {output}')
    linear_tolerance_mm = _positive(linear_tolerance_mm, 'Linear tessellation tolerance')
    angular_tolerance_rad = _positive(angular_tolerance_rad, 'Angular tessellation tolerance')
    document, identity = read_specification(source)
    if set(document) - {'schema', 'units', 'trees', 'provenance'}:
        raise ValueError('Native exchange accepts only schema, units, trees and provenance')
    if document.get('schema') != SCHEMA or document.get('units') != 'mm':
        raise ValueError('Unsupported native exchange schema or units')
    dag = from_trees(document.get('trees'), provenance=document.get('provenance'))
    evaluator = ShapeEvaluator(dag)

    import cadquery as cq
    shapes, roots, positions, indices = [], [], [], []
    vertex_count = index_count = 0
    for ordinal, name in enumerate(dag['roots']):
        shape = evaluator.evaluate(name)
        solid_count = len(shape.Solids())
        if not shape.isValid() or not solid_count or shape.Volume() <= 0:
            raise ValueError(f'Native exchange root must contain valid nonempty solids: {name}')
        vertices, triangles = shape.tessellate(linear_tolerance_mm, angular_tolerance_rad)
        xyz = np.asarray([vertex.toTuple() for vertex in vertices], dtype='<f4')
        faces = np.asarray(triangles, dtype=np.int64)
        if (xyz.ndim != 2 or xyz.shape[1] != 3 or not len(xyz) or not np.isfinite(xyz).all()
                or faces.ndim != 2 or faces.shape[1] != 3 or not len(faces)
                or np.min(faces) < 0 or np.max(faces) >= len(xyz)):
            raise ValueError(f'Native tessellation has invalid coordinates or triangle indices: {name}')
        next_vertex_count = vertex_count + len(xyz)
        next_index_count = index_count + faces.size
        if max(next_vertex_count, next_index_count) > 0xffffffff:
            raise ValueError('Native exchange exceeds uint32 mesh limits')
        positions.append(xyz)
        indices.append(np.asarray(faces + vertex_count, dtype='<u4'))
        # Source root names stay in JSON; indexed filenames cannot escape output
        # or collide with reserved files, even for unusual source identifiers.
        roots.append(dict(name=name, step_file=f'roots/{ordinal:05d}.step',
                          solid_count=solid_count, valid=True, volume_mm3=shape.Volume(),
                          bounds_mm=_bounds(shape), vertex_start=vertex_count,
                          vertex_count=len(xyz), index_start=index_count, index_count=faces.size))
        vertex_count, index_count = next_vertex_count, next_index_count
        shapes.append(shape)

    combined = shapes[0] if len(shapes) == 1 else cq.Compound.makeCompound(shapes)
    output.mkdir(parents=True, exist_ok=False)
    (output / 'roots').mkdir()
    _json(output / 'native-source.json', document)
    _json(output / 'shape-dag.json', dag)
    for row, shape in zip(roots, shapes):
        cq.exporters.export(shape, str(output / row['step_file']))
    if len(shapes) == 1:
        # The combined shape is exactly this root. Reuse its native file, not
        # a second potentially expensive STEP traversal of identical geometry.
        shutil.copyfile(output / roots[0]['step_file'], output / 'geometry.step')
    else:
        cq.exporters.export(combined, str(output / 'geometry.step'))
    with (output / 'geometry.mesh').open('wb') as stream:
        stream.write(struct.pack('<IIII', MESH_MAGIC, MESH_VERSION, vertex_count, index_count))
        for xyz in positions:
            stream.write(xyz.tobytes(order='C'))
        for faces in indices:
            stream.write(faces.tobytes(order='C'))

    result = dict(schema='cybrgeo-native-exchange-result/1', purpose='inspection',
                  units='mm', coordinate_frame='unchanged from authored source',
                  construction='mechanism_lab.shape_dag.ShapeEvaluator',
                  hardware_qualified=False, roots=roots, solid_count=sum(r['solid_count'] for r in roots),
                  volume_mm3=combined.Volume(), bounds_mm=_bounds(combined),
                  source_file='native-source.json', dag_file='shape-dag.json',
                  step_file='geometry.step', mesh_file='geometry.mesh',
                  mesh=dict(magic=MESH_MAGIC, version=MESH_VERSION,
                            vertex_count=vertex_count, index_count=index_count,
                            linear_tolerance_mm=linear_tolerance_mm,
                            angular_tolerance_rad=angular_tolerance_rad),
                  source_identity=identity)
    result['files_sha256'] = {str(path.relative_to(output)).replace('\\', '/'):
                             hashlib.sha256(path.read_bytes()).hexdigest()
                             for path in sorted(output.rglob('*')) if path.is_file()}
    _json(output / 'result.json', result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inspection', action='store_true', help='Explicit diagnostic native geometry; not manufacturing release')
    parser.add_argument('--quiet', action='store_true', help='Do not write success metadata to stdout')
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args(argv)
    try:
        result = construct(args.source, args.output, purpose='inspection' if args.inspection else 'manufacturing')
    except (ValueError, OSError) as error:
        parser.exit(1, f'Native geometry exchange failed: {error}\n')
    if not args.quiet:
        print(json.dumps(dict(output=str(args.output), roots=len(result['roots']),
                              solids=result['solid_count'], vertices=result['mesh']['vertex_count']), allow_nan=False))


if __name__ == '__main__':
    main()
