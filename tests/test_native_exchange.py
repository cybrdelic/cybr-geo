"""Native CSG exchange is the same geometry, not a parallel CAD implementation."""
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess
import sys

import numpy as np
import pytest

from mechanism_lab.native_exchange import MESH_MAGIC, MESH_VERSION, SCHEMA, _bounds, construct as _construct
from mechanism_lab.shape_dag import ShapeEvaluator, from_trees


def construct(*args, **kwargs):
    return _construct(*args, purpose='inspection', **kwargs)

def test_explicit_inspection_required_before_export(tmp_path):
    from mechanism_lab.qualification import QualificationError
    with pytest.raises(QualificationError):_construct(source(),tmp_path/'native')
    assert not (tmp_path/'native').exists()

def node(op, args=(), *children):
    return dict(op=op, args=list(args), children=list(children))


def source(trees=None):
    return dict(schema=SCHEMA, units='mm', trees=trees or {'block': node('box', [2, 3, 4])},
                provenance={'source': 'test-authored geometry; no physical qualification'})


def read_mesh(path):
    data = path.read_bytes()
    magic, version, vertices, indices = struct.unpack_from('<IIII', data)
    assert (magic, version) == (MESH_MAGIC, MESH_VERSION)
    assert len(data) == 16 + vertices * 12 + indices * 4
    xyz = np.frombuffer(data, dtype='<f4', count=vertices * 3, offset=16).reshape(-1, 3)
    triangles = np.frombuffer(data, dtype='<u4', count=indices, offset=16 + vertices * 12).reshape(-1, 3)
    assert np.isfinite(xyz).all()
    assert len(triangles) and np.max(triangles) < vertices
    return xyz, triangles


def signed_volume(xyz, triangles):
    a, b, c = np.asarray(xyz, dtype=float)[triangles].transpose(1, 0, 2)
    return np.einsum('ij,ij->i', a, np.cross(b, c)).sum() / 6


def test_exact_native_readback_and_binary_triangle_orientation(tmp_path):
    import cadquery as cq
    # The hole remains a native cylinder and Boolean cut. Native output is not
    # reconstructed from the sampled mesh or its polygonal circular contour.
    tree = node('cut', [], node('box', [5, 7, 2]), node('cylinder', [2, 3]))
    # Apply a reflection with a nonzero placement to exercise frame/winding.
    transform = [-1, 0, 0, 10, 0, 1, 0, 20, 0, 0, 1, 30, 0, 0, 0, 1]
    original = source({'bored plate': node('transform', transform, tree)})
    preserved = deepcopy(original)
    output = tmp_path / 'native'
    result = construct(original, output)
    assert original == preserved
    assert result['purpose'] == 'inspection' and not result['hardware_qualified']
    assert result['construction'] == 'mechanism_lab.shape_dag.ShapeEvaluator'
    assert result['solid_count'] == 1
    native = cq.importers.importStep(str(output / 'geometry.step')).val()
    authoritative = ShapeEvaluator(from_trees(original['trees'])).evaluate()
    assert native.isValid() and len(native.Solids()) == 1
    assert native.Volume() == pytest.approx(authoritative.Volume(), rel=1e-10)
    assert native.cut(authoritative).Volume() < 1e-8
    assert authoritative.cut(native).Volume() < 1e-8
    assert any(face.geomType() == 'CYLINDER' for face in native.Faces())
    xyz, triangles = read_mesh(output / 'geometry.mesh')
    assert signed_volume(xyz, triangles) == pytest.approx(native.Volume(), rel=.001)
    assert xyz.min(axis=0) == pytest.approx([5, 13, 28])
    assert xyz.max(axis=0) == pytest.approx([15, 27, 32])
    assert json.loads((output / 'native-source.json').read_text()) == original
    assert json.loads((output / 'shape-dag.json').read_text())['roots'].keys() == original['trees'].keys()
    assert json.loads((output / 'result.json').read_text()) == result
    for path, digest in result['files_sha256'].items():
        assert hashlib.sha256((output / path).read_bytes()).hexdigest() == digest


def test_named_roots_stay_separate_with_safe_indexed_paths(tmp_path):
    import cadquery as cq
    # Intentional overlap must not be silently fused by a transport adapter.
    document = source({'../outside': node('box', [1, 2, 3]),
                       'geometry': node('box', [1, 2, 3])})
    result = construct(document, tmp_path / 'native')
    native = cq.importers.importStep(str(tmp_path / 'native/geometry.step')).val()
    assert len(native.Solids()) == 2
    assert native.Volume() == pytest.approx(96)
    assert result['solid_count'] == 2
    assert [row['name'] for row in result['roots']] == ['../outside', 'geometry']
    assert [row['step_file'] for row in result['roots']] == ['roots/00000.step', 'roots/00001.step']
    assert not (tmp_path / 'outside.step').exists()
    xyz, triangles = read_mesh(tmp_path / 'native/geometry.mesh')
    assert signed_volume(xyz, triangles) == pytest.approx(96)
    for row in result['roots']:
        shape = cq.importers.importStep(str(tmp_path / 'native' / row['step_file'])).val()
        assert shape.isValid() and shape.Volume() == pytest.approx(48)
        first = row['index_start'] // 3
        last = first + row['index_count'] // 3
        assert np.min(triangles[first:last]) >= row['vertex_start']
        assert np.max(triangles[first:last]) < row['vertex_start'] + row['vertex_count']


@pytest.mark.parametrize('root_count,export_count', [(1, 1), (2, 3)])
def test_single_root_step_is_reused_without_second_native_export(tmp_path, monkeypatch, root_count, export_count):
    import cadquery as cq
    original_export = cq.exporters.export
    calls = []

    def counted_export(shape, filename, *args, **kwargs):
        calls.append(filename)
        return original_export(shape, filename, *args, **kwargs)

    monkeypatch.setattr(cq.exporters, 'export', counted_export)
    document = source({f'part-{i}': node('box', [1, 2, 3]) for i in range(root_count)})
    output = tmp_path / 'native'
    result = construct(document, output)
    assert len(calls) == export_count
    if root_count == 1:
        assert (output / 'geometry.step').read_bytes() == (output / 'roots/00000.step').read_bytes()
        assert result['files_sha256']['geometry.step'] == result['files_sha256']['roots/00000.step']


def test_native_bounds_are_invariant_to_tessellation_and_step_roundtrip(tmp_path):
    import cadquery as cq
    document = source({'cylinder': node('cylinder', [3, 7])})
    native = ShapeEvaluator(from_trees(document['trees'])).evaluate()
    expected = dict(min=[-3, -3, -7], max=[3, 3, 7])
    before = _bounds(native)
    native.tessellate(.8, .5)
    after = _bounds(native)
    for edge in ('min', 'max'):
        assert before[edge] == pytest.approx(expected[edge], abs=1e-10)
        assert after[edge] == pytest.approx(before[edge], abs=1e-10)
    for label, tolerance in [('fine', .01), ('coarse', .8)]:
        output = tmp_path / label
        result = construct(document, output, linear_tolerance_mm=tolerance, angular_tolerance_rad=.5)
        restored = cq.importers.importStep(str(output / 'geometry.step')).val()
        readback = _bounds(restored)
        for edge in ('min', 'max'):
            assert result['bounds_mm'][edge] == pytest.approx(expected[edge], abs=1e-10)
            assert result['roots'][0]['bounds_mm'][edge] == pytest.approx(expected[edge], abs=1e-10)
            assert readback[edge] == pytest.approx(expected[edge], abs=1e-10)


@pytest.mark.parametrize('mutate', [
    lambda doc: doc.update(schema='other'),
    lambda doc: doc.update(units='m'),
    lambda doc: doc.update(unexpected=True),
    lambda doc: doc.update(trees={}),
    lambda doc: doc.update(trees={'bad': node('box', [-1, 2, 3])}),
    lambda doc: doc.update(trees={'bad': node('sphere', [float('nan')])}),
    lambda doc: doc.update(trees={'bad': node('unimplemented', [])}),
    lambda doc: doc.update(trees={'empty': node('cut', [], node('box', [1, 2, 3]), node('box', [2, 3, 4]))}),
])
def test_malformed_or_empty_source_never_creates_output(tmp_path, mutate):
    document = source()
    mutate(document)
    with pytest.raises(ValueError):
        construct(document, tmp_path / 'native')
    assert not (tmp_path / 'native').exists()


def test_duplicate_json_keys_and_existing_directory_are_rejected(tmp_path):
    input_file = tmp_path / 'input.json'
    input_file.write_text('{"schema":"x", "schema":"y"}')
    with pytest.raises(ValueError, match='Duplicate specification key'):
        construct(input_file, tmp_path / 'native')
    assert not (tmp_path / 'native').exists()
    (tmp_path / 'native').mkdir()
    marker = tmp_path / 'native/keep.txt'
    marker.write_text('original')
    with pytest.raises(FileExistsError, match='fresh directory'):
        construct(source(), tmp_path / 'native')
    assert marker.read_text() == 'original'
    assert sorted(p.name for p in (tmp_path / 'native').iterdir()) == ['keep.txt']


def test_module_cli_uses_same_authoritative_construction(tmp_path):
    path = tmp_path / 'source.json'
    path.write_text(json.dumps(source()))
    env = dict(os.environ, PYTHONPATH=str(Path(__file__).resolve().parents[1] / 'src'))
    command = [sys.executable, '-m', 'mechanism_lab.native_exchange', '--inspection', str(path), str(tmp_path / 'native')]
    completed = subprocess.run(command, capture_output=True, text=True, env=env)
    assert completed.returncode == 0, completed.stderr
    result = json.loads(completed.stdout)
    assert result['roots'] == 1 and result['solids'] == 1
    read_mesh(tmp_path / 'native/geometry.mesh')
    failed = subprocess.run(command, capture_output=True, text=True, env=env)
    assert failed.returncode == 1 and 'fresh directory' in failed.stderr
    quiet_command = [sys.executable, '-m', 'mechanism_lab.native_exchange', '--inspection', '--quiet',
                     str(path), str(tmp_path / 'quiet')]
    quiet = subprocess.run(quiet_command, capture_output=True, text=True, env=env)
    assert quiet.returncode == 0, quiet.stderr
    assert quiet.stdout == ''
    read_mesh(tmp_path / 'quiet/geometry.mesh')
    quiet_failed = subprocess.run(quiet_command, capture_output=True, text=True, env=env)
    assert quiet_failed.returncode == 1 and quiet_failed.stdout == ''
    assert 'fresh directory' in quiet_failed.stderr
