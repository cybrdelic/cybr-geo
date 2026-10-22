"""Actual compact source geometry exercises the shared DAG and explicit frames."""
import ast
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

cq = pytest.importorskip('cadquery')
from OCP.gp import gp_Trsf
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform

from examples.engineering.arx_compact.design import (
    COMMIT, PINNED_BODY_ALIASES, SOURCE, SOURCE_TO_CAD, body_frames, bounds,
    build_records, geometry_dag, geometry_scope, load_source, verify_pinned_benchmark,
)
from mechanism_lab.shape_dag import ShapeEvaluator


def test_entire_source_lowers_without_a_second_generator():
    design, pose = load_source()
    dag = geometry_dag(design, source=SOURCE)
    assert len(dag['roots']) == len(design['parts']) == 278
    assert len(pose['body_names']) == 10
    assert len(dag['nodes']) < 2000
    assert sum(node['op'] == 'helix' for node in dag['nodes'].values()) == 2
    assert set(p['body'] for p in design['parts']) <= body_frames(pose, aliases=PINNED_BODY_ALIASES).keys()


def test_source_si_pose_and_local_millimetres_are_composed_in_order():
    design, pose = load_source()
    frame = body_frames(pose)['upper']
    row = np.asarray(pose['body_poses'][pose['body_names'].index('upper')])
    local_point = np.array([15., 23., -7.])
    expected_source = row[3:].reshape(3, 3) @ local_point + 1000 * row[:3]
    assert np.allclose(frame @ np.r_[local_point, 1], SOURCE_TO_CAD @ np.r_[expected_source, 1])
    evaluator = ShapeEvaluator(geometry_dag(design, source=SOURCE))
    source = evaluator.evaluate('0')
    world = evaluator.evaluate('0', frame=body_frames(pose, aliases=PINNED_BODY_ALIASES)['fixed'])
    assert world.Volume() == pytest.approx(source.Volume(), rel=1e-10)


@pytest.mark.parametrize('invalid', ['missing', 'duplicate', 'nonrigid', 'nan'])
def test_invalid_body_frames_fail_closed(invalid):
    _, pose = load_source()
    pose = copy.deepcopy(pose)
    if invalid == 'missing':
        pose['body_names'][0] = 'unrecognized_fixed'
    elif invalid == 'duplicate':
        pose['body_names'][0] = pose['body_names'][1]
    elif invalid == 'nonrigid':
        pose['body_poses'][0][3] = 2
    else:
        pose['body_poses'][0][0] = float('nan')
    with pytest.raises(ValueError):
        body_frames(pose, aliases=PINNED_BODY_ALIASES)


def synthetic_records(part_count, body_count):
    """Adapter contract fixture only; these boxes are not another ARX revision."""
    names = [f'fixture_body_{index}' for index in range(body_count)]
    pose = dict(body_names=names, time_s=1.25,
                body_poses=[[index * .01, .002, .003, 1, 0, 0, 0, 1, 0, 0, 0, 1]
                            for index in range(body_count)])
    design = dict(units='mm', revision='synthetic-adapter-contract', scope='Not ARX source evidence',
                  parts=[dict(id=index, name=f'fixture_box_{index}', body=names[index % body_count],
                              shape=dict(op='box', args=[1, 2, 3], children=[]),
                              bounds_mm=[[-1, -2, -3], [1, 2, 3]], finish='steel',
                              group='fixture', role='adapter test', status='synthetic')
                         for index in range(part_count)])
    source = dict(repository='test-fixture:compact-adapter', commit='synthetic-contract-v1')
    return design, pose, source


@pytest.mark.parametrize('part_count,body_count', [(1, 1), (279, 11)])
def test_adapter_cardinality_is_independent_of_pinned_benchmark(part_count, body_count):
    design, pose, source = synthetic_records(part_count, body_count)
    dag = geometry_dag(design, source=source)
    assert len(dag['roots']) == part_count
    assert len(body_frames(pose)) == body_count
    assert dag['provenance']['commit'] == 'synthetic-contract-v1'
    assert dag['provenance']['commit'] != COMMIT
    with pytest.raises(ValueError, match='pinned benchmark'):
        verify_pinned_benchmark(design, pose)


@pytest.mark.parametrize('aliases', [{'fixed_hardware': 'fixed_hardware'},
                                     {'fixed': 'missing'}, {'fixed': 'other', 'other': 'fixed_hardware'}])
def test_aliases_cannot_overwrite_or_invent_captured_frames(aliases):
    _, pose = load_source()
    with pytest.raises(ValueError, match='alias|Alias'):
        body_frames(pose, aliases=aliases)


def test_existing_fixed_frame_is_not_silently_replaced():
    _, pose = load_source()
    pose['body_names'].append('fixed')
    pose['body_poses'].append([.5, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1])
    assert body_frames(pose)['fixed'][0, 3] == 500
    with pytest.raises(ValueError, match='conflicts'):
        body_frames(pose, aliases=PINNED_BODY_ALIASES)


@pytest.mark.parametrize('source', [None, {}, {'repository': 'fixture'}, {'repository': '', 'commit': 'fixture'}])
def test_alternate_records_require_explicit_provenance(source):
    design, _, _ = synthetic_records(1, 1)
    with pytest.raises(ValueError, match='provenance'):
        geometry_dag(design, source=source)


def test_alternate_units_and_missing_pose_fail_before_output(tmp_path):
    design, pose, source = synthetic_records(1, 1)
    design['units'] = 'm'
    with pytest.raises(ValueError, match='millimetres'):
        build_records(tmp_path / 'bad-units', design, pose, source=source)
    assert not (tmp_path / 'bad-units').exists()
    design['units'] = 'mm'
    design['parts'][0]['body'] = 'absent'
    with pytest.raises(ValueError, match='Missing source body pose'):
        build_records(tmp_path / 'bad-pose', design, pose, source=source)
    assert not (tmp_path / 'bad-pose').exists()


def test_variable_count_native_build_retains_units_provenance_and_artifacts(tmp_path):
    design, pose, source = synthetic_records(11, 11)
    assembly, report = build_records(tmp_path, design, pose, source=source)
    assert len(assembly.parts) == report['part_count'] == 11
    assert report['captured_body_count'] == report['part_body_count'] == 11
    assert report['source'] == source
    assert report['time_s'] == 1.25
    assert assembly.views['hero'].note == '11 source parts / captured t=1.25 / geometry study only'
    assert geometry_scope(report) == ('Source CAD at captured t=1.25 placement; '
                                      'no engineering acceptance, dynamics or hardware qualification')
    assert not report['qualification']['engineering_acceptance']
    assert np.asarray(report['body_frames_to_world']['fixture_body_10'])[:3, 3] == pytest.approx([100, -3, 2])
    for row in report['parts']:
        native = cq.importers.importStep(str(tmp_path / row['local_step'])).val()
        assert native.Volume() == pytest.approx(48)
        assert hashlib.sha256((tmp_path / row['local_step']).read_bytes()).hexdigest() == row['local_step_sha256']
    for name, identity in report['artifacts'].items():
        assert hashlib.sha256((tmp_path / name).read_bytes()).hexdigest() == identity['sha256']
    from mechanism_lab.importers import import_geometry
    reloaded = import_geometry(tmp_path / 'reference.glb')
    assert len(reloaded.parts) == 11
    assert reloaded.bounds == pytest.approx(assembly.bounds, abs=1e-4)


def test_client_edit_during_export_cannot_produce_accepted_report(tmp_path, monkeypatch):
    from examples.engineering.arx_compact import design as compact
    from mechanism_lab import exporters
    key = 'examples/engineering/arx_compact/design.py'
    path = tmp_path / 'design.py'
    path.write_bytes(compact.CONSTRUCTION_CODE_PATHS[key].read_bytes())
    monkeypatch.setitem(compact.CONSTRUCTION_CODE_PATHS, key, path)
    export = exporters.export_glb
    def changed(*args, **kwargs):
        result = export(*args, **kwargs)
        path.write_bytes(path.read_bytes() + b'\n# simulated concurrent edit\n')
        return result
    monkeypatch.setattr(exporters, 'export_glb', changed)
    design, pose, source = synthetic_records(1, 1)
    with pytest.raises(ValueError, match='Construction code changed'):
        compact.build_records(tmp_path / 'rejected', design, pose, source=source)
    assert not (tmp_path / 'rejected/geometry-study.json').exists()


@pytest.fixture(scope='module')
def original_native_build():
    """Run only the hash-pinned original builder, without its file-writing loop."""
    fixture = Path(__file__).parent / 'fixtures/arx_compact/build_meshes.py'
    raw = fixture.read_bytes()
    assert COMMIT == '0003382bd6a2e96f3bed065003efec35001982f7'
    expected_sha256 = '1b2e4f087634f8ff41ad450195a2a35ef3ef6445d1bc25efbad9c797f7fb2e50'
    assert hashlib.sha256(raw).hexdigest() == expected_sha256 == SOURCE['native_builder_sha256']
    original = ast.parse(raw, filename=str(fixture))
    functions = [node for node in original.body if isinstance(node, ast.FunctionDef) and node.name == 'build']
    assert len(functions) == 1
    assert (functions[0].lineno, functions[0].end_lineno) == (10, 27)
    namespace = dict(json=json, np=np, cq=cq, gp_Trsf=gp_Trsf,
                     BRepBuilderAPI_Transform=BRepBuilderAPI_Transform, cache={})
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(fixture), 'exec'), namespace)
    return namespace['build']


@pytest.mark.parametrize('part_id,z_range', [
    ('263', (33., 92.)),
    ('269', (-67., -33.)),
])
def test_actual_springs_preserve_independently_measured_source_defects(part_id, z_range, original_native_build):
    # The independent, unchanged source and shared DAG run on the same kernel.
    # OCP 7.8/7.9 sweep integration differs, so a previous Windows measurement
    # is evidence for that run, not a portable mathematical volume constant.
    design, _ = load_source()
    row = next(p for p in design['parts'] if str(p['id']) == part_id)
    reference = original_native_build(copy.deepcopy(row['shape']))
    shape = ShapeEvaluator(geometry_dag(design, source=SOURCE)).evaluate(part_id)
    assert shape.isValid() and reference.isValid()
    assert len(shape.Solids()) == len(reference.Solids()) == 2
    assert shape.Volume() == pytest.approx(reference.Volume(), rel=1e-8)
    assert np.asarray(bounds(shape)) == pytest.approx(np.asarray(bounds(reference)), abs=1e-5)
    for actual_solid, source_solid in zip(sorted(shape.Solids(), key=lambda s: s.Volume()),
                                         sorted(reference.Solids(), key=lambda s: s.Volume())):
        assert actual_solid.isValid() and source_solid.isValid()
        assert actual_solid.Volume() == pytest.approx(source_solid.Volume(), rel=1e-8)
        assert actual_solid.Center().toTuple() == pytest.approx(source_solid.Center().toTuple(), abs=1e-5)
        assert np.asarray(bounds(actual_solid)) == pytest.approx(np.asarray(bounds(source_solid)), abs=1e-5)
    bb = bounds(shape)
    assert (bb[0][2], bb[1][2]) == pytest.approx(z_range, abs=1e-5)
    if part_id == '263':
        assert bb[1][2] - row['bounds_mm'][1][2] == pytest.approx(18., abs=1e-5)
