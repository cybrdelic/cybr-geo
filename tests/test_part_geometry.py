"""Resolver contracts; synthetic parts are not Refined compatibility evidence."""
from copy import deepcopy
import hashlib

import numpy as np
import pytest

cq = pytest.importorskip('cadquery')

from mechanism_lab.part_geometry import PartGeometryResolver
from mechanism_lab.physical_model import digest
from mechanism_lab.shape_dag import from_trees


def fixture(tmp_path):
    # The complete CSG root already includes a local offset. Its BREP must
    # replace that root, not receive the same translation a second time.
    frame = np.eye(4)
    frame[:3, 3] = [10, 20, 30]
    tree = dict(op='transform', args=frame.ravel().tolist(), children=[
        dict(op='box', args=[1, 2, 3], children=[])])
    records = {'part': dict(id='part', body='body', material='steel', shape=tree),
               'other': dict(id='other', body='body', material='steel', shape=tree)}
    dag = from_trees({key: row['shape'] for key, row in records.items()})
    shape = cq.Workplane('XY').box(2, 4, 6).val().translate((10, 20, 30))
    path = tmp_path / 'part.brep'
    shape.exportBrep(str(path))
    override = dict(part='part', record_sha256=digest(records['part']), path='part.brep',
                    sha256=hashlib.sha256(path.read_bytes()).hexdigest(), units='mm', frame='body-local')
    return dag, records, [override], dict(asset_root=tmp_path,
        source={'repository': 'synthetic-test-fixture', 'commit': 'contract-only'})


def test_whole_root_replacement_has_one_body_transform_and_csg_fallback(tmp_path):
    dag, records, overrides, options = fixture(tmp_path)
    resolver = PartGeometryResolver(dag, records, overrides=overrides, **options)
    frame = np.array([[0., -1, 0, 100], [1, 0, 0, 200], [0, 0, 1, 300], [0, 0, 0, 1]])
    for name in records:
        local = resolver.resolve(name)
        assert local.Center().toTuple() == pytest.approx((10, 20, 30))
        world = resolver.resolve(name, frame=frame)
        assert world.Center().toTuple() == pytest.approx((80, 210, 330))
        assert world.Volume() == pytest.approx(48)
    assert resolver.describe('part')['kind'] == 'brep'
    assert resolver.describe('other')['kind'] == 'csg'


def test_snapshot_imports_verified_bytes_and_returns_isolated_shapes(tmp_path):
    dag, records, overrides, options = fixture(tmp_path)
    resolver = PartGeometryResolver(dag, records, overrides=overrides, **options)
    (tmp_path / 'part.brep').write_bytes(b'changed after snapshot')
    first = resolver.resolve('part')
    first.move(cq.Location(cq.Vector(100, 0, 0)))
    assert resolver.resolve('part').Center().toTuple() == pytest.approx((10, 20, 30))
    report = resolver.report()
    report['parts'][0]['override']['path'] = 'tampered'
    assert resolver.describe('part')['override']['path'] == 'part.brep'
    with pytest.raises(ValueError, match='file identity'):
        PartGeometryResolver(dag, records, overrides=overrides, **options)


@pytest.mark.parametrize('change,message', [
    ({'record_sha256': '0' * 64}, 'record identity'),
    ({'sha256': '0' * 64}, 'file identity'),
    ({'part': 'absent'}, 'Unknown or duplicate'),
    ({'units': 'm'}, 'body-local millimetres'),
    ({'frame': 'world'}, 'body-local millimetres'),
    ({'path': '../part.brep'}, 'confined'),
    ({'path': '/part.brep'}, 'confined'),
    ({'path': 'C:/part.brep'}, 'confined'),
    ({'path': 'folder\\part.brep'}, 'confined'),
])
def test_invalid_override_never_silently_uses_csg(tmp_path, change, message):
    dag, records, overrides, options = fixture(tmp_path)
    overrides[0].update(change)
    with pytest.raises(ValueError, match=message):
        PartGeometryResolver(dag, records, overrides=overrides, **options)


def test_changed_material_or_body_is_not_the_same_part_record(tmp_path):
    dag, records, overrides, options = fixture(tmp_path)
    for field, value in [('material', 'rubber'), ('body', 'other_body')]:
        changed = deepcopy(records)
        changed['part'][field] = value
        with pytest.raises(ValueError, match='record identity'):
            PartGeometryResolver(dag, changed, overrides=overrides, **options)


def test_record_shape_cannot_name_a_different_dag(tmp_path):
    dag, records, overrides, options = fixture(tmp_path)
    records['part']['shape']['args'][3] += 1
    overrides[0]['record_sha256'] = digest(records['part'])
    with pytest.raises(ValueError, match='match the supplied DAG'):
        PartGeometryResolver(dag, records, overrides=overrides, **options)


def test_missing_declared_brep_fails_and_duplicate_root_is_rejected(tmp_path):
    dag, records, overrides, options = fixture(tmp_path)
    with pytest.raises(ValueError, match='duplicate'):
        PartGeometryResolver(dag, records, overrides=overrides * 2, **options)
    (tmp_path / 'part.brep').unlink()
    with pytest.raises(FileNotFoundError):
        PartGeometryResolver(dag, records, overrides=overrides, **options)


def test_explicit_density_mass_and_inertia_are_in_si(tmp_path):
    dag, records, overrides, options = fixture(tmp_path)
    resolver = PartGeometryResolver(dag, records, overrides=overrides, **options)
    properties = resolver.mass_properties('part', density_kg_m3=7850)
    mass = 48 * 7850e-9
    assert properties['mass_kg'] == pytest.approx(mass)
    assert properties['com_mm'] == pytest.approx([10, 20, 30])
    assert np.asarray(properties['inertia_kg_m2']) == pytest.approx(np.diag([
        mass * (4**2 + 6**2) / 12e6, mass * (2**2 + 6**2) / 12e6,
        mass * (2**2 + 4**2) / 12e6]))
    with pytest.raises(ValueError, match='Density'):
        resolver.mass_properties('part', density_kg_m3=0)


@pytest.mark.parametrize('matrix', [np.diag([-1, 1, 1, 1]), np.diag([2, 1, 1, 1])])
def test_body_frame_cannot_reflect_or_scale_native_part(tmp_path, matrix):
    dag, records, overrides, options = fixture(tmp_path)
    resolver = PartGeometryResolver(dag, records, overrides=overrides, **options)
    with pytest.raises(ValueError, match='proper rigid'):
        resolver.resolve('part', frame=matrix)


def test_surface_only_brep_cannot_replace_solid_part(tmp_path):
    dag, records, overrides, options = fixture(tmp_path)
    path = tmp_path / 'part.brep'
    cq.Face.makePlane(2, 3).exportBrep(str(path))
    overrides[0]['sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
    resolver = PartGeometryResolver(dag, records, overrides=overrides, **options)
    with pytest.raises(ValueError, match='positive-volume solid'):
        resolver.resolve('part')


def test_physical_bounds_ignore_cached_display_tessellation():
    from mechanism_lab.part_geometry import native_properties
    shape = cq.Solid.makeCylinder(10, 20).rotate((0, 0, 0), (0, 1, 0), 23)
    before = native_properties(shape, density_kg_m3=2700)
    shape.tessellate(2, .8)
    after = native_properties(shape, density_kg_m3=2700)
    assert np.asarray(after['bounds_mm']) == pytest.approx(np.asarray(before['bounds_mm']), abs=1e-12)
    assert after['volume_mm3'] == pytest.approx(before['volume_mm3'])


@pytest.mark.parametrize('name', ['part_geometry', 'shape_dag', 'native_construction'])
def test_loaded_dependency_drift_is_rejected_at_start_resolve_and_report(tmp_path, monkeypatch, name):
    from mechanism_lab import part_geometry
    key = 'src/mechanism_lab/' + name + '.py'
    path = tmp_path / (name + '.py')
    path.write_bytes(part_geometry.CONSTRUCTION_CODE_PATHS[key].read_bytes())
    monkeypatch.setitem(part_geometry.CONSTRUCTION_CODE_PATHS, key, path)
    dag, records, overrides, options = fixture(tmp_path)
    resolver = PartGeometryResolver(dag, records, overrides=overrides, **options)
    assert resolver.report()['construction_code_sha256'] == part_geometry._LOADED_CODE_HASHES
    path.write_bytes(path.read_bytes() + b'\n# simulated edit after import\n')
    for action in (lambda: PartGeometryResolver(dag, records, overrides=overrides, **options),
                   lambda: resolver.resolve('part'), resolver.report):
        with pytest.raises(ValueError, match='Construction code changed'):
            action()


def test_dependency_drift_during_evaluation_cannot_return_a_shape(tmp_path, monkeypatch):
    from mechanism_lab import part_geometry
    key = 'src/mechanism_lab/shape_dag.py'
    path = tmp_path / 'shape_dag.py'
    path.write_bytes(part_geometry.CONSTRUCTION_CODE_PATHS[key].read_bytes())
    monkeypatch.setitem(part_geometry.CONSTRUCTION_CODE_PATHS, key, path)
    dag, records, _, _ = fixture(tmp_path)
    resolver = PartGeometryResolver(dag, records)
    evaluate = resolver.evaluator.evaluate
    def changed(part):
        result = evaluate(part)
        path.write_bytes(path.read_bytes() + b'\n# simulated concurrent edit\n')
        return result
    monkeypatch.setattr(resolver.evaluator, 'evaluate', changed)
    with pytest.raises(ValueError, match='Construction code changed'):
        resolver.resolve('part')
