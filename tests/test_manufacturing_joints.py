"""P6B structural geometry compared against the executed original builder."""
from contextlib import redirect_stdout
from dataclasses import replace
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import shutil
import struct

import cadquery as cq
import numpy as np
import pytest

from mechanism_lab.manufacturing_joints import CaptiveNutPattern, RoundedBoxTube, SleevedCrossBore, SplitLapSplice, SOURCE, native_bounds


ROOT = Path(__file__).resolve().parents[1]


def _module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def client():
    return _module('arx_print_first_client', ROOT/'examples/engineering/arx_compact/print_first.py')


@pytest.fixture(scope='module')
def original_and_adapted(tmp_path_factory, client):
    source = ROOT/'tests/fixtures/arx_latest/structural_prototypes.py'
    assert hashlib.sha256(source.read_bytes()).hexdigest() == SOURCE['sha256']
    original = _module('original_p6b_structural', source)
    output = tmp_path_factory.mktemp('original-p6b-structural')
    for name in ('cad', 'stl', 'reports', 'inputs'):
        (output/name).mkdir()
    for local, upstream in (('print-first-upper.brep', 'P5-upper-frame.brep'),
                            ('print-first-fore.brep', 'P5-fore-frame.brep')):
        assert hashlib.sha256((client.DATA/local).read_bytes()).hexdigest() == client.INPUT_SHA256[local]
        shutil.copyfile(client.DATA/local, output/'inputs'/upstream)
    # Unmodified source runs only when invoked, with its output rooted in this
    # isolated temporary directory. Import does not run its main or write files.
    original.ROOT = output
    with redirect_stdout(io.StringIO()):
        original.main()
    report = json.loads((output/'reports/structural-prototypes.json').read_text())
    snapshot_inputs = tmp_path_factory.mktemp('verified-input-snapshot')
    for name in ('print-first-inputs.json', *client.INPUT_SHA256):
        shutil.copyfile(client.DATA/name, snapshot_inputs/name)
    original_snapshot = client.verified_input_snapshot

    def capture_then_change_paths(data):
        manifest, payloads = original_snapshot(data)
        # A competing filesystem edit after verification must not replace the
        # native input consumed by this build. Only these temporary copies move.
        for name in payloads:
            (Path(data)/name).write_bytes(b'changed after verified snapshot')
        return manifest, payloads

    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(client, 'verified_input_snapshot', capture_then_change_paths)
        result = client.build(tmp_path_factory.mktemp('adapted-p6b-structural'), data=snapshot_inputs)
    assert all((snapshot_inputs/name).read_bytes() == b'changed after verified snapshot' for name in client.INPUT_SHA256)
    assert result.provenance['construction_code_sha256'] == client.construction_code_hashes()
    assert hashlib.sha256(source.read_bytes()).hexdigest() == SOURCE['sha256']
    return result, report, output


@pytest.mark.parametrize('name', ['upper-frame-proximal', 'upper-frame-distal',
                                  'upper-frame-lap-coupler', 'fore-frame-reinforced'])
def test_exact_source_geometry_native_mass_and_step(original_and_adapted, name):
    result, report, output = original_and_adapted
    shape = result.parts[name]
    source_shape = cq.Shape.importBrep(str(output/'cad'/f'{name}.brep'))
    record = next(row for row in report['parts'] if row['name'] == name)
    assert shape.isValid() and len(shape.Solids()) == 1
    assert shape.Volume() == pytest.approx(record['volume_mm3'], rel=1e-10)
    assert shape.Center().toTuple() == pytest.approx(record['com_parent_mm'], abs=1e-7)
    assert np.asarray(cq.Shape.matrixOfInertia(shape))*1.25e-12 == pytest.approx(
        np.asarray(record['inertia_tensor_about_COM_kg_m2']), rel=1e-8, abs=1e-10)
    assert shape.cut(source_shape).Volume() < 1e-5
    assert source_shape.cut(shape).Volume() < 1e-5
    assert result.part_metadata[name]['mass_kg'] == pytest.approx(record['solid_PETG_mass_kg'], rel=1e-10)
    restored = cq.importers.importStep(str(result.output/'cad'/f'{name}.step')).val()
    assert restored.isValid() and len(restored.Solids()) == 1
    assert restored.Volume() == pytest.approx(record['volume_mm3'], rel=1e-9)
    bb = native_bounds(shape)
    assert [bb.xlen, bb.ylen, bb.zlen] == pytest.approx(record['print_bounding_size_mm'], abs=1e-7)
    printed = cq.importers.importStep(str(result.output/'print-local'/f'{name}.step')).val()
    print_bounds = native_bounds(printed)
    assert [print_bounds.xmin, print_bounds.ymin, print_bounds.zmin] == pytest.approx([5, 5, 0], abs=1e-6)


def test_source_joint_mates_holes_and_all_six_native_deliverables(original_and_adapted):
    result, report, _ = original_and_adapted
    requirements = json.loads((result.output/'native-interface-requirements.json').read_text())
    assert requirements['passed']
    assert len(requirements['checks']) == 3
    assert [row['required_minimum_clearance_mm'] for row in requirements['checks']] == [.3, .2, .2]
    assert all(row['intersection_volume_mm3'] == pytest.approx(0, abs=1e-8) for row in requirements['checks'])
    assert all(row['clearance_passed'] for row in requirements['checks'])
    assert len(result.parts) == 6 and len(result.placed_records) == 6
    assert len(result.assemblies['upper-test-assembly'].objects) == 4  # root + 3 mating solids
    for y in (118, 136, 160, 178):
        half = result.parts['upper-frame-proximal' if y < 148 else 'upper-frame-distal']
        # Material is removed on the actual sleeve/beam side walls, both sides.
        for x in (-27, 27):
            assert not half.isInside(cq.Vector(x, y, 0), 1e-7)
        for x in (-21, 21):
            assert not result.parts['upper-frame-lap-coupler'].isInside(cq.Vector(x, y, 0), 1e-7)
    expected_mass = sum(row['solid_PETG_mass_kg'] for row in report['parts'] if row['name'].startswith('upper-frame'))
    assert result.mass_properties['upper-test-assembly']['mass_kg'] == pytest.approx(expected_mass, rel=1e-10)
    manifest = json.loads((result.output/'native-build.json').read_text())
    plan = json.loads((result.output/'print-first-plan.json').read_text())
    assert len(plan['print_files_sha256']) == 12
    for relative, expected in {**manifest['files_sha256'], **plan['print_files_sha256']}.items():
        assert hashlib.sha256((result.output/relative).read_bytes()).hexdigest() == expected
    assert all((result.output/'cad'/(name+'.stl')).stat().st_size > 84 for name in result.parts)
    assert 'not modeled' in ' '.join(plan['limitations'])


def test_exported_plate_stls_contain_the_actual_solid_surfaces(original_and_adapted):
    result, _, _ = original_and_adapted
    triangle_type = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    for name, shape in result.parts.items():
        binary = (result.output/'print-local'/f'{name}.stl').read_bytes()
        triangles = struct.unpack_from('<I', binary, 80)[0]
        assert triangles > 0 and len(binary) == 84+50*triangles
        vertices = np.frombuffer(binary, dtype=triangle_type, count=triangles, offset=84)['vertices'].astype(float)
        assert np.isfinite(vertices).all()
        assert vertices.reshape(-1, 3).min(axis=0) == pytest.approx([5, 5, 0], abs=.03)
        mesh_volume = abs(np.einsum('ij,ij->i', vertices[:, 0], np.cross(vertices[:, 1], vertices[:, 2])).sum()/6)
        assert mesh_volume == pytest.approx(shape.Volume(), rel=.003)


def test_generalized_tube_splice_and_open_captive_pockets():
    tube = RoundedBoxTube(36, 28, 100, 4, 50)
    splice = SplitLapSplice(tube, 60, .4, .3, 3, (-20, -10, 10, 20), 2)
    assert splice.coupler.width_mm == pytest.approx(27.4)
    assert splice.coupler.height_mm == pytest.approx(19.4)
    pockets = CaptiveNutPattern((15, 85), (-9, 9), 1.1, 4, 1.5)
    hollow = pockets.cut(tube.native_shape(), tube)
    assert not hollow.isInside(cq.Vector(9, 15, 10.1), 1e-7)
    assert hollow.isInside(cq.Vector(9, 15, 13), 1e-7) is False  # through clearance hole
    shapes = splice.native_shapes(hollow)
    assert all(shape.isValid() and len(shape.Solids()) == 1 for shape in shapes.values())
    assert splice.interface_requirements(shapes).report()['passed']


@pytest.mark.parametrize('changes', [
    {'wall_mm':22}, {'inner_radius_mm':16}, {'outer_radius_mm':9}, {'length_mm':float('nan')},
])
def test_invalid_hollow_sections_reject_before_cad(changes):
    with pytest.raises(ValueError):
        RoundedBoxTube(**(dict(width_mm=60, height_mm=44, length_mm=168, wall_mm=6) | changes))


@pytest.mark.parametrize('changes', [
    {'overlap_length_mm':170}, {'gap_mm':80}, {'hole_offsets_y_mm':(-2, 2)},
    {'hole_offsets_y_mm':(12, 30)}, {'hole_offsets_y_mm':(-39, 39)},
    {'hole_offsets_y_mm':(-30, -28, 12, 30)}, {'radial_clearance_mm':20}, {'coupler_wall_mm':16},
])
def test_invalid_coupled_splices_reject_before_cad(changes):
    base = SplitLapSplice(RoundedBoxTube(60, 44, 168, 6, 148), 80, .3, .2, 5, (-30, -12, 12, 30), 3.1)
    with pytest.raises(ValueError):
        replace(base, **changes)


def test_sleeve_candidate_couples_bore_and_rejects_impossible_hardware_fit():
    beam = RoundedBoxTube(60, 44, 168, 6, 148)
    candidate = SleevedCrossBore(6, 4.5, 4, .1, 60)
    candidate.validate_for(beam)
    assert candidate.hole_radius_mm == pytest.approx(3.1)
    assert candidate.to_source()['bolt_radial_clearance_mm'] == .25
    assert replace(candidate, sleeve_outer_diameter_mm=8).hole_radius_mm == pytest.approx(4.1)
    with pytest.raises(ValueError, match='no positive wall'):
        replace(candidate, sleeve_outer_diameter_mm=4)
    with pytest.raises(ValueError, match='no clearance'):
        replace(candidate, bolt_diameter_mm=4.5)
    with pytest.raises(ValueError, match='nominal flush'):
        replace(candidate, sleeve_length_mm=59).validate_for(beam)


def test_client_rejects_unsupported_source_frame_and_corrupt_inputs(client, tmp_path):
    spec = json.loads(client.DEFAULT_SPEC.read_text())
    spec['upper_span']['height_mm'] = 50
    with pytest.raises(ValueError, match='original span dimensions'):
        client.build(tmp_path/'not-created', spec)
    assert not (tmp_path/'not-created').exists()
    spec = json.loads(client.DEFAULT_SPEC.read_text())
    spec['upper_pockets']['pocket_depth_mm'] = 7
    with pytest.raises(ValueError, match='outside wall'):
        client.specification(spec)
    for name in ('print-first-inputs.json', *client.INPUT_SHA256):
        shutil.copyfile(client.DATA/name, tmp_path/name)
    (tmp_path/'print-first-upper.brep').write_bytes(b'corrupt native input')
    with pytest.raises(ValueError, match='BREP hash mismatch'):
        client.verified_inputs(tmp_path)


def test_large_inner_corner_radius_cannot_close_the_captive_pocket(client):
    spec = json.loads(client.DEFAULT_SPEC.read_text())
    spec['fore_span']['inner_radius_mm'] = 10
    rounded = RoundedBoxTube(**spec['fore_span'])
    # This point is in the proposed pocket footprint but remains in the wall
    # above the rounded cavity. The old width-only guard accepted this edit.
    assert rounded.native_shape().isInside(cq.Vector(17, 96, 23.95), 1e-7)
    with pytest.raises(ValueError, match='flat inner roof'):
        client.specification(spec)


def test_changed_construction_code_rejects_before_outputs(client, tmp_path, monkeypatch):
    key = 'src/mechanism_lab/manufacturing_joints.py'
    copied = tmp_path/'manufacturing_joints.py'
    copied.write_bytes(client.CONSTRUCTION_CODE_PATHS[key].read_bytes())
    monkeypatch.setitem(client.CONSTRUCTION_CODE_PATHS, key, copied)
    assert client.construction_code_hashes() == client._LOADED_CODE_HASHES
    copied.write_bytes(copied.read_bytes()+b'\n# changed construction after import\n')
    with pytest.raises(ValueError, match='Construction code changed'):
        client.build(tmp_path/'rejected-output')
    assert not (tmp_path/'rejected-output').exists()


def test_planning_cube_includes_the_actual_plate_placement_margin(client, tmp_path):
    spec = json.loads(client.DEFAULT_SPEC.read_text())
    # The longest article is 180.85 mm, but its +5 mm plate origin occupies
    # 185.85 mm. Raw dimension checking incorrectly accepted this envelope.
    spec['planning_cube_mm'] = 181
    with pytest.raises(ValueError, match='plate placement margin'):
        client.build(tmp_path/'too-small-plate', spec)
    assert not (tmp_path/'too-small-plate').exists()
