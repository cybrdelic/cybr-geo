"""Physical invariants of reusable native CAD bookkeeping, independent of ARX."""
import json
import hashlib

import cadquery as cq
import numpy as np
import pytest

from mechanism_lab.native_construction import (
    NativeConstruction, aggregate_mass_properties, shape_mass_properties,
    transform_shape, validate_inertia, read_specification, NativeInterfaceRequirements,
)


def test_native_volume_mass_override_and_center():
    shape = cq.Workplane('XY').box(10, 20, 30).translate((40, -10, 5))
    mass, center, inertia = shape_mass_properties(shape, 2700)
    assert mass == pytest.approx(.0162)
    assert center == pytest.approx([.04, -.01, .005])
    assert np.diag(inertia) == pytest.approx(mass/12*np.array([.02**2+.03**2, .01**2+.03**2, .01**2+.02**2]))
    overridden, overridden_center, overridden_inertia = shape_mass_properties(shape, 2700, mass_kg=.1)
    assert overridden == .1
    assert overridden_center == pytest.approx(center)
    assert overridden_inertia == pytest.approx(inertia*.1/mass)


def test_specification_path_dict_and_formatting_have_one_canonical_identity(tmp_path):
    from mechanism_lab.physical_model import digest
    original = {'z':.125, 'a':{'limits':[1, 2.5], 'label':'nominal \u03bc'}}
    pretty, compact = tmp_path/'pretty.json', tmp_path/'compact.json'
    pretty.write_text(json.dumps(original, indent=2)+'\n', encoding='utf-8')
    compact.write_text(json.dumps(original, sort_keys=True, separators=(',', ':')), encoding='utf-8')
    document, identity = read_specification(original)
    loaded, file_identity = read_specification(pretty)
    _, compact_identity = read_specification(compact)
    assert document == loaded == original
    assert identity['input_spec_sha256'] == file_identity['input_spec_sha256'] == compact_identity['input_spec_sha256'] == digest(original)
    assert identity['canonical_specification_sha256'] == digest(original)
    assert 'input_spec_file_sha256' not in identity
    assert file_identity['input_spec_file_sha256'] == hashlib.sha256(pretty.read_bytes()).hexdigest()
    assert file_identity['input_spec_file_sha256'] != compact_identity['input_spec_file_sha256']
    loaded['a']['limits'][0] = 9
    assert original['a']['limits'][0] == 1


def test_specification_rejects_ambiguous_or_nonfinite_input(tmp_path):
    duplicate = tmp_path/'duplicate.json'
    duplicate.write_text('{"mass":1,"mass":2}')
    with pytest.raises(ValueError, match='Duplicate'):
        read_specification(duplicate)
    with pytest.raises(ValueError, match='Invalid native specification'):
        read_specification({'mass':float('nan')})
    with pytest.raises(ValueError, match='specification object'):
        read_specification([])


def test_specification_digest_retains_parameter_changes():
    _, first = read_specification({'mass':1.})
    _, changed = read_specification({'mass':1.000000000000001})
    assert first['canonical_specification_sha256'] != changed['canonical_specification_sha256']


def test_parallel_axis_aggregation_retains_mass_records():
    records = [dict(name=str(x), mass_kg=2., com_m=[x, 0, 0],
                    inertia_at_com_kg_m2=np.diag([.1, .1, .1]).tolist(),
                    provenance='Explicit test inertial') for x in (-.5, .5)]
    body = aggregate_mass_properties(records)
    assert body['mass_kg'] == 4
    assert body['com_m'] == [0, 0, 0]
    assert np.asarray(body['inertia_tensor_kg_m2']) == pytest.approx(np.diag([.2, 1.2, 1.2]))
    assert body['components'] == records
    with pytest.raises(ValueError, match='triangle'):
        validate_inertia(np.diag([1., 1., 3.]))


def test_axis_placement_preserves_volume():
    shape = cq.Workplane('XY').circle(2).extrude(10)
    placed = transform_shape(shape, pos=(2, 3, 4), axis=(0, -1, 0))
    assert placed.val().Volume() == pytest.approx(shape.val().Volume())
    assert placed.val().Center().toTuple() == pytest.approx((2, -2, 4))


def test_build_registry_isolated_and_exports_declared_geometry(tmp_path):
    unrelated = tmp_path/'retained.txt'
    unrelated.write_text('keep')
    (tmp_path/'native-build.json').write_text('{"stale":true}')
    build = NativeConstruction(tmp_path, {'alloy':2700}, body_ids=['link'])
    assert not (tmp_path/'native-build.json').exists()
    build.addpart('plate', cq.Workplane('XY').box(10, 20, 30), 'alloy', 'Nominal part')
    build.place_part('link', 'plate', pos=(15, 0, 0))
    result = build.finish('Test body')
    assert result.placed_shapes.keys() == {('link', 'plate_0')}
    assert result.mass_properties['link']['com_m'] == pytest.approx([.015, 0, 0])
    manifest = json.loads((tmp_path/'native-build.json').read_text())
    assert manifest['placements'] == 1
    assert len(manifest['provenance']['native_construction_sha256']) == 64
    assert 'cad/placed/link__plate_0.step' in manifest['files_sha256']
    assert unrelated.read_text() == 'keep'
    other = NativeConstruction(tmp_path/'second', {'alloy':2700})
    assert not other.parts and not other.body_records
    with pytest.raises(ValueError, match='Duplicate'):
        build.addpart('plate', cq.Workplane('XY').box(1, 1, 1), 'alloy')


def test_native_interface_failure_prevents_build_acceptance(tmp_path):
    build = NativeConstruction(tmp_path, {'alloy':2700})
    first = cq.Workplane('XY').box(10, 10, 10)
    second = first.translate((5, 0, 0))
    requirements = NativeInterfaceRequirements({'source':'authored fit'})
    row = requirements.noninterference('shaft_vs_housing', first, second)
    assert row['intersection_volume_mm3'] == pytest.approx(500)
    with pytest.raises(ValueError, match='shaft_vs_housing'):
        build.finish('Rejected construction', native_requirements=requirements)
    assert not (tmp_path/'native-build.json').exists()
    report = json.loads((tmp_path/'native-interface-requirements.json').read_text())
    assert not report['passed']
    assert report['checks'][0]['volume_tolerance_mm3'] == 1e-5


def test_touching_nominal_interface_is_measured_and_receipt_bound(tmp_path):
    build = NativeConstruction(tmp_path, {'alloy':2700})
    first = cq.Workplane('XY').box(10, 10, 10)
    second = first.translate((10, 0, 0))
    build.addpart('plate', first, 'alloy')
    build.place_part('link', 'plate')
    requirements = NativeInterfaceRequirements()
    row = requirements.noninterference('intentional_face_contact', first, second)
    assert row['pass'] and row['minimum_distance_mm'] == 0
    build.finish('Nominal fit only', native_requirements=requirements)
    manifest = json.loads((tmp_path/'native-build.json').read_text())
    report = tmp_path/'native-interface-requirements.json'
    assert manifest['files_sha256'][report.name] == hashlib.sha256(report.read_bytes()).hexdigest()
    with pytest.raises(ValueError, match='Duplicate'):
        requirements.noninterference('intentional_face_contact', first, second)
    with pytest.raises(ValueError, match='finite and nonnegative'):
        requirements.noninterference('invalid_tolerance', first, second, allowed_volume_mm3=float('nan'))


def test_assembly_mass_uses_actual_handed_components_and_parent_locations(tmp_path):
    block = cq.Workplane('XY').box(10, 20, 30)
    child = cq.Assembly(name='handed', loc=cq.Location(cq.Vector(30, 0, 0)))
    child.add(block, name='body', loc=cq.Location(cq.Vector(5, 0, 0)))
    root = cq.Assembly(name='opposed')
    root.add(child)
    build = NativeConstruction(tmp_path, {'alloy':2700})
    with pytest.raises(ValueError, match='explicit material'):
        build.add_assembly_mass('jaw', root, {})
    build.add_assembly_mass('jaw', root, {'opposed/handed/body':'alloy'}, offset_mm=(0,-20,0))
    result = aggregate_mass_properties(build.body_records['jaw'])
    assert result['mass_kg'] == pytest.approx(.0162)
    assert result['com_m'] == pytest.approx([.035,-.02,0])
    assert result['components'][0]['material'] == 'alloy'
    assert not build.placed_records and not build.parts
