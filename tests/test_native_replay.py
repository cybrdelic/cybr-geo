"""Real tiny native builds exercise replay identity and analytic export boundaries."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import shutil

import cadquery as cq
import numpy as np
import pytest

from mechanism_lab.core import Material, pose_cad
from mechanism_lab.engineering_source import (
    EngineeringProject, construct_project_native, project_source_identity,
)
from mechanism_lab.native_construction import NativeConstruction
from mechanism_lab.native_replay import build_assembly, mesh_record
from mechanism_lab.physical_model import PhysicalModel, digest
from mechanism_lab.physical_replay import Y_UP_TO_Z_UP, pose


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n', encoding='utf-8')


@pytest.fixture(scope='module')
def completed_native(tmp_path_factory):
    root = tmp_path_factory.mktemp('native-replay-source')
    source = root / 'project'
    source.mkdir()
    (source / 'design.py').write_text('# Synthetic native replay test constructor.\n')
    spec = {'size_mm': [10, 20, 30]}

    def construct(specification, output):
        native = NativeConstruction(output, {'metal': 2700},
            provenance={'input_spec_sha256': digest(specification)})
        shape = cq.Workplane('XY').box(*specification['size_mm']).val().translate((5, 0, 1))
        native.addpart('plate', shape, 'metal')
        native.place('local_plate', 'one', shape, 'metal')
        native.finish('Synthetic test density, not measured material')

    project = EngineeringProject('fixture', spec, construct, None, None, None, None)
    sources = project_source_identity(source)
    directory = root / 'native'
    receipt, binding = construct_project_native(project, source, sources, directory)
    masses = json.loads((directory / 'mass_properties.json').read_text())
    physical = PhysicalModel.from_xml('<mujoco><worldbody><body name="carriage"/>'
        '<body name="loose"/></worldbody></mujoco>', provenance={
            'native_build_sha256': digest(receipt),
            'native_source_binding_sha256': digest(binding)})
    physical.bind_inertias(masses, {'carriage': 'local_plate'})
    parameters = {'construction': deepcopy(spec), 'task': 'synthetic stored poses'}
    reference = {'task_parameters': deepcopy(parameters), 'source_files': sources}
    frames = [dict(time_s=time, bodies={
        'carriage': dict(position_m=[time + .1, .2, .3],
                         rotation_row_major=[0,-1,0,1,0,0,0,0,1]),
        'loose': dict(position_m=[.7, .4-time*.1, 0], rotation_row_major=np.eye(3).ravel().tolist()),
        'world': dict(position_m=[0,0,0], rotation_row_major=np.eye(3).ravel().tolist()),
    }) for time in (0., .5, 1.)]
    result = dict(schema='cybrgeo-causal-simulation/1', frames=frames,
        physical_source=physical.to_source(), model_sha256=digest(physical.to_source()),
        control_sha256=digest({'synthetic': 'fixture'}), task_parameters=parameters,
        reference_source=reference, reference_sha256=digest(reference))
    return directory, spec, result


@pytest.fixture
def fixture(completed_native, tmp_path):
    directory, spec, result = completed_native
    copied = tmp_path / 'native'
    shutil.copytree(directory, copied)
    return copied, deepcopy(spec), deepcopy(result)


def build(fixture, **kwargs):
    directory, specification, result = fixture
    options = dict(specification=specification,
        body_mapping={'local_plate': 'carriage'}, mass_mapping={'carriage': 'local_plate'},
        materials={'metal': Material('Test appearance', (.4, .5, .6), .5, .3)},
        world_rotation=Y_UP_TO_Z_UP)
    options.update(kwargs)
    return build_assembly(directory, result, **options)


def change_completed_artifact(fixture, filename, mutate):
    """Rebind a synthetic receipt to test deeper manifest validation."""
    directory, _, result = fixture
    path = directory / filename
    value = json.loads(path.read_text())
    mutate(value)
    write_json(path, value)
    receipt_path = directory / 'native-build.json'
    receipt = json.loads(receipt_path.read_text())
    receipt['files_sha256'][filename] = hashlib.sha256(path.read_bytes()).hexdigest()
    write_json(receipt_path, receipt)
    rebind_receipt(fixture)


def rebind_receipt(fixture):
    directory, _, result = fixture
    receipt_path = directory / 'native-build.json'
    receipt = json.loads(receipt_path.read_text())
    binding_path = directory / 'native-source-binding.json'
    binding = json.loads(binding_path.read_text())
    binding.update(native_build_sha256=digest(receipt),
                   native_build_file_sha256=hashlib.sha256(receipt_path.read_bytes()).hexdigest())
    write_json(binding_path, binding)
    result['physical_source']['provenance'].update(native_build_sha256=digest(receipt),
                                                   native_source_binding_sha256=digest(binding))
    result['model_sha256'] = digest(result['physical_source'])


def test_real_native_cad_basis_and_export_are_not_double_transformed(fixture, tmp_path):
    from mechanism_lab.exporters import export_step
    assembly = build(fixture)
    part = assembly.parts[0]
    assert part.cad is not None and part.cad.isValid()
    np.testing.assert_allclose(part.cad.Center().toTuple(), [5, 0, 1], atol=1e-9)
    world = pose_cad(part.cad, assembly.pose(part, 0))
    np.testing.assert_allclose(world.Center().toTuple(), [100, -301, 205], atol=1e-9)
    coverage = export_step(assembly, tmp_path/'replay.step', individual=False)
    assert coverage['analytic_components'] == ['local_plate__one']
    assert not coverage['mesh_only_components_not_in_STEP']
    reopened = cq.importers.importStep(str(tmp_path/'replay.step')).val()
    assert reopened.isValid() and len(reopened.Solids()) == 1
    assert reopened.Volume() == pytest.approx(6000)
    np.testing.assert_allclose(reopened.Center().toTuple(), world.Center().toTuple(), atol=1e-7)
    np.testing.assert_array_equal(assembly.pose(part, .499), assembly.pose(part, 0))
    assembly.motion_function = pose.bind(assembly)
    np.testing.assert_array_equal(assembly.pose(part, .5)[:3,3], [600, -300, 200])
    assert assembly.metadata['reference_sha256'] == fixture[2]['reference_sha256']
    assert assembly.metadata['qualification']['hardware'] == 'not qualified'


def test_explicit_scene_cad_remains_an_independent_integrated_body(fixture):
    shape = cq.Workplane('XY').sphere(2).val()
    row = mesh_record(shape, body='scene', name='free', material='metal')
    assembly = build(fixture, body_mapping={'local_plate':'carriage','scene':'loose'},
                     scene_records=[row], scene_shapes={'scene__free': shape})
    part = assembly.parts[1]
    np.testing.assert_allclose(assembly.pose(part, .5)[:3,3], [700, 0, 350])
    assert part.cad is shape
    assert row['body'] == 'scene'  # Input records are not rewritten.


def test_scene_cad_can_supply_mesh_from_metadata_only(fixture):
    shape = cq.Workplane('XY').box(2, 4, 6).val()
    assembly = build(fixture, body_mapping={'local_plate':'carriage','scene':'loose'},
        scene_records=[{'body':'scene','name':'free','material':'metal'}],
        scene_shapes={'scene__free':shape})
    np.testing.assert_allclose(assembly.parts[1].bounds, [[-1,-2,-3],[1,2,3]])
    assert assembly.parts[1].cad is shape


def test_scene_mesh_cannot_diverge_from_supplied_cad(fixture):
    shape = cq.Workplane('XY').box(2, 4, 6).val()
    row = mesh_record(shape, body='scene', name='free', material='metal')
    with pytest.raises(ValueError, match='Scene display mesh differs'):
        build(fixture, body_mapping={'local_plate':'carriage','scene':'loose'},
              scene_records=[row], scene_shapes={'scene__free':shape.translate((50,0,0))})


def test_mesh_only_inspection_is_explicit_and_does_not_gain_cad(fixture):
    assembly = build(fixture, analytic=False)
    assert assembly.parts[0].cad is None
    assert assembly.metadata['analytic_replay'] is False


@pytest.mark.parametrize('change,message', [
    (lambda s:s.pop('physical_source'), 'physical source'),
    (lambda s:s.update(model_sha256='stale'), 'model identity'),
    (lambda s:s['task_parameters'].update(task='changed'), 'task parameters'),
    (lambda s:s.update(reference_sha256='stale'), 'reference source'),
    (lambda s:s['frames'][1]['bodies'].pop('carriage'), 'Missing recorded body'),
])
def test_unbound_or_incomplete_runs_fail(fixture, change, message):
    change(fixture[2])
    with pytest.raises(ValueError, match=message):
        build(fixture)


def test_changed_source_closure_even_with_recomputed_reference_fails(fixture):
    result = fixture[2]
    result['reference_source']['source_files']['design.py'] = 'changed'
    result['reference_sha256'] = digest(result['reference_source'])
    with pytest.raises(ValueError, match='source/data closure'):
        build(fixture)


def test_parameter_identity_preserves_json_numeric_types(fixture):
    fixture[2]['task_parameters']['construction']['size_mm'][0] = 10.0
    with pytest.raises(ValueError, match='task parameters'):
        build(fixture)


def test_rehashed_model_cannot_change_native_mass_provenance(fixture):
    result = fixture[2]
    result['physical_source']['mass_records']['carriage']['status'] = 'other provenance'
    result['model_sha256'] = digest(result['physical_source'])
    with pytest.raises(ValueError, match='Mapped native mass records differ'):
        build(fixture)


@pytest.mark.parametrize('key', ['native_build_sha256', 'native_source_binding_sha256'])
def test_rehashed_physical_source_cannot_substitute_other_native_build(fixture, key):
    result = fixture[2]
    result['physical_source']['provenance'][key] = 'different'
    result['model_sha256'] = digest(result['physical_source'])
    with pytest.raises(ValueError, match='not bound'):
        build(fixture)


def test_changed_native_bytes_fail_before_replay(fixture):
    (fixture[0]/'geometry-meshes.json').write_text('{}')
    with pytest.raises(ValueError, match='Native artifact differs'):
        build(fixture)


def test_wrong_specification_fails(fixture):
    fixture[1]['size_mm'][0] += 1
    with pytest.raises(ValueError, match='different design specification'):
        build(fixture)


@pytest.mark.parametrize('mapping', [{}, {'carriage':'missing'}, {'carriage':'local_plate','loose':'local_plate'}])
def test_missing_or_extra_mass_bindings_fail(fixture, mapping):
    with pytest.raises(ValueError, match='Mass mapping'):
        build(fixture, mass_mapping=mapping)


@pytest.mark.parametrize('mapping', [{}, {'local_plate':'invented'}, {'local_plate':'carriage','extra':'loose'}])
def test_body_aliases_require_exact_declared_coverage(fixture, mapping):
    with pytest.raises(ValueError, match='[Bb]ody mapping'):
        build(fixture, body_mapping=mapping)


def test_native_mesh_cannot_move_on_body_other_than_its_inertia(fixture):
    with pytest.raises(ValueError, match='disagrees with its CAD inertia binding'):
        build(fixture, body_mapping={'local_plate':'loose'})


def test_scene_cannot_replace_native_or_omit_required_cad(fixture):
    shape = cq.Workplane('XY').sphere(2).val()
    row = mesh_record(shape, body='local_plate', name='one', material='metal')
    with pytest.raises(ValueError, match='cannot replace'):
        build(fixture, scene_records=[row])
    row.update(body='scene', name='extra')
    with pytest.raises(ValueError, match='Scene analytic CAD identities'):
        build(fixture, scene_records=[row])


@pytest.mark.parametrize('mutation,message', [
    (lambda x:x['records'].append(deepcopy(x['records'][0])), 'Duplicate'),
    (lambda x:x['records'][0].update(name='different'), 'identities do not match'),
    (lambda x:x['records'][0].update(step_file='../outside.step'), 'escapes'),
    (lambda x:x.update(units='m'), 'explicitly use mm'),
    (lambda x:x['records'][0].update(material='wrong'), 'material binding'),
])
def test_bound_but_invalid_placement_manifest_fails(fixture, mutation, message):
    change_completed_artifact(fixture, 'placed-native-parts.json', mutation)
    with pytest.raises(ValueError, match=message):
        build(fixture)


def test_loader_rejects_required_file_omitted_from_completion_receipt(fixture):
    path = fixture[0] / 'native-build.json'
    receipt = json.loads(path.read_text())
    receipt['files_sha256'].pop('geometry-meshes.json')
    write_json(path, receipt)
    rebind_receipt(fixture)
    with pytest.raises(ValueError, match='absent from the completion receipt'):
        build(fixture)


def test_imported_pass_flags_and_metadata_cannot_qualify_replay(fixture):
    fixture[2]['passed'] = True
    assembly = build(fixture, metadata={'native_build_sha256':'forged',
                      'qualification':{'hardware':'qualified','continuous_clearance':'passed'}})
    assert assembly.metadata['native_build_sha256'] != 'forged'
    assert assembly.metadata['qualification']['hardware'] == 'not qualified'
    assert assembly.metadata['qualification']['continuous_clearance'] == 'not established by replay'
