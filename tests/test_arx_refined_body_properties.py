"""Independent analytic checks of current Refined body property assembly."""
from copy import deepcopy
import ast
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

cq = pytest.importorskip('cadquery')

from examples.engineering.arx_compact import refined
from mechanism_lab.physical_model import digest


SOURCE = {'repository': 'synthetic-property-contract', 'commit': 'not-an-arx-benchmark'}


def part(number, material, half_extents, center, *, mass=None, body='loaded_body'):
    frame = np.eye(4)
    frame[:3, 3] = center
    return dict(id=number, name=f'component_{number}', body=body, material=material,
        finish='unrelated-optical-finish', mass_override_kg=mass, status='Synthetic analytic fixture',
        shape=dict(op='transform', args=frame.ravel().tolist(), children=[
            dict(op='box', args=half_extents, children=[])]))


def box_tensor(mass, dimensions_mm):
    x, y, z = np.asarray(dimensions_mm) * .001
    return np.diag([mass*(y*y+z*z)/12, mass*(x*x+z*z)/12, mass*(x*x+y*y)/12])


def analytical_aggregate(masses, centers, tensors):
    # Direct analytic box moments, independent of CAD integration and the
    # implementation's shared aggregator. All vectors here are in SI units.
    total = sum(masses)
    center = np.average(centers, weights=masses, axis=0)
    inertia = np.zeros((3, 3))
    for mass, local_center, tensor in zip(masses, centers, tensors):
        displacement = np.asarray(local_center) - center
        inertia += tensor + mass*(np.dot(displacement, displacement)*np.eye(3)
                                  - np.outer(displacement, displacement))
    return total, center, inertia


def replacement_fixture():
    centers = np.array([[.015, -.004, .002], [-.025, .030, .005]])
    records = [dict(name=name, mass_kg=mass, com_m=center.tolist(),
        inertia_at_com_kg_m2=box_tensor(mass, dimensions).tolist(), provenance='Analytic box fixture')
        for name, mass, center, dimensions in zip(['retained', 'changed'], [.2, .4], centers, [[20, 10, 6], [8, 4, 12]])]
    mass, center, tensor = analytical_aggregate([.2, .4], centers,
        [np.array(row['inertia_at_com_kg_m2']) for row in records])
    body = dict(mass_kg=mass, com_m=center.tolist(), inertia_tensor_kg_m2=tensor.tolist(),
                components=deepcopy(records))
    after = dict(name='changed', mass_kg=.3, com_m=[-.020, .025, .007],
        inertia_at_com_kg_m2=box_tensor(.3, [7, 4, 10]).tolist(), provenance='Analytic replacement box')
    return body, records[1], after


@pytest.mark.parametrize('inventory', ['full_records', 'archival_names'])
def test_shared_component_replacement_matches_direct_analytic_reassembly(inventory):
    from mechanism_lab.native_construction import replace_mass_components
    body, before, after = replacement_fixture()
    if inventory == 'archival_names':
        body['components'] = [record['name'] for record in body['components']]
    original = deepcopy(body)
    measured = replace_mass_components(body, [dict(before=before, after=after)])
    expected = analytical_aggregate([.2, .3], [[.015, -.004, .002], after['com_m']],
        [box_tensor(.2, [20, 10, 6]), box_tensor(.3, [7, 4, 10])])
    assert body == original
    assert measured['mass_kg'] == pytest.approx(expected[0], abs=1e-14)
    assert measured['com_m'] == pytest.approx(expected[1], abs=1e-14)
    assert np.asarray(measured['inertia_tensor_kg_m2']) == pytest.approx(expected[2], abs=1e-14)
    assert measured['replacement_source_body_sha256'] == digest(body)


@pytest.mark.parametrize('damage', ['unknown_name', 'renamed_after', 'duplicate', 'changed_before', 'negative_final_mass', 'invalid_tensor'])
def test_shared_replacement_rejects_inventory_and_physical_errors(damage):
    from mechanism_lab.native_construction import replace_mass_components
    body, before, after = replacement_fixture()
    replacements = [dict(before=before, after=after)]
    if damage == 'unknown_name':
        before['name'] = after['name'] = 'absent'
    elif damage == 'renamed_after':
        after['name'] = 'different'
    elif damage == 'duplicate':
        replacements *= 2
    elif damage == 'changed_before':
        before['mass_kg'] = .41
    elif damage == 'negative_final_mass':
        body['components'] = ['retained', 'changed']
        before['mass_kg'] = 2.
    else:
        after['inertia_at_com_kg_m2'] = np.diag([1, 1, 4]).tolist()
    with pytest.raises(ValueError):
        replace_mass_components(body, replacements)


def test_mixed_material_local_parts_brep_replacement_and_declared_mass(tmp_path):
    first = part(7, 'aluminum', [10, 5, 3], [15, -4, 2])
    second = part(31, 'steel', [1, 1, 1], [-25, 30, 5], mass=.5)
    design = dict(units='mm', parts=[first, second])
    native = cq.Workplane('XY').box(4, 8, 12).val().rotate((0, 0, 0), (0, 0, 1), 90).translate((-25, 30, 5))
    path = tmp_path / 'component.brep'
    native.exportBrep(str(path))
    overrides = [dict(part='31', record_sha256=digest(second), path=path.name,
        sha256=hashlib.sha256(path.read_bytes()).hexdigest(), units='mm', frame='body-local')]
    inherited = {'loaded_body': dict(mass_kg=2., com_m=[1., 2., 3.],
        inertia_tensor_kg_m2=np.eye(3).tolist(), components=['unrelated archived inventory'])}
    original_archive = deepcopy(inherited)
    result = refined.current_body_properties(design, SOURCE, overrides, tmp_path, inherited)
    assert inherited == original_archive
    assert result['mass_inputs_complete'] and result['resolved_parts'] == result['part_count'] == 2
    assert result['source_mass_override_count'] == 1
    assert result['parts'][1]['geometry_source']['kind'] == 'brep'
    assert result['parts'][1]['declared_mass_properties']['override_applied']
    centers = np.array([[.015, -.004, .002], [-.025, .030, .005]])
    masses = [20*10*6*2700e-9, 4*8*12*7850e-9]
    for key, actual_masses in [('source_density', masses), ('declared_envelope_masses', [masses[0], .5])]:
        # 90-degree native rotation exchanges X/Y dimensions, without a second
        # application of either the encoded or archived capture transforms.
        expected = analytical_aggregate(actual_masses, centers,
            [box_tensor(actual_masses[0], [20, 10, 6]), box_tensor(actual_masses[1], [8, 4, 12])])
        body = result['bodies']['loaded_body'][key]
        assert body['mass_kg'] == pytest.approx(expected[0], rel=1e-12)
        assert body['com_m'] == pytest.approx(expected[1], abs=1e-12)
        assert np.asarray(body['inertia_tensor_kg_m2']) == pytest.approx(expected[2], abs=1e-14)
        assert abs(body['inertia_tensor_kg_m2'][0][1]) > 1e-7
    comparison = result['bodies']['loaded_body']['archival_comparison']
    assert comparison['comparable']
    assert comparison['mass_delta_kg'] == pytest.approx(masses[0]+.5-2.)
    assert result['inherited_mass_properties_sha256'] == digest(inherited)
    assert not result['inherited_mass_properties_modified']
    assert not result['dynamics_rerun']


def test_missing_density_stays_explicit_and_never_uses_archive_residual(tmp_path):
    design = dict(units='mm', parts=[part(1, 'aluminum', [2, 2, 2], [0, 0, 0]),
                                   part(5, 'pcb', [2, 2, .5], [20, 0, 0]),
                                   part(9, 'pcb', [2, 2, .5], [0, 0, 0], body='unknown_only')])
    archive = {'loaded_body': dict(mass_kg=500., com_m=[0, 0, 0], inertia_tensor_kg_m2=np.eye(3).tolist())}
    result = refined.current_body_properties(design, SOURCE, [], tmp_path, archive)
    assert result['part_count'] == 3 and result['resolved_parts'] == 1
    assert not result['mass_inputs_complete'] and result['unresolved_part_ids'] == [5, 9]
    body = result['bodies']['loaded_body']
    assert body['records_are_partial_subtotals']
    assert body['source_density']['mass_kg'] == pytest.approx(64*2700e-9)
    assert not body['archival_comparison']['comparable']
    assert 'mass_delta_kg' not in body['archival_comparison']
    assert result['bodies']['unknown_only']['source_density'] is None
    assert result['bodies']['unknown_only']['declared_envelope_masses'] is None


@pytest.mark.parametrize('override', [True, 0, -1, float('nan'), float('inf')])
def test_invalid_source_override_cannot_silently_fall_back_to_density(tmp_path, override):
    design = dict(units='mm', parts=[part(1, 'steel', [2, 2, 2], [0, 0, 0], mass=override)])
    with pytest.raises(ValueError):
        refined.current_body_properties(design, SOURCE, [], tmp_path, {})


def test_disconnected_native_components_remain_reported(tmp_path):
    first = part(1, 'steel', [2, 2, 2], [0, 0, 0])
    second = part(2, 'steel', [1, 1, 1], [10, 0, 0])
    first['shape'] = dict(op='union', args=[], children=[first['shape'], second['shape']])
    result = refined.current_body_properties(dict(units='mm', parts=[first]), SOURCE, [], tmp_path, {})
    assert result['parts'][0]['solid_count'] == 2
    assert result['bodies']['loaded_body']['disconnected_part_ids'] == [1]
    assert result['bodies']['loaded_body']['source_density']['mass_kg'] == pytest.approx(72*7850e-9)


def test_actual_source_delta_algorithm_matches_published_corrected_body_records():
    """Execute only the unchanged source's arithmetic AST, never its file writes."""
    path = Path(__file__).parent / 'fixtures/arx_latest/verify_geometry.py'
    payload = path.read_bytes()
    assert hashlib.sha256(payload).hexdigest() == 'f0dbdd6cbb2d69b5a114f65e88ecd65a89d207ac46a6532c4c0e294f7f253ae1'
    manifest, files = refined.load_bundle()
    design = json.loads(files['design.json'])
    archived = json.loads(files['mass-properties.json'])
    corrected = json.loads(files['refined-mass-properties.json'])
    published_deltas = json.loads(files['mass-property-deltas.json'])
    tree = ast.parse(payload, filename=str(path))
    nodes = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == 'shift']
    nodes += [node for node in tree.body if isinstance(node, ast.For)
              and isinstance(node.target, ast.Tuple)
              and [item.id for item in node.target.elts if isinstance(item, ast.Name)] == ['body', 'v']]
    assert len(nodes) == 2
    namespace = dict(np=np, D=design, r=json.loads(files['cad-refinement.json']), old=archived,
        new=deepcopy(archived), lookup={row['name']: row for row in design['parts']}, mass_report=[])
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), 'exec'), namespace)
    assert len(namespace['mass_report']) == len(corrected) == 9
    assert sum(len(row['changed_components']) for row in namespace['mass_report']) == 123
    for body, actual in namespace['new'].items():
        for field in ('mass_kg', 'com_m', 'inertia_tensor_kg_m2', 'fullinertia_mujoco_kg_m2'):
            assert np.asarray(actual[field]) == pytest.approx(np.asarray(corrected[body][field]), abs=1e-13)
        assert actual['components'] == corrected[body]['components']
    assert sum(row['delta_grams'] for row in namespace['mass_report']) == pytest.approx(published_deltas['total_mass_delta_g'], abs=1e-10)
    assert manifest['files']['refined-mass-properties.json']['git_blob_sha'] == 'f51d95968be91120ac64820a585b45f3e07eb190'
