"""One located component source must feed exported CAD, mesh and inertia."""
import json

import cadquery as cq
import numpy as np
import pytest

from mechanism_lab.native_construction import NativeConstruction


def test_located_mixed_material_assembly_has_one_geometry_and_mass_authority(tmp_path):
    shapes = {'alloy': cq.Workplane('XY').box(10, 20, 30).val(),
              'steel': cq.Workplane('XY').box(6, 8, 12).val()}
    offsets = {'alloy': (5, 7, 11), 'steel': (-13, 17, 19)}
    densities = {'alloy': 2700., 'steel': 7850.}
    child = cq.Assembly(name='child', loc=cq.Location(cq.Vector(30, 0, 0), cq.Vector(0, 0, 1), 90))
    for name, shape in shapes.items():
        child.add(shape, name=name, loc=cq.Location(cq.Vector(*offsets[name])))
    root = cq.Assembly(name='root', loc=cq.Location(cq.Vector(0, 40, 0), cq.Vector(1, 0, 0), 90))
    root.add(child)
    materials = {'root/child/'+name: name for name in shapes}
    names = {'root/child/'+name: name+'_located' for name in shapes}
    construction = NativeConstruction(tmp_path, densities)
    construction.place_assembly('moving', root, materials, component_names=names,
                                offset_mm=(0, -20, 10), provenance='Analytic test blocks')
    result = construction.finish('Two exact located blocks')
    assert len(result.placed_records) == len(result.meshes) == 2
    assert len(result.mass_properties['moving']['components']) == 2
    assert len(json.loads((tmp_path/'native-build.json').read_text())['files_sha256']) == 7

    # Independent closed-form cuboid properties and explicit quarter turns.
    rx = np.array([[1., 0, 0], [0, 0, -1], [0, 1, 0]])
    rz = np.array([[0., -1, 0], [1, 0, 0], [0, 0, 1]])
    rotation = rx @ rz
    dimensions = {'alloy': np.array([10., 20., 30.]), 'steel': np.array([6., 8., 12.])}
    masses, centers, tensors = [], [], []
    for name in shapes:
        center_mm = np.array([0., 40., 0.]) + rx @ np.array([30., 0., 0.]) + rotation @ offsets[name] + [0., -20., 10.]
        size = dimensions[name] * .001
        mass = float(np.prod(size)*densities[name])
        tensor = rotation @ np.diag(mass/12*(np.dot(size, size)-size**2)) @ rotation.T
        path = tmp_path/'cad'/'placed'/('moving__'+names['root/child/'+name]+'.step')
        reopened = cq.importers.importStep(str(path)).val()
        assert reopened.isValid()
        assert reopened.Volume() == pytest.approx(float(np.prod(dimensions[name])), rel=1e-10)
        assert reopened.Center().toTuple() == pytest.approx(center_mm, abs=1e-10)
        record = next(row for row in result.meshes if row['name'] == names['root/child/'+name])
        assert record['body'] == 'moving' and record['material'] == name
        assert np.mean(np.asarray(record['vertices_mm']), axis=0) == pytest.approx(center_mm, abs=1e-5)
        masses.append(mass)
        centers.append(center_mm*.001)
        tensors.append(tensor)
    total = sum(masses)
    center = sum(mass*point for mass, point in zip(masses, centers))/total
    inertia = sum(tensor + mass*(np.dot(point-center, point-center)*np.eye(3)-np.outer(point-center, point-center))
                  for mass, point, tensor in zip(masses, centers, tensors))
    actual = result.mass_properties['moving']
    assert actual['mass_kg'] == pytest.approx(total, rel=1e-12)
    assert actual['com_m'] == pytest.approx(center, abs=1e-12)
    assert np.asarray(actual['inertia_tensor_kg_m2']) == pytest.approx(inertia, rel=1e-10, abs=1e-14)


def test_assembly_placement_rejects_ambiguous_names_before_export(tmp_path):
    assembly = cq.Assembly(name='root')
    for name in ('a', 'b'):
        assembly.add(cq.Workplane('XY').box(1, 2, 3).val(), name=name)
    construction = NativeConstruction(tmp_path, {'alloy': 2700.})
    with pytest.raises(ValueError, match='Duplicate'):
        construction.place_assembly('body', assembly, {'root/a': 'alloy', 'root/b': 'alloy'},
                                   component_names={'root/a': 'same', 'root/b': 'same'})
    assert not construction.placed_records and not construction.body_records
    assert not list((tmp_path/'cad'/'placed').glob('*.step'))
