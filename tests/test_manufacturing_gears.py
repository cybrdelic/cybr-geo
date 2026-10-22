"""Actual P6B native construction parity and declared transmission scaling."""
from contextlib import redirect_stdout
from dataclasses import asdict
import hashlib
import importlib.util
import io
import json
import math
from pathlib import Path

import cadquery as cq
import numpy as np
import pytest

from mechanism_lab.manufacturing_gears import InvoluteRackPair, SOURCE_SHA256
from mechanism_lab.native_construction import NativeConstruction


@pytest.fixture(scope='module')
def original_and_adapted(tmp_path_factory):
    source = Path(__file__).parent/'fixtures/arx_latest/printed_gear_trials.py'
    assert hashlib.sha256(source.read_bytes()).hexdigest() == SOURCE_SHA256
    spec = importlib.util.spec_from_file_location('original_p6b_gear_trials', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    output = tmp_path_factory.mktemp('original-p6b-gear')
    for directory in ('cad', 'stl', 'reports'):
        (output/directory).mkdir()
    # The unmodified, explicitly invoked upstream main runs in isolated output.
    # Importing it does not execute the builder or create source-tree outputs.
    module.ROOT = output
    with redirect_stdout(io.StringIO()):
        module.main()
    assert hashlib.sha256(source.read_bytes()).hexdigest() == SOURCE_SHA256
    report = json.loads((output/'reports/printed-gear-trials.json').read_text())
    shapes = InvoluteRackPair().native_shapes()
    source_names = {'pinion':'gripper-gear-tooth-trial-m1p2-z20', 'rack':'gripper-rack-tooth-trial-m1p2'}
    original = {name:cq.importers.importStep(str(output/'cad'/f'{source_name}.step')).val()
                for name, source_name in source_names.items()}
    return shapes, original, report, source_names


def _vertices(shape):
    return sorted(tuple(round(x, 8) for x in vertex.toTuple()) for vertex in shape.Vertices())


@pytest.mark.parametrize('name', ['pinion', 'rack'])
def test_default_solids_match_executed_actual_source(original_and_adapted, name):
    shapes, original, report, source_names = original_and_adapted
    shape, reference = shapes[name], original[name]
    record = next(row for row in report['parts'] if row['name'] == source_names[name])
    assert shape.isValid() and len(shape.Solids()) == 1
    assert reference.isValid() and len(reference.Solids()) == 1
    assert shape.Volume() == pytest.approx(record['volume_mm3'], rel=1e-11)
    assert shape.Volume() == pytest.approx(reference.Volume(), rel=1e-10)
    assert shape.Center().toTuple() == pytest.approx(reference.Center().toTuple(), abs=1e-9)
    assert np.asarray(cq.Shape.matrixOfInertia(shape)) == pytest.approx(np.asarray(cq.Shape.matrixOfInertia(reference)), rel=1e-9, abs=1e-7)
    assert len(shape.Faces()) == len(reference.Faces())
    assert len(shape.Edges()) == len(reference.Edges())
    assert np.asarray(_vertices(shape)) == pytest.approx(np.asarray(_vertices(reference)), abs=1e-8)
    assert sorted(edge.geomType() for edge in shape.Edges()) == sorted(edge.geomType() for edge in reference.Edges())
    bb = shape.BoundingBox()
    assert [bb.xlen, bb.ylen, bb.zlen] == pytest.approx(record['dimensions_mm'], abs=1e-7)


def test_native_registry_derives_mass_and_roundtrips_source_geometry(original_and_adapted, tmp_path):
    shapes, _, report, source_names = original_and_adapted
    specification = InvoluteRackPair().to_source()
    builder = NativeConstruction(tmp_path, {'PETG':1250}, provenance=specification)
    for name, shape in shapes.items():
        builder.addpart(name, shape, 'PETG', 'Source-derived tooth test article; no load rating')
        builder.place_part(name, name)
    result = builder.finish('Separate geometric test articles; not a meshing or robot assembly')
    manifest = json.loads((tmp_path/'native-build.json').read_text())
    assert manifest['placements'] == 2 and len(manifest['files_sha256']) > 2
    assert result.provenance['source']['sha256'] == SOURCE_SHA256
    for name, shape in shapes.items():
        record = next(row for row in report['parts'] if row['name'] == source_names[name])
        assert result.mass_properties[name]['mass_kg'] == pytest.approx(record['solid_PETG_mass_g']/1000, rel=1e-11)
        assert result.mass_properties[name]['com_m'] == pytest.approx(np.asarray(shape.Center().toTuple())*.001, abs=1e-12)
        actual = cq.importers.importStep(str(tmp_path/'cad'/f'{name}.step')).val()
        assert actual.isValid() and len(actual.Solids()) == 1
        assert actual.Volume() == pytest.approx(record['volume_mm3'], rel=1e-10)
        assert (tmp_path/'cad'/f'{name}.stl').stat().st_size > 0
    for relative, expected in manifest['files_sha256'].items():
        assert hashlib.sha256((tmp_path/relative).read_bytes()).hexdigest() == expected


def test_nondefault_native_shapes_use_one_declared_tooth_specification():
    pair = InvoluteRackPair(module_mm=.8, teeth=24, face_width_mm=7.5,
                            rack_teeth=5, rack_back_depth_mm=4., pinion_bore_mm=0.)
    shapes = pair.native_shapes()
    assert all(shape.isValid() and len(shape.Solids()) == 1 for shape in shapes.values())
    assert shapes['pinion'].isInside(cq.Vector(0, 0, 3), 1e-7)
    assert shapes['pinion'].BoundingBox().zlen == pytest.approx(7.5)
    assert shapes['rack'].BoundingBox().xlen == pytest.approx(5*math.pi*.8)
    p = pair.to_source()['dimensions_mm_and_radians']
    tooth_area = (p['rack_tip_half_width_mm']+p['rack_root_half_width_mm'])*2.25*pair.module_mm
    backing_area = pair.rack_teeth*p['pitch_mm']*(pair.rack_back_depth_mm-1.25*pair.module_mm)
    assert shapes['rack'].Volume() == pytest.approx((backing_area+pair.rack_teeth*tooth_area)*pair.face_width_mm)
    assert InvoluteRackPair(**asdict(pair)) == pair


def test_pitch_kinematics_and_force_demand_have_explicit_units():
    pair = InvoluteRackPair()
    assert pair.pitch_radius_m == pytest.approx(.012)
    assert pair.pinion_torque_for_rack_force(100.) == pytest.approx(1.2)
    assert pair.rack_force_for_pinion_torque(1.2) == pytest.approx(100.)
    assert pair.pinion_torque_for_rack_force(100., efficiency=.8) == pytest.approx(1.5)
    assert pair.rack_force_for_pinion_torque(1.5, efficiency=.8) == pytest.approx(100.)
    displacement = pair.rack_travel_for_pinion_angle(-math.pi/2)
    assert displacement == pytest.approx(-.012*math.pi/2)
    assert pair.pinion_angle_for_rack_travel(displacement) == pytest.approx(-math.pi/2)
    assert 1.2*(-math.pi/2) == pytest.approx(100.*displacement)
    change = pair.scaling_from(module_mm=.3, teeth=20)
    assert change['pitch_radius_ratio'] == pytest.approx(4.)
    assert change['torque_ratio_for_same_rack_force'] == pytest.approx(4.)
    assert change['rack_force_ratio_at_same_pinion_torque'] == pytest.approx(.25)
    assert change['rack_travel_ratio_per_pinion_radian'] == pytest.approx(4.)
    metadata = json.loads(json.dumps(pair.to_source(), allow_nan=False))
    assert metadata['parameters'] == asdict(pair)
    assert metadata['source']['sha256'] == SOURCE_SHA256
    assert 'no meshing assembly' in metadata['scope']


def test_minimum_wall_accounts_for_polygonal_root_chords():
    # A 19 mm bore leaves 1 mm to the nominal 10.5 mm root circle, but
    # 0.999622894 mm to the source's straight sampled root edges.
    with pytest.raises(ValueError, match='minimum wall'):
        InvoluteRackPair(pinion_bore_mm=19., minimum_wall_mm=1.)
    pair = InvoluteRackPair(pinion_bore_mm=18.999, minimum_wall_mm=1.)
    p = pair.to_source()['dimensions_mm_and_radians']
    assert p['root_polygon_apothem_mm'] == pytest.approx(10.499622894376048)
    assert p['root_polygon_apothem_mm']-pair.pinion_bore_mm/2 >= pair.minimum_wall_mm


@pytest.mark.parametrize('changes', [
    {'module_mm':float('nan')}, {'module_mm':0.}, {'module_mm':1e308},
    {'teeth':True}, {'teeth':20.5}, {'teeth':100}, {'teeth':3},
    {'pressure_angle_deg':0.}, {'pressure_angle_deg':90.},
    {'face_width_mm':-1.}, {'pinion_bore_mm':-1.}, {'pinion_bore_mm':20.},
    {'tooth_thickness_reduction_mm':2.}, {'minimum_wall_mm':0.},
    {'rack_back_depth_mm':1.5}, {'rack_teeth':False}, {'flank_samples':1},
    {'rack_teeth':10**400},
])
def test_invalid_nominal_construction_is_rejected_before_cad(changes):
    with pytest.raises(ValueError):
        InvoluteRackPair(**changes)


@pytest.mark.parametrize('efficiency', [0., -1., 1.01, True, float('inf')])
def test_force_conversion_cannot_hide_an_invalid_efficiency(efficiency):
    with pytest.raises(ValueError):
        InvoluteRackPair().pinion_torque_for_rack_force(10., efficiency=efficiency)


def test_nonfinite_or_boolean_physical_inputs_are_rejected():
    pair = InvoluteRackPair()
    for method in (pair.pinion_torque_for_rack_force, pair.rack_force_for_pinion_torque,
                   pair.pinion_angle_for_rack_travel, pair.rack_travel_for_pinion_angle):
        for invalid in (float('inf'), float('nan'), True):
            with pytest.raises(ValueError):
                method(invalid)


def test_finite_inputs_cannot_produce_nonfinite_transmission_values():
    pair = InvoluteRackPair()
    with pytest.raises(ValueError):
        pair.rack_force_for_pinion_torque(1e308)
    with pytest.raises(ValueError):
        pair.pinion_torque_for_rack_force(1e308, efficiency=1e-308)
    with pytest.raises(ValueError):
        pair.pinion_angle_for_rack_travel(1e308)
    with pytest.raises(ValueError):
        pair.scaling_from(module_mm=1e308, teeth=100)
