"""Regression evidence for extracted ARX contact geometry, not hardware proof."""
from copy import deepcopy
import gzip
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from mechanism_lab.contact_geometry import ProxyBuilder, align_z, rotation_z, tabletop_tiles


ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / 'examples/engineering/arx_belt/data'
module_spec = importlib.util.spec_from_file_location('arx_belt_contacts', DATA.parent / 'contacts.py')
contacts = importlib.util.module_from_spec(module_spec)
module_spec.loader.exec_module(contacts)


@pytest.fixture(scope='module')
def inputs():
    return (json.loads((DATA / 'spec.json').read_text()),
            json.loads((DATA / 'folded-transmissions.json').read_text()))


@pytest.fixture(scope='module')
def built(inputs):
    return contacts.build(*inputs)


@pytest.fixture(scope='module')
def reference():
    path = Path(__file__).with_name('data') / 'arx_belt_contact_reference.json.gz'
    return json.loads(gzip.decompress(path.read_bytes()))


def _covered(points, rows):
    """Independently intersect points with the emitted oriented box records."""
    points = np.asarray(points, float)
    result = np.zeros(len(points), dtype=bool)
    for row in rows:
        assert row['type'] == 'box'
        w, x, y, z = row.get('quat_wxyz', [1, 0, 0, 0])
        rotation = Rotation.from_quat([x, y, z, w]).as_matrix()
        local = (points - np.asarray(row['center_mm'])) @ rotation
        result |= np.all(np.abs(local) <= np.asarray(row['half_size_mm']) + 1e-10, axis=1)
    return result


def test_all_source_records_are_retained_with_their_geometry(built, reference):
    assert reference['source_commit'] == 'aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2'
    assert len(built['records']) == len(reference['records']) == 2095
    for row, expected in zip(built['records'], reference['records'], strict=True):
        for field in ('name', 'body', 'type', 'native_part'):
            assert row[field] == expected[field], row['name']
        for field in ('center_mm', 'half_size_mm', 'fromto_mm', 'radius_mm'):
            if field in expected:
                np.testing.assert_allclose(row[field], expected[field], rtol=0, atol=1e-10,
                                           err_msg=f'{row["name"]}: {field}')
        if 'quat_wxyz' in expected:
            a, b = np.asarray(row['quat_wxyz']), np.asarray(expected['quat_wxyz'])
            # Antipodal quaternions are the same physical orientation.
            assert min(np.max(np.abs(a - b)), np.max(np.abs(a + b))) < 1e-12, row['name']
    for key in ('replace_names', 'replace_prefixes', 'joint_extra_armature_kg_m2', 'tabletop'):
        assert built[key] == reference['config'][key]


def test_every_source_table_tile_matches_serialized_mjcf(built, reference):
    rows = contacts.table_contacts(built)
    assert len(rows) == len(reference['tabletop_tiles']) == 315
    for row, expected in zip(rows, reference['tabletop_tiles'], strict=True):
        assert row['name'] == expected['name']
        assert row['type'] == expected['type'] == 'box'
        for field in ('center_mm', 'half_size_mm'):
            # Original MJCF stores metres formatted to 13 significant digits.
            np.testing.assert_allclose(row[field], expected[field], rtol=0, atol=1e-9)


def test_source_build_is_pure_and_frames_come_from_native_transmissions(inputs, built):
    spec, transmissions = deepcopy(inputs)
    before = deepcopy((spec, transmissions))
    repeated = contacts.build(spec, transmissions)
    assert (spec, transmissions) == before
    assert repeated == built
    offset = np.array([3., -2., 5.])
    transmissions[0]['origin_mm'] = (np.array(transmissions[0]['origin_mm']) + offset).tolist()
    moved = contacts.build(spec, transmissions)
    for original, row in zip(repeated['records'], moved['records'], strict=True):
        if row['name'].startswith('P5_J1_'):
            field = 'center_mm' if row['type'] == 'box' else 'fromto_mm'
            np.testing.assert_allclose(row[field], np.array(original[field]) + offset, rtol=0, atol=1e-10)
        else:
            assert row == original
    repeated['records'][0]['body'] = 'changed'
    assert built['records'][0]['body'] != 'changed'


def test_source_proxy_metadata_reports_approximation_limits(built):
    assert built['coverage_policy']['material_containment_claimed'] is False
    for row in built['records']:
        assert row['coverage']['material_containment_claimed'] is False
        assert row['coverage']['native_brep_equivalence'] == 'not_proven'
        assert row['coverage']['bounds_relative_to'] == 'declared_analytic_region'
        assert row['coverage']['native_material_overcoverage_bound_mm'] is None
    rings = [row for row in built['records'] if row['coverage'].get('geometry') == 'annulus']
    assert len(rings) == 1920
    assert all(row['coverage']['max_inward_radial_overcoverage_mm'] > 0 for row in rings)
    assert all(row['coverage']['max_outward_radial_overcoverage_mm'] > 0 for row in rings)


def test_annular_cover_has_no_material_gaps_and_retains_a_bounded_cavity():
    builder = ProxyBuilder()
    frame = align_z([1, 2, -3]) @ rotation_z(17)
    origin, xy = np.array([35, -12, 4]), np.array([7, -3])
    rows = builder.annulus('bearing', 'body', 10, 20, -2, 4, frame, origin, xy)
    theta = np.linspace(0, 2 * math.pi, 721)
    material = [[xy[0] + radius * math.cos(a), xy[1] + radius * math.sin(a), 1]
                for radius in (10, 15, 20) for a in theta]
    material = origin + np.asarray(material) @ frame.T
    assert _covered(material, rows).all()
    guaranteed = rows[0]['coverage']['guaranteed_clear_radius_mm']
    cavity = [[xy[0] + .999 * guaranteed * math.cos(a), xy[1] + .999 * guaranteed * math.sin(a), 1]
              for a in theta]
    cavity = origin + np.asarray(cavity) @ frame.T
    assert not _covered(cavity, rows).any()


def test_annular_proxy_deliberately_overcovers_both_curved_edges():
    rows = ProxyBuilder().annulus('ring', 'body', 10, 20, 0, 4)
    delta = math.pi / 24
    inner = 10 * (1 + math.cos(delta)) / 2
    outer = np.array([20., 20 * math.sin(delta), 2.])
    assert inner < 10 and _covered([[inner, 0, 2]], rows)[0]
    assert np.linalg.norm(outer[:2]) > 20 and _covered([outer], rows)[0]
    assert np.linalg.norm(outer[:2]) - 20 <= rows[0]['coverage']['max_outward_radial_overcoverage_mm']


def test_capsule_strips_retain_holes_and_cover_declared_material():
    holes = [dict(center_xy_mm=[0, 0], radius_mm=10), dict(center_xy_mm=[40, 0], radius_mm=5)]
    rows = ProxyBuilder().capsule_plate_strips('plate', 'fixed', 30, 40, 0, 6, holes)
    theta = np.linspace(0, 2 * math.pi, 121)
    openings = [[hole['center_xy_mm'][0] + .999 * (hole['radius_mm'] - 4) * math.cos(a),
                 .999 * (hole['radius_mm'] - 4) * math.sin(a), 3]
                for hole in holes for a in theta]
    assert not _covered(openings, rows).any()
    material = []
    for x in np.linspace(-30, 70, 91):
        for y in np.linspace(-30, 30, 59):
            distance = math.hypot(x - min(40, max(0, x)), y)
            in_hole = any(math.hypot(x - h['center_xy_mm'][0], y) < h['radius_mm'] for h in holes)
            if distance <= 30 and not in_hole:
                material.append([x, y, 3])
    assert _covered(material, rows).all()


def test_source_table_and_foot_keep_both_service_openings(built):
    foot = [row for row in built['records'] if row['native_part'] == 'bench_mount_plate']
    table = contacts.table_contacts(built)
    for x, radius in ((0, 55), (110, 23)):
        theta = np.linspace(0, 2 * math.pi, 181)
        foot_points = [[x + .999 * (radius - 4) * math.cos(a), 6,
                        .999 * (radius - 4) * math.sin(a)] for a in theta]
        table_points = [[x + .999 * (radius - 1) * math.cos(a), -20,
                         .999 * (radius - 1) * math.sin(a)] for a in theta]
        assert not _covered(foot_points, foot).any()
        assert not _covered(table_points, table).any()
    assert _covered([[200, -20, 200]], table)[0]
    assert _covered([[20, 6, 80]], foot)[0]


def test_table_configuration_is_not_modified_and_bad_layout_is_rejected(built):
    config = deepcopy(built['tabletop'])
    before = deepcopy(config)
    tabletop_tiles(config)
    assert config == before
    config['holes'][1]['center_xz_mm'] = [10, 0]
    with pytest.raises(ValueError, match='separated'):
        tabletop_tiles(config)


def test_edited_bench_hole_uses_the_same_body_xz_frame_in_foot_and_table(inputs):
    spec, transmissions = deepcopy(inputs)
    spec['bench_interface']['through_bores'][0]['center_xz_mm'] = [0, 20]
    config = contacts.build(spec, transmissions)
    foot = [row for row in config['records'] if row['native_part'] == 'bench_mount_plate']
    table = contacts.table_contacts(config)
    assert not _covered([[0, 6, 65.9]], foot)[0]
    assert not _covered([[0, -20, 65.9]], table)[0]
    assert _covered([[0, 6, -65.9]], foot)[0]
    assert _covered([[0, -20, -65.9]], table)[0]


@pytest.mark.parametrize('operation', [
    lambda b: b.box('bad', 'body', [0, 0, 0], [1, 0, 1]),
    lambda b: b.box('bad', 'body', [0, 0, 0], [1, 1, 1], np.diag([1, 1, -1])),
    lambda b: b.cylinder('bad', 'body', [0, 0, 0], [0, 0, 0], 2),
    lambda b: b.annulus('bad', 'body', 10, 9, 0, 1),
    lambda b: b.annulus('bad', 'body', 1, 2, 0, 1, sectors=2),
    lambda b: b.capsule_plate_strips('bad', 'body', 10, 20, 0, 1, [], strip_width_mm=0),
])
def test_invalid_proxy_geometry_fails_before_record_creation(operation):
    builder = ProxyBuilder()
    with pytest.raises(ValueError):
        operation(builder)
    assert builder.records == []


def test_axes_and_record_names_are_validated():
    with pytest.raises(ValueError, match='nonzero'):
        align_z([0, 0, 0])
    with pytest.raises(ValueError, match='finite'):
        rotation_z(float('nan'))
    builder = ProxyBuilder()
    builder.box('part', 'body', [0, 0, 0], [1, 1, 1])
    with pytest.raises(ValueError, match='Duplicate'):
        builder.box('part', 'body', [1, 0, 0], [1, 1, 1])
