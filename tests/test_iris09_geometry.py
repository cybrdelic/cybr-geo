"""Direct IRIS source equations against actual CAD, without mechanical claims."""
import math

import numpy as np
import pytest
import cadquery as cq

from examples.engineering.iris09.design import (
    SOURCE, angles, leaf_dag, leaf_frame, leaf_tree, lens_geometry,
    rounded_radial_profile,
    _radial_face, _revolve_profile,
    _cut_radial_profiles,
)
from mechanism_lab.shape_dag import ShapeEvaluator, from_trees


def test_canted_leaf_matches_source_circle_and_plane_membership():
    scale = 50.
    shape = ShapeEvaluator(from_trees({'leaf': leaf_tree(scale)})).evaluate()
    row = SOURCE['leaf']
    random = np.random.default_rng(916)
    for u, v in random.uniform([-.6, -.6], [2., 1.6], (80, 2)):
        expected = all(math.hypot(u - arc[0], v - arc[1]) < arc[2]
                       for arc in [row['front_arc_center_xz_radius'], row['trailing_arc_center_xz_radius']])
        z = row['height'] + row['cant'] * v - row['vertical_thickness'] / 2
        assert shape.isInside((u * scale, -v * scale, z * scale), 1e-7) == expected
    u, v = .4, .3  # Strictly inside both source circular constraints.
    top = row['height'] + row['cant'] * v
    for z, expected in [(top - row['vertical_thickness'] / 2, True),
                        (top + .0001, False), (top - row['vertical_thickness'] - .0001, False)]:
        assert shape.isInside((u * scale, -v * scale, z * scale), 1e-7) == expected


def test_prescribed_frames_match_source_hinge_basis_and_twenty_second_period():
    scale, hinge, u, v = 50., .45, .4, .3
    for index in range(9):
        azimuth = index * 2 * math.pi / 9
        theta = azimuth + hinge
        source_x = .86 * math.cos(azimuth) + math.cos(theta) * u - math.sin(theta) * v
        source_z = .86 * math.sin(azimuth) + math.sin(theta) * u + math.cos(theta) * v
        actual = leaf_frame(index, hinge, scale) @ [u * scale, -v * scale, .26 * scale, 1]
        assert actual == pytest.approx([source_x * scale, -source_z * scale, .26 * scale, 1])
    assert angles(0) == pytest.approx(angles(20))
    assert angles(0)['hinge_radians'] == pytest.approx(.9)
    assert angles(10)['hinge_radians'] == pytest.approx(0)
    for t in [0, 5, 10]:
        pose = angles(t)
        assert pose['actuator_radians'] == pytest.approx(.7 - 1.2 * pose['hinge_radians'])


def test_world_clip_changes_displayed_material_but_rigid_template_is_invariant():
    volumes, invariant = [], []
    for t in (0, 5, 10):
        evaluator = ShapeEvaluator(leaf_dag(t))
        invariant.append(evaluator.evaluate('invariant_leaf').Volume())
        volumes.append(evaluator.evaluate('visual_leaf_local').Volume())
        assert evaluator.evaluate('visual_leaf_08').Volume() == pytest.approx(volumes[-1], rel=1e-8)
    assert invariant == pytest.approx([invariant[0]] * 3)
    assert min(volumes) > 0 and max(volumes) - min(volumes) > 30
    assert max(volumes) < invariant[0]


def test_annular_rounded_source_profile_and_closed_optical_cap_volumes():
    scale = 50.
    r, z, wr, hz, corner = 1.298, .255, .212, .255, .038
    shape = rounded_radial_profile(r, z, wr, hz, corner, scale)
    area = 4 * wr * hz - (4 - math.pi) * corner ** 2
    assert shape.Volume() == pytest.approx(2 * math.pi * r * area * scale ** 3, rel=1e-9)
    for (_, shape, _), (radius, base, edge) in zip(lens_geometry(scale), SOURCE['lenses']['sphere_radius_base_y_edge_radius']):
        height = radius - math.sqrt(radius ** 2 - edge ** 2)
        expected = math.pi * height ** 2 * (radius - height / 3) * scale ** 3
        assert shape.isValid() and len(shape.Solids()) == 1
        assert shape.Volume() == pytest.approx(expected, rel=1e-8)
        assert shape.BoundingBox().zmin == pytest.approx(base * scale)


@pytest.mark.parametrize('row', SOURCE['housing']['rounded_radial_profiles'], ids=lambda row: row[0])
def test_radial_plane_boolean_optimization_preserves_each_source_profile(row):
    values = row[1:6]
    original = rounded_radial_profile(*values, 50.)
    revolved = _revolve_profile(_radial_face(*values, 50.))
    assert revolved.isValid()
    assert revolved.Volume() == pytest.approx(original.Volume(), rel=1e-8)
    assert original.cut(revolved).Volume() < 1e-6
    assert revolved.cut(original).Volume() < 1e-6


def test_bounded_cut_batches_match_unpruned_native_boolean():
    face = _radial_face(4, 2, 1, 1, .1, 1)
    tools = [cq.Solid.makeCylinder(.24, 4, (4.9 * math.cos(a), 4.9 * math.sin(a), 0))
             for a in np.linspace(0, 2 * math.pi, 24, endpoint=False)]
    # Above, outside, and inside the radial hole must all be safely pruned.
    tools += [cq.Solid.makeCylinder(2, 2, (0, 0, 8)),
              cq.Solid.makeCylinder(.5, 4, (8, 8, 0)),
              cq.Solid.makeCylinder(1, 4, (0, 0, 0))]
    expected = _revolve_profile(face).cut(cq.Compound.makeCompound(tools))
    actual = _cut_radial_profiles(face, tools, 'test')
    assert actual.isValid()
    assert actual.Volume() == pytest.approx(expected.Volume(), rel=1e-9)
    assert expected.cut(actual).Volume() < 1e-7
    assert actual.cut(expected).Volume() < 1e-7


@pytest.mark.parametrize('scale', [0, -1, True, float('nan')])
def test_invalid_scene_unit_assumption_is_rejected(scale):
    with pytest.raises(ValueError, match='scene_unit_mm'):
        leaf_dag(scene_unit_mm=scale)


def test_actual_nine_leaf_build_keeps_body_and_part_names_distinct(tmp_path, monkeypatch):
    # Exercise the genuine NativeConstruction placement/export path without
    # rebuilding the expensive housing. CadQuery reserves each assembly name
    # inside that assembly's object registry, so identical part names fail.
    from examples.engineering.iris09 import design
    monkeypatch.setattr(design, 'housing_geometry', lambda scale: [])
    monkeypatch.setattr(design, 'lens_geometry', lambda scale: [])
    monkeypatch.setattr(design, 'handle_geometry', lambda scale: [])
    result = design.build(tmp_path)
    assert len(result.native.parts) == 10
    assert len(result.native.placed_records) == 9
    assert all(row['body'] != row['name'] for row in result.native.placed_records)
    assert len(result.reference.parts) == 9
    assert (tmp_path / 'native-build.json').is_file()
