"""Regression tests for the CYBR NOCTURNE v2 procedural outfit."""
from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def _build():
    path = ROOT / "examples" / "cybr_nocturne_outfit.py"
    spec = importlib.util.spec_from_file_location("cybr_nocturne_outfit_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.build()


def test_nocturne_geometry_contract():
    outfit = _build()
    report = outfit.validate()

    assert outfit.name == "CYBR NOCTURNE"
    assert report["part_count"] >= 140
    assert report["triangles"] >= 20_000
    assert report["unique_names"]
    assert report["finite_vertices"]
    assert report["valid_indices"]

    lo, hi = np.asarray(report["bounds_mm"], dtype=float)
    assert lo[2] >= -20
    assert hi[2] > 1840
    assert hi[0] - lo[0] > 850
    assert hi[1] - lo[1] > 550


def test_nocturne_named_construction_layers():
    outfit = _build()
    names = {part.name for part in outfit.parts}
    groups = {part.group for part in outfit.parts}
    materials = {material.name for material in outfit.materials}

    expected_parts = {
        "high_neck_underlayer",
        "underlayer_turtleneck",
        "coat_front_left_upper",
        "coat_front_right_upper",
        "coat_back_center",
        "coat_left_front_skirt",
        "coat_right_front_skirt",
        "coat_left_back_tail",
        "coat_right_back_tail",
        "rear_vent_lining",
        "left_lapel",
        "right_lapel",
        "collar_back",
        "left_collar_wing",
        "right_collar_wing",
        "front_diagonal_harness",
        "back_diagonal_harness",
        "front_waist_belt",
        "back_waist_belt",
        "back_spine_binding",
        "left_trouser_leg",
        "right_trouser_leg",
        "left_boot_upper",
        "right_boot_upper",
        "left_boot_sole",
        "right_boot_sole",
    }
    assert expected_parts <= names

    assert {
        "mannequin",
        "underlayer",
        "coat",
        "coat_back",
        "coat_skirt",
        "coat_trim",
        "lining",
        "sleeves",
        "gloves",
        "cuffs",
        "epaulettes",
        "harness",
        "back_detail",
        "seams",
        "trousers",
        "footwear",
        "hardware",
    } <= groups

    assert {
        "Nocturne black wool",
        "Graphite technical textile",
        "Deep amethyst satin",
        "Black calf leather",
        "Brushed gunmetal hardware",
        "Deep amethyst leather",
        "Carbon rubber",
        "Black seam binding",
    } <= materials


def test_coat_is_panel_based_not_inflated_shell():
    outfit = _build()

    panel_names = [
        "coat_front_left_upper",
        "coat_front_right_upper",
        "coat_back_center",
        "coat_back_left",
        "coat_back_right",
        "coat_left_front_skirt",
        "coat_right_front_skirt",
        "coat_left_back_tail",
        "coat_right_back_tail",
    ]
    for name in panel_names:
        part = next(p for p in outfit.parts if p.name == name)
        assert part.metadata["generator"] == "curved_panel"
        assert part.metadata["thickness_mm"] >= 6.0
        assert part.metadata["samples_across"] >= 8

    # Long coat tails must contain actual drape displacement, not flat slabs.
    for name in (
        "coat_left_front_skirt",
        "coat_right_front_skirt",
        "coat_left_back_tail",
        "coat_right_back_tail",
    ):
        part = next(p for p in outfit.parts if p.name == name)
        assert part.metadata["fold_amp_mm"] >= 12.0
        assert part.metadata["fold_cycles"] >= 2.0

    # The v1 single inflated "coat_shell" should no longer exist.
    assert "coat_shell" not in {p.name for p in outfit.parts}


def test_back_is_explicitly_designed():
    outfit = _build()
    names = {part.name for part in outfit.parts}

    assert "back_spine_binding" in names
    assert "back_diagonal_harness" in names
    assert "rear_vent_lining" in names
    assert len([n for n in names if n.startswith("back_diamond_")]) == 48
    assert len([n for n in names if "back_princess_seam" in n]) == 2


def test_footwear_is_directionally_constructed():
    outfit = _build()
    for side in ("left", "right"):
        upper = next(p for p in outfit.parts if p.name == f"{side}_boot_upper")
        sole = next(p for p in outfit.parts if p.name == f"{side}_boot_sole")
        assert upper.metadata["generator"] == "y_loft"
        assert sole.metadata["generator"] == "y_loft"
        assert len([p for p in outfit.parts if p.name.startswith(f"{side}_boot_strap_")]) == 3


def test_nocturne_meshes_are_finite_and_have_unit_normals():
    outfit = _build()
    for part in outfit.parts:
        assert np.isfinite(part.vertices).all(), part.name
        assert np.isfinite(part.normals).all(), part.name
        assert part.faces.min() >= 0, part.name
        assert part.faces.max() < len(part.vertices), part.name
        lengths = np.linalg.norm(part.normals, axis=1)
        assert np.all(lengths > 0.90), part.name
        assert np.all(lengths < 1.10), part.name


def test_nocturne_is_procedural_not_scan_or_generated_mesh_backed():
    outfit = _build()
    assert outfit.metadata["authoring"] == "procedural geometry only"
    assert outfit.metadata["render_pair"] == "CYBR GEO geometry + CYBR LIGHT spectral path tracing"
    construction = outfit.metadata["construction"]
    assert construction["panel_based_coat"]
    assert construction["separate_front_back_tail_panels"]
    assert construction["rear_centre_vent"]
    assert construction["procedural_back_lattice"]
    assert construction["directional_footwear_lofts"]
    limitations = " ".join(outfit.metadata["limitations"]).lower()
    assert "no body scan" in limitations
    assert "image-generated mesh" in limitations
