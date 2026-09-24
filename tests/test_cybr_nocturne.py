"""Regression tests for the CYBR NOCTURNE procedural outfit."""
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
    assert report["part_count"] >= 55
    assert report["triangles"] >= 10_000
    assert report["unique_names"]
    assert report["finite_vertices"]
    assert report["valid_indices"]

    lo, hi = np.asarray(report["bounds_mm"], dtype=float)
    assert lo[2] >= -20
    assert hi[2] > 1800
    assert hi[0] - lo[0] > 1100
    assert hi[1] - lo[1] > 550


def test_nocturne_named_layers_and_materials():
    outfit = _build()
    names = {part.name for part in outfit.parts}
    groups = {part.group for part in outfit.parts}
    materials = {material.name for material in outfit.materials}

    expected_parts = {
        "coat_shell",
        "standing_collar",
        "vest_body",
        "diagonal_chest_harness",
        "waist_belt",
        "belt_buckle",
        "left_trouser_leg",
        "right_trouser_leg",
        "left_boot_upper",
        "right_boot_upper",
        "lapel_left_trim",
        "lapel_right_trim",
    }
    assert expected_parts <= names
    assert {"mannequin", "coat", "coat_trim", "underlayer", "trousers", "footwear", "hardware", "accent"} <= groups
    assert {
        "Obsidian shell textile",
        "Graphite technical weave",
        "Ink satin lining",
        "Blackened anisotropic metal",
        "Spectral indigo trim",
        "Carbon rubber",
    } <= materials

    shell = next(part for part in outfit.parts if part.name == "coat_shell")
    assert shell.metadata["generator"] == "finite_thickness_open_shell"
    assert shell.metadata["thickness_mm"] == 10
    assert 40 <= shell.metadata["front_gap_degrees"] <= 50


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


def test_nocturne_is_procedural_not_scan_backed():
    outfit = _build()
    assert outfit.metadata["authoring"] == "procedural geometry only"
    assert outfit.metadata["render_pair"] == "CYBR GEO geometry + CYBR LIGHT spectral path tracing"
    assert any("not based on a scanned person" in item.lower() for item in outfit.metadata["limitations"])
