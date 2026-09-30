from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "examples" / "del_playa_6503" / "recipe.py"


def load_recipe():
    spec = importlib.util.spec_from_file_location("del_playa_6503_recipe_test", SOURCE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_del_playa_reference_assembly_is_finite_and_procedural():
    module = load_recipe()
    assembly = module.build()
    assert assembly.metadata["no_image_generation"] is True
    assert assembly.metadata["no_photogrammetry"] is True
    assert assembly.metadata["no_scan_geometry"] is True
    assert assembly.metadata["no_scan_textures"] is True
    assert len(assembly.parts) > 250
    assert sum(len(part.faces) for part in assembly.parts) > 8000
    names = {part.name for part in assembly.parts}
    assert "upper_main_shell" in names
    assert "front_rail_toprail" in names
    assert "bluff_terrain" in names
    assert any(name.startswith("front_rail_baluster_") for name in names)
    assert any(name.startswith("front_deck_board_") for name in names)
    for part in assembly.parts:
        assert np.isfinite(part.vertices).all(), part.name
        assert np.isfinite(part.normals).all(), part.name
        assert (part.faces >= 0).all(), part.name
        assert (part.faces < len(part.vertices)).all(), part.name


def test_del_playa_bounds_cover_house_bluff_and_ocean():
    assembly = load_recipe().build()
    lo, hi = assembly.bounds
    assert lo[1] < -70_000
    assert hi[1] > 10_000
    assert lo[2] < -8_000
    assert hi[2] > 8_000
    assert hi[0] - lo[0] > 70_000
