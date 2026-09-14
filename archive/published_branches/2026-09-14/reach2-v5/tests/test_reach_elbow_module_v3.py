from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
RECIPE = ROOT / "examples" / "reach_elbow_module_v3.py"


def load_recipe():
    spec = importlib.util.spec_from_file_location("reach_v3_test_recipe", RECIPE)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def built():
    recipe = load_recipe()
    return recipe, recipe.build()


def part(a, name):
    return next(p for p in a.parts if p.name == name)


def test_v3_builds_complete_dual_supported_cad(built):
    _, a = built
    assert a.name == "CYBR REACH-1 v3 + ORBIT"
    assert len(a.parts) >= 220
    assert a.metadata["added_v3_parts"] >= 20
    assert len({p.name for p in a.parts}) == len(a.parts)
    for p in a.parts:
        assert p.cad is not None
        assert p.cad.isValid(), p.name
        assert len(p.faces) > 0, p.name
        assert np.isfinite(p.vertices).all(), p.name
        assert np.isfinite(p.normals).all(), p.name


def test_v3_has_dual_sided_gearbox_and_input_interface(built):
    _, a = built
    names = {p.name for p in a.parts}
    required = {
        "R40_Outboard_bearing_bridge",
        "R41_Intermediate_outboard_bearing",
        "R42_Input_outboard_bearing",
        "R45_Output_shaft_extension",
        "R46_Output_shaft_endcap",
        "R47_Motor_encoder_adapter",
        "R49_Input_coupling_sleeve",
        "R51_Lower_center_spine",
    }
    assert required <= names
    assert sum(n.startswith("R43_") for n in names) == 4
    assert sum(n.startswith("R44_") for n in names) == 4
    assert sum(n.startswith("R48_") for n in names) == 4
    assert sum(n.startswith("R50_") for n in names) == 2


def test_v3_rotating_completion_parts_follow_correct_groups(built):
    recipe, a = built
    v2 = recipe.load_v2()
    base = v2.load_base()
    t = base.PERIOD / 4.0

    output = part(a, "R45_Output_shaft_extension")
    expected_output = base.rotation_y(base.elbow_angle(t), base.PIVOT)
    assert np.allclose(a.pose(output, t, 0.0), expected_output, atol=1e-10)

    input_part = part(a, "R49_Input_coupling_sleeve")
    expected_input = base.rotation_y(base.elbow_angle(t) * base.REDUCTION, base.INPUT)
    assert np.allclose(a.pose(input_part, t, 0.0), expected_input, atol=1e-10)


def test_v3_fixed_supports_remain_grounded_to_reach_frame(built):
    _, a = built
    for name in (
        "R40_Outboard_bearing_bridge",
        "R43_0_Gearbox_standoff",
        "R47_Motor_encoder_adapter",
        "R51_Lower_center_spine",
    ):
        assert np.allclose(a.pose(part(a, name), 2.0, 0.0), np.eye(4), atol=1e-12)
