from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
RECIPE = ROOT / "examples" / "reach_elbow_module_v2.py"


def load_recipe():
    spec = importlib.util.spec_from_file_location("reach_v2_test_recipe", RECIPE)
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


def test_v2_builds_complete_analytic_cad(built):
    _, a = built
    assert a.name == "CYBR REACH-1 v2 + ORBIT"
    assert len(a.parts) >= 200
    assert a.metadata["added_integration_parts"] >= 25
    assert len({p.name for p in a.parts}) == len(a.parts)
    for p in a.parts:
        assert p.cad is not None
        assert p.cad.isValid(), p.name
        assert len(p.faces) > 0, p.name
        assert np.isfinite(p.vertices).all(), p.name
        assert np.isfinite(p.normals).all(), p.name


def test_v2_contains_finished_bearing_and_structure_hardware(built):
    _, a = built
    required = {
        "R21_L_Output_thrust_spacer",
        "R21_R_Output_thrust_spacer",
        "R22_L_Bearing_retainer",
        "R22_R_Bearing_retainer",
        "R24_Intermediate_stage_spacer",
        "R25_Intermediate_retaining_collar",
        "R26_Input_gear_spacer",
        "R27_Input_retaining_collar",
        "R31_L_Saddle_gusset",
        "R31_R_Saddle_gusset",
        "R32_Service_gland",
        "R33_L_Lower_flange_gusset",
        "R33_R_Lower_flange_gusset",
    }
    names = {p.name for p in a.parts}
    assert required <= names
    assert sum(n.startswith("R23_") for n in names) == 8
    assert sum(n.startswith("R30_") for n in names) == 4


def test_added_output_hardware_tracks_elbow_rigidly(built):
    base, a = built
    # v2 wraps the validated v1 motion function. A new reach_output part must
    # therefore receive the exact elbow transform at maximum flex.
    t = base.load_base().PERIOD / 4.0
    p = part(a, "R21_L_Output_thrust_spacer")
    T = a.pose(p, t, 0.0)
    b = base.load_base()
    expected = b.rotation_y(b.elbow_angle(t), b.PIVOT)
    assert np.allclose(T, expected, atol=1e-10)


def test_fixed_integration_hardware_stays_fixed(built):
    _, a = built
    t = 2.5
    for name in (
        "R22_L_Bearing_retainer",
        "R30_L_0_Crossbrace_socket_screw",
        "R33_R_Lower_flange_gusset",
    ):
        assert np.allclose(a.pose(part(a, name), t, 0.0), np.eye(4), atol=1e-12)
