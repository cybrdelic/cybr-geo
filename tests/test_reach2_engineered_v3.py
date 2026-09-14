from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
RECIPE = ROOT / "examples" / "reach2_engineered_elbow_v3.py"
ENGINEERING = ROOT / "tools" / "reach2_engineering_v3.py"


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def built():
    r = load(RECIPE, "reach2_v3_recipe_test")
    return r, r.build()


def test_v3_cad_builds_and_replaces_size20_drive(built):
    r, a = built
    names = {p.name for p in a.parts}
    assert len(a.parts) >= 185
    required = {
        "R3_10_CSG25_80_2UH_envelope",
        "R3_11_Housing_pilot_adapter",
        "R3_12_A_Fixed_housing_web",
        "R3_12_B_Fixed_housing_web",
        "R3_20_Motor_adapter_plate",
        "R3_22_ECMA_200W_servo_envelope",
        "R3_23_Flexible_input_coupler",
    }
    assert required <= names
    assert not any(n.startswith("R2_10") for n in names)
    assert not any(n.startswith("R2_20") for n in names)
    for p in a.parts:
        assert p.cad is not None
        assert p.cad.isValid(), p.name
        assert len(p.faces) > 0, p.name


def test_vendor_size25_geometry_and_performance_constants(built):
    r, _ = built
    assert r.RATIO == 80.0
    assert r.GEAR_OD == 107.0
    assert r.GEAR_LEN == 52.0
    assert r.HOUSING_BOLT_COUNT == 10
    assert r.HOUSING_BOLT_RADIUS == 48.0
    assert r.GEAR_ALLOWABLE_MOMENT_NM == 156.0
    assert r.GEAR_RATED_TORQUE_NM == 82.0
    assert r.ORBIT_PATTERN == r._v2.ORBIT_PATTERN
    assert r.LOWER_PATTERN == r._v2.LOWER_PATTERN


def test_output_and_orbit_remain_one_rigid_body(built):
    _, a = built
    lookup = {p.name: p for p in a.parts}
    orbit = lookup["O_B01_Mounting_shoe"]
    saddle = lookup["R2_01_ORBIT_saddle"]
    t = 1.7
    To = np.asarray(a.pose(orbit, t, 0.0), float)
    Ts = np.asarray(a.pose(saddle, t, 0.0), float)
    assert np.allclose(To, Ts, atol=1e-9)


def test_size25_reducer_is_fixed_and_coupler_tracks_ratio(built):
    r, a = built
    lookup = {p.name: p for p in a.parts}
    fixed = lookup["R3_10_CSG25_80_2UH_envelope"]
    coupler = lookup["R3_23_Flexible_input_coupler"]
    assert np.allclose(a.pose(fixed, 0.0, 0.0), np.eye(4), atol=1e-12)
    assert np.allclose(a.pose(fixed, 2.1, 0.0), np.eye(4), atol=1e-12)
    T = np.asarray(a.pose(coupler, 1.0, 0.0), float)
    expected = r.rotation_y(r.elbow_angle(1.0) * r.RATIO, r.PIVOT)
    assert np.allclose(T, expected, atol=1e-9)


def test_strict_engineering_qualification_passes():
    e = load(ENGINEERING, "reach2_v3_engineering_test")
    report = e.qualify()
    failed = [c["name"] for c in report["checks"] if not c["passed"]]
    assert report["qualified"], failed
    by = {c["name"]: c for c in report["checks"]}
    assert by["minimum continuous reducer margin"]["value"] >= 5.0
    assert by["minimum output-bearing moment margin"]["value"] >= 4.0
    assert by["minimum input-speed margin"]["value"] >= 3.0
    assert by["output bearing L10 revolutions"]["value"] >= 100_000_000
    assert by["servo RMS torque thermal load"]["passed"]
    assert by["carrier high-cycle alternating stress screen"]["passed"]
    assert by["Ø86 reducer pilot minimum diametral clearance"]["passed"]
    assert by["input coupling radial misalignment stack"]["passed"]
    assert by["full-speed hard-stop peak torque screen"]["passed"]
