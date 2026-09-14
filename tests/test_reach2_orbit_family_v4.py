from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RECIPE = ROOT / "examples" / "reach2_orbit_family_v4.py"
ENGINEERING = ROOT / "tools" / "reach2_engineering_v4.py"


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_v4_builds_and_replaces_generic_fixed_packaging():
    r = load(RECIPE, "reach2_v4_recipe_test")
    a = r.build()
    names = {p.name for p in a.parts}
    required = {
        "R4_12_A_Orbit_family_pedestal",
        "R4_12_B_Orbit_family_pedestal",
        "R4_14_L_Rounded_lower_gusset",
        "R4_14_R_Rounded_lower_gusset",
        "R4_20_Circular_motor_adapter",
        "R4_30_Output_service_ring",
        "R4_31_Housing_service_ring",
        "R4_32_Motor_rear_service_cap",
    }
    assert required <= names
    assert "R3_12_A_Fixed_housing_web" not in names
    assert "R3_12_B_Fixed_housing_web" not in names
    assert "R3_20_Motor_adapter_plate" not in names
    assert r.RATIO == 80.0
    assert r.GEAR_OD == 107.0
    assert r.ORBIT_PATTERN == r._v3.ORBIT_PATTERN
    assert r.LOWER_PATTERN == r._v3.LOWER_PATTERN
    for p in a.parts:
        assert p.cad is not None
        assert p.cad.isValid(), p.name
        assert len(p.faces) > 0, p.name


def test_v4_preserves_output_motion_and_counts_service_ring_as_moving():
    r = load(RECIPE, "reach2_v4_motion_test")
    a = r.build()
    lookup = {p.name: p for p in a.parts}
    ring = lookup["R4_30_Output_service_ring"]
    assert ring.group == "reach3_output"
    t = 1.3
    T = np.asarray(a.pose(ring, t, 0.0), float)
    expected = r.rotation_y(r.elbow_angle(t), r.PIVOT)
    assert np.allclose(T, expected, atol=1e-9)


def test_v4_orbit_family_release_constraints():
    r = load(RECIPE, "reach2_v4_constraints_test")
    assert r.MAX_FIXED_SILHOUETTE_MM <= 124.0
    assert r.WEB_THICKNESS_MM >= 12.0
    assert r.RING_RADIAL_SECTION_MM >= 7.0
    assert r.RIB_MIN_WIDTH_MM >= 24.0
    assert r.GUSSET_THICKNESS_MM >= 18.0
    assert r.MOTOR_ADAPTER_OD_MM <= 82.0


def test_v4_complete_engineering_gate_passes():
    e = load(ENGINEERING, "reach2_v4_engineering_test")
    report = e.qualify()
    failed = [c["name"] for c in report["checks"] if not c["passed"]]
    assert report["qualified"], failed
    assert report["mass_accounting"]["complete"] is True
    by = {c["name"]: c for c in report["checks"]}
    assert by["v3b continuous reducer margin"]["value"] >= 5.0
    assert by["v3b bearing moment margin"]["value"] >= 4.0
    assert by["v3b input speed margin"]["value"] >= 3.0
    assert by["v4 fixed housing silhouette"]["passed"]
    assert by["v4 load web thickness"]["passed"]
    assert by["v4 tapered rib minimum width"]["passed"]
