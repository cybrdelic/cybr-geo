from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
RECIPE = ROOT / "examples" / "reach2_engineered_elbow_v2.py"
ENGINEERING = ROOT / "tools" / "reach2_engineering_v2.py"


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def built():
    r = load(RECIPE, "reach2_v2_recipe_test")
    return r, r.build()


def test_reach2_v2_cad_is_valid_and_complete(built):
    r, a = built
    assert a.name == "CYBR REACH-2 v2 + ORBIT"
    assert len(a.parts) >= 180
    assert len({p.name for p in a.parts}) == len(a.parts)
    assert a.metadata["selected_reducer"] == "Harmonic Drive CSG-20-120-2UH"
    assert a.metadata["reducer_ratio"] == 120.0
    assert "fillet_policy" in a.metadata
    for p in a.parts:
        assert p.cad is not None, p.name
        assert p.cad.isValid(), p.name
        assert len(p.faces) > 0, p.name
        assert np.isfinite(p.vertices).all(), p.name
        assert np.isfinite(p.normals).all(), p.name


def test_orbit_interface_is_preserved(built):
    r, a = built
    assert a.metadata["orbit_interface_pattern_mm"] == [list(p) for p in r.ORBIT_PATTERN]
    assert not any(p.name.startswith("O_B02_") for p in a.parts)
    for i in range(4):
        assert any(p.name == f"R2_02_{i}_ORBIT_M6_10p9" for p in a.parts)


def test_output_and_orbit_move_rigidly_together(built):
    r, a = built
    t = r.PERIOD / 4.0
    orbit_shoe = next(p for p in a.parts if p.name == "O_B01_Mounting_shoe")
    saddle = next(p for p in a.parts if p.name == "R2_01_ORBIT_saddle")
    face = next(p for p in a.parts if p.name == "R2_05_Output_face_plate")
    T0 = a.pose(orbit_shoe, t, 0.0)
    assert np.allclose(T0, a.pose(saddle, t, 0.0), atol=1e-10)
    assert np.allclose(T0, a.pose(face, t, 0.0), atol=1e-10)


def test_fixed_reducer_and_motor_remain_fixed(built):
    _, a = built
    t = 2.7
    for name in ("R2_10_CSG20_120_2UH_envelope", "R2_20_Motor_adapter_plate",
                 "R2_22_ECMA_200W_servo_envelope", "R2_30_Lower_modular_flange"):
        p = next(p for p in a.parts if p.name == name)
        assert np.allclose(a.pose(p, t, 0.0), np.eye(4), atol=1e-12), name


def test_engineering_qualification_passes():
    e = load(ENGINEERING, "reach2_v2_engineering_test")
    report = e.qualify()
    failures = [c for c in report["checks"] if not c["passed"]]
    assert report["qualified"], failures
    assert report["load_case"]["payload_kg"] == 2.0
    assert report["load_case"]["shock_g"] == 3.0
    assert report["drive"]["ratio"] == 120.0
