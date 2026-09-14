from __future__ import annotations

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RECIPE = ROOT / "examples" / "reach2_orbit_integrated_v5b.py"
ENGINEERING = ROOT / "tools" / "reach2_engineering_v5.py"


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_v5_builds_without_legacy_square_drive_package():
    r = load(RECIPE, "reach2_v5_recipe_test")
    a = r.build()
    names = {p.name for p in a.parts}
    required = {
        "R5_10_FHA25C_H_100_integrated_actuator",
        "R5_05_Output_adapter",
        "R5_12_ORBIT_family_actuator_pedestal",
        "R5_30_Output_service_retainer",
        "R5_31_Hollow_shaft_cable_gland",
        "R5_32_Rear_connector_guard",
    }
    assert required <= names
    assert not any(n.startswith(("R2_20", "R2_22", "R3_20", "R3_22")) for n in names)
    assert r.RATIO == 100.0
    assert r.ACTUATOR_HOLLOW_BORE_MM == 42.0
    assert r.ORBIT_PATTERN == r._v5._base.ORBIT_PATTERN
    assert r.LOWER_PATTERN == r._v5._base.LOWER_PATTERN
    for p in a.parts:
        assert p.cad is not None
        assert p.cad.isValid(), p.name
        assert len(p.faces) > 0, p.name


def test_v5_release_pilot_fits_are_controlled():
    r = load(RECIPE, "reach2_v5_fit_test")
    assert abs(r.OUTPUT_PILOT_BORE_MM - 125.020) < 1e-9
    assert abs(r.PEDESTAL_PILOT_BORE_MM - 128.020) < 1e-9


def test_v5_complete_engineering_gate_passes():
    e = load(ENGINEERING, "reach2_v5_engineering_test")
    report = e.qualify()
    failed = [c["name"] for c in report["checks"] if not c["passed"]]
    assert report["qualified"], failed
    assert report["mass_accounting"]["complete"] is True
    by = {c["name"]: c for c in report["checks"]}
    assert by["v5 continuous torque margin"]["value"] >= 5.0
    assert by["v5 maximum torque margin"]["value"] >= 5.0
    assert by["v5 bearing moment margin"]["value"] >= 4.0
    assert by["v5 speed margin"]["value"] >= 3.0
    assert by["v5 actuator-axis package depth"]["passed"]
    assert by["v5 legacy square motor package parts"]["passed"]
