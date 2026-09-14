"""REACH-2 v4 qualification.

Runs the complete-moving-CAD v3b gate against the v4 recipe, then adds explicit
release checks for the ORBIT-family fixed packaging. The drivetrain, moving
carrier, reducer and interfaces remain v3b; only the fixed housing architecture
is changed.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "tools" / "reach2_engineering_v3b.py"
CORE = ROOT / "tools" / "reach2_engineering.py"
RECIPE = ROOT / "examples" / "reach2_orbit_family_v4.py"


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def qualify():
    base = load(BASE, "reach2_v4_v3b_base")
    core = load(CORE, "reach2_v4_core")
    r = load(RECIPE, "reach2_v4_recipe")

    # Critical: v3b recomputes mass/inertia from this v4 CAD, including the
    # teal output retainer because it deliberately remains in reach3_output.
    base.RECIPE = RECIPE
    report = base.qualify()
    a = r.build()
    lookup = {p.name: p for p in a.parts}

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
    missing = sorted(required - set(lookup))

    # Servo M5 head envelope around a 48 x 48 mm square bolt pattern.
    motor_bolt_radius = math.hypot(24.0, 24.0)
    motor_head_radius = 4.3
    motor_adapter_edge_margin = r.MOTOR_ADAPTER_OD_MM / 2.0 - motor_bolt_radius - motor_head_radius
    reducer_radial_clearance = r.RING_INNER_RADIUS_MM - r.GEAR_OD / 2.0

    extra = [
        core.check("v4 required ORBIT-family parts present", 0 if not missing else len(missing), 0, units="missing"),
        core.check("v4 fixed housing silhouette", r.MAX_FIXED_SILHOUETTE_MM, 124.0, units="mm"),
        core.check("v4 load web thickness", r.WEB_THICKNESS_MM, 12.0, relation=">=", units="mm"),
        core.check("v4 annular pedestal radial section", r.RING_RADIAL_SECTION_MM, 7.0, relation=">=", units="mm"),
        core.check("v4 tapered rib minimum width", r.RIB_MIN_WIDTH_MM, 24.0, relation=">=", units="mm"),
        core.check("v4 lower gusset thickness", r.GUSSET_THICKNESS_MM, 18.0, relation=">=", units="mm"),
        core.check("v4 reducer-to-pedestal radial clearance", reducer_radial_clearance, 1.0, relation=">=", units="mm"),
        core.check("v4 motor adapter M5 edge margin", motor_adapter_edge_margin, 2.5, relation=">=", units="mm"),
    ]

    # CAD validity is part of release qualification, not merely render setup.
    invalid = [p.name for p in a.parts if p.cad is None or not p.cad.isValid() or len(p.faces) == 0]
    extra.append(core.check("v4 invalid CAD parts", len(invalid), 0, units="parts"))

    report["checks"].extend(extra)
    report["qualified"] = all(c["passed"] for c in report["checks"])
    report["engineering_revision"] = "REACH-2 v4 ORBIT-family + complete-moving-CAD v3b gate"
    report["aesthetic_packaging"] = {
        "family": "CYBR ORBIT",
        "missing_required_parts": missing,
        "invalid_parts": invalid,
        "fixed_silhouette_mm": r.MAX_FIXED_SILHOUETTE_MM,
        "web_thickness_mm": r.WEB_THICKNESS_MM,
        "ring_radial_section_mm": r.RING_RADIAL_SECTION_MM,
        "rib_min_width_mm": r.RIB_MIN_WIDTH_MM,
        "gusset_thickness_mm": r.GUSSET_THICKNESS_MM,
        "reducer_radial_clearance_mm": reducer_radial_clearance,
        "motor_adapter_edge_margin_mm": motor_adapter_edge_margin,
    }
    return report


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path)
    ap.add_argument("--require-pass", action="store_true")
    args = ap.parse_args()
    report = qualify()
    text = json.dumps(report, indent=2)
    print(text)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n")
    if args.require_pass and not report["qualified"]:
        failed = [c["name"] for c in report["checks"] if not c["passed"]]
        raise SystemExit("REACH-2 v4 engineering gate failed: " + ", ".join(failed))


if __name__ == "__main__":
    main()
