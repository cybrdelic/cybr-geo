"""REACH-2 v3b qualification: strict v3 checks with complete CAD mass accounting.

The v3 recipe introduced reach3_output parts. The legacy REACH-2 mass collector
only recognized reach2_output, so this layer deliberately rebuilds the v3 CAD
and recomputes every load quantity from all O_*, reach2_output and reach3_output
parts before applying corrected torque/bearing/motor/margin/hard-stop gates.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
STRICT_V3 = ROOT / "tools" / "reach2_engineering_v3.py"
CORE = ROOT / "tools" / "reach2_engineering.py"
RECIPE = ROOT / "examples" / "reach2_engineered_elbow_v3.py"


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def qualify():
    strict = load(STRICT_V3, "reach2_v3_strict_base")
    core = load(CORE, "reach2_v3_core")
    r = load(RECIPE, "reach2_v3b_recipe")
    report = strict.qualify()
    a = r.build()
    pivot = np.asarray(r.PIVOT, float)

    moving = [
        p for p in a.parts
        if p.name.startswith("O_") or p.group in ("reach2_output", "reach3_output")
    ]
    moving_mass = sum(core.part_mass_kg(p) for p in moving)
    moving_gravity = sum(
        core.part_mass_kg(p) * core.GRAVITY * core.center_radius_m(p, pivot)
        for p in moving
    )
    moving_inertia = sum(
        core.part_mass_kg(p) * core.center_radius_m(p, pivot) ** 2
        for p in moving
    )
    max_radius = max(core.bounds_radial_radius_m(p, pivot) for p in moving)
    payload_radius = max_radius + core.TOOL_EXTENSION_M
    payload_gravity = core.PAYLOAD_KG * core.GRAVITY * payload_radius
    payload_inertia = core.PAYLOAD_KG * payload_radius ** 2
    static_torque = moving_gravity + payload_gravity
    total_inertia = moving_inertia + payload_inertia
    accel_torque = total_inertia * core.ANGULAR_ACCEL_RAD_S2
    continuous = static_torque * core.CONTINUOUS_SAFETY
    repeated = (static_torque + accel_torque) * core.PEAK_SAFETY
    momentary = static_torque * core.SHOCK_G * core.PEAK_SAFETY

    moving_radial_moment = sum(
        core.part_mass_kg(p) * core.GRAVITY * core.SHOCK_G * core.center_radius_m(p, pivot)
        for p in moving
    )
    payload_radial_moment = core.PAYLOAD_KG * core.GRAVITY * core.SHOCK_G * payload_radius
    bearing_moment = (moving_radial_moment + payload_radial_moment) * core.BEARING_MOMENT_SAFETY
    radial_force = (moving_mass + core.PAYLOAD_KG) * core.GRAVITY * core.SHOCK_G * core.BEARING_MOMENT_SAFETY

    input_rpm = report["load_case"]["max_output_speed_deg_s"] / 360.0 * 60.0 * r.RATIO
    required_cont_motor = continuous / (r.RATIO * core.GEAR_EFFICIENCY)
    required_peak_motor = repeated / (r.RATIO * core.GEAR_EFFICIENCY)
    max_speed_rad_s = math.radians(report["load_case"]["max_output_speed_deg_s"])
    output_power = continuous * max_speed_rad_s
    torsional_deflection = math.degrees(continuous / r.GEAR_TORSIONAL_K_NM_PER_RAD)

    # Correct report values first so downstream consumers cannot accidentally use
    # the optimistic legacy subset.
    report["cad_derived"].update({
        "moving_parts": len(moving),
        "moving_mass_kg": moving_mass,
        "max_geometry_radius_m": max_radius,
        "payload_radius_m": payload_radius,
        "moving_gravity_torque_nm": moving_gravity,
        "payload_gravity_torque_nm": payload_gravity,
        "static_torque_nm": static_torque,
        "moving_inertia_kg_m2": moving_inertia,
        "payload_inertia_kg_m2": payload_inertia,
        "acceleration_torque_nm": accel_torque,
        "continuous_design_torque_nm": continuous,
        "repeated_design_torque_nm": repeated,
        "momentary_3g_design_torque_nm": momentary,
        "bearing_design_moment_nm": bearing_moment,
        "bearing_design_radial_load_n": radial_force,
    })
    report["drive"].update({
        "required_continuous_motor_torque_nm": required_cont_motor,
        "required_peak_motor_torque_nm": required_peak_motor,
        "input_speed_rpm_at_max_output_speed": input_rpm,
        "output_mechanical_power_w": output_power,
        "torsional_deflection_deg_at_continuous_design": torsional_deflection,
    })

    # Corrected gates. They coexist with the original checks, but these are the
    # authoritative v3b checks and use the complete moving set.
    corrected = [
        core.check("v3b continuous reducer torque", continuous, r.GEAR_RATED_TORQUE_NM, units="Nm"),
        core.check("v3b repeated peak reducer torque", repeated, r.GEAR_REPEATED_PEAK_NM, units="Nm"),
        core.check("v3b 3g momentary reducer torque", momentary, r.GEAR_MOMENTARY_PEAK_NM, units="Nm"),
        core.check("v3b output bearing moment", bearing_moment, r.GEAR_ALLOWABLE_MOMENT_NM, units="Nm"),
        core.check("v3b output bearing static radial load", radial_force, r.GEAR_STATIC_LOAD_N, units="N"),
        core.check("v3b motor continuous torque", required_cont_motor, r.MOTOR_RATED_TORQUE_NM, units="Nm"),
        core.check("v3b motor peak torque", required_peak_motor, r.MOTOR_MAX_TORQUE_NM, units="Nm"),
        core.check("v3b motor input speed", input_rpm, min(r.MOTOR_RATED_RPM, r.GEAR_MAX_AVG_INPUT_RPM), units="rpm"),
        core.check("v3b torsional elastic deflection", torsional_deflection, 0.15, units="deg"),
        core.check("v3b continuous reducer margin", r.GEAR_RATED_TORQUE_NM / continuous, 5.0, relation=">=", units="x"),
        core.check("v3b bearing moment margin", r.GEAR_ALLOWABLE_MOMENT_NM / bearing_moment, 4.0, relation=">=", units="x"),
        core.check("v3b input speed margin", min(r.MOTOR_RATED_RPM, r.GEAR_MAX_AVG_INPUT_RPM) / input_rpm, 3.0, relation=">=", units="x"),
        core.check("v3b motor torque margin", r.MOTOR_RATED_TORQUE_NM / required_cont_motor, 2.5, relation=">=", units="x"),
    ]

    # Bearing L10 from corrected dynamic load (roller exponent 10/3).
    dynamic_load = radial_force / core.BEARING_MOMENT_SAFETY
    l10 = (r.GEAR_DYNAMIC_LOAD_N / max(dynamic_load, 1e-9)) ** (10.0 / 3.0) * 1e6
    corrected.append(core.check("v3b output bearing L10 revolutions", l10, 100_000_000.0,
                                relation=">=", units="rev"))

    # Corrected RMS thermal screen and reducer dissipation.
    peak_motor = max(required_peak_motor, required_cont_motor)
    servo_rms = math.sqrt(0.90 * required_cont_motor**2 + 0.10 * peak_motor**2)
    loss_w = output_power * (1.0 / core.GEAR_EFFICIENCY - 1.0)
    corrected.extend([
        core.check("v3b servo RMS torque thermal load", servo_rms,
                   0.50 * r.MOTOR_RATED_TORQUE_NM, units="Nm"),
        core.check("v3b reducer continuous loss budget", loss_w, 20.0, units="W"),
    ])

    # Corrected hard-stop energy uses complete CAD inertia.
    kinetic_j = 0.5 * total_inertia * max_speed_rad_s**2
    stop_angle = 0.008 / 0.110
    stop_peak = 3.0 * kinetic_j / max(stop_angle, 1e-9)
    corrected.append(core.check("v3b full-speed hard-stop peak torque", stop_peak,
                                r.GEAR_MOMENTARY_PEAK_NM, units="Nm"))

    report["checks"].extend(corrected)
    report["qualified"] = all(c["passed"] for c in report["checks"])
    report["engineering_revision"] = "REACH-2 v3b complete-moving-CAD strict gate"
    report["drive"]["bearing_l10_revolutions"] = l10
    report["drive"]["servo_rms_torque_nm"] = servo_rms
    report["drive"]["reducer_continuous_loss_w"] = loss_w
    report["mass_accounting"] = {
        "moving_group_rule": "O_* + reach2_output + reach3_output",
        "moving_parts": len(moving),
        "complete": True,
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
        raise SystemExit("REACH-2 v3b engineering gate failed: " + ", ".join(failed))


if __name__ == "__main__":
    main()
