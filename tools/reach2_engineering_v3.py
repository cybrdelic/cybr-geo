"""Stricter engineering gate for CYBR REACH-2 v3.

Extends the existing deterministic load qualification with explicit minimum
margin requirements, M8/M5 interface checks, bearing L10 estimate, servo RMS
thermal loading, reducer dissipation, structural/fastener fatigue screens, and
coaxial/tolerance-stack checks for the size-25 installation.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE_ENGINEERING = ROOT / "tools" / "reach2_engineering.py"
RECIPE = ROOT / "examples" / "reach2_engineered_elbow_v3.py"


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def qualify():
    base = _load(BASE_ENGINEERING, "reach2_engineering_v3_base")
    base.RECIPE = RECIPE
    r = _load(RECIPE, "reach2_v3_recipe_for_engineering")
    report = base.qualify()

    checks = list(report["checks"])
    by_name = {c["name"]: c for c in checks}
    cad = report["cad_derived"]
    drive = report["drive"]

    # Replace the conservative legacy M6-output labels with the actual v3 M8
    # adapter-circle calculations. Keep the legacy values out of the final gate
    # so the report cannot claim a fastener that is not present in the CAD.
    checks = [c for c in checks if not c["name"].startswith("output M6")]

    # Actual v3 output circle: 8x M8. Use class-10.9 proof/yield values despite
    # the CAD specifying 12.9; this intentionally under-credits the real bolts.
    m8_preload = base.bolt_preload_n(base.M8_STRESS_AREA_MM2)
    output_capacity = base.bolt_circle_torque_capacity_nm(
        r.OUTPUT_BOLT_COUNT, m8_preload, base.CLAMP_FRICTION, r.OUTPUT_BOLT_RADIUS
    )
    output_force = cad["repeated_design_torque_nm"] * 1000.0 / r.OUTPUT_BOLT_RADIUS
    output_shear = base.bolt_shear_stress_mpa(
        output_force, r.OUTPUT_BOLT_COUNT, base.M8_STRESS_AREA_MM2
    )
    checks.extend([
        base.check("output M8 friction torque capacity", output_capacity,
                   cad["repeated_design_torque_nm"] * base.STRUCTURAL_SAFETY,
                   relation=">=", units="Nm"),
        base.check("output M8 bolt shear stress", output_shear,
                   0.58 * base.BOLT_10P9_YIELD_MPA / base.STRUCTURAL_SAFETY,
                   units="MPa"),
    ])

    # Published size-25 motor-side housing interface: 10xM5 on Ø96. This is a
    # torque-transfer screen for the adapter flange; bearing moment is carried
    # by the locating pilot and fixed webs, not by bolt shanks alone.
    M5_AREA = 14.2
    m5_preload = base.bolt_preload_n(M5_AREA)
    housing_capacity = base.bolt_circle_torque_capacity_nm(
        r.HOUSING_BOLT_COUNT, m5_preload, base.CLAMP_FRICTION, r.HOUSING_BOLT_RADIUS
    )
    housing_force = cad["repeated_design_torque_nm"] * 1000.0 / r.HOUSING_BOLT_RADIUS
    housing_shear = base.bolt_shear_stress_mpa(
        housing_force, r.HOUSING_BOLT_COUNT, M5_AREA
    )
    checks.extend([
        base.check("housing 10xM5 friction torque capacity", housing_capacity,
                   cad["repeated_design_torque_nm"] * base.STRUCTURAL_SAFETY,
                   relation=">=", units="Nm"),
        base.check("housing M5 direct shear stress", housing_shear,
                   0.58 * base.BOLT_10P9_YIELD_MPA / base.STRUCTURAL_SAFETY,
                   units="MPa"),
    ])

    # Hard minimum margin policy. Passing a limit by a few percent is not enough
    # to release a prototype CAD package.
    continuous_margin = r.GEAR_RATED_TORQUE_NM / cad["continuous_design_torque_nm"]
    bearing_margin = r.GEAR_ALLOWABLE_MOMENT_NM / cad["bearing_design_moment_nm"]
    speed_limit = min(r.MOTOR_RATED_RPM, r.GEAR_MAX_AVG_INPUT_RPM)
    speed_margin = speed_limit / drive["input_speed_rpm_at_max_output_speed"]
    motor_margin = r.MOTOR_RATED_TORQUE_NM / drive["required_continuous_motor_torque_nm"]
    checks.extend([
        base.check("minimum continuous reducer margin", continuous_margin, 5.0,
                   relation=">=", units="x"),
        base.check("minimum output-bearing moment margin", bearing_margin, 4.0,
                   relation=">=", units="x"),
        base.check("minimum input-speed margin", speed_margin, 3.0,
                   relation=">=", units="x"),
        base.check("minimum motor continuous-torque margin", motor_margin, 2.5,
                   relation=">=", units="x"),
    ])

    # Cross-roller bearing L10 screening. Roller-bearing exponent 10/3. The
    # dynamic load in the base report already excludes the extra bearing safety
    # factor used for static moment qualification.
    dyn = by_name["output bearing dynamic radial load"]["value"]
    bearing_l10_revs = (r.GEAR_DYNAMIC_LOAD_N / max(dyn, 1e-9)) ** (10.0 / 3.0) * 1e6
    checks.append(base.check("output bearing L10 revolutions", bearing_l10_revs,
                             100_000_000.0, relation=">=", units="rev"))

    # Servo RMS torque thermal screen for a deliberately conservative mixed duty:
    # 55% holding, 35% moving at continuous demand, 10% repeated peak.
    cont = drive["required_continuous_motor_torque_nm"]
    peak = max(drive["required_peak_motor_torque_nm"], cont)
    servo_rms = math.sqrt(0.55 * cont**2 + 0.35 * cont**2 + 0.10 * peak**2)
    checks.append(base.check("servo RMS torque thermal load", servo_rms,
                             0.50 * r.MOTOR_RATED_TORQUE_NM, units="Nm",
                             detail="50% rated-torque acceptance limit for thermal headroom"))

    # Energy converted to heat in the reducer at the maximum continuous
    # mechanical operating point under the conservative 75% sizing efficiency.
    reducer_loss_w = drive["output_mechanical_power_w"] * (1.0 / base.GEAR_EFFICIENCY - 1.0)
    checks.append(base.check("reducer continuous loss budget", reducer_loss_w, 20.0,
                             units="W", detail="passive aluminium-housing design budget"))

    # High-cycle structural screens. 6061-T6 has no true endurance limit, so use
    # a deliberately low 55 MPa alternating-stress design screen for the in-house
    # plates. Legacy beam values are conservative for the thicker v3 fixed webs.
    carrier_stress = by_name["moving carrier plate bending stress"]["value"]
    fixed_stress = by_name["fixed housing web bending stress"]["value"]
    orbit_bolt_increment = by_name["ORBIT M6 bolt tension increment stress"]["value"]
    checks.extend([
        base.check("carrier high-cycle alternating stress screen", carrier_stress, 55.0,
                   units="MPa"),
        base.check("housing high-cycle alternating stress screen", fixed_stress, 55.0,
                   units="MPa"),
        base.check("ORBIT bolt fatigue increment screen", orbit_bolt_increment, 80.0,
                   units="MPa"),
    ])

    # Size-25 locating pilot tolerance stack. Vendor external pilot is Ø86 h7.
    # For the 80-120 mm ISO band conservatively use a 35 µm h7 width. Our mating
    # bore is Ø86.020 ±0.010, so the worst diametral clearance is 0.010..0.065 mm.
    gear_pilot_max = 86.000
    gear_pilot_min = 85.965
    bore_min = r.HOUSING_PILOT_BORE_NOMINAL_MM - r.HOUSING_PILOT_BORE_TOL_MM
    bore_max = r.HOUSING_PILOT_BORE_NOMINAL_MM + r.HOUSING_PILOT_BORE_TOL_MM
    pilot_min = bore_min - gear_pilot_max
    pilot_max = bore_max - gear_pilot_min
    checks.extend([
        base.check("Ø86 reducer pilot minimum diametral clearance", pilot_min, 0.008,
                   relation=">=", units="mm"),
        base.check("Ø86 reducer pilot maximum diametral clearance", pilot_max, 0.080,
                   units="mm"),
    ])

    # Coaxiality/misalignment budget to the flexible input coupling. These are
    # drawing requirements for the manufactured adapter stack, not guessed CAD
    # perfection. RSS radial error must stay below 0.20 mm and angular stack
    # below 0.5 degrees.
    housing_concentricity_mm = 0.030
    motor_pilot_position_mm = 0.050
    adapter_runout_mm = 0.030
    radial_rss = math.sqrt(housing_concentricity_mm**2 + motor_pilot_position_mm**2 + adapter_runout_mm**2)
    angular_stack_deg = 0.12
    checks.extend([
        base.check("input coupling radial misalignment stack", radial_rss, 0.20, units="mm"),
        base.check("input coupling angular misalignment stack", angular_stack_deg, 0.50, units="deg"),
    ])

    # Mechanical-stop energy screen at maximum command speed. Assume an 8 mm
    # effective elastomer compression at a 110 mm stop radius and require the
    # resulting average stop torque (with a 3x peak factor) to stay below reducer
    # momentary torque. This is intentionally pessimistic for a controller that
    # should never hit the stop at full command speed.
    omega = math.radians(report["load_case"]["max_output_speed_deg_s"])
    total_inertia = cad["moving_inertia_kg_m2"] + cad["payload_inertia_kg_m2"]
    kinetic_j = 0.5 * total_inertia * omega**2
    stop_angle = 0.008 / 0.110
    stop_peak_torque = 3.0 * kinetic_j / max(stop_angle, 1e-9)
    checks.append(base.check("full-speed hard-stop peak torque screen", stop_peak_torque,
                             r.GEAR_MOMENTARY_PEAK_NM, units="Nm"))

    report["checks"] = checks
    report["qualified"] = all(c["passed"] for c in checks)
    report["cad_revision"] = "examples/reach2_engineered_elbow_v3.py"
    report["engineering_revision"] = "REACH-2 v3 strict margin/fatigue/thermal/life gate"
    report["drive"].update({
        "reducer": "Harmonic Drive CSG-25-80-2UH",
        "ratio": r.RATIO,
        "bearing_l10_revolutions": bearing_l10_revs,
        "servo_rms_torque_nm": servo_rms,
        "reducer_continuous_loss_w": reducer_loss_w,
    })
    report["interfaces"].update({
        "output_fasteners": "8x M8 class 12.9 (qualified conservatively at 10.9 material values)",
        "reducer_housing_fasteners": "10x M5 class 12.9 on Ø96 BCD",
        "reducer_housing_pilot": "Ø86.020 ±0.010 mating bore against published Ø86 h7 pilot",
        "reducer_housing_pilot_clearance_mm": [pilot_min, pilot_max],
        "coaxiality_drawing_requirements_mm": {
            "housing_concentricity": housing_concentricity_mm,
            "motor_pilot_position": motor_pilot_position_mm,
            "adapter_runout": adapter_runout_mm,
        },
    })
    report["margin_policy"] = {
        "reducer_continuous_min_x": 5.0,
        "bearing_moment_min_x": 4.0,
        "input_speed_min_x": 3.0,
        "motor_continuous_min_x": 2.5,
    }
    report["limitations"] = [
        "Analytical qualification still requires prototype proof-load and cycle testing before human-adjacent use.",
        "Final manufacturing release must overlay the current vendor STEP/PDF revision and verify all reducer hole phases and pilots.",
        "Thermal screen is a conservative RMS/loss budget, not a measured winding or reducer temperature-rise test.",
        "Fatigue checks are conservative stress screens; production life requires a measured duty spectrum and physical endurance test.",
    ]
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
        raise SystemExit("REACH-2 v3 engineering gate failed: " + ", ".join(failed))


if __name__ == "__main__":
    main()
