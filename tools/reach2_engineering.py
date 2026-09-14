"""Deterministic engineering checks for CYBR REACH-2.

This is a conservative analytical qualification layer for the concept CAD. It
checks actual CAD-derived moving mass and radius, gravity/inertial torque, motor
and reducer margins, reducer output-bearing load/moment, output/housing/interface
fasteners, shaft torsion, aluminium section stresses, deflection, speed/power,
and worst-case assembly tolerances.

It is not a substitute for vendor confirmation drawings, FEA correlation,
fatigue testing, prototype proof-load testing, thermal testing, or certification.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RECIPE = ROOT / "examples" / "reach2_engineered_elbow.py"

# Qualification envelope. These values intentionally exceed a tabletop demo.
PAYLOAD_KG = 2.0
TOOL_EXTENSION_M = 0.050
ANGULAR_ACCEL_RAD_S2 = 4.0
MAX_OUTPUT_SPEED_DEG_S = 60.0
GRAVITY = 9.80665
SHOCK_G = 3.0
CONTINUOUS_SAFETY = 1.50
PEAK_SAFETY = 1.25
BEARING_MOMENT_SAFETY = 1.50
STRUCTURAL_SAFETY = 2.00
GEAR_EFFICIENCY = 0.75  # conservative for sizing

# Density by ORBIT material index, kg/mm^3.
DENSITY = {
    0: 2.70e-6,   # anodized aluminium
    1: 2.70e-6,   # machined aluminium
    2: 7.85e-6,   # steel
    3: 8.80e-6,   # bronze
    4: 1.20e-6,   # elastomer
    5: 2.70e-6,   # teal anodized aluminium
    6: 2.50e-6,   # ceramic marking
    7: 8.96e-6,   # copper
}

# Fastener material/geometry approximations, conservative and explicit.
M6_STRESS_AREA_MM2 = 20.1
M8_STRESS_AREA_MM2 = 36.6
BOLT_10P9_PROOF_MPA = 830.0
BOLT_10P9_YIELD_MPA = 940.0
BOLT_PRELOAD_FACTOR = 0.70
CLAMP_FRICTION = 0.15

# Structural material values for 6061-T6 at room temperature.
AL6061_YIELD_MPA = 276.0
AL6061_E_MPA = 68900.0
STEEL_YIELD_MPA = 620.0  # intentionally below typical hardened steel used here


def load_recipe():
    spec = importlib.util.spec_from_file_location("reach2_recipe", RECIPE)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def part_mass_kg(part):
    # Commercial unit/motor envelopes are not solid-density mass models.
    if part.name == "R2_10_CSG20_120_2UH_envelope":
        return 0.98
    if part.name == "R2_22_ECMA_200W_servo_envelope":
        return 1.25
    return float(part.cad.Volume()) * DENSITY.get(part.material, 2.70e-6)


def bounds_radial_radius_m(part, pivot):
    bb = part.cad.BoundingBox()
    xs = (bb.xmin, bb.xmax)
    zs = (bb.zmin, bb.zmax)
    r = 0.0
    for x in xs:
        for z in zs:
            r = max(r, math.hypot(x - pivot[0], z - pivot[2]))
    return r / 1000.0


def center_radius_m(part, pivot):
    c = np.array(part.cad.BoundingBox().center.toTuple(), float)
    return math.hypot(c[0] - pivot[0], c[2] - pivot[2]) / 1000.0


def section_bending_stress_mpa(moment_nm, plate_thickness_mm, section_depth_mm, count=1):
    # In-plane bending of rectangular plate neck: Z=t*h^2/6.
    Z = plate_thickness_mm * section_depth_mm**2 / 6.0
    return (moment_nm * 1000.0 / count) / Z


def cantilever_tip_deflection_mm(force_n, length_mm, thickness_mm, depth_mm, count=1):
    # Conservative rectangular-beam idealization, each plate shares load equally.
    I = thickness_mm * depth_mm**3 / 12.0
    return (force_n / count) * length_mm**3 / (3.0 * AL6061_E_MPA * I)


def torsional_shear_mpa(torque_nm, ro_mm, ri_mm=0.0):
    J = math.pi / 2.0 * (ro_mm**4 - ri_mm**4)
    return torque_nm * 1000.0 * ro_mm / J


def bolt_preload_n(area_mm2):
    return BOLT_PRELOAD_FACTOR * BOLT_10P9_PROOF_MPA * area_mm2


def bolt_shear_stress_mpa(total_force_n, count, area_mm2):
    return total_force_n / count / area_mm2


def bolt_circle_torque_capacity_nm(count, preload_n, mu, radius_mm):
    return count * preload_n * mu * radius_mm / 1000.0


def bolt_group_axial_increment_n(moment_nm, coords_mm, axis):
    # Elastic bolt-group estimate for overturning moment about x or y.
    vals = np.array([p[1] if axis == "x" else p[0] for p in coords_mm], float)
    vals -= vals.mean()
    denom = float(np.sum(vals**2))
    if denom <= 0:
        return float("inf")
    return abs(moment_nm * 1000.0) * float(np.max(np.abs(vals))) / denom


def tolerance_margin(hole_min_mm, screw_max_mm, pos_tol_a_mm, pos_tol_b_mm):
    radial_clearance = (hole_min_mm - screw_max_mm) / 2.0
    # Worst-case independent +/- X/Y positioning on both mating patterns.
    relative = math.sqrt(2.0) * (pos_tol_a_mm + pos_tol_b_mm)
    return radial_clearance - relative


def check(name, value, limit, relation="<=", units="", detail=""):
    if relation == "<=":
        passed = value <= limit
        margin = limit / value if value > 0 else float("inf")
    elif relation == ">=":
        passed = value >= limit
        margin = value / limit if limit > 0 else float("inf")
    else:
        raise ValueError(relation)
    return {
        "name": name,
        "passed": bool(passed),
        "value": float(value),
        "limit": float(limit),
        "relation": relation,
        "units": units,
        "factor_margin": float(margin),
        "detail": detail,
    }


def qualify():
    r = load_recipe()
    a = r.build()
    pivot = np.asarray(r.PIVOT, float)

    moving = [p for p in a.parts if p.name.startswith("O_") or p.group == "reach2_output"]
    moving_mass = sum(part_mass_kg(p) for p in moving)
    moving_gravity_torque = sum(part_mass_kg(p) * GRAVITY * center_radius_m(p, pivot) for p in moving)
    moving_inertia = sum(part_mass_kg(p) * center_radius_m(p, pivot)**2 for p in moving)
    max_geometry_radius = max(bounds_radial_radius_m(p, pivot) for p in moving)
    payload_radius = max_geometry_radius + TOOL_EXTENSION_M
    payload_gravity_torque = PAYLOAD_KG * GRAVITY * payload_radius
    payload_inertia = PAYLOAD_KG * payload_radius**2

    static_torque = moving_gravity_torque + payload_gravity_torque
    total_inertia = moving_inertia + payload_inertia
    accel_torque = total_inertia * ANGULAR_ACCEL_RAD_S2
    continuous_design = static_torque * CONTINUOUS_SAFETY
    repeated_design = (static_torque + accel_torque) * PEAK_SAFETY
    momentary_design = static_torque * SHOCK_G * PEAK_SAFETY

    # Worst external bearing moment uses 3g radial loading at full radius.
    moving_radial_moment = sum(part_mass_kg(p) * GRAVITY * SHOCK_G * center_radius_m(p, pivot) for p in moving)
    payload_radial_moment = PAYLOAD_KG * GRAVITY * SHOCK_G * payload_radius
    bearing_moment = (moving_radial_moment + payload_radial_moment) * BEARING_MOMENT_SAFETY
    radial_force = (moving_mass + PAYLOAD_KG) * GRAVITY * SHOCK_G * BEARING_MOMENT_SAFETY

    max_speed_rad_s = math.radians(MAX_OUTPUT_SPEED_DEG_S)
    input_rpm = MAX_OUTPUT_SPEED_DEG_S / 360.0 * 60.0 * r.RATIO
    required_cont_motor = continuous_design / (r.RATIO * GEAR_EFFICIENCY)
    required_peak_motor = repeated_design / (r.RATIO * GEAR_EFFICIENCY)
    output_mech_power = continuous_design * max_speed_rad_s

    # Torsional stiffness: use vendor K3 above T2 for a conservative single slope.
    torsional_deflection_deg = math.degrees(continuous_design / r.GEAR_TORSIONAL_K_NM_PER_RAD)

    # Output adapter/hub and carrier plate structural idealizations.
    output_hub_shear = torsional_shear_mpa(repeated_design, 15.0, 7.0)
    carrier_stress = section_bending_stress_mpa(repeated_design, 9.0, 34.0, count=2)
    fixed_web_stress = section_bending_stress_mpa(repeated_design, 10.0, 38.0, count=2)
    gravity_force = (moving_mass + PAYLOAD_KG) * GRAVITY * SHOCK_G
    carrier_deflection = cantilever_tip_deflection_mm(gravity_force, 72.0, 9.0, 34.0, count=2)
    fixed_deflection = cantilever_tip_deflection_mm(gravity_force, 82.0, 10.0, 38.0, count=2)

    # Output bolt circle: 8x M6 class 10.9.
    m6_preload = bolt_preload_n(M6_STRESS_AREA_MM2)
    output_clamp_torque = bolt_circle_torque_capacity_nm(r.OUTPUT_BOLT_COUNT, m6_preload,
                                                         CLAMP_FRICTION, r.OUTPUT_BOLT_RADIUS)
    output_tangential_force = repeated_design * 1000.0 / r.OUTPUT_BOLT_RADIUS
    output_bolt_shear = bolt_shear_stress_mpa(output_tangential_force, r.OUTPUT_BOLT_COUNT,
                                               M6_STRESS_AREA_MM2)

    # ORBIT saddle 4x M6. Use shock load and worst bearing moment as bolt-group demand.
    orbit_coords = list(r.ORBIT_PATTERN)
    orbit_bolt_axial_x = bolt_group_axial_increment_n(momentary_design, orbit_coords, "x")
    orbit_bolt_axial_y = bolt_group_axial_increment_n(momentary_design, orbit_coords, "y")
    orbit_bolt_axial = math.hypot(orbit_bolt_axial_x, orbit_bolt_axial_y)
    orbit_bolt_tension_stress = orbit_bolt_axial / M6_STRESS_AREA_MM2
    orbit_shear = bolt_shear_stress_mpa(radial_force, 4, M6_STRESS_AREA_MM2)

    # Lower flange 4x M8.
    m8_preload = bolt_preload_n(M8_STRESS_AREA_MM2)
    lower_coords = list(r.LOWER_PATTERN)
    lower_axial_x = bolt_group_axial_increment_n(bearing_moment, lower_coords, "x")
    lower_axial_y = bolt_group_axial_increment_n(bearing_moment, lower_coords, "y")
    lower_tension = math.hypot(lower_axial_x, lower_axial_y) / M8_STRESS_AREA_MM2
    lower_shear = bolt_shear_stress_mpa(radial_force, 4, M8_STRESS_AREA_MM2)

    # Assembly tolerance stacks.
    orbit_tol_margin = tolerance_margin(6.6, 6.03, 0.05, 0.05)
    lower_tol_margin = tolerance_margin(9.0, 8.03, 0.05, 0.05)
    # Designed adapter pilot, independent of vendor reducer pilot:
    # female 50 H7 = 50.000..50.025, male 50 g6 = 49.975..49.991 mm.
    pilot_min_clearance = 50.000 - 49.991
    pilot_max_clearance = 50.025 - 49.975

    checks = [
        check("continuous reducer torque", continuous_design, r.GEAR_RATED_TORQUE_NM, units="Nm"),
        check("average reducer torque", continuous_design, r.GEAR_AVERAGE_LIMIT_NM, units="Nm"),
        check("repeated peak reducer torque", repeated_design, r.GEAR_REPEATED_PEAK_NM, units="Nm"),
        check("3g momentary reducer torque", momentary_design, r.GEAR_MOMENTARY_PEAK_NM, units="Nm"),
        check("output bearing moment", bearing_moment, r.GEAR_ALLOWABLE_MOMENT_NM, units="Nm"),
        check("output bearing static radial load", radial_force, r.GEAR_STATIC_LOAD_N, units="N"),
        check("output bearing dynamic radial load", radial_force / BEARING_MOMENT_SAFETY,
              r.GEAR_DYNAMIC_LOAD_N, units="N"),
        check("motor continuous torque", required_cont_motor, r.MOTOR_RATED_TORQUE_NM, units="Nm"),
        check("motor repeated-peak torque", required_peak_motor, r.MOTOR_MAX_TORQUE_NM, units="Nm"),
        check("motor/reducer input speed", input_rpm,
              min(r.MOTOR_RATED_RPM, r.GEAR_MAX_AVG_INPUT_RPM), units="rpm"),
        check("motor mechanical output power", output_mech_power / GEAR_EFFICIENCY,
              r.MOTOR_POWER_W, units="W"),
        check("torsional elastic deflection", torsional_deflection_deg, 0.15, units="deg"),
        check("steel output hub torsional shear", output_hub_shear,
              STEEL_YIELD_MPA / STRUCTURAL_SAFETY, units="MPa"),
        check("moving carrier plate bending stress", carrier_stress,
              AL6061_YIELD_MPA / STRUCTURAL_SAFETY, units="MPa"),
        check("fixed housing web bending stress", fixed_web_stress,
              AL6061_YIELD_MPA / STRUCTURAL_SAFETY, units="MPa"),
        check("moving carrier elastic deflection", carrier_deflection, 0.35, units="mm"),
        check("fixed housing elastic deflection", fixed_deflection, 0.35, units="mm"),
        check("output M6 friction torque capacity", output_clamp_torque,
              repeated_design * STRUCTURAL_SAFETY, relation=">=", units="Nm"),
        check("output M6 bolt shear stress", output_bolt_shear,
              0.58 * BOLT_10P9_YIELD_MPA / STRUCTURAL_SAFETY, units="MPa"),
        check("ORBIT M6 bolt tension increment stress", orbit_bolt_tension_stress,
              BOLT_10P9_PROOF_MPA * 0.30, units="MPa",
              detail="incremental load only; joint remains preloaded"),
        check("ORBIT M6 direct shear stress", orbit_shear,
              0.58 * BOLT_10P9_YIELD_MPA / STRUCTURAL_SAFETY, units="MPa"),
        check("lower M8 bolt tension increment stress", lower_tension,
              BOLT_10P9_PROOF_MPA * 0.30, units="MPa"),
        check("lower M8 direct shear stress", lower_shear,
              0.58 * BOLT_10P9_YIELD_MPA / STRUCTURAL_SAFETY, units="MPa"),
        check("ORBIT pattern worst-case assembly clearance", orbit_tol_margin, 0.05,
              relation=">=", units="mm"),
        check("lower flange worst-case assembly clearance", lower_tol_margin, 0.15,
              relation=">=", units="mm"),
        check("50H7/50g6 pilot minimum diametral clearance", pilot_min_clearance, 0.005,
              relation=">=", units="mm"),
        check("50H7/50g6 pilot maximum diametral clearance", pilot_max_clearance, 0.080,
              units="mm"),
    ]

    report = {
        "qualified": all(c["passed"] for c in checks),
        "qualification_scope": "analytical pre-prototype engineering gate",
        "load_case": {
            "payload_kg": PAYLOAD_KG,
            "tool_extension_mm": TOOL_EXTENSION_M * 1000,
            "angular_acceleration_rad_s2": ANGULAR_ACCEL_RAD_S2,
            "max_output_speed_deg_s": MAX_OUTPUT_SPEED_DEG_S,
            "shock_g": SHOCK_G,
            "continuous_safety_factor": CONTINUOUS_SAFETY,
            "peak_safety_factor": PEAK_SAFETY,
            "structural_safety_factor": STRUCTURAL_SAFETY,
        },
        "cad_derived": {
            "moving_parts": len(moving),
            "moving_mass_kg": moving_mass,
            "max_geometry_radius_m": max_geometry_radius,
            "payload_radius_m": payload_radius,
            "moving_gravity_torque_nm": moving_gravity_torque,
            "payload_gravity_torque_nm": payload_gravity_torque,
            "static_torque_nm": static_torque,
            "moving_inertia_kg_m2": moving_inertia,
            "payload_inertia_kg_m2": payload_inertia,
            "acceleration_torque_nm": accel_torque,
            "continuous_design_torque_nm": continuous_design,
            "repeated_design_torque_nm": repeated_design,
            "momentary_3g_design_torque_nm": momentary_design,
            "bearing_design_moment_nm": bearing_moment,
            "bearing_design_radial_load_n": radial_force,
        },
        "drive": {
            "reducer": "Harmonic Drive CSG-20-120-2UH",
            "ratio": r.RATIO,
            "assumed_efficiency_for_sizing": GEAR_EFFICIENCY,
            "required_continuous_motor_torque_nm": required_cont_motor,
            "required_peak_motor_torque_nm": required_peak_motor,
            "input_speed_rpm_at_max_output_speed": input_rpm,
            "output_mechanical_power_w": output_mech_power,
            "torsional_deflection_deg_at_continuous_design": torsional_deflection_deg,
        },
        "interfaces": {
            "orbit_fasteners": "4x M6 class 10.9",
            "output_fasteners": "8x M6 class 10.9 on 64 mm BCD",
            "lower_fasteners": "4x M8 class 10.9",
            "orbit_pattern_tolerance_each_part_mm": 0.05,
            "pilot_fit": "50H7/50g6 intermediate adapter pilot",
            "pilot_clearance_range_mm": [pilot_min_clearance, pilot_max_clearance],
        },
        "checks": checks,
        "limitations": [
            "Reducer-side exact pilot/bolt geometry must match the purchased Harmonic Drive confirmation drawing before machining.",
            "Analytical beam/bolt models are conservative but are not full 3D nonlinear FEA.",
            "No fatigue spectrum, thermal soak, gearbox life-cycle, cable-flex, ingress, or physical proof-load test has yet been run.",
            "Qualification is for the stated 2 kg payload, 50 mm tool extension, 60 deg/s output speed, 4 rad/s^2 acceleration and 3g shock envelope only.",
        ],
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
        raise SystemExit("REACH-2 engineering gate failed: " + ", ".join(failed))


if __name__ == "__main__":
    main()
