"""Engineering gate for REACH-2 v5 integrated ORBIT-family elbow.

The load envelope is unchanged: 2 kg payload, +50 mm tool extension, 4 rad/s^2
elbow acceleration, 60 deg/s output speed and a 3g shock case. Unlike the v3b
gate, motor and reducer checks are replaced by the published output-side limits
of the integrated FHA-25C-H actuator.

This is deterministic analytical pre-prototype qualification, not certification.
Final manufacturing release remains blocked on purchased-unit STEP/confirmation
drawing overlay for exact bolt-circle phase and connector envelope.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RECIPE = ROOT / "examples" / "reach2_orbit_integrated_v5b.py"
CORE = ROOT / "tools" / "reach2_engineering.py"


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def qualify():
    r = load(RECIPE, "reach2_v5_recipe")
    core = load(CORE, "reach2_v5_core")
    a = r.build()
    pivot = np.asarray(r.PIVOT, float)

    def mass(p):
        if p.name == "R5_10_FHA25C_H_100_integrated_actuator":
            return r.ACTUATOR_MASS_KG
        return core.part_mass_kg(p)

    moving = [
        p for p in a.parts
        if p.name.startswith("O_") or p.group in ("reach2_output", "reach5_output")
    ]
    moving_mass = sum(mass(p) for p in moving)
    moving_gravity = sum(mass(p) * core.GRAVITY * core.center_radius_m(p, pivot) for p in moving)
    moving_inertia = sum(mass(p) * core.center_radius_m(p, pivot) ** 2 for p in moving)
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
        mass(p) * core.GRAVITY * core.SHOCK_G * core.center_radius_m(p, pivot)
        for p in moving
    )
    payload_radial_moment = core.PAYLOAD_KG * core.GRAVITY * core.SHOCK_G * payload_radius
    bearing_moment = (moving_radial_moment + payload_radial_moment) * core.BEARING_MOMENT_SAFETY
    radial_force = (moving_mass + core.PAYLOAD_KG) * core.GRAVITY * core.SHOCK_G * core.BEARING_MOMENT_SAFETY

    output_rpm = core.MAX_OUTPUT_SPEED_DEG_S / 360.0 * 60.0
    output_speed_rad_s = math.radians(core.MAX_OUTPUT_SPEED_DEG_S)
    output_power_w = continuous * output_speed_rad_s
    continuous_current = continuous / r.ACTUATOR_TORQUE_CONSTANT_NM_ARMS
    peak_current = momentary / r.ACTUATOR_TORQUE_CONSTANT_NM_ARMS
    moment_deflection_deg = math.degrees(bearing_moment / r.ACTUATOR_MOMENT_STIFFNESS_NM_RAD)

    # Hard-stop screen uses complete moving + payload inertia.
    kinetic_j = 0.5 * total_inertia * output_speed_rad_s**2
    stop_angle_rad = 0.008 / 0.110
    stop_peak_nm = 3.0 * kinetic_j / max(stop_angle_rad, 1e-9)

    # Primary custom structure: two 12 mm tapered legs with at least 24 mm depth.
    pedestal_stress = core.section_bending_stress_mpa(
        bearing_moment, r.PEDESTAL_THICKNESS_MM, r.LOAD_LEG_MIN_WIDTH_MM, count=2)
    pedestal_force = radial_force / core.BEARING_MOMENT_SAFETY
    pedestal_deflection = core.cantilever_tip_deflection_mm(
        pedestal_force, 86.0, r.PEDESTAL_THICKNESS_MM, r.LOAD_LEG_MIN_WIDTH_MM, count=2)

    # FHA output joint: 8x M6 around the published drawing dimension-L circle.
    m6_preload = core.bolt_preload_n(core.M6_STRESS_AREA_MM2)
    output_radius = r.ACTUATOR_OUTPUT_PCD_MM / 2.0
    output_clamp_capacity = core.bolt_circle_torque_capacity_nm(
        r.ACTUATOR_OUTPUT_THREAD_COUNT, m6_preload, core.CLAMP_FRICTION, output_radius)
    output_tangential = repeated * 1000.0 / output_radius
    output_bolt_shear = core.bolt_shear_stress_mpa(
        output_tangential, r.ACTUATOR_OUTPUT_THREAD_COUNT, core.M6_STRESS_AREA_MM2)

    # FHA fixed mounting flange: 8x M6-equivalent fasteners on the drawing-B circle.
    mount_radius = r.ACTUATOR_MOUNT_PCD_MM / 2.0
    mount_coords = [
        (mount_radius * math.cos(math.tau * (i + .5) / r.ACTUATOR_MOUNT_COUNT),
         mount_radius * math.sin(math.tau * (i + .5) / r.ACTUATOR_MOUNT_COUNT))
        for i in range(r.ACTUATOR_MOUNT_COUNT)
    ]
    mount_axial_x = core.bolt_group_axial_increment_n(bearing_moment, mount_coords, "x")
    mount_axial_y = core.bolt_group_axial_increment_n(bearing_moment, mount_coords, "y")
    mount_tension_stress = math.hypot(mount_axial_x, mount_axial_y) / core.M6_STRESS_AREA_MM2
    mount_shear = core.bolt_shear_stress_mpa(radial_force, r.ACTUATOR_MOUNT_COUNT, core.M6_STRESS_AREA_MM2)

    # Existing ORBIT 4xM6 interface.
    orbit_coords = list(r.ORBIT_PATTERN)
    orbit_ax_x = core.bolt_group_axial_increment_n(momentary, orbit_coords, "x")
    orbit_ax_y = core.bolt_group_axial_increment_n(momentary, orbit_coords, "y")
    orbit_tension = math.hypot(orbit_ax_x, orbit_ax_y) / core.M6_STRESS_AREA_MM2
    orbit_shear = core.bolt_shear_stress_mpa(radial_force, 4, core.M6_STRESS_AREA_MM2)

    # Lower 4xM8 interface carries the entire module, including the fixed actuator.
    total_module_mass = sum(mass(p) for p in a.parts)
    lower_center = np.array([-22.0, 9.0, -151.0], float)
    fixed_overturn = sum(
        mass(p) * core.GRAVITY * core.SHOCK_G * core.center_radius_m(p, lower_center)
        for p in a.parts
    ) * core.BEARING_MOMENT_SAFETY
    payload_lower_radius = payload_radius + abs(pivot[2] - lower_center[2]) / 1000.0
    payload_lower_moment = core.PAYLOAD_KG * core.GRAVITY * core.SHOCK_G * payload_lower_radius * core.BEARING_MOMENT_SAFETY
    lower_moment = fixed_overturn + payload_lower_moment
    lower_force = (total_module_mass + core.PAYLOAD_KG) * core.GRAVITY * core.SHOCK_G * core.BEARING_MOMENT_SAFETY
    lower_coords = list(r.LOWER_PATTERN)
    lower_ax_x = core.bolt_group_axial_increment_n(lower_moment, lower_coords, "x")
    lower_ax_y = core.bolt_group_axial_increment_n(lower_moment, lower_coords, "y")
    lower_tension = math.hypot(lower_ax_x, lower_ax_y) / core.M8_STRESS_AREA_MM2
    lower_shear = core.bolt_shear_stress_mpa(lower_force, 4, core.M8_STRESS_AREA_MM2)

    # Assembly tolerances. Select a controlled pilot-mount actuator option and
    # machine mating bores to +20 +/-10 um nominal. Conservative 35 um h7 width.
    output_pilot_min = (r.OUTPUT_PILOT_BORE_MM - .010) - r.ACTUATOR_FRONT_PILOT_OD_MM
    output_pilot_max = (r.OUTPUT_PILOT_BORE_MM + .010) - (r.ACTUATOR_FRONT_PILOT_OD_MM - .035)
    mount_pilot_nominal = 128.0
    mount_pilot_min = (r.PEDESTAL_PILOT_BORE_MM - .010) - mount_pilot_nominal
    mount_pilot_max = (r.PEDESTAL_PILOT_BORE_MM + .010) - (mount_pilot_nominal - .035)
    orbit_tol = core.tolerance_margin(6.6, 6.03, .05, .05)
    lower_tol = core.tolerance_margin(9.0, 8.03, .05, .05)

    # Actual envelope check in the actuator axis. This is the packaging quantity
    # that made v3 visibly too wide.
    ymins = [p.cad.BoundingBox().ymin for p in a.parts if p.name.startswith("R5_")]
    ymaxs = [p.cad.BoundingBox().ymax for p in a.parts if p.name.startswith("R5_")]
    actuator_package_depth = max(ymaxs) - min(ymins)

    checks = [
        core.check("v5 continuous actuator torque", continuous, r.ACTUATOR_CONTINUOUS_TORQUE_NM, units="Nm"),
        core.check("v5 3g actuator max torque", momentary, r.ACTUATOR_MAX_TORQUE_NM, units="Nm"),
        core.check("v5 actuator bearing moment", bearing_moment, r.ACTUATOR_ALLOWABLE_MOMENT_NM, units="Nm"),
        core.check("v5 actuator radial load", radial_force, r.ACTUATOR_ALLOWABLE_RADIAL_N, units="N"),
        core.check("v5 output speed", output_rpm, r.ACTUATOR_MAX_OUTPUT_RPM, units="rpm"),
        core.check("v5 continuous torque margin", r.ACTUATOR_CONTINUOUS_TORQUE_NM / continuous, 5.0, relation=">=", units="x"),
        core.check("v5 maximum torque margin", r.ACTUATOR_MAX_TORQUE_NM / momentary, 5.0, relation=">=", units="x"),
        core.check("v5 bearing moment margin", r.ACTUATOR_ALLOWABLE_MOMENT_NM / bearing_moment, 4.0, relation=">=", units="x"),
        core.check("v5 speed margin", r.ACTUATOR_MAX_OUTPUT_RPM / output_rpm, 3.0, relation=">=", units="x"),
        core.check("v5 continuous current thermal screen", continuous_current, .50 * r.ACTUATOR_CONTINUOUS_CURRENT_ARMS, units="Arms"),
        core.check("v5 momentary current", peak_current, r.ACTUATOR_MAX_CURRENT_ARMS, units="Arms"),
        core.check("v5 bearing angular deflection", moment_deflection_deg, .05, units="deg"),
        core.check("v5 full-speed hard-stop peak torque", stop_peak_nm, r.ACTUATOR_MAX_TORQUE_NM, units="Nm"),
        core.check("v5 pedestal bending stress", pedestal_stress, core.AL6061_YIELD_MPA / core.STRUCTURAL_SAFETY, units="MPa"),
        core.check("v5 pedestal elastic deflection", pedestal_deflection, .35, units="mm"),
        core.check("v5 output M6 friction torque capacity", output_clamp_capacity, repeated * core.STRUCTURAL_SAFETY, relation=">=", units="Nm"),
        core.check("v5 output M6 shear stress", output_bolt_shear, .58 * core.BOLT_10P9_YIELD_MPA / core.STRUCTURAL_SAFETY, units="MPa"),
        core.check("v5 actuator-mount M6 tension increment", mount_tension_stress, core.BOLT_10P9_PROOF_MPA * .30, units="MPa"),
        core.check("v5 actuator-mount M6 direct shear", mount_shear, .58 * core.BOLT_10P9_YIELD_MPA / core.STRUCTURAL_SAFETY, units="MPa"),
        core.check("v5 ORBIT M6 tension increment", orbit_tension, core.BOLT_10P9_PROOF_MPA * .30, units="MPa"),
        core.check("v5 ORBIT M6 direct shear", orbit_shear, .58 * core.BOLT_10P9_YIELD_MPA / core.STRUCTURAL_SAFETY, units="MPa"),
        core.check("v5 lower M8 tension increment", lower_tension, core.BOLT_10P9_PROOF_MPA * .30, units="MPa"),
        core.check("v5 lower M8 direct shear", lower_shear, .58 * core.BOLT_10P9_YIELD_MPA / core.STRUCTURAL_SAFETY, units="MPa"),
        core.check("v5 output pilot minimum diametral clearance", output_pilot_min, .005, relation=">=", units="mm"),
        core.check("v5 output pilot maximum diametral clearance", output_pilot_max, .080, units="mm"),
        core.check("v5 mount pilot minimum diametral clearance", mount_pilot_min, .005, relation=">=", units="mm"),
        core.check("v5 mount pilot maximum diametral clearance", mount_pilot_max, .080, units="mm"),
        core.check("v5 ORBIT pattern assembly clearance", orbit_tol, .05, relation=">=", units="mm"),
        core.check("v5 lower pattern assembly clearance", lower_tol, .15, relation=">=", units="mm"),
        core.check("v5 actuator-axis package depth", actuator_package_depth, 120.0, units="mm"),
    ]

    invalid = [p.name for p in a.parts if p.cad is None or not p.cad.isValid() or len(p.faces) == 0]
    old_boxy = [p.name for p in a.parts if p.name.startswith(("R2_20", "R2_22", "R3_20", "R3_22"))]
    checks.extend([
        core.check("v5 invalid CAD parts", len(invalid), 0, units="parts"),
        core.check("v5 legacy square motor package parts", len(old_boxy), 0, units="parts"),
    ])

    qualified = all(c["passed"] for c in checks)
    return {
        "model": "CYBR REACH-2 v5 + ORBIT",
        "engineering_revision": "v5 integrated FHA complete-CAD gate",
        "qualified": qualified,
        "manufacturing_release": False,
        "release_blockers": [
            "Overlay purchased FHA-25C-H confirmation drawing/STEP and lock exact bolt-circle phase.",
            "Verify selected pilot-mount option and connector/rear-exit envelope.",
            "Prototype proof-load, thermal, cable-flex and fatigue/cycle testing remain required.",
        ],
        "load_case": {
            "payload_kg": core.PAYLOAD_KG,
            "tool_extension_m": core.TOOL_EXTENSION_M,
            "angular_acceleration_rad_s2": core.ANGULAR_ACCEL_RAD_S2,
            "max_output_speed_deg_s": core.MAX_OUTPUT_SPEED_DEG_S,
            "shock_g": core.SHOCK_G,
        },
        "cad_derived": {
            "moving_parts": len(moving),
            "moving_mass_kg": moving_mass,
            "total_module_mass_kg": total_module_mass,
            "max_geometry_radius_m": max_radius,
            "payload_radius_m": payload_radius,
            "continuous_design_torque_nm": continuous,
            "repeated_design_torque_nm": repeated,
            "momentary_3g_design_torque_nm": momentary,
            "bearing_design_moment_nm": bearing_moment,
            "bearing_design_radial_load_n": radial_force,
            "lower_interface_design_moment_nm": lower_moment,
            "lower_interface_design_force_n": lower_force,
        },
        "drive": {
            "actuator": r.ACTUATOR_MODEL,
            "continuous_current_arms": continuous_current,
            "momentary_current_arms": peak_current,
            "output_speed_rpm": output_rpm,
            "output_mechanical_power_w": output_power_w,
            "bearing_angular_deflection_deg": moment_deflection_deg,
            "hard_stop_peak_torque_nm": stop_peak_nm,
            "package_depth_mm": actuator_package_depth,
        },
        "mass_accounting": {
            "moving_group_rule": "O_* + reach2_output + reach5_output",
            "moving_parts": len(moving),
            "complete": True,
        },
        "checks": checks,
    }


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
        raise SystemExit("REACH-2 v5 engineering gate failed: " + ", ".join(failed))


if __name__ == "__main__":
    main()
