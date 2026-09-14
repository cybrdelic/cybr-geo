"""CYBR REACH-2: load-bearing elbow module for ORBIT.

This replaces the exposed REACH-1 spur train with a compact coaxial architecture
built around the published envelope/performance of a Harmonic Drive
CSG-20-120-2UH gear unit and a 60 mm / 200 W servo envelope.

The CAD represents all in-house structure, fasteners, adapters, housing, output
carrier, motor envelope and the ORBIT interface. The commercial gear unit and
servo are represented by installation envelopes rather than reverse-engineered
internal geometry. Native units are millimetres.

Engineering validation lives in tools/reach2_engineering.py and is intentionally
separate from rendering. A render is not evidence of load capacity.
"""
from __future__ import annotations

import importlib.util
import math
from dataclasses import replace
from pathlib import Path

import cadquery as cq
import numpy as np

from mechanism_lab.core import Assembly, View, cad_part

ROOT = Path(__file__).resolve().parents[1]
ORBIT_RECIPE = ROOT / "examples" / "orbit_inspection_wrist.py"

# Existing ORBIT B01 interface. Do not change this pattern.
ORBIT_X = (-46.0, 2.0)
ORBIT_Y = (-45.0, 63.0)
ORBIT_PATTERN = tuple((x, y) for x in ORBIT_X for y in ORBIT_Y)

# Elbow axis and kinematics.
PIVOT = np.array([-22.0, 9.0, -60.0])
RATIO = 120.0
PERIOD = 12.0
MAX_ELBOW_DEG = 70.0

# Selected commercial reducer envelope/performance source:
# Harmonic Drive CSG-20-120-2UH. Published size-20 2UH OD is 93 mm and
# axial length B is 45.5 mm. Detailed vendor mounting drawing remains the
# procurement authority for the final reducer-side pilot and bolt pattern.
GEAR_OD = 93.0
GEAR_LEN = 45.5
GEAR_MASS_KG = 0.98
GEAR_RATED_TORQUE_NM = 52.0
GEAR_AVERAGE_LIMIT_NM = 64.0
GEAR_REPEATED_PEAK_NM = 113.0
GEAR_MOMENTARY_PEAK_NM = 191.0
GEAR_ALLOWABLE_MOMENT_NM = 91.0
GEAR_DYNAMIC_LOAD_N = 5780.0
GEAR_STATIC_LOAD_N = 9000.0
GEAR_MAX_AVG_INPUT_RPM = 3500.0
GEAR_TORSIONAL_K_NM_PER_RAD = 29000.0

# Delta ECMA C06 200 W class envelope/specification used for sizing.
MOTOR_FRAME = 60.0
MOTOR_BODY_LEN = 82.0
MOTOR_RATED_TORQUE_NM = 0.64
MOTOR_MAX_TORQUE_NM = 1.92
MOTOR_RATED_RPM = 3000.0
MOTOR_POWER_W = 200.0
MOTOR_MASS_KG = 1.25  # conservative installation-envelope mass

# Designed interfaces.
LOWER_PATTERN = ((-57.0, -26.0), (-57.0, 44.0), (13.0, -26.0), (13.0, 44.0))
OUTPUT_BOLT_RADIUS = 32.0
OUTPUT_BOLT_COUNT = 8
HOUSING_BOLT_RADIUS = 39.0
HOUSING_BOLT_COUNT = 8


def load_orbit_recipe():
    spec = importlib.util.spec_from_file_location("reach2_orbit_recipe", ORBIT_RECIPE)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def box(dx, dy, dz, center, fillet=0.0):
    q = cq.Workplane("XY").box(dx, dy, dz)
    if fillet:
        q = q.edges().fillet(fillet)
    return q.val().translate(tuple(center))


def cylinder(radius, length, origin, axis="Y"):
    direction = {"X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1)}[axis]
    return cq.Solid.makeCylinder(radius, length, cq.Vector(*origin), cq.Vector(*direction))


def annulus_y(ro, ri, y0, length, x, z):
    outer = cylinder(ro, length, (x, y0, z), "Y")
    if not ri:
        return outer
    return outer.cut(cylinder(ri, length + 2, (x, y0 - 1, z), "Y"))


def annulus_z(ro, ri, z0, length, x, y):
    outer = cylinder(ro, length, (x, y, z0), "Z")
    if not ri:
        return outer
    return outer.cut(cylinder(ri, length + 2, (x, y, z0 - 1), "Z"))


def plate_xz(points, y0, thickness, fillet=0.0):
    q = cq.Workplane("XZ", origin=(0, y0, 0)).polyline(points).close().extrude(thickness)
    if fillet:
        try:
            q = q.edges("|Y").fillet(fillet)
        except Exception:
            pass
    return q.val()


def socket_screw_z(x, y, z_bottom=-8.0, z_head=1.0, shaft_r=3.0, head_r=5.0, head_h=4.0):
    stem = cylinder(shaft_r, z_head - z_bottom, (x, y, z_bottom), "Z")
    head = cylinder(head_r, head_h, (x, y, z_head), "Z")
    socket = cq.Workplane("XY", origin=(x, y, z_head + head_h - .2)).polygon(6, 3.0).extrude(-2.0).val()
    return stem.fuse(head).cut(socket)


def socket_screw_y(x, y0, z, length=18.0, shaft_r=3.0, head_r=5.0, head_h=4.0):
    stem = cylinder(shaft_r, length, (x, y0, z), "Y")
    head = cylinder(head_r, head_h, (x, y0 + length, z), "Y")
    return stem.fuse(head)


def rotation_y(angle, pivot=(0, 0, 0)):
    c, s = math.cos(angle), math.sin(angle)
    R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)
    p = np.asarray(pivot, float)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p - R @ p
    return T


def elbow_angle(t):
    return math.radians(MAX_ELBOW_DEG) * math.sin(math.tau * t / PERIOD)


def build():
    orbit_recipe = load_orbit_recipe()
    orbit = orbit_recipe.build()

    # Remove only ORBIT's removable bench feet; preserve every other part.
    orbit_source = [p for p in orbit.parts if not p.name.startswith("B02_")]
    orbit_original = {}
    parts = []
    for p in orbit_source:
        q = replace(p, name="O_" + p.name, group="orbit_" + p.group)
        orbit_original[q.name] = p
        parts.append(q)

    mats = list(orbit.materials)

    def add(name, shape, material=0, group="reach2_fixed", role="", tiny=False):
        print("CAD", name, flush=True)
        p = cad_part(
            name, shape, material,
            tolerance=.018, angular=.12 if tiny else .050,
            analytic_normals=True,
            group=group, motion=group,
            role=role or name.replace("_", " "), provenance="designed-concept",
            finish_axis=(0., 1., 0.), finish_origin=tuple(shape.Center().toTuple()),
        )
        parts.append(p)
        return shape

    px, py, pz = map(float, PIVOT)

    # ------------------------------------------------------------------
    # Moving ORBIT/output structure.
    # ------------------------------------------------------------------
    saddle = box(80, 144, 8, (-22, 9, -4), 2.5)
    for x, y in ORBIT_PATTERN:
        saddle = saddle.cut(cylinder(3.4, 12, (x, y, -10), "Z"))
        saddle = saddle.cut(cylinder(5.8, 3.5, (x, y, -1.5), "Z"))
    add("R2_01_ORBIT_saddle", saddle, 0, "reach2_output", "8 mm 6061-T6 ORBIT saddle")

    for i, (x, y) in enumerate(ORBIT_PATTERN):
        add(f"R2_02_{i}_ORBIT_M6_10p9", socket_screw_z(x, y), 2, "reach2_output",
            "M6 class 10.9 ORBIT interface fastener", tiny=True)

    # Two compact output-side carrier plates. They sit entirely on the reducer
    # output side, avoiding a fake shaft passing through the commercial unit.
    carrier_profile = [(-62, -8), (18, -8), (12, -23), (2, -38),
                       (-7, -49), (-12, -60), (-32, -60), (-38, -49),
                       (-49, -38), (-57, -23)]
    for side, y0 in (("A", -42.0), ("B", -29.0)):
        plate = plate_xz(carrier_profile, y0, 9.0, 1.8)
        plate = plate.cut(cylinder(15.1, 11, (px, y0 - 1, pz), "Y"))
        plate = plate.fuse(annulus_y(26.0, 15.1, y0, 9.0, px, pz)).clean()
        add(f"R2_03_{side}_Output_carrier_plate", plate, 0, "reach2_output",
            "9 mm 6061-T6 output carrier plate")

    # Short steel output adapter hub, hollow for local wiring/service routing.
    add("R2_04_Output_adapter_hub", annulus_y(15.0, 7.0, -43.0, 30.0, px, pz),
        2, "reach2_output", "Steel hollow reducer-output adapter hub")
    add("R2_05_Output_face_plate", annulus_y(38.0, 15.1, -20.0, 7.0, px, pz),
        1, "reach2_output", "Machined aluminium reducer output face plate")

    for i in range(OUTPUT_BOLT_COUNT):
        a = math.tau * i / OUTPUT_BOLT_COUNT
        x = px + OUTPUT_BOLT_RADIUS * math.cos(a)
        z = pz + OUTPUT_BOLT_RADIUS * math.sin(a)
        add(f"R2_06_{i}_Output_M6_10p9", socket_screw_y(x, -27.0, z, 14.0),
            2, "reach2_output", "M6 class 10.9 reducer-output bolt", tiny=True)

    # Cross-braces tie the carrier plates into the wide ORBIT saddle.
    for i, x in enumerate((-48.0, 4.0)):
        rib = box(12, 28, 44, (x, -27.5, -27.0), 2.0)
        add(f"R2_07_{i}_Carrier_crossbrace", rib, 0, "reach2_output",
            "Carrier-to-saddle closed-section crossbrace")

    # ------------------------------------------------------------------
    # Commercial strain-wave reducer envelope and fixed housing.
    # ------------------------------------------------------------------
    gear_y0 = py - GEAR_LEN / 2.0
    gear = annulus_y(GEAR_OD / 2, 17.0, gear_y0, GEAR_LEN, px, pz)
    add("R2_10_CSG20_120_2UH_envelope", gear, 2, "reach2_fixed",
        "Commercial Harmonic Drive CSG-20-120-2UH installation envelope")

    # Reducer housing clamp rings and two load-path webs to lower structure.
    for side, y0 in (("OUT", gear_y0 - 5.0), ("MOTOR", gear_y0 + GEAR_LEN + 1.0)):
        ring = annulus_y(52.0, 46.8, y0, 5.0, px, pz)
        add(f"R2_11_{side}_Housing_clamp_ring", ring, 0, "reach2_fixed",
            "Split-machined reducer housing support ring")

    fixed_profile = [(-76, -154), (32, -154), (28, -122), (18, -94),
                     (10, -78), (2, -69), (-8, -64), (-36, -64),
                     (-47, -69), (-57, -78), (-66, -96), (-72, -122)]
    for side, y0 in (("A", -17.0), ("B", 35.0)):
        web = plate_xz(fixed_profile, y0, 10.0, 2.0)
        web = web.cut(cylinder(47.0, 12.0, (px, y0 - 1.0, pz), "Y"))
        add(f"R2_12_{side}_Fixed_housing_web", web, 0, "reach2_fixed",
            "10 mm 6061-T6 reducer housing load-path web")

    # Eight housing bolts represented on a 78 mm bolt circle. Final reducer-side
    # hole/pilot details must be reconciled to the purchased vendor drawing.
    for i in range(HOUSING_BOLT_COUNT):
        a = math.tau * (i + .5) / HOUSING_BOLT_COUNT
        x = px + HOUSING_BOLT_RADIUS * math.cos(a)
        z = pz + HOUSING_BOLT_RADIUS * math.sin(a)
        add(f"R2_13_{i}_Housing_M6_10p9", socket_screw_y(x, 31.0, z, 12.0),
            2, "reach2_fixed", "M6 class 10.9 housing-adapter bolt", tiny=True)

    # ------------------------------------------------------------------
    # Servo input package.
    # ------------------------------------------------------------------
    motor_face_y = gear_y0 + GEAR_LEN + 8.0
    motor_adapter = box(72, 8, 72, (px, motor_face_y + 4.0, pz), 4.0)
    motor_adapter = motor_adapter.cut(cylinder(20.0, 10.0, (px, motor_face_y - 1.0, pz), "Y"))
    add("R2_20_Motor_adapter_plate", motor_adapter, 0, "reach2_fixed",
        "6061-T6 60 mm servo adapter plate")

    for i, (dx, dz) in enumerate(((-24, -24), (-24, 24), (24, -24), (24, 24))):
        add(f"R2_21_{i}_Motor_M5_10p9", socket_screw_y(px + dx, motor_face_y + 8.0, pz + dz,
                                                      12.0, 2.5, 4.3, 3.5),
            2, "reach2_fixed", "M5 class 10.9 servo flange fastener", tiny=True)

    motor_body_y = motor_face_y + 12.0
    motor = box(MOTOR_FRAME, MOTOR_BODY_LEN, MOTOR_FRAME,
                (px, motor_body_y + MOTOR_BODY_LEN / 2.0, pz), 5.0)
    add("R2_22_ECMA_200W_servo_envelope", motor, 0, "reach2_fixed",
        "60 mm 200 W servo motor installation envelope")
    add("R2_23_Input_coupler", annulus_y(11.0, 4.0, gear_y0 + GEAR_LEN, 12.0, px, pz),
        5, "reach2_input", "Replaceable elastomer/metal servo coupling")

    # ------------------------------------------------------------------
    # Lower modular structure.
    # ------------------------------------------------------------------
    lower = box(104, 104, 10, (-22, 9, -151), 3.0)
    for x, y in LOWER_PATTERN:
        lower = lower.cut(cylinder(4.5, 14, (x, y, -158), "Z"))
        lower = lower.cut(cylinder(8.0, 4.0, (x, y, -147), "Z"))
    add("R2_30_Lower_modular_flange", lower, 0, "reach2_fixed",
        "10 mm 6061-T6 next-link flange with M8 clearance pattern")

    lower_cross = box(86, 72, 14, (-22, 9, -126), 3.0)
    add("R2_31_Lower_crossmember", lower_cross, 0, "reach2_fixed",
        "Closed lower crossmember distributing reducer loads")

    for side, x in (("L", -55.0), ("R", 11.0)):
        spine = box(16, 72, 58, (x, 9, -112), 3.0)
        add(f"R2_32_{side}_Vertical_spine", spine, 0, "reach2_fixed",
            "Vertical box spine between housing web and lower flange")

    for i, (x, y) in enumerate(LOWER_PATTERN):
        add(f"R2_33_{i}_Lower_M8_10p9", socket_screw_z(x, y, -159.0, -147.0, 4.0, 6.8, 5.0),
            2, "reach2_fixed", "M8 class 10.9 next-link fastener", tiny=True)

    # Elastomer hard stops are physically grounded and contact the moving carrier
    # only beyond the commanded ±70° range.
    for side, x in (("NEG", -64.0), ("POS", 20.0)):
        add(f"R2_34_{side}_Hard_stop", cylinder(5.0, 14.0, (x, -13.0, -84.0), "Y"),
            4, "reach2_fixed", "Elastomer overtravel hard stop")

    def motion(part, t, e):
        theta = elbow_angle(t)
        elbow_T = rotation_y(theta, PIVOT)
        if part.name.startswith("O_"):
            return elbow_T @ orbit.pose(orbit_original[part.name], t, 0.0)
        if part.group == "reach2_output":
            return elbow_T
        if part.group == "reach2_input":
            return rotation_y(theta * RATIO, PIVOT)
        return np.eye(4)

    views = {
        "hero": View(az=31, el=17, scale=170, target=(-13, 9, -49),
                     projection="perspective", focal_length_mm=72, f_stop=16,
                     title="CYBR REACH-2 + ORBIT"),
        "drive_side": View(az=80, el=8, scale=158, target=(-22, 34, -72),
                           projection="perspective", focal_length_mm=82, f_stop=18,
                           title="REACH-2 COAXIAL DRIVE"),
        "output_side": View(az=-53, el=16, scale=158, target=(-22, -28, -55),
                            projection="perspective", focal_length_mm=78, f_stop=18,
                            title="REACH-2 OUTPUT CARRIER"),
    }

    metadata = {
        "concept": "CYBR REACH-2 engineered elbow",
        "selected_reducer": "Harmonic Drive CSG-20-120-2UH",
        "reducer_ratio": RATIO,
        "reducer_envelope_mm": {"diameter": GEAR_OD, "length": GEAR_LEN},
        "reducer_mass_kg": GEAR_MASS_KG,
        "reducer_ratings_nm": {
            "rated": GEAR_RATED_TORQUE_NM,
            "average_limit": GEAR_AVERAGE_LIMIT_NM,
            "repeated_peak": GEAR_REPEATED_PEAK_NM,
            "momentary_peak": GEAR_MOMENTARY_PEAK_NM,
            "allowable_moment": GEAR_ALLOWABLE_MOMENT_NM,
        },
        "motor_envelope": "Delta ECMA C06 200 W class, 60 mm frame",
        "motor_rated_torque_nm": MOTOR_RATED_TORQUE_NM,
        "motor_peak_torque_nm": MOTOR_MAX_TORQUE_NM,
        "motor_rated_rpm": MOTOR_RATED_RPM,
        "orbit_interface_pattern_mm": [list(p) for p in ORBIT_PATTERN],
        "lower_interface_pattern_mm": [list(p) for p in LOWER_PATTERN],
        "output_bolt_circle_mm": 2 * OUTPUT_BOLT_RADIUS,
        "output_bolt_count": OUTPUT_BOLT_COUNT,
        "housing_bolt_circle_mm": 2 * HOUSING_BOLT_RADIUS,
        "housing_bolt_count": HOUSING_BOLT_COUNT,
        "elbow_range_degrees": [-MAX_ELBOW_DEG, MAX_ELBOW_DEG],
        "notes": [
            "Commercial reducer and servo internals are not reverse-engineered; installation envelopes are modeled.",
            "Reducer-side pilot and exact bolt details must be reconciled to the purchased vendor confirmation drawing before machining.",
            "All in-house structural parts are analytic CAD and are checked by tools/reach2_engineering.py.",
            "No physical prototype has yet been tested; engineering checks are deterministic analytical qualification, not certification.",
        ],
    }

    return Assembly("CYBR REACH-2 + ORBIT", parts, mats, views, metadata, motion)


if __name__ == "__main__":
    a = build()
    print(a.name, len(a.parts), "parts", "ratio", RATIO)
