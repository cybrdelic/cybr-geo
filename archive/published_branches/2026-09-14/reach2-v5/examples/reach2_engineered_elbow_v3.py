"""CYBR REACH-2 v3: engineering-hardened ORBIT elbow.

This revision keeps the validated ORBIT/output/lower-link structure from the
robust v2 model but replaces the size-20 / 120:1 drive package with a size-25 /
80:1 Harmonic Drive CSG-25-80-2UH installation package. The size-25 choice raises
output-bearing moment capacity and reducer torque margin while the lower ratio
reduces servo/reducer input speed. Commercial internals remain installation
envelopes; the published external dimensions and motor-side 10xM5/96 mm BCD
pattern are represented explicitly.

Native units are millimetres. This is engineering CAD, not certification.
"""
from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import cadquery as cq
import numpy as np

from mechanism_lab.core import Assembly, View, cad_part

ROOT = Path(__file__).resolve().parents[1]
V2 = ROOT / "examples" / "reach2_engineered_elbow_v2.py"


def _load_v2():
    spec = importlib.util.spec_from_file_location("reach2_v2_base", V2)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


_v2 = _load_v2()

# Preserve the existing ORBIT and lower-link interfaces exactly.
ORBIT_PATTERN = _v2.ORBIT_PATTERN
LOWER_PATTERN = _v2.LOWER_PATTERN
PIVOT = np.asarray(_v2.PIVOT, dtype=float)
PERIOD = _v2.PERIOD
MAX_ELBOW_DEG = _v2.MAX_ELBOW_DEG

# CSG-25-80-2UH published performance / envelope.
RATIO = 80.0
GEAR_OD = 107.0
GEAR_LEN = 52.0
GEAR_MASS_KG = 1.50
GEAR_RATED_TORQUE_NM = 82.0
GEAR_AVERAGE_LIMIT_NM = 113.0
GEAR_REPEATED_PEAK_NM = 178.0
GEAR_MOMENTARY_PEAK_NM = 332.0
GEAR_ALLOWABLE_MOMENT_NM = 156.0
GEAR_DYNAMIC_LOAD_N = 9600.0
GEAR_STATIC_LOAD_N = 15100.0
GEAR_MAX_AVG_INPUT_RPM = 3500.0
GEAR_TORSIONAL_K_NM_PER_RAD = 57000.0
GEAR_MOMENT_K_NM_PER_RAD = 242000.0

# Existing 60 mm / 200 W servo remains intentionally over-sized for thermal
# margin. Its torque envelope is still much larger than required at 80:1.
MOTOR_FRAME = _v2.MOTOR_FRAME
MOTOR_BODY_LEN = _v2.MOTOR_BODY_LEN
MOTOR_RATED_TORQUE_NM = _v2.MOTOR_RATED_TORQUE_NM
MOTOR_MAX_TORQUE_NM = _v2.MOTOR_MAX_TORQUE_NM
MOTOR_RATED_RPM = _v2.MOTOR_RATED_RPM
MOTOR_POWER_W = _v2.MOTOR_POWER_W
MOTOR_MASS_KG = _v2.MOTOR_MASS_KG

# In-house adapter-side interfaces.
OUTPUT_BOLT_RADIUS = 32.0
OUTPUT_BOLT_COUNT = 8
OUTPUT_BOLT_SIZE_MM = 8.0
HOUSING_BOLT_RADIUS = 48.0  # published Ø96 motor-side pattern
HOUSING_BOLT_COUNT = 10
HOUSING_BOLT_SIZE_MM = 5.0

# Published Ø86 h7 locating feature is received by a controlled custom bore.
# The machined support bore is specified as Ø86.020 ±0.010 mm, yielding a
# nominally slip-assemblable 0.010..0.090 mm diametral window against h7 worst
# case when conservative 35 µm h7 width is used in the qualification layer.
HOUSING_PILOT_BORE_NOMINAL_MM = 86.020
HOUSING_PILOT_BORE_TOL_MM = 0.010


def box(dx, dy, dz, center, fillet=0.0):
    q = cq.Workplane("XY").box(dx, dy, dz)
    if fillet:
        radius = min(float(fillet), 0.40 * min(float(dx), float(dy), float(dz)))
        try:
            candidate = q.edges().fillet(radius)
            if candidate.val().isValid():
                q = candidate
        except Exception:
            pass
    return q.val().translate(tuple(center))


def cylinder(radius, length, origin, axis="Y"):
    direction = {"X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1)}[axis]
    return cq.Solid.makeCylinder(radius, length, cq.Vector(*origin), cq.Vector(*direction))


def annulus_y(ro, ri, y0, length, x, z):
    outer = cylinder(ro, length, (x, y0, z), "Y")
    return outer if not ri else outer.cut(cylinder(ri, length + 2.0, (x, y0 - 1.0, z), "Y"))


def socket_screw_y(x, y0, z, length, shaft_r, head_r, head_h):
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
    old = _v2.build()

    # Drop the size-20 commercial package and its adapter hardware. Preserve the
    # ORBIT, moving carrier, lower flange, hard stops, and all other validated
    # structure from v2.
    remove_prefixes = (
        "R2_05", "R2_06_", "R2_10", "R2_11_", "R2_12_", "R2_13_",
        "R2_20", "R2_21_", "R2_22", "R2_23",
    )
    parts = [p for p in old.parts if not p.name.startswith(remove_prefixes)]
    mats = list(old.materials)

    def add(name, shape, material=0, group="reach3_fixed", role="", tiny=False):
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
        return p

    px, py, pz = map(float, PIVOT)
    gear_y0 = py - GEAR_LEN / 2.0

    # ------------------------------------------------------------------
    # Size-25 reducer and output adapter.
    # ------------------------------------------------------------------
    # Vendor drawing shows a stepped body, Ø86 h7 locating feature, Ø107 maximum
    # diameter and 52 mm overall axial envelope. Model the actual installation
    # silhouette rather than a generic cylinder.
    output_pilot = annulus_y(43.0, 10.0, gear_y0, 29.0, px, pz)
    main_flange = annulus_y(53.5, 13.0, gear_y0 + 29.0, 23.0, px, pz)
    reducer = output_pilot.fuse(main_flange).clean()
    add("R3_10_CSG25_80_2UH_envelope", reducer, 2, "reach3_fixed",
        "Commercial Harmonic Drive CSG-25-80-2UH installation envelope")

    # Output adapter: eight M8 fasteners into the published 8-M8 output face.
    # The in-house adapter keeps a 64 mm BCD; vendor STEP overlay remains the
    # manufacturing authority for final hole phase before release.
    output_plate = annulus_y(44.0, 15.1, gear_y0 - 8.0, 8.0, px, pz)
    add("R3_05_Output_face_adapter", output_plate, 1, "reach3_output",
        "Machined size-25 reducer output adapter plate")
    for i in range(OUTPUT_BOLT_COUNT):
        a = math.tau * i / OUTPUT_BOLT_COUNT
        x = px + OUTPUT_BOLT_RADIUS * math.cos(a)
        z = pz + OUTPUT_BOLT_RADIUS * math.sin(a)
        add(f"R3_06_{i}_Output_M8_12p9",
            socket_screw_y(x, gear_y0 - 7.0, z, 16.0, 4.0, 6.8, 5.0),
            2, "reach3_output", "M8 class 12.9 reducer output fastener", tiny=True)

    # ------------------------------------------------------------------
    # Fixed housing interface using published Ø86 locating feature and
    # 10xØ5.5/M5 holes on Ø96.
    # ------------------------------------------------------------------
    pilot_bore_r = HOUSING_PILOT_BORE_NOMINAL_MM / 2.0
    fixed_ring = annulus_y(61.0, pilot_bore_r, gear_y0 + 26.0, 12.0, px, pz)
    add("R3_11_Housing_pilot_adapter", fixed_ring, 0, "reach3_fixed",
        "Machined Ø86.020 locating-bore reducer housing adapter")

    for i in range(HOUSING_BOLT_COUNT):
        a = math.tau * i / HOUSING_BOLT_COUNT
        x = px + HOUSING_BOLT_RADIUS * math.cos(a)
        z = pz + HOUSING_BOLT_RADIUS * math.sin(a)
        add(f"R3_13_{i}_Housing_M5_12p9",
            socket_screw_y(x, gear_y0 + 30.0, z, 12.0, 2.5, 4.3, 3.5),
            2, "reach3_fixed", "M5 class 12.9 reducer housing fastener on Ø96 BCD", tiny=True)

    # Two 12 mm closed webs carry reducer bearing moment into the lower structure.
    fixed_profile = [(-86, -154), (42, -154), (38, -124), (30, -101),
                     (19, -82), (7, -70), (-7, -65), (-37, -65),
                     (-52, -70), (-64, -82), (-75, -101), (-82, -124)]
    for side, y0 in (("A", -19.0), ("B", 39.0)):
        q = cq.Workplane("XZ", origin=(0, y0, 0)).polyline(fixed_profile).close().extrude(12.0)
        web = q.val().cut(cylinder(55.0, 14.0, (px, y0 - 1.0, pz), "Y"))
        add(f"R3_12_{side}_Fixed_housing_web", web, 0, "reach3_fixed",
            "12 mm 6061-T6 size-25 reducer load-path web")

    # Add direct gusseted load paths from reducer webs to the existing lower
    # crossmember. These are intentionally simple prismatic solids so there is no
    # decorative geometry in the primary load path.
    for side, x in (("L", -60.0), ("R", 16.0)):
        add(f"R3_14_{side}_Housing_lower_gusset",
            box(18.0, 64.0, 54.0, (x, 9.0, -112.0), 2.0),
            0, "reach3_fixed", "Reducer-web to lower-crossmember box gusset")

    # ------------------------------------------------------------------
    # Servo package. Lower ratio drops rated input speed to 800 rpm at 60 deg/s.
    # ------------------------------------------------------------------
    motor_face_y = gear_y0 + GEAR_LEN + 10.0
    motor_adapter = box(80.0, 10.0, 80.0, (px, motor_face_y + 5.0, pz), 2.5)
    motor_adapter = motor_adapter.cut(cylinder(22.0, 12.0, (px, motor_face_y - 1.0, pz), "Y"))
    add("R3_20_Motor_adapter_plate", motor_adapter, 0, "reach3_fixed",
        "10 mm 6061-T6 60 mm servo adapter plate")

    for i, (dx, dz) in enumerate(((-24, -24), (-24, 24), (24, -24), (24, 24))):
        add(f"R3_21_{i}_Motor_M5_10p9",
            socket_screw_y(px + dx, motor_face_y + 10.0, pz + dz, 12.0, 2.5, 4.3, 3.5),
            2, "reach3_fixed", "M5 class 10.9 servo flange fastener", tiny=True)

    motor_body_y = motor_face_y + 14.0
    add("R3_22_ECMA_200W_servo_envelope",
        box(MOTOR_FRAME, MOTOR_BODY_LEN, MOTOR_FRAME,
            (px, motor_body_y + MOTOR_BODY_LEN / 2.0, pz), 3.0),
        0, "reach3_fixed", "60 mm 200 W servo motor installation envelope")
    add("R3_23_Flexible_input_coupler",
        annulus_y(13.0, 4.0, gear_y0 + GEAR_LEN, 15.0, px, pz),
        5, "reach3_input", "Flexible servo coupling with misalignment allowance")

    def motion(part, t, e):
        if part.name.startswith("R3_"):
            if part.group == "reach3_output":
                return rotation_y(elbow_angle(t), PIVOT)
            if part.group == "reach3_input":
                return rotation_y(elbow_angle(t) * RATIO, PIVOT)
            return np.eye(4)
        return old.pose(part, t, e)

    views = {
        "hero": View(az=33, el=18, scale=182, target=(-13, 9, -52),
                     projection="perspective", focal_length_mm=72, f_stop=16,
                     title="CYBR REACH-2 v3 + ORBIT"),
        "drive_side": View(az=78, el=8, scale=172, target=(-22, 37, -75),
                           projection="perspective", focal_length_mm=82, f_stop=18,
                           title="REACH-2 v3 SIZE-25 DRIVE"),
        "output_side": View(az=-49, el=17, scale=168, target=(-22, -28, -58),
                            projection="perspective", focal_length_mm=78, f_stop=18,
                            title="REACH-2 v3 OUTPUT CARRIER"),
    }

    metadata = dict(old.metadata)
    metadata.update(
        concept="CYBR REACH-2 v3 engineered elbow",
        selected_reducer="Harmonic Drive CSG-25-80-2UH",
        reducer_ratio=RATIO,
        reducer_envelope_mm={"diameter": GEAR_OD, "length": GEAR_LEN},
        reducer_mass_kg=GEAR_MASS_KG,
        reducer_ratings_nm={
            "rated": GEAR_RATED_TORQUE_NM,
            "average_limit": GEAR_AVERAGE_LIMIT_NM,
            "repeated_peak": GEAR_REPEATED_PEAK_NM,
            "momentary_peak": GEAR_MOMENTARY_PEAK_NM,
            "allowable_moment": GEAR_ALLOWABLE_MOMENT_NM,
        },
        vendor_geometry={
            "max_od_mm": 107.0,
            "overall_length_mm": 52.0,
            "housing_pilot": "Ø86 h7",
            "housing_pattern": "10xØ5.5 / 10xM5 on Ø96",
            "output_threads": "8xM8x12",
        },
        housing_pilot_bore="Ø86.020 ±0.010 mm custom mating bore",
        output_fasteners="8x M8 class 12.9",
        housing_fasteners="10x M5 class 12.9 on Ø96 BCD",
        cad_revision="REACH-2 v3 size-25/80 engineering hardening",
        notes=[
            "Commercial reducer and servo internals remain installation envelopes.",
            "Published CSG-25 external/pattern geometry is represented; final manufacturing release still requires vendor STEP overlay inspection.",
            "Primary in-house load paths are prismatic 6061-T6/steel CAD, not decorative surfaces.",
            "Engineering qualification is deterministic analytical pre-prototype validation, not physical certification.",
        ],
    )

    return Assembly(
        "CYBR REACH-2 v3 + ORBIT",
        parts,
        mats,
        views,
        metadata,
        motion,
    )


if __name__ == "__main__":
    a = build()
    print(a.name, len(a.parts), "parts", "ratio", RATIO)
