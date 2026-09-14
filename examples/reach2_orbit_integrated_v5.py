"""CYBR REACH-2 v5: ORBIT-family integrated hollow-shaft elbow.

The previous CSG-25 + square 60 mm servo package was engineering-capable but
visually unrelated to ORBIT. This revision preserves the validated ORBIT saddle,
carrier, lower interface and elbow kinematics while replacing the separate
reducer/motor/coupler stack with one coaxial FHA-25C-H 100:1 hollow-shaft servo
actuator installation envelope.

The fixed mounting structure deliberately follows ORBIT's rear pedestal language:
a graphite annular mounting plate with two tapered load legs, satin circular
interfaces, visible steel fasteners, and teal removable service retainers.
Commercial actuator internals remain an installation envelope. Vendor
confirmation drawing remains the manufacturing authority for final hole phase.
Native units are millimetres.
"""
from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import cadquery as cq
import numpy as np

from mechanism_lab.core import Assembly, View, cad_part

ROOT = Path(__file__).resolve().parents[1]
BASE_PATH = ROOT / "examples" / "reach2_engineered_elbow.py"


def _load_base():
    spec = importlib.util.spec_from_file_location("reach2_v5_base", BASE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


_base = _load_base()

ORBIT_PATTERN = _base.ORBIT_PATTERN
LOWER_PATTERN = _base.LOWER_PATTERN
PIVOT = np.asarray(_base.PIVOT, dtype=float)
PERIOD = _base.PERIOD
MAX_ELBOW_DEG = _base.MAX_ELBOW_DEG

# Harmonic Drive FHA-25C-H, 100:1, no-brake installation envelope / ratings.
RATIO = 100.0
ACTUATOR_MODEL = "Harmonic Drive FHA-25C-H 100:1"
ACTUATOR_MASS_KG = 4.60
ACTUATOR_CONTINUOUS_TORQUE_NM = 72.0
ACTUATOR_MAX_TORQUE_NM = 233.0
ACTUATOR_MAX_OUTPUT_RPM = 45.0
ACTUATOR_ALLOWABLE_RADIAL_N = 4900.0
ACTUATOR_ALLOWABLE_AXIAL_N = 14700.0
ACTUATOR_ALLOWABLE_MOMENT_NM = 370.0
ACTUATOR_MOMENT_STIFFNESS_NM_RAD = 490000.0
ACTUATOR_TORQUE_CONSTANT_NM_ARMS = 86.0
ACTUATOR_CONTINUOUS_CURRENT_ARMS = 1.30
ACTUATOR_MAX_CURRENT_ARMS = 3.0

# Published size-25 outline dimensions.
ACTUATOR_BODY_OD_MM = 142.0
ACTUATOR_MAX_OD_MM = 155.0
ACTUATOR_LENGTH_MM = 106.5
ACTUATOR_FRONT_PILOT_OD_MM = 125.0
ACTUATOR_REAR_PILOT_OD_MM = 85.0
ACTUATOR_HOLLOW_BORE_MM = 42.0
ACTUATOR_OUTPUT_THREAD_COUNT = 8
ACTUATOR_OUTPUT_THREAD_SIZE_MM = 6.0
ACTUATOR_OUTPUT_PCD_MM = 123.0  # drawing dimension L; verify phase against purchased unit
ACTUATOR_MOUNT_COUNT = 8
ACTUATOR_MOUNT_CLEARANCE_MM = 6.6
ACTUATOR_MOUNT_PCD_MM = 142.0   # drawing dimension B; verify phase against purchased unit

# In-house ORBIT-family packaging constraints.
PEDESTAL_THICKNESS_MM = 12.0
PEDESTAL_RING_OD_MM = 164.0
PEDESTAL_PILOT_BORE_MM = 128.20
LOAD_LEG_MIN_WIDTH_MM = 24.0
OUTPUT_ADAPTER_THICKNESS_MM = 8.0
OUTPUT_ADAPTER_OD_MM = 142.0
OUTPUT_PILOT_BORE_MM = 125.12


def box(dx, dy, dz, center, fillet=0.0):
    q = cq.Workplane("XY").box(dx, dy, dz)
    if fillet:
        r = min(float(fillet), 0.40 * min(float(dx), float(dy), float(dz)))
        try:
            candidate = q.edges().fillet(r)
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


def plate_xz(points, y0, thickness):
    return cq.Workplane("XZ", origin=(0, y0, 0)).polyline(points).close().extrude(thickness).val()


def socket_screw_y(x, y0, z, length=16.0, shaft_r=3.0, head_r=5.0, head_h=4.0):
    stem = cylinder(shaft_r, length, (x, y0, z), "Y")
    head = cylinder(head_r, head_h, (x, y0 + length, z), "Y")
    return stem.fuse(head)


def rotation_y(angle, pivot=(0, 0, 0)):
    return _base.rotation_y(angle, pivot)


def elbow_angle(t):
    return math.radians(MAX_ELBOW_DEG) * math.sin(math.tau * t / PERIOD)


def build():
    old = _base.build()

    # Keep ORBIT, saddle, output carrier/hub/crossbraces, lower flange/fasteners
    # and hard stops. Remove the separate reducer, motor, coupler, old output face
    # interface and generic lower cross/spines.
    remove_prefixes = (
        "R2_05", "R2_06_",
        "R2_10", "R2_11_", "R2_12_", "R2_13_",
        "R2_20", "R2_21_", "R2_22", "R2_23",
        "R2_31", "R2_32_",
    )
    parts = [p for p in old.parts if not p.name.startswith(remove_prefixes)]
    mats = list(old.materials)

    def add(name, shape, material=0, group="reach5_fixed", role="", tiny=False):
        if not shape.isValid():
            raise ValueError(f"invalid REACH-2 v5 CAD: {name}")
        p = cad_part(
            name, shape, material,
            tolerance=.018, angular=.12 if tiny else .050,
            analytic_normals=True, group=group, motion=group,
            role=role or name.replace("_", " "), provenance="designed-concept",
            finish_axis=(0., 1., 0.), finish_origin=tuple(shape.Center().toTuple()),
        )
        parts.append(p)
        return p

    px, py, pz = map(float, PIVOT)

    # Output plane remains at the old carrier's inner face so ORBIT location and
    # elbow axis do not move. Actuator extends rearward (+Y), reducing total axial
    # package length versus reducer + adapter + separate servo.
    output_y = -13.0
    body_y0 = output_y

    # Commercial actuator silhouette: exposed satin/silver circular core with a
    # 42 mm true hollow bore. It is fixed; only its output flange rotates.
    front_flange = annulus_y(ACTUATOR_MAX_OD_MM / 2.0,
                            ACTUATOR_HOLLOW_BORE_MM / 2.0,
                            body_y0, 15.0, px, pz)
    main_body = annulus_y(ACTUATOR_BODY_OD_MM / 2.0,
                          ACTUATOR_HOLLOW_BORE_MM / 2.0,
                          body_y0 + 15.0, ACTUATOR_LENGTH_MM - 15.0, px, pz)
    actuator = front_flange.fuse(main_body).clean()
    add("R5_10_FHA25C_H_100_integrated_actuator", actuator, 1, "reach5_fixed",
        "Commercial FHA-25C-H 100:1 hollow-shaft servo actuator envelope")

    # Moving output adapter: vendor output pilot + eight M6 threads connect the
    # actuator's rotating flange directly to the existing steel carrier hub.
    output_adapter = annulus_y(OUTPUT_ADAPTER_OD_MM / 2.0,
                               OUTPUT_PILOT_BORE_MM / 2.0,
                               output_y - OUTPUT_ADAPTER_THICKNESS_MM,
                               OUTPUT_ADAPTER_THICKNESS_MM, px, pz)
    add("R5_05_Output_adapter", output_adapter, 1, "reach5_output",
        "Satin machined FHA output-to-ORBIT carrier adapter")
    output_bolt_r = ACTUATOR_OUTPUT_PCD_MM / 2.0
    for i in range(ACTUATOR_OUTPUT_THREAD_COUNT):
        a = math.tau * i / ACTUATOR_OUTPUT_THREAD_COUNT
        x = px + output_bolt_r * math.cos(a)
        z = pz + output_bolt_r * math.sin(a)
        add(f"R5_06_{i}_Output_M6_12p9",
            socket_screw_y(x, output_y - 7.0, z, 14.0, 3.0, 5.0, 4.0),
            2, "reach5_output", "M6 class 12.9 FHA output fastener", tiny=True)

    # ORBIT-like graphite front pedestal. The large circular ring is the fixed
    # actuator mount; twin tapered legs carry bearing moment directly into the
    # existing lower modular flange instead of a rectangular gearbox cage.
    mount_y0 = py - PEDESTAL_THICKNESS_MM / 2.0
    mount_ring = annulus_y(PEDESTAL_RING_OD_MM / 2.0,
                           PEDESTAL_PILOT_BORE_MM / 2.0,
                           mount_y0, PEDESTAL_THICKNESS_MM, px, pz)
    leg_bottom_z = -146.0
    left_leg = plate_xz([
        (px - 71, pz - 18), (px - 52, pz - 39),
        (px - 48, leg_bottom_z), (px - 72, leg_bottom_z),
        (px - 79, pz - 44),
    ], mount_y0, PEDESTAL_THICKNESS_MM)
    right_leg = plate_xz([
        (px + 71, pz - 18), (px + 52, pz - 39),
        (px + 48, leg_bottom_z), (px + 24, leg_bottom_z),
        (px + 79, pz - 44),
    ], mount_y0, PEDESTAL_THICKNESS_MM)
    pedestal = mount_ring.fuse(left_leg).fuse(right_leg).clean()
    add("R5_12_ORBIT_family_actuator_pedestal", pedestal, 0, "reach5_fixed",
        "12 mm graphite anodized annular actuator pedestal with tapered load legs")

    # Eight mounting fasteners on the vendor front mounting circle. Hole phase is
    # explicit in CAD but remains subject to confirmation-drawing overlay.
    mount_r = ACTUATOR_MOUNT_PCD_MM / 2.0
    for i in range(ACTUATOR_MOUNT_COUNT):
        a = math.tau * (i + 0.5) / ACTUATOR_MOUNT_COUNT
        x = px + mount_r * math.cos(a)
        z = pz + mount_r * math.sin(a)
        add(f"R5_13_{i}_Mount_M6_12p9",
            socket_screw_y(x, mount_y0 - 1.0, z, 16.0, 3.0, 5.0, 4.0),
            2, "reach5_fixed", "M6 class 12.9 FHA fixed-mount fastener", tiny=True)

    # Teal physical service hardware carries the same color hierarchy as ORBIT.
    add("R5_30_Output_service_retainer",
        annulus_y(68.5, 63.0, output_y - 10.5, 2.5, px, pz),
        5, "reach5_output", "Teal anodized removable output service retainer")
    add("R5_31_Hollow_shaft_cable_gland",
        annulus_y(20.5, 16.0, output_y - 3.0, 4.0, px, pz),
        5, "reach5_output", "Teal anodized hollow-shaft cable/service gland")
    add("R5_32_Rear_connector_guard",
        annulus_y(47.0, 43.0, body_y0 + ACTUATOR_LENGTH_MM - 3.0, 3.0, px, pz),
        5, "reach5_fixed", "Teal anodized rear connector/service guard")

    def motion(part, t, e):
        theta = elbow_angle(t)
        elbow_T = rotation_y(theta, PIVOT)
        if part.name.startswith("R5_"):
            return elbow_T if part.group == "reach5_output" else np.eye(4)
        # Retained ORBIT/R2 output parts use the original elbow motion.
        return old.pose(part, t, e)

    views = {
        "hero": View(az=30, el=18, scale=174, target=(-15, 10, -51),
                     projection="perspective", focal_length_mm=78, f_stop=16,
                     title="CYBR REACH-2 v5 + ORBIT"),
        "output_side": View(az=-47, el=16, scale=166, target=(-20, -20, -58),
                            projection="perspective", focal_length_mm=82, f_stop=18,
                            title="REACH-2 v5 ORBIT FAMILY OUTPUT"),
        "rear_side": View(az=58, el=12, scale=170, target=(-18, 40, -58),
                          projection="perspective", focal_length_mm=82, f_stop=18,
                          title="REACH-2 v5 HOLLOW-SHAFT ACTUATOR"),
    }

    metadata = dict(old.metadata)
    metadata.update(
        concept="CYBR REACH-2 v5 ORBIT-family integrated elbow",
        selected_actuator=ACTUATOR_MODEL,
        reducer_ratio=RATIO,
        actuator_ratings={
            "continuous_torque_nm": ACTUATOR_CONTINUOUS_TORQUE_NM,
            "max_torque_nm": ACTUATOR_MAX_TORQUE_NM,
            "max_output_rpm": ACTUATOR_MAX_OUTPUT_RPM,
            "allowable_radial_n": ACTUATOR_ALLOWABLE_RADIAL_N,
            "allowable_axial_n": ACTUATOR_ALLOWABLE_AXIAL_N,
            "allowable_moment_nm": ACTUATOR_ALLOWABLE_MOMENT_NM,
            "moment_stiffness_nm_rad": ACTUATOR_MOMENT_STIFFNESS_NM_RAD,
        },
        actuator_envelope_mm={
            "body_od": ACTUATOR_BODY_OD_MM,
            "max_od": ACTUATOR_MAX_OD_MM,
            "length": ACTUATOR_LENGTH_MM,
            "front_pilot_od": ACTUATOR_FRONT_PILOT_OD_MM,
            "hollow_bore": ACTUATOR_HOLLOW_BORE_MM,
        },
        aesthetic_family="CYBR ORBIT",
        cad_revision="REACH-2 v5 integrated FHA ORBIT-family packaging",
        notes=[
            "Square separate servo, flexible coupler and standalone CSG reducer are eliminated.",
            "FHA-25C-H is represented by its published installation envelope and ratings, not reverse-engineered internals.",
            "Vendor confirmation drawing/STEP remains the manufacturing authority for final bolt-circle phase and connector clearance.",
            "Teal components are real removable service retainers/glands, not render-only decoration.",
        ],
    )

    return Assembly("CYBR REACH-2 v5 + ORBIT", parts, mats, views, metadata, motion)


if __name__ == "__main__":
    a = build()
    print(a.name, len(a.parts), "parts", ACTUATOR_MODEL)
