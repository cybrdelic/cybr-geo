"""CYBR REACH-2 v4: ORBIT-family elbow packaging.

This revision keeps the qualified size-25 / 80:1 drivetrain, ORBIT interface,
output carrier, reducer fasteners and motion model from REACH-2 v3b, but replaces
the visually unrelated fixed housing pieces with an ORBIT-family architecture:
paired annular bearing pedestals, tapered twin load ribs, rounded lower gussets,
a circular servo adapter, and explicit teal/satin service rings.

The primary load-path minimum sections are not reduced relative to v3:
12 mm web thickness, >=24 mm rib width and 18 mm lower gusset thickness are
retained as release constraints. Native units are millimetres.
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

import cadquery as cq
import numpy as np

from mechanism_lab.core import Assembly, View, cad_part

ROOT = Path(__file__).resolve().parents[1]
V3 = ROOT / "examples" / "reach2_engineered_elbow_v3.py"


def _load_v3():
    spec = importlib.util.spec_from_file_location("reach2_v3_base", V3)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


_v3 = _load_v3()

RATIO = _v3.RATIO
GEAR_OD = _v3.GEAR_OD
GEAR_LEN = _v3.GEAR_LEN
GEAR_MASS_KG = _v3.GEAR_MASS_KG
GEAR_RATED_TORQUE_NM = _v3.GEAR_RATED_TORQUE_NM
GEAR_AVERAGE_LIMIT_NM = _v3.GEAR_AVERAGE_LIMIT_NM
GEAR_REPEATED_PEAK_NM = _v3.GEAR_REPEATED_PEAK_NM
GEAR_MOMENTARY_PEAK_NM = _v3.GEAR_MOMENTARY_PEAK_NM
GEAR_ALLOWABLE_MOMENT_NM = _v3.GEAR_ALLOWABLE_MOMENT_NM
GEAR_DYNAMIC_LOAD_N = _v3.GEAR_DYNAMIC_LOAD_N
GEAR_STATIC_LOAD_N = _v3.GEAR_STATIC_LOAD_N
GEAR_MAX_AVG_INPUT_RPM = _v3.GEAR_MAX_AVG_INPUT_RPM
GEAR_TORSIONAL_K_NM_PER_RAD = _v3.GEAR_TORSIONAL_K_NM_PER_RAD
GEAR_MOMENT_K_NM_PER_RAD = _v3.GEAR_MOMENT_K_NM_PER_RAD
MOTOR_FRAME = _v3.MOTOR_FRAME
MOTOR_BODY_LEN = _v3.MOTOR_BODY_LEN
MOTOR_RATED_TORQUE_NM = _v3.MOTOR_RATED_TORQUE_NM
MOTOR_MAX_TORQUE_NM = _v3.MOTOR_MAX_TORQUE_NM
MOTOR_RATED_RPM = _v3.MOTOR_RATED_RPM
MOTOR_POWER_W = _v3.MOTOR_POWER_W
MOTOR_MASS_KG = _v3.MOTOR_MASS_KG
ORBIT_PATTERN = _v3.ORBIT_PATTERN
LOWER_PATTERN = _v3.LOWER_PATTERN
PIVOT = np.asarray(_v3.PIVOT, dtype=float)
PERIOD = _v3.PERIOD
MAX_ELBOW_DEG = _v3.MAX_ELBOW_DEG
OUTPUT_BOLT_RADIUS = _v3.OUTPUT_BOLT_RADIUS
OUTPUT_BOLT_COUNT = _v3.OUTPUT_BOLT_COUNT
HOUSING_BOLT_RADIUS = _v3.HOUSING_BOLT_RADIUS
HOUSING_BOLT_COUNT = _v3.HOUSING_BOLT_COUNT
HOUSING_PILOT_BORE_NOMINAL_MM = _v3.HOUSING_PILOT_BORE_NOMINAL_MM
HOUSING_PILOT_BORE_TOL_MM = _v3.HOUSING_PILOT_BORE_TOL_MM

WEB_THICKNESS_MM = 12.0
RING_OUTER_RADIUS_MM = 62.0
RING_INNER_RADIUS_MM = 55.0
RING_RADIAL_SECTION_MM = RING_OUTER_RADIUS_MM - RING_INNER_RADIUS_MM
RIB_MIN_WIDTH_MM = 24.0
GUSSET_THICKNESS_MM = 18.0
MAX_FIXED_SILHOUETTE_MM = 124.0
MOTOR_ADAPTER_OD_MM = 82.0


def cylinder(radius, length, origin, axis="Y"):
    direction = {"X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1)}[axis]
    return cq.Solid.makeCylinder(radius, length, cq.Vector(*origin), cq.Vector(*direction))


def annulus_y(ro, ri, y0, length, x, z):
    outer = cylinder(ro, length, (x, y0, z), "Y")
    return outer.cut(cylinder(ri, length + 2.0, (x, y0 - 1.0, z), "Y")) if ri else outer


def plate_xz(points, y0, thickness):
    return cq.Workplane("XZ", origin=(0, y0, 0)).polyline(points).close().extrude(thickness).val()


def box(dx, dy, dz, center, fillet=0.0):
    q = cq.Workplane("XY").box(dx, dy, dz)
    if fillet:
        radius = min(float(fillet), 0.40 * min(float(dx), float(dy), float(dz)))
        try:
            q = q.edges().fillet(radius)
        except Exception:
            pass
    return q.val().translate(tuple(center))


def elbow_angle(t):
    return _v3.elbow_angle(t)


def rotation_y(angle, pivot=(0, 0, 0)):
    return _v3.rotation_y(angle, pivot)


def build():
    old = _v3.build()
    remove = {
        "R3_12_A_Fixed_housing_web",
        "R3_12_B_Fixed_housing_web",
        "R3_14_L_Housing_lower_gusset",
        "R3_14_R_Housing_lower_gusset",
        "R3_20_Motor_adapter_plate",
    }
    parts = [p for p in old.parts if p.name not in remove]
    mats = list(old.materials)

    def add(name, shape, material=0, group="reach4_fixed", role="", tiny=False):
        if not shape.isValid():
            raise ValueError(f"invalid v4 CAD: {name}")
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
    gear_y0 = py - GEAR_LEN / 2.0

    for side, y0 in (("A", -19.0), ("B", 39.0)):
        ring = annulus_y(RING_OUTER_RADIUS_MM, RING_INNER_RADIUS_MM,
                         y0, WEB_THICKNESS_MM, px, pz)
        zroot = pz - 89.0
        left = plate_xz([
            (px - 56, pz - 18), (px - 36, pz - 38),
            (px - 49, zroot), (px - 73, zroot),
            (px - 57, pz - 42),
        ], y0, WEB_THICKNESS_MM)
        right = plate_xz([
            (px + 56, pz - 18), (px + 36, pz - 38),
            (px + 49, zroot), (px + 25, zroot),
            (px + 57, pz - 42),
        ], y0, WEB_THICKNESS_MM)
        web = ring.fuse(left).fuse(right).clean()
        add(f"R4_12_{side}_Orbit_family_pedestal", web, 0, "reach4_fixed",
            "12 mm graphite anodized annular reducer pedestal with twin tapered load ribs")

    for side, x in (("L", px - 38.0), ("R", px + 38.0)):
        g = box(GUSSET_THICKNESS_MM, 54.0, 48.0, (x, py, pz - 69.0), 3.0)
        relief = cylinder(9.0, 56.0, (x, py - 28.0, pz - 58.0), "Y")
        g = g.cut(relief)
        add(f"R4_14_{side}_Rounded_lower_gusset", g, 0, "reach4_fixed",
            "18 mm graphite anodized rounded lower reducer gusset")

    motor_face_y = gear_y0 + GEAR_LEN + 10.0
    motor_adapter = annulus_y(MOTOR_ADAPTER_OD_MM / 2.0, 22.0,
                              motor_face_y, 10.0, px, pz)
    add("R4_20_Circular_motor_adapter", motor_adapter, 1, "reach4_fixed",
        "Satin machined circular 60 mm servo adapter")

    # Count this moving retainer in the same qualified output group as the rest
    # of the v3 output structure so v3b mass/inertia accounting remains complete.
    add("R4_30_Output_service_ring",
        annulus_y(48.0, 42.0, gear_y0 - 10.5, 2.5, px, pz),
        5, "reach3_output", "Teal anodized removable output service retainer")
    add("R4_31_Housing_service_ring",
        annulus_y(61.5, 55.5, gear_y0 + 39.0, 2.5, px, pz),
        5, "reach4_fixed", "Teal anodized reducer housing service retainer")

    motor_body_y = motor_face_y + 14.0
    add("R4_32_Motor_rear_service_cap",
        box(58.0, 3.0, 58.0,
            (px, motor_body_y + MOTOR_BODY_LEN + 1.5, pz), 3.0),
        5, "reach4_fixed", "Teal anodized removable servo rear service cap")

    def motion(part, t, e):
        if part.name.startswith("R4_"):
            if part.group in ("reach3_output", "reach4_output"):
                return rotation_y(elbow_angle(t), PIVOT)
            return np.eye(4)
        return old.pose(part, t, e)

    views = {
        "hero": View(az=31, el=18, scale=178, target=(-13, 7, -50),
                     projection="perspective", focal_length_mm=76, f_stop=16,
                     title="CYBR REACH-2 v4 + ORBIT"),
        "family_side": View(az=-40, el=15, scale=170, target=(-18, -12, -58),
                            projection="perspective", focal_length_mm=82, f_stop=18,
                            title="REACH-2 v4 ORBIT FAMILY"),
        "drive_side": View(az=67, el=11, scale=174, target=(-18, 30, -62),
                           projection="perspective", focal_length_mm=82, f_stop=18,
                           title="REACH-2 v4 DRIVE"),
    }

    metadata = dict(old.metadata)
    metadata.update(
        concept="CYBR REACH-2 v4 ORBIT-family engineered elbow",
        cad_revision="REACH-2 v4 ORBIT-family packaging",
        aesthetic_family="CYBR ORBIT",
        packaging_constraints={
            "fixed_silhouette_max_mm": MAX_FIXED_SILHOUETTE_MM,
            "web_thickness_mm": WEB_THICKNESS_MM,
            "ring_radial_section_mm": RING_RADIAL_SECTION_MM,
            "rib_min_width_mm": RIB_MIN_WIDTH_MM,
            "gusset_thickness_mm": GUSSET_THICKNESS_MM,
            "motor_adapter_od_mm": MOTOR_ADAPTER_OD_MM,
        },
        notes=list(old.metadata.get("notes", [])) + [
            "Primary drivetrain and moving load path are unchanged from qualified v3b.",
            "Fixed housing packaging is restyled as annular ORBIT-family pedestals and tapered ribs while preserving section minima.",
            "Teal parts are removable physical service retainers/caps, not render-only decoration.",
        ],
    )

    return Assembly("CYBR REACH-2 v4 + ORBIT", parts, mats, views, metadata, motion)


if __name__ == "__main__":
    a = build()
    print(a.name, len(a.parts), "parts", "ratio", RATIO)
