"""CYBR REACH-1: a two-stage geared elbow/forearm module for CYBR ORBIT.

The module bolts directly to ORBIT's existing B01 four-hole mounting shoe after
the removable non-marking feet are removed. REACH adds one pitch DOF around a
+Y elbow axis, a 16:1 two-stage involute spur reduction, a hollow output shaft,
and a lower four-bolt flange for the next arm segment.

This is geometric/kinematic concept CAD, not a qualified load-bearing actuator.
Native units are millimetres.
"""
from __future__ import annotations

import importlib.util
import math
from dataclasses import replace
from pathlib import Path

import cadquery as cq
import numpy as np

from cybrgeo.features import involute_spur_gear
from mechanism_lab.core import Assembly, View, cad_part

ROOT = Path(__file__).resolve().parents[1]
ORBIT_RECIPE = ROOT / "examples" / "orbit_inspection_wrist.py"

# Exact existing ORBIT B01 pattern. The 6.6 mm clearance bores are appropriate
# for a nominal M6 attachment and are not modified by REACH.
ORBIT_X = (-46.0, 2.0)
ORBIT_Y = (-45.0, 63.0)
ORBIT_PATTERN = tuple((x, y) for x in ORBIT_X for y in ORBIT_Y)
ORBIT_FASTENER_D = 6.0

PIVOT = np.array([-22.0, 9.0, -58.0])
INTERMEDIATE = np.array([13.0, 9.0, -58.0])
INPUT = np.array([13.0, 9.0, -88.0])

STAGE2_OUTPUT_TEETH = 56
STAGE2_PINION_TEETH = 14
STAGE1_GEAR_TEETH = 48
STAGE1_PINION_TEETH = 12
GEAR_MODULE = 1.0
REDUCTION = (STAGE2_OUTPUT_TEETH / STAGE2_PINION_TEETH) * (STAGE1_GEAR_TEETH / STAGE1_PINION_TEETH)
PERIOD = 10.0
MAX_ELBOW_DEG = 28.0


def load_orbit_recipe():
    spec = importlib.util.spec_from_file_location("reach_orbit_recipe", ORBIT_RECIPE)
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


def gear_y(teeth, module, width, x, y0, z, bore=0.0, phase=0.0):
    # CYBR GEO's gear primitive is authored about +X. +90 degrees around Z
    # rotates that shaft axis to +Y while retaining the generated involute flanks.
    g = involute_spur_gear(teeth, module, width, bore=bore, phase=phase).val()
    return g.rotate((0, 0, 0), (0, 0, 1), 90).translate((x, y0, z))


def plate_xz(points, y0, thickness, fillet=0.0):
    q = cq.Workplane("XZ", origin=(0, y0, 0)).polyline(points).close().extrude(thickness)
    if fillet:
        try:
            q = q.edges("|Y").fillet(fillet)
        except Exception:
            pass
    return q.val()


def socket_screw_z(x, y, z_bottom=-6.4, z_head=5.65, shaft_r=2.9, head_r=5.2, head_h=3.0):
    stem = cylinder(shaft_r, z_head - z_bottom, (x, y, z_bottom), "Z")
    head = cylinder(head_r, head_h, (x, y, z_head), "Z")
    socket = cq.Workplane("XY", origin=(x, y, z_head + head_h - .15)).polygon(6, 3.15).extrude(-1.8).val()
    return stem.fuse(head).cut(socket)


def fluted_knob_y(x, y0, z, radius=12.0, length=12.0, bore=4.1):
    q = annulus_y(radius, bore, y0, length, x, z)
    q = cq.Workplane(obj=q).edges().chamfer(.55).val()
    cuts = []
    for a in np.linspace(0, math.tau, 32, endpoint=False):
        cx = x + (radius + .25) * math.cos(a)
        cz = z + (radius + .25) * math.sin(a)
        cuts.append(cylinder(.72, length + 2, (cx, y0 - 1, cz), "Y"))
    return q.cut(cq.Compound.makeCompound(cuts))


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


def shaft_spin_y(angle, center):
    return rotation_y(angle, center)


def build():
    orbit_recipe = load_orbit_recipe()
    orbit = orbit_recipe.build()

    # B02 feet are removable bench accessories. Attachment mode removes them;
    # the original B01 shoe, holes, pedestal and every functional ORBIT part stay unchanged.
    orbit_source = [p for p in orbit.parts if not p.name.startswith("B02_")]
    orbit_original = {}
    parts = []
    for p in orbit_source:
        q = replace(p, name="O_" + p.name, group="orbit_" + p.group)
        orbit_original[q.name] = p
        parts.append(q)

    mats = list(orbit.materials)

    def add(name, shape, material=0, group="reach_fixed", role=""):
        print("CAD", name, flush=True)
        tiny = name.startswith(("R06_", "R07_", "R08_", "R19_"))
        p = cad_part(
            name, shape, material, tolerance=.020, angular=.12 if tiny else .055,
            analytic_normals=True, group=group, motion=group,
            role=role or name.replace("_", " "), provenance="designed-concept",
            finish_axis=(0., 1., 0.), finish_origin=tuple(shape.Center().toTuple()),
        )
        parts.append(p)
        return shape

    # --- exact ORBIT mounting interface ---------------------------------
    # B01 bottom is z=0. REACH saddle top is also z=0: actual face-to-face contact.
    saddle = box(76, 140, 6, (-22, 9, -3), 2.0)
    for x, y in ORBIT_PATTERN:
        saddle = saddle.cut(cylinder(5.1, 8, (x, y, -7), "Z"))
    add("R01_ORBIT_interface_saddle", saddle, 0, "reach_output", "Moving saddle matching ORBIT B01")

    for i, (x, y) in enumerate(ORBIT_PATTERN):
        insert = annulus_z(5.0, 3.05, -6.0, 6.0, x, y)
        add(f"R02_{i}_Steel_thread_insert", insert, 2, "reach_output", "Captive threaded-insert representation at ORBIT mounting station")
        add(f"R06_{i}_ORBIT_M6_socket_screw", socket_screw_z(x, y), 2, "reach_output", "M6-equivalent ORBIT attachment screw")

    # Moving yokes turn the entire ORBIT shoe about the elbow axis.
    yoke_profile = [(-60, -6), (16, -6), (10, -20), (1, -38), (-13, -52),
                    (-22, -66), (-31, -52), (-48, -38), (-55, -20)]
    for side, y0 in (("L", -51.0), ("R", 61.0)):
        yoke = plate_xz(yoke_profile, y0, 8.0, 2.0)
        yoke = yoke.cut(cylinder(8.15, 10, (PIVOT[0], y0 - 1, PIVOT[2]), "Y"))
        yoke = yoke.fuse(annulus_y(16.0, 8.15, y0, 8.0, PIVOT[0], PIVOT[2]))
        add(f"R03_{side}_Moving_yoke", yoke, 0, "reach_output", "Rigid saddle-to-elbow yoke")

    # Hollow shaft preserves a through path for future electrical/pneumatic service.
    add("R04_Hollow_output_shaft", annulus_y(8.0, 4.2, -56.0, 138.0, PIVOT[0], PIVOT[2]),
        2, "reach_output", "Hollow elbow output shaft")

    # --- fixed forearm clevis --------------------------------------------
    fixed_profile = [(-62, -46), (-49, -35), (-37, -52), (-31, -67),
                     (-41, -101), (-55, -126), (11, -126), (0, -99),
                     (-8, -70), (-13, -52), (-1, -35), (13, -46)]
    for side, y0 in (("L", -63.0), ("R", 72.0)):
        cheek = plate_xz(fixed_profile, y0, 7.0, 2.5)
        cheek = cheek.cut(cylinder(13.1, 9, (PIVOT[0], y0 - 1, PIVOT[2]), "Y"))
        add(f"R05_{side}_Fixed_clevis", cheek, 0, "reach_fixed", "Fixed elbow bearing clevis")

    for side, y0 in (("L", -63.0), ("R", 72.0)):
        add(f"R07_{side}_Bearing_outer_race", annulus_y(13.0, 10.1, y0, 7.0, PIVOT[0], PIVOT[2]), 2, "reach_fixed")
        add(f"R08_{side}_Bearing_inner_race", annulus_y(10.0, 8.1, y0, 7.0, PIVOT[0], PIVOT[2]), 2, "reach_output")

    # Lower flange deliberately exposes another simple four-bolt interface so
    # the next module can be an upper arm/shoulder without redesigning REACH.
    lower = box(82, 104, 8, (-22, 9, -130), 2.5)
    lower_pattern = [(-46, -23), (-46, 41), (2, -23), (2, 41)]
    for x, y in lower_pattern:
        lower = lower.cut(cylinder(3.3, 12, (x, y, -136), "Z"))
        lower = lower.cut(cylinder(6.0, 3.8, (x, y, -130), "Z"))
    add("R09_Lower_link_flange", lower, 0, "reach_fixed", "Interface for next upper-arm module")
    add("R10_Lower_crossbrace", box(58, 128, 10, (-22, 8.5, -112), 3), 0, "reach_fixed")
    add("R11_Cable_guard", annulus_y(9.5, 5.0, -48, 112, PIVOT[0], PIVOT[2]), 4, "reach_fixed", "Elastomer-lined hollow service passage")

    # --- two-stage 16:1 involute reducer ---------------------------------
    out_gear = gear_y(STAGE2_OUTPUT_TEETH, GEAR_MODULE, 8.0,
                      PIVOT[0], 82.0, PIVOT[2], bore=16.1,
                      phase=math.pi / STAGE2_OUTPUT_TEETH)
    int_pinion = gear_y(STAGE2_PINION_TEETH, GEAR_MODULE, 8.0,
                        INTERMEDIATE[0], 82.0, INTERMEDIATE[2], bore=8.1)
    add("R12_56T_Output_gear", out_gear, 1, "reach_output", "Elbow output gear")
    add("R13_14T_Intermediate_pinion", int_pinion, 2, "reach_intermediate", "Second-stage pinion")

    int_gear = gear_y(STAGE1_GEAR_TEETH, GEAR_MODULE, 8.0,
                      INTERMEDIATE[0], 94.0, INTERMEDIATE[2], bore=8.1,
                      phase=math.pi / STAGE1_GEAR_TEETH)
    input_pinion = gear_y(STAGE1_PINION_TEETH, GEAR_MODULE, 8.0,
                          INPUT[0], 94.0, INPUT[2], bore=6.1)
    add("R14_48T_Intermediate_gear", int_gear, 1, "reach_intermediate", "First-stage driven gear")
    add("R15_12T_Input_pinion", input_pinion, 2, "reach_input", "First-stage input pinion")

    add("R16_Intermediate_shaft", annulus_y(4.0, 0, 79.0, 27.0, INTERMEDIATE[0], INTERMEDIATE[2]),
        2, "reach_intermediate", "Compound reduction shaft")
    add("R17_Input_shaft", annulus_y(3.0, 0, 79.0, 36.0, INPUT[0], INPUT[2]),
        2, "reach_input", "Manual/motor-ready reduction input shaft")
    add("R18_Input_knob", fluted_knob_y(INPUT[0], 108.0, INPUT[2], 12.0, 12.0, 6.1),
        5, "reach_input", "Manual commissioning knob; replaceable by motor coupler")

    # Separate support sits immediately outside the fixed right cheek (72..79 mm)
    # and immediately inside the first gear stage (82..90 mm).
    support = plate_xz([(-7, -36), (30, -36), (34, -103), (-4, -103)], 79.2, 2.6, 1.0)
    support = support.cut(cylinder(4.2, 4.0, (INTERMEDIATE[0], 78.5, INTERMEDIATE[2]), "Y"))
    support = support.cut(cylinder(3.2, 4.0, (INPUT[0], 78.5, INPUT[2]), "Y"))
    add("R19_Service_side_shaft_support", support, 0, "reach_fixed", "Input/intermediate bearing support")

    guard = box(101, 4, 83, (-11, 108, -68), 3.0)
    guard = guard.cut(cylinder(30.5, 6, (PIVOT[0], 107, PIVOT[2]), "Y"))
    guard = guard.cut(cylinder(27.0, 6, (INTERMEDIATE[0], 107, INTERMEDIATE[2]), "Y"))
    guard = guard.cut(cylinder(14.0, 6, (INPUT[0], 107, INPUT[2]), "Y"))
    add("R20_Perforated_gear_guard", guard, 0, "reach_fixed", "Open inspection guard around reduction train")

    def motion(part, t, e):
        theta = elbow_angle(t)
        elbow_T = rotation_y(theta, PIVOT)
        if part.name.startswith("O_"):
            original = orbit_original[part.name]
            return elbow_T @ orbit.pose(original, t, 0.0)
        if part.group == "reach_output":
            return elbow_T
        if part.group == "reach_intermediate":
            return shaft_spin_y(-theta * STAGE2_OUTPUT_TEETH / STAGE2_PINION_TEETH, INTERMEDIATE)
        if part.group == "reach_input":
            return shaft_spin_y(theta * REDUCTION, INPUT)
        return np.eye(4)

    views = {
        "hero": View(az=34, el=20, scale=155, target=(-10, 8, -18),
                     projection="perspective", focal_length_mm=68, f_stop=16,
                     title="CYBR REACH-1 + ORBIT"),
        "gear_side": View(az=88, el=7, scale=145, target=(-8, 58, -55),
                          projection="perspective", focal_length_mm=76, f_stop=18,
                          title="REACH-1 16:1 REDUCTION"),
        "interface": View(az=-42, el=54, scale=120, target=(-22, 9, -20),
                          projection="perspective", focal_length_mm=72, f_stop=18,
                          title="ORBIT / REACH INTERFACE"),
    }

    metadata = dict(
        concept="CYBR REACH-1 modular elbow/forearm",
        orbit_interface_pattern_mm=[list(p) for p in ORBIT_PATTERN],
        orbit_fastener_nominal="M6",
        reduction_ratio=REDUCTION,
        stage_ratios=[STAGE1_GEAR_TEETH / STAGE1_PINION_TEETH,
                      STAGE2_OUTPUT_TEETH / STAGE2_PINION_TEETH],
        gear_center_distances_mm=[30.0, 35.0],
        elbow_range_degrees=[-MAX_ELBOW_DEG, MAX_ELBOW_DEG],
        output_axis="+Y",
        lower_interface_pattern_mm=[[x, y] for x, y in lower_pattern],
        notes=[
            "ORBIT B02 non-marking bench feet are removed in attached configuration.",
            "Gear root transitions remain visualization geometry, not cutter-certified roots.",
            "Threaded inserts are geometric envelopes; thread form is not modeled.",
            "No motor, torque, bearing life, preload, tolerance-stack, or structural certification claim is made.",
        ],
    )

    return Assembly("CYBR REACH-1 + ORBIT", parts, mats, views, metadata, motion)


if __name__ == "__main__":
    a = build()
    print(a.name, len(a.parts), "parts", "ratio", REDUCTION)
