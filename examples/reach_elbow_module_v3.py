"""CYBR REACH-1 v3: complete the visible gearbox cassette and service-side support.

Builds on the validated v2 completion pass. v3 adds the major structure that was
still missing visually/mechanically: a second shaft-support bridge, outboard
plain bearings, perimeter standoffs, output-shaft extension/end-cap, motor-ready
input adapter, and hard-stop bumpers. This removes the cantilevered-gear look.

Concept CAD only; no torque, bearing-life, motor-sizing or structural certification.
"""
from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import cadquery as cq

from mechanism_lab.core import Assembly, cad_part

ROOT = Path(__file__).resolve().parents[1]
V2_FIXED = ROOT / "examples" / "reach_elbow_module_v2_fixed.py"


def load_v2():
    spec = importlib.util.spec_from_file_location("reach_v2_fixed_base", V2_FIXED)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def cylinder(radius, length, origin, axis="Y"):
    direction = {"X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1)}[axis]
    return cq.Solid.makeCylinder(radius, length, cq.Vector(*origin), cq.Vector(*direction))


def annulus_y(ro, ri, y0, length, x, z):
    outer = cylinder(ro, length, (x, y0, z), "Y")
    if not ri:
        return outer
    return outer.cut(cylinder(ri, length + 2, (x, y0 - 1, z), "Y"))


def box(dx, dy, dz, center, fillet=0.0):
    q = cq.Workplane("XY").box(dx, dy, dz)
    if fillet:
        q = q.edges().fillet(fillet)
    return q.val().translate(tuple(center))


def build():
    v2 = load_v2()
    a = v2.build()
    base = v2.load_base()
    parts = list(a.parts)

    def add(name, shape, material=0, group="reach_fixed", role="", tiny=False):
        print("CAD", name, flush=True)
        p = cad_part(
            name, shape, material,
            tolerance=.020, angular=.12 if tiny else .055,
            analytic_normals=True,
            group=group, motion=group,
            role=role or name.replace("_", " "),
            provenance="designed-concept",
            finish_axis=(0., 1., 0.),
            finish_origin=tuple(shape.Center().toTuple()),
        )
        parts.append(p)
        return p

    px, _, pz = map(float, base.PIVOT)
    ix, _, iz = map(float, base.INTERMEDIATE)
    qx, _, qz = map(float, base.INPUT)

    # --- second-side gearbox support ------------------------------------
    # A narrow vertical bridge supports both reduction shafts without hiding
    # the exposed gear meshes. It sits beyond the gear planes (gears end y=102).
    bridge = box(22.0, 3.0, 72.0, (13.0, 104.5, -73.0), 1.3)
    bridge = bridge.cut(cylinder(4.25, 5.0, (ix, 102.5, iz), "Y"))
    bridge = bridge.cut(cylinder(3.25, 5.0, (qx, 102.5, qz), "Y"))
    add("R40_Outboard_bearing_bridge", bridge, 0, "reach_fixed",
        "Outboard bridge supporting both reduction shafts")

    add("R41_Intermediate_outboard_bearing",
        annulus_y(6.6, 4.08, 103.0, 3.0, ix, iz),
        3, "reach_fixed", "Outboard bronze bearing for compound shaft")
    add("R42_Input_outboard_bearing",
        annulus_y(5.8, 3.08, 103.0, 3.0, qx, qz),
        3, "reach_fixed", "Outboard bronze bearing for input shaft")

    # Four perimeter standoffs connect the inner service support/cage to the
    # outboard bridge plane and make the gearbox a real cassette rather than a
    # collection of cantilevered gears.
    for i, (x, z) in enumerate(((-42.0, -35.0), (31.0, -35.0), (-42.0, -104.0), (31.0, -104.0))):
        rod = cylinder(2.6, 23.0, (x, 81.0, z), "Y")
        add(f"R43_{i}_Gearbox_standoff", rod, 2, "reach_fixed",
            "Gearbox cassette standoff", tiny=True)
        # Thin washer at the outboard end gives the standoff a readable seat.
        add(f"R44_{i}_Standoff_washer", annulus_y(4.2, 2.65, 103.5, 1.0, x, z),
            1, "reach_fixed", "Outboard standoff washer", tiny=True)

    # --- output shaft closure -------------------------------------------
    # v1's output shaft ends at the output gear. Extend the hollow service path
    # just beyond the gear and retain it with a visible machined end cap.
    add("R45_Output_shaft_extension",
        annulus_y(8.0, 4.2, 90.0, 7.0, px, pz),
        2, "reach_output", "Hollow output-shaft service extension")
    add("R46_Output_shaft_endcap",
        annulus_y(10.2, 4.2, 97.0, 2.0, px, pz),
        1, "reach_output", "Machined hollow output-shaft end cap")

    # --- motor/encoder-ready input interface -----------------------------
    # A compact square adapter sits outside the outboard input bearing. The
    # existing green commissioning knob remains usable through its central bore.
    adapter = box(30.0, 2.5, 30.0, (qx, 107.25, qz), 2.0)
    adapter = adapter.cut(cylinder(6.4, 4.5, (qx, 105.0, qz), "Y"))
    for dx in (-9.0, 9.0):
        for dz in (-9.0, 9.0):
            adapter = adapter.cut(cylinder(1.65, 4.5, (qx + dx, 105.0, qz + dz), "Y"))
    add("R47_Motor_encoder_adapter", adapter, 0, "reach_fixed",
        "Motor/encoder-ready four-hole input adapter")

    for i, (dx, dz) in enumerate(((-9, -9), (-9, 9), (9, -9), (9, 9))):
        add(f"R48_{i}_Adapter_thread_insert",
            annulus_y(2.6, 1.7, 106.0, 2.0, qx + dx, qz + dz),
            2, "reach_fixed", "Steel insert for future motor/encoder adapter", tiny=True)

    # Short coupling sleeve between adapter and the manual knob; replaceable by
    # an actual motor coupling without changing the gearbox shafts.
    add("R49_Input_coupling_sleeve",
        annulus_y(6.0, 3.08, 106.0, 2.0, qx, qz),
        5, "reach_input", "Replaceable elastomer/metal input coupling sleeve")

    # --- hard stops and lower structure readability ---------------------
    for side, x in (("A", -51.0), ("B", 7.0)):
        bumper = cylinder(4.5, 8.0, (x, -4.0, -76.0), "Y")
        add(f"R50_{side}_Elbow_hard_stop", bumper, 4, "reach_fixed",
            "Elastomer elbow hard-stop bumper")

    # Central lower spine closes the load path from crossbrace to the next-link
    # flange without hiding either side clevis.
    spine = box(18.0, 42.0, 26.0, (-22.0, 8.5, -121.0), 2.0)
    add("R51_Lower_center_spine", spine, 0, "reach_fixed",
        "Central lower spine tying crossbrace into lower modular flange")

    metadata = dict(a.metadata)
    metadata.update(
        revision="REACH-1 v3 dual-sided gearbox cassette",
        added_v3_parts=len(parts) - len(a.parts),
        gearbox_support="inner + outboard bearing support with four perimeter standoffs",
        input_interface="manual commissioning knob plus motor/encoder-ready 4-hole adapter",
    )

    return Assembly(
        "CYBR REACH-1 v3 + ORBIT",
        parts,
        list(a.materials),
        dict(a.views),
        metadata,
        a.motion_function,
    )


if __name__ == "__main__":
    assembly = build()
    print(assembly.name, len(assembly.parts), "parts")
