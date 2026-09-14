"""CYBR REACH-1 v2: mechanically completed integration hardware for ORBIT.

This layer keeps the validated REACH-1 kinematic architecture and ORBIT interface,
then adds the hardware that makes the elbow read and assemble like a finished
mechanism: thrust spacers, bearing retainers, shaft collars, gearbox spacers,
clevis/crossbrace fasteners, saddle reinforcement and a service gland.

Native units are millimetres. This remains concept CAD, not a load/torque/life
qualification or manufacturing drawing set.
"""
from __future__ import annotations

import importlib.util
import math
from dataclasses import replace
from pathlib import Path

import cadquery as cq
import numpy as np

from mechanism_lab.core import Assembly, cad_part

ROOT = Path(__file__).resolve().parents[1]
BASE_RECIPE = ROOT / "examples" / "reach_elbow_module.py"


def load_base():
    spec = importlib.util.spec_from_file_location("reach_v1_base", BASE_RECIPE)
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


def socket_screw_y(x, y_start, z, length, direction=1, shaft_r=2.15, head_r=4.25, head_h=2.8):
    """Simple socket-head service fastener along +/-Y."""
    if direction > 0:
        stem = cylinder(shaft_r, length, (x, y_start, z), "Y")
        head = cylinder(head_r, head_h, (x, y_start - head_h, z), "Y")
        socket_plane_y = y_start - head_h - .05
        socket = cq.Workplane("XZ", origin=(x, socket_plane_y, z)).polygon(6, 2.45).extrude(head_h * .72).val()
    else:
        stem = cylinder(shaft_r, length, (x, y_start, z), "Y").rotate((x, y_start, z), (1, 0, 0), 180)
        # Author head from the opposite end directly so the body remains simple.
        head = cylinder(head_r, head_h, (x, y_start, z), "Y")
        socket = cq.Workplane("XZ", origin=(x, y_start + head_h + .05, z)).polygon(6, 2.45).extrude(-head_h * .72).val()
    return stem.fuse(head).cut(socket)


def build():
    b = load_base()
    a = b.build()
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

    px, py, pz = map(float, b.PIVOT)
    ix, iy, iz = map(float, b.INTERMEDIATE)
    qx, qy, qz = map(float, b.INPUT)

    # --- elbow bearing stack completion ---------------------------------
    # These occupy the intentional axial gaps between moving yokes and fixed
    # clevis cheeks. They prevent the joint from visually/mechanically reading
    # as a shaft floating between two unrelated plates.
    add("R21_L_Output_thrust_spacer",
        annulus_y(10.0, 8.18, -56.0, 5.0, px, pz),
        3, "reach_output", "Bronze thrust spacer between left cheek and moving yoke")
    add("R21_R_Output_thrust_spacer",
        annulus_y(10.0, 8.18, 69.0, 3.0, px, pz),
        3, "reach_output", "Bronze thrust spacer between right yoke and fixed cheek")

    # Thin retainers outside the fixed bearing cheeks. Right retainer ends at
    # y=81, leaving a 1 mm service gap before the output gear begins at y=82.
    add("R22_L_Bearing_retainer",
        annulus_y(15.0, 10.2, -65.0, 2.0, px, pz),
        1, "reach_fixed", "Machined bearing retainer ring, left")
    add("R22_R_Bearing_retainer",
        annulus_y(15.0, 10.2, 79.0, 2.0, px, pz),
        1, "reach_fixed", "Machined bearing retainer ring, right")

    # Retainer screws on a 12 mm radius bolt circle. These are deliberately
    # small and sit outside the shaft/bearing bore.
    for side, y0, direction in (("L", -67.2, 1), ("R", 81.0, -1)):
        for j, ang in enumerate((45, 135, 225, 315)):
            rad = math.radians(ang)
            x = px + 12.0 * math.cos(rad)
            z = pz + 12.0 * math.sin(rad)
            if side == "L":
                screw = socket_screw_y(x, y0, z, 4.2, direction=1, shaft_r=1.25, head_r=2.4, head_h=1.7)
            else:
                # Right-side screw is a simple inward-facing stem + head.
                stem = cylinder(1.25, 4.2, (x, 76.8, z), "Y")
                head = cylinder(2.4, 1.7, (x, 81.0, z), "Y")
                screw = stem.fuse(head)
            add(f"R23_{side}_{j}_Retainer_screw", screw, 2, "reach_fixed",
                "Bearing retainer socket screw", tiny=True)

    # --- gearbox shaft location -----------------------------------------
    # Axial spacers and collars locate the compound and input shafts between
    # the two gear planes and the service-side support.
    add("R24_Intermediate_stage_spacer",
        annulus_y(6.0, 4.08, 90.0, 4.0, ix, iz),
        3, "reach_intermediate", "Spacer between 14T and 48T gears")
    add("R25_Intermediate_retaining_collar",
        annulus_y(6.8, 4.08, 102.0, 3.2, ix, iz),
        2, "reach_intermediate", "Outer retaining collar on compound shaft")
    add("R26_Input_gear_spacer",
        annulus_y(5.4, 3.08, 102.0, 5.0, qx, qz),
        3, "reach_input", "Spacer between input pinion and hand/motor coupler")
    add("R27_Input_retaining_collar",
        annulus_y(6.2, 3.08, 107.0, 1.0, qx, qz),
        2, "reach_input", "Input shaft retaining collar")

    # Visible support bushings directly behind the first gear plane. These sit
    # between support face and gear without penetrating the support solid.
    add("R28_Intermediate_support_bushing",
        annulus_y(5.8, 4.08, 81.8, .18, ix, iz),
        3, "reach_fixed", "Service-side compound-shaft bushing shoulder", tiny=True)
    add("R29_Input_support_bushing",
        annulus_y(5.1, 3.08, 81.8, .18, qx, qz),
        3, "reach_fixed", "Service-side input-shaft bushing shoulder", tiny=True)

    # --- structural completion ------------------------------------------
    # Crossbrace already spans between the fixed cheeks. Add real side screws
    # so the lower structure does not read as three unconnected plates.
    for side, y0, inward in (("L", -66.5, 1), ("R", 74.0, -1)):
        for j, x in enumerate((-42.0, -2.0)):
            if side == "L":
                screw = socket_screw_y(x, y0, -112.0, 13.0, direction=1)
            else:
                stem = cylinder(2.15, 13.0, (x, 61.0, -112.0), "Y")
                head = cylinder(4.25, 2.8, (x, 74.0, -112.0), "Y")
                screw = stem.fuse(head)
            add(f"R30_{side}_{j}_Crossbrace_socket_screw", screw, 2, "reach_fixed",
                "Clevis-to-crossbrace socket screw", tiny=True)

    # Compact saddle ribs make the load path from ORBIT's mounting shoe into
    # the moving yokes visually explicit. They fit beneath the saddle and do
    # not alter ORBIT itself.
    for side, y in (("L", -47.0), ("R", 65.0)):
        rib = box(42.0, 4.0, 16.0, (-22.0, y, -14.0), 1.2)
        # Taper the rib with a diagonal cut so it reads like a machined gusset.
        cutter = cq.Workplane("XZ", origin=(0, y - 2.1, 0)).polyline([
            (-45, -24), (4, -24), (4, -5), (-8, -5)
        ]).close().extrude(4.2).val()
        rib = rib.intersect(cutter)
        add(f"R31_{side}_Saddle_gusset", rib, 0, "reach_output",
            "Saddle-to-moving-yoke reinforcement gusset")

    # Elastomer service gland at the exposed hollow-shaft end makes the cable
    # path obvious while preserving the 8.4 mm through-bore.
    add("R32_Service_gland",
        annulus_y(10.5, 4.25, -70.0, 4.0, px, pz),
        4, "reach_output", "Elastomer service gland on hollow elbow shaft")

    # A pair of lower flange gussets closes the visual load path into the next
    # module interface. They are fixed with REACH, not part of ORBIT.
    for side, y in (("L", -38.0), ("R", 56.0)):
        g = cq.Workplane("XZ", origin=(0, y, 0)).polyline([
            (-48, -126), (-18, -126), (-30, -100)
        ]).close().extrude(5.0).val()
        add(f"R33_{side}_Lower_flange_gusset", g, 0, "reach_fixed",
            "Lower flange reinforcement gusset")

    metadata = dict(a.metadata)
    metadata.update(
        revision="REACH-1 v2 integration-complete visual/mechanical pass",
        added_integration_parts=len(parts) - len(a.parts),
        notes=list(metadata.get("notes", [])) + [
            "v2 adds bearing retainers, thrust spacers, shaft collars, structural fasteners, saddle/lower gussets and a service gland.",
            "These parts complete visible load paths and service hardware; they do not constitute torque/life certification.",
        ],
    )

    views = dict(a.views)
    if "hero" in views:
        views["hero"] = replace(views["hero"], az=56, el=17, scale=160,
                                target=(-12, 12, -22), focal_length_mm=72,
                                title="CYBR REACH-1 v2 + ORBIT")

    return Assembly(
        "CYBR REACH-1 v2 + ORBIT",
        parts,
        list(a.materials),
        views,
        metadata,
        a.motion_function,
    )


if __name__ == "__main__":
    assembly = build()
    print(assembly.name, len(assembly.parts), "parts")
