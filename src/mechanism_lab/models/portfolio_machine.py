"""Procedural portfolio-machine modules derived from the 2026-10-04 concept sheets.

The source images are design references, not geometry inputs. Every visible part here is
re-authored as analytic CAD or generated tube geometry so the six portfolio objects can
be exported, inspected, exploded and rendered through the normal CYBR GEO pipeline.

Reference targets:
- CYBR SCENES: glass habitat dome over a mechanical rotation/service base.
- CYBR GEO: large exposed electromechanical motor/generator, copper windings visible.
- CYBR LIGHT: optical bench with emitter, prism, steering mirror and output lens.
- CYBR ELEMENTS: 820 mm containment cube with rock, fire/lava, water and steam.
- CYBR MATERIALS: 980 x 640 mm sample machine with stepped wood/stone panels and arm.
- CYBR FOREST: sealed biome dome with terrain, trees, stream/waterfall and life support.

Dimensions are concept dimensions taken from the generated reference sheets where shown;
they are not claims about real manufactured hardware.
"""
from __future__ import annotations

import math
from dataclasses import replace
import numpy as np
import cadquery as cq

from ..core import Assembly, Material, Part, View, cad_part, mesh_part, axis_pose
from ..geometry import ring, bolt_circle, tube_mesh


REFERENCE_ID = "portfolio-reference-sheets-2026-10-04"


MATERIALS = [
    Material("Black oxide steel", (.030, .034, .041), .86, .28, coat=.12, coat_rough=.20,
             microfinish="fine-machined", material_source="designed-concept"),
    Material("Gunmetal", (.095, .105, .118), .92, .25, coat=.10, coat_rough=.18,
             microfinish="fine-machined", material_source="designed-concept"),
    Material("Machined aluminium", (.52, .56, .61), .96, .22, anisotropy=.20,
             microfinish="radial-machined", material_source="designed-concept"),
    Material("Copper winding", (.76, .29, .075), .88, .20, coat=.18, coat_rough=.14,
             microfinish="drawn-wire", material_source="designed-concept"),
    Material("Warm brass", (.54, .32, .11), .92, .24, anisotropy=.12,
             microfinish="brushed", material_source="designed-concept"),
    Material("Optical glass", (.78, .88, .98), .0, .035, ior=1.49, coat=.55,
             coat_rough=.025, opacity=.18, material_source="designed-concept"),
    Material("Water", (.045, .29, .43), .0, .055, ior=1.333, coat=.25,
             coat_rough=.03, opacity=.42, material_source="designed-concept"),
    Material("Hot element", (1.0, .21, .025), .12, .18, coat=.05, coat_rough=.12,
             material_source="designed-concept"),
    Material("Basalt", (.055, .060, .066), .08, .72,
             microfinish="fractured", material_source="designed-concept"),
    Material("Oak", (.43, .24, .105), .0, .42, anisotropy=.34,
             microfinish="wood-grain", material_source="designed-concept"),
    Material("Maple", (.72, .57, .37), .0, .37, anisotropy=.28,
             microfinish="wood-grain", material_source="designed-concept"),
    Material("Stone", (.43, .42, .40), .02, .48,
             microfinish="stone", material_source="designed-concept"),
    Material("Leaf", (.035, .18, .065), .0, .66,
             microfinish="organic", material_source="designed-concept"),
    Material("Moss", (.075, .23, .075), .0, .84,
             microfinish="organic", material_source="designed-concept"),
    Material("Laser blue", (.16, .46, 1.0), .0, .06, ior=1.01, coat=.08,
             coat_rough=.04, opacity=.78, material_source="designed-concept"),
    Material("Steam", (.68, .72, .76), .0, .92, ior=1.0003, opacity=.18,
             material_source="designed-concept"),
    Material("Rubber", (.012, .014, .017), .0, .82,
             microfinish="rubber", material_source="designed-concept"),
    Material("White ceramic", (.72, .72, .69), .0, .31, coat=.08,
             coat_rough=.12, material_source="designed-concept"),
]


def _box(size, center=(0, 0, 0), fillet_radius=0.0):
    sx, sy, sz = map(float, size)
    shape = cq.Workplane("XY").box(sx, sy, sz).translate(tuple(map(float, center)))
    if fillet_radius > 0:
        try:
            shape = shape.edges().fillet(float(fillet_radius))
        except Exception:
            pass
    return shape.val()


def _cyl_z(radius, height, center=(0, 0, 0)):
    x, y, z = map(float, center)
    return cq.Solid.makeCylinder(float(radius), float(height), cq.Vector(x, y, z - height / 2.0), cq.Vector(0, 0, 1))


def _sphere(radius, center=(0, 0, 0)):
    return cq.Solid.makeSphere(float(radius), cq.Vector(*map(float, center)))


def _hemisphere(radius, center=(0, 0, 0)):
    return cq.Solid.makeSphere(
        float(radius),
        cq.Vector(*map(float, center)),
        cq.Vector(0, 0, 1),
        0.0,
        90.0,
        360.0,
    )


def _cylinder_between(a, b, radius):
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    d = b - a
    length = float(np.linalg.norm(d))
    if length <= 1e-7:
        raise ValueError("zero-length cylinder")
    return cq.Solid.makeCylinder(float(radius), length, cq.Vector(*a), cq.Vector(*(d / length)))


def _part(parts, name, shape, material, group, *, role="", explode=(0, 0, 0), motion="fixed",
          provenance="designed-concept", tags=(), tolerance=.14, angular=.10):
    p = cad_part(
        name,
        shape,
        material,
        tolerance=tolerance,
        angular=angular,
        group=group,
        motion=motion,
        explode=np.asarray(explode, float),
        role=role,
        provenance=provenance,
        tags=tuple(tags),
    )
    parts.append(p)
    return p


def _tube(parts, name, points, radius, material, group, *, role="", explode=(0, 0, 0), tags=()):
    v, f = tube_mesh(points, radius=radius, sides=10)
    p = mesh_part(
        name,
        v,
        f,
        material,
        group=group,
        explode=np.asarray(explode, float),
        role=role,
        provenance="designed-concept",
        tags=tuple(tags),
    )
    parts.append(p)
    return p


def _fastener_ring(parts, prefix, radius, z, count, material=4, screw_r=4.0, screw_h=6.0, group="fasteners"):
    for i, (x, y) in enumerate(bolt_circle(radius, count)):
        _part(
            parts,
            f"{prefix}_{i+1:02}",
            _cyl_z(screw_r, screw_h, (x, y, z)),
            material,
            group,
            role="Visible service fastener from concept reference",
            explode=(0, 0, 14),
        )


def _base_module(parts, prefix, width, depth, height, z=0.0, label_index=None):
    _part(parts, f"{prefix}_Base_shell", _box((width, depth, height), (0, 0, z + height / 2), 16),
          0, "base", role="Black machined portfolio-machine base", explode=(0, 0, -60))
    _part(parts, f"{prefix}_Base_cap", _box((width * .92, depth * .92, 12), (0, 0, z + height + 6), 5),
          1, "base", role="Upper service cap", explode=(0, 0, -35))
    leg_x = width * .39
    leg_y = depth * .39
    for ix in (-1, 1):
        for iy in (-1, 1):
            _part(parts, f"{prefix}_Foot_{ix}_{iy}",
                  _cyl_z(18, 28, (ix * leg_x, iy * leg_y, z - 14)),
                  4, "base", role="Isolation foot", explode=(0, 0, -90))
    if label_index is not None:
        plate = _box((150, 12, 72), (0, -depth / 2 - 8, z + height * .42), 5)
        _part(parts, f"{prefix}_Index_plate_{label_index:02}", plate, 1, "markings",
              role=f"Module index {label_index:02}", explode=(0, -35, 0))


def _rail_pair(parts, prefix, length, span, z, x_center=0.0):
    for y in (-span / 2, span / 2):
        _part(parts, f"{prefix}_Rail_{'L' if y < 0 else 'R'}",
              _box((length, 26, 28), (x_center, y, z), 5),
              2, "rails", role="Precision linear rail", explode=(0, y * .12, -20))
        _part(parts, f"{prefix}_Rail_insert_{'L' if y < 0 else 'R'}",
              _box((length - 44, 8, 7), (x_center, y, z + 18), 2),
              4, "rails", role="Brass wear strip", explode=(0, y * .12, -15))


def _service_lines(parts, prefix, starts, end_x, material=4, z_offset=0):
    for i, p in enumerate(starts):
        x, y, z = p
        pts = np.array([
            [x, y, z],
            [x - 40, y, z + z_offset],
            [end_x + 55, y * .92, z + z_offset],
            [end_x, y * .82, z + z_offset],
        ], float)
        _tube(parts, f"{prefix}_Service_line_{i+1:02}", pts, 5.0, material, "service",
              role="Power / coolant / data service routing", explode=(0, 0, -30))


def _concept_metadata(name, dimensions, reference_notes):
    return {
        "truth_intent": "concept",
        "fidelity": "Procedurally re-authored from generated portfolio concept reference sheet",
        "reference_id": REFERENCE_ID,
        "reference_notes": list(reference_notes),
        "concept_dimensions_mm": dict(dimensions),
        "sources": [],
        "assumptions": [
            "Reference sheets are visual design targets, not fabrication drawings.",
            "Fastener thread forms, hidden wiring, seals, bearing fits and wall thicknesses are illustrative.",
            "Optical, fluid, thermal and ecosystem behavior is represented geometrically; no engineering qualification is implied.",
        ],
    }


def build_scenes() -> Assembly:
    parts = []
    base_d = 1120.0
    base_h = 190.0
    _base_module(parts, "SC", base_d, base_d, base_h, label_index=1)

    _part(parts, "SC_Rotation_ring_outer", ring(505, 438, base_h, base_h + 70), 1, "mechanism",
          role="Large mechanical rotation ring", explode=(0, 0, 90))
    _part(parts, "SC_Rotation_ring_inner", ring(430, 392, base_h + 12, base_h + 58), 4, "mechanism",
          role="Bronze inner guide ring", explode=(0, 0, 105))
    _fastener_ring(parts, "SC_Ring_bolt", 470, base_h + 72, 20, screw_r=5.5, screw_h=9)

    dome_r = 430.0
    dome_center_z = base_h + 74
    _part(parts, "SC_Glass_dome", _hemisphere(dome_r, (0, 0, dome_center_z)), 5, "glass",
          role="Optical glass habitat dome from reference silhouette", explode=(0, 0, 145), tolerance=.32, angular=.20)

    _part(parts, "SC_Environment_floor", _cyl_z(386, 28, (0, 0, dome_center_z + 16)), 8, "environment",
          role="Terrain substrate", explode=(0, 0, 125))

    mesa_specs = [
        (-210, -105, 82, 130, 175), (-170, 120, 66, 102, 138), (185, -82, 95, 142, 215),
        (222, 105, 54, 86, 118), (32, 165, 48, 70, 92), (-20, -185, 38, 58, 65),
    ]
    for i, (x, y, r, h1, h2) in enumerate(mesa_specs):
        low = _cyl_z(r, h1, (x, y, dome_center_z + 28 + h1 / 2))
        high = _cyl_z(r * .62, h2, (x + r * .04, y - r * .03, dome_center_z + 28 + h1 + h2 / 2))
        _part(parts, f"SC_Mesa_{i+1:02}", cq.Compound.makeCompound([low, high]), 11, "environment",
              role="Stylized desert mesa", explode=(0, 0, 120))

    water = _cyl_z(250, 10, (18, -12, dome_center_z + 48))
    _part(parts, "SC_Water_basin", water, 6, "environment",
          role="Central turquoise water basin", explode=(0, 0, 118))

    for i, (x, y, h) in enumerate([(-125, -35, 54), (118, 22, 44), (55, -148, 36), (-42, 126, 42)]):
        _part(parts, f"SC_Tree_trunk_{i+1:02}", _cyl_z(7, h, (x, y, dome_center_z + 55 + h / 2)), 9, "environment",
              role="Miniature tree trunk", explode=(0, 0, 120))
        _part(parts, f"SC_Tree_crown_{i+1:02}", _sphere(24 + i * 2, (x, y, dome_center_z + 55 + h + 12)), 12, "environment",
              role="Miniature tree crown", explode=(0, 0, 120))

    arm_points = [
        ((-470, 330, 150), (-520, 330, 500), (-350, 250, 745)),
        ((470, -330, 150), (520, -330, 500), (360, -260, 740)),
    ]
    for j, chain in enumerate(arm_points):
        a, b, c = chain
        _part(parts, f"SC_Arm_{j+1}_lower", _cylinder_between(a, b, 24), 0, "support",
              role="Articulated dome support arm", explode=(0, (-1 if j == 0 else 1) * 45, 0))
        _part(parts, f"SC_Arm_{j+1}_upper", _cylinder_between(b, c, 24), 0, "support",
              role="Articulated dome support arm", explode=(0, (-1 if j == 0 else 1) * 45, 20))
        _part(parts, f"SC_Arm_{j+1}_joint_a", _sphere(42, a), 1, "support", role="Support pivot")
        _part(parts, f"SC_Arm_{j+1}_joint_b", _sphere(42, b), 1, "support", role="Support pivot")
        _part(parts, f"SC_Arm_{j+1}_joint_c", _sphere(34, c), 4, "support", role="Dome clamp pivot")

    _service_lines(parts, "SC", [(520, 240, 125), (520, 180, 112), (520, 120, 100)], 355, z_offset=8)

    views = {
        "hero": View(az=42, el=22, scale=760, target=(0, 0, 360), title="CYBR SCENES / HABITAT DOME",
                     note="Contained procedural world inside a mechanical service platform.",
                     f_stop=12, environment_strength=.32, light_size=2.0),
        "front": View(az=0, el=10, scale=690, target=(0, 0, 360), projection="orthographic",
                      title="CYBR SCENES / FRONT"),
        "side": View(az=90, el=10, scale=690, target=(0, 0, 360), projection="orthographic",
                     title="CYBR SCENES / SIDE"),
        "top": View(az=0, el=89, scale=690, target=(0, 0, 300), projection="orthographic",
                    title="CYBR SCENES / TOP"),
        "exploded": View(az=44, el=24, scale=900, target=(0, 0, 380), explode=1.0,
                         title="CYBR SCENES / EXPLODED REFERENCE"),
    }
    return Assembly(
        "portfolio_scenes",
        parts,
        MATERIALS,
        views,
        metadata=_concept_metadata(
            "portfolio_scenes",
            {"diameter": base_d, "height": 865.0, "dome_radius": dome_r},
            ["Large clear dome, black annular base, paired articulated support arms, desert miniature with water."],
        ),
    )


def _geo_pose(part, t, explode):
    angle = 0.0
    if part.motion == "rotor":
        angle = t * math.tau * .11
    return axis_pose(angle, part.center, part.explode, explode)


def build_geo() -> Assembly:
    parts = []
    length = 1280.0
    body_x0 = -310.0
    body_x1 = 310.0
    radius = 270.0

    _base_module(parts, "GE", 1180, 610, 120, z=-340, label_index=2)
    _rail_pair(parts, "GE", 1110, 430, -250, 0)

    for x, ro, ri, mat in [
        (-330, 285, 190, 0), (-292, 270, 198, 1), (292, 270, 198, 1), (330, 285, 190, 0)
    ]:
        _part(parts, f"GE_End_ring_{x:+.0f}", ring(ro, ri, x - 24, x + 24), mat, "housing",
              role="Layered generator end-bell ring", explode=((x / abs(x)) * 150 if x else 0, 0, 0))

    _part(parts, "GE_Stator_shell", ring(radius, 214, body_x0, body_x1), 0, "housing",
          role="Open stator shell exposing copper bars", explode=(0, 0, 95))

    for i in range(24):
        a = math.tau * i / 24
        y = 238 * math.cos(a)
        z = 238 * math.sin(a)
        bar = _box((520, 22, 24), (0, 238, 0), 6).rotate((0, 0, 0), (1, 0, 0), math.degrees(a))
        _part(parts, f"GE_Copper_bar_{i+1:02}", bar, 3, "windings",
              role="Visible concept winding bar", explode=(0, 42 * math.cos(a), 42 * math.sin(a)))

    _part(parts, "GE_Rotor_core", ring(180, 46, -295, 295), 2, "rotor", motion="rotor",
          role="Machined rotor core", explode=(95, 0, 0))
    _part(parts, "GE_Rotor_shaft", ring(52, 0, -515, 515), 2, "rotor", motion="rotor",
          role="Hardened drive shaft", explode=(185, 0, 0))

    for i, x in enumerate((-430, 430)):
        _part(parts, f"GE_Bearing_housing_{i+1}", ring(122, 54, x - 62, x + 62), 1, "bearings",
              role="Oversized bearing housing", explode=((i * 2 - 1) * 220, 0, 0))
        _part(parts, f"GE_Bearing_inner_{i+1}", ring(72, 53, x - 34, x + 34), 2, "bearings",
              role="Bearing inner race", motion="rotor", explode=((i * 2 - 1) * 245, 0, 0))

    for i, a in enumerate(np.linspace(0, math.tau, 8, endpoint=False)):
        y = 315 * math.cos(a)
        z = 315 * math.sin(a)
        a0 = (-305, y, z)
        a1 = (305, y, z)
        _part(parts, f"GE_Tie_rod_{i+1:02}", _cylinder_between(a0, a1, 7), 4, "structure",
              role="Axial housing tie rod", explode=(0, 34 * math.cos(a), 34 * math.sin(a)))

    for side, x in [("L", -505), ("R", 505)]:
        _part(parts, f"GE_Shaft_collar_{side}", ring(78, 53, x - 22, x + 22), 4, "rotor",
              motion="rotor", role="External shaft collar", explode=((1 if x > 0 else -1) * 270, 0, 0))

    for x in (-370, 370):
        for y in (-245, 245):
            _part(parts, f"GE_Mount_{x}_{y}", _box((160, 72, 90), (x, y, -252), 10), 0, "mounts",
                  role="Vibration-isolated generator mount", explode=(0, y * .20, -80))

    _service_lines(parts, "GE", [(285, -285, -90), (245, -305, -120), (200, -320, -145)], 430, material=4, z_offset=-15)

    views = {
        "hero": View(az=38, el=18, scale=610, target=(0, 0, -20), title="CYBR GEO / ELECTROMECHANICAL CORE",
                     note="1280 mm concept envelope. Exposed copper stator and layered bearing housings.",
                     f_stop=11, environment_strength=.34, light_size=2.1),
        "front": View(az=90, el=0, scale=430, target=(0, 0, 0), projection="orthographic",
                      title="CYBR GEO / END ELEVATION"),
        "side": View(az=0, el=0, scale=430, target=(0, 0, 0), projection="orthographic",
                     title="CYBR GEO / SIDE ELEVATION"),
        "top": View(az=0, el=89, scale=430, target=(0, 0, -20), projection="orthographic",
                    title="CYBR GEO / TOP"),
        "internal": View(az=36, el=16, scale=500, target=(0, 0, 0), hide=("housing",),
                         title="CYBR GEO / ROTOR + WINDINGS"),
        "exploded": View(az=38, el=17, scale=900, target=(0, 0, 0), explode=1,
                         title="CYBR GEO / EXPLODED ASSEMBLY"),
    }
    return Assembly(
        "portfolio_geo",
        parts,
        MATERIALS,
        views,
        metadata=_concept_metadata(
            "portfolio_geo",
            {"length": length, "width": 640.0, "height": 680.0},
            ["Reference sheet calls for a large black-and-copper motor/generator with exposed windings and axial exploded views."],
        ),
        motion_function=_geo_pose,
    )


def build_light() -> Assembly:
    parts = []
    _base_module(parts, "LI", 1420, 520, 105, z=-250, label_index=3)
    _rail_pair(parts, "LI", 1320, 310, -155, 0)

    # Emitter barrel.
    for i, (x0, x1, ro, ri, mat) in enumerate([
        (-620, -540, 98, 46, 0), (-540, -485, 78, 44, 1), (-485, -440, 62, 40, 4)
    ]):
        _part(parts, f"LI_Emitter_stage_{i+1}", ring(ro, ri, x0, x1), mat, "optics",
              role="Collimated laser emitter barrel", explode=(-130, 0, 0))
    _part(parts, "LI_Emitter_window", ring(44, 0, -443, -436), 5, "glass",
          role="Emitter optical window", explode=(-130, 0, 0))

    # Prism tower and prism.
    tower_x = -150
    for y in (-105, 105):
        _part(parts, f"LI_Tower_post_{y:+}", _box((46, 46, 500), (tower_x, y, 95), 6), 0, "support",
              role="Optical tower post", explode=(0, y * .18, 80))
    _part(parts, "LI_Tower_crossbar", _box((72, 250, 46), (tower_x, 0, 335), 5), 0, "support",
          role="Optical tower crossbar", explode=(0, 0, 110))
    prism = cq.Workplane("XY").polyline([(-86, -72), (90, -72), (0, 98)]).close().extrude(118, both=True).val()
    prism = prism.rotate((0, 0, 0), (0, 1, 0), 90).translate((tower_x, 0, 155))
    _part(parts, "LI_Prism", prism, 5, "glass", role="Large optical-grade triangular prism", explode=(0, 0, 160))

    # Central vertical glass relay tube.
    _part(parts, "LI_Relay_glass", cq.Solid.makeCylinder(52, 280, cq.Vector(-20, 0, -40), cq.Vector(0, 0, 1)),
          5, "glass", role="Vertical relay optic", explode=(0, 0, 90))
    _part(parts, "LI_Relay_base", _cyl_z(94, 32, (-20, 0, -68)), 4, "optics",
          role="Relay optic rotary base", explode=(0, 0, 45))

    # Steering mirror mount.
    mirror_center = np.array([290.0, 0.0, 10.0])
    _part(parts, "LI_Mirror_pedestal", _box((130, 128, 190), (290, 0, -40), 8), 0, "support",
          role="Precision steering mirror pedestal", explode=(0, 0, 65))
    mirror = _box((16, 154, 154), mirror_center, 4).rotate(tuple(mirror_center), tuple(mirror_center + np.array([0, 1, 0])), 42)
    _part(parts, "LI_Steering_mirror", mirror, 2, "optics",
          role="Beam steering mirror", explode=(35, 0, 60))

    # Output lens.
    for i, (x0, x1, ro, ri, mat) in enumerate([
        (505, 560, 96, 42, 0), (560, 610, 118, 42, 1), (610, 626, 104, 0, 5)
    ]):
        _part(parts, f"LI_Output_stage_{i+1}", ring(ro, ri, x0, x1), mat, "optics",
              role="Output lens assembly", explode=(130, 0, 0))

    # Visible beam segments as actual tube geometry.
    beam_pts = [
        ([-436, 0, 0], [-150, 0, 155]),
        ([-150, 0, 155], [290, 0, 40]),
        ([290, 0, 40], [505, 0, 0]),
        ([626, 0, 0], [770, 0, 0]),
    ]
    for i, (a, b) in enumerate(beam_pts):
        _part(parts, f"LI_Beam_{i+1}", _cylinder_between(a, b, 4.0), 14, "beam",
              role="Visible portfolio laser path", explode=(0, 0, 0), tolerance=.5, angular=.3)

    for x in (-560, -300, 0, 300, 560):
        for y in (-155, 155):
            _part(parts, f"LI_Bench_clamp_{x}_{y}", _box((34, 46, 72), (x, y, -125), 4), 4, "mounts",
                  role="Optical bench clamp", explode=(0, y * .12, -35))

    views = {
        "hero": View(az=26, el=18, scale=500, target=(0, 0, 40), title="CYBR LIGHT / OPTICAL BENCH",
                     note="Emitter, prism, relay optic, steering mirror and output lens connected by one beam path.",
                     f_stop=10, environment_strength=.24, light_size=1.7),
        "front": View(az=0, el=4, scale=420, target=(0, 0, 40), projection="orthographic",
                      title="CYBR LIGHT / SIDE ELEVATION"),
        "top": View(az=0, el=89, scale=440, target=(0, 0, -20), projection="orthographic",
                    title="CYBR LIGHT / TOP"),
        "beam_path": View(az=0, el=0, scale=390, target=(0, 0, 55), projection="orthographic",
                          hide=("base", "rails", "mounts"), title="CYBR LIGHT / BEAM PATH"),
        "exploded": View(az=26, el=18, scale=720, target=(0, 0, 60), explode=1.0,
                         title="CYBR LIGHT / EXPLODED OPTICS"),
    }
    return Assembly(
        "portfolio_light",
        parts,
        MATERIALS,
        views,
        metadata=_concept_metadata(
            "portfolio_light",
            {"length": 1420.0, "width": 520.0, "height": 760.0},
            ["Reference sheet specifies emitter, prism, beam steering and output lens on a dense black precision bench."],
        ),
    )


def build_elements() -> Assembly:
    parts = []
    cube = 820.0
    base_h = 125.0
    _base_module(parts, "EL", cube, cube, base_h, z=-410, label_index=4)

    # Transparent containment: six thin panels rather than one solid block.
    panel = 12.0
    half = cube / 2
    center_z = 0.0
    _part(parts, "EL_Glass_front", _box((cube, panel, cube), (0, -half, center_z)), 5, "glass",
          role="Front containment panel", explode=(0, -120, 0))
    _part(parts, "EL_Glass_back", _box((cube, panel, cube), (0, half, center_z)), 5, "glass",
          role="Rear containment panel", explode=(0, 120, 0))
    _part(parts, "EL_Glass_left", _box((panel, cube, cube), (-half, 0, center_z)), 5, "glass",
          role="Left containment panel", explode=(-120, 0, 0))
    _part(parts, "EL_Glass_right", _box((panel, cube, cube), (half, 0, center_z)), 5, "glass",
          role="Right containment panel", explode=(120, 0, 0))
    _part(parts, "EL_Glass_top", _box((cube, cube, panel), (0, 0, half)), 5, "glass",
          role="Top containment panel", explode=(0, 0, 120))

    # Corner frame.
    for x in (-half - 24, half + 24):
        for y in (-half - 24, half + 24):
            _part(parts, f"EL_Frame_post_{x:+.0f}_{y:+.0f}",
                  _box((42, 42, cube + 130), (x, y, -5), 6), 0, "frame",
                  role="Machined containment corner frame", explode=(np.sign(x) * 70, np.sign(y) * 70, 0))
    for z in (-half - 42, half + 42):
        for y in (-half - 24, half + 24):
            _part(parts, f"EL_Frame_x_{z:+.0f}_{y:+.0f}", _box((cube + 130, 38, 38), (0, y, z), 5), 1, "frame",
                  role="Containment frame rail", explode=(0, np.sign(y) * 60, np.sign(z) * 60))

    # Rock core: clustered basalt solids.
    rock_specs = [
        (-135, 30, -205, 155), (-60, -15, -105, 175), (35, 20, -10, 185),
        (112, 12, 80, 150), (45, -12, 175, 125), (-90, 18, 120, 110),
    ]
    for i, (x, y, z, r) in enumerate(rock_specs):
        s = _sphere(r, (x, y, z))
        # flatten in Y to form a mountain face visible through front glass
        _part(parts, f"EL_Rock_{i+1:02}", s, 8, "terrain",
              role="Fractured basalt core", explode=(0, 0, 45))

    # Lava tendrils on left.
    lava_paths = [
        [(-220, -80, 250), (-180, -100, 140), (-210, -90, 20), (-155, -75, -120), (-185, -60, -260)],
        [(-85, -75, 300), (-125, -90, 200), (-90, -82, 90), (-125, -70, -25)],
        [(-285, -55, 100), (-230, -70, 25), (-265, -75, -80)],
    ]
    for i, pts in enumerate(lava_paths):
        _tube(parts, f"EL_Lava_{i+1:02}", np.asarray(pts, float), 14 if i == 0 else 10, 7, "fire",
              role="Hot lava/fire interaction strand", explode=(-55, 0, 0))

    # Water sheets on right.
    water_paths = [
        [(260, -75, 300), (220, -90, 180), (270, -95, 40), (210, -80, -105), (250, -70, -255)],
        [(140, -88, 250), (175, -92, 150), (145, -90, 55), (188, -82, -40)],
        [(320, -62, 120), (275, -75, 55), (305, -72, -30)],
    ]
    for i, pts in enumerate(water_paths):
        _tube(parts, f"EL_Water_stream_{i+1:02}", np.asarray(pts, float), 22 if i == 0 else 14, 6, "water",
              role="Water sheet/stream interacting with hot core", explode=(55, 0, 0))

    # Droplets and steam.
    rng = np.random.default_rng(404)
    for i in range(22):
        p = np.array([rng.uniform(70, 340), rng.uniform(-120, -40), rng.uniform(-250, 320)])
        _part(parts, f"EL_Droplet_{i+1:02}", _sphere(rng.uniform(4, 9), p), 6, "water",
              role="Water droplet", explode=(45, 0, 0), tolerance=.5, angular=.25)
    for i, x in enumerate((-110, -20, 80, 170)):
        pts = np.array([[x, 20, 70], [x + 28, 5, 150], [x - 18, 0, 235], [x + 15, 10, 330]], float)
        _tube(parts, f"EL_Steam_{i+1:02}", pts, 13, 15, "steam",
              role="Steam plume at thermal interaction boundary", explode=(0, 25, 40))

    # Side service wheel.
    _part(parts, "EL_Service_wheel", ring(126, 72, -470, -430), 1, "service",
          role="Side circulation/service wheel", explode=(-110, 0, 0))
    _fastener_ring(parts, "EL_Wheel_bolt", 96, 0, 12, screw_r=4.5, screw_h=6)

    views = {
        "hero": View(az=36, el=14, scale=610, target=(0, 0, -10), title="CYBR ELEMENTS / CONTAINMENT VOLUME",
                     note="820 mm fire-water-matter interaction chamber.", f_stop=11,
                     environment_strength=.28, light_size=2.0),
        "front": View(az=0, el=0, scale=500, target=(0, 0, 0), projection="orthographic",
                      title="CYBR ELEMENTS / FRONT"),
        "side": View(az=90, el=0, scale=500, target=(0, 0, 0), projection="orthographic",
                     title="CYBR ELEMENTS / SIDE"),
        "section": View(az=33, el=12, scale=520, target=(0, 0, 0), hide=("glass",),
                        title="CYBR ELEMENTS / INTERNAL STUDY"),
        "exploded": View(az=34, el=14, scale=760, target=(0, 0, 0), explode=1,
                         title="CYBR ELEMENTS / EXPLODED"),
    }
    return Assembly(
        "portfolio_elements",
        parts,
        MATERIALS,
        views,
        metadata=_concept_metadata(
            "portfolio_elements",
            {"width": cube, "depth": cube, "height": cube},
            ["Reference sheet shows a square optical containment chamber with central dark rock, orange fire/lava at left and blue water at right."],
        ),
    )


def build_materials() -> Assembly:
    parts = []
    width, depth = 980.0, 640.0
    _base_module(parts, "MA", width, depth, 130, z=-320, label_index=5)
    _rail_pair(parts, "MA", 900, 360, -220, 0)

    # Linear carriage.
    _part(parts, "MA_Carriage", _box((540, 420, 88), (-40, 0, -145), 12), 1, "mechanism",
          role="Material sample linear carriage", explode=(0, 0, -80))
    for x in (-300, -40, 220):
        _part(parts, f"MA_Linear_bearing_{x:+}", _box((92, 455, 64), (x, 0, -185), 8), 4, "mechanism",
              role="Precision linear bearing block", explode=(0, 0, -55))

    sample_specs = [
        (-210, 70, 280, 410, 42, 11, "Dark stone"),
        (-125, 45, 300, 450, 44, 9, "Walnut"),
        (-35, 18, 310, 470, 44, 9, "Oak"),
        (60, -5, 295, 450, 42, 10, "Maple"),
        (150, -28, 270, 405, 40, 17, "Ceramic"),
        (235, -50, 245, 360, 38, 11, "Marble"),
    ]
    for i, (x, y, sx, sz, sy, mat, role) in enumerate(sample_specs):
        _part(parts, f"MA_Sample_{i+1:02}_{role.replace(' ', '_')}",
              _box((sx, sy, sz), (x, y, -70 + sz / 2), 5),
              mat, "samples", role=f"{role} material specimen", explode=(0, 55 + i * 8, 80 + i * 18))

    # Rear inspection wheel.
    _part(parts, "MA_Inspection_wheel_outer", ring(135, 56, 318, 374), 1, "mechanism",
          role="Material inspection / presentation wheel", explode=(95, 0, 0))
    _part(parts, "MA_Inspection_wheel_inner", ring(90, 57, 322, 370), 2, "mechanism",
          role="Inner spindle", explode=(110, 0, 0))

    # Articulated arm on right.
    joints = [
        np.array([310.0, 220.0, 20.0]),
        np.array([380.0, 220.0, 310.0]),
        np.array([190.0, 120.0, 520.0]),
        np.array([30.0, 65.0, 410.0]),
    ]
    for i in range(len(joints) - 1):
        _part(parts, f"MA_Arm_link_{i+1}", _cylinder_between(joints[i], joints[i+1], 22), 0, "arm",
              role="Material handling arm link", explode=(0, 40, 25 * i))
    for i, p in enumerate(joints):
        _part(parts, f"MA_Arm_joint_{i+1}", _sphere(38 if i < 3 else 30, p), 4 if i in (1, 2) else 1,
              "arm", role="Articulated arm joint", explode=(0, 45, 25 * i))

    # Clamp head.
    _part(parts, "MA_Clamp_head", _box((130, 72, 80), tuple(joints[-1]), 8), 0, "arm",
          role="Non-destructive sample clamp", explode=(0, 55, 85))
    _part(parts, "MA_Clamp_pad_A", _box((48, 78, 28), tuple(joints[-1] + np.array([-45, 0, -45])), 5), 16, "arm",
          role="Rubber clamp pad", explode=(-18, 55, 85))
    _part(parts, "MA_Clamp_pad_B", _box((48, 78, 28), tuple(joints[-1] + np.array([45, 0, -45])), 5), 16, "arm",
          role="Rubber clamp pad", explode=(18, 55, 85))

    _service_lines(parts, "MA", [(390, -230, -170), (340, -245, -190)], 270, material=4, z_offset=-8)

    views = {
        "hero": View(az=34, el=18, scale=530, target=(0, 0, 60), title="CYBR MATERIALS / SAMPLE MACHINE",
                     note="Stepped wood, stone and ceramic specimens on a precision carriage.",
                     f_stop=11, environment_strength=.32, light_size=2.0),
        "front": View(az=0, el=0, scale=450, target=(0, 0, 50), projection="orthographic",
                      title="CYBR MATERIALS / FRONT"),
        "side": View(az=90, el=0, scale=450, target=(0, 0, 50), projection="orthographic",
                     title="CYBR MATERIALS / SIDE"),
        "top": View(az=0, el=89, scale=430, target=(0, 0, -80), projection="orthographic",
                    title="CYBR MATERIALS / TOP"),
        "exploded": View(az=36, el=18, scale=690, target=(0, 0, 70), explode=1.0,
                         title="CYBR MATERIALS / EXPLODED"),
    }
    return Assembly(
        "portfolio_materials",
        parts,
        MATERIALS,
        views,
        metadata=_concept_metadata(
            "portfolio_materials",
            {"length": width, "depth": depth, "height": 720.0},
            ["Reference sheet shows stepped material samples, exposed linear drives, black/copper craft language and an articulated handling arm."],
        ),
    )


def _tree(parts, prefix, x, y, z, trunk_h, crown_r, explode=(0, 0, 0)):
    _part(parts, f"{prefix}_trunk", _cyl_z(13, trunk_h, (x, y, z + trunk_h / 2)), 9, "biome",
          role="Tree trunk", explode=explode)
    for j, (dx, dy, dz, scale) in enumerate([
        (0, 0, 0, 1.0), (crown_r * .48, 0, -crown_r * .10, .72),
        (-crown_r * .42, crown_r * .18, -crown_r * .08, .68),
        (0, -crown_r * .45, crown_r * .02, .64),
    ]):
        s = _sphere(crown_r * scale, (x + dx, y + dy, z + trunk_h + dz))
        _part(parts, f"{prefix}_crown_{j+1}", s, 12, "biome",
              role="Tree canopy", explode=explode, tolerance=.5, angular=.24)


def build_forest() -> Assembly:
    parts = []
    base_d = 900.0
    base_h = 170.0
    _base_module(parts, "FO", base_d, base_d, base_h, z=-170, label_index=6)
    _part(parts, "FO_Biome_ring", ring(420, 370, 0, 62), 1, "mechanism",
          role="Sealed biome foundation ring", explode=(0, 0, 70))
    _fastener_ring(parts, "FO_Ring_bolt", 392, 66, 18, screw_r=5.0, screw_h=8)

    # Tall glass vessel, closer to the reference than a simple sphere.
    _part(parts, "FO_Glass_lower", cq.Solid.makeCylinder(390, 410, cq.Vector(0, 0, 50), cq.Vector(0, 0, 1)),
          5, "glass", role="Cylindrical sealed biome enclosure", explode=(0, 0, 115), tolerance=.34, angular=.22)
    _part(parts, "FO_Glass_dome", _hemisphere(390, (0, 0, 460)), 5, "glass",
          role="Rounded upper biome enclosure", explode=(0, 0, 145), tolerance=.34, angular=.22)

    # Layered terrain.
    terrain_levels = [
        (340, 45, 72, 8), (290, 35, 118, 13), (220, 30, 160, 13),
    ]
    for i, (r, h, z, mat) in enumerate(terrain_levels):
        _part(parts, f"FO_Terrain_{i+1}", _cyl_z(r, h, (0, 0, z)), mat, "biome",
              role="Biome terrain / moss substrate", explode=(0, 0, 60 + i * 18))

    rocks = [
        (-210, 80, 165, 80), (-120, 90, 220, 75), (150, 60, 190, 72),
        (215, -70, 160, 62), (40, 115, 265, 54), (-45, -120, 185, 58),
    ]
    for i, (x, y, z, r) in enumerate(rocks):
        _part(parts, f"FO_Rock_{i+1:02}", _sphere(r, (x, y, z)), 8, "biome",
              role="Biome rock", explode=(0, 0, 75))

    # Waterfall and stream.
    fall_pts = np.array([[60, 65, 420], [50, 62, 355], [35, 52, 290], [5, 30, 235], [-40, 5, 190]], float)
    _tube(parts, "FO_Waterfall", fall_pts, 20, 6, "water", role="Waterfall / stream", explode=(0, 0, 80))
    stream_pts = np.array([[-40, 5, 190], [-95, -25, 165], [-155, -50, 145], [-220, -30, 132]], float)
    _tube(parts, "FO_Stream", stream_pts, 22, 6, "water", role="Biome stream", explode=(0, 0, 70))

    # Trees.
    _tree(parts, "FO_Tree_A", -145, 10, 160, 270, 82, explode=(0, 0, 95))
    _tree(parts, "FO_Tree_B", 105, 55, 180, 230, 70, explode=(0, 0, 95))
    _tree(parts, "FO_Tree_C", 5, -125, 150, 185, 58, explode=(0, 0, 95))
    _tree(parts, "FO_Tree_D", 205, -45, 140, 150, 48, explode=(0, 0, 95))

    # Moss clumps.
    for i, a in enumerate(np.linspace(0, math.tau, 16, endpoint=False)):
        r = 250 - (i % 3) * 32
        p = (r * math.cos(a), r * math.sin(a), 170 + (i % 4) * 12)
        _part(parts, f"FO_Moss_{i+1:02}", _sphere(28 + (i % 3) * 6, p), 13, "biome",
              role="Moss / ground cover", explode=(0, 0, 70), tolerance=.6, angular=.26)

    # Life support.
    for x, y in [(-440, -255), (440, 255), (440, -255)]:
        _part(parts, f"FO_Life_support_{x}_{y}", _cyl_z(42, 320, (x, y, 85)), 0, "life_support",
              role="Filtration / pump column", explode=(np.sign(x) * 55, np.sign(y) * 55, 0))
        _part(parts, f"FO_Life_cap_{x}_{y}", _cyl_z(50, 30, (x, y, 260)), 4, "life_support",
              role="Life-support service cap", explode=(np.sign(x) * 55, np.sign(y) * 55, 0))

    # Articulated inspection arm.
    joints = [
        np.array([-430., 285., 40.]),
        np.array([-480., 285., 360.]),
        np.array([-310., 240., 650.]),
        np.array([-160., 150., 660.]),
    ]
    for i in range(3):
        _part(parts, f"FO_Arm_link_{i+1}", _cylinder_between(joints[i], joints[i+1], 23), 0, "support",
              role="Environmental inspection arm", explode=(-45, 35, 25 * i))
    for i, p in enumerate(joints):
        _part(parts, f"FO_Arm_joint_{i+1}", _sphere(39, p), 4 if i in (1, 2) else 1, "support",
              role="Inspection-arm joint", explode=(-45, 35, 25 * i))

    views = {
        "hero": View(az=36, el=17, scale=650, target=(0, 0, 275), title="CYBR FOREST / SEALED BIOME",
                     note="Contained forest, waterfall and life-support hardware in one machine.",
                     f_stop=12, environment_strength=.32, light_size=2.0),
        "front": View(az=0, el=3, scale=590, target=(0, 0, 300), projection="orthographic",
                      title="CYBR FOREST / FRONT"),
        "side": View(az=90, el=3, scale=590, target=(0, 0, 300), projection="orthographic",
                     title="CYBR FOREST / SIDE"),
        "top": View(az=0, el=89, scale=520, target=(0, 0, 230), projection="orthographic",
                    title="CYBR FOREST / TOP"),
        "internal": View(az=36, el=17, scale=570, target=(0, 0, 270), hide=("glass",),
                         title="CYBR FOREST / BIOME INTERNALS"),
        "exploded": View(az=37, el=17, scale=820, target=(0, 0, 290), explode=1,
                         title="CYBR FOREST / EXPLODED"),
    }
    return Assembly(
        "portfolio_forest",
        parts,
        MATERIALS,
        views,
        metadata=_concept_metadata(
            "portfolio_forest",
            {"diameter": base_d, "height": 900.0},
            ["Reference sheet shows a sealed forest terrarium with large trees, waterfall, black mechanical base, support arm and filtration hardware."],
        ),
    )


BUILDERS = {
    "portfolio_scenes": build_scenes,
    "portfolio_geo": build_geo,
    "portfolio_light": build_light,
    "portfolio_elements": build_elements,
    "portfolio_materials": build_materials,
    "portfolio_forest": build_forest,
}


def _translated(part: Part, offset, prefix):
    return part.moved(np.asarray(offset, float), prefix=prefix)


def build_machine() -> Assembly:
    """Assemble the six reference modules into the preferred horizontal portfolio machine."""
    assemblies = [
        build_scenes(),
        build_geo(),
        build_light(),
        build_elements(),
        build_materials(),
        build_forest(),
    ]
    x_positions = [-3350, -2050, -700, 650, 1950, 3200]
    z_offsets = [0, 20, 25, 0, 40, -10]
    merged = []
    for assembly, x, z in zip(assemblies, x_positions, z_offsets):
        prefix = assembly.name.replace("portfolio_", "").upper() + "__"
        for part in assembly.parts:
            merged.append(_translated(part, (x, 0, z), prefix))

    # Continuous rails and service spine make the six exhibits read as one machine.
    _part(merged, "PM_Main_spine", _box((7600, 92, 92), (0, 280, -360), 12), 0, "spine",
          role="Shared structural/data spine connecting all portfolio modules", explode=(0, 0, -130))
    _part(merged, "PM_Service_spine", _box((7600, 46, 58), (0, -295, -352), 7), 4, "spine",
          role="Shared warm-metal service bus", explode=(0, 0, -115))
    for x in np.linspace(-3600, 3600, 13):
        _part(merged, f"PM_Spine_clamp_{x:+.0f}", _box((62, 710, 86), (x, 0, -347), 8), 1, "spine",
              role="Cross-machine structural clamp", explode=(0, 0, -95))

    views = {
        "hero": View(az=14, el=10, scale=1120, target=(0, 0, 120), title="CYBR / A PORTFOLIO MACHINE",
                     note="Six projects. One continuous physical system.", focal_length_mm=72,
                     f_stop=13, environment_strength=.30, light_size=2.4),
        "wide": View(az=0, el=6, scale=960, target=(0, 0, 120), projection="orthographic",
                     title="CYBR / PORTFOLIO MACHINE / FRONT"),
        "three_quarter": View(az=24, el=14, scale=1280, target=(0, 0, 100),
                              title="CYBR / PORTFOLIO MACHINE / THREE QUARTER"),
    }
    meta = _concept_metadata(
        "portfolio_machine",
        {"length": 7600.0, "depth": 1050.0, "height": 1050.0},
        ["Horizontal machine layout matching the user's preferred first portfolio concept: six functionally distinct project objects linked by a common mechanical spine."],
    )
    meta["modules"] = list(BUILDERS)
    return Assembly("portfolio_machine", merged, MATERIALS, views, metadata=meta)


def build(name="portfolio_machine") -> Assembly:
    if name == "portfolio_machine":
        return build_machine()
    try:
        return BUILDERS[name]()
    except KeyError as error:
        raise ValueError(f"Unknown portfolio module {name!r}") from error
