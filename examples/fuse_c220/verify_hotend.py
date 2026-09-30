"""Independent BREP checks for the corrected FUSE feed and cooling passages."""
from __future__ import annotations

from itertools import combinations
import json
from pathlib import Path

import cadquery as cq

from printer import BED, build, cylinder


def verify(assembly):
    names = {part.name: part for part in assembly.parts}
    hotend = [
        "nozzle_040", "heater_block", "heater_cartridge",
        "temperature_sensor_envelope", "heatbreak", "heatsink_core",
        *[f"heatsink_fin_{index}" for index in range(10)],
        "cooling_duct_-1", "cooling_duct_1", "part_blower_-1", "part_blower_1",
    ]
    checks = []
    def add(name, passed, detail):
        checks.append({"check": name, "passed": bool(passed), "detail": detail})

    # The nominal 1.75 mm feed remains in a 2 mm bore until the melt taper.
    # The final 0.38 mm witness checks the specified 0.40 mm nozzle outlet.
    feed = cylinder(.875, 126.9, (0, 0, BED + 2.1))
    taper = cq.Solid.makeCone(.19, .99, 1., cq.Vector(0, 0, BED + 1.1))
    outlet = cylinder(.19, 1.21, (0, 0, BED - .1))
    witness = feed.fuse(taper).fuse(outlet)
    passage_parts = [
        p for p in assembly.parts
        if p.cad is not None and p.group in ("hotend", "extruder", "cooling")
    ]
    blocked = []
    for part in passage_parts:
        volume = part.cad.intersect(witness).Volume()
        if volume > 1e-6:
            blocked.append({"part": part.name, "intersection_mm3": volume})
    add("continuous feed, taper and nozzle outlet are unobstructed",
        not blocked, {"blocked": blocked, "feed_diameter_mm": 1.75,
                      "outlet_witness_diameter_mm": .38,
                      "feed_start_z_mm": BED + 2.1, "feed_end_z_mm": BED + 129})

    overlaps = []
    queries = 0
    for left, right in combinations(hotend, 2):
        a, b = names[left].cad, names[right].cad
        ba, bb = a.BoundingBox(), b.BoundingBox()
        if not all(min(getattr(ba, axis + "max"), getattr(bb, axis + "max")) >
                   max(getattr(ba, axis + "min"), getattr(bb, axis + "min")) + 1e-7
                   for axis in "xyz"):
            continue
        queries += 1
        volume = a.intersect(b).Volume()
        if volume > 1e-5:
            overlaps.append({"parts": [left, right], "intersection_mm3": volume})
    add("hotend and cooling solids do not interpenetrate",
        not overlaps, {"narrow_phase_queries": queries, "collisions": overlaps})

    contacts = []
    for left, right in [
        ("nozzle_040", "heatbreak"), ("heatbreak", "heatsink_core"),
        *[("heatsink_core", f"heatsink_fin_{index}") for index in range(10)],
        ("heatsink_core", "filament_guide_sleeve"),
    ]:
        contacts.append({"parts": [left, right],
                         "gap_mm": names[left].cad.distance(names[right].cad)})
    add("nozzle, heatbreak, heatsink and guide meet at nominal interfaces",
        all(row["gap_mm"] < 1e-6 for row in contacts), contacts)

    # Query the bore envelopes themselves, not just a subtraction of constants.
    central_bore = cylinder(3.05, 12, (0, 0, BED + 6.5))
    heater_bore = cylinder(3.05, 20, (-10, 7, BED + 12.5), (1, 0, 0))
    ligament = central_bore.distance(heater_bore)
    cartridge_block = names["heater_cartridge"].cad.intersect(names["heater_block"].cad).Volume()
    add("heater bore is separate from feed bore and cartridge clears block",
        ligament >= .89 and cartridge_block < 1e-6,
        {"nominal_interbore_ligament_mm": ligament,
         "cartridge_block_intersection_mm3": cartridge_block,
         "qualification": "Nominal geometry only; not a thermal or strength margin"})

    # Sweep an independent small flow witness from each jet through the blower
    # outlet. This tests passage continuity rather than only hollow part labels.
    duct_results = []
    for side in (-1, 1):
        bottom = (side * 11.5, -1.)
        top = (side * 24., 15.)
        air = (cq.Workplane("XY", origin=(*bottom, BED + 3.9)).rect(1, 1)
               .workplane(offset=32.2).center(top[0] - bottom[0], top[1] - bottom[1])
               .rect(1, 1).loft().val())
        air = air.fuse(cylinder(.4, 8, (*top, BED + 35.9)))
        blockers = []
        for name in hotend:
            volume = air.intersect(names[name].cad).Volume()
            if volume > 1e-6:
                blockers.append({"part": name, "intersection_mm3": volume})
        duct_results.append({"side": side, "blocked": blockers})
    add("both cooling ducts have open passages to blower outlets",
        all(not row["blocked"] for row in duct_results), duct_results)
    return {
        "all_passed": all(row["passed"] for row in checks),
        "checks": checks,
        "scope": "Nominal analytic passage and intersection checks. No thread-fit, "
                 "fastener-retention, heater/sensor calibration, thermal, airflow, "
                 "pressure, melt, adhesion, stiffness or physical printing qualification.",
    }


def main():
    out = Path("deliverables")
    out.mkdir(parents=True, exist_ok=True)
    report = verify(build())
    (out / "FUSE_C220_hotend_validation.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)
    if not report["all_passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
