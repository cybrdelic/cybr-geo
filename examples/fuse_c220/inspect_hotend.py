"""Render an explicit CAD section of the corrected feed path through shared V9."""
from dataclasses import replace
import json
from pathlib import Path

import cadquery as cq

from printer import BED, State, build, motion
from mechanism_lab.core import cad_part
from mechanism_lab.render_profiles import V9
from mechanism_lab.v9_dispatch import render_v9


def section(assembly):
    selected = {"nozzle_040", "heater_block", "heater_cartridge",
                "temperature_sensor_envelope", "heatbreak", "heatsink_core",
                "filament_guide_sleeve", "hotend_clamp_bridge"}
    selected.update(f"heatsink_fin_{index}" for index in range(10))
    # Keep the back half and truncate the guide beyond the inspection region.
    front = cq.Workplane("XY").box(200, 200, 300).translate((0, -100, BED + 70)).val()
    above = cq.Workplane("XY").box(200, 200, 300).translate((0, 0, BED + 205)).val()
    parts = []
    for part in assembly.parts:
        if part.name not in selected:
            continue
        shape = part.cad.cut(front).cut(above)
        if shape.Volume() < 1e-8:
            continue
        parts.append(cad_part(part.name, shape, part.material, tolerance=.012, angular=.10,
                              group=part.group, motion="head",
                              provenance=part.provenance,
                              role=part.role + "; inspection-only Y=0 and Z=55 CAD section"))
    view = replace(assembly.views["printing"], az=270, el=12, scale=34,
                   target=(0, 2, BED + 27), f_stop=16, floor=False)
    return replace(assembly, parts=parts, views={"feed_section": view},
                   motion_function=motion(State(0, 0, 0)),
                   metadata={**assembly.metadata,
                             "inspection_section": "Negative-Y half and material above nozzle Z+55 removed by analytic BREP cuts; not the assembled part."})


def main():
    output = Path("deliverables/FUSE_C220_feed_section.png")
    assembly = section(build())
    receipt = render_v9(assembly, output, view_name="feed_section",
                        size=(1200, 1200), spp=V9.still_spp, depth=V9.still_depth)
    receipt["inspection_section"] = assembly.metadata["inspection_section"]
    output.with_suffix(".json").write_text(json.dumps(receipt, indent=2) + "\n")


if __name__ == "__main__":
    main()
