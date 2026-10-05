"""Final portfolio machine composition.

The six modules are independently authored assemblies. This file places them on one
continuous service/structural spine without flattening them into cards or fusing them
into an uninspectable monolith.
"""
from __future__ import annotations

import math
import numpy as np

from mechanism_lab.core import Assembly, View

from .portfolio_final_common import MATERIALS, box, annulus, cylinder, add_cad, pipe
from .portfolio_final_machines import build_geo, build_light, build_materials, _geo_pose
from .portfolio_final_worlds import build_scenes, build_elements, build_forest


BUILDERS={
    "portfolio_scenes":build_scenes,
    "portfolio_geo":build_geo,
    "portfolio_light":build_light,
    "portfolio_elements":build_elements,
    "portfolio_materials":build_materials,
    "portfolio_forest":build_forest,
}


def build_machine():
    assemblies=[
        build_scenes(),
        build_geo(),
        build_light(),
        build_elements(),
        build_materials(),
        build_forest(),
    ]
    centers=[-3500.,-2200.,-800.,500.,1600.,2700.]
    z_offsets=[0.,120.,100.,70.,120.,10.]
    parts=[]
    for assembly,x,z in zip(assemblies,centers,z_offsets):
        prefix=assembly.name.replace("portfolio_","").upper()+"__"
        for p in assembly.parts:
            parts.append(p.moved(np.array([x,0,z]),prefix=prefix))

    # Continuous structural spine: twin hollow beams with bolted module saddles.
    for y in (-420.,420.):
        beam=box(7200,72,92,(-400,y,-480),9)
        beam=beam.cut(box(7100,38,56,(-400,y,-480),6))
        add_cad(parts,f"PM_Backbone_{'L' if y<0 else 'R'}",beam,0,"spine",
                role="Continuous hollow portfolio-machine backbone")
    # Crossmembers below every module and inter-module node.
    for i,x in enumerate([-3800,-3500,-2850,-2200,-1500,-800,-150,500,1050,1600,2150,2700,3150]):
        cross=box(86,900,82,(x,0,-475),8)
        cross=cross.cut(box(52,820,48,(x,0,-475),5))
        add_cad(parts,f"PM_Crossmember_{i:02}",cross,0,"spine",role="Hollow backbone crossmember")
        for y in (-420,420):
            add_cad(parts,f"PM_Cross_bolt_{i}_{int(y)}",cylinder(6,110,(x,y-55,-475),"Y"),2,"fasteners",
                    role="Backbone crossmember through bolt")

    # Service bus is physically routed, with junction blocks at every project.
    add_cad(parts,"PM_Power_bus_tray",box(7050,66,42,(-400,-330,-410),7),1,"spine",
            role="Shared power/data/cooling tray")
    for i,(x,z) in enumerate(zip(centers,z_offsets)):
        block=box(120,110,90,(x,-330,-360),10)
        for dz in (-18,18):
            block=block.cut(cylinder(9,130,(x,-395,-360+dz),"Y"))
        add_cad(parts,f"PM_Junction_{i+1}",block,0,"spine",role="Module service junction block")
        # Short umbilical rises toward the local module instead of decorative disconnected tubing.
        pipe(parts,f"PM_Umbilical_{i+1}",[(x,-330,-315),(x,-300,-250),(x+35,-260,-160+z*.15),(x+55,-220,-80+z*.20)],
             9,3,"spine","Shared service umbilical from backbone to module",analytic=True)

    # Mechanical couplers visually and structurally connect adjacent sections.
    for i,(a,b) in enumerate(zip(centers[:-1],centers[1:])):
        x=(a+b)/2
        add_cad(parts,f"PM_Coupler_outer_{i}",annulus(52,36,(x-24,-420,-480),48,"X"),1,"spine",
                role="Backbone service/structural coupler")
        add_cad(parts,f"PM_Coupler_inner_{i}",annulus(35,25,(x-18,-420,-480),36,"X"),3,"spine",
                role="Warm-metal coupler sleeve")

    views={
        # Looking perpendicular to the X lineup is essential: previous review looked down
        # the machine and collapsed six modules onto one another.
        "hero":View(az=90,el=9,scale=2120,target=(-400,0,220),title="CYBR / A PORTFOLIO MACHINE",
                    note="Six independently modeled systems on one continuous structural and service backbone.",
                    projection="orthographic",environment_strength=.30,light_size=2.5),
        "wide":View(az=90,el=3,scale=2080,target=(-400,0,190),projection="orthographic",
                    title="CYBR / PORTFOLIO MACHINE / FRONT"),
        "three_quarter":View(az=78,el=11,scale=2180,target=(-400,0,200),projection="orthographic",
                             title="CYBR / PORTFOLIO MACHINE / THREE QUARTER"),
    }
    metadata=dict(
        truth_intent="concept",
        fidelity="final authored portfolio machine; modules retain their independent full geometry",
        reference_id="portfolio-reference-sheets-2026-10-04-final",
        modules=list(BUILDERS),
        concept_dimensions_mm={"length":7200.,"depth":1100.,"height":1200.},
        assumptions=["Portfolio-scale concept assembly; shared services are illustrative rather than engineering-rated."],
    )
    return Assembly("portfolio_machine",parts,MATERIALS,views,metadata=metadata)


def build(name="portfolio_machine"):
    if name=="portfolio_machine":
        return build_machine()
    try:
        return BUILDERS[name]()
    except KeyError as exc:
        raise ValueError(f"Unknown portfolio module {name!r}") from exc


__all__=["build","build_machine","BUILDERS","_geo_pose"]
