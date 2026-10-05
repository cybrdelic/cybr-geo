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

    # A visible mid-level process spine makes the modules parts of one machine
    # rather than six exhibits sitting on a rail. Twin shafts carry power/fluid/
    # data through every project and terminate into local manifolds.
    for pi,(y,z,ro,ri,mat) in enumerate([
        (-305.,-250.,22.,13.,0),
        (-260.,-205.,14.,8.,3),
        (305.,-250.,18.,10.,1),
    ]):
        add_cad(parts,f"PM_Process_spine_{pi}",annulus(ro,ri,(-4000,y,z),7200,"X"),mat,"spine",
                role="Continuous cross-project process/service spine")
        for ci,x in enumerate([-3820,-3220,-2850,-1550,-1450,-150, -50,1050,1150,2150,2250,3050]):
            add_cad(parts,f"PM_Process_spine_{pi}_collar_{ci}",
                    annulus(ro+9,ro+1,(x-9,y,z),18,"X"),3 if ci%2 else 1,"spine",
                    role="Process-spine retaining/coupling collar")

    # Each module has a local gearbox/manifold pedestal tied into the common
    # process spine by paired risers and an articulated service loop.
    for i,(x,zoff) in enumerate(zip(centers,z_offsets)):
        pedestal_z=-185+zoff*.10
        add_cad(parts,f"PM_Module_pedestal_{i+1}",box(150,130,170,(x,-300,pedestal_z),12),0,"spine",
                role="Local module process/manifold pedestal")
        add_cad(parts,f"PM_Module_bearing_ring_{i+1}",
                annulus(55,28,(x-26,-300,pedestal_z+35),52,"X"),1,"spine",
                role="Local service-shaft bearing/coupler")
        add_cad(parts,f"PM_Module_coupling_ring_{i+1}",
                annulus(38,28,(x+30,-300,pedestal_z+35),24,"X"),3,"spine",
                role="Warm-metal module coupling sleeve")
        for ry in (-318,-282):
            add_cad(parts,f"PM_Module_riser_{i+1}_{int(ry)}",
                    cylinder(7,145,(x,ry,-250),"Z"),3,"spine",
                    role="Module service riser from common process shaft")
        pipe(parts,f"PM_Module_loop_{i+1}",
             [(x,-305,-185),(x+38,-265,-115+zoff*.12),(x+70,-235,-45+zoff*.20),(x+55,-215,30+zoff*.24)],
             6.5,3,"spine","Visible module service loop into project subsystem",analytic=True)

    # Short inter-module trusses and overhead conduits make the horizontal
    # composition read as a single engineered mechanism at portfolio scale.
    for i,(a,b) in enumerate(zip(centers[:-1],centers[1:])):
        mid=(a+b)/2
        span=b-a-620
        if span>80:
            add_cad(parts,f"PM_Intermodule_bridge_{i}",
                    box(span,42,46,(mid,0,-125),6),1,"spine",
                    role="Bolted inter-module bridge beam")
            add_cad(parts,f"PM_Intermodule_trim_{i}",
                    box(span-28,10,16,(mid,-25,-102),3),3,"spine",
                    role="Warm-metal bridge reinforcement")
        pipe(parts,f"PM_Overhead_conduit_{i}",
             [(a+360,250,-70),(mid-120,250,-35),(mid+120,250,-35),(b-360,250,-70)],
             5.5,3,"spine","Inter-module routed data/coolant conduit",analytic=True)

    views={
        # Looking perpendicular to the X lineup is essential: previous review looked down
        # the machine and collapsed six modules onto one another.
        "hero":View(az=-90,el=9,scale=2120,target=(-400,0,220),title="CYBR / A PORTFOLIO MACHINE",
                    note="Six independently modeled systems on one continuous structural and service backbone.",
                    projection="orthographic",environment_strength=.30,light_size=2.5),
        "wide":View(az=-90,el=3,scale=2080,target=(-400,0,190),projection="orthographic",
                    title="CYBR / PORTFOLIO MACHINE / FRONT"),
        "three_quarter":View(az=-78,el=11,scale=2180,target=(-400,0,200),projection="orthographic",
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
