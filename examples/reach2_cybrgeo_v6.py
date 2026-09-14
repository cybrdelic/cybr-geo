"""CYBR REACH-2 v6: compact ORBIT-family elbow built on native CYBR GEO APIs.

Public model contract: build() -> cybrgeo.Assembly.
Geometry kernel: CadQuery BReps, ingested through cybrgeo.from_shape.
Rendering/export are delegated to CYBR GEO public APIs.
Native units are millimetres, Z-up.
"""
from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import cadquery as cq
import numpy as np

from cybrgeo import Assembly, Material, from_shape, rotation

ROOT = Path(__file__).resolve().parents[1]
ORBIT_RECIPE = ROOT / "examples" / "orbit_inspection_wrist.py"

PIVOT = np.array([-22.0, 9.0, -60.0], float)
ORBIT_PATTERN = ((-46.0, -45.0), (-46.0, 63.0), (2.0, -45.0), (2.0, 63.0))
LOWER_PATTERN = ((-57.0, -26.0), (-57.0, 44.0), (13.0, -26.0), (13.0, 44.0))
MAX_ELBOW_DEG = 70.0
PERIOD = 12.0

# Compact real-component drivetrain envelopes.
GEAR_MODEL = "Harmonic Drive CSG-20-160-2UH-LW"
GEAR_RATIO = 160.0
GEAR_OD_MM = 93.0
GEAR_LENGTH_MM = 45.5
GEAR_MASS_KG = 0.64
GEAR_RATED_TORQUE_NM = 52.0
GEAR_REPEATED_PEAK_NM = 120.0
GEAR_MOMENTARY_PEAK_NM = 191.0
GEAR_ALLOWABLE_MOMENT_NM = 74.6
GEAR_STATIC_LOAD_N = 9000.0
GEAR_DYNAMIC_LOAD_N = 5780.0
GEAR_MAX_AVG_INPUT_RPM = 3500.0

MOTOR_MODEL = "Delta ECMA-C10401 100 W / 40 mm servo class"
MOTOR_FRAME_MM = 40.0
MOTOR_BODY_LENGTH_MM = 70.0
MOTOR_RATED_TORQUE_NM = 0.32
MOTOR_MAX_TORQUE_NM = 0.96
MOTOR_RATED_RPM = 3000.0
MOTOR_MAX_RPM = 5000.0
MOTOR_MASS_KG = 0.50

# Optional external support pair, sized from a real NSK 6805 envelope/rating.
SUPPORT_BEARING_MODEL = "NSK 6805"
SUPPORT_BEARING_ID_MM = 25.0
SUPPORT_BEARING_OD_MM = 37.0
SUPPORT_BEARING_WIDTH_MM = 7.0
SUPPORT_BEARING_DYNAMIC_N = 4950.0
SUPPORT_BEARING_STATIC_N = 3150.0
SUPPORT_BEARING_SPACING_MM = 23.0


def _load_orbit():
    spec=importlib.util.spec_from_file_location("orbit_for_cybrgeo_v6", ORBIT_RECIPE)
    m=importlib.util.module_from_spec(spec); assert spec.loader is not None; spec.loader.exec_module(m)
    return m


def _adapt_materials(old):
    return [Material(m.name, tuple(m.color), m.metal, m.rough, ior=m.ior,
                     coat=m.coat, coat_rough=m.coat_rough,
                     anisotropy=m.anisotropy, anisotropy_rotation=m.anisotropy_rotation,
                     opacity=m.opacity, microfinish=m.microfinish,
                     material_source=m.material_source) for m in old.materials]


def box(dx,dy,dz,center,fillet=0.0):
    q=cq.Workplane("XY").box(dx,dy,dz)
    if fillet:
        r=min(float(fillet),0.40*min(dx,dy,dz))
        try:
            c=q.edges().fillet(r)
            if c.val().isValid(): q=c
        except Exception:
            pass
    return q.val().translate(tuple(center))


def cyl(r,length,origin,axis="Y"):
    d={"X":(1,0,0),"Y":(0,1,0),"Z":(0,0,1)}[axis]
    return cq.Solid.makeCylinder(r,length,cq.Vector(*origin),cq.Vector(*d))


def annulus_y(ro,ri,y0,length,x,z):
    s=cyl(ro,length,(x,y0,z),"Y")
    return s.cut(cyl(ri,length+2,(x,y0-1,z),"Y")) if ri else s


def plate_xz(points,y0,thickness):
    return cq.Workplane("XZ",origin=(0,y0,0)).polyline(points).close().extrude(thickness).val()


def _part(name,shape,material,group,motion="fixed",role=""):
    return from_shape(name,shape,material=material,group=group,motion=motion,
                      role=role or name,metadata={"provenance":"designed-concept","finish_axis":(0,1,0)})


def elbow_angle(t):
    return math.radians(MAX_ELBOW_DEG)*math.sin(math.tau*t/PERIOD)


def build() -> Assembly:
    orbit_mod=_load_orbit(); old=orbit_mod.build()
    materials=_adapt_materials(old)
    parts=[]; cad={}

    # Preserve exact ORBIT CAD, but expose it through the CYBR GEO assembly contract.
    for p in old.parts:
        if p.name.startswith("B02_"): continue
        name="O_"+p.name; shape=p.cad
        cp=_part(name,shape,p.material,"orbit_"+p.group,"elbow",p.role)
        parts.append(cp); cad[name]=shape

    px,py,pz=map(float,PIVOT)
    def add(name,shape,material=0,group="reach6_fixed",motion="fixed",role=""):
        if not shape.isValid(): raise ValueError(f"invalid CAD: {name}")
        p=_part(name,shape,material,group,motion,role);parts.append(p);cad[name]=shape;return shape

    saddle=box(78,140,8,(-22,9,-4),2.2)
    for x,y in ORBIT_PATTERN:
        saddle=saddle.cut(cyl(3.4,12,(x,y,-10),"Z")).cut(cyl(5.8,3.5,(x,y,-1.5),"Z"))
    add("R6_01_ORBIT_saddle",saddle,0,"reach6_output","elbow","8 mm ORBIT interface saddle")

    carrier_profile=[(-58,-8),(14,-8),(8,-22),(-2,-36),(-10,-48),(-14,-60),(-30,-60),(-37,-48),(-48,-36),(-55,-22)]
    for tag,y0 in (("A",-38.0),("B",-27.0)):
        s=plate_xz(carrier_profile,y0,8.0)
        s=s.cut(cyl(13.1,10,(px,y0-1,pz),"Y")).fuse(annulus_y(24,13.1,y0,8,px,pz)).clean()
        add(f"R6_03_{tag}_Output_carrier",s,0,"reach6_output","elbow","8 mm graphite output carrier")

    # 25 mm bearing journal keeps the carrier compact and supplies a real support path.
    add("R6_04_Hollow_output_hub",annulus_y(12.5,7.0,-40.0,30.0,px,pz),2,"reach6_output","elbow","25 mm steel hollow output journal")
    add("R6_05_Output_face_ring",annulus_y(36,14.1,-18,6,px,pz),1,"reach6_output","elbow","satin output adapter ring")

    # Two real thin-section bearing envelopes. Outer races are fixed; inner journal rotates.
    for tag,y0 in (("FRONT",-39.0),("REAR",-16.0)):
        add(f"R6_06_{tag}_6805_outer",annulus_y(18.5,15.2,y0,7.0,px,pz),2,"reach6_fixed","fixed",f"{SUPPORT_BEARING_MODEL} outer support envelope")
        add(f"R6_07_{tag}_6805_inner",annulus_y(15.0,12.5,y0,7.0,px,pz),2,"reach6_output","elbow",f"{SUPPORT_BEARING_MODEL} inner support envelope")

    gear_y0=py-GEAR_LENGTH_MM/2
    add("R6_10_CSG20_160_LW_envelope",annulus_y(GEAR_OD_MM/2,16.0,gear_y0,GEAR_LENGTH_MM,px,pz),1,"reach6_fixed","fixed","compact strain-wave gearhead envelope")

    # One connected ORBIT-like bearing/gear pedestal, only 104 mm across the main ring.
    support_y0=py-6.0
    ring=annulus_y(52.0,46.8,support_y0,12.0,px,pz)
    left=plate_xz([(px-49,pz-12),(px-34,pz-30),(px-46,-146),(px-68,-146),(px-56,pz-36)],support_y0,12)
    right=plate_xz([(px+49,pz-12),(px+34,pz-30),(px+46,-146),(px+24,-146),(px+56,pz-36)],support_y0,12)
    bridge=box(104,12,22,(px,py,pz-38),1.5)
    support=ring.fuse(left).fuse(right).fuse(bridge).clean()
    add("R6_12_ORBIT_family_gear_pedestal",support,0,"reach6_fixed","fixed","graphite annular gear pedestal")

    motor_y0=gear_y0+GEAR_LENGTH_MM+8
    add("R6_20_Circular_motor_adapter",annulus_y(31,12,motor_y0,7,px,pz),1,"reach6_fixed","fixed","satin circular motor adapter")
    add("R6_21_ECMA100W_motor_envelope",box(MOTOR_FRAME_MM,MOTOR_BODY_LENGTH_MM,MOTOR_FRAME_MM,(px,motor_y0+7+MOTOR_BODY_LENGTH_MM/2,pz),3),0,"reach6_fixed","fixed","40 mm 100 W servo envelope")
    # Removable cylindrical shell makes the installed silhouette match ORBIT rather than a generic square servo.
    add("R6_22_ORBIT_motor_service_shell",annulus_y(27.0,21.0,motor_y0+6,MOTOR_BODY_LENGTH_MM+4,px,pz),0,"reach6_fixed","fixed","graphite removable cylindrical motor shell")

    # Existing ORBIT material index 5 is teal.
    add("R6_30_Output_service_ring",annulus_y(39,36.5,-20.5,2.2,px,pz),5,"reach6_output","elbow","teal output service ring")
    add("R6_31_Rear_service_ring",annulus_y(28.5,25.5,motor_y0+MOTOR_BODY_LENGTH_MM+8,2.5,px,pz),5,"reach6_fixed","fixed","teal rear service ring")
    add("R6_32_Hollow_cable_gland",annulus_y(10.5,7.5,-14,4,px,pz),5,"reach6_output","elbow","teal hollow-axis cable gland")

    lower=box(104,104,10,(-22,9,-151),3)
    for x,y in LOWER_PATTERN:
        lower=lower.cut(cyl(4.5,14,(x,y,-158),"Z")).cut(cyl(8,4,(x,y,-147),"Z"))
    add("R6_40_Lower_modular_flange",lower,0,"reach6_fixed","fixed","next-link M8 flange")
    for tag,x in (("L",-55),("R",11)):
        add(f"R6_41_{tag}_Lower_spine",box(14,54,50,(x,9,-113),2.5),0,"reach6_fixed","fixed","narrow graphite lower spine")

    metadata={
        "schema":"cybrgeo.reach2/6",
        "aesthetic_family":"CYBR ORBIT",
        "geometry_api":"cybrgeo.Assembly + cybrgeo.from_shape",
        "render_api":"cybrgeo.photoreal.render (V9)",
        "selected_gear":GEAR_MODEL,"selected_motor":MOTOR_MODEL,
        "support_bearings":SUPPORT_BEARING_MODEL,"ratio":GEAR_RATIO,
        "native_units":"mm","generated_imagery":False,"manufacturing_release":False,
        "notes":["Native CYBR GEO public-API model.","CadQuery is the BRep kernel beneath cybrgeo.from_shape, not the public scene/render API.","Purchased gear and motor STEP overlays remain mandatory before machining."],
    }
    return Assembly("CYBR_REACH2_v6_ORBIT",parts,materials,metadata,cad)


def poses(assembly:Assembly,t:float):
    T=rotation(elbow_angle(t),axis=(0,1,0),center=tuple(PIVOT))
    return {p.name:(T if p.motion=="elbow" else np.eye(4)) for p in assembly.parts}


if __name__=="__main__":
    print(build().validate())
