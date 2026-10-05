"""Final manufactured portfolio modules.

The construction style is intentionally modeled after CYBR ORBIT and ROAM: real bores,
bearing races, rolling elements, fasteners, split housings, retained interfaces,
ground guides, carriages, actuators and routed services. These are not decorative
greeble passes over primitive silhouettes.
"""
from __future__ import annotations

import math
import numpy as np
import cadquery as cq

from mechanism_lab.core import Assembly, View, axis_pose
from mechanism_lab.geometry import sector
from cybrgeo.features import involute_spur_gear

from .portfolio_final_common import (
    MATERIALS, box, cylinder, annulus, sphere, capsule_plate, add_cad, add_mesh,
    cylinder_between, socket_screw_x, socket_screw_z, hex_nut_x, fluted_knob_x,
    flange_x, bearing_x, bolt_circle_x, bolt_circle_z, pipe, mesh_normals,
)


REFERENCE_ID="portfolio-reference-sheets-2026-10-04-final"


def _meta(name,dimensions,notes):
    return dict(
        truth_intent="concept",
        fidelity="final authored portfolio-machine geometry using CYBR GEO analytic/mesh construction",
        reference_id=REFERENCE_ID,
        concept_dimensions_mm=dimensions,
        reference_notes=list(notes),
        assumptions=[
            "Concept object: not qualified production hardware.",
            "Fastener interfaces, bearing fits, optical prescriptions and electrical/thermal ratings are illustrative.",
            "All visible geometry is authored; no image projection, photogrammetry or generated-image mesh is used.",
        ],
    )


def _rotate_x(shape,deg):
    return shape.rotate((0,0,0),(1,0,0),deg)


def _rotate_z(shape,deg):
    return shape.rotate((0,0,0),(0,0,1),deg)


def _skid(parts,prefix,length,width,z=-340):
    # Two real longitudinal box rails and three cross-members, not one monolithic block.
    for y in (-width*.34,width*.34):
        rail=box(length,72,82,(0,y,z),8)
        rail=rail.cut(box(length-70,38,46,(0,y,z),5))
        add_cad(parts,f"{prefix}_Skid_rail_{'L' if y<0 else 'R'}",rail,0,"base",
                role="Hollow structural skid rail",explode=(0,0,-65))
    for i,x in enumerate((-length*.38,0,length*.38)):
        cross=box(82,width*.76,72,(x,0,z+5),7)
        cross=cross.cut(box(48,width*.76-64,38,(x,0,z+5),4))
        add_cad(parts,f"{prefix}_Crossmember_{i+1}",cross,0,"base",
                role="Hollow skid crossmember",explode=(0,0,-58))
    for x in (-length*.43,length*.43):
        for y in (-width*.34,width*.34):
            add_cad(parts,f"{prefix}_Foot_{x:+.0f}_{y:+.0f}",annulus(26,8,(x,y,z-52),32,"Z"),
                    3,"base",role="Machined isolation foot",explode=(0,0,-90))
            add_cad(parts,f"{prefix}_Foot_pad_{x:+.0f}_{y:+.0f}",cylinder(29,8,(x,y,z-60),"Z"),
                    5,"base",role="Elastomer isolation pad",explode=(0,0,-98))


# ---------------------------------------------------------------------------
# CYBR GEO — full exposed electromechanical machine
# ---------------------------------------------------------------------------

def _geo_pose(part,t,e):
    if part.motion=="rotor":
        return axis_pose(t*math.tau*.115,part.center,part.explode,e)
    T=np.eye(4);T[:3,3]=np.asarray(part.explode)*e;return T


def build_geo():
    parts=[]
    _skid(parts,"GEO",1180,650,-335)

    # Four pedestal castings with real bored saddles and bolted feet.
    for side,x in [("L",-365),("R",365)]:
        for y in (-220,220):
            ped=box(150,110,250,(x,y,-190),12)
            ped=ped.cut(cylinder(76,170,(x-85,y,-40),"X"))
            ped=ped.cut(box(82,70,90,(x,y,-110),8))
            add_cad(parts,f"GEO_{side}_Pedestal_{'A' if y<0 else 'B'}",ped,0,"mounts",
                    role="Machined/cast generator support pedestal",explode=(0,np.sign(y)*42,-35))
            for dx in (-45,45):
                add_cad(parts,f"GEO_{side}_{int(y)}_Foot_bolt_{dx:+}",
                        socket_screw_z(x+dx,y,-330,46,8),2,"fasteners",
                        role="Pedestal-to-skid through fastener",explode=(0,0,-60))

    # Layered end bells, each with genuinely cut ventilation windows and bolt circles.
    for side,x,sgn in [("rear",-338,-1),("front",338,1)]:
        bell=annulus(292,112,(x-34,0,0),68,"X")
        # 12 axial windows cut through the bell.
        for j,a in enumerate(np.linspace(0,math.tau,12,endpoint=False)):
            cutter=box(88,62,128,(x,235,0),12)
            cutter=_rotate_x(cutter,math.degrees(a))
            bell=bell.cut(cutter)
        bell=bell.clean()
        add_cad(parts,f"GEO_{side}_Vented_end_bell",bell,0,"housing",
                explode=(sgn*150,0,0),role="Vented machined end bell with actual cut-through windows",tol=.024,ang=.052,
                tags=("boolean-windows","annular-casting"))
        trim=flange_x(x+sgn*42,312,284,18,bolt_radius=270,bolt_count=18,bolt_hole=5.2)
        add_cad(parts,f"GEO_{side}_Outer_trim_flange",trim,1,"housing",explode=(sgn*175,0,0),
                role="Bolted end-bell trim flange")
        bolt_circle_x(parts,f"GEO_{side}_Outer_socket",x+sgn*54,270,18,diam=8,length=22,phase=math.pi/18,explode=sgn*190)

    # Layered open housing hoops give the machine the reference's heavy
    # multi-shell silhouette while leaving most copper visible between bands.
    for bi,x in enumerate((-265,-175,-85,85,175,265)):
        hoop=annulus(286,256,(x-12,0,0),24,"X")
        # radial service/vent notches make each hoop a manufactured frame,
        # not a featureless ring.
        for a in np.linspace(0,math.tau,12,endpoint=False):
            notch=box(30,46,68,(x,273,0),7)
            notch=_rotate_x(notch,math.degrees(a))
            hoop=hoop.cut(notch)
        add_cad(parts,f"GEO_Housing_hoop_{bi+1:02}",hoop.clean(),0 if bi in (0,5) else 1,"housing",
                role="Windowed structural housing hoop around exposed stator",
                explode=(0,0,48+bi*5),tol=.028,ang=.060)
        bolt_circle_x(parts,f"GEO_Hoop_socket_{bi+1:02}",x+13,272,12,diam=4.5,length=14,phase=(bi%2)*math.pi/12)
    # Eight longitudinal cage rails mechanically tie the hoop stack together.
    for ri,a in enumerate(np.linspace(0,math.tau,8,endpoint=False)):
        rail=box(550,18,28,(0,276,0),4)
        rail=_rotate_x(rail,math.degrees(a))
        add_cad(parts,f"GEO_Longitudinal_housing_rail_{ri:02}",rail,0,"housing",
                role="Longitudinal stator housing rail")
        for x in (-250,-165,-80,80,165,250):
            y,z=276*math.cos(a),276*math.sin(a)
            add_cad(parts,f"GEO_Rail_clamp_{ri:02}_{x:+}",box(34,34,38,(x,y,z),4).rotate((x,0,0),(x+1,0,0),math.degrees(a)),
                    3,"housing",role="Rail-to-hoop clamp block")

    # Actual laminated stator stack with teeth and exposed slots.
    stator_x0=-245;stator_len=490
    # 17 discrete lamination packs show the stack edge in geometry.
    for layer,x in enumerate(np.linspace(stator_x0+8,stator_x0+stator_len-8,17)):
        core=annulus(220,166,(x-5,0,0),10,"X")
        for j,a in enumerate(np.linspace(0,math.tau,36,endpoint=False)):
            tooth=sector(252,218,x-5,x+5,a-.050,a+.050)
            core=core.fuse(tooth)
        add_cad(parts,f"GEO_Lamination_pack_{layer+1:02}",core.clean(),2,"stator_core",
                explode=(0,0,0),role="Electrical-steel lamination pack with 36 actual teeth",tol=.035,ang=.070)

    # Slot liners, copper hairpin legs and end turns.
    for j,a in enumerate(np.linspace(0,math.tau,36,endpoint=False)):
        ca,sa=math.cos(a),math.sin(a)
        # Two rectangular-ish enamel bars per tooth position.
        for k,rr in enumerate((226,240)):
            leg=box(430,12,20,(0,rr,0),3)
            leg=_rotate_x(leg,math.degrees(a))
            add_cad(parts,f"GEO_Copper_leg_{j:02}_{k}",leg,4,"windings",
                    explode=(0,25*ca,25*sa),role="Exposed enamelled stator conductor")
        # Rounded end turn built as analytic spline sweep.
        y0,z0=226*ca,226*sa
        a2=a+math.tau/36*.72
        y1,z1=240*math.cos(a2),240*math.sin(a2)
        for end,x in [("rear",-225),("front",225)]:
            pts=[
                (x,y0,z0),
                (x+( -28 if end=="rear" else 28),y0,z0),
                (x+( -46 if end=="rear" else 46),(y0+y1)*.54,(z0+z1)*.54),
                (x+( -28 if end=="rear" else 28),y1,z1),
                (x,y1,z1),
            ]
            pipe(parts,f"GEO_{end}_Hairpin_{j:02}",pts,7.0,4,"windings",
                 "Analytic copper end-turn sweep",explode=(0,22*ca,22*sa))

    # Copper retaining rings and nonconductive wedges.
    for end,x in [("rear",-252),("front",252)]:
        add_cad(parts,f"GEO_{end}_Coil_retainer",annulus(270,252,(x-9,0,0),18,"X"),3,"windings",
                role="Coil end-turn retaining ring",explode=(( -1 if x<0 else 1)*60,0,0))
        bolt_circle_x(parts,f"GEO_{end}_Retainer_bolt",x+( -10 if x<0 else 10),261,18,diam=5,length=16)

    # Rotor shaft, rotor spider and 32 magnet poles.
    shaft_profile=[(-520,52),(-455,52),(-455,68),(-300,68),(-300,88),(300,88),(300,68),(455,68),(455,52),(520,52)]
    shaft=cq.Workplane("XY").polyline(shaft_profile).close().revolve(360,(0,0),(1,0)).val()
    shaft=shaft.cut(annulus(28,0,(-521,0,0),1042,"X"))
    add_cad(parts,"GEO_Stepped_hollow_rotor_shaft",shaft,2,"rotor",motion="rotor",
            role="Revolved hollow stepped rotor shaft",explode=(210,0,0),tol=.022,ang=.052,
            tags=("revolve","bore"))
    spider=annulus(156,90,(-210,0,0),420,"X")
    for j,a in enumerate(np.linspace(0,math.tau,8,endpoint=False)):
        opening=sector(148,104,-212,212,a+.11,a+.62)
        spider=spider.cut(opening)
    add_cad(parts,"GEO_Rotor_spider",spider.clean(),1,"rotor",motion="rotor",
            role="Windowed rotor spider",explode=(120,0,0),tol=.025,ang=.055)

    backiron=annulus(162,151,(-214,0,0),428,"X")
    add_cad(parts,"GEO_Rotor_backiron",backiron,2,"rotor",motion="rotor",
            role="Rotor magnetic backiron",explode=(110,0,0))
    for j,a in enumerate(np.linspace(0,math.tau,32,endpoint=False)):
        magnet=sector(165,153,-202,202,a-.077,a+.077)
        add_cad(parts,f"GEO_Rotor_magnet_{j:02}",magnet,2 if j%2 else 1,"rotor_magnets",motion="rotor",
                role="Concept permanent-magnet pole",explode=(105,18*math.cos(a),18*math.sin(a)),tol=.04,ang=.08)

    # ORBIT-level bearings with torus raceways, balls and cages.
    bearing_x(parts,"GEO_Rear_main_bearing",-326,108,68,42,explode=150,motion_inner="rotor")
    bearing_x(parts,"GEO_Front_main_bearing",284,108,68,42,explode=150,motion_inner="rotor")

    # Bearing covers and retainers with real radial relief pockets.
    for side,x,sgn in [("rear",-360,-1),("front",360,1)]:
        cover=flange_x(x,154,70,24,bolt_radius=132,bolt_count=12,bolt_hole=4.2)
        for a in np.linspace(0,math.tau,8,endpoint=False):
            pocket=sector(126,92,x-13,x+13,a-.13,a+.13)
            cover=cover.cut(pocket)
        add_cad(parts,f"GEO_{side}_Bearing_cover",cover.clean(),0,"bearings",explode=(sgn*205,0,0),
                role="Removable relieved bearing cover with radial inspection pockets",tol=.028,ang=.060)
        add_cad(parts,f"GEO_{side}_Bearing_trim",annulus(92,70,(x-15,0,0),30,"X"),3,"bearings",
                explode=(sgn*210,0,0),role="Bronze bearing cover trim ring")
        bolt_circle_x(parts,f"GEO_{side}_Bearing_cover_screw",x+sgn*14,132,12,diam=6,length=24,explode=sgn*215)

    # Axial tie rods, clamp blocks, nuts and washers.
    for j,a in enumerate(np.linspace(0,math.tau,12,endpoint=False)):
        y,z=282*math.cos(a),282*math.sin(a)
        rod=cylinder(5,700,(-350,y,z),"X")
        add_cad(parts,f"GEO_Tie_rod_{j:02}",rod,2,"housing",role="Through tie rod")
        for side,x in [("rear",-364),("front",350)]:
            washer=annulus(12,5.2,(x,y,z),2.2,"X")
            add_cad(parts,f"GEO_{side}_Tie_washer_{j:02}",washer,2,"fasteners",role="Tie-rod washer")
            nut=cq.Workplane("YZ",origin=(x+( -7 if side=="rear" else 2),y,z)).polygon(6,20).extrude(7).val()
            nut=nut.cut(cylinder(5.2,9,(x+( -8 if side=="rear" else 1),y,z),"X"))
            add_cad(parts,f"GEO_{side}_Tie_nut_{j:02}",nut,2,"fasteners",role="Tie-rod hex nut")

    # Front encoder / resolver housing and inspection pickup.
    encoder=annulus(82,52,(390,0,0),46,"X")
    add_cad(parts,"GEO_Front_encoder_housing",encoder,0,"instrumentation",role="Front encoder/resolver housing",explode=(240,0,0))
    add_cad(parts,"GEO_Front_encoder_cap",flange_x(444,90,48,14,bolt_radius=72,bolt_count=8,bolt_hole=3.2),
            1,"instrumentation",role="Encoder service cap",explode=(265,0,0))
    bolt_circle_x(parts,"GEO_Encoder_cap_screw",452,72,8,diam=4,length=14,explode=275)
    add_cad(parts,"GEO_Speed_pickup_body",box(42,34,88,(332,-248,92),6),0,"instrumentation",
            role="Rotor speed/position pickup housing")
    pipe(parts,"GEO_Speed_pickup_cable",[(332,-265,92),(300,-310,76),(220,-338,48),(150,-320,20)],4.2,5,"service",
         "Position-sensor cable routed to service harness",analytic=True)

    # Terminal/junction box with removable lid, ceramic feedthroughs and cable glands.
    terminal=box(260,220,145,(30,-300,260),16)
    terminal=terminal.cut(box(220,180,120,(30,-300,275),12))
    add_cad(parts,"GEO_Terminal_box",terminal,0,"service",role="Hollow electrical terminal enclosure",explode=(0,-65,70))
    lid=box(238,198,18,(30,-300,341),7)
    add_cad(parts,"GEO_Terminal_lid",lid,1,"service",role="Removable terminal enclosure lid",explode=(0,-80,105))
    for x in (-65,30,125):
        for y in (-375,-225):
            add_cad(parts,f"GEO_Terminal_lid_screw_{x}_{y}",socket_screw_z(x,y,349,22,5),2,"fasteners",
                    role="Terminal lid screw",explode=(0,-80,120))
    for i,x in enumerate((-55,30,115)):
        ins=cylinder(23,42,(x,-414,240),"Y").cut(cylinder(8,44,(x,-415,240),"Y"))
        add_cad(parts,f"GEO_Ceramic_feedthrough_{i}",ins,22,"service",role="Ceramic phase feedthrough")
        pipe(parts,f"GEO_Phase_cable_{i}",[(x,-435,240),(x,-505,220),(x+25,-560,160)],13,5,"service",
             "Heavy phase cable routed from terminal box",analytic=True)
    for i,z in enumerate((205,255,305)):
        gland=annulus(20,10,(-112,-300,z),24,"X")
        add_cad(parts,f"GEO_Cable_gland_{i}",gland,3,"service",role="Threaded service cable gland")

    # Cooling manifold with properly terminated routed tubes.
    manifold=box(180,130,120,(-430,-320,-60),12)
    for z in (-90,-40):
        manifold=manifold.cut(cylinder(13,32,(-446,-320,z),"X"))
    add_cad(parts,"GEO_Cooling_manifold",manifold,0,"service",role="External cooling/service manifold")
    for i,(z,rr) in enumerate([(-95,258),(-55,272),(-15,286)]):
        a=-2.05+i*.08
        y=rr*math.cos(a);zz=rr*math.sin(a)
        pipe(parts,f"GEO_Cooling_line_{i}",[(-430,-385,z),(-365,-400,z+10),(-280,-350,z+28),(-215,y,zz)],6.5,3,"service",
             "Cooling line terminating at stator/end housing")

    # Identification plate.
    plate=box(150,10,62,(30,-414,-245),4)
    add_cad(parts,"GEO_Module_index_plate",plate,1,"markings",role="CYBR GEO module identity plate")

    views={
        "hero":View(az=54,el=18,scale=575,target=(0,-28,-10),title="CYBR GEO / ELECTROMECHANICAL CORE",
                    note="Exposed 36-slot stator, end turns, windowed rotor, real bearing internals and service hardware.",
                    f_stop=11,environment_strength=.32,light_size=1.9),
        "front":View(az=90,el=0,scale=420,target=(0,0,0),projection="orthographic",title="CYBR GEO / END ELEVATION"),
        "side":View(az=0,el=0,scale=420,target=(0,0,0),projection="orthographic",title="CYBR GEO / SIDE ELEVATION"),
        "internal":View(az=52,el=16,scale=495,target=(0,0,0),hide=("housing","service","markings"),title="CYBR GEO / ROTOR + STATOR"),
        "exploded":View(az=42,el=21,scale=900,target=(0,0,0),explode=1,title="CYBR GEO / EXPLODED"),
    }
    return Assembly("portfolio_geo",parts,MATERIALS,views,
                    metadata=_meta("portfolio_geo",{"length":1280.,"width":640.,"height":680.},
                                   ["Reference target: dense black-and-copper motor/generator, exposed windings, layered end-bells and serviceable internals."]),
                    motion_function=_geo_pose)


# ---------------------------------------------------------------------------
# CYBR LIGHT — precision optical bench
# ---------------------------------------------------------------------------

def _breadboard(length=1420,width=500,thick=42):
    board=box(length,width,thick,(0,0,-220),10)
    # actual tapped-hole lattice
    for x in np.arange(-620,621,80):
        for y in np.arange(-180,181,80):
            board=board.cut(cylinder(4.2,thick+6,(x,y,-244),"Z"))
            board=board.cut(cylinder(7.0,4,(x,y,-202),"Z"))
    return board.clean()


def _post_mount(parts,prefix,x,y,height=250):
    base=box(76,76,18,(x,y,-188),7)
    base=base.cut(cylinder(5.2,22,(x,y,-198),"Z"))
    add_cad(parts,prefix+"_Base",base,0,"mounts",role="Optical post base with through bore")
    post=cylinder(12,height,(x,y,-180),"Z")
    add_cad(parts,prefix+"_Ground_post",post,2,"mounts",role="Ground optical support post")
    collar=annulus(25,12.2,(x,y,-180+height*.45),28,"Z")
    collar=collar.cut(cylinder(2.2,52,(x-26,y,-180+height*.45+14),"X"))
    add_cad(parts,prefix+"_Split_collar",collar,1,"mounts",role="Split optical post collar")
    add_cad(parts,prefix+"_Collar_screw",socket_screw_x(x-30,y,-180+height*.45+14,20,4),2,"fasteners",role="Post collar clamp screw")


def build_light():
    parts=[]
    add_cad(parts,"LIGHT_Optical_breadboard",_breadboard(),0,"base",role="Machined optical breadboard with real counterbored hole lattice")
    # two under-frame rails
    for y in (-205,205):
        rail=box(1350,34,50,(0,y,-260),5)
        rail=rail.cut(box(1280,16,24,(0,y,-260),3))
        add_cad(parts,f"LIGHT_Underrail_{'L' if y<0 else 'R'}",rail,0,"base",role="Hollow optical-bench underrail")

    # Emitter assembly: concentric barrels, aperture, heat sink fins and clamp mount.
    x0=-535
    stages=[(-650,-600,112,54,0),(-600,-548,100,48,1),(-548,-500,86,44,3),(-500,-455,70,38,0)]
    for i,(a,b,ro,ri,mat) in enumerate(stages):
        q=annulus(ro,ri,(a,0,0),b-a,"X")
        add_cad(parts,f"LIGHT_Emitter_barrel_{i+1}",q,mat,"optics",role="Emitter barrel / focus stage",explode=(-60,0,0))
    for i,x in enumerate(np.linspace(-590,-515,8)):
        fin=annulus(122,101,(x-3,0,0),6,"X")
        add_cad(parts,f"LIGHT_Emitter_heat_fin_{i:02}",fin,1,"optics",role="Emitter heat-sink fin")
    aperture=annulus(58,16,(-457,0,0),8,"X")
    add_cad(parts,"LIGHT_Emitter_aperture",aperture,0,"optics",role="Emitter aperture plate")
    bolt_circle_x(parts,"LIGHT_Emitter_face_screw",-452,52,8,diam=4,length=12)

    # emitter cradle, post and split clamp
    _post_mount(parts,"LIGHT_Emitter_post",-555,0,190)
    cradle=annulus(126,113,(-585,0,0),58,"X")
    cradle=cradle.cut(box(70,280,70,(-556,0,-110),8))
    add_cad(parts,"LIGHT_Emitter_split_cradle",cradle,0,"mounts",role="Split emitter barrel cradle")
    for y in (-102,102):
        add_cad(parts,f"LIGHT_Emitter_cradle_bolt_{y:+}",socket_screw_z(-555,y,-118,44,6),2,"fasteners",role="Emitter cradle clamp bolt")

    # Prism tower — four ground posts, cross stages, enclosed prism clamp and three-point kinematics.
    for x in (-170,-55):
        for y in (-82,82):
            _post_mount(parts,f"LIGHT_Prism_post_{x}_{y}",x,y,480)
    for z in (80,265):
        stage=box(230,220,24,(-112,0,z),5)
        # central optical clearance
        stage=stage.cut(box(130,130,30,(-112,0,z),8))
        add_cad(parts,f"LIGHT_Prism_stage_{z}",stage,1,"mounts",role="Prism tower cross stage")
    prism=cq.Workplane("XY").polyline([(-72,-58),(75,-58),(0,88)]).close().extrude(92,both=True).val()
    prism=prism.rotate((0,0,0),(0,1,0),90).translate((-112,0,145))
    add_cad(parts,"LIGHT_Optical_prism",prism,6,"glass",role="Triangular optical prism",explode=(0,0,90),tol=.035,ang=.055)
    # prism edge clamps
    for i,(y,z) in enumerate([(-82,90),(82,90),(0,245)]):
        bracket=box(46,34,86,(-112,y,z),6)
        bracket=bracket.cut(cylinder(5.2,50,(-135,y,z),"X"))
        add_cad(parts,f"LIGHT_Prism_clamp_{i}",bracket,0,"mounts",role="Prism edge clamp / kinematic support")
        add_cad(parts,f"LIGHT_Prism_adjuster_{i}",socket_screw_x(-150,y,z,65,6),2,"fasteners",role="Prism kinematic adjuster")

    # Vertical relay optic in a real cage.
    relay_x=50
    lens=cylinder(45,260,(relay_x,0,-35),"Z")
    add_cad(parts,"LIGHT_Relay_glass",lens,6,"glass",role="Vertical relay optic",explode=(0,0,60))
    for z in (-45,205):
        ret=annulus(72,47,(relay_x,0,z),16,"Z")
        add_cad(parts,f"LIGHT_Relay_retainer_{z}",ret,1,"optics",role="Relay optic retaining ring")
        bolt_circle_z(parts,f"LIGHT_Relay_retainer_screw_{z}",z+8,60,8,diam=4,length=12)
    for a in np.linspace(0,math.tau,6,endpoint=False):
        x=relay_x+78*math.cos(a);y=78*math.sin(a)
        add_cad(parts,f"LIGHT_Relay_cage_post_{int(a*1000):04}",cylinder(5,300,(x,y,-55),"Z"),3,"support",role="Relay optic cage post")

    # Kinematic steering mirror on a two-axis gimbal.
    mx=315
    _post_mount(parts,"LIGHT_Mirror_post",mx,0,230)
    outer=annulus(112,91,(mx-14,0,25),28,"X")
    add_cad(parts,"LIGHT_Mirror_outer_gimbal",outer,0,"optics",role="Yaw gimbal ring")
    inner=annulus(88,72,(mx+20,0,25),20,"X")
    add_cad(parts,"LIGHT_Mirror_inner_gimbal",inner,3,"optics",role="Pitch gimbal ring")
    substrate=cylinder(70,10,(mx+42,0,25),"X")
    add_cad(parts,"LIGHT_Steering_mirror_substrate",substrate,1,"optics",role="Precision steering mirror substrate")
    for j,a in enumerate((0,math.pi/2,math.pi,3*math.pi/2)):
        y=118*math.cos(a);z=25+118*math.sin(a)
        add_cad(parts,f"LIGHT_Mirror_adjuster_{j}",socket_screw_x(mx-60,y,z,68,5),2,"fasteners",role="Micrometer-style gimbal adjuster")

    # Output lens: nested retained optics, aperture and mounting cradle.
    ox=550
    for i,(a,b,ro,ri,mat) in enumerate([(470,505,82,44,0),(505,545,106,52,1),(545,585,124,62,3),(585,625,110,56,0)]):
        add_cad(parts,f"LIGHT_Output_barrel_{i}",annulus(ro,ri,(a,0,0),b-a,"X"),mat,"optics",role="Output lens barrel stage",explode=(45,0,0))
    add_cad(parts,"LIGHT_Output_front_glass",cylinder(58,12,(620,0,0),"X"),6,"glass",role="Output optical element",explode=(75,0,0))
    bolt_circle_x(parts,"LIGHT_Output_face_screw",632,92,10,diam=4,length=14,explode=85)
    _post_mount(parts,"LIGHT_Output_post",550,0,190)
    clamp=annulus(132,124,(530,0,0),40,"X")
    clamp=clamp.cut(box(48,300,75,(550,0,-112),8))
    add_cad(parts,"LIGHT_Output_split_cradle",clamp,0,"mounts",role="Split output-optic cradle")

    # Beam path as thin geometry exactly connecting optical stations.
    beam_paths=[
        [(-455,0,0),(-220,0,135),(-112,0,145)],
        [(-112,0,145),(50,0,95),(315,0,25)],
        [(315,0,25),(470,0,0),(620,0,0),(770,0,0)],
    ]
    for i,path in enumerate(beam_paths):
        pipe(parts,f"LIGHT_Beam_{i}",path,2.8,21,"beam","Visible collimated beam path",analytic=True)

    # Cable tray, routed services, and connector blocks.
    tray=box(1240,48,30,(0,-226,-190),5)
    tray=tray.cut(box(1180,28,20,(0,-226,-186),3))
    add_cad(parts,"LIGHT_Cable_tray",tray,0,"service",role="Open cable/service tray")
    cable_paths=[
        [(-560,-220,-185),(-430,-220,-160),(-290,-205,-120),(-170,-110,30)],
        [(20,-220,-185),(80,-215,-145),(250,-210,-120),(315,-110,-20)],
        [(330,-220,-185),(450,-215,-150),(555,-180,-120),(555,-105,-30)],
    ]
    for i,p in enumerate(cable_paths):
        pipe(parts,f"LIGHT_Service_cable_{i}",p,5.5,5,"service","Routed flexible optical-system cable")

    views={
        "hero":View(az=26,el=18,scale=485,target=(0,0,25),title="CYBR LIGHT / PRECISION OPTICAL BENCH",
                    note="Retained optics, kinematic mounts, ground posts, counterbored breadboard and explicit beam path.",
                    f_stop=10,environment_strength=.27,light_size=1.7),
        "side":View(az=0,el=2,scale=390,target=(0,0,20),projection="orthographic",title="CYBR LIGHT / SIDE"),
        "top":View(az=0,el=89,scale=420,target=(0,0,-30),projection="orthographic",title="CYBR LIGHT / TOP"),
        "beam_path":View(az=0,el=0,scale=330,target=(0,0,65),projection="orthographic",hide=("base","service"),title="CYBR LIGHT / OPTICAL PATH"),
        "exploded":View(az=26,el=18,scale=700,target=(0,0,35),explode=1,title="CYBR LIGHT / EXPLODED"),
    }
    return Assembly("portfolio_light",parts,MATERIALS,views,
                    metadata=_meta("portfolio_light",{"length":1420.,"width":500.,"height":760.},
                                   ["Reference target: dense metrology-grade laser bench rather than a few optics on blocks."]))


# ---------------------------------------------------------------------------
# CYBR MATERIALS — retained linear stage + specimen cassettes + handling arm
# ---------------------------------------------------------------------------

def _linear_stage(parts):
    # ROAM-inspired retained dual-guide screw stage, enlarged for the sample machine.
    length=820
    base=box(length+80,390,34,(0,0,-250),8)
    # underside relief pockets while preserving perimeter.
    base=base.cut(box(length-40,270,22,(0,0,-254),6))
    add_cad(parts,"MAT_Stage_base",base,1,"base",role="Machined linear-stage base with underside relief")
    for side in (-1,1):
        x=side*(length/2)
        ped=box(70,330,160,(x,0,-155),12)
        for y in (-110,110):
            ped=ped.cut(cylinder(12.2,76,(x-38,y,-150),"X"))
        ped=ped.cut(cylinder(10.2,76,(x-38,0,-150),"X"))
        ped=ped.cut(cylinder(28,28,(x-14,0,-150),"X"))
        add_cad(parts,f"MAT_End_pedestal_{side:+}",ped,0,"stage",role="Split-service linear-stage end pedestal")
        for y in (-145,145):
            add_cad(parts,f"MAT_Pedestal_bolt_{side:+}_{y:+}",socket_screw_z(x,y,-267,48,7),2,"fasteners",role="End pedestal base bolt")
    for y in (-110,110):
        add_cad(parts,f"MAT_Ground_guide_{y:+}",cylinder(12,length-45,(-length/2+22,y,-150),"X"),2,"stage",role="24 mm ground guide shaft")
    # split carriage
    lower=box(260,330,44,(0,0,-176),10);upper=box(260,330,64,(0,0,-122),10)
    for y in (-110,110):
        seat=cylinder(22,180,(-90,y,-150),"X")
        clear=cylinder(12.5,270,(-135,y,-150),"X")
        lower=lower.cut(seat).cut(clear);upper=upper.cut(seat).cut(clear)
        add_cad(parts,f"MAT_Linear_bearing_{y:+}",annulus(22,12.1,(-90,y,-150),180,"X"),2,"stage",
                role="Retained linear-bearing cartridge")
    nutseat=cq.Workplane("YZ",origin=(-25,0,-150)).polygon(6,40).extrude(50).val()
    lower=lower.cut(nutseat).cut(cylinder(10.5,270,(-135,0,-150),"X"))
    upper=upper.cut(nutseat).cut(cylinder(10.5,270,(-135,0,-150),"X"))
    add_cad(parts,"MAT_Carriage_lower",lower,0,"carriage",role="Split lower carriage with bearing/nut seats")
    add_cad(parts,"MAT_Carriage_upper",upper,0,"carriage",role="Removable upper carriage cap")
    for x in (-100,100):
        for y in (-145,145):
            add_cad(parts,f"MAT_Carriage_bolt_{x}_{y}",socket_screw_z(x,y,-205,105,7),2,"fasteners",
                    role="Split carriage through fastener")
    nut=cq.Workplane("YZ",origin=(-25,0,-150)).polygon(6,38).extrude(48).val().cut(cylinder(10.2,52,(-27,0,-150),"X"))
    add_cad(parts,"MAT_Captive_feed_nut",nut,3,"stage",role="Captured bronze feed nut")
    screw=cylinder(10,length+110,(-length/2-45,0,-150),"X")
    add_cad(parts,"MAT_Feed_screw",screw,2,"stage",role="Retained manual feed screw")
    bearing_x(parts,"MAT_Left_feed_bearing",-438,31,10.2,20,explode=40)
    bearing_x(parts,"MAT_Right_feed_bearing",418,31,10.2,20,explode=40)
    knob=fluted_knob_x(445,0,-150,42,10.4,28,40)
    add_cad(parts,"MAT_Feed_handwheel",knob,0,"controls",role="Fluted feed handwheel")
    add_cad(parts,"MAT_Handwheel_lock",socket_screw_z(465,-28,-150,48,5),2,"fasteners",role="Flat-engaging handwheel lock screw")


def _sample_profile(material_index,i,x):
    # tongue-and-groove / eased-edge flooring-like profile as actual geometry.
    width=260-i*10;height=400-i*20;thick=30
    q=box(thick,width,height,(x,0,80+height/2),3)
    # long shallow edge reliefs and tongue/groove.
    q=q.cut(box(thick+4,8,height-28,(x,-width/2+7,80+height/2),2))
    if i in (1,2,3):
        tongue=box(thick,12,height-70,(x,width/2+4,80+height/2),2)
        q=q.fuse(tongue)
        q=q.cut(box(thick+4,8,height-70,(x,-width/2+4,80+height/2),2))
    # bevel exposed vertical edges.
    try:q=cq.Workplane(obj=q).edges("|Z").chamfer(1.5).val()
    except Exception:pass
    return q


def build_materials():
    parts=[]
    _linear_stage(parts)

    # Replaceable fixture deck on carriage.
    deck=box(620,360,28,(40,0,-70),8)
    for x in np.arange(-220,301,80):
        for y in np.arange(-120,121,80):
            deck=deck.cut(cylinder(5.2,34,(x,y,-88),"Z"))
    add_cad(parts,"MAT_Fixture_deck",deck,1,"carriage",role="Replaceable fixture deck with real mounting grid")

    sample_defs=[
        (-190,14,"Dark_stone"),(-115,12,"Oak"),(-35,12,"Walnut"),(50,13,"Maple"),(135,22,"Ceramic"),(215,15,"Light_stone")
    ]
    for i,(x,mat,label) in enumerate(sample_defs):
        q=_sample_profile(mat,i,x)
        add_cad(parts,f"MAT_Sample_{i+1}_{label}",q,mat,"samples",
                role=f"{label.replace('_',' ')} specimen cassette",explode=(0,65+i*8,80+i*16))
        back=box(18,290-i*10,440-i*20,(x+25,0,275-i*10),4)
        back=back.cut(box(22,230-i*10,360-i*20,(x+25,0,275-i*10),3))
        add_cad(parts,f"MAT_Cassette_frame_{i+1}",back,0,"samples",role="Metal-backed removable sample cassette",
                explode=(0,55+i*7,70+i*14))
        # bottom and top clamps
        add_cad(parts,f"MAT_Sample_bottom_clamp_{i+1}",box(72,300-i*10,30,(x,-2,58),4),0,"mounts",
                role="Non-destructive lower sample clamp")
        add_cad(parts,f"MAT_Sample_top_clamp_{i+1}",box(68,285-i*10,26,(x,-2,490-i*20),4),3,"mounts",
                role="Upper sample retaining clamp",explode=(0,30,40))
        for y in (-125+i*4,125-i*4):
            add_cad(parts,f"MAT_Sample_clamp_screw_{i+1}_{int(y)}",socket_screw_z(x,y,510-i*20,42,5),2,"fasteners",
                    role="Sample cassette clamp screw",explode=(0,30,50))

    # Rear inspection wheel with actual involute perimeter and bearing hub.
    gear=involute_spur_gear(72,3.1,24,bore=76,origin=(305,0,110),backlash=.10).val()
    add_cad(parts,"MAT_Inspection_index_wheel",gear,1,"mechanism",role="Large indexed inspection wheel",tol=.025,ang=.055)
    hub=annulus(62,30,(281,0,110),48,"X")
    add_cad(parts,"MAT_Inspection_hub",hub,3,"mechanism",role="Inspection wheel hub")
    bearing_x(parts,"MAT_Inspection_bearing",276,46,30,28,explode=55)
    bolt_circle_x(parts,"MAT_Inspection_hub_bolt",334,48,8,diam=5,length=18)

    # Articulated material handling arm built from hollow machined links with real pivot hardware.
    joints=[
        np.array([310.,205.,-40.]),np.array([385.,205.,245.]),np.array([195.,115.,500.]),np.array([25.,60.,405.])
    ]
    for i,(a,b) in enumerate(zip(joints[:-1],joints[1:])):
        d=b-a;L=np.linalg.norm(d);mid=(a+b)/2
        # capsule side plates separated by spacers form a serviceable hollow link.
        for off in (-20,20):
            aa=a+np.array([0,off,0]);bb=b+np.array([0,off,0])
            link=cylinder_between(aa,bb,13)
            add_cad(parts,f"MAT_Arm_link_{i+1}_{off:+}",link,0,"arm",role="Twin structural handler link")
        # transverse spacers
        for t in (.22,.50,.78):
            p=a+d*t
            add_cad(parts,f"MAT_Arm_spacer_{i+1}_{int(t*100)}",cylinder(8,40,(p[0],p[1]-20,p[2]),"Y"),1,"arm",
                    role="Arm crush/compression spacer")
        # bearing joint at child
        axis_origin=(b[0]-26,b[1],b[2])
        add_cad(parts,f"MAT_Arm_joint_shell_{i+1}",annulus(42,22,axis_origin,52,"X"),0,"arm",role="Handler arm joint shell")
        bearing_x(parts,f"MAT_Arm_joint_bearing_{i+1}",b[0]-18,31,18,16,explode=18)
        add_cad(parts,f"MAT_Arm_joint_shaft_{i+1}",cylinder(18,72,(b[0]-36,b[1],b[2]),"X"),2,"arm",role="Handler pivot shaft")
        for x in (b[0]-38,b[0]+28):
            add_cad(parts,f"MAT_Arm_joint_retainer_{i+1}_{int(x)}",annulus(28,18.2,(x,b[1],b[2]),8,"X"),3,"arm",role="Pivot thrust/retainer washer")

    # Clamp head with screw-driven opposed pads.
    head=joints[-1]
    body=box(160,92,92,head,10)
    body=body.cut(box(108,100,50,(head[0],head[1],head[2]),6))
    add_cad(parts,"MAT_Clamp_body",body,0,"arm",role="Hollow handling clamp body")
    screw=cylinder(7,180,(head[0]-90,head[1],head[2]),"X")
    add_cad(parts,"MAT_Clamp_screw",screw,2,"arm",role="Opposed sample clamp screw")
    for side in (-1,1):
        pad=box(24,76,58,(head[0]+side*52,head[1],head[2]),5)
        add_cad(parts,f"MAT_Clamp_pad_{side:+}",pad,5,"arm",role="Elastomer-lined sample clamp jaw")
    knob=fluted_knob_x(head[0]-110,head[1],head[2],24,7.2,14,24)
    add_cad(parts,"MAT_Clamp_knob",knob,0,"controls",role="Manual clamp input knob")

    # Flexible cable chain along carriage travel.
    for i,x in enumerate(np.linspace(-360,260,25)):
        z=-205+20*math.sin(i*.28)
        link=box(34,30,24,(x,178,z),4)
        link=link.cut(box(20,34,12,(x,178,z),3))
        add_cad(parts,f"MAT_Cable_chain_link_{i:02}",link,5,"service",role="Articulated cable-chain link")
    pipe(parts,"MAT_Arm_service_hose",[(-280,178,-190),(-100,175,-160),(150,170,-120),(310,205,-10),(385,205,245),(195,115,500)],6.0,5,"service",
         "Flexible service hose routed to handler")

    views={
        "hero":View(az=34,el=19,scale=515,target=(0,0,95),title="CYBR MATERIALS / PRECISION SAMPLE MACHINE",
                    note="ROAM-derived retained feed stage, real specimen cassettes, indexed inspection wheel and serviceable handler.",
                    f_stop=11,environment_strength=.31,light_size=1.9),
        "front":View(az=0,el=0,scale=440,target=(0,0,80),projection="orthographic",title="CYBR MATERIALS / FRONT"),
        "side":View(az=90,el=0,scale=440,target=(0,0,80),projection="orthographic",title="CYBR MATERIALS / SIDE"),
        "top":View(az=0,el=89,scale=410,target=(0,0,-90),projection="orthographic",title="CYBR MATERIALS / TOP"),
        "exploded":View(az=36,el=19,scale=720,target=(0,0,95),explode=1,title="CYBR MATERIALS / EXPLODED"),
    }
    return Assembly("portfolio_materials",parts,MATERIALS,views,
                    metadata=_meta("portfolio_materials",{"length":980.,"depth":640.,"height":720.},
                                   ["Reference target: material specimens are removable machine cassettes, not boards placed on a pedestal."]))
