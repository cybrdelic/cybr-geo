"""Final environment/process portfolio modules.

CYBR SCENES is not approximated with cones and spheres here. The environment
construction follows the same hybrid strategy as the actual cybr-scenes desert
hot springs work: dense displaced terrain, irregular water surfaces, implicit
closed rocks and connected vegetation. Mechanical containment follows ORBIT /
ROAM conventions with separable BRep hardware and routed services.
"""
from __future__ import annotations

import math
import numpy as np
import cadquery as cq
import trimesh

from mechanism_lab.core import Assembly, View

from .portfolio_final_common import (
    MATERIALS, box, cylinder, annulus, sphere, add_cad, add_mesh, cylinder_between,
    socket_screw_x, socket_screw_z, fluted_knob_x, bolt_circle_z, pipe,
    terrain_mesh, water_pool_mesh, rock_field, tree_mesh, shrub_mesh,
    join_mesh, mesh_normals, ribbon_surface, implicit_rock_template, transform_mesh,
)


REFERENCE_ID="portfolio-reference-sheets-2026-10-04-final"


def _meta(name,dimensions,notes):
    return dict(
        truth_intent="concept",
        fidelity="final authored portfolio-machine geometry using CYBR GEO / CYBR SCENES construction",
        reference_id=REFERENCE_ID,
        concept_dimensions_mm=dimensions,
        reference_notes=list(notes),
        assumptions=[
            "Portfolio concept object; no fabrication, geotechnical, optical or process qualification is asserted.",
            "Environment geometry is authored rather than surveyed or photogrammetric.",
            "No image-generated mesh, photographic billboard vegetation or image projection is used.",
        ],
    )


def _hemisphere_shell(radius,wall,z0=0):
    outer=cq.Solid.makeSphere(radius,cq.Vector(0,0,z0),cq.Vector(0,0,1),0,90,360)
    inner=cq.Solid.makeSphere(radius-wall,cq.Vector(0,0,z0),cq.Vector(0,0,1),0,90,360)
    return outer.cut(inner)


def _circular_machine_base(parts,prefix,radius=500,z0=0,height=190):
    # Four nested annular structural levels.
    add_cad(parts,f"{prefix}_Lower_plinth",cylinder(radius,height*.34,(0,0,z0),"Z"),0,"base",
            role="Circular machined lower plinth",explode=(0,0,-90))
    add_cad(parts,f"{prefix}_Lower_trim",annulus(radius+12,radius-18,(0,0,z0+height*.30),18,"Z"),3,"base",
            role="Warm metal lower trim ring",explode=(0,0,-72))
    add_cad(parts,f"{prefix}_Service_ring",annulus(radius*.98,radius*.70,(0,0,z0+height*.38),height*.28,"Z"),1,"base",
            role="Open annular service chassis",explode=(0,0,-55))
    add_cad(parts,f"{prefix}_Upper_deck",annulus(radius*.94,radius*.31,(0,0,z0+height*.66),height*.34,"Z"),0,"base",
            role="Upper annular mechanism deck",explode=(0,0,-35))
    # 24 individually removable radial service covers.
    for i,a in enumerate(np.linspace(0,math.tau,24,endpoint=False)):
        r=radius*.84;x=r*math.cos(a);y=r*math.sin(a)
        panel=box(64,32,86,(x,y,z0+height*.53),5).rotate((x,y,z0+height*.53),(x,y,z0+height*.53+1),math.degrees(a))
        add_cad(parts,f"{prefix}_Service_panel_{i:02}",panel,1,"service_panels",
                role="Removable radial service cover",explode=(16*math.cos(a),16*math.sin(a),0))
        add_cad(parts,f"{prefix}_Panel_latch_{i:02}",cylinder(5.5,11,(x,y,z0+height*.82),"Z"),3,"fasteners",
                role="Quarter-turn service latch")
    bolt_circle_z(parts,f"{prefix}_Deck_socket",z0+height+2,radius*.77,24,diam=6,length=18,phase=math.pi/24)
    # Eight isolation feet.
    for i,a in enumerate(np.linspace(0,math.tau,8,endpoint=False)):
        x=radius*.73*math.cos(a);y=radius*.73*math.sin(a)
        add_cad(parts,f"{prefix}_Isolation_foot_{i}",annulus(26,8,(x,y,z0-34),30,"Z"),3,"base",
                role="Machined vibration isolation foot",explode=(0,0,-105))
        add_cad(parts,f"{prefix}_Isolation_pad_{i}",cylinder(30,8,(x,y,z0-42),"Z"),5,"base",
                role="Elastomer isolation pad",explode=(0,0,-113))


def _turntable_bearing(parts,prefix,radius=420,z=175):
    # Actual paired race rings plus 48 rolling elements.
    add_cad(parts,f"{prefix}_Outer_race",annulus(radius+26,radius+9,(0,0,z),18,"Z"),2,"bearings",
            role="Turntable outer bearing race",explode=(0,0,35))
    add_cad(parts,f"{prefix}_Inner_race",annulus(radius-9,radius-26,(0,0,z),18,"Z"),2,"bearings",
            role="Turntable inner bearing race",explode=(0,0,45))
    for i,a in enumerate(np.linspace(0,math.tau,48,endpoint=False)):
        x=radius*math.cos(a);y=radius*math.sin(a)
        add_cad(parts,f"{prefix}_Ball_{i:02}",sphere(8.0,(x,y,z+9)),2,"bearing_balls",
                role="Turntable rolling element",explode=(0,0,42),tol=.06,ang=.12)
    cage=annulus(radius+10,radius-10,(0,0,z+5),8,"Z")
    for a in np.linspace(0,math.tau,48,endpoint=False):
        x=radius*math.cos(a);y=radius*math.sin(a)
        cage=cage.cut(cylinder(8.8,10,(x,y,z+4),"Z"))
    add_cad(parts,f"{prefix}_Pocketed_cage",cage,3,"bearings",role="Pocketed turntable bearing cage",explode=(0,0,40))


def _articulated_arm(parts,prefix,joints,side_sign=1,link_r=12):
    joints=[np.asarray(p,float) for p in joints]
    for i,(a,b) in enumerate(zip(joints[:-1],joints[1:])):
        d=b-a
        # twin tubes + three crush spacers
        offset=np.array([0,18*side_sign,0.])
        for j,off in enumerate((-offset,offset)):
            add_cad(parts,f"{prefix}_Link_{i+1}_{j}",cylinder_between(a+off,b+off,link_r),0,"support",
                    role="Twin structural articulated arm member",explode=(0,18*side_sign,8*i))
        for k,t in enumerate((.22,.50,.78)):
            p=a+d*t
            add_cad(parts,f"{prefix}_Spacer_{i+1}_{k}",cylinder(7,36,(p[0],p[1]-18*side_sign,p[2]),"Y"),1,"support",
                    role="Arm compression spacer")
        # pivot housing at end.
        p=b
        add_cad(parts,f"{prefix}_Joint_shell_{i+1}",annulus(42,21,(p[0]-20,p[1],p[2]),40,"X"),0,"support",
                role="Machined pivot housing")
        add_cad(parts,f"{prefix}_Joint_pin_{i+1}",cylinder(20,64,(p[0]-32,p[1],p[2]),"X"),2,"support",
                role="Hardened pivot shaft")
        for x in (p[0]-34,p[0]+26):
            add_cad(parts,f"{prefix}_Joint_washer_{i+1}_{int(x)}",annulus(29,20.2,(x,p[1],p[2]),7,"X"),3,"support",
                    role="Pivot thrust washer")
        for ang in (0,math.pi):
            y=p[1]+34*math.cos(ang);z=p[2]+34*math.sin(ang)
            add_cad(parts,f"{prefix}_Joint_clamp_{i+1}_{int(ang*10)}",socket_screw_x(p[0]-46,y,z,78,5),2,"fasteners",
                    role="Pivot housing through fastener")
    # hydraulic/service actuator paralleling first link
    a,b=joints[0],joints[1]
    p0=a+np.array([0,-32*side_sign,54]);p1=b+np.array([0,-32*side_sign,-38])
    mid=p0+(p1-p0)*.58
    add_cad(parts,f"{prefix}_Actuator_body",cylinder_between(p0,mid,16),0,"service",role="Articulated-arm actuator body")
    add_cad(parts,f"{prefix}_Actuator_rod",cylinder_between(mid,p1,7),2,"service",role="Polished actuator rod")


def _gravel_field(seed,radius,zfunc,count=1300):
    rng=np.random.default_rng(seed)
    ico=trimesh.creation.icosphere(subdivisions=1,radius=1.0)
    base_v=np.asarray(ico.vertices);base_f=np.asarray(ico.faces)
    items=[]
    for i in range(count):
        # biased toward center/front.
        a=rng.uniform(0,math.tau);r=radius*np.sqrt(rng.uniform(.05,.98))
        x=r*math.cos(a);y=r*math.sin(a)
        rad=np.exp(rng.uniform(np.log(2.0),np.log(10.5)))
        sc=np.array([rad*rng.uniform(.8,1.5),rad*rng.uniform(.55,1.05),rad*rng.uniform(.35,.75)])
        zz=zfunc(x,y)-sc[2]*.35
        ca,sa=math.cos(rng.uniform(0,math.tau)),math.sin(rng.uniform(0,math.tau))
        R=np.array([[ca,-sa,0],[sa,ca,0],[0,0,1]])
        v=(base_v*sc)@R.T+[x,y,zz]
        items.append((v,base_f,mesh_normals(v,base_f)))
    return join_mesh(items)


# ---------------------------------------------------------------------------
# CYBR SCENES
# ---------------------------------------------------------------------------

def build_scenes():
    parts=[]
    base_z=0.;base_h=205.;radius=510.
    _circular_machine_base(parts,"SCN",radius,base_z,base_h)
    _turntable_bearing(parts,"SCN_Turntable",420,base_h-26)

    # Rotating habitat cradle with nested seal and clamp rings.
    for i,(ro,ri,z,mat) in enumerate([
        (455,420,base_h-2,0),(446,425,base_h+18,3),(438,412,base_h+36,1),(430,405,base_h+52,3)
    ]):
        add_cad(parts,f"SCN_Dome_ring_{i}",annulus(ro,ri,(0,0,z),18,"Z"),mat,"dome_mount",
                role="Layered habitat dome clamp/seal ring",explode=(0,0,55+i*10))
    bolt_circle_z(parts,"SCN_Dome_clamp_socket",base_h+72,426,32,diam=5,length=16,phase=math.pi/32,explode=80)

    # Thin real glass shell rather than solid hemisphere.
    dome_z=base_h+72;dome_r=420
    add_cad(parts,"SCN_Optical_dome",_hemisphere_shell(dome_r,9,dome_z),6,"glass",
            role="9 mm concept optical habitat shell",explode=(0,0,130),tol=.085,ang=.10)

    # Actual CYBR-SCENES style dense displaced environment.
    tv,tf,tn=terrain_mesh("desert",radius=382,z0=dome_z+24,res=321,seed=20260914)
    add_mesh(parts,"SCN_Continuous_desert_shelf",tv,tf,tn,11,"environment",
             "Dense displaced mineral terrain adapted from CYBR SCENES construction",explode=(0,0,95),
             tags=("dense-heightfield","actual-surface-relief"))
    # height sampling for gravel / vegetation placement
    xs=np.linspace(-382,382,321);ys=np.linspace(-382,382,321);H=tv[:,2].reshape(321,321)
    def ground(x,y):
        ix=np.clip(np.searchsorted(xs,x),1,len(xs)-1);iy=np.clip(np.searchsorted(ys,y),1,len(ys)-1)
        x0,x1=xs[ix-1],xs[ix];y0,y1=ys[iy-1],ys[iy]
        tx=(x-x0)/(x1-x0);ty=(y-y0)/(y1-y0)
        return (H[iy-1,ix-1]*(1-tx)+H[iy-1,ix]*tx)*(1-ty)+(H[iy,ix-1]*(1-tx)+H[iy,ix]*tx)*ty

    wv,wf,wn=water_pool_mesh(-5,-18,188,140,dome_z+15,nr=40,nt=192,seed=7)
    add_mesh(parts,"SCN_Irregular_spring_water",wv,wf,wn,7,"water",
             "Irregular tessellated spring surface with authored ripple geometry",explode=(0,0,104))

    # Hero outcrop + many grounded rocks use closed non-convex implicit geometry.
    rng=np.random.default_rng(9132026);placements=[]
    hero=[
        (-205,85,ground(-205,85)+45,85,68,115,.2,.10),
        (-165,105,ground(-165,105)+72,62,54,145,-.4,-.08),
        (180,115,ground(180,115)+60,95,78,160,.7,.12),
        (225,95,ground(225,95)+95,62,55,120,-.1,-.10),
        (245,-125,ground(245,-125)+45,62,55,92,.4,.05),
        (-225,-150,ground(-225,-150)+35,54,45,82,-.5,.08),
    ]
    placements.extend(hero)
    for _ in range(58):
        a=rng.uniform(0,math.tau);r=rng.uniform(150,360)
        x=r*math.cos(a);y=r*math.sin(a)
        if ((x+5)/205)**2+((y+18)/155)**2<1.18:continue
        s=np.exp(rng.uniform(np.log(12),np.log(44)))
        placements.append((x,y,ground(x,y)+s*.35,s*rng.uniform(.8,1.35),s*rng.uniform(.65,1.05),s*rng.uniform(.75,1.45),rng.uniform(0,math.tau),rng.uniform(-.25,.25)))
    rock_field(parts,"SCN_Implicit_weathered_rocks",placements,10,seed=300,templates=10,resolution=52,group="environment",explode=(0,0,100))

    # Actual gravel microgeometry concentrated on dry shelves.
    gv,gf,gn=_gravel_field(212,350,ground,count=1500)
    add_mesh(parts,"SCN_Grounded_gravel_field",gv,gf,gn,10,"environment",
             "Physically tessellated grounded gravel clasts",explode=(0,0,96),tags=("microgeometry","no-texture-substitute"))

    # Connected woody shrubs with leaf geometry.
    woods=[];leaves=[]
    plant_positions=[(-290,160,58),(-275,-40,48),(-205,210,62),(255,180,66),(300,20,52),(210,-220,44),(-120,-250,40),(90,245,48),(315,-120,45)]
    for i,(x,y,h) in enumerate(plant_positions):
        w,l=shrub_mesh(700+i,(x,y,ground(x,y)),h)
        woods.append(w);leaves.append(l)
    v,f,n=join_mesh(woods);add_mesh(parts,"SCN_Connected_shrub_branches",v,f,n,19,"vegetation","Connected shrub branch geometry",explode=(0,0,105))
    v,f,n=join_mesh(leaves);add_mesh(parts,"SCN_Actual_shrub_leaves",v,f,n,20,"vegetation","Attached leaf geometry",explode=(0,0,108))

    # Paired articulated dome-service arms.
    _articulated_arm(parts,"SCN_Left_arm",[(-455,310,120),(-500,310,470),(-360,245,710),(-270,190,725)],-1)
    _articulated_arm(parts,"SCN_Right_arm",[(455,-310,120),(500,-310,470),(365,-245,705),(275,-190,720)],1)

    # Terminated services from base to arm/dome.
    for i,z in enumerate((92,118,144,170)):
        pipe(parts,f"SCN_Service_line_{i}",[(470,-265,z),(540,-250,z+8),(515,-120,z+30),(438,-65,dome_z+55+i*4)],5.2,3,"service",
             "Power/fluid/data line terminating at habitat ring")
    # local instrumentation pods around rim
    for i,a in enumerate(np.linspace(0,math.tau,12,endpoint=False)):
        x=392*math.cos(a);y=392*math.sin(a)
        pod=box(34,26,44,(x,y,dome_z+66),5).rotate((x,y,dome_z+66),(x,y,dome_z+67),math.degrees(a))
        add_cad(parts,f"SCN_Rim_sensor_{i:02}",pod,0,"instrumentation",role="Habitat rim sensor / utility pod")

    views={
        "hero":View(az=40,el=21,scale=690,target=(0,0,410),title="CYBR SCENES / CONTAINED WORLD",
                    note="Dense CYBR-SCENES terrain and vegetation inside a serviceable mechanical habitat.",
                    f_stop=12,environment_strength=.30,light_size=2.0),
        "internal":View(az=40,el=23,scale=620,target=(0,0,410),hide=("glass",),title="CYBR SCENES / WORLD GEOMETRY"),
        "front":View(az=0,el=6,scale=620,target=(0,0,405),projection="orthographic",title="CYBR SCENES / FRONT"),
        "top":View(az=0,el=89,scale=570,target=(0,0,340),projection="orthographic",title="CYBR SCENES / TOP"),
        "exploded":View(az=43,el=22,scale=880,target=(0,0,405),explode=1,title="CYBR SCENES / EXPLODED"),
    }
    return Assembly("portfolio_scenes",parts,MATERIALS,views,
                    metadata=_meta("portfolio_scenes",{"diameter":1020.,"height":880.,"dome_radius":420.},
                                   ["Reference target uses the actual Desert Hot Springs visual language, not primitive mesa stand-ins."]))


# ---------------------------------------------------------------------------
# CYBR FOREST
# ---------------------------------------------------------------------------

def build_forest():
    parts=[]
    base_z=-120;base_h=190;radius=455
    _circular_machine_base(parts,"FOR",radius,base_z,base_h)
    _turntable_bearing(parts,"FOR_Turntable",374,base_z+base_h-28)
    vessel_z=base_z+base_h+58
    # lower cylindrical shell plus hemispherical roof, both thin shells.
    outer=cylinder(392,390,(0,0,vessel_z),"Z")
    inner=cylinder(383,392,(0,0,vessel_z-1),"Z")
    lower=outer.cut(inner)
    add_cad(parts,"FOR_Cylindrical_glass_shell",lower,6,"glass",role="9 mm sealed biome vessel wall",explode=(0,0,110),tol=.085,ang=.10)
    add_cad(parts,"FOR_Upper_glass_dome",_hemisphere_shell(392,9,vessel_z+390),6,"glass",
            role="Sealed upper biome dome",explode=(0,0,140),tol=.085,ang=.10)
    for i,(ro,ri,z,mat) in enumerate([(420,392,vessel_z-22,0),(410,388,vessel_z-4,3),(405,386,vessel_z+12,1)]):
        add_cad(parts,f"FOR_Vessel_clamp_{i}",annulus(ro,ri,(0,0,z),16,"Z"),mat,"dome_mount",
                role="Biome vessel clamp/seal ring",explode=(0,0,55+i*9))
    bolt_circle_z(parts,"FOR_Vessel_socket",vessel_z+28,397,28,diam=5,length=16,phase=math.pi/28)

    # External tie posts and top clamps.
    for i,a in enumerate(np.linspace(0,math.tau,12,endpoint=False)):
        x=418*math.cos(a);y=418*math.sin(a)
        add_cad(parts,f"FOR_Tie_post_{i:02}",cylinder(7,500,(x,y,vessel_z-10),"Z"),3,"support",
                role="External vessel tie/support post")
        add_cad(parts,f"FOR_Top_clamp_{i:02}",annulus(18,7.2,(x,y,vessel_z+482),18,"Z"),1,"support",
                role="Upper vessel post clamp")

    # Dense authored forest terrain.
    tv,tf,tn=terrain_mesh("forest",radius=350,z0=vessel_z+18,res=321,seed=441)
    add_mesh(parts,"FOR_Continuous_forest_terrain",tv,tf,tn,18,"environment",
             "Dense eroded forest terrain / moss substrate",explode=(0,0,85),tags=("dense-heightfield","stream-ravine"))
    xs=np.linspace(-350,350,321);ys=np.linspace(-350,350,321);H=tv[:,2].reshape(321,321)
    def ground(x,y):
        ix=np.clip(np.searchsorted(xs,x),1,len(xs)-1);iy=np.clip(np.searchsorted(ys,y),1,len(ys)-1)
        x0,x1=xs[ix-1],xs[ix];y0,y1=ys[iy-1],ys[iy]
        tx=(x-x0)/(x1-x0);ty=(y-y0)/(y1-y0)
        return (H[iy-1,ix-1]*(1-tx)+H[iy-1,ix]*tx)*(1-ty)+(H[iy,ix-1]*(1-tx)+H[iy,ix]*tx)*ty

    # Waterfall and stream are tessellated ribbon surfaces with flow undulation.
    rv,rf,rn=ribbon_surface([(35,120,vessel_z+455),(28,110,vessel_z+380),(10,90,vessel_z+300),(-20,65,vessel_z+235),(-45,35,vessel_z+205)],50,segments=130,cross=18,waves=2.2)
    add_mesh(parts,"FOR_Waterfall_surface",rv,rf,rn,7,"water","Tessellated waterfall sheet",explode=(0,0,90))
    rv,rf,rn=ribbon_surface([(-45,35,vessel_z+205),(-90,5,vessel_z+185),(-150,-25,vessel_z+165),(-220,-45,vessel_z+150),(-285,-20,vessel_z+145)],62,segments=150,cross=20,waves=1.4)
    add_mesh(parts,"FOR_Stream_surface",rv,rf,rn,7,"water","Tessellated forest stream",explode=(0,0,88))

    # Closed implicit rock field.
    rng=np.random.default_rng(731);placements=[]
    for _ in range(44):
        a=rng.uniform(0,math.tau);r=rng.uniform(80,330);x=r*math.cos(a);y=r*math.sin(a)
        s=np.exp(rng.uniform(np.log(16),np.log(58)))
        placements.append((x,y,ground(x,y)+s*.30,s*rng.uniform(.8,1.3),s*rng.uniform(.7,1.15),s*rng.uniform(.75,1.4),rng.uniform(0,math.tau),rng.uniform(-.28,.28)))
    rock_field(parts,"FOR_Implicit_forest_rocks",placements,9,seed=912,templates=9,resolution=50,group="environment",explode=(0,0,90))

    # Six genuinely branched trees with attached low-poly foliage clusters.
    tree_specs=[
        (-145,25,300,14,32),(115,65,260,13,28),(15,-135,225,12,26),(220,-45,195,10,23),(-235,-80,180,9,21),(65,190,165,8,20)
    ]
    woods=[];fols=[]
    for i,(x,y,h,r,lr) in enumerate(tree_specs):
        w,l=tree_mesh(800+i,(x,y,ground(x,y)-4),h,r,levels=3,leaf=True,leaf_radius=lr)
        woods.append(w);fols.append(l)
    v,f,n=join_mesh(woods);add_mesh(parts,"FOR_Connected_tree_wood",v,f,n,16,"vegetation","Connected trunks and recursively branched limbs",explode=(0,0,100))
    v,f,n=join_mesh(fols);add_mesh(parts,"FOR_Attached_foliage_clusters",v,f,n,17,"vegetation","Foliage attached to terminal branches",explode=(0,0,105))

    # Moss and understory as grounded low-poly geometry.
    ico=trimesh.creation.icosphere(subdivisions=1,radius=1.0)
    rng=np.random.default_rng(1001);moss=[]
    for i in range(430):
        a=rng.uniform(0,math.tau);r=330*np.sqrt(rng.uniform(.04,.98));x=r*math.cos(a);y=r*math.sin(a)
        rad=rng.uniform(4,13);p=np.array([x,y,ground(x,y)+rad*.15])
        sc=np.array([rad*rng.uniform(.8,1.3),rad*rng.uniform(.8,1.3),rad*rng.uniform(.25,.55)])
        v=np.asarray(ico.vertices)*sc+p;f=np.asarray(ico.faces);moss.append((v,f,mesh_normals(v,f)))
    v,f,n=join_mesh(moss);add_mesh(parts,"FOR_Grounded_moss_clumps",v,f,n,18,"vegetation","Grounded moss and understory clumps",explode=(0,0,88))

    # Four complete life-support columns with removable caps, filters and plumbing.
    stations=[(-430,-245),(430,245),(430,-245),(-430,245)]
    for i,(x,y) in enumerate(stations):
        body=annulus(48,39,(x,y,base_z+65),310,"Z")
        add_cad(parts,f"FOR_Filter_body_{i}",body,0,"life_support",role="Hollow life-support filter/pump canister")
        add_cad(parts,f"FOR_Filter_cap_{i}",annulus(56,18,(x,y,base_z+365),26,"Z"),3,"life_support",role="Removable filter service cap")
        bolt_circle_z(parts,f"FOR_Filter_cap_screw_{i}",base_z+392,42,6,diam=4,length=13,phase=i*.11)
        # inner filter stack visible in internal view
        for j,z in enumerate(np.linspace(base_z+95,base_z+330,6)):
            add_cad(parts,f"FOR_Filter_disc_{i}_{j}",annulus(36,12,(x,y,z),10,"Z"),1 if j%2 else 22,"life_support",role="Filter / pump cartridge disc")
    lines=[
        [(-430,-245,base_z+100),(-500,-190,base_z+95),(-470,-80,base_z+125),(-390,-25,vessel_z+20)],
        [(430,245,base_z+130),(500,190,base_z+130),(470,85,base_z+150),(390,35,vessel_z+35)],
        [(430,-245,base_z+170),(500,-190,base_z+165),(470,-90,base_z+185),(390,-40,vessel_z+50)],
        [(-430,245,base_z+200),(-500,190,base_z+190),(-470,90,base_z+205),(-390,40,vessel_z+65)],
    ]
    for i,p in enumerate(lines):pipe(parts,f"FOR_Life_support_line_{i}",p,7,3,"life_support","Filtration/pump line terminating at biome vessel")

    _articulated_arm(parts,"FOR_Inspection_arm",[(-420,285,base_z+70),(-475,285,vessel_z+300),(-310,235,vessel_z+625),(-155,150,vessel_z+645)],-1,11)

    views={
        "hero":View(az=36,el=17,scale=640,target=(0,0,vessel_z+300),title="CYBR FOREST / SEALED BIOME",
                    note="Dense terrain, connected tree geometry, water and serviceable life support inside a mechanical vessel.",
                    f_stop=12,environment_strength=.30,light_size=2.0),
        "internal":View(az=36,el=18,scale=590,target=(0,0,vessel_z+290),hide=("glass",),title="CYBR FOREST / BIOME GEOMETRY"),
        "front":View(az=0,el=3,scale=580,target=(0,0,vessel_z+300),projection="orthographic",title="CYBR FOREST / FRONT"),
        "top":View(az=0,el=89,scale=510,target=(0,0,vessel_z+235),projection="orthographic",title="CYBR FOREST / TOP"),
        "exploded":View(az=38,el=18,scale=850,target=(0,0,vessel_z+300),explode=1,title="CYBR FOREST / EXPLODED"),
    }
    return Assembly("portfolio_forest",parts,MATERIALS,views,
                    metadata=_meta("portfolio_forest",{"diameter":910.,"height":960.},
                                   ["Reference target: a living biome with actual branches, rock and terrain geometry supported by visible mechanical life-support systems."]))


# ---------------------------------------------------------------------------
# CYBR ELEMENTS
# ---------------------------------------------------------------------------

def _frame_rail_between(a,b,width=38,mat=0):
    return cylinder_between(a,b,width/2)


def build_elements():
    parts=[]
    cube=820.;half=cube/2;base_z=-455
    # machine base with open service chassis
    add_cad(parts,"ELM_Base_lower",box(930,930,120,(0,0,base_z+60),18),0,"base",role="Machined containment-machine base",explode=(0,0,-80))
    lower=box(850,850,68,(0,0,base_z+145),12).cut(box(760,760,72,(0,0,base_z+145),8))
    add_cad(parts,"ELM_Base_service_frame",lower,1,"base",role="Open lower service frame",explode=(0,0,-55))
    # corner feet
    for x in (-395,395):
        for y in (-395,395):
            add_cad(parts,f"ELM_Isolation_{x}_{y}",annulus(28,9,(x,y,base_z-24),30,"Z"),3,"base",role="Isolation foot",explode=(0,0,-95))

    # Six separate containment panels and structural corner posts.
    panel=10
    for name,shape in [
        ("front",box((cube, panel, cube),(0,-half,0),0)),
        ("back",box((cube, panel, cube),(0,half,0),0)),
        ("left",box((panel,cube,cube),(-half,0,0),0)),
        ("right",box((panel,cube,cube),(half,0,0),0)),
        ("top",box((cube,cube,panel),(0,0,half),0)),
    ]:
        add_cad(parts,f"ELM_Glass_{name}",shape,6,"glass",role=f"Optical containment {name} panel",explode=({"front":(0,-100,0),"back":(0,100,0),"left":(-100,0,0),"right":(100,0,0),"top":(0,0,100)}[name]))
    # corner posts with actual panel slots.
    for x in (-half-30,half+30):
        for y in (-half-30,half+30):
            post=box(62,62,cube+150,(x,y,-5),7)
            # two orthogonal glass-panel grooves
            post=post.cut(box(14,38,cube+120,(x-np.sign(x)*22,y,-5),2))
            post=post.cut(box(38,14,cube+120,(x,y-np.sign(y)*22,-5),2))
            add_cad(parts,f"ELM_Corner_post_{int(x)}_{int(y)}",post,0,"frame",role="Slotted containment corner extrusion",explode=(np.sign(x)*45,np.sign(y)*45,0))
            for z in (-320,0,320):
                clamp=box(78,78,34,(x,y,z),5)
                add_cad(parts,f"ELM_Corner_clamp_{int(x)}_{int(y)}_{z}",clamp,1,"frame",role="Machined panel/corner clamp")
                add_cad(parts,f"ELM_Clamp_socket_{int(x)}_{int(y)}_{z}",socket_screw_z(x,y,z+24,24,5),2,"fasteners",role="Corner clamp socket screw")

    # top/bottom structural rails with bolted joints
    rails=[
        ((-half-25,-half-25,-half-35),(half+25,-half-25,-half-35)),
        ((-half-25,half+25,-half-35),(half+25,half+25,-half-35)),
        ((-half-25,-half-25,half+35),(half+25,-half-25,half+35)),
        ((-half-25,half+25,half+35),(half+25,half+25,half+35)),
    ]
    for i,(a,b) in enumerate(rails):
        add_cad(parts,f"ELM_X_frame_rail_{i}",_frame_rail_between(a,b,42),0,"frame",role="Containment structural rail")

    # Top gantry: twin rails, carriage, actuator and suspended probe.
    for y in (-185,185):
        add_cad(parts,f"ELM_Gantry_rail_{y:+}",box(680,34,46,(0,y,505),5),1,"gantry",role="Top process gantry rail")
    carriage=box(190,430,78,(70,0,505),10)
    carriage=carriage.cut(box(128,350,84,(70,0,505),7))
    add_cad(parts,"ELM_Gantry_carriage",carriage,0,"gantry",role="Open gantry carriage")
    add_cad(parts,"ELM_Gantry_leadscrew",cylinder(12,690,(-345,0,505),"X"),2,"gantry",role="Gantry lead screw")
    add_cad(parts,"ELM_Gantry_handwheel",fluted_knob_x(348,0,505,36,12.2,18,32),0,"controls",role="Gantry positioning handwheel")
    add_cad(parts,"ELM_Process_probe",annulus(24,8,(70,0,290),250,"Z"),1,"gantry",role="Vertical process probe / injector")
    add_cad(parts,"ELM_Probe_tip",cylinder(12,45,(70,0,245),"Z"),3,"gantry",role="Process probe tip")

    # Internal fractured process core: many closed non-convex implicit masses.
    rng=np.random.default_rng(822);placements=[]
    for _ in range(42):
        x=rng.normal(0,150);y=rng.normal(0,90);z=rng.uniform(-280,250)
        s=np.exp(rng.uniform(np.log(28),np.log(92)))
        placements.append((x,y,z,s*rng.uniform(.8,1.35),s*rng.uniform(.55,.95),s*rng.uniform(.7,1.45),rng.uniform(0,math.tau),rng.uniform(-.35,.35)))
    rock_field(parts,"ELM_Fractured_basalt_core",placements,9,seed=444,templates=11,resolution=54,group="process_core",explode=(0,0,35))

    # Branching fire/lava channels as connected swept geometry.
    fire=[
        [(-265,-120,305),(-230,-130,220),(-275,-135,120),(-215,-132,15),(-255,-125,-110),(-205,-110,-280)],
        [(-160,-125,285),(-190,-132,205),(-155,-134,115),(-185,-130,30),(-145,-122,-70)],
        [(-330,-100,120),(-275,-115,65),(-315,-122,-25),(-270,-116,-115)],
        [(-90,-118,220),(-125,-128,150),(-90,-130,75),(-130,-124,-20),(-100,-118,-135)],
        [(-225,-88,-20),(-155,-110,-55),(-95,-115,-125)],
    ]
    for i,p in enumerate(fire):
        pipe(parts,f"ELM_Hot_channel_{i}",p,10 if i<2 else 7.5,8,"fire","Branching hot material / lava channel",explode=(-30,0,0),analytic=True)

    # Water as real triangulated sheets/ribbons rather than tubes.
    water_lines=[
        [(245,-145,320),(220,-150,225),(265,-152,115),(220,-148,-10),(252,-135,-280)],
        [(155,-145,285),(185,-150,200),(150,-152,105),(188,-146,10),(155,-135,-230)],
        [(325,-125,180),(282,-145,105),(315,-148,20),(285,-135,-120)],
    ]
    for i,p in enumerate(water_lines):
        v,f,n=ribbon_surface(p,width=64 if i==0 else 48,segments=135,cross=18,waves=3.0)
        add_mesh(parts,f"ELM_Water_sheet_{i}",v,f,n,7,"water","Tessellated water sheet interacting with hot core",explode=(32,0,0))
    # spray/droplet field as actual geometry
    rng=np.random.default_rng(93);ico=trimesh.creation.icosphere(subdivisions=1,radius=1.0);drops=[]
    for i in range(180):
        x=rng.uniform(70,355);y=rng.uniform(-180,-90);z=rng.uniform(-280,330);r=rng.uniform(2.5,8.5)
        sc=np.array([r*rng.uniform(.7,1.0),r*rng.uniform(.7,1.0),r*rng.uniform(1.0,2.2)])
        v=np.asarray(ico.vertices)*sc+[x,y,z];f=np.asarray(ico.faces);drops.append((v,f,mesh_normals(v,f)))
    v,f,n=join_mesh(drops);add_mesh(parts,"ELM_Water_droplets",v,f,n,7,"water","Individual spray/droplet geometry",explode=(35,0,0))

    # steam spirals from the interaction boundary.
    for i,x in enumerate((-90,-20,55,125)):
        pts=[];turns=2.2
        for t in np.linspace(0,1,28):
            a=turns*math.tau*t+i*.8
            pts.append((x+24*math.cos(a)*t,-20+18*math.sin(a)*t,80+300*t))
        pipe(parts,f"ELM_Steam_path_{i}",pts,6.5,22,"steam","Geometric steam/plume streamline",explode=(0,15,35),analytic=False)

    # Side circulation wheel with real perforated handwheel and retained spindle.
    wheel=annulus(132,92,(-475,0,-40),34,"X")
    cuts=[]
    for a in np.linspace(0,math.tau,28,endpoint=False):
        y=113*math.cos(a);z=-40+113*math.sin(a)
        cuts.append(cylinder(5.2,38,(-477,y,z),"X"))
    wheel=wheel.cut(cq.Compound.makeCompound(cuts))
    add_cad(parts,"ELM_Circulation_handwheel",wheel,0,"controls",role="Perforated circulation/service handwheel")
    add_cad(parts,"ELM_Handwheel_hub",annulus(52,18,(-485,0,-40),50,"X"),3,"controls",role="Handwheel spindle hub")
    bearing_like=annulus(34,18.2,(-438,0,-40),28,"X")
    add_cad(parts,"ELM_Handwheel_bearing_cartridge",bearing_like,2,"controls",role="Handwheel bearing cartridge")

    # Real circulation manifold/piping.
    manifold=box(150,310,170,(-490,0,-245),14)
    for z in (-290,-235,-180):
        manifold=manifold.cut(cylinder(13,45,(-515,-80,z),"X"))
    add_cad(parts,"ELM_Cooling_manifold",manifold,0,"service",role="Fluid cooling/circulation manifold")
    lines=[
        [(-510,-120,-290),(-570,-100,-270),(-545,20,-210),(-430,95,-145)],
        [(-510,-40,-235),(-575,-30,-220),(-540,75,-140),(-430,150,-60)],
        [(-510,45,-180),(-570,55,-165),(-525,160,-80),(-410,210,20)],
    ]
    for i,p in enumerate(lines):pipe(parts,f"ELM_Coolant_line_{i}",p,7,3,"service","Cooling line terminating at containment frame/process chamber")

    # Lower service panels and accessible latches.
    for i,x in enumerate(np.linspace(-340,340,9)):
        add_cad(parts,f"ELM_Base_service_panel_{i}",box(64,22,96,(x,-466,base_z+155),5),1,"service_panels",role="Lower service panel")
        add_cad(parts,f"ELM_Base_panel_latch_{i}",cylinder(5,12,(x,-480,base_z+155),"Y"),3,"fasteners",role="Service panel latch")

    views={
        "hero":View(az=35,el=14,scale=600,target=(0,-20,0),title="CYBR ELEMENTS / PROCESS CONTAINMENT",
                    note="Fractured implicit core, branching hot channels, tessellated water sheets, gantry and circulation hardware.",
                    f_stop=11,environment_strength=.29,light_size=2.0),
        "section":View(az=35,el=14,scale=570,target=(0,-15,0),hide=("glass",),title="CYBR ELEMENTS / PROCESS CORE"),
        "front":View(az=0,el=0,scale=500,target=(0,0,0),projection="orthographic",title="CYBR ELEMENTS / FRONT"),
        "side":View(az=90,el=0,scale=500,target=(0,0,0),projection="orthographic",title="CYBR ELEMENTS / SIDE"),
        "exploded":View(az=35,el=15,scale=780,target=(0,0,0),explode=1,title="CYBR ELEMENTS / EXPLODED"),
    }
    return Assembly("portfolio_elements",parts,MATERIALS,views,
                    metadata=_meta("portfolio_elements",{"width":820.,"depth":820.,"height":820.},
                                   ["Reference target: real containment machine plus dense interacting matter geometry, not a cube with a few tubes."]))
