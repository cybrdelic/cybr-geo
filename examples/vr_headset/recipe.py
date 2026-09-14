"""Original printable mobile VR headset using CYBR GEO's native assembly API.

X is wearer left/right, +Y points toward the phone, Z is up. Every rigid
component is analytic CAD. Phone detail and cushion conformance are declared
envelopes; no invented internal electronics are used. Flexible straps are
swept watertight meshes with an explicitly nominal wearing shape.
"""
from __future__ import annotations
from dataclasses import replace
from pathlib import Path
import math
import sys
import numpy as np
import cadquery as cq

sys.path.insert(0, str(Path(__file__).resolve().parent))
from spec import Spec
from mechanism_lab.core import Assembly, Material, View, cad_part, mesh_part
from mechanism_lab.geometry import tube_mesh

MATERIALS = [
    Material("warm ivory PA12 shell", (.57,.59,.55), rough=.43,
             material_source="authored PA12 appearance; no measured BRDF"),
    Material("graphite optical baffle", (.018,.023,.028), rough=.72),
    Material("cushion fabric", (.036,.043,.047), rough=.94),
    Material("titanium phone envelope", (.16,.17,.18), metal=.90, rough=.29),
    Material("display glass", (.006,.010,.016), rough=.075, coat=.6),
    Material("stainless fasteners", (.48,.50,.52), metal=1, rough=.25),
    Material("copper orange anodized marking", (.78,.19,.047), metal=.25, rough=.36),
    Material("optical PMMA", (.99,.99,.99), rough=.002, ior=1.49,
             transmission=1.0, material_source="design n=1.49; no dispersion or measured coating"),
    Material("woven nylon strap", (.025,.031,.035), rough=.92),
    Material("silicone pads", (.08,.09,.09), rough=.87),
]

def rounded_box(w,d,h,center,r=0):
    q=cq.Workplane("XY").box(w,d,h)
    if r:
        q=q.edges("|Y").fillet(r)
    return q.val().translate(center)

def cylinder_y(r,y0,y1,x=0,z=65):
    return cq.Solid.makeCylinder(r,y1-y0,cq.Vector(x,y0,z),cq.Vector(0,1,0))

def annulus(ro,ri,y0,y1,x=0,z=65):
    return cylinder_y(ro,y0,y1,x,z).cut(cylinder_y(ri,y0-.1,y1+.1,x,z))

def x_slot(x0,x1,r,y0,y1,z):
    a=cylinder_y(r,y0,y1,x0,z)
    b=cylinder_y(r,y0,y1,x1,z)
    bridge=rounded_box(abs(x1-x0),y1-y0,2*r,
                       ((x0+x1)/2,(y0+y1)/2,z))
    return a.fuse(b).fuse(bridge).clean()

def lens_solid(s,x=0):
    R=s.lens_radius
    c=R-s.lens_thickness/2
    def ball(y):
        return cq.Solid.makeSphere(R,cq.Vector(x,y,s.center_z),
                                  cq.Vector(0,0,1),-90,90,360)
    return ball(s.focus_offset-c).intersect(
        ball(s.focus_offset+c)).intersect(
        cylinder_y(s.lens_diameter/2,s.focus_offset-s.lens_thickness,
                   s.focus_offset+s.lens_thickness,x,s.center_z)).clean()

def strap_mesh(points,width=25,thickness=1.5,top=False):
    """A solid ribbon with explicit cross sections, caps and consistent winding."""
    p=np.asarray(points,float)
    tangent=np.gradient(p,axis=0)
    tangent/=np.linalg.norm(tangent,axis=1)[:,None]
    across=np.tile([1.,0,0] if top else [0.,0,1.],(len(p),1))
    across-=tangent*np.sum(across*tangent,axis=1)[:,None]
    across/=np.linalg.norm(across,axis=1)[:,None]
    normal=np.cross(tangent,across)
    vertices=[]
    for q,u,v in zip(p,across,normal):
        vertices.extend([q-width*u/2-thickness*v/2,
                         q+width*u/2-thickness*v/2,
                         q+width*u/2+thickness*v/2,
                         q-width*u/2+thickness*v/2])
    faces=[]
    for i in range(len(p)-1):
        for j in range(4):
            a=4*i+j;b=4*i+(j+1)%4;c=b+4;d=a+4
            faces.extend([(a,b,c),(a,c,d)])
    faces.extend([(0,2,1),(0,3,2)])
    n=4*(len(p)-1)
    faces.extend([(n,n+1,n+2),(n,n+2,n+3)])
    return np.asarray(vertices),np.asarray(faces)

def pose(part,t=0.,explode=0.):
    T=np.eye(4)
    # IPD demonstration sweeps the physically machined slot; no oscillating
    # arbitrary decoration. Static default is exactly 64 mm.
    if part.motion in ("left_ipd","right_ipd"):
        sign=-1 if part.motion=="left_ipd" else 1
        # two-second 64 -> 72 -> 58 -> 64 cycle is smooth and bounded
        ipd=65+7*math.sin(math.tau*t/4-math.asin(1/7))
        T[0,3]=sign*(ipd-64)/2
    T[:3,3]+=np.asarray(part.explode)*explode
    return T

def build(spec=None, include_phone=True, include_straps=True):
    s=spec or Spec()
    parts=[]
    z=s.center_z
    def add(name,shape,mat=0,group="shell",motion="fixed",explode=(0,0,0),
            role="",provenance="designed-concept",tags=()):
        p=cad_part(name,shape,mat,group=group,motion=motion,
                   explode=np.array(explode,float),role=role,provenance=provenance,
                   tolerance=.075,angular=.08,tags=tags)
        parts.append(p)
        return p

    # Continuous optical tunnel, open at the eyes and phone. A finite wall
    # separates the eye cavities without entering the screen or lens apertures.
    outer=rounded_box(180,52,98,(0,17,z),17)
    inner=rounded_box(166,54,84,(0,17,z),12)
    shell=outer.cut(inner)
    nose=rounded_box(34,36,26,(0,-7,24),12)
    shell=shell.cut(nose)
    fasteners=[(x,z+v) for x in (-84,84) for v in (-34,34)]
    for x,zz in fasteners:
        shell=shell.cut(cylinder_y(1.65,34,45,x,zz))
    for x in (-72,72):
        for zz in (z-28,z+28):
            ear=rounded_box(14,18,10,(math.copysign(78.5,x),3,zz),2)
            shell=shell.fuse(ear).cut(cylinder_y(1.65,-7,13,x,zz))
    add("01_optical_tunnel",shell,role="PA12 or PETG opaque tunnel; 7 mm nominal radial walls",
        tags=("print",))
    septum=rounded_box(1.4,43,84,(0,21.5,z),0).cut(nose)
    add("02_binocular_septum",septum,1,"baffle",role="Bond along tunnel top/bottom; black optical isolation",
        tags=("print",))

    # Eye plate with real obround IPD apertures and captive nut tracks.
    f=s.focus_offset
    plate=rounded_box(166,3,84,(0,-10.5+f,z),12).cut(
        rounded_box(35,8,24,(0,-10.5+f,25),11))
    for sign in (-1,1):
        lo,hi=sorted([sign*29,sign*36])
        plate=plate.cut(x_slot(lo,hi,20.4,-14+f,-7+f,z))
        for zz in (z-29,z+29):
            plate=plate.cut(x_slot(lo,hi,1.65,-14+f,-7+f,zz))
    add("03_IPD_bridge",plate,1,"bridge",explode=(0,-35,0),
        role="Sliding lens slots 58–72 mm IPD; loosen all four bridge clamp screws",
        tags=("print",))
    # Four perimeter bridge screws and fixed/replacement focus spacers.
    for k,(x,zz) in enumerate([(x,z+v) for x in (-72,72) for v in (-28,28)]):
        boss=annulus(4.5,1.65,-9+f,-6,x,zz)
        add(f"04_focus_spacer_{k}",boss,0,"bridge",explode=(0,-22,0),
            role="Replace the four spacers together for -1/0/+1 mm optical focus shift",
            tags=("print",))
        # Carve attachment clearance into plate; nut modeled with bore below.
        bridge=next(p for p in parts if p.name=="03_IPD_bridge")
        shape=bridge.cad.cut(cylinder_y(1.65,-13+f,-5,x,zz))
        parts[parts.index(bridge)]=cad_part(bridge.name,shape,1,group=bridge.group,
            explode=bridge.explode,role=bridge.role,tags=bridge.tags,
            tolerance=.075,angular=.08)
        add(f"05_bridge_screw_{k}",annulus(2.75,1.,-14+f,-12+f,x,zz).fuse(
            cylinder_y(1.45,-12+f,-5.5,x,zz)),5,"fasteners",explode=(0,-50,0),
            role="M3 clamp envelope; thread form omitted",provenance="assumed-fastener")
    # Lens carriers have a sliding flange behind the bridge, a through bore,
    # an annular front seat, rear removable retainer and elastomer rim pads.
    for sign,label in [(-1,"L"),(1,"R")]:
        x=sign*s.ipd/2
        motion="left_ipd" if sign<0 else "right_ipd"
        # static nondefault configurations are checked with motion disabled below
        tube=annulus(20,17.3,-12+f,2.5+f,x,z)
        flange=annulus(23,17.3,-15+f,-12+f,x,z)
        for zz in (z-29,z+29):
            tab=rounded_box(12,3,13,(x,-13.5+f,zz),2).cut(
                cylinder_y(1.65,-16+f,-12+f,x,zz))
            flange=flange.fuse(tab)
        seat=annulus(17.3,16.6,1.4+f,2.5+f,x,z)
        add(f"10_{label}_sliding_barrel",tube.fuse(flange).fuse(seat),1,"oculars",motion,
            (sign*18,-65,0),"Print as a single component; slide, align IPD, clamp",
            tags=("print",))
        add(f"11_{label}_lens",lens_solid(s,x),7,"lenses",motion,(sign*18,-42,0),
            "Analytic symmetric biconvex PMMA prescription; source a measured matching optic",
            provenance="designed-concept")
        add(f"12_{label}_rim_front",annulus(17.2,16.6,.7+f,1.4+f,x,z),
            9,"oculars",motion,(sign*18,-50,0),"Compliant optical rim seat; compression unqualified")
        add(f"13_{label}_rim_rear",annulus(17.2,16.6,-1.4+f,-.7+f,x,z),
            9,"oculars",motion,(sign*18,-48,0),"Compliant optical rim seat; compression unqualified")
        add(f"14_{label}_retaining_ring",annulus(17.25,16.6,-3.0+f,-1.4+f,x,z),
            1,"oculars",motion,(sign*18,-72,0),
            "Removable friction-fit ring; adhesive-free retention requires a fit coupon",
            tags=("print",))
        # Sleeve behind the seated ring supports it through the barrel length.
        rear_flange=annulus(23,16.6,-17.5+f,-15+f,x,z)
        for zz in (z-29,z+29):
            rear_flange=rear_flange.fuse(rounded_box(12,2.5,13,
                (x,-16.25+f,zz),2)).cut(cylinder_y(1.65,-18+f,-14+f,x,zz))
        add(f"15_{label}_retainer_sleeve",annulus(17.25,16.6,-15+f,-3.0+f,x,z).fuse(rear_flange),
            1,"oculars",motion,(sign*18,-80,0),
            "Retainer spacer bears on rim ring; flange is clamped by same bridge screws",
            tags=("print",))
        for j,zz in enumerate((z-29,z+29)):
            add(f"16_{label}_IPD_thumb_screw_{j}",cylinder_y(3.6,-20+f,-17.5+f,x,zz).fuse(
                cylinder_y(1.45,-17.5+f,-7.5+f,x,zz)),6,"oculars",motion,
                (sign*18,-90,0),"M3 thumb screw and recessed captive nut envelope",
                provenance="assumed-fastener")
            nut=annulus(3.2,1.5,-9+f,-7+f,x,zz)
            add(f"17_{label}_IPD_nut_{j}",nut,5,"oculars",motion,(sign*18,-24,0),
                "M3 purchased nut; circular clearance envelope; thread omitted",
                provenance="assumed-fastener")

    # Nominal silicone face seal; nose relief is open, no solid bar across nose.
    gasket=rounded_box(169,12,89,(0,-24,z),15).cut(
        rounded_box(135,15,59,(0,-24,z+2),18)).cut(
        rounded_box(40,16,28,(0,-24,26),13))
    add("20_removable_face_cushion",gasket,2,"cushion",explode=(0,-105,0),
        role="Cut foam or molded silicone with textile cover; face conformance unmeasured",
        provenance="assumed-anthropometric-envelope")
    # Adhesive-backed hook-and-loop patches attach cushion to bridge perimeter.
    for i,x in enumerate((-68,68)):
        add(f"21_cushion_hook_loop_{i}",rounded_box(9,6,40,(x,-15,z),2),
            8,"cushion",explode=(0,-80,0),role="Hook-and-loop spacer pad")

    # Open-backed cradle allows convective cooling and camera-bump clearance.
    cradle=rounded_box(180,14,98,(0,50,z),17).cut(
        rounded_box(165.8,17,80.6,(0,50,z),10))
    for x,zz in fasteners:
        cradle=cradle.cut(cylinder_y(1.65,42,59,x,zz))
    add("30_phone_cradle",cradle,0,"cradle",explode=(0,55,0),
        role="Bare-phone cavity 165.8 x 80.6 mm; phone withdraws along +Y",
        tags=("print",))
    # Thin flange holds bezel only, beyond the derived full display rectangle.
    ledge=rounded_box(165.8,1.0,80.6,(0,43,z),10).cut(
        rounded_box(160.2,2,74.6,(0,43,z),8))
    add("31_screen_bezel_stop",ledge,1,"cradle",explode=(0,55,0),
        role="Bezel support; no clamp load on active display",tags=("print",))
    for sign in (-1,1):
        # Pads stop short of the active glass and camera bump envelope.
        add(f"32_side_pad_{sign}",rounded_box(.6,6,40,(sign*81.7,47,z),.2),
            9,"cradle",explode=(0,55,0),role="0.6 mm silicone body-edge pad; compress to fit")
        add(f"33_lower_upper_pad_{sign}",rounded_box(90,6,.6,(0,47,z+sign*39.1),.2),
            9,"cradle",explode=(0,55,0),role="Silicone body-edge pad")

    cover=rounded_box(180,3,98,(0,58.5,z),17)
    # Actual through ventilation; ribs remain as structural ligaments.
    for x in range(-58,59,12):
        cover=cover.cut(rounded_box(5,5,48,(x,58.5,z),2.4))
    for x,zz in fasteners:
        cover=cover.cut(cylinder_y(1.65,56,61,x,zz))
    add("40_ventilated_phone_cover",cover,0,"cover",explode=(0,105,0),
        role="Four accessible M3 screws; camera allowance 4 mm, rear gap 5.3 mm",
        tags=("print",))
    # Compression bars contact phone outside a conservative camera keepout.
    for sign in (-1,1):
        add(f"41_phone_retention_pad_{sign}",rounded_box(16,5.3,40,
            (sign*30,54.35,z-12),3),9,"cover",explode=(0,105,0),
            role="Foam rear preload pads; excluded from rigid collision claims",
            provenance="assumed-compliant-pad")
    for k,(x,zz) in enumerate(fasteners):
        head=annulus(2.8,.8,60,62.4,x,zz)
        shaft=cylinder_y(1.45,35.5,60,x,zz)
        add(f"42_cover_M3_screw_{k}",head.fuse(shaft),5,"fasteners",
            explode=(0,133,0),role="M3 x 25 fastener envelope, tapped shell bosses",
            provenance="assumed-fastener")

    if include_phone:
        phone=rounded_box(s.phone_width,s.phone_thickness,s.phone_height,
            (0,s.screen_y+s.phone_thickness/2,z),8)
        add("50_S25_Ultra_body_envelope",phone,3,"phone",explode=(0,80,0),
            role="Samsung nominal body bounds; 8 mm corner radius is an assumed clearance envelope",
            provenance="source-guided-concept")
        add("51_active_display_envelope",rounded_box(s.screen_width,.12,
            s.screen_height,(0,s.screen_y-.07,z),7),4,"phone",explode=(0,80,0),
            role="Derived 6.9-inch rectangular active area; corners/cutout unmeasured",
            provenance="assumed-display-envelope")
        # Camera cluster is a conservative keepout, not a claimed factory model.
        for j,xx in enumerate((53,66)):
            for k,zz in enumerate((z+20,z+5)):
                add(f"52_camera_keepout_{j}_{k}",cylinder_y(5.5,51.7,55.7,xx,zz),
                    4,"phone",explode=(0,80,0),
                    role="Conservative camera envelope; measure actual protrusions",
                    provenance="assumed-camera-envelope")

    # Side strap anchors are slotted solids and attached to the wall.
    for sign in (-1,1):
        anchor=rounded_box(10,22,34,(sign*92,-1,z),4).cut(
            rounded_box(12,9,27,(sign*92,-3,z),3))
        add(f"60_side_strap_anchor_{sign}",anchor,0,"harness",
            role="25 mm woven strap slot; integral bond/joint to shell requires load test",
            tags=("print",))
    top_anchor=rounded_box(32,24,7,(0,0,116),3).cut(
        rounded_box(26,10,10,(0,0,116),2))
    add("61_top_strap_anchor",top_anchor,0,"harness",role="25 mm top strap slot",tags=("print",))

    if include_straps:
        theta=np.linspace(0,math.pi,81)
        points=np.c_[92*np.cos(theta),-3-174*np.sin(theta),65+6*np.sin(theta)]
        v,fa=strap_mesh(points)
        parts.append(mesh_part("62_adjustable_head_strap",v,fa,8,group="straps",
            role="25 x 1.5 mm woven nylon ribbon; nominal unloaded wearing curve",
            provenance="assumed-flexible-envelope"))
        t=np.linspace(0,1,70)
        points=np.c_[np.zeros_like(t),-3-173*t,
                     116+50*np.sin(math.pi*t)-43*t]
        v,fa=strap_mesh(points,top=True)
        parts.append(mesh_part("63_adjustable_top_strap",v,fa,8,group="straps",
            role="25 mm crown strap with hook-and-loop adjustment",
            provenance="assumed-flexible-envelope"))
        rear=rounded_box(58,9,38,(0,-177,72),8).cut(
            rounded_box(34,12,16,(0,-177,72),6))
        add("64_rear_strap_adjuster",rear,1,"harness",role="Rear strap overlap guide",
            tags=("print",))
        add("65_occipital_pad",rounded_box(66,10,45,(0,-167,72),10),
            2,"cushion",role="Nominal removable foam rear pad; fit not qualified",
            provenance="assumed-anthropometric-envelope")

    # Small dimensional IPD ticks are actual raised geometry.
    for sign in (-1,1):
        for value in range(58,73,2):
            x=sign*value/2
            add(f"70_IPD_tick_{sign}_{value}",rounded_box(.35,.18,
                2 if value%4 else 3,(x,-12.12+f,z+36)),6,"markings",explode=(0,-35,0),
                role=f"IPD setting {value} mm; both carriers must match")
    # Original embossed identifier, real CAD text geometry on the front fascia.
    text=cq.Workplane("XZ").text("CYBR / M1",7,.35,combine=True,
                                 font="DejaVu Sans",kind="bold").val()
    # XZ text extrudes toward -Y; rotate to face +Y and translate above vents.
    text=text.rotate((0,0,0),(0,0,1),180).translate((0,60.35,z+35))
    add("71_embossed_identifier",text,1,"cover",explode=(0,105,0),role="Original product identifier bonded to removable cover")

    views={
        "hero":View(az=55,el=25,scale=152,target=(0,-51,78),
                    focal_length_mm=72,f_stop=16),
        "eyes":View(az=236,el=22,scale=112,target=(0,-25,70),
                    focal_length_mm=72,f_stop=16),
        "optics":View(az=248,el=13,scale=76,target=(0,-7,66),
                    hide=("straps","harness","cushion","cover","phone"),
                    focal_length_mm=82,f_stop=16),
        "exploded":View(az=40,el=24,scale=220,target=(0,10,66),
                    explode=1,hide=("straps",),focal_length_mm=72,f_stop=22),
    }
    materials=list(MATERIALS)
    materials[7]=replace(materials[7],ior=s.lens_ior)
    a=Assembly("cybr_visor_m1",parts,materials,views=views,
        metadata={"truth_intent":"inspection","design":s.report(),
                  "hardware_status":"unbuilt mobile VR prototype",
                  "note":"All renders inspect declared design/envelopes; no claimed device certification."})
    if s.ipd==64:
        a.motion_function=pose
    return a
