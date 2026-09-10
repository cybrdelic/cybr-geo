"""Five source-rendered Nitinol actuator variants for Mechanism Lab.

The family is intentionally mechanically diverse:
- compact: short 8-fiber precision linear actuator
- high_force: 24-fiber linear bundle actuator
- antagonistic: two opposed bundles driving one carriage bidirectionally
- rotary: linear SMA rack driving a pinion/output horn
- cartridge: semi-enclosed serviceable linear cartridge

All motion is prescribed demonstration kinematics. No thermal/FEA/contact/fatigue
qualification is claimed.
"""
from __future__ import annotations
from dataclasses import dataclass, asdict, replace
import math
import numpy as np
import cadquery as cq

from ..core import Assembly, Material, View, Part, cad_part
from ..geometry import ring, drill, bolt_circle
from .nitinol_actuator import _straight_cylinder_part, _torus_part, _mesh_tube_part

MATERIALS = [
    Material("Black anodized aluminium", (0.025,0.031,0.040), 0.82, 0.24),
    Material("Machined aluminium", (0.58,0.61,0.65), 0.95, 0.18),
    Material("Polished stainless", (0.57,0.60,0.63), 0.97, 0.14),
    Material("Nitinol wire", (0.43,0.46,0.50), 0.97, 0.17),
    Material("Copper bus", (0.72,0.28,0.075), 0.94, 0.19),
    Material("Spring steel", (0.31,0.34,0.38), 0.97, 0.22),
    Material("PEEK / ceramic", (0.71,0.64,0.43), 0.02, 0.42),
    Material("Nickel crimp", (0.65,0.68,0.72), 0.95, 0.14),
    Material("Polymer bushing", (0.045,0.052,0.062), 0.01, 0.36),
    Material("Cartridge shell", (0.075,0.085,0.102), 0.76, 0.28),
]

SOURCE = "https://dynalloy.com/technical-data-wires/"

def _smooth01(x):
    x=max(0.0,min(1.0,float(x))); return x*x*(3-2*x)

def _activation(t):
    q=float(t)%5.5
    if q<1.0:return _smooth01(q)
    if q<1.45:return 1.0
    if q<4.65:return 1.0-_smooth01((q-1.45)/3.2)
    return 0.0

def _rz(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]],float)

def _pivot_pose_z(angle, pivot, explode=(0,0,0), amount=0):
    R=_rz(angle);p=np.asarray(pivot,float)
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=p-R@p+np.asarray(explode,float)*amount
    return T

@dataclass(frozen=True)
class LinearSpec:
    name:str
    title:str
    fiber_count:int
    active_length:float
    strain:float
    series_per_string:int
    frame_radius:float
    guide_radius:float
    fiber_radius:float
    guide_rod_radius:float
    active_x0:float
    spring_preload:float
    spring_rate:float
    enclosure:bool=False

    @property
    def moving_anchor_x(self): return self.active_x0+self.active_length
    @property
    def carriage_x(self): return self.moving_anchor_x+4.0
    @property
    def front_x(self): return self.carriage_x+27.0
    @property
    def output_end_x(self): return self.front_x+24.0
    @property
    def stroke(self): return self.active_length*self.strain

COMPACT=LinearSpec("nitinol_compact","COMPACT PRECISION SMA",8,92.,.032,4,18.5,13.0,7.3,1.2,12.,10.,.30,False)
HIGH=LinearSpec("nitinol_high_force","HIGH-FORCE 24-FIBER SMA",24,150.,.035,4,31.,23.,15.2,1.8,12.,34.,.78,False)
CARTRIDGE=LinearSpec("nitinol_cartridge","ENCLOSED SMA CARTRIDGE",12,120.,.035,4,23.5,16.5,10.0,1.45,12.,20.,.50,True)

def _linear_pose(p,t,e,spec):
    a=_activation(t);dx=-spec.stroke*a
    d=0.0
    if p.motion=="carriage":
        d=dx
    elif p.motion=="fiber":
        u=(float(p.center[0])-spec.active_x0)/spec.active_length
        d=dx*max(0,min(1,u))
    elif p.motion=="spring":
        sx0=spec.active_x0+10
        sx1=spec.moving_anchor_x-5
        u=(float(p.center[0])-sx0)/max(1e-6,sx1-sx0)
        d=dx*max(0,min(1,u))
    T=np.eye(4);T[:3,3]=np.asarray(p.explode,float)*float(e);T[0,3]+=d
    return T

def pose_compact(p,t,e): return _linear_pose(p,t,e,COMPACT)
def pose_high_force(p,t,e): return _linear_pose(p,t,e,HIGH)
def pose_cartridge(p,t,e): return _linear_pose(p,t,e,CARTRIDGE)

def _box(name,size,center,mat,group,**kw):
    sx,sy,sz=size;cx,cy,cz=center
    shape=cq.Workplane("XY").box(sx,sy,sz).translate((cx,cy,cz)).val()
    return cad_part(name,shape,mat,group=group,**kw)

def _linear_build(spec:LinearSpec):
    if spec.fiber_count%spec.series_per_string:
        raise ValueError("fiber_count must divide into series strings")
    parts=[]
    def add(name,shape,mat,group,**kw):
        p=cad_part(name,shape,mat,group=group,**kw);parts.append(p);return p

    guides=bolt_circle(spec.guide_radius,4,phase=math.pi/4)
    fibers=bolt_circle(spec.fiber_radius,spec.fiber_count,phase=math.pi/spec.fiber_count)
    rear=ring(spec.frame_radius,3.8,0,5.5)
    rear=drill(rear,guides,spec.guide_rod_radius+.16,-.4,5.9)
    add("LF01_Rear_frame",rear,0,"structure",explode=np.array([-26.,0,0]),role="Fixed reaction frame")

    front=ring(spec.frame_radius,3.8,spec.front_x,spec.front_x+5.5)
    front=drill(front,guides,spec.guide_rod_radius+.16,spec.front_x-.4,spec.front_x+5.9)
    add("LF02_Front_frame",front,0,"structure",explode=np.array([28.,0,0]),role="Front guide frame")

    for i,(y,z) in enumerate(guides):
        parts.append(_straight_cylinder_part(
            f"LF03_Guide_{i+1:02}",5.5,spec.front_x,y,z,spec.guide_rod_radius,2,
            group="guides",explode=(0,6*math.cos(i*math.pi/2),6*math.sin(i*math.pi/2)),
            role="Precision guide rod",sides=18))

    anchor_ro=max(spec.fiber_radius+3.0,8.8)
    fixed=ring(anchor_ro,3.2,spec.active_x0-4,spec.active_x0)
    fixed=drill(fixed,fibers,.30,spec.active_x0-4.5,spec.active_x0+.5)
    add("LF04_Fixed_anchor",fixed,6,"anchors",explode=np.array([-15.,0,0]),role="Fixed insulated fiber carrier")
    moving=ring(anchor_ro,3.2,spec.moving_anchor_x,spec.moving_anchor_x+4)
    moving=drill(moving,fibers,.30,spec.moving_anchor_x-.5,spec.moving_anchor_x+4.5)
    add("LF05_Moving_anchor",moving,6,"carriage",motion="carriage",
        center=np.array([spec.moving_anchor_x+2,0,0]),explode=np.array([14.,0,0]),role="Moving insulated fiber carrier")

    carriage_ro=spec.guide_radius+3.3
    carriage=ring(carriage_ro,3.6,spec.carriage_x,spec.carriage_x+6)
    carriage=drill(carriage,guides,spec.guide_rod_radius+.28,spec.carriage_x-.5,spec.carriage_x+6.5)
    add("LF06_Carriage",carriage,1,"carriage",motion="carriage",
        center=np.array([spec.carriage_x+3,0,0]),explode=np.array([21.,0,0]),role="Rigid translating carriage")
    for i,(y,z) in enumerate(guides):
        add(f"LF07_Bushing_{i+1:02}",
            ring(spec.guide_rod_radius+1.1,spec.guide_rod_radius+.06,spec.carriage_x-.1,spec.carriage_x+6.1).translate((0,y,z)),
            8,"carriage",motion="carriage",center=np.array([spec.carriage_x+3,y,z]),
            explode=np.array([21.,4*math.cos(i*math.pi/2),4*math.sin(i*math.pi/2)]),role="Guide bushing")

    add("LF08_Output_rod",ring(2.7,0,spec.carriage_x+6,spec.output_end_x-6),2,"carriage",
        motion="carriage",center=np.array([(spec.carriage_x+6+spec.output_end_x-6)/2,0,0]),explode=np.array([30.,0,0]),role="Linear output rod")
    add("LF09_Output_eye",ring(5.0,2.2,spec.output_end_x-6,spec.output_end_x),1,"carriage",
        motion="carriage",center=np.array([spec.output_end_x-3,0,0]),explode=np.array([35.,0,0]),role="Output eye")

    sx0=spec.active_x0+9;sx1=spec.moving_anchor_x-5
    add("LF10_Spring_fixed_seat",ring(6.8,2.0,sx0-3,sx0),1,"spring",role="Return spring fixed seat")
    add("LF11_Spring_moving_seat",ring(6.8,2.0,sx1,sx1+3),1,"carriage",
        motion="carriage",center=np.array([sx1+1.5,0,0]),role="Return spring moving seat")
    parts.append(_straight_cylinder_part("LF12_Spring_pilot",sx0,sx1,0,0,1.05,2,group="spring",role="Return spring pilot",sides=16))
    for i in range(14):
        u=i/13;x=sx0+3+u*(sx1-sx0-6)
        parts.append(_torus_part(f"LF13_Spring_turn_{i+1:02}",x,5.4,.42,5,group="spring",motion="spring",
                                 center=(x,0,0),explode=(0,0,-6),role="Discretized return spring"))

    segs=10 if spec.fiber_count<=12 else 8
    seglen=spec.active_length/segs
    for fi,(y,z) in enumerate(fibers):
        for si in range(segs):
            xa=spec.active_x0+si*seglen;xb=spec.active_x0+(si+1)*seglen
            parts.append(_straight_cylinder_part(
                f"LF14_Fiber_{fi+1:02}_{si+1:02}",xa,xb,y,z,.10,3,group="fibers",motion="fiber",
                center=((xa+xb)/2,y,z),
                explode=(0,4*math.cos(2*math.pi*fi/spec.fiber_count),4*math.sin(2*math.pi*fi/spec.fiber_count)),
                role="0.20 mm Nitinol active fiber",sides=10))
        add(f"LF15_Fixed_crimp_{fi+1:02}",ring(.55,.13,spec.active_x0-2,spec.active_x0+.5).translate((0,y,z)),7,"electrical",
            explode=np.array([-10.,0,0]),role="Representative fixed crimp")
        add(f"LF16_Moving_crimp_{fi+1:02}",ring(.55,.13,spec.moving_anchor_x-.5,spec.moving_anchor_x+2).translate((0,y,z)),7,"carriage",
            motion="carriage",center=np.array([spec.moving_anchor_x+.75,y,z]),explode=np.array([10.,0,0]),role="Representative moving crimp")

    strings=spec.fiber_count//spec.series_per_string
    term_y=spec.frame_radius+5
    parts.append(_box("LF17_Positive_terminal",(5,7,9),(5,term_y,5),4,"electrical",explode=np.array([-18.,5,0]),role="Fixed positive bus terminal"))
    parts.append(_box("LF18_Negative_terminal",(5,7,9),(5,term_y,-7),4,"electrical",explode=np.array([-18.,5,0]),role="Fixed negative bus terminal"))
    for st in range(strings):
        f0=st*spec.series_per_string
        y,z=fibers[f0]
        parts.append(_mesh_tube_part(f"LF19_Pos_branch_{st+1}",[(5,term_y,5),(7,term_y-2,5),(spec.active_x0-2,y,z)],.32,4,
                                     group="electrical",explode=(-12,3,0),role="Positive bus branch"))
        y,z=fibers[f0+spec.series_per_string-1]
        parts.append(_mesh_tube_part(f"LF20_Neg_branch_{st+1}",[(5,term_y,-7),(7,term_y-2,-7),(spec.active_x0-2,y,z)],.32,4,
                                     group="electrical",explode=(-12,-3,0),role="Negative bus branch"))
        for j in range(spec.series_per_string-1):
            a=fibers[f0+j];b=fibers[f0+j+1]
            pts=[(spec.moving_anchor_x+2,a[0],a[1]),(spec.moving_anchor_x+5,(a[0]+b[0])/2,(a[1]+b[1])/2),(spec.moving_anchor_x+2,b[0],b[1])]
            parts.append(_mesh_tube_part(f"LF21_String_{st+1}_jumper_{j+1}",pts,.27,4,group="carriage",motion="carriage",
                                         center=(spec.moving_anchor_x+3,np.mean([a[0],b[0]]),np.mean([a[1],b[1]])),
                                         explode=(10,0,0),role="Moving-end electrical series jumper"))

    for i,(y,z) in enumerate(guides):
        add(f"LF22_Cold_stop_{i+1:02}",ring(spec.guide_rod_radius+1.0,spec.guide_rod_radius+.05,
            spec.carriage_x+9,spec.carriage_x+10.4).translate((0,y,z)),1,"stops",role="Cold travel stop")
        hx=spec.carriage_x+9-spec.stroke
        add(f"LF23_Hot_stop_{i+1:02}",ring(spec.guide_rod_radius+1.0,spec.guide_rod_radius+.05,hx,hx+1.2).translate((0,y,z)),1,"stops",role="Hot travel stop")

    if spec.enclosure:
        L=spec.front_x+6
        parts += [
            _box("LF24_Top_shell",(L-10,2.0,8.0),((L-4)/2,0,spec.frame_radius+3.0),9,"enclosure",explode=np.array([0,0,18.]),role="Removable top cooling/service panel"),
            _box("LF25_Bottom_shell",(L-10,2.0,8.0),((L-4)/2,0,-spec.frame_radius-3.0),9,"enclosure",explode=np.array([0,0,-18.]),role="Removable bottom panel"),
            _box("LF26_Left_rail",(L-10,7.0,2.0),((L-4)/2,spec.frame_radius+3.0,0),9,"enclosure",explode=np.array([0,18.,0]),role="Longitudinal shell rail"),
            _box("LF27_Right_rail",(L-10,7.0,2.0),((L-4)/2,-spec.frame_radius-3.0,0),9,"enclosure",explode=np.array([0,-18.,0]),role="Longitudinal shell rail"),
        ]

    mid=spec.output_end_x/2
    scale=max(spec.output_end_x*.34,spec.frame_radius*2.0)
    views={
        "hero":View(42,24,scale,(mid,0,0),title=spec.title,note=f"{spec.fiber_count} x 0.20 mm fibers / {spec.stroke:.2f} mm modeled stroke"),
        "opposite":View(218,22,scale,(mid,0,0),title=spec.title+" / REVERSE 3/4"),
        "side":View(270,8,scale*.82,(mid,0,0),title=spec.title+" / SIDE"),
        "top":View(0,88,scale*.82,(mid,0,0),title=spec.title+" / TOP"),
        "end":View(2,2,spec.frame_radius*1.45,(spec.front_x,0,0),title=spec.title+" / OUTPUT FACE"),
        "bundle":View(45,16,scale*.70,(spec.active_x0+spec.active_length/2,0,0),hide=("structure","enclosure"),title=spec.title+" / ACTIVE BUNDLE"),
        "electrical":View(145,18,max(35,spec.frame_radius*1.8),(spec.active_x0,0,0),title=spec.title+" / TERMINATIONS"),
        "section":View(42,12,scale*.86,(mid,0,0),section=(0,1,0),hide=("enclosure",),title=spec.title+" / LONGITUDINAL SECTION"),
        "exploded":View(58,22,scale*1.25,(mid,0,0),explode=1,title=spec.title+" / EXPLODED"),
        "hot":View(42,24,scale,(mid,0,0),title=spec.title+" / HOT CONTRACTED"),
    }
    drawing_parts=["LF01_Rear_frame","LF02_Front_frame","LF04_Fixed_anchor","LF05_Moving_anchor","LF06_Carriage","LF08_Output_rod","LF09_Output_eye"]
    if spec.enclosure:drawing_parts += ["LF24_Top_shell","LF26_Left_rail"]
    meta={
        "family":"CYBR Nitinol Actuator Family","variant":asdict(spec),"sources":[SOURCE],
        "fidelity":"Original parametric concept geometry; 0.20 mm SMA wire geometry/source reference only.",
        "drawing_groups":["structure","anchors","carriage","enclosure"],
        "drawings":{"default":{"parts":drawing_parts,"title":spec.title+" / INTERFACE WHITEPRINT","auto_dimensions":True,
            "annotations":[
                {"kind":"note","paper_xy":[18,30],"text":f"ACTIVE LENGTH {spec.active_length:.1f} mm / STROKE {spec.stroke:.2f} mm","size":2.5},
                {"kind":"note","paper_xy":[18,36],"text":f"{spec.fiber_count} x DIA 0.20 mm SMA / {strings} PARALLEL STRINGS","size":2.5},
            ]}},
        "motion_model":"Prescribed heating/cooling contraction proxy; not force/thermal/FEA simulation.",
    }
    return Assembly(spec.name,parts,MATERIALS,views,metadata=meta,motion_function=lambda p,t,e:_linear_pose(p,t,e,spec))

def build_compact(): return _linear_build(COMPACT)
def build_high_force(): return _linear_build(HIGH)
def build_cartridge(): return _linear_build(CARTRIDGE)

ANTI_NAME="nitinol_antagonistic"
ANTI_CENTER=112.0
ANTI_STROKE=4.2
ANTI_LEFT=16.0
ANTI_RIGHT=208.0
ANTI_FIBERS=8
ANTI_RADIUS=9.5

def pose_antagonistic(p,t,e):
    dx=ANTI_STROKE*math.sin(math.tau*(float(t)%6.0)/6.0)
    d=0.
    if p.motion=="carriage": d=dx
    elif p.motion=="left_fiber":
        u=(float(p.center[0])-ANTI_LEFT)/(ANTI_CENTER-8-ANTI_LEFT)
        d=dx*max(0,min(1,u))
    elif p.motion=="right_fiber":
        u=(float(p.center[0])-(ANTI_CENTER+8))/(ANTI_RIGHT-(ANTI_CENTER+8))
        d=dx*(1-max(0,min(1,u)))
    T=np.eye(4);T[:3,3]=np.asarray(p.explode,float)*e;T[0,3]+=d;return T

def build_antagonistic():
    parts=[]
    def add(name,shape,mat,group,**kw):
        p=cad_part(name,shape,mat,group=group,**kw);parts.append(p);return p
    guides=bolt_circle(17.5,4,phase=math.pi/4);fibers=bolt_circle(ANTI_RADIUS,ANTI_FIBERS,phase=math.pi/ANTI_FIBERS)
    rear=drill(ring(24,3.8,0,6),guides,1.6,-.5,6.5);add("AG01_Left_frame",rear,0,"structure",explode=np.array([-30.,0,0]))
    front=drill(ring(24,3.8,218,224),guides,1.6,217.5,224.5);add("AG02_Right_frame",front,0,"structure",explode=np.array([30.,0,0]))
    for i,(y,z) in enumerate(guides):
        parts.append(_straight_cylinder_part(f"AG03_Guide_{i+1}",6,218,y,z,1.45,2,group="guides",explode=(0,6*math.cos(i*math.pi/2),6*math.sin(i*math.pi/2)),sides=18))
    la=drill(ring(13.8,3.0,12,16),fibers,.3,11.5,16.5);add("AG04_Left_anchor",la,6,"anchors",explode=np.array([-18.,0,0]))
    ra=drill(ring(13.8,3.0,208,212),fibers,.3,207.5,212.5);add("AG05_Right_anchor",ra,6,"anchors",explode=np.array([18.,0,0]))
    maL=drill(ring(13.8,3.0,ANTI_CENTER-8,ANTI_CENTER-4),fibers,.3,ANTI_CENTER-8.5,ANTI_CENTER-3.5)
    add("AG06_Carriage_left_anchor",maL,6,"carriage",motion="carriage",center=np.array([ANTI_CENTER-6,0,0]),explode=np.array([-10.,0,0]))
    maR=drill(ring(13.8,3.0,ANTI_CENTER+4,ANTI_CENTER+8),fibers,.3,ANTI_CENTER+3.5,ANTI_CENTER+8.5)
    add("AG07_Carriage_right_anchor",maR,6,"carriage",motion="carriage",center=np.array([ANTI_CENTER+6,0,0]),explode=np.array([10.,0,0]))
    car=drill(ring(21.0,3.6,ANTI_CENTER-4,ANTI_CENTER+4),guides,1.75,ANTI_CENTER-4.5,ANTI_CENTER+4.5)
    add("AG08_Central_carriage",car,1,"carriage",motion="carriage",center=np.array([ANTI_CENTER,0,0]))
    for i,(y,z) in enumerate(guides):
        add(f"AG09_Bushing_{i+1}",ring(2.7,1.52,ANTI_CENTER-4.2,ANTI_CENTER+4.2).translate((0,y,z)),8,"carriage",
            motion="carriage",center=np.array([ANTI_CENTER,y,z]))
    add("AG10_Output_rod",ring(2.8,0,ANTI_CENTER+4,246),2,"carriage",motion="carriage",center=np.array([(ANTI_CENTER+250)/2,0,0]),explode=np.array([25.,0,0]))
    add("AG11_Output_eye",ring(5.2,2.2,246,252),1,"carriage",motion="carriage",center=np.array([249,0,0]),explode=np.array([32.,0,0]))
    segs=9
    for fi,(y,z) in enumerate(fibers):
        L=(ANTI_CENTER-8)-ANTI_LEFT
        for si in range(segs):
            xa=ANTI_LEFT+L*si/segs;xb=ANTI_LEFT+L*(si+1)/segs
            parts.append(_straight_cylinder_part(f"AG12_LFiber_{fi+1:02}_{si+1:02}",xa,xb,y,z,.10,3,group="left_fibers",motion="left_fiber",
                center=((xa+xb)/2,y,z),explode=(0,5,0),sides=10))
        R0=ANTI_CENTER+8;L2=ANTI_RIGHT-R0
        for si in range(segs):
            xa=R0+L2*si/segs;xb=R0+L2*(si+1)/segs
            parts.append(_straight_cylinder_part(f"AG13_RFiber_{fi+1:02}_{si+1:02}",xa,xb,y,z,.10,3,group="right_fibers",motion="right_fiber",
                center=((xa+xb)/2,y,z),explode=(0,-5,0),sides=10))
        add(f"AG14_Left_crimp_{fi+1}",ring(.55,.13,13.5,16.5).translate((0,y,z)),7,"electrical",explode=np.array([-8.,0,0]))
        add(f"AG15_Right_crimp_{fi+1}",ring(.55,.13,207.5,210.5).translate((0,y,z)),7,"electrical",explode=np.array([8.,0,0]))
    parts += [
        _box("AG16_Left_power",(5,9,9),(5,29,6),4,"electrical",explode=np.array([-14.,8,0])),
        _box("AG17_Right_power",(5,9,9),(219,-29,6),4,"electrical",explode=np.array([14.,-8,0])),
    ]
    views={
        "hero":View(42,24,92,(120,0,0),title="ANTAGONISTIC DUAL-BUNDLE SMA",note="Opposed bundles / bidirectional carriage"),
        "opposite":View(220,22,92,(120,0,0),title="ANTAGONISTIC SMA / REVERSE"),
        "side":View(270,8,82,(120,0,0),title="ANTAGONISTIC SMA / SIDE"),
        "top":View(0,88,82,(120,0,0),title="ANTAGONISTIC SMA / TOP"),
        "end":View(2,2,36,(218,0,0),title="ANTAGONISTIC SMA / OUTPUT FACE"),
        "bundle":View(45,16,74,(112,0,0),hide=("structure","guides"),title="OPPOSED ACTIVE BUNDLES"),
        "electrical":View(145,18,58,(20,0,0),title="LEFT POWER END"),
        "section":View(42,12,82,(112,0,0),section=(0,1,0),title="ANTAGONISTIC / SECTION"),
        "exploded":View(58,22,120,(112,0,0),explode=1,title="ANTAGONISTIC / EXPLODED"),
        "hot":View(42,24,92,(120,0,0),title="ANTAGONISTIC / PHASED MOTION"),
    }
    meta={"family":"CYBR Nitinol Actuator Family","variant":"antagonistic","sources":[SOURCE],
          "motion_model":"Opposed sinusoidal demonstration drive; not coupled thermal/force simulation.",
          "drawing_groups":["structure","anchors","carriage"],
          "drawings":{"default":{"parts":["AG01_Left_frame","AG02_Right_frame","AG04_Left_anchor","AG05_Right_anchor","AG08_Central_carriage","AG10_Output_rod","AG11_Output_eye"],
             "title":"ANTAGONISTIC DUAL-BUNDLE SMA / WHITEPRINT","auto_dimensions":True,
             "annotations":[{"kind":"note","paper_xy":[18,30],"text":"2 x 8 FIBERS / BIDIRECTIONAL CARRIAGE","size":2.5}]}}}
    return Assembly(ANTI_NAME,parts,MATERIALS,views,metadata=meta,motion_function=pose_antagonistic)

ROT_NAME="nitinol_rotary"
ROT_ACTIVE_X0=12.0
ROT_ACTIVE_LEN=96.0
ROT_STROKE=3.4
ROT_PIVOT=np.array([137.0,-4.0,0.0])
ROT_PITCH_R=11.0
ROT_FIBERS=8
ROT_CENTER_Y=-18.0

def pose_rotary(p,t,e):
    a=_activation(t);dx=-ROT_STROKE*a
    T=np.eye(4);T[:3,3]=np.asarray(p.explode,float)*e
    if p.motion=="rack":
        T[0,3]+=dx
    elif p.motion=="fiber":
        u=(float(p.center[0])-ROT_ACTIVE_X0)/ROT_ACTIVE_LEN
        T[0,3]+=dx*max(0,min(1,u))
    elif p.motion=="rotor":
        angle=-dx/ROT_PITCH_R
        T=_pivot_pose_z(angle,ROT_PIVOT,p.explode,e)
    return T

def _pinion_shape(px,py,r=11.5,teeth=18,thickness=7.0):
    base=cq.Workplane("XY").center(px,py).circle(r-1.2).extrude(thickness/2,both=True).val()
    shape=base
    for i in range(teeth):
        a=360*i/teeth
        tooth=cq.Workplane("XY").box(3.3,2.3,thickness).translate((px+r,py,0)).val()
        tooth=tooth.rotate((px,py,0),(px,py,1),a)
        shape=shape.fuse(tooth)
    return shape.clean()

def build_rotary():
    parts=[]
    def add(name,shape,mat,group,**kw):
        p=cad_part(name,shape,mat,group=group,**kw);parts.append(p);return p
    parts.append(_box("RT01_Base",(166,58,6),(82,-5,-14),0,"structure",explode=np.array([0,0,-18.])))
    for y in (-25.,-11.):
        parts.append(_straight_cylinder_part(f"RT02_Guide_{int(abs(y))}",6,128,y,0,1.35,2,group="guides",explode=(0,y*.18,0),sides=18))
    fp=bolt_circle(8.2,ROT_FIBERS,phase=math.pi/ROT_FIBERS)
    fixed=drill(ring(12.5,2.7,8,12),fp,.28,7.5,12.5).translate((0,ROT_CENTER_Y,0))
    add("RT03_Fixed_anchor",fixed,6,"anchors",explode=np.array([-16.,0,0]))
    moving=drill(ring(12.5,2.7,108,112),fp,.28,107.5,112.5).translate((0,ROT_CENTER_Y,0))
    add("RT04_Moving_anchor",moving,6,"rack",motion="rack",center=np.array([110,ROT_CENTER_Y,0]),explode=np.array([10.,0,0]))
    rack=cq.Workplane("XY").box(36,8,8).translate((126,ROT_CENTER_Y,0)).val()
    for i in range(9):
        x=111+i*4
        tooth=cq.Workplane("XY").box(2.2,3.4,8).translate((x,ROT_CENTER_Y+5.4,0)).val()
        rack=rack.fuse(tooth)
    add("RT05_Rack",rack.clean(),1,"rack",motion="rack",center=np.array([126,ROT_CENTER_Y,0]),explode=np.array([16.,-8,0]),role="SMA-driven linear rack")
    for i,y in enumerate((-25.,-11.)):
        add(f"RT06_Rack_bushing_{i+1}",ring(2.5,1.42,108,116).translate((0,y,0)),8,"rack",motion="rack",
            center=np.array([112,y,0]),explode=np.array([12.,-4 if i==0 else 4,0]))
    pinion=_pinion_shape(*ROT_PIVOT[:2])
    add("RT07_Pinion",pinion,1,"rotor",motion="rotor",center=ROT_PIVOT.copy(),explode=np.array([20.,12,0]),role="Rack-driven output pinion")
    shaft=cq.Solid.makeCylinder(3.2,26,cq.Vector(ROT_PIVOT[0],ROT_PIVOT[1],-13),cq.Vector(0,0,1))
    add("RT08_Output_shaft",shaft,2,"rotor",motion="rotor",center=ROT_PIVOT.copy(),explode=np.array([22.,12,0]))
    horn=cq.Workplane("XY").box(7,32,4).translate((ROT_PIVOT[0],ROT_PIVOT[1]+16,6)).val()
    horn=horn.cut(cq.Solid.makeCylinder(2.3,6,cq.Vector(ROT_PIVOT[0],ROT_PIVOT[1]+29,3),cq.Vector(0,0,1)))
    add("RT09_Output_horn",horn,1,"rotor",motion="rotor",center=ROT_PIVOT.copy(),explode=np.array([25.,16,4]),role="Rotary output horn")
    tower=cq.Workplane("XY").box(14,14,30).translate((ROT_PIVOT[0],ROT_PIVOT[1],0)).val()
    tower=tower.cut(cq.Solid.makeCylinder(4.0,34,cq.Vector(ROT_PIVOT[0],ROT_PIVOT[1],-17),cq.Vector(0,0,1)))
    add("RT10_Bearing_tower",tower,0,"structure",explode=np.array([18.,10,0]))
    segs=9
    for fi,(yy,zz) in enumerate(fp):
        y=ROT_CENTER_Y+yy;z=zz
        for si in range(segs):
            xa=ROT_ACTIVE_X0+ROT_ACTIVE_LEN*si/segs;xb=ROT_ACTIVE_X0+ROT_ACTIVE_LEN*(si+1)/segs
            parts.append(_straight_cylinder_part(f"RT11_Fiber_{fi+1:02}_{si+1:02}",xa,xb,y,z,.10,3,group="fibers",motion="fiber",
                center=((xa+xb)/2,y,z),explode=(0,4,0),sides=10))
        add(f"RT12_Fixed_crimp_{fi+1}",ring(.55,.13,10,12.5).translate((0,y,z)),7,"electrical")
    parts.append(_box("RT13_Power_terminal",(5,9,10),(4,ROT_CENTER_Y+17,0),4,"electrical",explode=np.array([-12.,8,0])))
    views={
        "hero":View(45,38,76,(88,-8,0),title="ROTARY SMA RACK / PINION",note="8-fiber bundle / linear-to-rotary conversion"),
        "opposite":View(220,34,76,(88,-8,0),title="ROTARY SMA / REVERSE"),
        "side":View(270,20,70,(88,-8,0),title="ROTARY SMA / SIDE"),
        "top":View(0,88,72,(88,-8,0),title="ROTARY SMA / TOP GEAR MESH"),
        "end":View(2,4,34,(137,-4,0),title="ROTARY SMA / OUTPUT"),
        "bundle":View(44,20,55,(62,ROT_CENTER_Y,0),hide=("structure","rotor","guides"),title="ROTARY SMA / ACTIVE BUNDLE"),
        "electrical":View(145,24,42,(16,ROT_CENTER_Y,0),title="ROTARY SMA / POWER END"),
        "section":View(44,26,70,(88,-8,0),section=(0,0,1),title="ROTARY SMA / PLAN SECTION"),
        "exploded":View(58,34,102,(88,-8,0),explode=1,title="ROTARY SMA / EXPLODED"),
        "hot":View(45,38,76,(88,-8,0),title="ROTARY SMA / ACTUATED"),
    }
    meta={"family":"CYBR Nitinol Actuator Family","variant":"rotary rack-and-pinion","sources":[SOURCE],
          "motion_model":"Prescribed SMA rack translation mechanically mapped to output pinion rotation.",
          "drawing_groups":["structure","anchors","rack","rotor"],
          "drawings":{"default":{"parts":["RT01_Base","RT03_Fixed_anchor","RT04_Moving_anchor","RT05_Rack","RT07_Pinion","RT08_Output_shaft","RT09_Output_horn","RT10_Bearing_tower"],
            "title":"ROTARY SMA RACK / PINION WHITEPRINT","auto_dimensions":True,
            "annotations":[{"kind":"note","paper_xy":[18,30],"text":"LINEAR SMA RACK -> PINION OUTPUT","size":2.5}]}}}
    return Assembly(ROT_NAME,parts,MATERIALS,views,metadata=meta,motion_function=pose_rotary)

FAMILY_NAMES=("nitinol_compact","nitinol_high_force","nitinol_antagonistic","nitinol_rotary","nitinol_cartridge")
