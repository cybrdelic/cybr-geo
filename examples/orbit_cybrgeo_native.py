"""Native CYBR GEO port of the ORBIT inspection wrist.

The geometry and dimensions are preserved from the original ORBIT revision-3
recipe, but the public model contract is exclusively CYBR GEO:
``build() -> cybrgeo.Assembly`` and all BReps enter through
``cybrgeo.from_shape``.  This module intentionally imports no ``mechanism_lab``.
"""
from __future__ import annotations

import math
import cadquery as cq
import numpy as np

from cybrgeo import Assembly, Material, from_shape, rotation
from cybrgeo.features import involute_spur_gear

AXIS=np.array([0.,0.,70.])
GEAR_TEETH=72
PINION_TEETH=20
MODULE=1.0
CENTER_DISTANCE=(GEAR_TEETH+PINION_TEETH)*MODULE/2
LEAD=3.0
JAW_TRAVEL=6.0
JAW_CENTER=24.0
PERIOD=8.0

MATERIALS=[
    Material('Graphite anodized aluminium',(.065,.080,.095),1.,.34,coat=.18,coat_rough=.28,anisotropy=.15,microfinish='anodized'),
    Material('Satin machined aluminium',(.62,.65,.68),1.,.29,anisotropy=.65,microfinish='brushed'),
    Material('Hardened steel',(.48,.52,.56),1.,.22,anisotropy=.38,microfinish='machined'),
    Material('Warm bronze',(.58,.35,.135),1.,.30,anisotropy=.32,microfinish='turned'),
    Material('Black elastomer',(.012,.014,.017),0.,.68,ior=1.46,microfinish='polymer'),
    Material('Teal anodized aluminium',(.012,.195,.16),1.,.31,coat=.3,coat_rough=.23,anisotropy=.18,microfinish='anodized'),
    Material('Ceramic white marking',(.78,.83,.86),.1,.42),
    Material('Connector copper',(.58,.245,.10),.95,.22,microfinish='copper-wire'),
]


def box(dx,dy,dz,center,fillet=0.):
    q=cq.Workplane('XY').box(dx,dy,dz)
    if fillet:q=q.edges().fillet(fillet)
    return q.val().translate(tuple(center))


def cylinder(radius,length,origin,axis='X'):
    return cq.Solid.makeCylinder(radius,length,cq.Vector(*origin),cq.Vector(*{'X':(1,0,0),'Y':(0,1,0),'Z':(0,0,1)}[axis]))


def annulus(ro,ri,start,length,axis='X'):
    q=cylinder(ro,length,start,axis)
    if not ri:return q
    d=np.array({'X':(1,0,0),'Y':(0,1,0),'Z':(0,0,1)}[axis],float)
    return q.cut(cylinder(ri,length+2,tuple(np.array(start)-d),axis))


def bolt_circle(radius,count=4,phase=0.):
    return [(radius*math.cos(phase+math.tau*i/count),radius*math.sin(phase+math.tau*i/count)) for i in range(count)]


def sector(ro,ri,x0,x1,a0,a1):
    p=lambda r,a:(r*math.cos(a),r*math.sin(a))
    w=(cq.Workplane('YZ').moveTo(*p(ro,a0))
       .threePointArc(p(ro,(a0+a1)/2),p(ro,a1))
       .lineTo(*p(ri,a1))
       .threePointArc(p(ri,(a0+a1)/2),p(ri,a0)).close())
    return w.extrude(x1-x0).translate((x0,0,0)).val()


def screw_x(x,y,z,length=8,radius=1.45):
    shaft=cylinder(radius,length,(x,y,z))
    head=cylinder(radius*1.9,radius*1.7,(x-radius*1.7,y,z))
    head=cq.Workplane(obj=head).edges().fillet(.25).val()
    socket=cq.Workplane('YZ',origin=(x-radius*1.7-.1,y,z)).polygon(6,radius*1.95).extrude(radius).val()
    return shaft.fuse(head).cut(socket)


def knob(x,y,z,radius=12,length=10,bore=3.6):
    q=annulus(radius,bore,(x,y,z),length)
    q=cq.Workplane(obj=q).edges().chamfer(.6).val()
    cuts=[cylinder(.7,length+2,(x-1,y+radius*math.cos(a),z+radius*math.sin(a))) for a in np.linspace(0,math.tau,40,endpoint=False)]
    return q.cut(cq.Compound.makeCompound(cuts))


def thread_ridge(length=36.,pitch=LEAD,left=False,clearance=0.):
    r0,r1=2.40,3.20
    helix=cq.Wire.makeHelix(pitch,length,2.75,lefthand=left)
    c=clearance
    profile=cq.Workplane('XZ').polyline([(r0-c,-.83-c),(r1+c,-.33-c),(r1+c,.33+c),(r0-c,.83+c)]).close()
    return profile.sweep(cq.Workplane(obj=helix),isFrenet=True).val()


def z_to_y(shape,start):
    return shape.rotate((0,0,0),(1,0,0),-90).translate(start)


def wrist_angle(t):return math.radians(18.)*math.sin(math.tau*t/PERIOD)
def jaw_displacement(t):return -.5*JAW_TRAVEL*(1-math.cos(math.tau*t/PERIOD))


def build()->Assembly:
    parts=[];cad={}
    def add(name,shape,material=0,group='structure',motion='fixed',explode=(0,0,0),role='',tags=()):
        tiny='_Ball_' in name or name.startswith(('B04_','H03_','P00_','P03_','J05_','G07_'))
        origin=(0.,46.,70.) if group=='input' or name.startswith('G02_') else (0.,0.,70.)
        axis=(1.,0.,0.)
        if group in ('guides','leadscrew'):axis=(0.,1.,0.);origin=(60.,0.,shape.Center().z)
        elif material==1:axis=(0.,1.,0.);origin=tuple(np.array(shape.BoundingBox().center.toTuple()))
        p=from_shape(name,shape,material=material,tolerance=.018,angular_tolerance=.12 if tiny else .055,
                     group=group,motion=motion,explode=np.array(explode,float),role=role or name.replace('_',' '),
                     metadata={'provenance':'designed-concept','tags':list(tags),'finish_axis':axis,'finish_origin':origin})
        parts.append(p);cad[name]=shape;return shape

    base=box(72,136,9,(-22,9,4.5),2)
    for x in (-46,2):
        for y in (-45,63):
            base=base.cut(cylinder(3.3,12,(x,y,-1),'Z')).cut(cylinder(6,4,(x,y,5.5),'Z'))
    for y in (-32,60):base=base.cut(cylinder(2.2,11,(-17,y,-1),'Z')).cut(cylinder(4.2,110,(-17,y,17),'Z'))
    etch=cq.Workplane('XY',origin=(-34,65,8.88)).text('ORBIT 01',3.2,.3,combine=False,halign='left',valign='center').val();base=base.cut(etch)
    add('B01_Mounting_shoe',base,0,'base',tags=('box','fillet','counterbore','pattern'))
    for i,(x,y) in enumerate([(-46,-45),(-46,63),(2,-45),(2,63)]):add(f'B02_{i}_Nonmarking_foot',cylinder(8,2,(x,y,-2),'Z'),4,'base')

    rear=cylinder(44,10,(-22,0,70)).fuse(cylinder(17,10,(-22,46,70)))
    leg=cq.Workplane('YZ',origin=(-22,0,70)).polyline([(-36,-22),(45,-22),(66,-61),(-43,-61)]).close().extrude(10).val();rear=rear.fuse(leg)
    rear=cq.Workplane(obj=rear.clean()).faces('>X').edges().chamfer(.22).val().cut(cylinder(21.05,12,(-23,0,70))).cut(cylinder(7.55,12,(-23,46,70))).cut(box(13,35,18,(-17,4,23),3))
    for y in (-32,60):rear=rear.cut(cylinder(2.5,18,(-17,y,8),'Z')).cut(cylinder(4.2,110,(-17,y,17),'Z'))
    cover_holes=bolt_circle(40.5,6,math.pi/6)
    for y,zz in cover_holes:rear=rear.cut(cylinder(1.65,12,(-23,y,zz+70)))
    add('B03_Rear_bearing_pedestal',rear,0,'pedestal',explode=(-38,0,0),tags=('union','difference','extrude'))
    for i,y in enumerate((-32,60)):
        s=screw_x(0,0,0,14,2.1).rotate((0,0,0),(0,1,0),90).translate((-17,y,17));add(f'B04_{i}_Pedestal_clamp',s,2,'fasteners')

    outer=cylinder(44,31,(-12,0,70)).fuse(cylinder(17,31,(-12,46,70)));inner=cylinder(40,33,(-13,0,70)).fuse(cylinder(13.5,33,(-13,46,70)))
    shell=outer.cut(inner);shell=cq.Workplane(obj=shell.clean()).faces('>X').edges().chamfer(.24).val()
    for y,zz in cover_holes:shell=shell.fuse(cylinder(3.,31,(-12,y,zz+70))).cut(cylinder(1.75,33,(-13,y,zz+70)))
    for x in (-5,5,15):shell=shell.cut(box(3,24,14,(x,0,111),1.3))
    shell=shell.cut(cylinder(1.8,24,(11,46,72),'Z'));add('H01_Removable_gear_shell',shell,0,'housing',explode=(-7,0,72),tags=('shell-by-difference','slot-array'))

    cover=cylinder(44,9,(19,0,70)).fuse(cylinder(17,9,(19,46,70))).cut(cylinder(21.05,11,(18,0,70))).cut(cylinder(7.55,11,(18,46,70)))
    cover=cq.Workplane(obj=cover.clean()).edges('|X').chamfer(.45).val();cover=cq.Workplane(obj=cover).faces('>X').edges().chamfer(.24).val()
    for i,(y,zz) in enumerate(cover_holes):
        cover=cover.cut(cylinder(1.75,12,(18,y,zz+70))).cut(cylinder(3.0,3,(25.,y,zz+70)))
        fast=screw_x(0,0,0,44,1.45).rotate((0,0,0),(0,1,0),180).translate((25,y,zz+70));add(f'H03_{i:02}_Cover_socket_screw',fast,2,'cover_fasteners',explode=(65,0,0))
    for side in (-1,1):
        half=cover.intersect(box(20,100,120,(24,side*50.1,70)));add(f'H02_{side}_Split_front_bearing_cover',half,1,'cover',explode=(0,side*70,0),tags=('split-bearing-housing','chamfer','counterbore'))

    def bearing(x,label,moving_inner=True):
        c=x+4.5;torus=cq.Solid.makeTorus(16.55,2.02,(c,0,70),(1,0,0));outer_race=annulus(21,17.,(x,0,70),9).cut(torus);inner_race=annulus(16.,14.,(x,0,70),9).cut(torus)
        add(label+'_Outer_race',outer_race,2,'bearings',explode=(-15 if x<0 else 25,0,0),tags=('revolve-equivalent','torus-cut'));add(label+'_Inner_race',inner_race,2,'bearings','wrist' if moving_inner else 'fixed')
        balls=[]
        for i,a in enumerate(np.linspace(0,math.tau,12,endpoint=False)):
            center=(c,16.55*math.cos(a),70+16.55*math.sin(a));balls.append(cq.Solid.makeSphere(2.06,center,angleDegrees1=-90));add(f'{label}_Ball_{i:02}',cq.Solid.makeSphere(1.90,center,angleDegrees1=-90),2,'bearing_balls')
        cage=annulus(17.25,15.85,(c-.7,0,70),1.4).cut(cq.Compound.makeCompound(balls));add(label+'_Pocketed_cage',cage,3,'bearings')
    bearing(-21,'R01_Rear');bearing(19,'R02_Front')

    shaft_profile=[(-32,8),(36,8),(36,34),(30,34),(30,14),(-32,14)]
    shaft=cq.Workplane('XY').polyline(shaft_profile).close().revolve(360,(0,0),(1,0)).val().translate((0,0,70));keytool=box(24,3.3,3.3,(4,14.8,70));shaft=shaft.cut(keytool)
    rim=[edge for edge in shaft.Edges() if edge.geomType()=='CIRCLE' and edge.radius()>20];shaft=cq.Workplane(obj=shaft).newObject(rim).chamfer(.18).val();flange_holes=[(y,z) for y in (-16.,16.) for z in (-26.,26.)]
    for y,z in flange_holes:shaft=shaft.cut(cylinder(1.75,8,(29,y,z+70)))
    add('W01_Revolved_hollow_shaft_and_flange',shaft,2,'wrist','wrist',explode=(14,0,0),tags=('revolve','bore','keyway'));add('W02_Parallel_drive_key',box(22,3.1,3.1,(4,14.8,70),.15),2,'wrist','wrist')
    add('W03_Front_thrust_washer',annulus(16,14.02,(28,0,70),2),2,'wrist','wrist');add('W04_Rear_thrust_washer',annulus(16,14.02,(-23.3,0,70),2.3),2,'wrist','wrist')
    add('W07_Rear_gear_spacer',annulus(18.5,14.02,(-12,0,70),4),2,'wrist','wrist');add('W07_Front_gear_spacer',annulus(18.5,14.02,(16,0,70),3),2,'wrist','wrist')
    collar=annulus(20,14.02,(-29.3,0,70),6)
    for y in (-17.,17.):collar=collar.cut(cylinder(1.3,28,(-26.3,y,56),'Z')).cut(cylinder(2.5,24,(-26.3,y,75),'Z'))
    for side in (-1,1):add(f'W05_{side}_Split_rear_clamp_collar',collar.intersect(box(10,50,30,(-26.3,0,70+side*15.1))),2,'wrist','wrist')
    for i,y in enumerate((-17.,17.)):add(f'W06_{i}_Collar_clamp_screw',screw_x(0,0,0,10,1.1).rotate((0,0,0),(0,1,0),90).translate((-26.3,y,75)),2,'wrist','wrist')

    gear=involute_spur_gear(GEAR_TEETH,MODULE,10,bore=28.2,origin=(-2,0,70),backlash=.055).val().fuse(annulus(18.5,14.1,(-8,0,70),24)).cut(keytool)
    for i in range(6):
        a=math.tau*i/6;gear=gear.cut(sector(30.5,21,-3,9,a+.18,a+.80).translate((0,0,70)))
    add('G01_72T_Involute_wrist_gear',gear,2,'gears','wrist',explode=(0,-58,0),tags=('involute','sector-pattern','boolean'))
    pinion=involute_spur_gear(PINION_TEETH,MODULE,10,bore=7.2,origin=(-2,46,70),phase=math.pi/PINION_TEETH,backlash=.055).val();pinion_hub=annulus(6,3.6,(8,46,70),6).cut(cylinder(1.1,7,(11,46,72),'Z'));pinion=pinion.fuse(pinion_hub)
    add('G02_20T_Involute_input_pinion',pinion,3,'gears','pinion',explode=(0,40,0),tags=('involute','integral-clamping-hub'))
    pshaft=cylinder(3.5,79,(-47,46,70))
    for x in (-41,11,29.5):pshaft=pshaft.cut(box(5,9,2,(x,46,73.8)))
    add('G03_Input_shaft',pshaft,2,'input','pinion')
    for i,x in enumerate((-21,19)):add(f'G04_{i}_Input_plain_bearing',annulus(7.5,3.55,(x,46,70),9),3,'input')
    input_knob=knob(-46,46,70).cut(cylinder(1.1,12,(-41,46,72),'Z'));add('G05_Fluted_rotary_input_knob',input_knob,5,'input','pinion',explode=(-25,0,0),tags=('flute-array','chamfer'))
    collar=annulus(6,3.5,(28.2,46,70),3).cut(cylinder(1.1,7,(29.5,46,72),'Z'));add('G06_Input_shaft_retaining_collar',collar,2,'input','pinion')
    for i,(x,top) in enumerate(((-41,81.5),(11,75.8),(29.5,75.8))):
        grub=cylinder(1.,top-72.8,(x,46,72.8),'Z').cut(cq.Workplane('XY',origin=(x,46,top-.7)).polygon(6,1.2).extrude(1).val());add(f'G07_{i}_Flat_engaging_grub_screw',grub,2,'input','pinion')

    palm=(cq.Workplane('YZ',origin=(36,0,70)).rect(52,40).workplane(offset=5).rect(70,42).workplane(offset=12).rect(94,40).loft(combine=True).val());palm=cq.Workplane(obj=palm).edges().fillet(.35).val();palm=palm.fuse(box(6,54,60,(39,0,70),1.))
    for side in (-1,1):palm=palm.fuse(box(10,12,46,(48,side*45,70),1.))
    palm=palm.cut(box(14,86,33,(49,0,70),2)).cut(cylinder(8.2,20,(35,0,70))).cut(cylinder(5.55,18,(46,-52,70),'Y'))
    for y,z in flange_holes:
        palm=palm.cut(cylinder(1.75,12,(35,y,z+70)));add(f'P00_{y:.1f}_{z:.1f}_Flange_screw',screw_x(0,0,0,12).rotate((0,0,0),(0,1,0),180).translate((42,y,z+70)),2,'palm','wrist')
    for side in (-1,1):
        for z in (51,89):palm=palm.cut(cylinder(1.5,12,(43,side*46,z)))
    add('P01_Hollow_sculpted_palm',palm,5,'palm','wrist',explode=(42,0,0),tags=('multi-section-loft','hollow','boolean'))

    for side in (-1,1):
        y=side*46;support=box(16,9,46,(61,y,70),1)
        for z,r in ((56,3.05),(70,3.6),(84,3.05)):support=support.cut(cylinder(r,12,(60,y-6,z),'Y'))
        for z in (51,89):support=support.cut(cylinder(1.75,22,(48,y,z)));add(f'P03_{side}_{z}_Support_screw',screw_x(0,0,0,20).rotate((0,0,0),(0,1,0),180).translate((69,y,z)),2,'palm','wrist')
        for z in (56,84):
            support=support.cut(cylinder(.98,12,(60,y,z))).clean();grub=cylinder(.9,5.7,(63.05,y,z)).cut(cq.Workplane('YZ',origin=(68.1,y,z)).polygon(6,1.0).extrude(1).val());add(f'L05_{side}_{z}_Guide_rod_lock_screw',grub,2,'guides','wrist')
        add(f'P02_{side}_Guide_end_support',support,1,'palm','wrist',explode=(50,side*15,0));add(f'P04_{side}_Screw_end_bushing',annulus(3.55,2.5,(60,y-4,70),8,'Y'),3,'palm','wrist')
    for i,z in enumerate((56,84)):add(f'L01_{i}_Ground_guide_rod',cylinder(3,100,(60,-50,z),'Y'),2,'guides','wrist',explode=(45,0,0))

    root=cylinder(2.48,120,(60,-60,70),'Y').cut(box(2,6,8,(63.1,55,70))).cut(box(2,6,8,(63.1,-54,70)));right=z_to_y(thread_ridge(left=False),(60,4,70));left=z_to_y(thread_ridge(left=True),(60,-40,70));screw=root.fuse(right).fuse(left)
    add('L02_Opposed_helical_lead_screw',screw,2,'leadscrew','leadscrew',explode=(75,0,0),tags=('helix','trapezoid-sweep','opposite-handed'))
    clear_right=z_to_y(thread_ridge(left=False,clearance=.16),(60,4,70));clear_left=z_to_y(thread_ridge(left=True,clearance=.16),(60,-40,70));clear_root=cylinder(2.64,122,(60,-61,70),'Y');nut_tool=clear_root.fuse(clear_right).fuse(clear_left)
    knob_y=knob(0,0,0,10,8,2.5).rotate((0,0,0),(0,0,1),90).translate((60,51,70)).cut(cylinder(1.1,9,(62,55,70)));add('L03_Fluted_gripper_input_knob',knob_y,5,'leadscrew','leadscrew',tags=('knurl-flutes',))
    grub=cylinder(1,7.7,(62.1,55,70)).cut(cq.Workplane('YZ',origin=(69.1,55,70)).polygon(6,1.2).extrude(1).val());add('L04_Flat_engaging_knob_screw',grub,2,'leadscrew','leadscrew')
    retainer=annulus(5,2.5,(60,-56,70),5,'Y').cut(cylinder(.95,5,(62,-53.5,70)));add('L06_Screw_axial_retaining_collar',retainer,2,'leadscrew','leadscrew');lock=cylinder(.9,2.75,(62.1,-53.5,70)).cut(cq.Workplane('YZ',origin=(64.2,-53.5,70)).polygon(6,1.0).extrude(1).val());add('L07_Screw_collar_lock',lock,2,'leadscrew','leadscrew')
    for side in (-1,1):add(f'L08_{side}_Screw_thrust_washer',annulus(4.5,2.5,(60,50.55 if side==1 else -50.95,70),.4,'Y'),3,'palm','wrist')

    for side in (-1,1):
        y=side*JAW_CENTER;motion='jaw_plus' if side==1 else 'jaw_minus';carriage=box(17,14,42,(61.5,y,70),1.2)
        for z in (56,84):carriage=carriage.cut(cylinder(4.45,16,(60,y-8,z),'Y'));add(f'J02_{side}_{z}_Guide_bushing',annulus(4.4,3.06,(60,y-7,z),14,'Y'),3,'jaws',motion)
        carriage=carriage.cut(cq.Workplane('XZ',origin=(60,y+8,70)).polygon(6,12.15).extrude(16).val());nut=cq.Workplane('XZ',origin=(60,y+6.5,70)).polygon(6,12).extrude(13).val().cut(nut_tool);add(f'J03_{side}_Helically_cut_bronze_nut',nut,3,'jaws',motion,tags=('internal-helical-boolean',))
        finger=cq.Workplane('YZ',origin=(70,y,70)).rect(14,40).workplane(offset=27).rect(12,33).workplane(offset=40).rect(10,27).loft(combine=True).val();finger=cq.Workplane(obj=finger).edges().fillet(.30).val();opening=cq.Workplane('XZ',origin=(101,y+12,70)).slot2D(38,13,0).extrude(24).val();finger=finger.cut(opening);pad=box(25,2.2,22,(125,y-side*6,70),.6);finger=finger.fuse(carriage).cut(pad)
        for z in (61,79):finger=finger.cut(cylinder(1.0,16,(66,y-8,z),'Y'))
        for end in (-1,1):
            plate=box(14,1.4,22,(61.5,y+end*7.7,70),.3).cut(cylinder(3.5,4,(60,y+end*7.7-2,70),'Y')).cut(box(12,5,7,(55,y+end*7.7,70)))
            for z in (61,79):plate=plate.cut(cylinder(1.1,4,(66,y+end*7.7-2,z),'Y'));add(f'J08_{side}_{end}_{z}_Nut_plate_screw',screw_x(0,0,0,5,.85).rotate((0,0,0),(0,0,1),-90*end).translate((66,y+end*8.4,z)),2,'jaws',motion)
            add(f'J07_{side}_{end}_Nut_retaining_plate',plate,2,'jaws',motion)
        add(f'J04_{side}_Lofted_windowed_finger',finger,0,'jaws',motion,explode=(95,side*20,0),role='One-piece lofted finger and guide carriage with hexagonal captive-nut seat',tags=('loft','slot-cut','union','hex-pocket'))
        for k in range(7):pad=pad.cut(box(.7,3,28,(115+k*3.4,y-side*7.2,70)).rotate((125,y,70),(125,y+1,70),22))
        add(f'J06_{side}_Grooved_elastomer_pad',pad,4,'jaws',motion,explode=(113,side*23,0),tags=('groove-array',))

    from scipy.interpolate import CubicSpline
    control=np.array([[-56,0,70],[-26,0,70],[20,0,70],[41,0,70],[46,-16,70],[46,-42,70]],float);lengths=np.r_[0,np.cumsum(np.linalg.norm(np.diff(control,axis=0),axis=1))];curve=CubicSpline(lengths,control,bc_type='natural')(np.linspace(0,lengths[-1],180));path=cq.Wire.assembleEdges([cq.Edge.makeSpline([cq.Vector(*p) for p in curve],tol=1e-5)]);plane=cq.Plane(origin=tuple(curve[0]),normal=tuple(curve[1]-curve[0]));conduit=cq.Workplane(plane).circle(2.35).sweep(cq.Workplane(obj=path),isFrenet=False).val();add('C01_Through_bore_service_conduit',conduit,4,'conduit','wrist',role='Analytic routed harness envelope; removed with the palm by axial feeding through the shaft',tags=('spline','analytic-pipe-sweep'))
    connector=annulus(6.8,2.4,(-64,0,70),8).cut(cylinder(5.25,2.5,(-64,0,70)));insulator=annulus(5.2,2.4,(-64,0,70),2.4)
    for i,a in enumerate(np.linspace(0,math.tau,5,endpoint=False)):
        yy,zz=3.9*math.cos(a),70+3.9*math.sin(a);insulator=insulator.cut(cylinder(.65,3,(-64.2,yy,zz)));add(f'C03_{i}_Connector_contact',cylinder(.6,3.2,(-64.7,yy,zz)),7,'conduit','wrist')
    add('C02_Rear_service_connector',connector,2,'conduit','wrist');add('C02b_Connector_insulator',insulator,4,'conduit','wrist');add('C04_Palm_service_gland',annulus(5.5,2.45,(46,-47,70),7,'Y'),2,'conduit','wrist')

    base_label=cq.Workplane('XY',origin=(-42,-51,9.04)).text('CYBR / GEO',4.3,.13,combine=False,halign='left',valign='center').val();add('M02_Mount_identification',base_label,6,'markings');infill=cq.Workplane('XY',origin=(-34,65,8.89)).text('ORBIT 01',3.2,.025,combine=False,halign='left',valign='center').val();add('M04_Recessed_serial_infill',infill,6,'markings')
    for i,a in enumerate(np.linspace(-1.15,1.15,25)):
        if i==12:continue
        mark=box(.13,.32,2.8 if i%4==0 else 1.5,(28.08,0,108.6)).rotate(tuple(AXIS),tuple(AXIS+np.array([1,0,0])),math.degrees(a))
        for y,zz in cover_holes:
            from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
            op=BRepAlgoAPI_Cut(mark.wrapped,cylinder(3.1,1,(27.8,y,zz+70)).wrapped);op.Build()
            if not op.IsDone():raise ValueError('Failed to trim scale marking')
            result=op.Shape()
            if result.IsNull():mark=None;break
            mark=cq.Shape.cast(result)
            if not mark.Solids():mark=None;break
        if mark is not None and abs(mark.Volume())>=1e-8:add(f'M03_{i:02}_Angular_scale_tick',mark,0,'markings',explode=(52,0,0))

    metadata={'schema':'cybrgeo.orbit/1','truth_intent':'concept','units':'mm','revision':3,'geometry_api':'cybrgeo.Assembly/cybrgeo.from_shape','generated_imagery':False,
              'limitations':['Prescribed motion; no contact-force, torque or stiffness simulation.','Original concept geometry, not fabrication release.']}
    return Assembly('cybr_orbit_inspection_wrist_native',parts,MATERIALS,metadata,cad)


def local_pose(part,t):
    d=jaw_displacement(t);theta=wrist_angle(t);local=np.eye(4)
    if part.motion in ('jaw_plus','jaw_minus'):
        local[1,3]=d if part.motion=='jaw_plus' else -d
    elif part.motion=='leadscrew':
        a=-math.tau*d/LEAD;local=rotation(a,axis=(0,1,0),center=(60,0,70))
    if part.motion=='pinion':
        local=rotation(-theta*GEAR_TEETH/PINION_TEETH,axis=(1,0,0),center=tuple(AXIS+np.array([0.,CENTER_DISTANCE,0.])))
    elif part.motion!='fixed':
        local=rotation(theta,axis=(1,0,0),center=tuple(AXIS))@local
    return local


def poses(assembly:Assembly,t:float):
    return {p.name:local_pose(p,t) for p in assembly.parts}

if __name__=='__main__':print(build().validate())
