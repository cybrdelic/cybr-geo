"""Custom parallel-axis carrier-drive study; deliberately not drop-in certification.

M8325s rotating face -> 32T pulley -> 600 mm nominal pitch-length belt ->
72T annular pulley -> eight-hole CARRIER flange. Both differential outputs stay free.
Pulley/belt tooth shapes are inspection approximations, not a licensed HTD profile.
"""
from __future__ import annotations
import math
from dataclasses import replace
import numpy as np
import cadquery as cq
from scipy.optimize import brentq
from ..core import Assembly,View,Material,Part,cad_part,mesh_part,axis_pose,rotation_x
from ..geometry import ring,bolt_circle,drill,sector
from . import motor,differential

PITCH=5.;DRIVER_TEETH=32;DRIVEN_TEETH=72;BELT_TEETH=120
R=DRIVEN_TEETH*PITCH/math.tau;r=DRIVER_TEETH*PITCH/math.tau

def belt_length(c):
    beta=math.acos(-(R-r)/c)
    return 2*math.sqrt(c*c-(R-r)**2)+r*(math.tau-2*beta)+R*2*beta
CENTER=brentq(lambda c:belt_length(c)-BELT_TEETH*PITCH,110.,250.)
BETA=math.acos(-(R-r)/CENTER);STRAIGHT=math.sqrt(CENTER**2-(R-r)**2)
SMALL_ARC=r*(math.tau-2*BETA);BIG_ARC=R*2*BETA
LENGTH=belt_length(CENTER);MOTOR_OFFSET=np.array([16.5,-CENTER,0])

def belt_sample(s):
    s=s%LENGTH;n0=np.array([math.cos(BETA),math.sin(BETA)])
    if s<STRAIGHT:
        a=R*n0;b=np.array([-CENTER,0])+r*n0;v=(b-a)/STRAIGHT
        return a+v*s,v,np.array([v[1],-v[0]])
    s-=STRAIGHT
    if s<SMALL_ARC:
        a=BETA+s/r;normal=np.array([math.cos(a),math.sin(a)])
        return np.array([-CENTER,0])+r*normal,np.array([-normal[1],normal[0]]),normal
    s-=SMALL_ARC
    if s<STRAIGHT:
        n=np.array([math.cos(BETA),-math.sin(BETA)])
        a=np.array([-CENTER,0])+r*n;b=R*n;v=(b-a)/STRAIGHT
        return a+v*s,v,np.array([v[1],-v[0]])
    s-=STRAIGHT;a=math.tau-BETA+s/R;normal=np.array([math.cos(a),math.sin(a)])
    return R*normal,np.array([-normal[1],normal[0]]),normal

def belt_body(x0=66.6,x1=79.4):
    v=[];f=[];N=640
    for s in np.linspace(0,LENGTH,N,endpoint=False):
        yz,t,n=belt_sample(s)
        for x,off in [(x0,.55),(x1,.55),(x1,2.5),(x0,2.5)]:v.append([x,*(yz+off*n)])
    for i in range(N):
        for j in range(4):
            a=4*i+j;b=4*i+(j+1)%4;c=4*((i+1)%N)+(j+1)%4;d=4*((i+1)%N)+j
            f.extend([[a,b,c],[a,c,d]])
    return v,f

def pulley(teeth,bore,x0=67,x1=79):
    pr=teeth*PITCH/math.tau
    # Rounded multi-point teeth for visual inspection, NOT certified HTD geometry.
    a=np.linspace(0,math.tau,teeth*16,endpoint=False)
    h=(.5+.5*np.cos(teeth*a))**.7
    rr=pr-2.2+1.7*h
    points=np.column_stack([rr*np.cos(a),rr*np.sin(a)])
    sh=cq.Workplane('YZ').polyline(points.tolist()).close().extrude(x1-x0).translate((x0,0,0)).val()
    return sh.cut(ring(bore,0,x0-1,x1+1))

def pose(p,t,e):
    # Deliberately slow inspection rates; they are not measured or commanded hardware speeds.
    wm=18*math.tau/60;wc=wm*DRIVER_TEETH/DRIVEN_TEETH;wd=2.4*math.tau/60
    if p.motion.startswith('belt_'):
        index=int(p.motion.split('_')[1]);yz,tan,n=belt_sample(index*PITCH+wm*r*t)
        T=axis_pose(math.atan2(tan[1],tan[0]));T[:3,3]=[73,*(yz-.05*n)];return T
    if p.motion=='rotor':return axis_pose(wm*t,p.center,p.explode,e)
    if p.motion=='fixed':return axis_pose(0,explode=p.explode,amount=e)
    c=wc*t;d=wd*t;angle=c+d if p.motion=='left' else c-d if p.motion=='right' else c
    T=axis_pose(angle,explode=p.explode,amount=e)
    if p.motion in ['planetA','planetB']:
        q=(-48/18 if p.motion=='planetA' else 48/18)*d
        Rc=rotation_x(c);Rl=rotation_x(q);ct=np.asarray(p.center)
        T[:3,:3]=Rc@Rl;T[:3,3]=Rc@(ct-Rl@ct)+np.asarray(p.explode)*e
    return T

def build():
    diff=differential.build('working');m=motor.build();materials=list(diff.materials)+list(m.materials)
    parts=list(diff.parts);offset=len(diff.materials)
    for p in m.parts:
        q=p.moved(MOTOR_OFFSET,prefix='Motor_');q.material+=offset;parts.append(q)
    aluminium=len(materials);materials.append(Material('Stage aluminium',(.48,.52,.57),.95,.25))
    black=len(materials);materials.append(Material('Timing belt rubber',(.018,.022,.026),0,.58))
    mark=len(materials);materials.append(Material('Belt index mark',(.72,.24,.04),0,.42))
    base_mat=len(materials);materials.append(Material('Fixture anodized base',(.07,.105,.14),.8,.35))
    def add(name,sh,mat=aluminium,motion='carrier',group='drive_stage',center=(0,0,0),ex=(0,0,0),role='Custom adapter concept; tolerances and loads unqualified'):
        p=cad_part(name,sh,mat,motion=motion,group=group,center=np.array(center,float),explode=np.array(ex,float),role=role,provenance='designed-concept');parts.append(p);return p
    # Rear flange uses eight holes at 49.8 mm radius / 22.5 degree phase in the actual preserved CAD.
    carrier_adapter=drill(ring(55.6,30.0,55,67),bolt_circle(49.8,8,math.pi/8),4.5,54,68)
    add('Drive_01_99p6_BCD_carrier_adapter',carrier_adapter,ex=(30,0,0),role='8 x 9 mm clearance on 99.6 mm BCD; matches nominal hole locations of preserved carrier flange. 60 mm central opening leaves output hub free.')
    driven=drill(pulley(72,30),bolt_circle(49.8,8,math.pi/8),4.5,66,80)
    add('Drive_02_72T_annular_pulley',driven,ex=(50,0,0),role='72 teeth / 5 mm pitch / nominal 2.25:1 ratio. Inspection tooth form only.')
    for side,x in [('back',66.1),('front',79.4)]:
        add('Drive_03_driven_'+side+'_guide',drill(ring(R+3,30,x,x+.8),bolt_circle(49.8,8,math.pi/8),4.5,x-1,x+2),ex=(50,0,0))
    motor_adapter=drill(ring(21,1.6,63,67),bolt_circle(15.5),1.7,62,68).translate((0,-CENTER,0))
    add('Drive_04_M3_face_adapter',motor_adapter,motion='rotor',center=(0,-CENTER,0),ex=(30,0,0),role='Torque through four M3 face bolts on 31 mm BCD. 3.2 mm center bore only locates over the 3 mm pin.')
    small=drill(pulley(32,1.6),bolt_circle(15.5),1.7,66,80)
    small=drill(small,bolt_circle(15.5),3.0,75.6,80).translate((0,-CENTER,0))
    add('Drive_05_32T_motor_pulley',small,motion='rotor',center=(0,-CENTER,0),ex=(50,0,0))
    for side,x in [('back',66.1),('front',79.4)]:
        sh=drill(ring(r+3,1.6,x,x+.8),bolt_circle(15.5),3,x-1,x+2).translate((0,-CENTER,0))
        add('Drive_06_motor_'+side+'_guide',sh,motion='rotor',center=(0,-CENTER,0),ex=(50,0,0))
    v,f=belt_body();parts.append(mesh_part('Drive_07_belt_backing',v,f,black,group='belt',motion='fixed',role='600 mm pitch loop / 120 teeth. No belt stretch, contact or tension model.'))
    # Real animated tooth blocks advect around exact tangent/arc pitch path.
    tooth=cq.Workplane('XY').box(12.2,2.45,1.55).val()
    for j in range(BELT_TEETH):
        add(f'Drive_08_belt_tooth_{j+1:03}',tooth,mark if j%20==0 else black,motion=f'belt_{j}',group='belt',role='Illustrative belt tooth with prescribed no-slip pitch-line travel')
    for j,(y,z) in enumerate(bolt_circle(49.8,8,math.pi/8)):
        sh=cq.Workplane('YZ').polygon(6,12).extrude(4.7).translate((80.2,y,z)).val()
        add(f'Drive_09_carrier_bolt_head_{j+1}',sh,ex=(64,0,0),role='Visual bolt head. Fastener engagement/grade/clamp load not qualified.')
    # Fixture and bearing envelopes are explicit concept parts, not asserted COTS bearing selections.
    for label,x0,x1,inner,outer in [('front',-57,-46,52.10,62.1),('rear',45,55,58.4,68.4)]:
        add('Fixture_'+label+'_bearing_envelope',ring(outer,inner,x0,x1),motion='fixed',group='fixture',role='Unspecified bearing envelope; journal fit, bearing type and radial-load capacity unvalidated')
        block=cq.Workplane('YZ').center(0,-16).rect(2*(outer+9),128).extrude(x1-x0).translate((x0,0,0)).val()
        block=block.cut(ring(outer+.1,0,x0-1,x1+1))
        add('Fixture_'+label+'_support',block,base_mat,motion='fixed',group='fixture',role='Support-envelope concept, not a bearing housing drawing released for machining')
    mount=cq.Workplane('YZ').center(-CENTER,-16).rect(103,128).extrude(6).translate((10.5,0,0)).val()
    mount=drill(mount,[(y-CENTER,z) for y,z in bolt_circle(20)],2.25,9,18)
    mount=mount.cut(ring(14,0,9,18).translate((0,-CENTER,0)))
    for j in range(4):mount=mount.cut(sector(39,25,9,18,j*math.pi/2+.15,(j+1)*math.pi/2-.15).translate((0,-CENTER,0)))
    add('Fixture_motor_M4_mount',mount,base_mat,motion='fixed',group='fixture',role='4 x 4.5 mm clearance on 40 mm BCD; slots and support dimensions are custom assumptions')
    base=cq.Workplane('XY').box(160,CENTER+190,8).translate((7,-CENTER/2,-84)).edges('|Z').fillet(6).val()
    add('Fixture_base',base,base_mat,motion='fixed',group='fixture')
    views={
      'hero':View(az=32,el=25,scale=149,target=(10,-CENTER/2,-6),title='MOTOR-TO-CARRIER DRIVE STUDY',note='32:72 pulley ratio. Two free differential outputs. Adapter-dependent concept, not drop-in hardware.'),
      'drive_face':View(az=3,el=9,scale=134,target=(20,-CENTER/2,0),hide=('fixture',),title='32T : 72T / 2.25:1',note='Motor face bolts drive the carrier flange; the locating pin carries no intended torque.'),
      'internal':View(az=230,el=27,scale=137,target=(0,-CENTER/2,0),hide=('carrier','front_flange','rear_flange','marking','rotor_shell','markings','fixture','front_hub','rear_hub'),title='COUPLED KINEMATIC INSPECTION',note='Prescribed motor, carrier, pinion and output motion; no loaded-contact or electromagnetic solution.'),
      'exploded':View(az=67,el=29,scale=205,target=(20,-CENTER/2,10),explode=1,hide=('fixture',),title='DRIVE ASSEMBLY / EXPLODED',note='Custom drive interface is separate from the preserved differential geometry.')
    }
    return Assembly('drivetrain',parts,materials,views,metadata=dict(ratio=DRIVEN_TEETH/DRIVER_TEETH,
        motor_teeth=DRIVER_TEETH,carrier_teeth=DRIVEN_TEETH,pitch_mm=PITCH,belt_teeth=BELT_TEETH,
        pitch_length_mm=LENGTH,center_distance_mm=CENTER,motor_offset_mm=MOTOR_OFFSET.tolist(),
        motor_rpm_demo=18.,carrier_rpm_demo=8.,left_rpm_demo=10.4,right_rpm_demo=5.6,
        compatibility='Nominal mounting layout only. Requires custom carrier adapter, belt stage, controller, bearings, guards and engineering qualification.',
        drawings={'Drive_01_99p6_BCD_carrier_adapter': {'title': 'CARRIER ADAPTER / CUSTOM INTERFACE', 'annotations': [{'kind': 'circle', 'radius': 49.8}, {'kind': 'line', 'points': [[-61, 0], [61, 0]]}, {'kind': 'line', 'points': [[0, -61], [0, 61]]}, {'kind': 'leader', 'point': [46.01, 19.06], 'elbow_paper_mm': [20, -24], 'end_paper_mm': [36, -24], 'text': '8 x DIA 9 / BCD 99.6'}, {'kind': 'leader', 'point': [0, -30], 'elbow_paper_mm': [32, 5], 'end_paper_mm': [46, 5], 'text': 'DIA 60 THROUGH'}, {'kind': 'note', 'paper_xy': [17, 34], 'text': 'Custom carrier interface / dimensions derived from our retained model, not manufacturer data.'}, {'kind': 'note', 'paper_xy': [194, 160], 'text': '8 holes equally spaced', 'size': 2.6}, {'kind': 'note', 'paper_xy': [194, 165], 'text': '22.5 deg initial phase', 'size': 2.6}, {'kind': 'dimension', 'view': 'side', 'points': [[55, -55.6], [67, -55.6]], 'label': '12'}, {'kind': 'dimension', 'view': 'end', 'points': [[-55.6, 0], [55.6, 0]], 'offset_model_mm': 55.6, 'offset_paper_mm': 7, 'label': 'DIA 111.2'}], 'auto_dimensions': False}, 'Drive_04_M3_face_adapter': {'title': 'MOTOR FACE ADAPTER / CUSTOM INTERFACE', 'annotations': [{'kind': 'circle', 'radius': 15.5}, {'kind': 'line', 'points': [[-25, 0], [25, 0]]}, {'kind': 'line', 'points': [[0, -25], [0, 25]]}, {'kind': 'leader', 'point': [15.5, 0], 'elbow_paper_mm': [20, -18], 'end_paper_mm': [30, -18], 'text': '4 x DIA 3.4 / BCD 31'}, {'kind': 'leader', 'point': [1.6, 0], 'elbow_paper_mm': [26, 14], 'end_paper_mm': [45, 14], 'text': 'DIA 3.2 LOCATING'}, {'kind': 'note', 'paper_xy': [17, 34], 'text': 'Custom face adapter / nominal clearance holes. Screw grade, length, engagement and clamping not qualified.'}, {'kind': 'dimension', 'view': 'side', 'points': [[63, -21], [67, -21]], 'label': '4'}, {'kind': 'dimension', 'view': 'end', 'points': [[-21, 0], [21, 0]], 'offset_model_mm': 21, 'offset_paper_mm': 7, 'label': 'DIA 42'}], 'auto_dimensions': False, 'anchor_origins': {'end': [-CENTER, 0]}}}, drawing_groups=['drive_stage','fixture']),motion_function=pose)
