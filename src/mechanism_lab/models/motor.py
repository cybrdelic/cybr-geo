"""ODrive M8325s envelope reconstruction from current manufacturer specs/photos.

Documented external interfaces are dimensional anchors. Hidden internals are
explicitly an illustrative radial-flux outrunner, NOT recovered factory CAD.
"""
from __future__ import annotations
import math
import numpy as np
import cadquery as cq
from ..core import Assembly,Material,View,cad_part,axis_pose
from ..geometry import ring,sector,drill,bolt_circle,winding,cable,mesh_label

MATERIALS=[
 Material('Black anodized shell',(.034,.041,.052),.72,.32),
 Material('Machined aluminium',(.56,.59,.63),.94,.24),
 Material('Electrical steel',(.11,.13,.15),.78,.42),
 Material('Copper enamel',(.68,.265,.075),.85,.25),
 Material('Rotor magnets',(.34,.36,.39),.92,.26),
 Material('Insulation',(.047,.084,.073),.05,.55),
 Material('Bearing steel',(.57,.61,.65),1,.20),
 Material('Cable jacket',(.015,.017,.021),.05,.58),
 Material('Connector orange',(.8,.23,.026),.0,.35),
 Material('Etched identification',(.60,.61,.57),.35,.50),
 Material('Dark magnet polarity alternate',(.21,.23,.25),.92,.31),
]
SPEC=dict(diameter_mm=92.,body_length_mm=46.5,locating_pin_diameter_mm=3.,locating_pin_height_mm=6.,
          input_mount_bcd_mm=40.,input_mount_thread='M4',output_mount_bcd_mm=31.,output_mount_thread='M3',
          pole_pairs=20,kv_rpm_per_volt=100.,kt_Nm_per_phase_amp=.083,
          current_free_air_A=40.,current_forced_air_A=60.,peak_3s_A=80.)
SOURCE='https://shop.odriverobotics.com/products/m8325s'

def motor_pose(p,t,e):
    return axis_pose(t*math.tau*.14 if p.motion=='rotor' else 0,p.center,p.explode,e)

def build() -> Assembly:
    parts=[]
    def add(name,shape,mat,group,rotor=False,ex=(0,0,0),provenance='inferred-internal',role=''):
        p=cad_part(name,shape,mat,group=group,motion='rotor' if rotor else 'fixed',explode=np.array(ex,float),
                   role=role,provenance=provenance);parts.append(p);return p
    # Closed cup with genuinely cut-through shoulder vents, not painted dark slots.
    prof=[(4,43.2),(4,45.6),(4.4,46),(31,46),(45.8,42.4),(46.5,42),(46.5,8),(42.6,8),(42.6,39.7),(30.6,43.2)]
    bell=cq.Workplane('XY').polyline(prof).close().revolve(360,(0,0),(1,0)).val()
    for j in range(12):
        cutter=cq.Workplane('XZ').center(37.3,0).slot2D(17.2,7.9,90).extrude(12,both=True).translate((0,43,0)).val().rotate((0,0,0),(1,0,0),j*30)
        bell=bell.cut(cutter)
    bell=drill(bell,bolt_circle(15.5),1.5,42.2,47)
    bell=drill(bell,bolt_circle(15.5),2.65,45.5,47)
    add('M01_Vented_rotor_bell',bell,0,'rotor_shell',True,(72,0,0),'documented-envelope/photo-estimate',
        '92 mm OD, 46.5 mm front plane and 31 mm output BCD documented; contour and wall thickness photo-estimated')
    # Separate magnetic backiron liner, hidden in assembled views.
    add('M02_Rotor_backiron',ring(43.15,42.7,8.5,34),2,'rotor_magnets',True,(45,0,0))
    for j in range(40):
        a=j*math.tau/40;da=math.radians(8.4)
        add(f'M03_Magnet_{j+1:02}',sector(42.66,40.15,9.0,33.7,a-da/2,a+da/2),4 if j%2==0 else 10,
            'rotor_magnets',True,(40,0,0),'documented-count/inferred-shape','40 representative permanent magnet poles; shape/grade/bondline not published')
    # Open stator mounting plate: four torque reaction spokes and 12 rim vents.
    plate=ring(44.5,8.7,0,3.5)
    for j in range(4):
        a=j*math.pi/2
        plate=plate.cut(sector(26,14,-1,4,a+math.radians(16),a+math.radians(74)))
    for j in range(12):
        a=j*math.pi/6
        plate=plate.cut(sector(41.8,29.0,-1,4,a+.065,a+math.pi/6-.065))
    plate=drill(plate,bolt_circle(20),2,-1,4)
    add('M04_Fixed_four_hole_backplate',plate,0,'stator_structure',False,(-75,0,0),'documented-interface/photo-estimate',
        '4 x M4 on 40 mm bolt circle; ventilation openings estimated from rear photographs')
    add('M05_Bearing_cartridge',ring(13.2,11.02,3.5,39.4),1,'stator_structure',ex=(-45,0,0))
    add('M06_Rotor_shaft',ring(4,0,-5,46.5),6,'rotor_shaft',True,(25,0,0))
    add('M07_Front_center_hub',ring(8,4,42.6,46.5),1,'rotor_shaft',True,(93,0,0),'photo-estimate')
    add('M08_3mm_locating_pin',ring(1.5,0,46.5,52.5),6,'rotor_shaft',True,(95,0,0),'documented-interface',
        'Locating only: diameter 3 mm, projection 6 mm; MUST NOT carry drive torque')
    add('M09_Rear_encoder_magnet',ring(3,0,-6.4,-5),4,'rotor_shaft',True,(-98,0,0),'photo-estimate')
    # 36 slots is a declared modeling assumption, not manufacturer data.
    steel=ring(28,13.2,10.0,34.8)
    for j in range(36):
        sh=cq.Workplane('YZ').polyline([(27.4,-1.8),(38.8,-2.6),(39.2,-2.6),(39.2,2.6),(38.8,2.6),(27.4,1.8)]).close().extrude(24.8).translate((10,0,0)).val()
        steel=steel.fuse(sh.rotate((0,0,0),(1,0,0),j*10))
    steel=steel.clean()
    add('M10_36_slot_lamination_stack',steel,2,'stator_core',ex=(-25,0,0),role='Assumed 36-slot stator; actual slot topology and stack schedule unverified')
    # Lamination reveal rings are individual thin geometry, not a texture.
    for j in range(19):
        add(f'M11_Stack_seam_{j+1:02}',ring(28.025,27.98,10.45+j*1.24,10.51+j*1.24),5,'stator_core',ex=(-25,0,0),role='Illustrative stack edge lines, not a lamination thickness claim')
    for j in range(36):
        a=j*math.tau/36
        ins=cq.Workplane('YZ').center(32.5,0).rect(10.0,4.0).extrude(26).translate((9.4,0,0)).val().rotate((0,0,0),(1,0,0),j*10)
        # Insulators are thin front/back caps, avoiding a filled coil cavity.
        cap1=sector(38.8,27.5,9.4,9.8,a-.065,a+.065)
        cap2=sector(38.8,27.5,35.0,35.4,a-.065,a+.065)
        add(f'M12_Bobbin_{j+1:02}',cq.Compound.makeCompound([cap1,cap2]),5,'windings',ex=(-30,14*math.cos(a),14*math.sin(a)))
        parts.append(winding(f'M13_Copper_coil_{j+1:02}',a,3))
    # Bearings: explicit races, shields and balls. Bearing internal kinematics omitted.
    for side,x in [('rear',4),('front',35.2)]:
        add(f'M14_{side}_outer_race',ring(11,8.5,x,x+6),6,'bearings',ex=(-105 if side=='rear' else 105,0,0))
        add(f'M15_{side}_inner_race',ring(6,4.02,x,x+6),6,'bearings',True,(-105 if side=='rear' else 105,0,0))
        for j in range(10):
            y,z=bolt_circle(7.2,10)[j]
            ball=cq.Solid.makeSphere(1.18,cq.Vector(x+3,y,z),angleDegrees1=-90,angleDegrees2=90)
            add(f'M16_{side}_ball_{j+1:02}',ball,6,'bearings',ex=(-107 if side=='rear' else 107,0,0),role='Visual rolling element; orbital/spin motion not simulated')
        for sign,xx in [('a',x+.3),('b',x+5.5)]:
            add(f'M17_{side}_shield_{sign}',ring(10.4,6.3,xx,xx+.2),0,'bearings',ex=(-110 if side=='rear' else 110,0,0))
    for j,(y,z) in enumerate(bolt_circle(15.5)):
        ins=ring(2.0,1.24,42.8,45.6).translate((0,y,z))
        add(f'M18_M3_thread_insert_{j+1}',ins,6,'rotor_shell',True,(72,0,0),'documented-thread/inferred-insert','M3 nominal thread shown with a bore; helical thread profile not modeled')
    # Bare exposed cable route and representative orange 3-pole connector.
    for j,z in enumerate([-5,0,5]):
        parts.append(cable(f'M19_Phase_lead_{j+1}',[(2,-40,z),(-4,-46,z),(-8,-58,z),(-6,-74,z)],7))
    connector=cq.Workplane('XY').box(15,18,23).translate((-6,-83,0)).edges('|Y').fillet(1.5).val()
    for z in [-6,0,6]:
        hole=cq.Solid.makeCylinder(2.35,20,cq.Vector(-6,-93,z),cq.Vector(0,1,0));connector=connector.cut(hole)
    add('M20_Representative_connector_shell',connector,8,'cables',ex=(-15,-20,0),provenance='photo-estimate',role='Representative connector envelope, not an interchangeable LCC40 CAD model')
    latch=cq.Workplane('XY').box(8,10,2).translate((-6,-83,12.3)).val()
    add('M21_Connector_latch',latch,8,'cables',ex=(-15,-20,3),provenance='photo-estimate')
    for j,z in enumerate([-6,0,6]):
        pin=cq.Solid.makeCylinder(1.45,8,cq.Vector(-6,-87,z),cq.Vector(0,1,0))
        add(f'M22_Connector_contact_{j+1}',pin,6,'cables',ex=(-15,-20,0),provenance='inferred-internal')
    parts.append(mesh_label('M8325s  100KV',x0=17,theta=-.7,radius=46.05,size=2.05,material=9))
    views={
      'hero':View(az=40,el=26,scale=77,target=(23,-17,0),title='M8325s / 100KV',note='Documented envelope. Photo-based exterior. Declared internal assumptions.'),
      'rear':View(az=142,el=25,scale=77,target=(15,-17,0),title='FIXED BACKPLATE / WINDINGS',note='40 mm M4 mounting circle. Magnet encoder and three phase leads.'),
      'internal':View(az=43,el=25,scale=65,target=(23,0,0),hide=('rotor_shell','markings','cables'),title='ROTOR / STATOR INTERNAL STUDY',note='40 magnet poles; 36-slot stator and winding schedule are illustrative.'),
      'stator':View(az=140,el=25,scale=56,target=(23,0,0),hide=('rotor_shell','rotor_magnets','rotor_shaft','markings','cables','stator_structure'),title='STATOR AND COPPER WINDINGS',note='Every visible conductor is a tube mesh, not an image texture.'),
      'section':View(az=47,el=17,scale=65,target=(23,-7,0),section=(0,1,0),hide=('cables','markings'),title='LONGITUDINAL SECTION',note='Geometric clipped section. Hidden dimensions are not factory data.'),
      'exploded':View(az=67,el=25,scale=108,target=(26,-8,0),explode=1,title='AXIAL EXPLODED INSPECTION',note='Inspection offsets, not a verified disassembly procedure.'),
    }
    return Assembly('m8325s',parts,MATERIALS,views,metadata=dict(specification=SPEC,sources=[SOURCE,'https://docs.odriverobotics.com/v/latest/hardware/odrive-motors.html'],
       fidelity='Documented external interface; photo-estimated exterior and illustrative internals',
       assumptions=['36 stator slots','14 displayed conductor turns per tooth','magnet dimensions and airgap','bearing size and fits','wire and connector envelope','no winding schedule or electromagnetic solution'],
       drawings={'default': {'parts': ['M01_Vented_rotor_bell', 'M04_Fixed_four_hole_backplate', 'M07_Front_center_hub', 'M08_3mm_locating_pin'], 'title': 'M8325s / EXTERNAL INTERFACE STUDY', 'auto_dimensions': False, 'annotations': [{'kind': 'dimension', 'view': 'side', 'points': [[0, -46], [46.5, -46]], 'label': '46.5'}, {'kind': 'dimension', 'view': 'end', 'points': [[-46, 0], [46, 0]], 'offset_paper_mm': 1, 'offset_model_mm': 55, 'label': 'DIA 92'}, {'kind': 'line', 'points': [[-51, 0], [51, 0]]}, {'kind': 'line', 'points': [[0, -51], [0, 51]]}, {'kind': 'circle', 'radius': 15.5}, {'kind': 'leader', 'point': [15.5, 0], 'elbow_paper_mm': [25, -15], 'end_paper_mm': [38, -15], 'text': '4 x M3 / BCD 31'}, {'kind': 'leader', 'view': 'side', 'point': [51, 0], 'elbow_paper_mm': [12, -22], 'end_paper_mm': [18, -22], 'text': 'DIA 3 x 6 locating pin'}, {'kind': 'note', 'paper_xy': [194, 166], 'text': 'REAR INTERFACE', 'size': 2.6, 'bold': True}, {'kind': 'note', 'paper_xy': [194, 172], 'text': '4 x M4 / BCD 40'}, {'kind': 'note', 'paper_xy': [194, 178], 'text': 'Torque via M3 face bolts.', 'size': 2.4}, {'kind': 'note', 'paper_xy': [194, 183], 'text': 'Pin is for centering only.', 'size': 2.4}, {'kind': 'note', 'paper_xy': [17, 34], 'text': 'External envelope / nominal manufacturer interface dimensions', 'size': 2.8}]}}, drawing_groups=['rotor_shell','stator_structure','rotor_shaft']),motion_function=motor_pose)
