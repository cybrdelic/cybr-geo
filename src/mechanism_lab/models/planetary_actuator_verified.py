"""Verified compact 5:1 planetary gearbox actuator concept.

Original design, not vendor CAD. The geometry is deliberately explicit about
which interfaces are clearance fits, close fits, and tapped-hole envelopes.
Thread helices, loaded contact, stress, lubrication, fatigue, thermal growth and
manufacturing qualification are not claimed.
"""
from __future__ import annotations

from dataclasses import asdict
import math
import numpy as np
import cadquery as cq

from ..core import Assembly,Material,View,cad_part
from ..gears import PlanetarySpec,external_spur_gear,internal_ring_gear,planetary_angles,planet_centers,planetary_validation_report
from ..gears.planetary import planet_initial_phases,orbit_and_spin,rotation_x4

SPEC=PlanetarySpec(sun_teeth=18,planet_teeth=27,ring_teeth=72,planets=3,module=.8,pressure_angle_deg=20.,backlash_mm=.10,face_width_mm=10.,input_speed_rps=.45)

MATERIALS=[
    Material('Black hard-anodized housing',(.018,.022,.028),.72,.29,coat=.06,coat_rough=.23,anisotropy=.05,microfinish='anodized',material_source='original concept finish'),
    Material('Machined 7075-like carrier',(.48,.52,.58),.92,.21,coat=.04,coat_rough=.18,anisotropy=.12,microfinish='machined',material_source='original concept finish'),
    Material('Nitrided gear steel',(.17,.19,.21),.96,.18,coat=.02,coat_rough=.16,anisotropy=.10,microfinish='machined',material_source='original concept finish'),
    Material('Ground/turned shaft steel',(.46,.50,.54),.97,.15,coat=.02,coat_rough=.14,anisotropy=.20,microfinish='turned',material_source='original concept finish'),
    Material('Oil-impregnated bronze bushing',(.42,.24,.075),.83,.29,coat=.02,coat_rough=.24,microfinish='machined',material_source='original concept bearing choice'),
    Material('Matte elastomer seal',(.012,.014,.017),0,.62,ior=1.48,microfinish='polymer',material_source='original concept seal'),
    Material('FR4 encoder PCB',(.025,.115,.055),0,.46,ior=1.55,coat=.08,coat_rough=.20,microfinish='polymer',material_source='original concept electronics envelope'),
    Material('Copper sensor/contact metal',(.63,.23,.065),.96,.19,coat=.02,coat_rough=.16,microfinish='copper-wire',material_source='original concept conductor'),
    Material('Encoder magnet',(.075,.085,.095),.72,.34,microfinish='bead-blasted',material_source='original concept magnet envelope'),
    Material('Stainless fastener steel',(.34,.37,.40),.95,.20,anisotropy=.12,microfinish='turned',material_source='original concept fastener'),
    Material('Connector polymer',(.025,.030,.038),0,.48,ior=1.50,microfinish='polymer',material_source='original concept connector shell'),
]

def _cyl(x0,x1,y,z,r):return cq.Workplane('YZ',origin=(x0,y,z)).circle(r).extrude(x1-x0).val()
def _ann(x0,x1,y,z,ro,ri):return cq.Workplane('YZ',origin=(x0,y,z)).circle(ro).circle(ri).extrude(x1-x0).val()
def _soft(shape,r=.35):
    try:return cq.Workplane(obj=shape).edges().fillet(r).val()
    except Exception:return shape

def _holes(shape,points,r,x0,x1):
    h=cq.Workplane('YZ').pushPoints(points).circle(r).extrude(x1-x0).translate((x0,0,0))
    return cq.Workplane(obj=shape).cut(h).val()

def _disk(x0,x1,radius,center_bore=0,clearance_holes=(),clearance_r=0,tap_holes=(),tap_r=0):
    w=cq.Workplane('YZ',origin=(x0,0,0)).circle(radius)
    if center_bore:w=w.circle(center_bore)
    s=w.extrude(x1-x0).val()
    if clearance_holes:s=_holes(s,clearance_holes,clearance_r,x0-.5,x1+.5)
    if tap_holes:s=_holes(s,tap_holes,tap_r,x0-.5,x1+.5)
    return _soft(s,.30)

def _bolt(x0,x1,y,z,head_at='start',shaft_r=1.48,head_r=2.75):
    shaft=_cyl(x0,x1,y,z,shaft_r)
    if head_at=='start':
        head=_cyl(x0-2.7,x0,y,z,head_r);socket=cq.Workplane('YZ',origin=(x0-2.85,y,z)).polygon(6,2.15).extrude(1.2).val()
    else:
        head=_cyl(x1,x1+2.7,y,z,head_r);socket=cq.Workplane('YZ',origin=(x1+1.65,y,z)).polygon(6,2.15).extrude(1.2).val()
    return _soft(shaft.fuse(head).cut(socket),.10)

def _front_housing(bolts):
    s=_cyl(0,22,0,0,36)
    s=s.cut(_cyl(-1,23,0,0,6.20))
    s=s.cut(_cyl(-.2,1.5,0,0,9.05)) # lip-seal seat
    s=s.cut(_cyl(1.5,9.5,0,0,9.05))
    s=s.cut(_cyl(11,19,0,0,9.05))
    s=s.cut(_cyl(19,23,0,0,30.30))
    s=_holes(s,bolts,1.62,-.5,22.5) # M3 clearance through housing
    return _soft(s,.65)

def _rear_housing(bolts):
    s=_cyl(36,60,0,0,36)
    s=s.cut(_cyl(35,61,0,0,4.25))
    s=s.cut(_cyl(35.5,43.15,0,0,30.30))
    s=s.cut(_cyl(44,50,0,0,7.05))
    s=s.cut(_cyl(51,57,0,0,7.05))
    s=s.cut(_cyl(58,60.2,0,0,7.05)) # input lip-seal seat
    s=_holes(s,bolts,1.62,35.5,60.5)
    pocket=cq.Workplane('XY').box(10,8,8,centered=(True,True,True)).translate((48,34.5,0)).val()
    return _soft(s.cut(pocket),.65)

def _ring(bolts):
    gear=internal_ring_gear(SPEC.ring_teeth,SPEC.module,SPEC.face_width_mm,outer_radius=35,origin=(24,0,0),pressure_angle_deg=SPEC.pressure_angle_deg,backlash=SPEC.backlash_mm)
    s=gear.fuse(_ann(22,24,0,0,35,30.20)).fuse(_ann(34,36,0,0,35,30.20))
    # Tap-drill core envelopes. Screw/thread overlap is deliberate and explicitly
    # represents unmodeled thread engagement; housing holes remain clearance holes.
    s=_holes(s,bolts,1.25,21.5,24.5)
    s=_holes(s,bolts,1.25,33.5,36.5)
    return s

def _output_shaft(carrier_holes):
    s=_cyl(-12,19,0,0,6).fuse(_cyl(-18,-12,0,0,15)).fuse(_cyl(16.5,19,0,0,12))
    downstream=[(11*math.cos(math.tau*i/6),11*math.sin(math.tau*i/6)) for i in range(6)]
    s=_holes(s,downstream,1.65,-18.5,-11.5)
    s=_holes(s,carrier_holes,1.25,16,19.5) # tapped core envelope
    return _soft(s,.30)

def _motor_adapter():
    s=_ann(60,64,0,0,30,7.2)
    h=[(24*math.cos(math.pi/4+math.tau*i/4),24*math.sin(math.pi/4+math.tau*i/4)) for i in range(4)]
    return _soft(_holes(s,h,2.05,59.5,64.5),.35)

def _radial_pin(x,y0,y1,z,r):return cq.Solid.makeCylinder(r,y1-y0,cq.Vector(x,y0,z),cq.Vector(0,1,0))

def pose(part,time_seconds:float,explode:float,spec:PlanetarySpec=SPEC)->np.ndarray:
    q=planetary_angles(spec,time_seconds);m=str(part.motion)
    if m=='sun':T=rotation_x4(q['sun'])
    elif m=='carrier':T=rotation_x4(q['carrier'])
    elif m.startswith('planet_'):
        i=int(m.split('_',1)[1]);base=planet_centers(spec,0)[i];p0=planet_initial_phases(spec)[i]
        T=orbit_and_spin(base,q['carrier'],(q['planets'][i]-p0)-q['carrier'])
    else:T=np.eye(4)
    T=np.asarray(T,float).copy();T[:3,3]+=np.asarray(part.explode,float)*float(explode);return T

def build()->Assembly:
    SPEC.validate();pv=planetary_validation_report(SPEC);parts=[]
    def add(name,shape,mat,group,role,motion='fixed',center=(0,0,0),explode=(0,0,0),tol=.028,ang=.045):
        parts.append(cad_part(name,shape,mat,tolerance=tol,angular=ang,group=group,role=role,motion=motion,center=np.asarray(center,float),explode=np.asarray(explode,float),provenance='designed-concept'))

    housing_bolts=[(32.4*math.cos(math.pi/6+math.tau*i/6),32.4*math.sin(math.pi/6+math.tau*i/6)) for i in range(6)]
    output_holes=[(9.2*math.cos(math.pi/4+math.tau*i/4),9.2*math.sin(math.pi/4+math.tau*i/4)) for i in range(4)]
    centers=planet_centers(SPEC,0);phases=planet_initial_phases(SPEC)

    add('PGA01_Front_housing',_front_housing(housing_bolts),0,'front_cover','Front machined housing; explicit seal and dual output-bushing seats',explode=(-55,0,0))
    add('PGA02_Rear_housing',_rear_housing(housing_bolts),0,'housing','Rear machined housing; explicit encoder cavity, input supports and connector pocket',explode=(55,0,0))
    add('PGA03_Fixed_internal_ring',_ring(housing_bolts),2,'ring','72T fixed internal involute ring with tapped-hole core envelopes',tol=.020,ang=.035)
    for i,(y,z) in enumerate(housing_bolts):
        add(f'PGA04_Front_ring_screw_{i+1:02}',_bolt(0,24,y,z,'start'),9,'fasteners','M3-class front housing screw; thread helix omitted, ring tap-core envelope modeled',explode=(-65,0,0))
        add(f'PGA05_Rear_ring_screw_{i+1:02}',_bolt(34,60,y,z,'end'),9,'fasteners','M3-class rear housing screw; thread helix omitted, ring tap-core envelope modeled',explode=(65,0,0))

    sun=external_spur_gear(SPEC.sun_teeth,SPEC.module,SPEC.face_width_mm,origin=(24,0,0),phase=0,pressure_angle_deg=SPEC.pressure_angle_deg,backlash=SPEC.backlash_mm)
    add('PGA06_Input_sun_shaft',sun.fuse(_cyl(24,72,0,0,4)),2,'sun','Integral 18T sun/input shaft; no hidden torque joint',motion='sun',explode=(72,0,0),tol=.020,ang=.035)
    add('PGA07_Output_shaft',_output_shaft(output_holes),3,'output','12 mm dual-supported carrier output shaft; tapped carrier-flange cores modeled',motion='carrier',explode=(-72,0,0))

    add('PGA08_Output_bushing_front',_ann(1.5,9.5,0,0,9,6.08),4,'bearings','Front bronze radial bushing with 0.08 mm radial shaft clearance',explode=(-38,0,0))
    add('PGA09_Output_bushing_rear',_ann(11,19,0,0,9,6.08),4,'bearings','Rear bronze radial bushing with 0.08 mm radial shaft clearance',explode=(-28,0,0))
    add('PGA10_Input_bushing_front',_ann(44,50,0,0,7,4.08),4,'bearings','Front input bronze bushing with 0.08 mm radial shaft clearance',explode=(30,0,0))
    add('PGA11_Input_bushing_rear',_ann(51,57,0,0,7,4.08),4,'bearings','Rear input bronze bushing with 0.08 mm radial shaft clearance',explode=(40,0,0))
    add('PGA12_Output_seal',_ann(-.5,1.5,0,0,9,6.03),5,'seals','Output lip-seal envelope seated in modeled housing counterbore',explode=(-45,0,0))
    add('PGA13_Input_seal',_ann(58,60,0,0,7,4.03),5,'seals','Input lip-seal envelope seated in modeled rear counterbore',explode=(48,0,0))

    pin_holes=list(centers)
    front=_disk(19,22,26.7,center_bore=6.15,clearance_holes=pin_holes,clearance_r=2.55,tap_holes=output_holes,tap_r=1.58)
    rear=_disk(36,39,26.7,center_bore=4.65,clearance_holes=pin_holes,clearance_r=2.55)
    add('PGA14_Carrier_front',front,1,'carrier','Front carrier plate with real 5 mm pin clearances and output-fastener clearances',motion='carrier',explode=(-14,0,0))
    add('PGA15_Carrier_rear',rear,1,'carrier','Rear carrier plate with 5 mm pin and input-shaft clearances',motion='carrier',explode=(14,0,0))
    for i,(y,z) in enumerate(output_holes):
        add(f'PGA16_Output_carrier_screw_{i+1:02}',_bolt(17,22,y,z,'end',1.42,2.55),9,'carrier_fasteners','Carrier screw into modeled tapped-core output flange; thread helix omitted',motion='carrier',explode=(-18,0,0))

    for i,(y,z) in enumerate(centers):
        ex=(0,10*math.cos(math.tau*i/3),10*math.sin(math.tau*i/3))
        add(f'PGA17_Planet_pin_{i+1:02}',_cyl(18.5,39.5,y,z,2.50),3,'planet_pins','5 mm carrier-fixed planet pin',motion='carrier',center=(29,y,z),explode=ex)
        add(f'PGA18_Pin_clip_front_{i+1:02}',_ann(18.25,18.65,y,z,3.45,2.55),9,'planet_pins','Front planet-pin retaining ring',motion='carrier',center=(18.45,y,z),explode=(-8,0,0))
        add(f'PGA19_Pin_clip_rear_{i+1:02}',_ann(39.35,39.75,y,z,3.45,2.55),9,'planet_pins','Rear planet-pin retaining ring',motion='carrier',center=(39.55,y,z),explode=(8,0,0))

    for i,((y,z),phase) in enumerate(zip(centers,phases)):
        ex=(0,14*math.cos(math.tau*i/3),14*math.sin(math.tau*i/3))
        gear=external_spur_gear(SPEC.planet_teeth,SPEC.module,SPEC.face_width_mm,bore=10.04,origin=(24,y,z),phase=phase,pressure_angle_deg=SPEC.pressure_angle_deg,backlash=SPEC.backlash_mm)
        add(f'PGA20_Planet_{i+1:02}',gear,2,'planets',f'27T involute planet {i+1}; 10.04 mm bushing bore',motion=f'planet_{i}',center=(29,y,z),explode=ex,tol=.020,ang=.035)
        add(f'PGA21_Planet_bushing_front_{i+1:02}',_ann(24,29,y,z,5.0,2.56),4,'planet_bushings','Front close-fit bronze planet bushing; fit not tolerance-qualified',motion=f'planet_{i}',center=(26.5,y,z),explode=ex)
        add(f'PGA22_Planet_bushing_rear_{i+1:02}',_ann(29,34,y,z,5.0,2.56),4,'planet_bushings','Rear close-fit bronze planet bushing; fit not tolerance-qualified',motion=f'planet_{i}',center=(31.5,y,z),explode=ex)
        add(f'PGA23_Thrust_washer_front_{i+1:02}',_ann(23.55,23.90,y,z,6.1,2.57),4,'thrust_washers','Front carrier-fixed bronze thrust washer',motion='carrier',center=(23.72,y,z),explode=(-5,0,0))
        add(f'PGA24_Thrust_washer_rear_{i+1:02}',_ann(34.10,34.45,y,z,6.1,2.57),4,'thrust_washers','Rear carrier-fixed bronze thrust washer',motion='carrier',center=(34.28,y,z),explode=(5,0,0))

    add('PGA25_Motor_adapter',_motor_adapter(),1,'motor_interface','Rear motor pilot/interface flange; motor intentionally not invented',explode=(72,0,0))
    add('PGA26_Encoder_magnet',_ann(40,41.5,0,0,5.6,4.02),8,'encoder','Input-shaft encoder magnet ring envelope',motion='sun',explode=(22,0,0))
    add('PGA27_Encoder_PCB',_ann(42,42.75,0,0,15,7.3),6,'encoder','Fixed annular encoder PCB envelope',explode=(20,0,0))
    add('PGA28_Encoder_copper_ring',_ann(42.75,42.95,0,0,12,8),7,'encoder','Visible copper sensing/contact region',explode=(20,0,0))
    connector=cq.Workplane('XY').box(8,8,6,centered=(True,True,True)).translate((48,37,0)).val()
    add('PGA29_Connector_shell',_soft(connector,.45),10,'connector','Side connector shell seated into rear housing pocket',explode=(0,18,0))
    for j,x in enumerate((46.5,49.5)):
        add(f'PGA30_Connector_pin_{j+1:02}',_radial_pin(x,33,39,0,.65),7,'connector','Copper connector contact pin',explode=(0,20,0))

    views={
        'exterior':View(az=225,el=18,scale=50,target=(23,0,0),focal_length_mm=78,sensor_width_mm=36,camera_distance_mm=285,f_stop=7.1,focus_distance_mm=285,environment_strength=.18,light_size=1.55,light_intensity=.96,floor_gap_mm=2,floor_roughness=.86,title='PLANETARY ACTUATOR / EXTERIOR',note='72 mm housing / coaxial input and carrier output / original concept'),
        'hero':View(az=225,el=19,scale=48,target=(28,0,0),focal_length_mm=82,sensor_width_mm=36,camera_distance_mm=265,f_stop=6.3,focus_distance_mm=268,environment_strength=.16,light_size=1.65,light_intensity=.98,floor_gap_mm=2,floor_roughness=.88,hide=('front_cover',),title='PLANETARY ACTUATOR / 5:1 INTERNAL HERO',note='18T sun / 3 x 27T planets / 72T fixed ring / exact carrier kinematics'),
        'gear_macro':View(az=205,el=10,scale=24,target=(29,15,1),focal_length_mm=90,sensor_width_mm=36,camera_distance_mm=175,f_stop=4.5,focus_distance_mm=177,environment_strength=.13,light_size=1.75,light_intensity=1.02,floor=False,hide=('front_cover','housing','motor_interface','connector'),title='INVOLUTE MESH / MACRO',note='Explicit 0.10 mm pitch-circle backlash; root transitions are not cutter-certified'),
        'kinematics':View(az=220,el=13,scale=40,target=(29,0,0),focal_length_mm=76,sensor_width_mm=36,camera_distance_mm=230,f_stop=6.3,focus_distance_mm=230,environment_strength=.14,light_size=1.65,light_intensity=.98,floor=False,hide=('front_cover','housing','motor_interface','connector','encoder'),title='WORKING PLANETARY SET',note='Sun input; fixed ring; carrier output exactly 1/5 input speed'),
        'encoder_macro':View(az=115,el=14,scale=28,target=(43,0,0),focal_length_mm=90,sensor_width_mm=36,camera_distance_mm=185,f_stop=4,focus_distance_mm=185,environment_strength=.15,light_size=1.6,light_intensity=.96,floor=False,hide=('front_cover',),title='INPUT SUPPORT / ENCODER INTERFACE',note='Two input bushings / magnet ring / fixed PCB envelope'),
        'exploded':View(az=225,el=20,scale=95,target=(25,0,0),explode=1,focal_length_mm=72,sensor_width_mm=36,camera_distance_mm=430,f_stop=9,focus_distance_mm=430,floor=False,environment_strength=.18,light_size=1.5,light_intensity=.96,title='PLANETARY ACTUATOR / EXPLODED INSPECTION',note='Explode offsets are inspection-only, not an assembly procedure'),
        'engineering_front':View(az=180,el=0,scale=42,target=(29,0,0),projection='orthographic',floor=False,title='PLANETARY ACTUATOR / FRONT ORTHOGRAPHIC',note='Concept geometry / units mm'),
        'section':View(az=222,el=9,scale=52,target=(27,-2,0),section=(0,1,0),floor=False,focal_length_mm=72,sensor_width_mm=36,camera_distance_mm=285,f_stop=8,focus_distance_mm=285,title='PLANETARY ACTUATOR / LONGITUDINAL SECTION',note='Dual shaft bushings, ring insert, carrier and encoder support'),
    }
    metadata={
        'truth_intent':'concept','design':asdict(SPEC),'planetary_validation':pv,
        'fidelity':'Original compact planetary actuator concept; no commercial reducer internals are claimed.',
        'kinematic_model':'Exact rigid simple-planetary Willis relation with fixed ring. No loaded tooth contact, compliance or friction model.',
        'interfaces':{'housing_outer_diameter_mm':72.,'nominal_body_length_mm':64.,'output_shaft_diameter_mm':12.,'input_shaft_diameter_mm':8.,'output_support':'two 18 mm OD bronze plain radial bushings','input_support':'two 14 mm OD bronze plain radial bushings','planet_support':'two 10 mm OD bronze bushings per 5 mm carrier-fixed pin'},
        'clearances':{'output_bushing_radial_mm':.08,'input_bushing_radial_mm':.08,'planet_pin_radial_mm':.06,'planet_bushing_bore_diametral_mm':.04,'carrier_to_ring_tip_radial_mm':1.3,'planet_addendum_to_ring_root_radial_mm':.2},
        'assumptions':['Gear flanks are analytic 20 degree involutes; root transitions are concept geometry and not generated cutter envelopes.','0.10 mm backlash is a design allowance at the pitch circle; no loaded deflection or thermal growth is solved.','Housing, shafts, bushings, seals, connector, encoder and fasteners are original design geometry, not vendor CAD.','Tapped holes are represented by core envelopes; thread helices and thread-strength calculations are not modeled.','No stress, bearing PV, tooth contact, lubrication, efficiency, fatigue, noise, tolerance-stack or manufacturing qualification is claimed.'],
        'safety':['Do not fabricate/load this concept without independent gear, shaft, bearing, fastener and housing calculations.','Guard rotating gears and shafts during any physical prototype testing.'],
    }
    return Assembly('planetary_actuator',parts,MATERIALS,views,metadata=metadata,motion_function=pose)
