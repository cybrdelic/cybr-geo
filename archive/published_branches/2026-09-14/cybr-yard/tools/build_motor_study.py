"""Prepare a source-faithful manufacturer model plus a separately authored drive study.

No electromagnetic internals or claimed motor manufacturing data are fabricated.
Housing appearance is an approximate material assignment from product photographs.
"""
from pathlib import Path
from copy import deepcopy
import json,math,sys
import numpy as np
import cadquery as cq
from cybrgeo import Assembly,Part,Material,from_shape,translation,rotation
ROOT=Path(__file__).resolve().parents[1]
MOTOR=ROOT/'examples/neo_vortex'
ROTATING={9,10,11,12,13,14,15,16,17,7,8}
ALIASES={33:'SPARK_Flex_shell',9:'Rotor_hub',10:'Outrunner_rotor_can',18:'Stationary_motor_frame',
         19:'Main_bearing',24:'Motor_docking_cover',29:'Phase_connector_insulator',30:'Phase_terminal_A',
         31:'Phase_terminal_B',32:'Phase_terminal_C',7:'8mm_keyed_output_shaft',8:'Shaft_clamp',
         40:'Controller_internal_carrier',35:'Controller_phase_terminal_A',36:'Controller_phase_terminal_B',
         37:'Controller_phase_terminal_C',38:'Power_lead_A',39:'Power_lead_B',43:'Signal_harness',
         45:'Dock_connector',46:'Controller_button',51:'USB_connector'}

def instance(p):return int(p.metadata['source_node'].split('/NAUO')[-1])

def prepare_motor():
    a=Assembly.load(MOTOR/'geometry',True)
    # Retain all arrays and B-reps. Only names, styling and inspection transforms change.
    original={p.name:p.vertices.copy() for p in a.parts}
    a.materials=[Material('Black anodized alloy',(.018,.021,.025),.75,.28),
       Material('Stainless steel',(.40,.43,.46),.95,.22),Material('Insulating polymer',(.018,.020,.022),0,.44),
       Material('Gold plated contacts',(.58,.33,.06),.95,.23),Material('Natural machined alloy',(.45,.48,.5),.95,.26)]
    cad={}
    for p in a.parts:
        num=instance(p);old=p.name
        alias=ALIASES.get(num,'Fastener' if num in [3,4,5,6,11,12,13,14,15,16,21,22,25,26,27,28,47,48] else 'Connector_component')
        p.name=f'{old[:3]}_{alias}'+('_'+old.split('_')[-1] if 'solid' in old else '')
        p.metadata.update(original_mesh_name=old,source_instance=num)
        p.group='cable' if p.bounds[0,1]<-100 else ('rotor' if num in [7,8,9,10,11,12,13,14,15,16,17] else 'stationary')
        p.motion='motor' if p.group=='rotor' else 'fixed'
        p.material=1
        if num in [33,10,18,24]:p.material=0
        elif num in [35,36,37,30,31,32]:p.material=3
        elif num in [9,29,40,45,46,49,50,52,53,54,38,39,43]:p.material=2
        elif num==51:p.material=4
        if p.group=='cable':p.explode=np.array([-70,0,-35.])
        elif num==33:p.explode=np.array([-108.,0,0])
        elif num==10:p.explode=np.array([100.,0,0])
        elif num==9:p.explode=np.array([47.,0,0])
        elif num in [7,8]:p.explode=np.array([-155. if num==7 else 122.,0,0])
        elif num in range(11,17):p.explode=np.array([140.,0,0])
        elif num==19:p.explode=np.array([-30.,0,0])
        elif p.group=='stationary' and p.bounds.mean(0)[0]<-19:p.explode=np.array([-70.,0,0])
        else:p.explode=np.zeros(3)
        cad[p.name]=a.cad[old]
        assert np.array_equal(original[old],p.vertices)
    a.cad=cad;a.metadata.update(known_solid_count=75,retained_surface_bodies=1,
        appearance='Photo-informed approximate materials, not measured optical properties',
        no_missing_geometry_invented=True,
        limits='Manufacturer mechanical export does not disclose detailed windings, magnet array, sensor/electronic circuitry or validated thermal behavior.')
    # Save using a new scene; retain raw vendor geometry/cache unmodified.
    a.save(MOTOR/'study');a.export_glb(MOTOR/'study/motor_complete.glb')
    a.export_parts(MOTOR/'parts/models')
    return a

def gear_shape(teeth,module,width,bore,x0,center_y=0,phase=0):
    """True involute flanks with polygonal sampling; root fillet remains approximate."""
    from cybrgeo.features import involute_profile
    p=involute_profile(teeth,module,backlash=.10,nflank=12)
    yz=np.c_[p[:,0]*np.cos(p[:,1]+phase),p[:,0]*np.sin(p[:,1]+phase)]
    sh=cq.Workplane('YZ',origin=(x0,center_y,0)).polyline(yz.tolist()).close().extrude(width)
    sh=sh.cut(cq.Workplane('YZ',origin=(x0-1,center_y,0)).circle(bore/2).extrude(width+2))
    return sh

def build_drive(motor):
    # Motor =20T, carrier=80T, transverse module1.75, standard external gear center distance87.5.
    n1,n2,module,width=20,80,1.75,10.;center_y=(n1+n2)*module/2
    sys.path.insert(0,str(ROOT/'examples/differential/src'))
    import model as old
    original=old.load_parts(old.GEOM/'working_variant_parts.npz')
    mats=[Material(**{k:v for k,v in m.items() if k in Material.__dataclass_fields__}) for m in old.MATERIALS]
    parts=[Part(p.name,p.vertices,p.faces,p.normals,p.material,p.group,p.explode,p.role,p.motion,p.center,
                metadata={'source':'preserved v3 working alternative'}) for p in original]
    motorstart=len(parts);off=len(mats);mats.extend(motor.materials)
    for p in motor.parts:
        if p.group=='cable':continue # Kept in full motor file, omitted from this clearance study.
        q=p.transformed(translation((135,center_y,0)));q.name='Motor_'+p.name;q.material+=off
        q.center=np.array([135,center_y,0]);q.explode=np.array([0,45,50.]);parts.append(q)
    cad={}
    # Ring-shaped driven gear fits around the existing output hub. Input acts on carrier flange.
    ring=gear_shape(n2,module,15,76,55,phase=math.pi/n2)
    # Existing rear-flange hole centers: R49.8 mm, angular phase22.5 degrees.
    centers=[(49.8*math.cos(math.radians(22.5+45*i)),49.8*math.sin(math.radians(22.5+45*i))) for i in range(8)]
    for y,z in centers:
        ring=ring.cut(cq.Workplane('YZ',origin=(54,y,z)).circle(4.9).extrude(18))
    pinion=gear_shape(n1,module,width,8.05,60,center_y,phase=0)
    # 2mm keyed shaft interface clearance; key length must be selected from actual shaft seating.
    pinion=pinion.cut(cq.Workplane('YZ',origin=(59,center_y+4,0)).rect(2.2,2.05).extrude(12))
    # Stationary motor bracket mates six documented holes and provides an output bearing opening.
    mounting=(cq.Workplane('YZ',origin=(90,center_y,-18)).rect(80,114).extrude(5).edges('|X').fillet(6))
    mounting=mounting.cut(cq.Workplane('YZ',origin=(89,center_y,0)).circle(16.5).extrude(7))
    for deg in [0,45,135,180,225,315]:
        y,z=25.4*math.cos(math.radians(deg)),25.4*math.sin(math.radians(deg))
        mounting=mounting.cut(cq.Workplane('YZ',origin=(89,center_y+y,z)).circle(2.6).extrude(7))
    base=(cq.Workplane('XY',origin=(114,center_y,-78)).box(62,90,6).edges('|Z').fillet(4))
    # Bracket foot and vertical face overlap structurally, union into one CAD solid.
    mounting=mounting.union(base)
    mats.extend([Material('Drive gear steel',(.22,.25,.29),.95,.27),Material('Bracket aluminum',(.24,.29,.34),.85,.32)])
    for name,shape,mi,motion in [('Carrier_drive_80T',ring,len(mats)-2,'carrier'),
                               ('Motor_pinion_20T',pinion,len(mats)-2,'motor'),
                               ('Stationary_motor_bracket',mounting,len(mats)-1,'fixed')]:
        sh=shape.val();p=from_shape(name,sh,mi,group='interface',motion=motion,
          center=(0,center_y,0) if motion=='motor' else (0,0,0),
          role='Author-designed interface study, unqualified fit and strength')
        p.explode=np.array([20.,0,0]) if motion!='fixed' else np.zeros(3)
        parts.append(p);cad[name]=sh
    for i,(y,z) in enumerate(centers):
        sh=(cq.Workplane('YZ',origin=(42,y,z)).circle(4).extrude(29)
            .union(cq.Workplane('YZ',origin=(70,y,z)).polygon(6,13).extrude(5))).val()
        name=f'Carrier_drive_bolt_{i+1:02d}';parts.append(from_shape(name,sh,len(mats)-2,motion='carrier',role='Illustrative fastener; preload/length not released'))
        cad[name]=sh
    a=Assembly('NEO Vortex / 4-to-1 carrier drive study',parts,mats,metadata={
        'gear_pair':{'motor_teeth':n1,'carrier_teeth':n2,'module_mm':module,'center_distance_mm':center_y,'face_width_mm':width},
        'motor_shift_mm':[135,center_y,0],
        'compatibility':'Custom interface concept, NOT a bolt-on or load-qualified recommendation',
        'input':'Carrier flange, not a differential output shaft',
        'limits':'Root fillets approximate; shaft retention, bracket support, enclosure, lubrication, speed rating and loads unvalidated'},cad=cad)
    a.save(ROOT/'examples/motor_drive/geometry');a.export_glb(ROOT/'examples/motor_drive/geometry/drive.glb')
    return a

if __name__=='__main__':
    a=Assembly.load(MOTOR/'study',True) if (MOTOR/'study/scene.json').exists() else prepare_motor();print('MOTOR STUDY',len(a.parts),flush=True)
    d=build_drive(a);print('DRIVE STUDY',len(d.parts),flush=True)
