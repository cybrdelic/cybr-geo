from pathlib import Path
from copy import deepcopy
import math,json,sys
import numpy as np
import cadquery as cq
from cybrgeo import Assembly,rotation,translation
from cybrgeo.render import Studio,labelled
from cybrgeo.media import video,catalogue
from cybrgeo.offline import render as pathtrace
from cybrgeo.whiteprint import write_whiteprint
R=Path(__file__).resolve().parents[1];M=R/'examples/neo_vortex';D=R/'examples/motor_drive'

def body(a):return Assembly(a.name,[p for p in a.parts if p.group!='cable'],a.materials,a.metadata)

def main():
    a=Assembly.load(M/'study',True);b=body(a);out=M/'renders';out.mkdir(exist_ok=True)
    if '--resume' not in sys.argv:
        with Studio(b,(1600,1100)) as s:
            for name,az,el in [('hero',235,22),('rear',38,25),('mounting_face',180,0)]:
                s.set_camera(az,el,65,(-14,0,0));labelled(s.render(),'NEO VORTEX / SPARK FLEX',
                 'Manufacturer mechanical CAD; photo-informed materials. Long cables hidden.').save(out/f'{name}.png')
            s.pose(explode=1);s.set_camera(245,22,132,(-12,0,0));labelled(s.render(),'MOTOR / EXPLODED MECHANICAL VIEW',
                 'Original vendor parts separated for inspection; offsets are not service instructions.').save(out/'exploded.png')
            s.pose();s.visible(lambda p:p.name not in ['001_SPARK_Flex_shell','028_Outrunner_rotor_can'])
            s.set_camera(235,25,65,(-14,0,0));labelled(s.render(),'MOTOR / DISCLOSED INTERNAL GEOMETRY',
                 'Shell and rotor can removed. Stator windings and magnet detail are not supplied by this CAD.').save(out/'internal.png')
        with Studio(b,(1600,1100),section='all') as s:
            s.set_camera(250,16,65,(-14,0,0));labelled(s.render(),'MOTOR / LONGITUDINAL SECTION',
                 'Geometric section through the manufacturer model; no hidden internals invented.').save(out/'section.png')
        with Studio(a,(1800,900)) as s:
            s.set_camera(256,55,158,(-10,-137,0));labelled(s.render(),'COMPLETE VENDOR ASSEMBLY / CABLES INCLUDED',
                 '75 solids plus one surface body retained; no supplied body discarded.').save(out/'complete_with_wires.png')
        print('MOTOR STILLS',flush=True)
        # Distinct newly rendered video: orbit+rotor rotation, then full explosion and reassembly.
        def inspect(t,u):
            spin=.7*t if u<.45 else 0
            poses={p.name:rotation(spin) for p in b.parts if p.motion=='motor'}
            ex=0 if u<.45 else math.sin(math.pi*(u-.45)/.55)**2
            return dict(poses=poses,explode=ex,camera=(235+28*math.sin(u*math.pi*2),23,65+ex*76,(-14,0,0)))
        video(b,M/'videos/motor_inspection.mp4',inspect,10,24,title='NEO VORTEX / MECHANICAL INSPECTION',
              subtitle='Outrunner rotor and shaft move together; controller and frame remain stationary.')
        catalogue(a,M/'parts',(640,480));print('MOTOR CATALOG',flush=True)
    # Whiteprint: external-envelope CAD selection rather than dense internal wiring.
    env=[s for n,s in a.cad.items() if n in ['001_SPARK_Flex_shell','028_Outrunner_rotor_can','036_Stationary_motor_frame','061_Motor_docking_cover','074_8mm_keyed_output_shaft']]
    write_whiteprint(cq.Compound.makeCompound(env),M/'drawings/motor_envelope','NEO VORTEX + SPARK FLEX / ENVELOPE','CYBR-MOT-001',
         notes=['Source: REV Robotics manufacturer STEP. Exterior selection; leads omitted.',
                'Documented interfaces: 8 mm shaft; 6 x #10-32 holes on 50.8 mm pitch circle.'])
    if '--motor-only' in sys.argv:
        pathtrace(b,out/'hero_pathtraced.png',1400,1000,64,camera=(235,23,66,(-14,0,0)),threads=4)
        exp={p.name:translation(p.explode) for p in b.parts}
        pathtrace(b,out/'exploded_pathtraced.png',1600,1000,48,camera=(250,21,143,(-14,0,0)),threads=4,poses=exp)
        return
    d=Assembly.load(D/'geometry',True)
    write_whiteprint(d.cad['Stationary_motor_bracket'],D/'drawings/motor_bracket','STATIONARY MOTOR BRACKET / STUDY','CYBR-DRV-003',
         notes=['Six clearance holes: 5.2 mm diameter, 50.8 mm pitch circle at 0,45,135,180,225,315 deg.',
                'Motor manufacturer specifies #10-32 threads and 6.3 mm maximum mounting depth.'])
    write_whiteprint(d.cad['Carrier_drive_80T'],D/'drawings/carrier_gear','80-TOOTH CARRIER DRIVE / STUDY','CYBR-DRV-001',
         notes=['Module 1.75; 20-degree pressure angle; 15 mm gear width; root fillet is approximate.',
                'Eight 9.8 mm clearance holes at radius49.8, phase22.5 deg; hub clearance diameter76.'])
    print('DRAWINGS',flush=True)
    # Preserve earlier differential gear constraints by calling the retained original transform.
    sys.path.insert(0,str(R/'examples/differential/src'));import model as old
    refs={p.name:p for p in old.load_parts(old.GEOM/'working_variant_parts.npz')}
    def drive_poses(t):
        carrier=.6*t;diff=.32*math.sin(t*.65);motor=-4*carrier;poses={}
        for p in d.parts:
            if p.name in refs:poses[p.name]=old.transform(refs[p.name],carrier,diff,0)
            elif p.motion=='motor':poses[p.name]=rotation(motor,(1,0,0),(0,87.5,0))
            elif p.motion=='carrier':poses[p.name]=rotation(carrier)
        return poses
    with Studio(d,(1800,1100)) as s:
        s.set_camera(224,25,127,(47,31,-3));labelled(s.render(),'MOTOR-TO-CARRIER DRIVE / 4:1 STUDY',
           '20T motor pinion drives80T carrier gear. Original differential shell is preserved.',
           'Custom interface, not a bolt-on assembly. No torque, thermal or load qualification.').save(D/'drive_hero.png')
    def driving(t,u):return {'poses':drive_poses(t),'camera':(224+8*math.sin(u*math.pi*2),25,127,(47,31,-3))}
    video(d,D/'drive_motion.mp4',driving,8,24,title='MOTOR / CARRIER / DIFFERENTIAL OUTPUTS',
          subtitle='Prescribed kinematics: motor = -4 x carrier; left + right = 2 x carrier.')
    # A fixed-camera gear verification shot; no exterior obscures the v3 kinematic core.
    def visible(p):return p.group not in ['carrier','marking','front_flange','rear_flange']
    video(d,D/'drive_cutaway.mp4',lambda t,u:{'poses':drive_poses(t)},6,24,
          camera=(259,24,120,(35,27,-2)),predicate=visible,title='DRIVE TRAIN / FIXED-CAMERA CORE VIEW',
          subtitle='Housing hidden; carrier-mounted pinions retain the original kinematic constraints.')
    # Measured algebraic constraints; not presented as gear-contact certification.
    errs=[]
    for t in np.linspace(0,10,601):
        c=.6*t;delta=.32*math.sin(t*.65);m=-4*c;l=c+delta;r=c-delta
        errs.append([abs(l+r-2*c),abs(20*m+80*c)])
    val={'samples':601,'max_sum_constraint_rad':float(np.max(np.array(errs)[:,0])),
        'max_input_gear_constraint_tooth_rad':float(np.max(np.array(errs)[:,1])),
        'center_distance_mm':87.5,'nominal_pitch_radii_mm':[17.5,70],
        'shaft_engagement_mm':10.0,'input_face_overlap_mm':10.0,
        'limitations':'Analytic ideal kinematics only; no force/contact/thermal qualification; shaft retention not designed.'}
    (D/'validation.json').write_text(json.dumps(val,indent=2))
    pathtrace(b,out/'hero_pathtraced.png',1400,1000,64,camera=(235,23,66,(-14,0,0)),threads=4)
    exp={p.name:translation(p.explode) for p in b.parts}
    pathtrace(b,out/'exploded_pathtraced.png',1600,1000,48,camera=(250,21,143,(-14,0,0)),threads=4,poses=exp)
    print('ALL NEW RENDERS COMPLETE',flush=True)
if __name__=='__main__':main()
