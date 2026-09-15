"""Reproducible first-order load model. Assumptions are explicit; no release rating."""
import os,sys,json,traceback,hashlib
import numpy as np
from model import OUT,DEFAULT,POSES,LENGTHS,fingerprint
from verify import load
DENSITY=[2700,2700,7850,1400,1150,700,0,1200,1050]

def calculate():
 m,meta=load();results={};parts=[]
 for name,q in POSES.items():
  fs,_=m.poses(q);rows=[]
  for p in m.parts:
   n=p['name'];shape=m.world_shape(p,q,fs);mass=shape.Volume()*DENSITY[p['mat']]/1e9;basis='nominal CAD volume x assumed material density'
   for condition,value,source in [
    ('monitor_envelope' in n,6,'assumed device mass'),('guide_block' in n,.3,'HIWIN catalog nominal mass'),
    ('rail_envelope' in n,shape.BoundingBox().zlen*.00221,'HIWIN 2.21 kg/m'),
    ('gas_body_envelope' in n,1.1,'unselected package mass assumption'),('gas_rod_envelope' in n,.2,'unselected rod mass assumption'),
    ('wheel_envelope' in n,.45,'unselected caster mass assumption'),('fork_envelope' in n,.45,'unselected caster mass assumption'),
    ('rail_clamp_envelope' in n,.26,'Zimmer complete unit catalog mass'),('brake_lever_envelope' in n,0,'included in complete clamp mass')]:
    if condition:mass=value;basis=source
   rows.append({'name':n,'kg':mass,'cg_mm':list(shape.Center().toTuple()),'group':p['group'],'basis':basis})
  M=sum(x['kg'] for x in rows);cg=sum(np.array(x['cg_mm'])*x['kg'] for x in rows)/M
  # Caster trail is not selected. Erode the wheel-centre rectangle by 40 mm.
  bounds={'left':-240,'right':240,'rear':-210,'front':230+q['base'][0]}
  restore={'left':M*9.81*(cg[0]+240)/1000,'right':M*9.81*(240-cg[0])/1000,'rear':M*9.81*(cg[1]+210)/1000,'front':M*9.81*(bounds['front']-cg[1])/1000}
  pts=[(fs['tray_roll']@np.array([x,-61,z,1]))[:3] for x in (-360,360) for z in (-5,325)]
  hands={'left':max(0,max(-240-p[0] for p in pts))*.1,'right':max(0,max(p[0]-240 for p in pts))*.1,'rear':max(0,max(-210-p[1] for p in pts))*.1,'front':max(0,max(p[1]-bounds['front'] for p in pts))*.1}
  remaining={k:restore[k]-hands[k]-36 for k in restore}
  lifts={};joint_loads={}
  for side in ['left','right','tray']:
   moving=[x for x in rows if x['group'].startswith(side+'_')]
   weight=sum(x['kg'] for x in moving)*9.81;root=(fs[side+'_carriage']@np.array([0,33,0,1]))[:3]
   moment=sum(np.cross((np.array(x['cg_mm'])-root)/1000,[0,0,-x['kg']*9.81]) for x in moving)
   lifts[side]={'gravity_N':weight,'gravity_bending_Nm':float(np.linalg.norm(moment[:2])),'ideal_block_reaction_couple_N':float(np.linalg.norm(moment[:2]))/.18,'rail_clamp_axial_ratio_gravity_only':1200/weight,'note':'Moment capacity, installation orientation and dynamic loads are separate; manual rail clamp is not a damper.'}
   moments=[moment]+([moment+np.cross((p-root)/1000,[0,0,-100]) for p in pts] if side=='tray' else [])
   local=[fs[side+'_carriage'][:3,:3].T@v for v in moments]
   lifts[side]['root_mount']={'vertical_bolt_pitch_mm':46,'bolts_in_tension_row':2,'peak_bending_about_horizontal_face_axis_Nm':max(abs(v[0]) for v in local),'ideal_additional_tension_each_upper_M6_N':max(abs(v[0]) for v in local)/(.046*2),'assumed_load_factor_2_tension_N':2*max(abs(v[0]) for v in local)/(.046*2),'limits':'Bolt-group estimate excludes prying, preload, local plate flex and thread stripping. Six mm tapped carriage plate engagement must be qualified; this is not a bolt rating.'}
   tip=[x for x in rows if x['group'] in [side+'_pitch',side+'_roll']]
   origin=fs[side+'_pitch'][:3,3];axis=fs[side+'_pitch'][:3,0]
   torque=abs(sum(np.dot(np.cross((np.array(x['cg_mm'])-origin)/1000,[0,0,-x['kg']*9.81]),axis) for x in tip))
   if side=='tray':torque+=max(abs(np.dot(np.cross((p-origin)/1000,[0,0,-100]),axis)) for p in pts)
   joint_loads[side]={'pitch_gravity_plus_tray_100N_hand_Nm':torque}
   if side=='tray':joint_loads[side]['required_preload_each_of_two_R75_clamps_N']={str(mu):torque/(2*.075*mu) for mu in (.1,.15,.25)}
  results[name]={'mass_kg':M,'cg_mm':cg.tolist(),'conservative_support_rectangle_mm':bounds,'restoring_Nm':restore,'residual_after_100N_at_worst_tray_corner_and_30N_at_1200mm_Nm':remaining,'static_scenario_positive':bool(min(remaining.values())>0),'lifts':lifts,'pitch_loads':joint_loads}
  if name=='Working':parts=rows
 beam=[]
 for side in ['left','tray']:
  b=.060;h=.040 if side=='tray' else .030;t=.002;I=(b*h**3-(b-2*t)*(h-2*t)**3)/12;L=sum(LENGTHS[side])/1000;F=180 if side=='tray' else 85
  beam.append({'support':side,'assumed_end_load_N':F,'section_I_m4':I,'ideal_uninterrupted_tube_deflection_mm':F*L**3/(3*69e9*I)*1000,'ideal_tube_stress_MPa':F*L*h/(2*I)/1e6,'excludes':'End tongues, lap joints, fastener slip, mast, guide compliance and torsion. Not whole-arm deflection.'})
 r={'source_sha256':fingerprint(),'STEP_sha256':hashlib.file_digest((OUT/'cad/workstation.step').open('rb'),'sha256').hexdigest(),'fabrication_release':False,'poses':results,'ideal_beams':beam,'parts':parts,'assumptions':{'gravity_m_s2':9.81,'device_kg_each':6,'keyboard_mouse_and_computer_mass_included':False,'hand_load_N':100,'lateral_push_N':30,'push_height_mm':1200,'caster_trail_allowance_mm':40,'aluminium_E_GPa':69,'dynamic_braking_impact_fatigue_tolerance_thermal_analysis':False},'limits':'Nominal rigid-body mass and static moments. Missing payloads, unselected caster/gas data, friction/contact behavior and fabrication detail prevent a load rating.'}
 (OUT/'verification/engineering.json').write_text(json.dumps(r,indent=2));print(json.dumps({'working_mass_kg':results['Working']['mass_kg'],'static_scenarios':{k:v['static_scenario_positive'] for k,v in results.items()}}),flush=True)
 return r
if __name__=='__main__':
 try:calculate();os._exit(0)
 except BaseException:traceback.print_exc();sys.stderr.flush();os._exit(1)
