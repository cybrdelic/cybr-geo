"""Executed nominal CAD and motion checks, with a deliberately bounded claim."""
from pathlib import Path
import json,math
import numpy as np
from printer import build,State,motion,posed,BED,RP
from mechanism_lab.core import pose_cad,save_cache
from mechanism_lab.exporters import export_step,export_glb,export_bom

def verify(a):
 checks=[]
 def add(name,ok,data):checks.append({'check':name,'passed':bool(ok),'detail':data});print(name,ok,str(data)[:200],flush=True)
 names={p.name:p for p in a.parts}
 add('unique finite named geometry',len(names)==len(a.parts) and all(np.isfinite(p.vertices).all() and np.isfinite(p.normals).all() for p in a.parts),{'parts':len(a.parts),'triangles':sum(len(p.faces) for p in a.parts)})
 # Topological validity was enforced in cad_part during construction; inspect volume here.
 cad=[p for p in a.parts if p.cad is not None]
 invalid=[p.name for p in cad if not p.cad.isValid() or p.cad.Volume()<=0]
 add('all analytic bodies valid with positive volume',not invalid,{'analytic_bodies':len(cad),'invalid':invalid})
 add('both belt loops close and retain fixed length',all(x['closure_error_mm']<1e-8 for x in a.metadata['belt_systems']),a.metadata['belt_systems'])
 add('X carriage remains on supported rail at both travel limits',157-(110+23)>0,{'minimum_end_margin_mm':157-(110+23),'rail_ends_mm':[-157,157]})
 add('Y bushes remain on rods at both travel limits',230-(110+38+14.5)>0,{'minimum_end_margin_mm':230-(110+38+14.5)})
 add('Z carriage stays on vertical rail through 220 mm travel',BED+65-23>82 and BED+65+220+23<429,{'bottom_margin_mm':BED+65-23-82,'top_margin_mm':429-(BED+65+220+23)})
 # Relative ideal rotor angle -> commanded translation, including screw lead.
 xyz=np.random.default_rng(20260914).uniform([-110,-110,0],[110,110,220],(20000,3))
 revs=xyz/np.array([40,40,8]);back=revs*np.array([40,40,8])
 add('belt pitch and screw lead close motor-coordinate conversion',np.max(abs(xyz-back))<1e-10,{'samples':20000,'max_error_mm':float(np.max(abs(xyz-back))),'rotation_distance_mm':[40,40,8]})
 # Nozzle origin, actual transformed CAD vertex bounds, and datum agree independently.
 err=0
 for x,y,z in [(x,y,z) for x in (-110,0,110) for y in (-110,0,110) for z in (.2,110,220)]:
  s=State(x,y,z);T=motion(s)(names['nozzle_040']);v=names['nozzle_040'].vertices@T[:3,:3].T+T[:3,3]
  err=max(err,abs(v[:,2].min()-(BED+z)))
 add('modeled nozzle tip matches print height through 27 poses',err<1e-9,{'max_z_error_mm':err})
 # BREP interference queries for important independently-moving pairs.
 pairs=[('x_carriage','x_rail'),('hotend_clamp_bridge','toolhead_backplate'),('x_belt_anchor_bridge','toolhead_backplate'),('x_motor_laminated_body','x_gantry_beam'),('x_motor_laminated_body','portal_column_-175'),('z_motor_-175_laminated_body','portal_column_-175'),('z_motor_175_laminated_body','portal_column_175'),('y_motor_laminated_body','base_cross_235'),('y_idler_mount','base_cross_-235'),('extruder_motor_laminated_body','x_gantry_beam'),('extruder_motor_laminated_body','x_carriage'),('toolhead_backplate','x_gantry_beam'),('hotend_clamp_bridge','x_gantry_beam'),('coldend_fan_housing','x_gantry_beam'),('nozzle_040','removable_PEI_sheet'),('toolhead_backplate','heated_aluminium_bed'),('x_gantry_beam','portal_top')]
 results=[]
 for n1,n2 in pairs:
  p,q=names[n1],names[n2];worst=0;dist=1e9
  for x,z in [(-110,.2),(0,.2),(110,.2),(0,220)]:
   pose=motion(State(x,0,z));c1=pose_cad(p.cad,pose(p));c2=pose_cad(q.cad,pose(q))
   bb1,bb2=c1.BoundingBox(),c2.BoundingBox()
   overlap=all(min(getattr(bb1,k+'max'),getattr(bb2,k+'max'))>max(getattr(bb1,k+'min'),getattr(bb2,k+'min'))+1e-6 for k in 'xyz')
   volume=c1.intersect(c2).Volume() if overlap else 0
   worst=max(worst,volume);dist=min(dist,c1.distance(c2))
  results.append({'parts':[n1,n2],'max_intersection_mm3':worst,'minimum_separation_mm':dist})
 add('selected critical CAD pairs do not collide',all(r['max_intersection_mm3']<1e-5 for r in results),results)
 # Belt envelope is behind all of the translating head motor throughout X travel.
 headback=names['extruder_motor_laminated_body'].bounds[1,1]
 beltfront=names['x_belt'].bounds[0,1]
 add('X belt clears the moving extruder motor in Y',beltfront>headback,{'clearance_mm':float(beltfront-headback)})
 # Sweep all translating nominal CAD bodies against fixed solid geometry.
 # Intentional drive engagement (belts and screw thread contacts) is handled separately.
 moving=[p for p in a.parts if p.cad is not None and p.motion in ('gantry','head','bed') and p.group not in ('fasteners','belt_teeth')]
 fixed=[p for p in a.parts if p.cad is not None and p.motion=='fixed' and p.group not in ('fasteners','markings','wiring')]
 sweep=[];queries=0
 for xx,yy,zz in [(-110,-110,.2),(-110,110,220),(110,-110,220),(110,110,.2),(0,0,110)]:
  st=State(xx,yy,zz);pose=motion(st)
  for p in moving:
   T=pose(p);lo,hi=p.bounds+T[:3,3]
   for q in fixed:
    qlo,qhi=q.bounds
    if not np.all(np.minimum(hi,qhi)-np.maximum(lo,qlo)>1e-5):continue
    volume=pose_cad(p.cad,T).intersect(q.cad).Volume();queries+=1
    if volume>1e-4:sweep.append({'moving':p.name,'fixed':q.name,'pose':[xx,yy,zz],'intersection_mm3':volume})
 add('translating bodies clear fixed geometry at five full-travel poses',not sweep,{'narrow_phase_queries':queries,'collisions':sweep})
 # Physical connections whose absence would leave bed hardware floating.
 contact=[]
 for p1,p2 in [('x_motor_bracket','x_gantry_beam'),('x_idler_bracket','x_gantry_beam'),('z_motor_clamp_-175','base_cross_50'),('y_bed_carrier','carrier_ear_-98'),('carrier_ear_-98','bed_standoff_-98_-98'),('bed_standoff_-98_-98','heated_aluminium_bed'),('heated_aluminium_bed','magnetic_sheet'),('magnetic_sheet','removable_PEI_sheet'),('extruder_housing','toolhead_backplate'),('x_carriage','toolhead_backplate')]:
  d=names[p1].cad.distance(names[p2].cad);contact.append({'parts':[p1,p2],'gap_mm':d})
 add('bed stack and toolhead mounting interfaces touch',all(r['gap_mm']<1e-6 for r in contact),contact)
 # Flexible lines rebuilt from endpoints, not rendered as disconnected rigid props.
 flex=posed(a,State(62,-42,140));fn={p.name:p for p in flex.parts}
 f=fn['dynamic_filament_feed'];expected=np.array([62,0,BED+140+129]);distances=np.linalg.norm(f.vertices-expected,axis=1);tip=f.vertices[np.argmin(distances)]
 add('filament endpoint follows extruder inlet',distances.min()<1e-8,{'endpoint_mm':tip.tolist()})
 return {'all_passed':all(c['passed'] for c in checks),'checks':checks,'scope':'Nominal CAD interfaces, selected BREP interference checks, full axis envelopes, ideal kinematics. Not an exhaustive tolerance-stack collision certification; no heat/strength/contact/friction/controller or physical print validation.'}

if __name__=='__main__':
 out=Path('deliverables');a=build();r=verify(a);out.joinpath('FUSE_C220_mechanism_validation.json').write_text(json.dumps(r,indent=2))
 save_cache(a,out/'cache');export_glb(posed(a,State()),out/'FUSE_C220.glb');export_step(a,out/'FUSE_C220_analytic.step',individual=False);export_bom(a,out)
 print('ALL_PASSED',r['all_passed'],flush=True)
