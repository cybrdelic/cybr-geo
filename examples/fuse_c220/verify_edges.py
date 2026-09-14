"""Adversarial pose checks and independent belt velocity checks on cached geometry."""
from pathlib import Path
import json,numpy as np
from printer import State,motion
from mechanism_lab.core import load_cache

def main():
 a=load_cache('deliverables/cache');by={p.name:p for p in a.parts};results=[]
 for label,vals in [('X overflow',(111,0,0)),('Y overflow',(0,-111,1)),('Z overflow',(0,0,221)),('NaN',(float('nan'),0,0))]:
  rejected=False
  try:State(*vals)
  except ValueError:rejected=True
  results.append({'check':'reject '+label,'passed':rejected})
 for name,axis in [('x_belt_tooth_2',0),('y_belt_tooth_2',1)]:
  p=by[name];h=.001;s0=State(0,0,20);s1=State(h if axis==0 else 0,h if axis==1 else 0,20)
  c=p.bounds.mean(0);T0=motion(s0)(p);T1=motion(s1)(p)
  velocity=((T1[:3,:3]@c+T1[:3,3])-(T0[:3,:3]@c+T0[:3,3]))/h
  results.append({'check':name+' unit straight-span velocity','passed':bool(abs(velocity[axis]-1)<1e-6),'measured':velocity.tolist()})
 nip=[]
 for name,x in [('drive_hob',-.875),('idler_hob',.875)]:
  p=by[name];h=1e-5;point=np.array([x,0,88+104.])
  T0=motion(State(0,0,20,0))(p);T1=motion(State(0,0,20,h))(p)
  velocity=((T1[:3,:3]@point+T1[:3,3])-(T0[:3,:3]@point+T0[:3,3]))/h
  nip.append(velocity.tolist())
 results.append({'check':'both hob contact surfaces feed filament downward for positive E','passed':all(abs(v[2]+1)<1e-5 for v in nip),'velocities_mm_per_mm_E':nip})
 p=by['spool_hub'];point=np.array([106,50,636.]);h=1e-5
 T0=motion(State(0,0,20,0))(p);T1=motion(State(0,0,20,h))(p)
 velocity=((T1[:3,:3]@point+T1[:3,3])-(T0[:3,:3]@point+T0[:3,3]))/h
 results.append({'check':'spool releases filament toward its front exit for positive E','passed':bool(abs(velocity[1]+1)<1e-5),'velocity_mm_per_mm_E':velocity.tolist()})
 errors=[];p=by['z_thread_-175']
 for z in np.random.default_rng(119).uniform(0,220,1000):
  T=motion(State(0,0,z))(p);angle=np.arctan2(T[1,0],T[0,0])
  phase=z/2+4*angle/(2*np.pi);errors.append(abs(phase-round(phase)))
 results.append({'check':'four-start right-handed screw phase follows rising nut','passed':bool(max(errors)<1e-10),'samples':1000,'maximum_phase_error_turns':float(max(errors))})
 report={'all_passed':all(r['passed'] for r in results),'checks':results}
 Path('deliverables/FUSE_C220_edge_validation.json').write_text(json.dumps(report,indent=2))
 print(json.dumps(report,indent=2))
 if not report['all_passed']:raise SystemExit(1)

if __name__=='__main__':main()
