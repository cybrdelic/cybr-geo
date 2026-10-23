"""Reimport checks, exact-body mass/inertia deltas, and operating-pose STEP export.
No claim that render refinement reruns the motor/contact qualification suite.
"""
from pathlib import Path
import json,time,hashlib
import numpy as np
import cadquery as cq
from OCP.gp import gp_Trsf
from cad_core import build,solid
from frames import pose,BODIES
ROOT=Path(__file__).resolve().parents[1]
D=json.loads((ROOT/'assets/design.json').read_text());r=json.loads((ROOT/'reports/cad-refinement.json').read_text());old=json.loads((ROOT/'assets/mass-properties.json').read_text());new=json.loads(json.dumps(old))
lookup={p['name']:p for p in D['parts']}
def shift(c):return np.eye(3)*np.dot(c,c)-np.outer(c,c)
mass_report=[]
for body,v in old.items():
 m=v['mass_kg'];c=np.array(v['com_m']);P=m*c;I=np.array(v['inertia_tensor_kg_m2'])+m*shift(c)
 changed=[]
 for record in r['parts']:
  if not record['operations'] or record['name'] not in v['components']:continue
  p=lookup[record['name']]
  if p['mass_override_kg'] is not None:raise RuntimeError('Purchased-mass override changed unexpectedly')
  for key,sign in [('original',-1),('refined',1)]:
   q=record[key];mm=q['mass_kg'];cc=np.array(q['com_mm'])*.001
   m+=sign*mm;P+=sign*mm*cc;I+=sign*(np.array(q['inertia_kg_m2'])+mm*shift(cc))
  changed.append(record['name'])
 c=P/m;I=I-m*shift(c);eigs=np.linalg.eigvalsh(I)
 if m<=0 or min(eigs)<=0:raise RuntimeError('Nonphysical revised mass tensor')
 new[body].update(mass_kg=m,com_m=c.tolist(),inertia_tensor_kg_m2=I.tolist(),fullinertia_mujoco_kg_m2=[I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]])
 mass_report.append(dict(body=body,original_mass_kg=v['mass_kg'],refined_mass_kg=m,delta_grams=(m-v['mass_kg'])*1000,positive_inertia_eigenvalues=eigs.tolist(),changed_components=changed))
(ROOT/'assets/refined-mass-properties.json').write_text(json.dumps(new,indent=2))
(ROOT/'reports/mass-property-deltas.json').write_text(json.dumps(dict(body_changes=mass_report,total_mass_delta_g=sum(x['delta_grams'] for x in mass_report),scope='Exact BREP delta correction to archived body inertias; dynamics not rerun'),indent=2))
# STEP checks intentionally separate validity from volume roundtrip precision.
checks=[];start=time.time()
for record in r['parts']:
 if not record['operations']:continue
 shape=cq.importers.importStep(str(ROOT/record['step'])).val()
 delta=abs(shape.Volume()/record['refined']['volume_mm3']-1)
 checks.append(dict(id=record['id'],name=record['name'],valid=shape.isValid(),solid_count=len(shape.Solids()),roundtrip_volume_relative_error=delta,volume_relative_error_below_1e_5=bool(delta<1e-5)))
 if not shape.isValid() or len(shape.Solids())!=1:raise RuntimeError('Invalid exported part')
print('STEP checked',len(checks),'elapsed',time.time()-start,flush=True)
# Include every original modeled part; changed BREP replaces that part only.
R,pos=pose(2.5);assembly=cq.Assembly(name='ARX6_P5_refined_review');inherited_compounds=[];assembly_solids=0
for p in D['parts']:
 path=ROOT/'cad'/f'{p["id"]:03d}.brep'
 shape=cq.Shape.importBrep(str(path)) if path.exists() else build(p['shape'])
 assembly_solids+=len(shape.Solids())
 if len(shape.Solids())!=1:inherited_compounds.append(dict(id=p['id'],name=p['name'],solids=len(shape.Solids()),valid=shape.isValid(),changed=False))
 bi=BODIES.index('fixed_hardware' if p['body']=='fixed' else p['body'])
 rows=np.concatenate([R[bi],(pos[bi]*1000)[:,None]],axis=1)
 tr=gp_Trsf();tr.SetValues(*rows.ravel().tolist())
 color=cq.Color(.56,.58,.61) if p['finish'] in ['paint','metal','steel'] else cq.Color(.06,.065,.07)
 assembly.add(shape,name=f'{p["id"]:03d}_{p["name"]}',loc=cq.Location(tr),color=color)
 if p['id']%60==0:print('ASSEMBLY',p['id'],'elapsed',round(time.time()-start,1),flush=True)
assembly.save(str(ROOT/'cad/ARX6-P5-Refined-operating-pose.step'))
report=dict(modified_parts_checked=len(checks),all_modified_parts_valid=all(x['valid'] and x['solid_count']==1 for x in checks),parts=checks,assembly_part_instances=len(D['parts']),assembly_solid_count=assembly_solids,inherited_multisolid_instances=inherited_compounds,reference_pose_time_s=2.5,archived_source_sha256=hashlib.sha256((ROOT/'assets/design.json').read_bytes()).hexdigest(),geometry_scope='Actual BREP fillets/chamfers and hex recesses; prototype review, not a production release',dynamics_revalidated=False)
(ROOT/'reports/geometry-verification.json').write_text(json.dumps(report,indent=2));print('DONE',round(time.time()-start,1),flush=True)
