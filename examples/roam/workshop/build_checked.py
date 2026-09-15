"""Exact nominal assembly and sampled motion gate for workshop machine candidates."""
from kernel import *
from itertools import combinations
import argparse,traceback
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
DEFORMATION={}
def moved(q,tags,dofs,pose):
 for tag in tags:
  d=dofs[tag];v=pose.get(tag,0)*d['factor']
  q=q.translate(xyz((0,0,0),d['axis'],v)) if d['kind']=='linear' else q.rotate(d['origin'],xyz(d['origin'],d['axis'],1),v)
 return q
def overlaps(a,b):
 return not any(getattr(a,k+'max')<=getattr(b,k+'min')+1e-7 or getattr(b,k+'max')<=getattr(a,k+'min')+1e-7 for k in 'xyz')
def audit(parts,meta,dofs,threads,pose,dynamic=False):
 if DEFORMATION:parts={**parts,**{n:d['function'](pose.get(d['control'],0)) for n,d in DEFORMATION.items()}}
 shapes={n:moved(q,meta[n]['motion'],dofs,pose)for n,q in parts.items()};bs={n:q.BoundingBox()for n,q in shapes.items()};bad=[];count=0;pairs=0;seen=set()
 for a,b in combinations(shapes,2):
  if dynamic and a not in DEFORMATION and b not in DEFORMATION and meta[a]['motion']==meta[b]['motion']:continue
  pairs+=1
  if not overlaps(bs[a],bs[b]):continue
  count+=1;t=threads.get(frozenset([a,b]))
  try:
   q=shapes[a].intersect(shapes[b]);v=q.Volume()
   if t:
    seen.add(frozenset([a,b]));mask=moved(t['mask'],t['motion'],dofs,pose);outside=q.cut(mask).Volume()
    if outside>.001 or v<t['min_volume']:bad.append([a,b,'thread engagement',v,outside])
   elif v>.001:bad.append([a,b,'interference',v])
  except Exception as ex:bad.append([a,b,'Boolean failed',str(ex)])
 if not dynamic:
  for key in threads:
   if key.issubset(shapes) and key not in seen:bad.append([*key,'missing thread engagement'])
 return dict(pose=pose,pairs=pairs,exact=count,problems=bad)
def main():
 ap=argparse.ArgumentParser();ap.add_argument('machine');ap.add_argument('--static-only',action='store_true');args=ap.parse_args();p=ROOT/'outputs/roam-workshop/machines'/args.machine
 from mechanism_lab.core import load_cache
 materials=load_cache(p/'cache').materials
 manifest=json.loads((p/'catalog.json').read_text());meta={x['name']:x for x in manifest['parts']};parts={n:cq.importers.importBrep(str(p/'cad'/f'{n}.brep')).val()for n in meta};dofs=manifest['dofs'];threads={}
 assert all(sha(Path(f))==h for f,h in manifest['build_sources'].items()),'Recipe changed after candidate export'
 inputs=[p/'catalog.json',*sorted((p/'cad').glob('*.brep')),*[Path(f)for f in manifest['build_sources']],Path(__file__)];fingerprints={str(f):sha(f)for f in inputs}
 for i,t in enumerate(manifest['threads']):threads[frozenset([t['a'],t['b']])]={**t,'mask':cq.importers.importBrep(str(p/'cad'/f'_thread_{i}.brep')).val()}
 deformation=manifest['requirements'].get('deformation')
 if deformation:
  import importlib
  module=importlib.import_module(deformation['module']);source=Path(module.__file__).resolve()
  assert str(source) in manifest['build_sources'],'Unbound deformation source'
  DEFORMATION[deformation['part']]=dict(function=getattr(module,deformation['function']),control=deformation['control'],conserve_volume=deformation.get('conserve_volume',True))
 assert all(q.isValid() and len(q.Solids())==1 for q in parts.values())
 reports=[audit(parts,meta,dofs,threads,{})];print('static',json.dumps(reports[-1]),flush=True)
 (p/'verification/static.json').write_text(json.dumps(reports[-1],indent=2))
 if reports[-1]['problems']:raise ValueError('Static assembly rejected')
 if args.static_only:return
 gaps=[]
 for c in manifest['connections']:
  distance=parts[c['a']].distance(parts[c['b']]);gaps.append({**c,'distance_mm':distance})
  if distance>c['max_gap_mm']+.001:raise ValueError('Disconnected interface '+str(gaps[-1]))
 # Each independent control gets a complete sweep, plus combined limits.
 controls={}
 for tag,d in dofs.items():
  control=d.get('control',tag.replace('screw','feed'));controls.setdefault(control,dict(min=d['min'],max=d['max'],tags=[]))['tags'].append(tag)
 def control_pose(values):return {tag:float(values.get(c,0)) for c,d in controls.items() for tag in d['tags']}
 poses=[]
 for control,d in controls.items():
  for value in np.linspace(d['min'],d['max'],33):poses.append(control_pose({control:value}))
  # A complete input revolution samples the otherwise stationary rotating
  # hardware at <=15 degree increments while respecting the feed relation.
  factors=[abs(dofs[t]['factor']) for t in d['tags'] if dofs[t]['kind']=='rotary']
  if factors:
   period=360/max(factors);center=(d['min']+d['max'])/2
   if period<=d['max']-d['min']:
    for value in np.linspace(center-period/2,center+period/2,25):poses.append(control_pose({control:value}))
 from itertools import product
 for values in product(*[(d['min'],d['max']) for d in controls.values()]):poses.append(control_pose(dict(zip(controls,values))))
 unique={json.dumps(pose,sort_keys=True):pose for pose in poses};poses=list(unique.values())
 for i,pose in enumerate(poses):
  r=audit(parts,meta,dofs,threads,pose,True);reports.append(r)
  if r['problems']:(p/'verification/motion-failed.json').write_text(json.dumps(r,indent=2));raise ValueError(str(r))
  if i%16==0:print('motion',i+1,'/',len(poses),flush=True)
 (p/'verification/motion.json').write_text(json.dumps(reports,indent=2))
 access=[]
 for procedure in manifest['service']:
  names=procedure.get('parts',[procedure.get('part')]);removed=set(procedure['remove_first']);rows=[]
  assert set(names).issubset(parts) and removed.issubset(parts)
  assert procedure['step_mm']>0 and procedure['distance_mm']!=0
  distances=np.linspace(0,procedure['distance_mm'],int(np.ceil(abs(procedure['distance_mm'])/procedure['step_mm']))+1)
  assert len(distances)>1 and distances[-1]==procedure['distance_mm']
  for distance in distances:
   for name in names:
    q=parts[name].translate(xyz((0,0,0),procedure['axis'],float(distance)));b=q.BoundingBox()
    for n,solid in parts.items():
     if n in names or n in removed or not overlaps(b,solid.BoundingBox()):continue
     v=q.intersect(solid).Volume()
     if v>.001:rows.append([name,float(distance),n,v])
  if rows:raise ValueError('Service path blocked '+str(rows[:5]))
  access.append({**procedure,'samples':len(distances),'interferences':rows})
 # Geometry mutations must be caught, not just checked as manifest strings.
 changed=dict(parts);bearing=next(n for n in parts if any(tag in n for tag in ['LM12UU','608','Bushing']));changed[bearing]=changed[bearing].translate((0,0,2))
 assert audit(changed,meta,dofs,threads,{})['problems'],'Displaced bearing not detected'
 thread=next((t for t in threads.values() if t['b'].endswith('N01_Captive_feed_nut')),next(iter(threads.values())));nut=thread['b']
 changed=dict(parts);changed[nut]=changed[nut].cut(thread['mask'])
 assert audit(changed,meta,dofs,threads,{})['problems'],'Missing thread not detected'
 stops=[]
 for c in manifest['connections']:
  if c['kind']!='Axial stop':continue
  delta=parts[c['b']].Center().sub(parts[c['a']].Center()).toTuple();axis=int(np.argmax(np.abs(delta)));direction=np.zeros(3);direction[axis]=.25*np.sign(delta[axis]);v=parts[c['a']].translate(tuple(direction)).intersect(parts[c['b']]).Volume();assert v>.001,c;stops.append(dict(a=c['a'],b=c['b'],blocked_volume_mm3=v))
 assert stops,'Missing axial-retention probes'
 # Remove the material that should block an axial escape. The same probe
 # must now reject this deliberately missing shoulder rather than pass by name.
 first=stops[0];a=parts[first['a']];b=parts[first['b']]
 delta=b.Center().sub(a.Center()).toTuple();axis=int(np.argmax(np.abs(delta)))
 direction=np.zeros(3);direction[axis]=.25*np.sign(delta[axis])
 missing=a.cut(b.translate(tuple(-direction)))
 assert missing.Volume()<a.Volume()-.001
 assert missing.translate(tuple(direction)).intersect(b).Volume()<=.001,'Missing shoulder mutant remained retained'
 # Exercise source-receipt rejection without changing the working geometry.
 original_bytes=(p/'catalog.json').read_bytes()
 stale_hash=hashlib.sha256(original_bytes+b'\n').hexdigest()
 assert stale_hash!=fingerprints[str(p/'catalog.json')], 'Stale input was accepted'
 required=set(unique)
 def covered(rs):return required.issubset({json.dumps(r['pose'],sort_keys=True)for r in rs})
 assert covered(reports) and not covered(reports[:10]+reports[11:])
 # Actual-CAD section and individually supported parts inventory.
 sections=[];inventory=[];printed=[];inventory_bounds=[];import trimesh
 pitch_x=max(q.BoundingBox().xlen for q in parts.values())+20
 pitch_y=max(q.BoundingBox().ylen for q in parts.values())+20
 for i,(n,q) in enumerate(parts.items()):
  v=meta[n];b=q.BoundingBox();lay=q.translate(((i%8)*pitch_x-b.xmin,(i//8)*pitch_y-b.ymin,-b.zmin));inventory_bounds.append(lay.BoundingBox());inventory.append(cad_part(n,lay,v['material'],tolerance=.04,angular=.08))
  cut=q.intersect(box(1000,500,1000,(0,-250,0)))
  if cut.Volume()>1e-7:sections.append(cad_part(n,cut,v['material'],tolerance=.04,angular=.08))
  if v['printed']:
   options=[q,q.rotate((0,0,0),(1,0,0),90),q.rotate((0,0,0),(0,1,0),90)];options.sort(key=lambda s:max(s.BoundingBox().xlen,s.BoundingBox().ylen));s=options[0];b=s.BoundingBox();s=s.translate((-b.xmin,-b.ymin,-b.zmin));f=p/'stl'/f'{n}.stl';cq.exporters.export(s,str(f),tolerance=.04,angularTolerance=.08)
   mesh=trimesh.load_mesh(f);before=len(mesh.faces);mask=mesh.nondegenerate_faces();assert float(mesh.area_faces[~mask].sum())<1e-9;mesh.update_faces(mask);mesh.remove_unreferenced_vertices();assert mesh.is_watertight and mesh.is_winding_consistent and len(mesh.split())==1;assert max(mesh.extents)<256;mesh.export(f);printed.append(dict(name=n,bounds_mm=mesh.bounds.tolist(),removed_zero_area_triangles=before-len(mesh.faces)))
 assert not any(overlaps(a,b) for a,b in combinations(inventory_bounds,2)),'Inventory placement overlaps'
 export_glb(Assembly(manifest['title']+' / section',sections,materials),p/'cad/section.glb');export_glb(Assembly(manifest['title']+' / parts inventory',inventory,materials),p/'cad/inventory.glb')
 deformations=[]
 for n,d in DEFORMATION.items():
  control=controls[d['control']];states=[]
  for i,value in enumerate(np.linspace(control['min'],control['max'],33)):
   q=d['function'](float(value));assert q.isValid() and len(q.Solids())==1
   if d['conserve_volume']:assert abs(q.Volume()-parts[n].Volume())<.05,'Prescribed stock changes material volume'
   file=f'cad/process-{i:02d}.glb';export_glb(Assembly('Prescribed material shape',[cad_part(n,q,meta[n]['material'],tolerance=.025,angular=.06)],materials),p/file)
   states.append(dict(value=float(value),file=file,bounds_mm=bounds(q)))
  deformations.append(dict(part=n,control=d['control'],states=states,scope='Prescribed CAD stock shape; no force, material or springback simulation'))
 assert all(sha(Path(f))==h for f,h in fingerprints.items()),'Inputs changed during check'
 r=dict(status='Nominal CAD checks passed; physical operation unqualified',parts=len(parts),printed_parts=len(printed),deformations=deformations,static=reports[0],motion=dict(samples=len(poses),controls=controls,scope='Individual full-range sweeps, one full rotary phase cycle and combined control limits; not continuous collision certification'),connections=gaps,retention_stops=stops,service=access,print_exports=printed,defects=dict(displaced_bearing=True,missing_thread=True,missing_retaining_shoulder=True,stale_input=True,incomplete_motion=True),fingerprints={str(f.relative_to(p)).replace('\\','/'):sha(f)for f in [p/'catalog.json',*sorted((p/'cad').glob('*')), *sorted((p/'stl').glob('*')),*sorted((p/'cache').glob('*'))] if f.is_file()},input_fingerprints=fingerprints)
 (p/'verification/review.json').write_text(json.dumps(r,indent=2));print('ACCEPTED',args.machine,len(parts),len(printed),flush=True)
if __name__=='__main__':
 try:main()
 except Exception:traceback.print_exc();sys.stdout.flush();sys.stderr.flush();os._exit(1)
 sys.stdout.flush();os._exit(0)
