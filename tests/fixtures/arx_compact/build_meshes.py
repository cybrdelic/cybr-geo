"""Evaluate unchanged P5 CSG with OpenCascade and tessellate for native film capture."""
import json,time,hashlib
from pathlib import Path
import numpy as np
import cadquery as cq
from OCP.gp import gp_Trsf
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
ROOT=Path(__file__).resolve().parents[1]
D=json.loads((ROOT/'assets/design.json').read_text());cache={}
def build(n):
 key=json.dumps(n,separators=(',',':'))
 if key in cache:return cache[key]
 op=n['op'];a=n['args'];ch=n['children']
 if op=='box':s=cq.Solid.makeBox(a[0]*2,a[1]*2,a[2]*2,cq.Vector(-a[0],-a[1],-a[2]))
 elif op=='cylinder':s=cq.Solid.makeCylinder(a[0],a[1]*2,cq.Vector(0,0,-a[1]))
 elif op=='polygon':
  wire=cq.Wire.makePolygon([cq.Vector(x,y,-a[1]) for x,y in a[0]],close=True);s=cq.Solid.extrudeLinear(wire,[],cq.Vector(0,0,2*a[1]))
 elif op=='helix':
  r,wire,pitch,turns=a;path=cq.Wire.makeHelix(pitch,pitch*turns,r)
  profile=cq.Workplane('XZ').center(r,0).circle(wire).val();s=cq.Solid.sweep(profile,[],path,makeSolid=True,isFrenet=True)
 elif op=='transform':
  t=gp_Trsf();t.SetValues(*np.array(a).reshape(4,4)[:3].ravel().tolist());s=cq.Shape.cast(BRepBuilderAPI_Transform(build(ch[0]).wrapped,t,True).Shape())
 elif op=='union':s=build(ch[0]).fuse(build(ch[1]))
 elif op=='cut':s=build(ch[0]).cut(build(ch[1]))
 elif op=='intersection':s=build(ch[0]).intersect(build(ch[1]))
 else:raise ValueError(op)
 cache[key]=s;return s
out=ROOT/'assets/meshes';out.mkdir(exist_ok=True);reports=[];start=time.time()
for p in D['parts']:
 f=out/f"{p['id']:03d}.npz"
 if f.exists():continue
 s=build(p['shape']);v,fcs=s.tessellate(.045,.12);v=np.array([x.toTuple() for x in v],dtype='f4');fcs=np.asarray(fcs,dtype='i4')
 if not len(v) or not len(fcs):raise ValueError('Empty '+p['name'])
 np.savez_compressed(f,vertices=v,faces=fcs)
 reports.append({'id':p['id'],'name':p['name'],'vertices':len(v),'triangles':len(fcs),'valid':s.isValid(),'volume_mm3':s.Volume()})
 if p['id']%10==0:print(p['id'],p['name'],'seconds',round(time.time()-start,1),flush=True)
(ROOT/'reports/native-mesh-build.json').write_text(json.dumps({'source_sha256':hashlib.sha256((ROOT/'assets/design.json').read_bytes()).hexdigest(),'parts':reports,'source_unchanged':True,'linear_deflection_mm':.045,'angular_deflection_rad':.12},indent=2))
print('DONE',time.time()-start,flush=True)
