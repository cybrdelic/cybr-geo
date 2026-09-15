"""Shared interfaces, actual solids and declared joints for workshop candidates.

Internal coordinates are mm, Z up. CAD is original unless a source is named.
Nominal purchased thread envelopes are explicitly identified, never silently
excluded from collision checks. No physical performance is inferred here.
"""
from pathlib import Path
import os,sys,math,json,hashlib
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'work/cybr-geo/src'))
import cadquery as cq
import numpy as np
from mechanism_lab.core import Assembly,Material,View,cad_part,save_cache
from mechanism_lab.exporters import export_glb,export_step
AXES={'X':(1,0,0),'Y':(0,1,0),'Z':(0,0,1)}
MAT=[Material('Transparent PETG',(.82,.89,.86),0,.2,ior=1.57,opacity=.55),Material('Cut aluminum',(.54,.57,.59),.9,.27),Material('Ground steel',(.52,.55,.57),.95,.19),Material('Blackened steel',(.08,.1,.11),.8,.3),Material('Brass',(.61,.43,.18),.8,.28),Material('Reference workpiece',(.36,.39,.4),.8,.38)]
def xyz(p,axis,d):return tuple(np.array(p)+np.array(AXES[axis])*d)
def box(w,d,h,c,r=0):
 q=cq.Workplane('XY').box(w,d,h)
 if r:q=q.edges('|Z').fillet(r)
 return q.val().translate(c)
def cyl(r,l,p,axis='Z'):return cq.Solid.makeCylinder(r,l,cq.Vector(*p),cq.Vector(*AXES[axis]))
def ring(ro,ri,l,p,axis='Z'):return cyl(ro,l,p,axis).cut(cyl(ri,l+2,xyz(p,axis,-1),axis))
def prism(n,d,l,p,axis='Z'):
 q=cq.Workplane('XY').polygon(n,d).extrude(l).val()
 if axis=='X':q=q.rotate((0,0,0),(0,1,0),90)
 if axis=='Y':q=q.rotate((0,0,0),(1,0,0),-90)
 return q.translate(p)
def handwheel(ro,ri,l,p,axis='Z'):
 q=ring(ro,ri,l,(0,0,0));cuts=[]
 for i in range(24):
  a=i*math.tau/24;cuts.append(cyl(1.2,l+2,(ro*math.cos(a),ro*math.sin(a),-1)))
 q=q.cut(cq.Compound.makeCompound(cuts))
 if axis=='X':q=q.rotate((0,0,0),(0,1,0),90)
 if axis=='Y':q=q.rotate((0,0,0),(1,0,0),-90)
 return q.translate(p)
def bounds(s):
 b=s.BoundingBox();return [[b.xmin,b.ymin,b.zmin],[b.xmax,b.ymax,b.zmax]]
class Machine:
 def __init__(self,id,title):
  self.id=id;self.title=title;self.parts={};self.connections=[];self.threads=[];self.dofs={};self.requirements={};self.service=[];self.notes=[];self.sources=[]
 def add(self,name,shape,mat=0,motion=(),role='',make='Original part',parent=None,fit=.15):
  if name in self.parts:raise ValueError('Duplicate '+name)
  if not shape.isValid() or len(shape.Solids())!=1:raise ValueError('Invalid / disconnected '+name)
  self.parts[name]=dict(shape=shape,material=mat,motion=list(motion),role=role,make=make)
  if parent:self.connect(name,parent,'Fixed interface',fit)
  return name
 def connect(self,a,b,kind,gap=.15,**kw):self.connections.append(dict(a=a,b=b,kind=kind,max_gap_mm=gap,**kw))
 def cut(self,name,tool):
  q=self.parts[name]['shape'].cut(tool)
  if not q.isValid() or len(q.Solids())!=1:raise ValueError('Cut disconnected '+name)
  self.parts[name]['shape']=q
 def fuse(self,name,shape):
  q=self.parts[name]['shape'].fuse(shape).clean()
  if not q.isValid() or len(q.Solids())!=1:raise ValueError('Fuse disconnected '+name)
  self.parts[name]['shape']=q
 def dof(self,name,kind,axis,minimum,maximum,origin=(0,0,0),factor=1):
  self.dofs[name]=dict(kind=kind,axis=axis,min=minimum,max=maximum,origin=list(origin),factor=factor)
 def transform(self,name,pose):
  q=self.parts[name]['shape']
  for tag in self.parts[name]['motion']:
   d=self.dofs[tag];v=pose.get(tag,0)*d['factor']
   if d['kind']=='linear':q=q.translate(xyz((0,0,0),d['axis'],v))
   else:q=q.rotate(d['origin'],xyz(d['origin'],d['axis'],1),v)
  return q
 def bolt(self,name,a,b,start,grip,axis='Z',diam=4):
  """Through-bolt, two washers and nut; all driven by one coaxial interface.

  Start is the first member's outer face; positive axis traverses the grip.
  Bolt and nut thread volumes are nominal envelopes bounded by the nut seat.
  """
  motion=self.parts[a]['motion'];r=diam/2;wash=.8;nut_h=diam*.8
  assert motion==self.parts[b]['motion'],(name,'Cannot bolt two independently moving groups')
  drill=cyl(r+.2,grip+2,xyz(start,axis,-1),axis)
  self.cut(a,drill)
  if b!=a:self.cut(b,drill)
  shaft=cyl(r,grip+wash*2+nut_h,xyz(start,axis,-wash),axis)
  head=cyl(r*1.75,diam,xyz(start,axis,-wash-diam),axis)
  socket=prism(6,diam*.85,diam*.65,xyz(start,axis,-wash-diam-.01),axis)
  self.add(name,shaft.fuse(head).cut(socket),3,motion,'Nominal socket bolt; threads represented by bounded engagement envelope','Purchase: select standard length',a,fit=.21)
  for k,z in [('head',-wash),('nut',grip)]:
   n=name+'_'+k+'_washer';self.add(n,ring(r*2,r+.2,wash,xyz(start,axis,z),axis),2,motion,'Separate load-spreading washer','Purchase: nominal washer',a if k=='head' else b)
  n=name+'_nut';np0=xyz(start,axis,grip+wash)
  ns=prism(6,diam*1.8,nut_h,np0,axis).cut(cyl(r*.82,nut_h+2,xyz(np0,axis,-1),axis))
  self.add(n,ns,2,motion,'Nominal hex nut; actual thread flanks omitted','Purchase: nominal matching nut',name,fit=0)
  self.connect(name,name+'_head_washer','Head bearing face',0)
  self.connect(n,name+'_nut_washer','Nut bearing face',0)
  self.threads.append(dict(a=name,b=n,mask=ring(r+.001,r*.82-.001,nut_h+.002,xyz(np0,axis,-.001),axis),motion=motion,min_volume=.01,description='Bounded nominal fastener engagement'))
 def socket(self,name,host,p,axis='Z',diam=4,length=8,motion=None):
  """A screw in an explicitly modeled pilot; nominal thread region is bounded."""
  mo=self.parts[host]['motion'] if motion is None else motion;r=diam/2
  self.cut(host,cyl(r*.82,length+.1,p,axis))
  shaft=cyl(r,length,p,axis);head=cyl(r*1.75,diam,xyz(p,axis,-diam),axis)
  q=shaft.fuse(head).cut(prism(6,diam*.85,diam*.6,xyz(p,axis,-diam-.01),axis))
  self.add(name,q,3,mo,'Socket screw into nominal pilot','Select screw and validate PETG thread strength',host,0)
  self.threads.append(dict(a=name,b=host,mask=cyl(r+.001,length+.002,xyz(p,axis,-.001),axis),motion=mo,min_volume=.01,description='Bounded nominal pilot engagement'))
 def export(self,path):
  path=Path(path);path.mkdir(parents=True,exist_ok=True)
  for sub in ['cad','stl','cache','verification','renders']:(path/sub).mkdir(exist_ok=True)
  parts=[];cat=[]
  for n,v in self.parts.items():
   q=v['shape'];cq.exporters.export(q,str(path/'cad'/f'{n}.brep'))
   parts.append(cad_part(n,q,v['material'],tolerance=.04,angular=.08,motion='|'.join(v['motion']) or 'fixed',role=v['role']))
   cat.append(dict(name=n,material=v['material'],printed=v['material']==0,motion=v['motion'],role=v['role'],make=v['make'],bounds_mm=bounds(q),volume_mm3=q.Volume()))
  assembly=Assembly(self.title,parts,MAT);save_cache(assembly,path/'cache');export_step(assembly,path/'cad/machine.step');export_glb(assembly,path/'cad/machine.glb')
  for i,t in enumerate(self.threads):cq.exporters.export(t['mask'],str(path/'cad'/f'_thread_{i}.brep'))
  build_sources={str(Path(v.__file__).resolve()):hashlib.sha256(Path(v.__file__).read_bytes()).hexdigest() for v in list(sys.modules.values()) if getattr(v,'__file__',None) and Path(v.__file__).resolve().parent==Path(__file__).resolve().parent}
  manifest=dict(build_sources=build_sources,id=self.id,title=self.title,status='Candidate: not assembly-qualified',parts=cat,dofs=self.dofs,connections=self.connections,threads=[{k:v for k,v in t.items() if k!='mask'} for t in self.threads],requirements=self.requirements,service=self.service,notes=self.notes,sources=self.sources)
  (path/'catalog.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
  print(json.dumps({'machine':self.id,'parts':len(parts),'printed':sum(v['material']==0 for v in self.parts.values()),'status':manifest['status']}),flush=True)
  return manifest
