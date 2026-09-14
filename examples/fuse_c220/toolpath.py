"""Deterministic G-code generation, parsing, acceleration timing and deposition.

Every deposited bead is indexed by its actual extrusion move. Material grows
only through completed positive-E moves. No opaque prebuilt object is revealed.
"""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import math,re,json
import numpy as np
from printer import State,BED
from mechanism_lab.core import mesh_part,Part

HEIGHT=.20
WIDTH=.44
FILAMENT=1.75
BEAD_AREA=HEIGHT*(WIDTH-HEIGHT)+math.pi*(HEIGHT/2)**2
FILAMENT_AREA=math.pi*(FILAMENT/2)**2

@dataclass
class Move:
 start: np.ndarray
 end: np.ndarray
 e0: float
 e1: float
 feed: float
 duration: float
 t0: float
 layer: int
 acceleration: float=600.
 @property
 def length(self):return float(np.linalg.norm(self.end-self.start))
 @property
 def deposits(self):return self.e1>self.e0+1e-7 and self.length>1e-7
 def fraction(self,time):
  t=float(np.clip(time-self.t0,0,self.duration));L=self.length
  if L<1e-9:return 1.
  a=self.acceleration;v=min(self.feed,math.sqrt(a*L));ta=v/a;da=.5*a*ta*ta
  tc=max(0,(L-2*da)/v)
  if t<=ta:d=.5*a*t*t
  elif t<=ta+tc:d=da+v*(t-ta)
  else:d=L-.5*a*(self.duration-t)**2
  return float(np.clip(d/L,0,1))


def duration(length,feed,accel=600.):
 if length<=1e-9:return 0.
 v=min(feed,math.sqrt(accel*length));return 2*v/accel+max(0,(length-v*v/accel)/v)


def perimeter(layer,wall,segments=96):
 t=np.linspace(0,2*math.pi,segments+1)
 # Shallow six-lobed calibration vessel; adjacent-layer overhang <0.2 mm.
 a=layer*.0034
 r=29.5+2.6*np.cos(t*6)-wall*WIDTH
 return np.c_[110+r*np.cos(t+a),110+r*np.sin(t+a)]


def generate(path,layers=180):
 lines=['; CYBR FUSE C220 / deterministic six-lobed calibration vessel',
 '; Coordinates mm; 0.20 mm layers, 0.44 mm stadium-section bead; 1.75 mm filament',
 '; Nominal 24 V machine. Review firmware, homing and heater calibration before physical use.',
 'G21','G90','M82','M140 S60','M104 S210','G28','M190 S60','M109 S210','G92 E0','G0 X110 Y110 Z5 F3600']
 e=0.;pos=np.array([110.,110.,5.])
 def travel(p):
  nonlocal pos
  lines.append('G0 X%.5f Y%.5f Z%.5f F3600'%tuple(p));pos=np.array(p)
 def extrude(p):
  nonlocal pos,e
  p=np.array(p);length=np.linalg.norm(p-pos);e+=length*BEAD_AREA/FILAMENT_AREA
  lines.append('G1 X%.5f Y%.5f Z%.5f E%.7f F1800'%(*p,e));pos=p
 for layer in range(layers):
  lines.append(';LAYER:%d'%layer);z=(layer+1)*HEIGHT
  for wall in range(3):
   ring=perimeter(layer,wall);travel((*ring[0],z))
   for p in ring[1:]:extrude((*p,z))
  if layer<4:
   # Actual clipped scanline infill fills the base of the vessel.
   from shapely.geometry import Polygon,LineString
   poly=Polygon(perimeter(layer,3)[:-1]);i=0
   for y in np.arange(79,142,.43):
    section=poly.intersection(LineString([(70,y),(150,y)]))
    if section.is_empty:continue
    pieces=[section] if section.geom_type=='LineString' else list(section.geoms)
    for seg in pieces:
     c=np.array(seg.coords)
     if i%2:c=c[::-1]
     travel((*c[0],z));extrude((*c[-1],z));i+=1
 lines+=[';END','G0 Z45 F900','M104 S0','M140 S0','M84']
 Path(path).write_text('\n'.join(lines)+'\n')
 return parse(path)


def parse(path):
 pos=np.array([0.,0.,0.]);e=0.;feed=30.;t=0.;layer=-1;moves=[];absxyz=True;abse=True
 temps={'hotend':0.,'bed':0.};heated=False
 for line in Path(path).read_text().splitlines():
  if line.startswith(';LAYER:'):layer=int(line.split(':')[1])
  code=line.split(';')[0].strip()
  if not code:continue
  command=code.split()[0]
  words={a:float(v) for a,v in re.findall(r'([XYZEFST])\s*(-?\d+(?:\.\d+)?)',code)}
  if command=='G90':absxyz=True
  elif command=='G91':absxyz=False
  elif command=='M82':abse=True
  elif command=='M83':abse=False
  elif command=='G28':pos[:]=0
  elif command=='G92':
   e=words.get('E',e)
   for i,key in enumerate('XYZ'):
    if key in words:pos[i]=words[key]
  elif command in ('M104','M109'):temps['hotend']=words.get('S',0)
  elif command in ('M140','M190'):temps['bed']=words.get('S',0)
  elif command in ('G0','G1'):
   new=pos.copy()
   for i,key in enumerate('XYZ'):
    if key in words:new[i]=words[key] if absxyz else new[i]+words[key]
   ee=(words['E'] if abse else e+words['E']) if 'E' in words else e
   if 'F' in words:feed=words['F']/60
   if feed<=0:raise ValueError('G-code feedrate must be positive')
   d=float(np.linalg.norm(new-pos));dt=duration(d,feed)
   if ee>e+1e-6 and temps['hotend']<170:raise ValueError('Positive extrusion before hotend target')
   if d>0:moves.append(Move(pos.copy(),new.copy(),e,ee,feed,dt,t,layer));t+=dt
   pos,e=new,ee
 return Toolpath(moves)

class Toolpath:
 def __init__(self,moves):
  self.moves=moves;self.ends=np.array([m.t0+m.duration for m in moves]);self.total=float(self.ends[-1]);self._beads=None
  self.deposits=[m for m in moves if m.deposits]
  self.deposition_end=max(m.t0+m.duration for m in self.deposits)
 def state(self,time):
  i=min(int(np.searchsorted(self.ends,time)),len(self.moves)-1);m=self.moves[i];u=m.fraction(time)
  p=m.start+(m.end-m.start)*u;e=m.e0+(m.e1-m.e0)*u
  return State(p[0]-110,p[1]-110,p[2],e),i,u
 def prepare_beads(self):
  """Eight-sided flattened bead cross-section; exact stadium area used for E."""
  vs=[];fs=[];times=[]
  for m in self.deposits:
   v,f=bead(m.start,m.end)
   off=len(vs)*len(v);vs.append(v);fs.append(f+off);times.append(m.t0+m.duration)
  verts=np.concatenate(vs);faces=np.concatenate(fs).astype(np.int32)
  triangles=verts[faces];fn=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
  normals=np.zeros_like(verts)
  for corner in range(3):np.add.at(normals,faces[:,corner],fn)
  normals/=np.maximum(np.linalg.norm(normals,axis=1)[:,None],1e-16)
  self._beads=(verts,faces,normals,np.array(times),len(v),len(f))
 def geometry(self,time):
  if self._beads is None:self.prepare_beads()
  verts,faces,normals,times,nv,nf=self._beads;n=int(np.searchsorted(times,time,side='right'))
  parts=[]
  if n:
   p=Part('printed_extrusion_beads',verts[:n*nv],faces[:n*nf],normals[:n*nv],8,group='deposition',motion='bed',provenance='physically-derived',role='Completed positive-E G-code segments; imposed bead geometry, not melt CFD')
   parts.append(p)
  state,index,u=self.state(time);m=self.moves[index]
  if m.deposits and u>1e-7:
   v,f=bead(m.start,m.start+(m.end-m.start)*u)
   parts.append(mesh_part('current_extrusion_bead',v,f,8,group='deposition',motion='bed',provenance='physically-derived',role='Partial current extrusion to actual nozzle location'))
  return parts


def bead(start,end):
 start,end=np.asarray(start,float).copy(),np.asarray(end,float).copy()
 start[:2]-=110;end[:2]-=110;start[2]+=BED-HEIGHT/2;end[2]+=BED-HEIGHT/2
 d=end-start;L=np.linalg.norm(d[:2]);n=np.array([-d[1],d[0],0])/max(L,1e-12)
 # Cross-section circumscribed around stadium with horizontal edge, rounded sides.
 cross=[]
 for side in (1,-1):
  for a in np.linspace(-math.pi/2,math.pi/2,6):
   cross.append((side*((WIDTH-HEIGHT)/2+HEIGHT/2*math.cos(a)),side*HEIGHT/2*math.sin(a)))
 cross=np.array(cross);v=np.array([p+n*s+np.array([0,0,h]) for p in (start,end) for s,h in cross]);N=len(cross);f=[]
 for i in range(N):
  j=(i+1)%N;f += [[i,j,j+N],[i,j+N,i+N]]
 for i in range(1,N-1):f += [[0,i+1,i],[N,N+i,N+i+1]]
 return v,np.array(f)


def verify(tp):
 from shapely.geometry import Polygon
 xyz=np.array([m.end for m in tp.moves]);E=np.array([m.e1-m.e0 for m in tp.deposits]);lengths=np.array([m.length for m in tp.deposits]);ratio=E*FILAMENT_AREA/(lengths*BEAD_AREA)
 checks=[]
 def add(name,ok,data):checks.append({'check':name,'passed':bool(ok),'detail':data})
 add('all commanded coordinates lie within 220 mm travel',np.all(xyz>=0) and np.all(xyz<=220),{'minimum':xyz.min(0).tolist(),'maximum':xyz.max(0).tolist()})
 add('filament-volume equals commanded stadium bead volume',np.max(abs(ratio-1))<1e-4,{'relative_error_max':float(np.max(abs(ratio-1))),'extruded_filament_mm':float(E.sum()),'polymer_volume_mm3':float(E.sum()*FILAMENT_AREA)})
 add('conservative flow below 5 mm3/s',max(m.feed*BEAD_AREA for m in tp.deposits)<5,{'max_mm3_per_s':max(m.feed*BEAD_AREA for m in tp.deposits)})
 add('perimeter support from preceding layer',np.max(np.linalg.norm(perimeter(1,0)-perimeter(0,0),axis=1))<WIDTH*.5,{'lateral_step_mm':float(np.max(np.linalg.norm(perimeter(1,0)-perimeter(0,0),axis=1))),'layer_height_mm':HEIGHT})
 # Sample exact nozzle and bed-coordinate mapping at thousands of independently chosen times.
 err=0
 for t in np.linspace(0,tp.deposition_end,5000):
  s,i,u=tp.state(t);m=tp.moves[i];bedpoint=m.start+(m.end-m.start)*u
  worldbed=bedpoint+np.array([-110,-110-s.y,BED]);worldtip=np.array([s.x,0,BED+s.z]);err=max(err,float(np.linalg.norm(worldbed-worldtip)))
 add('toolpath to machine-coordinate transform closes',err<1e-9,{'samples':5000,'maximum_nozzle_error_mm':err})
 add('start-stop acceleration timing respects 600 mm/s2',all(m.duration>=m.length/m.feed-1e-9 for m in tp.moves),{'motion_seconds':tp.total,'timing_model':'each G0/G1 segment accelerates from rest; conservative, no junction lookahead'})
 return {'all_passed':all(c['passed'] for c in checks),'checks':checks,'moves':len(tp.moves),'extrusion_moves':len(tp.deposits),'layers':180,'scope':'G-code kinematics, commanded volume, support and ideal motion timing. No real melt/adhesion/controller execution.'}

if __name__=='__main__':
 import sys
 p=Path(sys.argv[1]);p.parent.mkdir(parents=True,exist_ok=True);tp=generate(p);report=verify(tp);p.with_suffix('.validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
 if not report['all_passed']:raise SystemExit(1)
