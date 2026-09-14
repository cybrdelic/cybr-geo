"""CYBR FUSE C220: original, millimetre-native Cartesian printer mechanism.

Geometry is built with CYBR GEO's OpenCascade/mesh part contracts. Native V9
rendering is used unmodified. No imported product meshes or generated images.
X = nozzle translation; Y = negative bed translation; Z = gantry elevation.
Nominal purchased-component envelopes require vendor reconciliation before build.
"""
from __future__ import annotations
from dataclasses import dataclass, replace
from functools import lru_cache
from pathlib import Path
import json, math, sys
import numpy as np
import cadquery as cq
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src'))
from mechanism_lab.core import Assembly, Material, View, Part, cad_part, mesh_part
from mechanism_lab.geometry import tube_mesh

BED = 88.0
PITCH = 2.0
TEETH = 20
RP = PITCH*TEETH/(2*math.pi)
LEAD = 8.0
VOLUME = (220.,220.,220.)

MATERIALS = [
 Material('Graphite anodized frame',(.025,.035,.037),.78,.32,microfinish='anodized'),
 Material('Teal powder coated brackets',(.018,.225,.190),.35,.34,coat=.18,microfinish='bead-blasted'),
 Material('Ground bearing steel',(.58,.62,.65),1.,.24,anisotropy=.35,microfinish='brushed'),
 Material('Black oxide fasteners',(.042,.045,.048),.92,.27,microfinish='turned'),
 Material('Machined aluminium',(.58,.61,.63),1.,.30,microfinish='machined'),
 Material('Brass nozzle and screw nuts',(.55,.33,.095),1.,.26,microfinish='turned'),
 Material('Timing belt rubber',(.009,.012,.013),0.,.72,microfinish='polymer'),
 Material('Textured PEI spring steel',(.31,.235,.105),.5,.69,microfinish='bead-blasted'),
 Material('Warm white PLA',(.79,.73,.61),0.,.38,microfinish='polymer'),
 Material('Copper conductors',(.64,.25,.11),1.,.27,microfinish='copper-wire'),
 Material('Black glass display',(.008,.019,.022),.18,.19,coat=.4),
 Material('Display lettering',(.44,.90,.75),0.,.4),
 Material('Off white silkscreen',(.69,.75,.70),0.,.5),
 Material('Cable jacket',(.023,.024,.028),0.,.58,microfinish='polymer'),
 Material('Workshop grey worktop',(.12,.145,.144),.08,.8,microfinish='bead-blasted'),
 Material('Ceramic heater sleeve',(.65,.62,.54),0.,.57),
]

@dataclass(frozen=True)
class State:
 x: float = 0.
 y: float = 0.
 z: float = 30.
 e: float = 0.
 def __post_init__(self):
  if not np.isfinite([self.x,self.y,self.z,self.e]).all():raise ValueError('Printer coordinates must be finite')
  if not (-110-1e-8<=self.x<=110+1e-8 and -110-1e-8<=self.y<=110+1e-8 and -1e-8<=self.z<=220+1e-8):
   raise ValueError('Commanded pose exceeds the C220 220x220x220 mm travel')


def cylinder(radius,length,pos,axis=(0,0,1),bore=0.):
 s=cq.Solid.makeCylinder(radius,length,cq.Vector(*pos),cq.Vector(*axis))
 if bore:
  s=s.cut(cq.Solid.makeCylinder(bore,length+2,cq.Vector(*(np.asarray(pos)-np.asarray(axis))),cq.Vector(*axis)))
 return s

@lru_cache(maxsize=100)
def box_shape(size,bevel=.3):
 s=cq.Workplane('XY').box(*size)
 if bevel:s=s.edges().chamfer(bevel)
 return s.val()

class Builder:
 def __init__(self):
  self.parts=[];self.links=[];self.cache={};self.serial=0
 def add(self,name,shape,mat=0,group='frame',motion='fixed',parent='base',role='',tol=.055,angular=.15):
  p=cad_part(name,shape,mat,tolerance=tol,angular=angular,group=group,motion=motion,
             role=role or name.replace('_',' '),provenance='designed-concept')
  self.parts.append(p);self.links.append((name,parent));return p
 def box(self,name,size,pos,mat=0,**kw):
  bevel=kw.pop('bevel',min(.45,min(size)*.12))
  key=(tuple(size),bevel,mat)
  if key not in self.cache:
   self.cache[key]=cad_part('cached',box_shape(tuple(size),bevel),mat,tolerance=.06,angular=.18)
  p=self.cache[key].moved(pos)
  p=replace(p,name=name,group=kw.pop('group','frame'),motion=kw.pop('motion','fixed'),role=kw.pop('role',name.replace('_',' ')))
  self.parts.append(p);self.links.append((name,kw.pop('parent','base')));return p
 def cyl(self,name,r,l,pos,axis=(0,0,1),mat=2,bore=0.,**kw):
  return self.add(name,cylinder(r,l,pos,axis,bore),mat,**kw)
 def mesh(self,name,v,f,mat=0,**kw):
  parent=kw.pop('parent','base')
  p=mesh_part(name,v,f,mat,provenance='designed-concept',role=kw.pop('role',name.replace('_',' ')),**kw)
  self.parts.append(p);self.links.append((name,parent));return p
 def tube(self,name,pts,r,mat=13,**kw):
  v,f=tube_mesh(pts,r,10);return self.mesh(name,v,f,mat,**kw)
 def bolt(self,name,pos,axis=(0,0,1),r=1.5,length=8,**kw):
  """A real socket recess, shank and head. Nominal thread represented by shank."""
  axis=np.asarray(axis,float);pos=np.asarray(pos,float)
  core=cylinder(r,length,pos-axis*length,axis)
  head=cylinder(r*1.9,r*1.65,pos,axis)
  # Orient local hex broach along the shaft, then cut an actual recess.
  hole=cq.Workplane('XY').polygon(6,r*2.0).extrude(r*1.2).val()
  if not np.allclose(axis,(0,0,1)):
   cr=np.cross([0,0,1],axis)
   if np.linalg.norm(cr)<1e-6:hole=hole.rotate((0,0,0),(1,0,0),180)
   else:hole=hole.rotate((0,0,0),tuple(cr),math.degrees(math.acos(axis[2])))
  hole=hole.translate(tuple(pos+axis*r*.65))
  return self.add(name,core.fuse(head.cut(hole)),3,**kw)
 def text(self,name,text,size,pos,mat=12,plane='XZ',motion='fixed',group='markings'):
  s=cq.Workplane(plane).text(text,size,.16,combine=False,halign='center',valign='center').val().translate(pos)
  return self.add(name,s,mat,group,motion,parent='front fascia',tol=.1,angular=.2)


def extrusion(b,name,start,end,width=20,height=20):
 start,end=np.asarray(start,float),np.asarray(end,float);d=end-start;L=np.linalg.norm(d)
 # Rectangular hollow profile with authentic open longitudinal slots, nominal custom profile.
 s=cq.Workplane('XY').rect(width,height).extrude(L).val()
 s=s.cut(cylinder(2.1,L+2,(0,0,-1)))
 for a in range(4):
  notch=cq.Workplane('XY').box(6,4,L+2).translate((0,height/2-1.6,L/2)).val()
  notch=notch.rotate((0,0,0),(0,0,1),90*a)
  s=s.cut(notch)
 if abs(d[2]/L-1)>1e-6:
  axis=np.cross([0,0,1],d/L)
  s=s.rotate((0,0,0),tuple(axis),math.degrees(math.acos(d[2]/L)))
 s=s.translate(tuple(start))
 return b.add(name,s,0,role='Hollow slotted aluminium structural profile')


def linear_rail(b,name,start,end,motion='fixed',parent='base'):
 """MGN12-style nominal 12x8 rail; actual bored rail and attached fasteners."""
 start,end=np.array(start,float),np.array(end,float);d=end-start;L=np.linalg.norm(d)
 # Local rail axis Z. Front face is -Y, depth8.
 s=box_shape((12,8,float(L)),.35).translate((0,0,L/2))
 for z in np.arange(15,L-8,25):
  s=s.cut(cylinder(1.8,10,(0,-5,z),(0,1,0))).cut(cylinder(3.,3,(0,-4,z),(0,1,0)))
 if abs(d[2]/L-1)>1e-6:
  ax=np.cross([0,0,1],d/L);angle=math.degrees(math.acos(d[2]/L))
  s=s.rotate((0,0,0),tuple(ax),angle)
 s=s.translate(tuple(start))
 p=b.add(name,s,2,'rails',motion,parent=parent,role='Nominal drilled 12 mm linear guide rail')
 return p


def carriage(b,name,pos,motion,axis='x',parent='rail'):
 # Custom U-block wraps the rail instead of overlapping solid rail volume.
 size=(46,23,27) if axis=='x' else (27,23,46)
 s=box_shape(size,.6).translate(pos)
 slot=box_shape((48,9.0,12.4) if axis=='x' else (12.4,9.0,48),0).translate((pos[0],pos[1]+8,pos[2]))
 s=s.cut(slot)
 p=b.add(name,s,4,'carriages',motion,parent=parent)
 for off in (-1,1):
  q=list(pos);q[0 if axis=='x' else 2]+=off*(25 if axis=='x' else 46)/2
  cap=box_shape((2,23,27) if axis=='x' else (27,23,2),.2).translate(q)
  cap=cap.cut(box_shape((4,9,12.4) if axis=='x' else (12.4,9,4),0).translate((q[0],q[1]+8,q[2])))
  b.add(name+f'_wiper_{off}',cap,1,'carriages',motion,parent=name)
 return p


def stepper(b,name,shaft_base,axis=(0,-1,0),motion='fixed',length=38):
 """NEMA17 mounting envelope: 42 mm face and 31 mm bolt centres."""
 axis=np.array(axis,float);p=np.array(shaft_base,float)
 # Build along local Z, orient to requested shaft direction.
 body=box_shape((42.,42.,float(length)),2).translate((0,0,-length/2))
 cap=box_shape((42.,42.,5.),1.4).translate((0,0,-2.5))
 shaft=cylinder(2.5,19,(0,0,0))
 def xf(s):
  if not np.allclose(axis,(0,0,1)):
   cross=np.cross([0,0,1],axis)
   if np.linalg.norm(cross)<1e-6:s=s.rotate((0,0,0),(1,0,0),180)
   else:s=s.rotate((0,0,0),tuple(cross),math.degrees(math.acos(axis[2])))
  return s.translate(tuple(p))
 b.add(name+'_laminated_body',xf(body),0,'motors',motion)
 b.add(name+'_front_endbell',xf(cap),4,'motors',motion)
 b.add(name+'_shaft',xf(shaft),2,'shafts',motion)
 b.add(name+'_pilot',xf(cylinder(11,2,(0,0,0),bore=2.6)),4,'motors',motion)
 for i,(u,v) in enumerate([(-15.5,-15.5),(-15.5,15.5),(15.5,-15.5),(15.5,15.5)]):
  s=cylinder(1.5,9,(u,v,-5)).fuse(cylinder(2.8,2.8,(u,v,2)))
  b.add(name+f'_mount_screw_{i}',xf(s),3,'fasteners',motion,parent=name)


def pulley(b,name,pos,axis=(0,1,0),motion='fixed',radius=RP,teeth=20):
 # Tooth pockets deliberately represented as circumferential scallops; pitch radius exact.
 s=cylinder(radius-.25,6.5,(0,0,-3.25),bore=2.5)
 for a in np.arange(teeth)*2*math.pi/teeth:
  s=s.cut(cylinder(.62,8,((radius-.3)*math.cos(a),(radius-.3)*math.sin(a),-4)))
 for z in (-4.1,3.25):s=s.fuse(cylinder(radius+1.1,.85,(0,0,z),bore=2.5))
 axis=np.array(axis,float)
 if not np.allclose(axis,(0,0,1)):
  cr=np.cross([0,0,1],axis)
  s=s.rotate((0,0,0),tuple(cr),math.degrees(math.acos(axis[2])))
 s=s.translate(pos)
 return b.add(name,s,4,'pulleys',motion,role='20T 2 mm pitch drive pulley; nominal tooth pocket envelope')


def belt_loop(b,name,a,bp,r,width,plane='xz',motion='gantry'):
 """Closed belt with tangent straight spans and sampled wrapped teeth."""
 a,bp=np.array(a,float),np.array(bp,float);u=(bp-a)/np.linalg.norm(bp-a)
 normal=np.array((0.,1,0)) if plane=='xz' else np.array((1.,0,0))
 v=np.cross(u,normal);v/=np.linalg.norm(v);L=np.linalg.norm(bp-a)
 pts=[]
 # a upper -> b upper -> b lower -> a lower -> a upper.
 for s in np.linspace(0,L,round(L/2)+1):pts.append(a+u*s+v*r)
 for t in np.linspace(0,math.pi,22)[1:]:pts.append(bp+v*r*math.cos(t)+u*r*math.sin(t))
 for s in np.linspace(0,L,round(L/2)+1)[1:]:pts.append(bp-u*s-v*r)
 for t in np.linspace(0,math.pi,22)[1:]:pts.append(a-v*r*math.cos(t)-u*r*math.sin(t))
 pts=np.array(pts);verts=[]
 tang=np.gradient(pts,axis=0);tang/=np.linalg.norm(tang,axis=1)[:,None]
 for p,t in zip(pts,tang):
  radial=np.cross(t,normal)
  for w,h in [(-width/2,-.55),(width/2,-.55),(width/2,.55),(-width/2,.55)]:verts.append(p+normal*w+radial*h)
 faces=[]
 for i in range(len(pts)-1):
  for j in range(4):
   q=i*4+j;n=i*4+(j+1)%4;faces += [[q,n,n+4],[q,n+4,q+4]]
 b.mesh(name,np.array(verts),np.array(faces),6,group='belts',motion=motion,role='Continuous belt backing on exact common tangents')
 # Every tooth follows the closed path, including both pulley wraps.
 key='x' if plane=='xz' else 'y'
 for j,s in enumerate(np.arange(1,2*L+2*math.pi*r,2)):
  p,tangent,radial=belt_point(float(s),a,bp,r,normal)
  p-=radial*.95
  sz=(.8,width,.8) if plane=='xz' else (width,.8,.8)
  tooth=b.box(name+f'_tooth_{j}',sz,p,6,motion=f'belttooth_{key}_{s:.6f}',group='belt_teeth',bevel=.06,parent=name)
 return {'name':name,'length_mm':2*L+2*math.pi*r,'center_distance_mm':L,'pitch_radius_mm':r,'closure_error_mm':float(np.linalg.norm(pts[0]-pts[-1]))}


def belt_point(s,a,bp,r,normal):
 a=np.array(a,float);bp=np.array(bp,float);normal=np.array(normal,float)
 u=(bp-a)/np.linalg.norm(bp-a);v=np.cross(u,normal);L=np.linalg.norm(bp-a);s%=2*L+2*math.pi*r
 if s<L:return a+u*s+v*r,u,v
 s-=L
 if s<math.pi*r:
  t=s/r;rad=v*math.cos(t)+u*math.sin(t);tangent=-v*math.sin(t)+u*math.cos(t)
  return bp+r*rad,tangent,rad
 s-=math.pi*r
 if s<L:return bp-u*s-v*r,-u,-v
 t=(s-L)/r;rad=-v*math.cos(t)-u*math.sin(t);tangent=v*math.sin(t)-u*math.cos(t)
 return a+r*rad,tangent,rad


def screw_thread(b,name,x,y,z0,z1,motion):
 # Four-start trapezoidal radial surface; 8 mm lead, 2 mm axial tooth pitch.
 ntheta=48;nz=round((z1-z0)/.25)+1
 th=np.arange(ntheta)*2*math.pi/ntheta;z=np.linspace(z0,z1,nz)
 phase=(z[:,None]/2 - 4*th[None,:]/(2*math.pi))%1
 dist=np.abs(phase-.5)
 radius=3.18+.82*np.clip((.42-dist)/.23,0,1)
 v=np.stack([x+radius*np.cos(th),y+radius*np.sin(th),np.broadcast_to(z[:,None],radius.shape)],axis=-1).reshape(-1,3)
 f=[]
 for k in range(nz-1):
  for j in range(ntheta):
   a=k*ntheta+j;c=k*ntheta+(j+1)%ntheta;f.extend([[a,c,c+ntheta],[a,c+ntheta,a+ntheta]])
 b.mesh(name,v,np.array(f),2,group='screws',motion=motion,center=np.array([x,y,0.]),role='Four-start T8x8 nominal trapezoidal screw surface, mesh thread')


def build():
 b=Builder();belts=[]
 # Base frame, feet, portal. Uprights attach to crossmember through gussets.
 for x in (-180,180):extrusion(b,f'base_side_{x}',(x,-245,27),(x,245,27))
 for y in (-235,50,235):extrusion(b,f'base_cross_{y}',(-170,y,27),(170,y,27))
 for x in (-175,175):
  extrusion(b,f'portal_column_{x}',(x,50,37),(x,50,466),width=20,height=40)
  for y in (-215,215):
   b.cyl(f'foot_{x}_{y}',16,12,(x,y,0),mat=6)
   b.cyl(f'foot_stud_{x}_{y}',3,10,(x,y,12),mat=3)
  for y in (25,75):
   s=cq.Workplane('XZ').polyline([(-22,0),(22,0),(0,65)]).close().extrude(5).val().translate((x,y,37))
   b.add(f'portal_gusset_{x}_{y}',s,1)
  linear_rail(b,f'z_rail_{x}',(x,25,82),(x,25,429),parent=f'portal_column_{x}')
  for z in np.arange(97,425,50):b.bolt(f'z_rail_screw_{x}_{z}',(x,21,z),(0,-1,0),length=8,group='fasteners')
 extrusion(b,'portal_top',(-185,50,476),(185,50,476),width=20,height=40)
 # Y axis twin round guides, four bored linear bushes, motor/belt and attached carrier.
 for x in (-66,66):
  b.cyl(f'y_guide_{x}',5,460,(x,-230,51),(0,1,0),mat=2,group='rails')
  for y in (-226,226):
   s=box_shape((24,16,27),.8).translate((x,y,45)).cut(cylinder(5.05,20,(x,y-10,51),(0,1,0)))
   b.add(f'y_rod_support_{x}_{y}',s,1,'brackets')
   b.bolt(f'y_support_bolt_{x}_{y}',(x+8,y,58),r=1.5,length=25,group='fasteners')
  for y in (-38,38):
   b.cyl(f'y_linear_bushing_{x}_{y}',9.5,29,(x,y-14.5,51),(0,1,0),mat=2,bore=5.03,group='carriages',motion='bed')
   s=box_shape((25,31,25),1).translate((x,y,53)).cut(cylinder(9.55,33,(x,y-16.5,51),(0,1,0)))
   b.add(f'y_bushing_clamp_{x}_{y}',s,0,'carriages','bed')
 b.box('y_bed_carrier',(178,168,5),(0,0,69),4,motion='bed',group='bed')
 # Fixed Y motor shaft is horizontal X; pulley lives on Y-Z plane below carrier.
 stepper(b,'y_motor',(-12,270,52),(1,0,0),length=38)
 pulley(b,'y_drive',(0,270,52),(1,0,0),'y_spin')
 pulley(b,'y_idler',(0,-213,52),(1,0,0),'y_idler_spin')
 mount=box_shape((5,48,50),.5).translate((-8,270,52)).fuse(box_shape((24,30,5),.3).translate((2,254,39.5)))
 mount=mount.cut(cylinder(11.3,8,(-12,270,52),(1,0,0)))
 for yy in (-15.5,15.5):
  for zz in (-15.5,15.5):mount=mount.cut(cylinder(1.7,8,(-12,270+yy,52+zz),(1,0,0)))
 b.add('y_motor_mount',mount,1,'brackets')
 b.box('y_idler_mount',(6,30,30),(-8,-221,52),1)
 b.cyl('y_idler_shaft',2.5,20,(-13,-213,52),(1,0,0),mat=2)
 belts.append(belt_loop(b,'y_belt',(0,-213,52),(0,270,52),RP,6,plane='yz',motion='fixed'))
 b.box('y_belt_anchor',(13,25,10),(0,0,62),1,motion='bed',group='bed',role='Bolted clamp joins bed carrier to upper Y belt span')
 # Four spring points connect carrier to heater/PEI stack.
 for x in (-98,98):
  for y in (-98,98):
   b.cyl(f'bed_standoff_{x}_{y}',5,10.5,(x,y,71.5),mat=15,group='bed',motion='bed')
   b.bolt(f'bed_screw_{x}_{y}',(x,y,85),r=1.5,length=19,group='bed',motion='bed')
 # Mounting ears extend the carrier to the four bed screw locations.
 for x in (-98,98):b.box(f'carrier_ear_{x}',(22,218,5),(x,0,69),4,motion='bed',group='bed')
 heater=box_shape((235,235,4),.5)
 for x in (-98,98):
  for y in (-98,98):heater=heater.cut(cylinder(1.8,6,(x,y,-3)))
 b.add('heated_aluminium_bed',heater.translate((0,0,84)),4,'bed','bed')
 b.box('heater_silicone_mat',(216,216,1),(0,0,81.5),13,motion='bed',group='bed')
 b.box('magnetic_sheet',(234,234,.7),(0,0,86.35),0,motion='bed',group='bed',bevel=.07)
 b.box('removable_PEI_sheet',(235,235,1.3),(0,0,87.35),7,motion='bed',group='bed',bevel=.09)
 b.box('PEI_front_lift_tab',(52,12,1.3),(0,-122,87.35),7,motion='bed',group='bed',bevel=.1)
 b.text('bed_mark','CYBR / FUSE',7,(0,-100,88.04),0,plane='XY',motion='bed',group='bed')
 # Gantry carriages, bored brass nuts and screw drives at x +/-137.
 for x in (-175,175):
  carriage(b,f'z_block_{x}',(x,13,BED+65),'gantry','z',f'z_rail_{x}')
  sx=np.sign(x)*137
  stepper(b,f'z_motor_{x}',(sx,49,77),(0,0,1),length=38)
  clamp=box_shape((50,44,22),.5).translate((sx,49,50)).cut(box_shape((42.2,46,24),.1).translate((sx,49,50)))
  clamp=clamp.fuse(box_shape((50,44,2),.3).translate((sx,49,38)))
  b.add(f'z_motor_clamp_{x}',clamp,1,'brackets')
  coupling=cylinder(9.5,22,(sx,49,79),bore=2.5).cut(cylinder(4.1,14,(sx,49,88)))
  b.add(f'z_coupler_{x}',coupling,4,'screws',f'z_spin_{int(sx)}')
  screw_thread(b,f'z_thread_{x}',sx,49,97,459,f'z_spin_{int(sx)}')
  b.cyl(f'z_nut_{x}',11,6,(sx,49,BED+31),mat=5,bore=4.1,group='gantry',motion='gantry')
  b.cyl(f'z_nut_barrel_{x}',5.1,15,(sx,49,BED+37),mat=5,bore=4.1,group='gantry',motion='gantry')
  # Arm touches both nut flange and carriage; bored for screw free travel.
  arm=box_shape((56,47,5),.4).translate((np.sign(x)*151,34,BED+39.5))
  arm=arm.cut(cylinder(5.2,9,(sx,49,BED+35)))
  arm=arm.cut(box_shape((40,60,9),0).translate((np.sign(x)*184.5,59.5,BED+39.5)))
  arm=arm.cut(box_shape((13,10,9),0).translate((x,25,BED+39.5)))
  b.add(f'z_nut_arm_{x}',arm,1,'gantry','gantry')
  bridge=box_shape((24,24,5),.3).translate((sx,42,BED+44.5)).cut(cylinder(6,7,(sx,49,BED+41)))
  b.add(f'z_beam_bridge_{x}',bridge,1,'gantry','gantry')
  for ox in (-7,7):b.bolt(f'z_nut_bolt_{x}_{ox}',(sx+ox,49,BED+42),r=1.2,length=9,group='gantry',motion='gantry')
  cap=box_shape((40.5,30,10),.8).translate((np.sign(x)*144.75,49,454)).cut(cylinder(4.4,12,(sx,49,447)))
  b.add(f'z_top_float_guard_{x}',cap,0,role='Radially clear guard; does not axially overconstrain screw')
 beam=box_shape((328,20,40),.6).translate((0,46,BED+67))
 for sx in (-137,137):beam=beam.cut(cylinder(6.,44,(sx,49,BED+45)))
 b.add('x_gantry_beam',beam,0,'gantry','gantry')
 linear_rail(b,'x_rail',(-157,32,BED+68),(157,32,BED+68),'gantry','x_gantry_beam')
 carriage(b,'x_carriage',(0,20,BED+68),'head','x','x_rail')
 for x in np.arange(-142,150,25):b.bolt(f'x_rail_bolt_{x}',(x,28,BED+68),(0,-1,0),length=8,motion='gantry',group='fasteners')
 stepper(b,'x_motor',(-141,48,BED+117),(0,-1,0),'gantry',38)
 pulley(b,'x_drive',(-141,39,BED+117),(0,1,0),'x_spin')
 pulley(b,'x_idler',(141,39,BED+117),(0,1,0),'x_idler_spin')
 xm=box_shape((46,5,46),.6).translate((-141,45,BED+116.5)).fuse(box_shape((46,34,7),.4).translate((-141,59,BED+90.5)))
 xm=xm.cut(cylinder(11.3,8,(-141,41,BED+117),(0,1,0)))
 for dx in (-15.5,15.5):
  for dz in (-15.5,15.5):xm=xm.cut(cylinder(1.7,8,(-141+dx,41,BED+117+dz),(0,1,0)))
 b.add('x_motor_bracket',xm,1,'gantry','gantry')
 xi=box_shape((28,6,48),.5).translate((141,50,BED+116)).fuse(box_shape((28,18,6),.3).translate((141,56,BED+90)))
 xi=xi.cut(cylinder(2.6,10,(141,45,BED+117),(0,1,0)))
 b.add('x_idler_bracket',xi,1,'gantry','gantry')
 b.cyl('x_idler_shaft',2.5,22,(141,33,BED+117),(0,1,0),group='gantry',motion='gantry')
 belts.append(belt_loop(b,'x_belt',(-141,39,BED+117),(141,39,BED+117),RP,6,motion='gantry'))
 b.box('x_belt_anchor',(24,10,8),(0,40,BED+123.5),1,motion='head',group='head')
 b.box('x_belt_anchor_bridge',(24,36.5,6),(0,26.75,BED+130.5),1,motion='head',group='head')
 # Head backplate joins carriage, clamp, feed and cooling hardware.
 plate=box_shape((55,5,89),1.2).translate((0,6,BED+97.5))
 for sx in (-6,6):plate=plate.cut(cylinder(3.,8,(sx,2,BED+104),(0,1,0)))
 b.add('toolhead_backplate',plate,1,'head','head')
 for x in (-18,18):
  for z in (BED+60,BED+78):b.bolt(f'head_carriage_bolt_{x}_{z}',(x,3,z),(0,-1,0),length=18,motion='head',group='head')
 # Nozzle: real bore and hex with conical melt outlet; outlet z=BED.
 nozzle=cq.Solid.makeCone(.35,3.3,4.5,cq.Vector(0,0,BED))
 nozzle=nozzle.fuse(cq.Workplane('XY').polygon(6,7.).extrude(3).val().translate((0,0,BED+4.5)))
 nozzle=nozzle.cut(cylinder(.2,12,(0,0,BED-1)))
 b.add('nozzle_040',nozzle,5,'hotend','head',role='0.40 mm outlet; tip coincides with commanded XYZ datum',tol=.018)
 block=box_shape((18,14,9),.4).translate((0,0,BED+11))
 block=block.cut(cylinder(3.05,20,(-10,1,BED+11),(1,0,0)))
 b.add('heater_block',block,4,'hotend','head')
 b.cyl('heater_cartridge',3.,18,(-9,1,BED+11),(1,0,0),mat=15,group='hotend',motion='head')
 b.cyl('heatbreak',2.5,24,(0,0,BED+12),mat=2,bore=1,group='hotend',motion='head')
 b.cyl('heatsink_core',4,24,(0,0,BED+25),mat=4,bore=1,group='hotend',motion='head')
 for j in range(10):b.cyl(f'heatsink_fin_{j}',10.5,1.25,(0,0,BED+25+j*2.3),mat=4,bore=1,group='hotend',motion='head')
 # Direct drive hob reaches filament centerline; spring idler opposite.
 stepper(b,'extruder_motor',(-6,14.5,BED+104),(0,-1,0),'head',20)
 for dx,dz in [(-15.5,-15.5),(-15.5,15.5),(15.5,-15.5),(15.5,15.5)]:
  b.cyl(f'extruder_motor_spacer_{dx}_{dz}',3,6,(-6+dx,8.5,BED+104+dz),(0,1,0),mat=4,bore=1.55,group='extruder',motion='head')
 b.cyl('drive_hob',5.125,7,(-6,-3.5,BED+104),(0,1,0),mat=2,group='extruder',motion='e_spin')
 b.cyl('idler_hob',5.125,7,(6,-3.5,BED+104),(0,1,0),mat=2,group='extruder',motion='e_idler_spin')
 housing=box_shape((44,15,30),1.).translate((0,-4,BED+107))
 housing=housing.cut(box_shape((30,20,17),.7).translate((0,-4,BED+105)))
 housing=housing.cut(cylinder(1.1,40,(0,0,BED+87)))
 b.add('extruder_housing',housing,0,'extruder','head')
 mount=box_shape((31,24,6),.6).translate((0,1,BED+50)).cut(cylinder(4.1,8,(0,0,BED+46)))
 b.add('hotend_clamp_bridge',mount,4,'hotend','head')
 b.cyl('filament_guide_sleeve',2,43,(0,0,BED+49),mat=15,bore=1,group='hotend',motion='head')
 b.cyl('filament_inlet',4.5,7,(0,0,BED+122),mat=4,bore=1,group='extruder',motion='head')
 b.box('idler_tension_arm',(5,18,25),(19,-4,BED+108),1,motion='head',group='extruder')
 b.cyl('idler_tension_knob',5.,9,(24,-4,BED+110),(1,0,0),mat=3,group='extruder',motion='head')
 # Cold-end axial fan, open housing, actual blades and wire guard.
 fan=box_shape((30,10,30),1).translate((0,-19,BED+38)).cut(cylinder(13.0,14,(0,-26,BED+38),(0,1,0)))
 b.add('coldend_fan_housing',fan,0,'cooling','head')
 b.cyl('coldend_fan_hub',5,4,(0,-23,BED+38),(0,1,0),mat=0,group='cooling',motion='head')
 for j in range(7):
  a=2*math.pi*j/7
  blade=box_shape((10,1.2,4),.15).translate((7,0,0)).rotate((0,0,0),(0,1,0),-a*180/math.pi)
  b.add(f'coldend_fan_blade_{j}',blade.translate((0,-22,BED+38)),0,'cooling','head')
 for r in (7,11):b.cyl(f'fan_guard_ring_{r}',r,1,(0,-25,BED+38),(0,1,0),mat=2,bore=r-.6,group='cooling',motion='head')
 for x in (-11.5,11.5):
  for z in (11.5,):b.bolt(f'fan_screw_{x}_{z}',(x,-25,BED+38+z),(0,-1,0),length=16,motion='head',group='cooling')
 for x in (-11.5,11.5):b.cyl(f'fan_mount_spacer_{x}',3,3,(x,-14,BED+49.5),(0,1,0),mat=4,bore=1.6,group='cooling',motion='head')
 # Part-cooling duct: open annular loft from blower to nozzle ring.
 duct=cq.Workplane('XY').workplane(offset=BED+8).circle(8).circle(5.5).extrude(3).val()
 b.add('part_cooling_ring',duct,0,'cooling','head')
 for sx in (-1,1):
  s=cq.Workplane('XY').workplane(offset=BED+11).center(sx*8,3).rect(4,5).workplane(offset=33).center(sx*9,8).rect(9,9).loft().val()
  inner=cq.Workplane('XY').workplane(offset=BED+10.5).center(sx*8,3).rect(2,3).workplane(offset=34).center(sx*9,8).rect(7,7).loft().val()
  b.add(f'cooling_duct_{sx}',s.cut(inner),0,'cooling','head',role='Hollow lofted duct with open inlet and outlet')
  b.box(f'part_blower_{sx}',(20,10,20),(sx*24,15,BED+46),0,motion='head',group='cooling')
 b.cyl('bed_probe_body',4,26,(25,0,BED+17),mat=2,group='head',motion='head')
 b.cyl('bed_probe_retracted_tip',1,6,(25,0,BED+11),mat=15,group='head',motion='head')
 # Limit switches and physical triggers.
 for name,pos,mode in [('x_endstop',(-125,18,BED+108),'gantry'),('y_endstop',(18,-201,64),'fixed'),('z_endstop',(-175,4,99),'fixed')]:
  b.box(name,(16,7,8),pos,0,motion=mode,group='sensors')
  b.box(name+'_lever',(18,1,1),(pos[0]+3,pos[1]-4,pos[2]+4),2,motion=mode,group='sensors')
 # Side PSU/control case, cooling slots, interface and exact connecting fasteners.
 case=box_shape((49,153,240),3).translate((219,95,153))
 # Hollow enclosure with an open service side concealed by removable cover.
 case=case.cut(box_shape((44,147,234),1).translate((222,95,153)))
 b.add('electronics_enclosure',case,0,'electronics')
 cover=box_shape((2,149,236),.3).translate((245,95,153))
 for z in range(72,145,8):cover=cover.cut(box_shape((4,50,3),.3).translate((245,110,z)))
 b.add('electronics_service_cover',cover,4,'electronics')
 for z in (70,245):b.box(f'electronics_frame_bridge_{z}',(34,38,8),(197,65,z),1,group='electronics')
 b.box('controller_pcb',(36,105,2),(220,95,198),1,group='electronics')
 b.box('power_supply_envelope',(36,127,65),(220,95,108),4,group='electronics',role='24 V supply envelope; vendor electrical design not modeled')
 for z in (49,257):
  for y in (32,158):b.bolt(f'cover_screw_{y}_{z}',(246,y,z),(1,0,0),r=1.5,length=8,group='fasteners')
 b.box('front_fascia',(185,8,38),(82,-247,47),0)
 b.box('control_display',(92,1.5,29),(85,-252,47),10,group='electronics')
 b.text('display_title','FUSE C220',6,(85,-253,54),11)
 b.text('display_readout','220 / 220 / 220',3.7,(85,-253,41),11)
 b.cyl('encoder_knob',9,9,(147,-253,46),(0,-1,0),mat=1,group='electronics')
 b.text('frame_brand','CYBR',10,(-93,-246,30),12)
 b.text('top_name','F U S E  /  C 2 2 0',9,(0,28.9,475),12)
 # Top spool mount: dual bearing-ended arms, axle and removable reel.
 for x in (72,143):
  seat=box_shape((8,22,72),.4).translate((x,50,522)).cut(cylinder(8.1,10,(x-5,50,558),(1,0,0)))
  b.add(f'spool_mount_arm_{x}',seat,1)
  b.bolt(f'spool_mount_bolt_{x}',(x,38,490),(0,-1,0),r=2,length=22,group='fasteners')
 b.cyl('spool_axle',4,97,(62,50,558),(1,0,0),mat=2)
 for x in (71,141):b.cyl(f'spool_bearing_{x}',8,9,(x,50,558),(1,0,0),mat=2,bore=4,group='spool')
 for x in (81,131):
  disc=cylinder(87,3,(x,50,558),(1,0,0),bore=25)
  for a in np.arange(8)*math.pi/4:
   y,z=50+54*math.cos(a),558+54*math.sin(a)
   disc=disc.cut(cylinder(14,5,(x-1,y,z),(1,0,0)))
  b.add(f'spool_flange_{x}',disc,0,'spool','spool_spin')
 b.cyl('spool_hub',28,50,(81,50,558),(1,0,0),mat=0,bore=8.1,group='spool',motion='spool_spin')
 for x in (84,123):b.cyl(f'spool_hub_bushing_{x}',8.1,7,(x,50,558),(1,0,0),mat=15,bore=4.05,group='spool',motion='spool_spin')
 b.cyl('filament_spool_wound',78,46,(84,50,558),(1,0,0),mat=8,bore=28,group='spool',motion='spool_spin')
 # Fine concentric winding grooves as real narrow toroidal rings, not an image.
 for x in np.arange(85,129,1.8):
  th=np.linspace(0,2*math.pi,160);pts=np.c_[np.full(len(th),x),50+78.1*np.cos(th),558+78.1*np.sin(th)]
  b.tube(f'filament_wind_{x:.1f}',pts,.7,8,group='spool',motion='spool_spin')
 # Fixed cable runs from the electronics to Z and Y motors; sleeves enter housings.
 for sx in (-137,137):
  b.tube(f'z_motor_cable_{sx}',[(sx,49,45),(sx,78,33),(165,95,36),(196,95,115)],2.2,group='wiring')
 b.tube('y_motor_cable',[(-35,280,52),(-60,284,36),(172,230,37),(207,162,94)],2.2,group='wiring')
 b.tube('power_inlet_cable',[(239,162,85),(272,188,50),(286,247,4),(395,390,3)],3.3,group='wiring')
 # Fastened base corners.
 for x in (-180,180):
  for y in (-235,235,50):
   b.box(f'base_join_plate_{x}_{y}',(36,36,3),(x,y,38.5),1)
   for dx,dy in [(-10,-10),(10,10)]:b.bolt(f'base_join_bolt_{x}_{y}_{dx}',(x+dx,y+dy,40),r=2,length=13,group='fasteners')
 a=Assembly('CYBR FUSE C220',b.parts,MATERIALS,metadata={'truth_intent':'concept','units':'mm','build_volume_mm':VOLUME,'belt_systems':belts,'connections':b.links,'design_status':'digital prototype; original CAD with nominal purchased interfaces','source_repository':'https://github.com/cybrdelic/cybr-geo','motor_mm_per_revolution':{'x':40.,'y':40.,'z':8.}})
 studio=dict(studio_target=(0,0,260),studio_scale=3.2,studio_az=235.,studio_el=28.,floor_z_mm=0.,floor_color=(.135,.15,.147),background_color=(.075,.084,.083),environment_strength=.42,light_size=1.5,light_intensity=1.3,exposure=1.10,f_stop=11.,tone_mapping='neutral')
 a.views={
 'hero':View(az=235,el=19,scale=430,target=(20,15,280),focal_length_mm=62,**studio),
 'rear':View(az=135,el=25,scale=345,target=(20,35,300),focal_length_mm=65,**studio),
 'printing':View(az=241,el=30,scale=145,target=(0,-10,137),focal_length_mm=75,**{**studio,'f_stop':8}),
 'drive':View(az=213,el=20,scale=106,target=(-110,25,190),focal_length_mm=85,**{**studio,'f_stop':11}),
 'front':View(az=270,el=5,scale=355,target=(15,0,300),focal_length_mm=70,**studio),
 }
 a.motion_function=motion(State())
 return a


def rotation(axis,angle,center):
 axis=np.asarray(axis,float);axis/=np.linalg.norm(axis)
 K=np.array([[0,-axis[2],axis[1]],[axis[2],0,-axis[0]],[-axis[1],axis[0],0]])
 R=np.eye(3)+math.sin(angle)*K+(1-math.cos(angle))*(K@K)
 T=np.eye(4);T[:3,:3]=R;T[:3,3]=np.asarray(center)-R@np.asarray(center);return T


def motion(state:State):
 def pose(p,t=0.,explode=0.):
  T=np.eye(4);m=p.motion
  if m=='bed':T[1,3]=-state.y
  elif m=='gantry':T[2,3]=state.z
  elif m=='head':T[:3,3]=(state.x,0,state.z)
  elif m in ('x_spin','x_idler_spin'):
   c=(-141 if m=='x_spin' else 141,39,BED+117)
   T=rotation((0,1,0),2*math.pi*state.x/40,c);T[2,3]+=state.z
  elif m in ('y_spin','y_idler_spin'):
   c=(0,270 if m=='y_spin' else -213,52)
   T=rotation((1,0,0),2*math.pi*state.y/40,c)
  elif m.startswith('belttooth_'):
   _,axis,seed=m.split('_');s=float(seed)
   if axis=='x':pa,pb,n,shift=(-141,39,BED+117),(141,39,BED+117),(0,1,0),state.x
   else:pa,pb,n,shift=(0,-213,52),(0,270,52),(1,0,0),state.y
   old,ot,orr=belt_point(s,pa,pb,RP,n);new,nt,nrr=belt_point(s+shift,pa,pb,RP,n)
   old-=orr*.95;new-=nrr*.95
   angle=math.atan2(np.dot(np.cross(ot,nt),n),np.dot(ot,nt))
   T=rotation(n,angle,old);T[:3,3]+=new-old
   if axis=='x':T[2,3]+=state.z
  elif m.startswith('z_spin_'):
   x=int(m.split('_')[-1]);T=rotation((0,0,1),-2*math.pi*state.z/8,(x,49,0))
  elif m in ('e_spin','e_idler_spin'):
   T=rotation((0,1,0),(1 if m=='e_spin' else -1)*state.e/5.125,(-6 if m=='e_spin' else 6,0,BED+104));T[:3,3]+=np.array((state.x,0,state.z))
  elif m=='spool_spin':T=rotation((1,0,0),state.e/78,(0,50,558))
  if explode:T[:3,3]+=np.asarray(p.explode)*explode
  return T
 return pose


def posed(a,state,deposit=None):
 """Recompute flexible routes at exact moving endpoints. No rigid dangling cables."""
 b=Builder()
 x,y,z=state.x,state.y,state.z
 # Bowden guide and filament connect fixed upper guide to actual head inlet.
 q=np.array([(106,50,636),(96,-42,645),(x*.4,-62,460),(x,-16,BED+z+174),(x,0,BED+z+129)])
 from scipy.interpolate import PchipInterpolator
 tt=np.linspace(0,1,len(q));u=np.linspace(0,1,90);pts=np.stack([PchipInterpolator(tt,q[:,i])(u) for i in range(3)],axis=1)
 b.tube('dynamic_filament_feed',pts,.875,8,group='flex')
 q=np.array([(192,60,263),(165,60,330+z*.3),(x+65,55,BED+z+146),(x+27.5,6,BED+z+136)])
 pts=np.stack([PchipInterpolator(np.linspace(0,1,len(q)),q[:,i])(u) for i in range(3)],axis=1)
 b.tube('dynamic_toolhead_umbilical',pts,3,13,group='flex')
 q=np.array([(208,88,120),(164,120,83),(140,90-y,75),(115,55-y,82)])
 pts=np.stack([PchipInterpolator(np.linspace(0,1,len(q)),q[:,i])(u) for i in range(3)],axis=1)
 b.tube('dynamic_bed_heater_cable',pts,2.6,13,group='flex')
 parts=list(a.parts)+b.parts
 if deposit is not None:parts+=deposit
 return replace(a,parts=parts,motion_function=motion(state))

if __name__=='__main__':
 from mechanism_lab.core import validate,save_cache
 from mechanism_lab.exporters import export_glb,export_step,export_bom
 out=Path(sys.argv[1] if len(sys.argv)>1 else 'outputs/fuse_c220');out.mkdir(parents=True,exist_ok=True)
 a=build();print('BUILT',len(a.parts),'parts',sum(len(p.faces) for p in a.parts),'triangles',flush=True)
 save_cache(a,out/'cache')
 export_glb(posed(a,State()),out/'FUSE_C220.glb')
 export_step(a,out/'FUSE_C220_analytic.step',individual=False)
 export_bom(a,out)
 print('EXPORTED',flush=True)
