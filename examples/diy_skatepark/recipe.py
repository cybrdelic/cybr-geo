"""CYBR YARD / parametric DIY skatepark built with CYBR GEO.

Millimetres, Z-up. Every visible object is procedural 3-D geometry. This is a
nominal design study, not a structural calculation or a permit-ready ramp plan.
No reference photos, generated images, image billboards, or hidden mesh assets.
"""
from __future__ import annotations
from dataclasses import dataclass, replace
from functools import lru_cache
from pathlib import Path
import math
import numpy as np
import cadquery as cq
import trimesh
from mechanism_lab.core import Assembly, Material, Part, View, cad_part, mesh_part, pose_cad

MM_PER_FOOT = 304.8
TAU = 2 * math.pi

@dataclass(frozen=True)
class ParkConfig:
    slab_length: float = 22000.
    slab_width: float = 16000.
    mini_width: float = 4800.
    mini_height: float = 1000.
    mini_radius: float = 2200.
    mini_flat: float = 3600.
    mini_deck: float = 1000.
    mini_base: float = 165.
    mini_center: tuple[float,float] = (-4900., 3000.)
    coping_radius: float = 30.15
    coping_reveal: float = 6.
    seed: int = 260913

    def check(self):
        if min(self.slab_length,self.slab_width,self.mini_width,self.mini_flat,self.mini_deck) <= 0:
            raise ValueError('Dimensions must be positive')
        if not 0 < self.mini_height < self.mini_radius:
            raise ValueError('Mini ramp requires a nonvertical circular transition')
        if self.mini_width % 1200 != 0:
            raise ValueError('This framing recipe uses 1200 mm width bays')
        if not 0 < self.coping_reveal < self.coping_radius:
            raise ValueError('Coping reveal must be smaller than its radius')

MATERIALS = [
    Material('Warm trowelled concrete', (.40,.408,.390), 0,.83,microfinish='concrete'),
    Material('Concrete alternate pour', (.375,.383,.369), 0,.86,microfinish='concrete'),
    Material('Concrete edge / aggregate base', (.24,.245,.222), 0,.92,microfinish='concrete'),
    Material('Birch plywood face', (.39,.258,.137), 0,.62,microfinish='wood'),
    Material('Birch plywood pale face', (.44,.300,.169), 0,.64,microfinish='wood'),
    Material('Birch plywood dark face', (.34,.223,.118), 0,.66,microfinish='wood'),
    Material('Sealed riding plywood', (.40,.275,.151), 0,.48,coat=.06,coat_rough=.45,microfinish='wood'),
    Material('Sealed riding plywood alternate', (.365,.244,.129), 0,.52,microfinish='wood'),
    Material('Fresh softwood framing', (.45,.340,.206), 0,.77,microfinish='wood'),
    Material('Plywood dark glue-line', (.125,.079,.039), 0,.74),
    Material('Galvanized coping / fasteners', (.49,.52,.55), .92,.28,microfinish='brushed'),
    Material('Blackened structural steel', (.033,.039,.042), .78,.36,microfinish='anodized'),
    Material('Waxed grind steel', (.24,.26,.275), .88,.25,microfinish='brushed'),
    Material('Weathered cedar fence', (.135,.100,.064), 0,.86,microfinish='wood'),
    Material('Cedar fence lighter', (.166,.126,.083), 0,.85,microfinish='wood'),
    Material('Powdercoat / CYBR blue-green', (.025,.110,.117), .18,.44),
    Material('Off-white painted inlay', (.72,.725,.67), 0,.65),
    Material('Planter earth', (.072,.052,.032), 0,.99,microfinish='concrete'),
    Material('Olive leaves', (.095,.16,.055), 0,.82),
    Material('Leaf highlights', (.145,.205,.075), 0,.78),
    Material('Urethane skateboard wheels', (.67,.645,.518), 0,.43),
    Material('Black skateboard grip', (.012,.014,.013), 0,.94,microfinish='grip'),
    Material('Graphite expansion joint', (.042,.047,.045), 0,.98),
    Material('Ochre line marking', (.58,.34,.067), 0,.76),
]

class Builder:
    def __init__(self, cfg: ParkConfig):
        self.cfg=cfg;self.parts=[];self.cutlist=[];self.rng=np.random.default_rng(cfg.seed)
        self.obstacles=[];self._templates={};self._serial=0

    def place(self, name, source: Part, center=(0,0,0), matrix=None, material=None,
              group='site', role='', explode=(0,0,0), axis=(1,0,0), stock=None):
        R=np.eye(3) if matrix is None else np.asarray(matrix,float)
        center=np.asarray(center,float);T=np.eye(4);T[:3,:3]=R;T[:3,3]=center
        p=replace(source,name=name,vertices=source.vertices@R.T+center,
                  normals=source.normals@R.T,material=source.material if material is None else material,
                  group=group,role=role or name.replace('_',' '),provenance='designed-concept',
                  center=center,explode=np.asarray(explode,float),finish_axis=tuple(R@np.asarray(axis)),
                  finish_origin=tuple(center),cad=pose_cad(source.cad,T) if source.cad is not None else None)
        self.parts.append(p)
        if stock is not None:
            self.cutlist.append(dict(name=name,group=group,material=MATERIALS[p.material].name,
                stock=stock,nominal_bounds_mm=np.ptp(p.vertices,axis=0).round(3).tolist()))
        return p

    def shape(self,name,shape,material,group,role='',explode=(0,0,0),axis=(1,0,0),stock=None):
        p=cad_part(name,shape,material,tolerance=.35,angular=.065,group=group,
                   role=role or name.replace('_',' '),provenance='designed-concept',
                   explode=np.asarray(explode,float),finish_axis=axis)
        p.finish_origin=tuple(p.bounds.mean(axis=0));self.parts.append(p)
        if stock is not None:
            self.cutlist.append(dict(name=name,group=group,material=MATERIALS[material].name,
                stock=stock,nominal_bounds_mm=np.ptp(p.vertices,axis=0).round(3).tolist()))
        return p

    def box(self,name,size,center,mat,group='site',bevel=1.,matrix=None,explode=(0,0,0),axis=(1,0,0),stock=None):
        size=tuple(float(v) for v in size);key=('box',size,float(bevel))
        if key not in self._templates:
            sh=cq.Workplane('XY').box(*size)
            if bevel > 0: sh=sh.edges().chamfer(min(bevel,min(size)*.18))
            self._templates[key]=cad_part('template',sh.val(),0,tolerance=.5,angular=.08)
        return self.place(name,self._templates[key],center,matrix,mat,group,explode=explode,axis=axis,stock=stock)

    def tube(self,name,a,b,r,mat,group='metal',wall=0.,sides=32):
        a=np.asarray(a,float);b=np.asarray(b,float);d=b-a;L=float(np.linalg.norm(d))
        key=('tube',round(L,6),r,wall,sides)
        if key not in self._templates:
            sh=cq.Workplane('XY').circle(r)
            if wall:sh=sh.circle(r-wall)
            sh=sh.extrude(L).translate((0,0,-L/2)).val()
            self._templates[key]=cad_part('template',sh,0,tolerance=.12,angular=TAU/sides)
        R=trimesh.geometry.align_vectors((0,0,1),d/L)[:3,:3]
        return self.place(name,self._templates[key],(a+b)/2,R,mat,group,axis=(0,0,1))

    def screw(self,name,p,n=(0,0,1),group='hardware',r=3.5):
        key=('screw',r)
        if key not in self._templates:
            sh=cq.Workplane('XY').circle(r).extrude(1.4).translate((0,0,-1.4))
            cut=cq.Workplane('XY').box(r*1.35,.8,.65).translate((0,0,-.22))
            cut=cut.union(cq.Workplane('XY').box(.8,r*1.35,.65).translate((0,0,-.22)))
            self._templates[key]=cad_part('template',sh.cut(cut).val(),0,tolerance=.12,angular=.32)
        n=np.asarray(n,float);n/=np.linalg.norm(n);R=trimesh.geometry.align_vectors((0,0,1),n)[:3,:3]
        return self.place(name,self._templates[key],np.asarray(p)+n*.07,R,10,group,
                          role='Nominal flush screw head; detailed countersink and load capacity unqualified')

    def text(self,name,text,pos,height,mat=16,plane='XY',group='markings',center=True):
        # CadQuery/OpenCascade font outlines become physical extruded solids.
        sh=cq.Workplane(plane).text(text,height,.35,combine=True,halign='center' if center else 'left',valign='center').val()
        return self.shape(name,sh.translate(pos),mat,group,role='Physical 0.35 mm text inlay')

    def beam(self,name,a,b,width,height,mat=8,group='framing',axis='Y'):
        a=np.asarray(a,float);b=np.asarray(b,float);d=b-a;L=np.linalg.norm(d)
        # Local X follows beam span; local Z is the closest upward cross section.
        xx=d/L;yy=np.cross((0,0,1),xx)
        if np.linalg.norm(yy)<1e-9:yy=np.array((0,1,0))
        yy=yy/np.linalg.norm(yy);zz=np.cross(xx,yy);R=np.column_stack((xx,yy,zz))
        return self.box(name,(L,width,height),(a+b)/2,mat,group,bevel=1.0,matrix=R,
                        stock=f'{width:g} x {height:g} timber, span {L:.1f} mm')


def arc_shell(radius,zcenter,a0,a1,offset,thickness,width):
    """Exact concentric cylindrical strip, not a faceted hand-shaped ramp."""
    def p(a,r):return (r*math.sin(a),zcenter-r*math.cos(a))
    r0=radius+offset;r1=r0+thickness;am=(a0+a1)/2
    return (cq.Workplane('XZ').moveTo(*p(a0,r0)).threePointArc(p(am,r0),p(a1,r0))
            .lineTo(*p(a1,r1)).threePointArc(p(am,r1),p(a0,r1)).close().extrude(width).val())


def quarter(b:Builder,prefix,toe,cy,height,radius,width,deck,sign=1,base=0.,standalone=False):
    theta=math.acos(1-height/radius);run=radius*math.sin(theta);top=base+height
    nbay=int(round(width/1200));bay=width/nbay;cop_r=b.cfg.coping_radius
    # Coping has the same authored 6 mm reveal relative to deck and tangent.
    dz=b.cfg.coping_reveal-cop_r
    dx=(cop_r-b.cfg.coping_reveal)*(1-math.cos(theta))/math.sin(theta)
    cx=toe+sign*(run+dx);cz=top+dz
    cutback=7.;amax=math.asin((run-cutback)/radius)
    amin=math.acos(1-45./radius) if standalone else 0.
    nseg=3 if not standalone else 2
    segments=np.linspace(amin,amax,nseg+1)
    group=prefix+'_skin';frame=prefix+'_frame';fascia=prefix+'_fascia'
    def world(shape):
        if sign<0:shape=shape.mirror('YZ')
        return shape.translate((toe,cy+width/2,0))
    for j in range(nbay):
        for k in range(nseg):
            a0,a1=segments[k:k+2];gap=.38/radius
            for layer,offset,thick in ((0,0.,6.),(1,6.,9.),(2,15.,9.)):
                local=arc_shell(radius,base+radius,a0+gap,a1-gap,offset,thick,bay-.7)
                local=local.translate((0,-j*bay-.35,0))
                material=(6+(j+k)%2) if layer==0 else (3+layer%3)
                b.shape(f'{prefix}_curve_panel_{j:02}_{k:02}_ply{layer}',world(local),material,group,
                    role=f'Cylindrical {thick:g} mm plywood layer; nominal R={radius:g} mm transition',
                    explode=(0,0,320+layer*130),axis=(0,1,0),stock=f'{thick:g} mm sheet; curved panel')
            for ai,a in enumerate(np.linspace(a0+.065,a1-.065,3)):
                for yi,y in enumerate((cy+width/2-j*bay-30,cy+width/2-(j+1)*bay+30)):
                    p=(toe+sign*radius*math.sin(a),y,base+radius*(1-math.cos(a)))
                    b.screw(f'{prefix}_skin_screw_{j}_{k}_{ai}_{yi}',p,(-sign*math.sin(a),0,math.cos(a)),prefix+'_screws')
    # Radial plywood diaphragms. Joists butt between webs rather than passing through them.
    ribstart=math.acos(1-80./radius) if standalone else 0.
    x0=radius*math.sin(ribstart);x1=run
    underside_radius=radius+24.
    a0rib=math.asin(x0/underside_radius);a1rib=math.asin(x1/underside_radius)
    z0=base+radius-underside_radius*math.cos(a0rib)
    z1=base+radius-underside_radius*math.cos(a1rib);am=(a0rib+a1rib)/2
    bottom=0. if standalone else 38.
    for j in range(nbay+1):
        sh=(cq.Workplane('XZ').moveTo(x0,bottom).lineTo(x1,bottom).lineTo(x1,z1)
            .threePointArc((underside_radius*math.sin(am),base+radius-underside_radius*math.cos(am)),(x0,z0)).close()
            .extrude(18).translate((0,-j*bay+9,0)).val())
        b.shape(f'{prefix}_curved_rib_{j}',world(sh),4,fascia if j in (0,nbay) else frame,
                axis=(1,0,0),stock='18 mm plywood diaphragm')
    for k,a in enumerate(np.linspace(max(.06,ribstart+.04),theta-.04,11)):
        n=np.array((-sign*math.sin(a),0,math.cos(a)))
        p=np.array((toe+sign*radius*math.sin(a),cy,base+radius*(1-math.cos(a))))-n*(24+44.5)
        if p[2]-44.5<0:continue
        R=np.array([[math.cos(a),0,-sign*math.sin(a)],[0,1,0],[sign*math.sin(a),0,math.cos(a)]])
        for j in range(nbay):
            y=cy+width/2-(j+.5)*bay
            b.box(f'{prefix}_curve_joist_{k:02}_{j}',(38,bay-18,89),(p[0],y,p[2]),8,frame,matrix=R,
                    axis=(0,1,0),stock=f'38 x 89 timber, {bay-18:g} mm')
    # Deck begins beyond the actual coping cross section, with no floating top.
    deckstart=run+34.;decklen=deck-34.
    for j in range(nbay):
        x=toe+sign*(deckstart+decklen/2);y=cy+width/2-(j+.5)*bay
        b.box(f'{prefix}_deck_{j}',(decklen,bay-.7,24),(x,y,top-12),6+j%2,group,
              bevel=.5,explode=(0,0,420),axis=(0,1,0),stock='24 mm layered deck skin')
        for xx in (-decklen/2+30,decklen/2-30):
            for yy in (-bay/2+30,bay/2-30):b.screw(f'{prefix}_deck_fastener_{j}_{xx}_{yy}',(x+xx,y+yy,top),group=prefix+'_screws')
        for t in (0.,.5,1.):
            px=toe+sign*(deckstart+25+t*(decklen-50))
            b.box(f'{prefix}_deck_joist_{j}_{t}',(38,bay-18,140),(px,y,top-24-70),8,frame,
                  axis=(0,1,0),stock=f'38 x 140 timber, {bay-18:g} mm')
    for j in range(nbay+1):
        y=cy+width/2-j*bay
        for k,xx in enumerate((run+85,run+deck-75)):
            x=toe+sign*xx;H=top-24-140
            b.box(f'{prefix}_deck_leg_{j}_{k}',(89,89,H),(x,y,H/2),8,frame,
                  axis=(0,0,1),stock=f'89 x 89 post, {H:.1f} mm')
            b.box(f'{prefix}_deck_foot_{j}_{k}',(150,150,8),(x,y,4),11,frame,bevel=2)
    # True hollow coping tube, closed ends are separate plugs.
    b.tube(f'{prefix}_hollow_coping',(cx,cy-width/2-12,cz),(cx,cy+width/2+12,cz),cop_r,10,
           prefix+'_coping',wall=3.2,sides=64)
    for j in range(nbay+1):
        y=cy-width/2+j*bay
        b.tube(f'{prefix}_coping_anchor_{j}',(cx+sign*16,y,cz-5),(cx+sign*60,y,cz-48),5,11,frame)
    for s in (-1,1):
        b.tube(f'{prefix}_coping_plug_{s}',(cx,cy+s*(width/2+10),cz),(cx,cy+s*(width/2+13),cz),cop_r-3.3,11,prefix+'_coping')
    # Back guardrails and intermediate infill; nominal geometry, not code certification.
    backx=toe+sign*(run+deck-44.5)
    for j in range(nbay+1):
        y=cy-width/2+j*bay
        b.box(f'{prefix}_guard_post_{j}',(89,89,1150),(backx,y,top+525),8,prefix+'_guard',axis=(0,0,1),stock='89 x 89 guard post')
    for k,z in enumerate((top+120,top+1000)):
        b.box(f'{prefix}_guard_rail_{k}',(38,width+89,89),(backx-sign*64,cy,z),8,prefix+'_guard',axis=(0,1,0),stock='38 x 89 guard rail')
    for j,y in enumerate(np.arange(cy-width/2+100,cy+width/2-60,120)):
        b.box(f'{prefix}_guard_infill_{j}',(25,38,850),(backx-sign*57,y,top+557),13,prefix+'_guard',bevel=.7,axis=(0,0,1))
    # Short side guards on the exposed deck; an access opening is retained on north-east mini deck.
    for side in (-1,1):
        if prefix=='mini_east' and side==1:continue
        y=cy+side*(width/2-28)
        x0=toe+sign*(run+85);x1=toe+sign*(run+deck-80)
        for k,x in enumerate((x0,x1)):
            b.box(f'{prefix}_side_post_{side}_{k}',(70,70,1060),(x,y,top+530),8,prefix+'_guard',axis=(0,0,1))
        for k,z in enumerate((top+200,top+995)):
            b.box(f'{prefix}_side_rail_{side}_{k}',(abs(x1-x0),38,80),((x0+x1)/2,y,z),8,prefix+'_guard')
        for k,x in enumerate(np.linspace(x0,x1,7)[1:-1]):
            b.box(f'{prefix}_side_infill_{side}_{k}',(38,25,740),(x,y,top+590),13,prefix+'_guard',bevel=.6,axis=(0,0,1))
    if standalone:
        # Hermite steel approach is tangent to grade and the cylindrical wood surface.
        xend=radius*math.sin(amin);xstart=-160.;L=xend-xstart;zA=1.5;zB=45.
        slope=math.tan(amin);ts=np.linspace(0,1,80);xs=xstart+ts*L
        zs=(2*ts**3-3*ts**2+1)*zA+(-2*ts**3+3*ts**2)*zB+(ts**3-ts**2)*(L*slope)
        v=[]
        for yy in (-width/2,width/2):
            for thick in (0.,1.4):v.extend([(toe+sign*x,cy+yy,z-thick) for x,z in zip(xs,zs)])
        n=len(xs);f=[]
        for i in range(n-1):
            f.extend([[i,i+1,2*n+i+1],[i,2*n+i+1,2*n+i],
                      [n+i,3*n+i+1,n+i+1],[n+i,3*n+i,3*n+i+1]])
            f.extend([[i,n+i+1,i+1],[i,n+i,n+i+1],
                      [2*n+i,2*n+i+1,3*n+i+1],[2*n+i,3*n+i+1,3*n+i]])
        f.extend([[0,2*n,3*n],[0,3*n,n],[n-1,2*n-1,4*n-1],[n-1,4*n-1,3*n-1]])
        part=mesh_part(f'{prefix}_tangent_steel_entry',np.array(v),np.array(f),10,group=prefix+'_entry',
            provenance='designed-concept',role='1.4 mm nominal steel approach; Hermite tangent continuity; anchorage unqualified')
        b.parts.append(part)
    return dict(prefix=prefix,toe_mm=toe,center_y_mm=cy,height_mm=height,radius_mm=radius,width_mm=width,
                base_mm=base,run_mm=run,deck_mm=deck,theta_rad=theta,sign=sign,
                coping_center_mm=[cx,cy,cz],coping_radius_mm=cop_r,coping_reveal_mm=b.cfg.coping_reveal)


def mini_ramp(b:Builder):
    c=b.cfg;cx,cy=c.mini_center;W=c.mini_width;F=c.mini_flat;B=c.mini_base
    west=quarter(b,'mini_west',cx-F/2,cy,c.mini_height,c.mini_radius,W,c.mini_deck,-1,B)
    east=quarter(b,'mini_east',cx+F/2,cy,c.mini_height,c.mini_radius,W,c.mini_deck,1,B)
    for i in range(3):
        for j in range(4):
            x=cx-F/2+(i+.5)*1200;y=cy-W/2+(j+.5)*1200
            for layer,z,thick,mat in ((0,B-3,6,6+(i+j)%2),(1,B-10.5,9,3),(2,B-19.5,9,4)):
                b.box(f'mini_flat_panel_{i}_{j}_ply{layer}',(1199.4,1199.4,thick),(x,y,z),mat,'mini_flat_skin',bevel=.2,
                      explode=(0,0,320+layer*130),axis=(0,1,0),stock=f'{thick} mm sheet, 1200 mm nominal module')
            for xx in (-570,0,570):
                for yy in (-570,570):b.screw(f'mini_flat_screw_{i}_{j}_{xx}_{yy}',(x+xx,y+yy,B),group='mini_flat_screws')
    for i,x in enumerate(np.linspace(cx-F/2+20,cx+F/2-20,10)):
        for j in range(4):
            y=cy-W/2+(j+.5)*1200
            b.box(f'mini_flat_joist_{i}_{j}',(38,1182,89),(x,y,B-24-44.5),8,'mini_flat_frame',axis=(0,1,0),stock='38 x 89 timber, 1182 mm')
    for j,y in enumerate(np.linspace(cy-W/2,cy+W/2,5)):
        b.box(f'mini_flat_rim_{j}',(F,18,141),(cx,y,70.5),4,'mini_flat_fascia' if j in (0,4) else 'mini_flat_frame',stock='18 mm plywood flat-bottom web')
        b.box(f'mini_flat_sleeper_{j}',(F,89,38),(cx,y,19),8,'mini_flat_frame',stock='38 x 89 ground sleeper')
    # Six timber treads on three continuous nominal stringers.
    top=B+c.mini_height;rise=top/6;tread=260.;run=6*tread
    x= east['toe_mm']+east['run_mm']+c.mini_deck/2
    yedge=cy+W/2
    for k in range(6):
        y=yedge+(5-k)*tread+tread/2;z=(k+1)*rise
        b.box(f'mini_access_tread_{k}',(920,tread-3,32),(x,y,z-16),8,'mini_access',bevel=2,stock='32 mm stair tread')
        b.box(f'mini_access_riser_{k}',(890,18,rise-32),(x,y+tread/2-12,z-32-(rise-32)/2),4,'mini_access')
    for j,xx in enumerate((x-405,x,x+405)):
        # Closed sawtooth profile in YZ, extruded as a genuine plywood stringer.
        pts=[(yedge,0),(yedge+run,0)]
        for k in range(6):
            y=yedge+run-k*tread;z=(k+1)*rise-32
            pts.extend([(y,z),(y-tread,z)])
        sh=cq.Workplane('YZ').polyline(pts).close().extrude(38).translate((xx-19,0,0)).val()
        b.shape(f'mini_access_stringer_{j}',sh,8,'mini_access',axis=(0,1,0),stock='38 mm nominal cut stringer')
    for side in (-1,1):
        xx=x+side*485
        for k,(y,z) in enumerate(((yedge+run-140,rise),(yedge+80,top))):
            b.box(f'mini_stair_post_{side}_{k}',(70,70,1000),(xx,y,z+480),8,'mini_access',axis=(0,0,1))
        b.beam(f'mini_stair_handrail_{side}',(xx,yedge+run-140,rise+960),(xx,yedge+80,top+960),55,70,8,'mini_access')
    b.obstacles.append(dict(name='Mini ramp',x0=cx-F/2-west['run_mm']-c.mini_deck,
         x1=cx+F/2+east['run_mm']+c.mini_deck,y0=cy-W/2,y1=cy+W/2,
         height_mm=c.mini_height,kind='transition'))
    return west,east


def bank(b:Builder):
    toe=-6500.;cy=-4400.;width=3000.;run=2350.;height=650.;deck=850.;base=18.
    top=height;angle=math.atan2(top-base,run)
    for j in range(3):
        y=cy-width/2+(j+.5)*1000
        # Thin rectangular riding skin, tilted as a plane and physically supported.
        L=math.hypot(run,top-base)
        R=np.array([[math.cos(angle),0,math.sin(angle)],[0,1,0],[-math.sin(angle),0,math.cos(angle)]])
        b.box(f'bank_skin_{j}',(L,999.2,18),(toe-run/2,y,(top+base)/2-9/math.cos(angle)),6+j%2,'bank_skin',
              bevel=.3,matrix=R,axis=(0,1,0),explode=(0,0,400),stock='18 mm nominal layered bank sheet')
        b.box(f'bank_deck_{j}',(deck,999.2,24),(toe-run-deck/2,y,top-12),6+j%2,'bank_skin',stock='24 mm deck skin')
    for j in range(4):
        y=cy-width/2+j*1000
        sh=cq.Workplane('XZ').polyline([(toe,0),(toe-run-deck,0),(toe-run-deck,top-24),(toe-run,top-24),(toe,1)]).close().extrude(18).translate((0,y+9,0)).val()
        b.shape(f'bank_rib_{j}',sh,4,'bank_frame',stock='18 mm shaped plywood diaphragm')
    for k,xlocal in enumerate(np.linspace(220,run-80,8)):
        z=base+(top-base)*xlocal/run
        if z-18-89 < 0: continue
        for j in range(3):
            b.box(f'bank_joist_{k}_{j}',(38,982,89),(toe-xlocal,cy-width/2+(j+.5)*1000,z-18-44.5),8,'bank_frame',axis=(0,1,0),stock='38 x 89 joist')
    for j in range(4):
        y=cy-width/2+j*1000
        b.box(f'bank_deck_post_{j}',(89,89,top-24),(toe-run-deck+75,y,(top-24)/2),8,'bank_frame',axis=(0,0,1))
    # Flat, thin transition plate tapers the small bank toe down toward the slab.
    sh=cq.Workplane('XZ').polyline([(toe+230,1.5),(toe-80,40),(toe-80,37),(toe+230,.1)]).close().extrude(width).translate((0,cy+width/2,0)).val()
    b.shape('bank_steel_entry',sh,10,'bank_entry')
    for side in (-1,1):
        y=cy+side*(width/2-50)
        b.box(f'bank_back_post_{side}',(70,70,1080),(toe-run-deck+60,y,top+520),8,'bank_guard',axis=(0,0,1))
    for k,z in enumerate((top+220,top+1000)):
        b.box(f'bank_back_rail_{k}',(38,width-20,80),(toe-run-deck+100,cy,z),8,'bank_guard',axis=(0,1,0))
    b.obstacles.append(dict(name='Bank',x0=toe-run-deck,x1=toe+230,y0=cy-width/2,y1=cy+width/2,height_mm=height,kind='bank'))


def street(b:Builder):
    # A low, replaceable-sheet manual pad with edge steel and visible framed base.
    cx,cy,L,W,H=-2450.,-4470.,2400.,1200.,200.
    for s in (-1,1):
        b.box(f'manual_long_rim_{s}',(L,38,H-24),(cx,cy+s*(W/2-19),(H-24)/2),8,'manual_frame',stock='38 x 176 rim')
        b.box(f'manual_short_rim_{s}',(38,W-76,H-24),(cx+s*(L/2-19),cy,(H-24)/2),8,'manual_frame')
    for j,x in enumerate(np.linspace(cx-L/2+300,cx+L/2-300,4)):
        b.box(f'manual_joist_{j}',(38,W-76,H-24),(x,cy,(H-24)/2),8,'manual_frame',axis=(0,1,0))
    for i in range(2):
        b.box(f'manual_top_{i}',(1199.3,W-1,24),(cx-L/2+(i+.5)*1200,cy,H-12),6+i%2,'manual_skin',bevel=.5,explode=(0,0,300),stock='24 mm manual pad skin')
    for s in (-1,1):
        b.box(f'manual_edge_angle_top_{s}',(L,40,4),(cx,cy+s*(W/2-20),H-2),12,'manual_edges',bevel=.7)
        b.box(f'manual_edge_angle_side_{s}',(L,4,40),(cx,cy+s*(W/2-2),H-20),11,'manual_edges',bevel=.7)
    b.obstacles.append(dict(name='Manual pad',x0=cx-L/2,x1=cx+L/2,y0=cy-W/2,y1=cy+W/2,height_mm=H,kind='manual'))
    # Low grind ledge: framed cabinet, plywood sides and independent steel angles.
    cx,cy,L,W,H=2250.,-2300.,3200.,620.,360.
    for s in (-1,1):
        b.box(f'ledge_side_ply_{s}',(L,18,H-24),(cx,cy+s*(W/2-9),(H-24)/2),4,'ledge_skin',stock='18 mm plywood side')
        b.box(f'ledge_end_ply_{s}',(18,W-36,H-24),(cx+s*(L/2-9),cy,(H-24)/2),3,'ledge_skin')
    for j,x in enumerate(np.linspace(cx-L/2+80,cx+L/2-80,5)):
        b.box(f'ledge_internal_web_{j}',(38,W-36,H-24),(x,cy,(H-24)/2),8,'ledge_frame')
    b.box('ledge_top',(L,W,24),(cx,cy,H-12),6,'ledge_skin',bevel=.5,explode=(0,0,300),stock='24 mm ledge top')
    for s in (-1,1):
        b.box(f'ledge_angle_top_{s}',(L,50,5),(cx,cy+s*(W/2-25),H-2.5),12,'ledge_edges',bevel=1.)
        b.box(f'ledge_angle_face_{s}',(L,5,50),(cx,cy+s*(W/2-2.5),H-25),11,'ledge_edges',bevel=1.)
    b.text('ledge_brand','CYBR / DIY',(cx,cy-W/2-.4,170),100,15,'XZ')
    for i in range(9):
        x=cx-L/2+80+i*(L-160)/8
        for s in (-1,1):b.screw(f'ledge_screw_{i}_{s}',(x,cy+s*(W/2-65),H),group='ledge_hardware')
    b.obstacles.append(dict(name='Grind ledge',x0=cx-L/2,x1=cx+L/2,y0=cy-W/2,y1=cy+W/2,height_mm=H,kind='ledge'))
    # Flat bar: hollow steel, three transverse bases, actual anchor plates and bolts.
    cx,cy,L,H=2450.,-5400.,3000.,340.
    bar=(cq.Workplane('YZ').rect(50,50).rect(44,44).extrude(L).translate((cx-L/2,cy,H-25)).val())
    b.shape('flatbar_top',bar,12,'rail',role='50 x 50 x 3 mm nominal hollow steel grind bar')
    for end in (-1,1): b.box(f'flatbar_end_cap_{end}',(3,44,44),(cx+end*(L/2-1.5),cy,H-25),11,'rail',bevel=.5)
    for j,x in enumerate((cx-L/2+300,cx,cx+L/2-300)):
        b.box(f'flatbar_vertical_{j}',(40,40,H-57),(x,cy,(H-57)/2+7),11,'rail',bevel=2.)
        b.box(f'flatbar_foot_{j}',(110,700,7),(x,cy,3.5),11,'rail',bevel=2)
        for side in (-1,1):b.screw(f'flatbar_anchor_{j}_{side}',(x,cy+side*285,7),group='rail',r=6)
    b.obstacles.append(dict(name='Flat bar',x0=cx-L/2,x1=cx+L/2,y0=cy-350,y1=cy+350,height_mm=H,kind='rail'))
    # Low concrete slappy curb occupies the otherwise empty north-east zone.
    cx,cy,L=6350.,3800.,3200.
    b.box('slappy_curb',(L,330,120),(cx,cy,60),1,'curb',bevel=12.)
    b.box('slappy_waxed_edge',(L-18,34,3),(cx,cy-143,118.8),23,'curb',bevel=.5)
    b.obstacles.append(dict(name='Slappy curb',x0=cx-L/2,x1=cx+L/2,y0=cy-165,y1=cy+165,height_mm=120,kind='curb'))


def site(b:Builder):
    c=b.cfg;L=c.slab_length;W=c.slab_width
    b.box('site_compacted_base',(L+280,W+280,160),(0,0,-185),2,'site',bevel=15)
    # Narrow, flush joints between actual concrete panels. No painted-on grid.
    for i in range(6):
        for j in range(4):
            dx=L/6;dy=W/4;x=-L/2+(i+.5)*dx;y=-W/2+(j+.5)*dy
            b.box(f'concrete_pour_{i}_{j}',(dx-2.5,dy-2.5,110),(x,y,-55),int((i*3+j)%6==0),'slab',bevel=.6)
    b.box('flush_joint_backing',(L-1,W-1,3),(0,0,-3),22,'site',bevel=0)
    # Fence is outside the skateable slab. The open south edge gives camera/access clearance.
    for j,x in enumerate(np.arange(-L/2,L/2+1,2000)):
        b.box(f'fence_north_post_{j}',(95,95,1850),(x,W/2+220,800),13,'fence',axis=(0,0,1))
    for k,z in enumerate((350,1250)):
        b.box(f'fence_north_rail_{k}',(L+150,38,95),(0,W/2+186,z),13,'fence')
    for j,x in enumerate(np.arange(-L/2+40,L/2,135)):
        b.box(f'fence_north_slat_{j}',(124,18,1700),(x,W/2+155,755),13+j%2,'fence',bevel=1.2,axis=(0,0,1))
    # Short return fence on the right, without obstructing street approaches.
    for j,y in enumerate(np.arange(-7200,8001,2000)):
        b.box(f'fence_east_post_{j}',(95,95,1850),(L/2+220,y,800),13,'fence',axis=(0,0,1))
    for k,z in enumerate((350,1250)):
        b.box(f'fence_east_rail_{k}',(38,15400,95),(L/2+186,400,z),13,'fence',axis=(0,1,0))
    for j,y in enumerate(np.arange(-7200,8000,135)):
        b.box(f'fence_east_slat_{j}',(18,124,1700),(L/2+155,y,755),13+j%2,'fence',bevel=1.2,axis=(0,0,1))
    # A small branded yard plaque is real CAD typography, not a texture billboard.
    b.box('yard_sign_backing',(2700,38,530),(3300,W/2+108,1140),15,'sign',bevel=10.)
    b.text('yard_sign_text','CYBR YARD',(3300,W/2+85,1160),240,16,'XZ')
    b.text('yard_sign_subtext','BUILT TO SKATE',(3300,W/2+84,965),75,16,'XZ')
    # Bench outside the central return aisle.
    cx,cy=3000.,6650.
    for j in range(4):
        b.box(f'bench_seat_slat_{j}',(1900,95,38),(cx,cy+(j-1.5)*110,455),8,'bench',bevel=3.)
    for s in (-1,1):
        for q in (-1,1):b.box(f'bench_leg_{s}_{q}',(55,55,430),(cx+s*730,cy+q*145,215),11,'bench',bevel=3.)
        b.box(f'bench_crossbar_{s}',(70,450,45),(cx+s*730,cy,400),11,'bench')
    for k in range(2):
        b.box(f'bench_back_slat_{k}',(1900,35,135),(cx,cy+230,690+k*170),8,'bench',bevel=3.)
    for s in (-1,1):b.box(f'bench_back_post_{s}',(50,40,690),(cx+s*730,cy+250,540),11,'bench',bevel=2.)
    # Planters and actual thin leaf meshes keep scale without a photographic background.
    for j,(px,py) in enumerate(((6200,6900),(8600,6900))):
        b.box(f'planter_{j}_bottom',(1050,650,30),(px,py,15),13,'planter')
        for s in (-1,1):
            b.box(f'planter_{j}_long_{s}',(1050,38,410),(px,py+s*306,205),13,'planter')
            b.box(f'planter_{j}_short_{s}',(38,575,410),(px+s*506,py,205),14,'planter')
        b.box(f'planter_{j}_soil',(972,572,30),(px,py,365),17,'planter',bevel=2)
        vertices=[];faces=[]
        rng=np.random.default_rng(710+j)
        for n in range(110):
            x=px+rng.uniform(-450,450);y=py+rng.uniform(-240,240);z=380
            angle=rng.uniform(0,TAU);length=rng.uniform(190,650);lean=rng.uniform(.14,.65);wid=rng.uniform(6,15)
            tangent=np.array((math.cos(angle),math.sin(angle),0));side=np.array((-math.sin(angle),math.cos(angle),0))
            start=len(vertices)
            for k,t in enumerate(np.linspace(0,1,7)):
                mid=np.array((x,y,z))+tangent*(lean*length*t*t)+np.array((0,0,length*(t-.35*t*t)))
                w=wid*math.sin(math.pi*max(.01,min(.99,t)))
                vertices.extend([mid-side*w,mid+side*w])
                if k:faces.extend([[start+2*k-2,start+2*k-1,start+2*k+1],[start+2*k-2,start+2*k+1,start+2*k]])
        b.parts.append(mesh_part(f'planter_{j}_leaf_blades',np.asarray(vertices),np.asarray(faces),18+j,group='vegetation',provenance='designed-concept',role='Procedural leaf geometry; visual context'))


def skateboard(b:Builder):
    # A full-size board provides a readable scale cue next to the mini ramp.
    cx,cy,angle=-5200.,-300.,-.20;L=810.;W=208.
    outline=[]
    for a in np.linspace(-math.pi/2,math.pi/2,25):outline.append((L/2-W/2+W/2*math.cos(a),W/2*math.sin(a)))
    for a in np.linspace(math.pi/2,3*math.pi/2,25):outline.append((-L/2+W/2+W/2*math.cos(a),W/2*math.sin(a)))
    sh=cq.Workplane('XY').polyline(outline).close().extrude(11).translate((0,0,72)).val()
    sh=sh.rotate((0,0,0),(0,0,1),math.degrees(angle)).translate((cx,cy,0))
    b.shape('skateboard_maple_deck',sh,4,'skateboard',role='Full-size nominal skateboard scale reference')
    grip=cq.Workplane('XY').polyline([(x*.986,y*.975) for x,y in outline]).close().extrude(.7).translate((0,0,83.1)).val()
    grip=grip.rotate((0,0,0),(0,0,1),math.degrees(angle)).translate((cx,cy,0));b.shape('skateboard_griptape',grip,21,'skateboard')
    def p(x,y,z):return(cx+x*math.cos(angle)-y*math.sin(angle),cy+x*math.sin(angle)+y*math.cos(angle),z)
    for j,x in enumerate((-255,255)):
        b.tube(f'skateboard_axle_{j}',p(x,-103,29),p(x,103,29),5,10,'skateboard')
        b.tube(f'skateboard_kingpin_{j}',p(x,0,34),p(x+12,0,72),8,10,'skateboard')
        for side in (-1,1):
            b.tube(f'skateboard_wheel_{j}_{side}',p(x,side*86,29),p(x,side*114,29),28,20,'skateboard',wall=19,sides=48)
            b.tube(f'skateboard_bearing_{j}_{side}',p(x,side*111,29),p(x,side*114,29),9,10,'skateboard',wall=4,sides=32)
        for xx in (-24,24):
            for yy in (-20,20):b.screw(f'skateboard_bolt_{j}_{xx}_{yy}',p(x+xx,yy,84),group='skateboard',r=3)


def build(config:ParkConfig|None=None)->Assembly:
    cfg=config or ParkConfig();cfg.check();b=Builder(cfg)
    site(b);mini=mini_ramp(b);bank(b);street(b)
    q=quarter(b,'street_quarter',7450.,-4150.,840.,2100.,3600.,760.,1,0.,True)
    b.obstacles.append(dict(name='Quarter pipe',x0=7290,x1=7450+q['run_mm']+760,y0=-5950,y1=-2350,height_mm=840,kind='transition'))
    skateboard(b)
    shared=dict(studio_style='outdoor',studio_target=(0,0,400),studio_scale=110.,studio_az=230.,studio_el=36.,
                light_size=1.,light_intensity=1.,environment_strength=1.65,background_strength=1.,
                background_color=(.36,.46,.57),floor=True,floor_z_mm=-275.,floor_color=(.32,.33,.30),
                floor_roughness=.96,f_stop=11.,tone_mapping='neutral',exposure=1.42)
    views={
        'hero':View(az=231,el=35,scale=10500,target=(0,-150,550),focal_length_mm=50,title='CYBR YARD / DIY SKATEPARK',**shared),
        'street':View(az=212,el=21,scale=4100,target=(1400,-3800,400),focal_length_mm=38,title='THE STREET LINE',**shared),
        'mini':View(az=230,el=24,scale=3850,target=(-4950,3100,620),focal_length_mm=48,title='THE MINI RAMP',**shared),
        'coping':View(az=237,el=26,scale=1200,target=(-1850,1850,1030),focal_length_mm=62,title='PLYWOOD / COPING / FRAMING',**shared),
        'overview':View(az=270,el=89.99,scale=9000,target=(0,0,0),projection='orthographic',title='NOMINAL LAYOUT / 22 x 16 m',**shared),
        'structure':View(az=232,el=29,scale=3500,target=(-5000,3000,400),focal_length_mm=48,
            hide=('mini_west_skin','mini_east_skin','mini_flat_skin','mini_west_screws','mini_east_screws','mini_flat_screws','mini_west_fascia','mini_east_fascia','mini_flat_fascia'),
            title='REAL TIMBER FRAMING / SKINS REMOVED',**shared),
    }
    metadata=dict(title='CYBR YARD / DIY skatepark',source_repository='cybrdelic/cybr-geo',
        source_commit='ce83e1dc06a1258e524c8af82e298e82fc223a6d',source_snapshot='e8e5d17c31ce4febe5b8ed805661cfe3df47fdb4',
        geometry_method='CYBR GEO named analytic CAD and procedural mesh assembly',
        renderer='CYBR GEO V9 native BVH/GGX/MIS path tracer with explicitly selected outdoor lighting and finishes',
        no_image_generation=True,units='mm',config=cfg.__dict__,transitions=[*mini,q],obstacles=b.obstacles,
        cutlist=b.cutlist,limitations=[
            'Concept geometry only; no structural, anchorage, fatigue, drainage, traction, fall-zone or code qualification.',
            'Plywood bend capability, support sizing and fastener specification require review before physical construction.',
            'Render materials are authored RGB appearance, not measured optics; not spectral transport.',
            'Vegetation is open visual mesh geometry. Nominal screw heads do not constitute resolved load-bearing screw joints.',
            'The site is an invented 22 x 16 m layout, not a survey of the user property.',
        ])
    return Assembly('cybr_yard_diy_skatepark',b.parts,MATERIALS,views,metadata)

if __name__=='__main__':
    a=build();print(a.name,len(a.parts),sum(len(p.faces) for p in a.parts))
