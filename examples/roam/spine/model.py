"""Shared-spine workstation. mm, Z up. Explicit driven CAD mates.

This is nominal design CAD, not a catalog-arm facsimile or a load certification.
Pivot fits derive from one interface. All fasteners are removable separately.
"""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='4'
from pathlib import Path
import sys, json, math, hashlib, copy
import numpy as np
import cadquery as cq
from OCP.gp import gp_Trsf
OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sys.path.insert(0,str(ROOT/'work/cybr-geo/src'))
from mechanism_lab import Assembly, Material, View
from mechanism_lab.core import cad_part, pose_cad, save_cache
from mechanism_lab.exporters import export_step, export_glb, export_bom

def T(x=0,y=0,z=0):
    a=np.eye(4);a[:3,3]=[x,y,z];return a
def R(axis,deg):
    a=np.eye(4);r=math.radians(deg);c,s=math.cos(r),math.sin(r)
    if axis=='x':a[:3,:3]=[[1,0,0],[0,c,-s],[0,s,c]]
    if axis=='y':a[:3,:3]=[[c,0,s],[0,1,0],[-s,0,c]]
    if axis=='z':a[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]]
    return a
def loc(a):
    t=gp_Trsf();t.SetValues(*map(float,a[:3,:4].ravel()));return cq.Location(t)
def matrix(location):
    t=location.wrapped.Transformation();a=np.eye(4)
    for i in range(3):
        for j in range(4):a[i,j]=t.Value(i+1,j+1)
    return a
def box(x,y,z,c=(0,0,0)):return cq.Workplane('XY').box(x,y,z).val().translate(c)
def cyl(r,a,b,inner=0):
    s=cq.Solid.makeCylinder(r,b-a,cq.Vector(0,0,a))
    return s.cut(cq.Solid.makeCylinder(inner,b-a,cq.Vector(0,0,a))) if inner else s
def holes(s,points,r,a,b):
    for x,y in points:s=s.cut(cyl(r,a,b).translate((x,y,0)))
    return s.clean()
def capsule(length,width,thick):
    return cq.Workplane('XY').moveTo(length/2,0).slot2D(length+width,width).extrude(thick).val().translate((0,0,-thick/2))
def tube(w,d,h):return box(w,d,h,(0,0,h/2)).cut(box(w-6,d-6,h+2,(0,0,h/2))).clean()
def hhex(d,a,b,inner):return cq.Workplane('XY').polygon(6,d).extrude(b-a).val().translate((0,0,a)).cut(cyl(inner,a-1,b+1))

PIVOT={'plate_mm':16.,'bore_mm':14.,'shaft_mm':12.,'installed_bush_id_mm':12.05,
       'bush_od_mm':14.,'bush_body_mm':16.,'flange_od_mm':20.,'flange_mm':1.,
       'layer_pitch_mm':18.,'part':'igus GFM-1214-17',
       'source':'https://www.igus.com/contentData/Product_Files/Download/pdf/2016%20iglide%20section.pdf'}
PLATE=PIVOT['plate_mm'];HALF=PLATE/2;BORE=PIVOT['bore_mm']/2;PITCH=PIVOT['layer_pitch_mm']
# ISO 7380-1 maximum head diameter/height and hex drive dimensions. The head
# cylinder is a conservative clearance envelope; the domed profile is omitted.
BUTTON_HEAD={2:(7.6,2.2,2.5,1.3),2.5:(9.5,2.75,3.,1.56),3:(10.5,3.3,4.,2.08),4:(14.,4.4,5.,2.6),5:(17.5,5.5,6.,3.12)}
BUTTON_SOURCE='https://bossard.partcommunity.com/3d-cad-models/?info=bossard%2F01%2F01_100%2F01_100_100%2F01_100_100_10%2Fbn_1593_8699%2Fbn_1593.prj'
def head_clearance(radius):return BUTTON_HEAD[radius][0]/2+.25
MAT=[Material('Graphite aluminium',(.08,.105,.12),.75,.3),Material('Clear anodized aluminium',(.47,.50,.52),.8,.27),
     Material('Steel hardware',(.33,.36,.39),.95,.24),Material('Purchased polymer bushing',(.12,.085,.045),0,.52),
     Material('Rubber',(.016,.019,.022),0,.65),Material('Warm grey tray',(.36,.33,.28),0,.43),
     Material('Screen envelope',(.025,.04,.055),.1,.25),Material('Orange identification',(.7,.19,.055),.25,.38)]

class BaseModel:
    def __init__(self):
        self.parts=[];self.frames=[{'name':'base','parent':None}];self.connections=[];self.mates=[]
    def add(self,name,s,group='base',mat=0,kind='original cut plate',parent=None,note=''):
        if not s.isValid() or len(s.Solids())!=1:raise ValueError(f'{name}: invalid or disconnected manufactured body')
        self.parts.append({'name':name,'shape':s,'group':group,'mat':mat,'kind':kind,'note':note})
        if parent:self.connections.append({'a':name,'b':parent,'kind':note or 'bolted or seated interface'})
    def frame(self,name,parent,offset,axis=None,channel=None):
        self.frames.append({'name':name,'parent':parent,'offset':offset.tolist(),'axis':axis,'channel':channel})
    def screw(self,name,where,group,length=30,r=4,parent=None):
        # Conventional simplified thread representation: major cylinder, no helical teeth.
        diameter,height,af,depth=BUTTON_HEAD[r]
        s=cyl(r,-length,0).fuse(cyl(diameter/2,0,height)).clean()
        s=s.cut(hhex(af*2/math.sqrt(3),height-depth,height+.1,0))
        self.add(name,pose_cad(s,where),group,2,'ISO 7380-1 nominal screw envelope',parent,'supplier-derived maximum head/drive dimensions; domed head conservatively cylindrical; thread teeth omitted; final grade and length availability pending')
    def tap_screw(self,name,targets,where,group,length=30,r=3):
        # Standard CAD simplification: tapping represented at nominal major diameter.
        # Actual thread form/fit class belongs on the drawing and supplier order.
        cutter=pose_cad(cyl(r,-length,0),where)
        for target in targets:
            p=next(p for p in self.parts if p['name']==target)
            p['shape']=p['shape'].cut(cutter).clean()
        self.screw(name,where,group,length,r,targets[0])
        self.connections.append({'a':name,'b':targets[-1],'kind':'nominal tapped screw; cosmetic threads, no thread-force qualification'})
    def pivot_hardware(self,name,group,parent=None):
        # Child plate -8..8; parent plate -26..-10. Shoulder -30..20 (50 mm).
        bush=cyl(PIVOT['bush_od_mm']/2,-HALF,HALF,PIVOT['installed_bush_id_mm']/2).fuse(cyl(PIVOT['flange_od_mm']/2,HALF,HALF+PIVOT['flange_mm'],PIVOT['installed_bush_id_mm']/2)).clean()
        self.add(name+'_bush',bush,group,3,'dimensioned purchased bushing',parent,'GFM-1214-17 installed boundary; chamfers omitted')
        self.add(name+'_thrust',cyl(18,-10,-8,6.15),group,3,'cut thrust washer',parent)
        self.add(name+'_top_spacer',cyl(10,9,20,6.15),group,2,'purchased spacer; cut length',parent)
        self.add(name+'_bottom_spacer',cyl(10,-30,-26,6.15),group,2,'purchased spacer; cut length',parent)
        bolt=cyl(6,-30,20).fuse(cyl(9,20,29)).fuse(cyl(5,-46,-30)).clean()
        bolt=bolt.cut(hhex(6.93,25,30,0))
        self.add(name+'_shoulder_screw',bolt,group,2,'norelem 07534-112X50 boundary',parent,'12 x 50 shoulder, M10 x 16 thread; 18 x 9 head, 6 hex; nominal thread')
        self.add(name+'_locknut',hhex(19.63,-38,-30,5),group,2,'nominal M10 nut',name+'_shoulder_screw','8 mm nominal thread engagement; axial retention')

    def poses(self,q,solve=False):
        frames={'base':np.eye(4)};receipts=[]
        for f in self.frames[1:]:
            rel=np.array(f['offset']);channel=f['channel']
            if channel:
                val=q[channel[0]][channel[1]]
                rel=(rel@T(**{f['axis']:val})) if channel[1]==0 else (rel@R(f['axis'],val))
            if solve:
                # Three non-collinear point mates are a driven datum frame. Their
                # points are shared with feature axes and seating planes above.
                parent_parts=[p['shape'] for p in self.parts if p['group']==f['parent']]
                child_parts=[p['shape'] for p in self.parts if p['group']==f['name']]
                a=cq.Assembly(name='mate_'+f['name'])
                a.add(cq.Compound.makeCompound(parent_parts),name='parent')
                a.add(cq.Compound.makeCompound(child_parts),name='child',loc=loc(T(1,2,3)@rel))
                a.constrain('parent','Fixed')
                for p in [(0,0,0),(30,0,0),(0,30,0)]:
                    target=(rel@np.r_[p,1])[:3]
                    a.constrain('parent',cq.Vertex.makeVertex(*target),'child',cq.Vertex.makeVertex(*p),'Point')
                a.solve();actual=matrix(a.objects['child'].loc)
                err=max(float(np.linalg.norm((actual@np.r_[p,1])[:3]-(rel@np.r_[p,1])[:3])) for p in [(0,0,0),(30,0,0),(0,30,0)])
                if err>.001 or not a._solve_result['success']:raise ValueError('Unsolved mate '+f['name'])
                receipts.append({'frame':f['name'],'parent':f['parent'],'type':'three-point driven frame mate','axis':f['axis'],'coordinate':channel,'error_mm':err,'solved_transform':actual.tolist()})
                rel=actual
            frames[f['name']]=frames[f['parent']]@rel
        return frames,receipts

MAT.append(Material('Printed PA12 guide cover',(.085,.15,.145),0,.55))
DEFAULT={'base':[360],'left':[1110,140,-40,-100,0,0],'right':[1110,40,40,-80,0,0],'tray':[590,80,20,-100,0,0]}
AXES=['height','shoulder','elbow','head_swivel','tilt','roll']
RANGES={'base':[[0,360]],'left':[[850,1350],[55,170],[-180,180],[-180,180],[-35,35],[-90,90]],'right':[[850,1350],[10,125],[-180,180],[-180,180],[-35,35],[-90,90]],'tray':[[450,950],[20,160],[-180,180],[-180,180],[-25,85],[-20,20]]}
POSES={'Working':DEFAULT,'Standing':dict(DEFAULT,left=[1330,140,-40,-100,0,0],right=[1330,40,40,-80,0,0],tray=[900,80,20,-100,-5,0]),'Low seat':dict(DEFAULT,left=[870,145,-40,-105,-12,0],right=[870,35,40,-75,-12,0],tray=[450,80,20,-100,-10,0]),'Portrait':dict(DEFAULT,left=[1190,140,-40,-100,0,90],right=[1150,40,40,-80,0,-90]),'Tray bank':dict(DEFAULT,tray=[590,85,15,-100,-10,12]),'Rolling':{'base':[180],'left':[1240,140,-100,-40,0,90],'right':[1240,40,100,-140,0,-90],'tray':[450,20,110,-130,80,0]}}
SIDES=['left','right','tray']
LENGTHS={'left':(280,260),'right':(280,260),'tray':(260,240)}
# Local face coordinates: u runs across the rail, v points out of the spine.
FACES={'left':T(-70,0,0)@R('z',90),'right':T(70,0,0)@R('z',-90),'tray':T(0,50,0)}
RAIL={'width':20,'height':17.5,'pitch':60,'block_width':44,'block_length':77.5,'block_height':30,'block_pitch':[32,36],'source':'https://www.hiwin.com/wp-content/uploads/Linear_Guideway-E.pdf'}
GAS={'body_diameter':28,'rod_diameter':10,'stroke':500,'status':'required package envelope, no selected configuration or invented internal parts','reference_family':'Bansbach lockable; final force, body length and release fitting must be configured','source':'https://www.bansbach.com/en/products/gas-springs/lockable-gas-springs/main-type-k/'}

BRAKE={'component':'Zimmer HK2001A + PHK20-1','holding_N':1200,'tightening_Nm':7,'mass_kg':.26,'source':'https://www.zimmer-group.com/swk-datasheet/HIW-HGQH-20-HGWHA-HK2001A-EN.pdf','scope':'HG20 rail drawing; schematic outline and mounting interface, not manufacturer internal CAD'}
def sector(r1,r2,a,b,h):
    p=lambda r,t:(r*np.cos(np.radians(t)),r*np.sin(np.radians(t)))
    return cq.Workplane('XY').moveTo(*p(r2,a)).threePointArc(p(r2,(a+b)/2),p(r2,b)).lineTo(*p(r1,b)).threePointArc(p(r1,(a+b)/2),p(r1,a)).close().extrude(h).val().translate((0,0,-h/2))
def arc_slot(r,a,b,h,rad=4.3):
    s=sector(r-rad,r+rad,a,b,h)
    for t in (a,b):s=s.fuse(cyl(rad,-h/2,h/2).translate((r*np.cos(np.radians(t)),r*np.sin(np.radians(t)),0)))
    return s.clean()
def rounded(w,d,h,c=(0,0,0),r=2):
    return cq.Workplane(obj=box(w,d,h,c)).edges('|Z').fillet(r).val()
def hollow(w,d,h,t,c=(0,0,0)):
    a=cq.Workplane(obj=box(w,d,h,c)).edges('|Z').fillet(t+1).val()
    b=cq.Workplane(obj=box(w-2*t,d-2*t,h+2,c)).edges('|Z').fillet(1).val()
    return a.cut(b).clean()
def along_x(length,w,h,t,start):
    a=cq.Workplane(obj=box(length,w,h,(start+length/2,0,0))).edges('|X').fillet(t+1).val()
    b=cq.Workplane(obj=box(length+2,w-2*t,h-2*t,(start+length/2,0,0))).edges('|X').fillet(1).val()
    return a.cut(b).clean()
def eye(thickness=16):
    # Supplier-cut original end fitting: round eye and tongue, one flat blank.
    s=cyl(27,-thickness/2,thickness/2).fuse(rounded(72,32,thickness,(48,0,0),3)).clean()
    return holes(s,[(0,0)],7,-20,20)

class Model(BaseModel):
    def get(self,n):return next(p for p in self.parts if p['name']==n)
    def cut(self,n,s):self.get(n)['shape']=self.get(n)['shape'].cut(s).clean()
    def component(self,name,s,g='base',mat=0,kind='original supplier-cut part',parent='spine',note=''):
        self.add(name,s,g,mat,kind,parent,note)
    def bolts(self,stem,targets,placements,g='base',r=3,length=25):
        for i,A in enumerate(placements):self.tap_screw(stem+'_'+str(i),targets,A,g,length,r)
    def wheel(self,name,x,y,g):
        # An opaque purchased-unit package is deliberately not fabricated internals.
        plate=holes(rounded(100,100,8,(x,y,107),5),[(x+a,y+b) for a in (-40,40) for b in (-40,40)],4.3,100,115)
        plate=plate.cut(cyl(6.2,100,115).translate((x,y,0)))
        self.component(name+'_adapter',plate,g,2,parent=f'base_outer_leg_{1 if x>0 else -1}' if g=='base' else f'base_inner_leg_{1 if x>0 else -1}',note='Original caster adapter; supplier pattern must be selected before fabrication')
        wh=pose_cad(cyl(48,-16,16),T(x,y,48)@R('y',90))
        self.component(name+'_wheel_envelope',wh,g,4,'unselected 96 mm caster wheel envelope',name+'_adapter')
        fork=box(44,50,57,(x,y,74.5)).cut(box(34,54,51,(x,y,71.5))).fuse(cyl(23,96,103).translate((x,y,0))).fuse(cyl(6,103,138).translate((x,y,0))).clean()
        self.component(name+'_fork_envelope',fork,g,1,'required M12 x 35 stem caster envelope',name+'_adapter',note='Unselected purchased unit; external stem interface required, internal bearing/brake not invented')
    def guard(self,side):
        # Two printable half-covers protect guide edges; structural metal stays separate.
        def loft(inset):
            q=cq.Workplane('XY',origin=(0,44,-198)).ellipse(45-inset,19-inset)
            for dz,w,d in [(38,57,24),(220,57,24),(30,45,20)]:q=q.workplane(offset=dz).ellipse(w-inset,d-inset)
            return q.loft(combine=True).val()
        s=loft(0).cut(loft(2)).cut(box(300,200,1000,(0,-63.5,0)))
        s=s.cut(cyl(16,-500,700).translate((36,64,0)))
        s=s.cut(box(200,400,1000,(120,0,0))) # open gas side; one continuous guide skin
        s=s.cut(box(72,200,70,(0,130,0)))
        s=s.cut(box(100,150,50,(-98,75,-60)))
        for u in (-43,-10):
            for z in (-110,65):s=s.fuse(pose_cad(cyl(5,36.5,70,2.2),T(u,0,z)@R('x',-90)))
        s=s.clean()
        for title,a,b in [('lower',-250,-55),('upper',-55,120)]:
            piece=s.intersect(box(400,400,b-a,(0,0,(a+b)/2)))
            self.component(side+'_guide_guard_'+title,piece,side+'_carriage',8,'printed nonstructural hollow loft; removable',side+'_carriage_plate','2 mm wall, open rear, gas and shoulder clearances; original printable cover')
        for u in (-43,-10):
            for z in (-110,65):
                targets=[side+'_carriage_plate',side+'_guide_guard_'+('lower' if z<0 else 'upper')]
                self.bolts(f'{side}_guard_fix_{u}_{z}',targets,[T(u,70,z)@R('x',-90)],side+'_carriage',2,38)
    def brake(self,side):
        g=side+'_carriage';body=box(60,20,24,(-18,19,-60)).cut(box(20.1,21,26,(-18,9.5,-60)))
        self.component(side+'_rail_clamp_envelope',body,g,1,'Zimmer HK2001A schematic exterior',side+'_rail_envelope',BRAKE['source'])
        self.component(side+'_brake_spacer',box(60,1,24,(-18,29.5,-60)),g,2,'PHK20-1 1 mm height-compensation outline',side+'_rail_clamp_envelope')
        self.bolts(side+'_brake_fix',[side+'_carriage_plate',side+'_brake_spacer',side+'_rail_clamp_envelope'],[T(-18+u,36,-60+z)@R('x',-90) for u in (-7.5,7.5) for z in (-7.5,7.5)],g,2.5,12)
        handle=cq.Workplane('XY').moveTo(12,6.5).lineTo(30,6.5).spline([(42,30),(53.5,76)],includeCurrent=True).lineTo(46,76).spline([(37,30),(27,19.5)],includeCurrent=True).lineTo(12,19.5).close().extrude(8).val().translate((0,0,-64))
        handle=pose_cad(handle,T(-18,0,-60)@R('y',180)@T(18,0,60))
        self.component(side+'_brake_lever_envelope',handle,g,7,'Zimmer lever schematic clearance envelope',side+'_rail_clamp_envelope','Clamp installed 180 degrees around its mounting normal; 41.5 mm projection / 63 mm rise; exact profile and re-indexing sweep require manufacturer CAD')
    def beam(self,side,j,L,h):
        g=side+'_arm'+str(j);root=side+f'_link_{j}_root';end=side+f'_link_{j}_end';tube_name=side+f'_closed_link_{j}'
        self.component(root,eye(),g,1,parent=side+'_shoulder_shelf' if j==1 else side+'_link_1_end')
        e=pose_cad(eye(),T(L,0,0)@R('z',180));e=holes(e,[(L,0)],6.1,-20,20)
        # The parent end is a clearance bore; the next child receives the bushing.
        e=eye().fuse(cyl(7,-8,8)).cut(cyl(6.1,-10,10));e=pose_cad(e,T(L,0,0)@R('z',180))
        self.component(end,e,g,1,parent=tube_name)
        tube_shape=along_x(L-110,60,h,2,55)
        self.component(tube_name,tube_shape,g,0,f'60 x {h} x 2 mm closed aluminium tube',root,'Closed section carries bending and torsion; original end tongues are bolted inside, not fused into the tube')
        for k,x in enumerate((65,80,L-80,L-65)):
            owner=root if k<2 else end
            for sign in (-1,1):
                a,b=sorted((sign*8,sign*(h/2-2)))
                self.component(f'{side}_link{j}_crush_{k}_{sign}',cyl(6,a,b,3.2).translate((x,0,0)),g,2,'cut compression spacer',owner)
            self.bolts(f'{side}_link{j}_fix{k}',[tube_name,owner],[T(x,0,h/2)],g,3,int(h+10))
            self.component(f'{side}_link{j}_nut{k}',hhex(11.547,-h/2-5,-h/2,3).translate((x,0,0)),g,2,'nominal M6 nut',tube_name)
        self.pivot_hardware(side+('_shoulder' if j==1 else '_elbow'),g,root)
        # Nonstructural edge guards leave the whole closed metal section visible.
        for k,x in enumerate((56,L-56)):
            guard=box(2,64,h+4,(x,0,0)).cut(box(4,60.2,h+.2,(x,0,0)))
            self.component(f'{side}_link{j}_edge_guard{k}',guard,g,7,'printed nonstructural edge guard',tube_name)
    def head(self,side,L):
        pitch_height=110 if side=='tray' else 70
        g=side+'_head';self.frame(g,side+'_arm2',T(L,0,18),'z',(side,3))
        shelf=eye().fuse(cyl(7,-8,8)).cut(cyl(7,-10,10))
        shelf=pose_cad(shelf,R('z',90))
        self.component(side+'_head_floor',shelf,g,1,parent=side+'_link_2_end')
        self.pivot_hardware(side+'_swivel',g,side+'_head_floor')
        # Two compact cut cheeks bolt onto the root tongue, not a large box gimbal.
        for sign in (-1,1):
            profile=cq.Workplane('YZ').moveTo(15,14).lineTo(68,14).lineTo(62,pitch_height+2).threePointArc((35,pitch_height+25),(15,pitch_height+2)).close().extrude(6).val().translate((sign*45-3,0,0))
            profile=profile.cut(pose_cad(cyl(6.1,-60,60),T(0,35,pitch_height)@R('y',90)))
            if side=='tray':
                sec=sector(62,88,150,270,6).fuse(box(54,18,6,(-47,0,0)))
                sec=sec.cut(arc_slot(75,155,265,10))
                rot=np.array([[0,0,1,sign*45],[1,0,0,35],[0,1,0,pitch_height],[0,0,0,1]],float)
                profile=profile.fuse(pose_cad(sec,rot)).cut(pose_cad(arc_slot(75,155,265,10),rot)).clean()
            self.component(side+f'_fixed_cheek_{sign}',profile,g,0,parent=side+'_head_bridge')
        bridge=rounded(100,52,6,(0,41,11),4)
        self.component(side+'_head_bridge',bridge,g,0,parent=side+'_head_floor')
        self.bolts(side+'_bridge_fix',[side+'_head_bridge',side+'_head_floor'],[T(0,y,14) for y in (28,57)],g,3,24)
        for sign in (-1,1):
            self.bolts(side+f'_fork_fix_{sign}',[side+'_head_bridge',side+f'_fixed_cheek_{sign}'],[T(sign*45,y,8)@R('x',180) for y in (25,57)],g,2.5,24)
        self.frame(side+'_pitch',g,T(0,35,pitch_height),'x',(side,4))
        for sign in (-1,1):
            A=T(sign*32,0,0)@R('y',90*sign)
            s=cq.Workplane('YZ').circle(22).extrude(16).val().translate((sign*32-8,0,0))
            s=s.fuse(box(16,44,36,(sign*32,22,0))).clean().cut(pose_cad(cyl(7,-9,9),A))
            if side=='tray':s=s.fuse(box(16,24,60,(sign*32,32,0))).fuse(box(16,100,24,(sign*32,-40,0))).clean()
            s=s.cut(pose_cad(cyl(7,-9,9),A))
            self.component(side+f'_pitch_cheek_{sign}',s,side+'_pitch',1,parent=side+f'_fixed_cheek_{sign}')
            self.component(side+f'_pitch_bush_{sign}',pose_cad(cyl(7,-8,8,6.025).fuse(cyl(10,8,9,6.025)),A),side+'_pitch',3,'igus GFM-1214-17 sourced envelope',side+f'_pitch_cheek_{sign}')
            self.component(side+f'_pitch_thrust_{sign}',pose_cad(cyl(18,9,10,6.15),A),side+'_pitch',3,'original thrust washer',side+f'_pitch_cheek_{sign}')
            self.component(side+f'_pitch_outer_spacer_{sign}',pose_cad(cyl(10,16,20,6.15),A),side+'_pitch',2,'cut spacer',side+f'_fixed_cheek_{sign}')
            self.component(side+f'_pitch_inner_spacer_{sign}',pose_cad(cyl(10,-10,-8,6.15),A),side+'_pitch',2,'cut spacer',side+f'_pitch_cheek_{sign}')
            bolt=cyl(6,-10,20).fuse(cyl(9,20,29)).fuse(cyl(5,-26,-10)).cut(hhex(6.93,25,30,0)).clean()
            self.component(side+f'_pitch_bolt_{sign}',pose_cad(bolt,A),side+'_pitch',2,'norelem 12 x 30 shoulder screw envelope',side+f'_fixed_cheek_{sign}')
            self.component(side+f'_pitch_nut_{sign}',pose_cad(hhex(19.63,-18,-10,5),A),side+'_pitch',2,'nominal M10 nut',side+f'_pitch_bolt_{sign}')
        back=rounded(80,100 if side=='tray' else 70,16,(0,0,0),5);back=holes(back,[(0,0)],6.1,-10,10)
        if side=='tray':
            for start in (-110,70):back=back.cut(arc_slot(40,start,start+40,20))
        back=pose_cad(back,T(0,52,0)@R('x',-90))
        self.component(side+'_roll_support',back,side+'_pitch',0,parent=side+'_pitch_cheek_1')
        for x in (-32,32):
            for z in ((-24,24) if side=='tray' else (-10,10)):self.cut(side+'_roll_support',pose_cad(cyl(5,57.2,60.1),T(x,0,z)@R('x',-90)))
        self.bolts(side+'_cradle_fix',[side+'_roll_support']+[side+f'_pitch_cheek_{s}' for s in (-1,1)],[T(x,57.2,z)@R('x',-90) for x in (-32,32) for z in ((-24,24) if side=='tray' else (-10,10))],side+'_pitch',2.5,30)
        if side=='tray':
            for sign in (-1,1):
                A=T(sign*32,-75,0)@R('y',90*sign)
                self.cut(side+f'_pitch_cheek_{sign}',pose_cad(cyl(4.3,-9,9),A))
                self.component(f'tray_pitch_lock_washer{sign}',pose_cad(cyl(12,8,10,4.3),A),side+'_pitch',3,'original friction washer',side+f'_pitch_cheek_{sign}')
                self.component(f'tray_pitch_lock_outer_washer{sign}',pose_cad(cyl(9,16,18,4.3),A),side+'_pitch',2,'steel washer envelope',side+f'_fixed_cheek_{sign}')
                self.screw(f'tray_pitch_lock_screw{sign}',A@T(z=18),side+'_pitch',40,4,side+f'_pitch_cheek_{sign}')
                self.component(f'tray_pitch_lock_nut{sign}',pose_cad(hhex(15.01,-14.5,-8,4),A),side+'_pitch',2,'nominal M8 locking nut',f'tray_pitch_lock_screw{sign}')
        self.frame(side+'_roll',side+'_pitch',T(0,70,0)@R('x',-90),'z',(side,5))
        face=rounded(126,126,16,r=14)
        # Four relieved quadrants leave broad load paths to both VESA patterns.
        for x,y in [(-1,0),(1,0),(0,-1),(0,1)]:
            cutter=rounded(35 if x else 46,46 if x else 35,20,(x*62,y*62,0),8);face=face.cut(cutter)
        face=holes(face,[(0,0)],7,-10,10)
        face=holes(face,[(x,y) for b in (37.5,50) for x in (-b,b) for y in (-b,b)],2.25,-10,10)
        if side=='tray':face=holes(cyl(60,-8,8),[(0,0)],7,-10,10)
        self.component(side+'_vesa_plate',face,side+'_roll',1,'original relieved VESA 75/100 mounting blank',side+'_roll_support')
        self.pivot_hardware(side+'_roll_joint',side+'_roll',side+'_vesa_plate')
        if side!='tray':
            for i,(x,y) in enumerate([(a,b) for a in (-50,50) for b in (-50,50)]):self.component(side+f'_vesa_spacer{i}',cyl(5,8,36,2.25).translate((x,y,0)),side+'_roll',2,'cut stand-off',side+'_vesa_plate')
            screen=rounded(610,360,38,(0,0,55),7);screen=holes(screen,[(x,y) for x in (-50,50) for y in (-50,50)],2.25,35,44)
            self.component(side+'_monitor_envelope',screen,side+'_roll',6,'610 x 360 x 38 mm device envelope; 6 kg assumed',side+'_vesa_plate','Device appearance, connectors and screw depth remain unknown')
        else:
            # The tray has a compact central roll plate, not protruding VESA lobes.
            self.cut('tray_vesa_plate',box(300,60,30,(0,-79,0)))
            for y in (-40,40):
                self.cut('tray_vesa_plate',cyl(4.3,-9,9).translate((0,y,0)))
                self.component(f'tray_roll_lock_washer{y}',cyl(10,-10,-8,4.3).translate((0,y,0)),side+'_roll',3,'original friction washer','tray_roll_support')
                self.screw(f'tray_roll_lock_screw{y}',T(0,y,8),side+'_roll',45,4,'tray_vesa_plate')
                self.component(f'tray_roll_lock_nut{y}',hhex(15.01,-32.5,-26,4).translate((0,y,0)),side+'_roll',2,'nominal M8 locking nut',f'tray_roll_lock_screw{y}')
            for sign in (-1,1):
                web=cq.Workplane('YZ').polyline([(-37,20),(45,20),(45,48),(-27,270),(-37,270)]).close().extrude(6).val().translate((sign*38-3,0,0))
                self.component('tray_web_'+str(sign),web,side+'_roll',0,parent='tray_vesa_plate')
                for y in (-25,25):self.component(f'tray_root_spacer_{sign}_{y}',cyl(6,8,20,2.6).translate((sign*38,y,0)),side+'_roll',1,'cut spacer','tray_vesa_plate')
                for y in (-25,25):self.cut('tray_vesa_plate',cyl(5,-8.1,-5.2).translate((sign*38,y,0)))
                self.bolts('tray_web_fix'+str(sign),['tray_vesa_plate','tray_web_'+str(sign)],[T(sign*38,y,-5.2)@R('x',180) for y in (-25,25)],side+'_roll',2.5,35)
            for i,z in enumerate((80,240)):
                beam=box(600,12,30,(0,-43,z)).cut(box(602,8,26,(0,-43,z)))
                self.component('tray_crossbeam_'+str(i),beam,side+'_roll',1,'30 x 12 x 2 closed aluminium tube','tray_web_1')
            panel=box(720,12,330,(0,-55,160));panel=cq.Workplane(obj=panel).edges('|Y').fillet(22).val()
            self.component('keyboard_mouse_tray',panel,side+'_roll',5,'12 mm supplier-cut birch plywood, rounded edge','tray_crossbeam_0')
            for i,z in enumerate((80,240)):
                self.bolts('tray_panel_fix'+str(i),['keyboard_mouse_tray','tray_crossbeam_'+str(i)],[T(x,-61,z)@R('x',90) for x in (-230,230)],side+'_roll',2,20)
                for x in (-38,38):
                    self.cut('keyboard_mouse_tray',pose_cad(cyl(5,58,61.1),T(x,0,z)@R('x',90)))
                    self.component(f'tray_cross_crush_{i}_{x}',pose_cad(cyl(5,39,47,2.6),T(x,0,z)@R('x',90)),side+'_roll',1,'cut compression sleeve','tray_crossbeam_'+str(i))
                self.bolts('tray_cross_fix'+str(i),['keyboard_mouse_tray','tray_crossbeam_'+str(i),'tray_web_-1','tray_web_1'],[T(x,-58,z)@R('x',90) for x in (-38,38)],side+'_roll',2.5,40)

    def build(self):
        # A shared closed spine, hollow structural legs, and positive extension pins.
        spine=hollow(140,100,1330,3,(0,0,835))
        self.component('spine',spine,parent=None,kind='140 x 100 x 3 aluminium tube; cut and drilled',note='Shared load path for all three independent guides')
        self.frame('base_extension','base',T(),'y',('base',0))
        for sign in (-1,1):
            x=sign*280
            outer=pose_cad(along_x(510,60,40,3,-290),T(x,0,135)@R('z',90))
            self.component(f'base_outer_leg_{sign}',outer,mat=0,parent='base_rear_crossmember',kind='60 x 40 x 3 aluminium rectangular tube')
            inner=pose_cad(along_x(555,50,30,3,-250),T(x,0,135)@R('z',90))
            # Open-end underside access slot clears the retained rear caster stem
            # when retracted; the top and two side walls remain continuous.
            inner=inner.cut(box(24,42,10,(x,-239,120)))
            self.component(f'base_inner_leg_{sign}',inner,'base_extension',1,'50 x 30 x 3 telescoping aluminium tube',f'base_outer_leg_{sign}')
            self.cut(f'base_outer_leg_{sign}',cyl(4.3,110,160).translate((x,160,0)))
            for y in (160,-20,-200):self.cut(f'base_inner_leg_{sign}',cyl(4.3,110,160).translate((x,y,0)))
            pin=cyl(4,114,161).fuse(cyl(10,161,166)).clean().translate((x,160,0))
            self.component(f'extension_lock_pin_{sign}',pin,mat=2,kind='required retained steel pull-pin envelope',parent=f'base_outer_leg_{sign}',note='Positive position at 0, 180, 360 mm extension; purchased retention/handle data pending')
            self.wheel('rear_'+str(sign),x,-250,'base');self.wheel('front_'+str(sign),x,270,'base_extension')
        cross=along_x(500,60,40,3,-250).translate((0,-220,135))
        self.component('base_rear_crossmember',cross,mat=0,parent='base_saddle',kind='60 x 40 x 3 aluminium crossmember')
        saddle=rounded(260,330,6,(0,-105,158),12)
        saddle=saddle.cut(rounded(120,130,10,(0,-100,158),12))
        self.component('base_saddle',saddle,mat=1,parent='spine',note='Supplier-cut bridge joins the mast and crossmember')
        foot=rounded(200,160,9,(0,0,165.5),8);self.component('spine_foot',foot,mat=1,parent='base_saddle')
        for sign in (-1,1):
            shoe=box(6,100,130,(sign*73,0,235))
            self.component(f'spine_shoe_{sign}',shoe,parent='spine_foot')
            self.bolts(f'spine_side_fix{sign}',['spine',f'spine_shoe_{sign}'],[T(sign*76,y,z)@R('y',90*sign) for y in (-32,32) for z in (200,270)],r=4,length=16)
        for side in SIDES:
            A=FACES[side];shift=230 if side=='tray' else 80;low=220 if side=='tray' else 650;high=1080 if side=='tray' else 1460
            rail=box(20,17.5,high-low,(-18,8.75,(high+low)/2))
            # Rail is supplier outline data, not invented ball-return internals.
            for z in np.arange(low+20,high-10,60):
                cutter=pose_cad(cyl(3,0,20),T(-18,0,z)@R('x',-90));rail=rail.cut(cutter)
                rail=rail.cut(pose_cad(cyl(4.75,9,18),T(-18,0,z)@R('x',-90)))
            self.component(side+'_rail_envelope',pose_cad(rail,A),mat=1,kind='HIWIN HGR20 mounting outline envelope',parent='spine',note='Published 20 x 17.5 profile boundary, 60 mm hole pitch; raceway profile omitted')
            for i,z in enumerate(np.arange(low+20,high-10,60)):
                self.bolts(side+'_rail_fix'+str(i),['spine',side+'_rail_envelope'],[A@T(-18,9,z)@R('x',-90)],r=2.5,length=15)
            self.frame(side+'_carriage','base',A,'z',(side,0));g=side+'_carriage'
            for i,z in enumerate((-145,35)):
                block=box(44,25.4,77.5,(-18,17.3,z)).cut(box(20.1,18,80,(-18,8.6,z)))
                # Seating side v=30; internal raceways/balls remain unknown.
                self.component(side+f'_guide_block_{i}_envelope',block,g,1,'HIWIN HGH20CA exterior mounting envelope',side+'_rail_envelope')
            carriage=box(100,6,270,(0,33,-55));carriage=cq.Workplane(obj=carriage).edges('|Y').fillet(10).val()
            carriage=carriage.fuse(box(30,6,shift-35,(36,33,(shift+35)/2))).clean()
            self.component(side+'_carriage_plate',carriage,g,0,parent=side+'_guide_block_0_envelope')
            self.cut(side+'_carriage_plate',box(100,20,50,(-98,33,-60)))
            self.brake(side)
            self.bolts(side+'_carriage_fix',[side+'_carriage_plate']+[side+f'_guide_block_{i}_envelope' for i in range(2)],[T(-18+x,36,z+dz)@R('x',-90) for z in (-145,35) for x in (-16,16) for dz in (-18,18)],g,2.5,12)
            shelf=eye().fuse(cyl(7,-8,8)).cut(cyl(6.1,-10,10));shelf=pose_cad(shelf,T(0,113,0)@R('z',-90))
            shelf=shelf.cut(box(200,200,100,(0,-64,0)))
            self.component(side+'_shoulder_shelf',shelf,g,1,parent=side+'_carriage_plate')
            # The slot supports both faces of the shelf tongue. Widely spaced M6
            # carriage screws carry root bending; the M5 ties retain the tongue.
            anchor=box(40,34,70,(0,53,0)).cut(box(32.2,36,16,(0,53,0)))
            anchor=anchor.cut(pose_cad(cyl(5.25,35.9,40),T(-2,0,17)@R('x',-90)))
            self.component(side+'_root_anchor',anchor,g,1,'original supplier-milled slotted aluminium block',side+'_carriage_plate','70 mm high load-transfer block; 46 mm vertical M6 mounting pitch, seated 16 mm shelf slot')
            self.bolts(side+'_anchor_mount',[side+'_root_anchor',side+'_carriage_plate'],[T(x,70,z)@R('x',-90) for x in (-10,10) for z in (-23,23)],g,3,40)
            self.bolts(side+'_shelf_tie',[side+'_root_anchor',side+'_shoulder_shelf'],[T(0,y,35) for y in (45,60)],g,2.5,60)
            # Root joint axes are global Z; side-face rotations are removed at the mate.
            yaw=90 if side=='left' else -90 if side=='right' else 0
            self.frame(side+'_arm1',g,T(0,113,18)@R('z',-yaw),'z',(side,1))
            l1,l2=LENGTHS[side];self.beam(side,1,l1,40 if side=='tray' else 30)
            self.frame(side+'_arm2',side+'_arm1',T(l1,0,18),'z',(side,2));self.beam(side,2,l2,40 if side=='tray' else 30)
            self.head(side,l2)
            # Required gas package, kept visibly separate from sourced parts.
            cap=660 if side=='tray' else 850;bottom=90 if side=='tray' else 290
            self.component(side+'_gas_body_envelope',pose_cad(cyl(14,bottom,cap).translate((36,64,0)),A),mat=0,kind='unselected 28 mm gas-lift package envelope',parent='spine',note=GAS['status'])
            top=DEFAULT[side][0]+shift+16
            self.component(side+'_gas_rod_envelope',pose_cad(cyl(5,cap,top).translate((36,64,0)),A),mat=1,kind='exposed rod envelope; parametric visible length',parent=side+'_gas_body_envelope')
            # A separate lift pickup supplies a physical mounting datum for sourcing.
            bracket=box(30,50,8,(36,55,shift+4)).cut(cyl(5.2,shift-1,shift+10).translate((36,64,0)))
            self.component(side+'_lift_pickup',bracket,g,1,parent=side+'_carriage_plate',note='Connection bridge is an original part; gas end release and attachment data remain pending')
            self.bolts(side+'_pickup_fix',[side+'_lift_pickup',side+'_carriage_plate'],[T(u,33,shift+8) for u in (29,43)],g,2,20)
            foot=box(64,74,6,(26,43,bottom-3))
            self.component(side+'_gas_seat',pose_cad(foot,A),mat=1,parent='spine',note='Original gas package mounting platform; axial support, supplier end adapter still open')
            if side=='tray':
                back=box(64,6,71,(26,28,125.5))
                self.component('tray_gas_return',pose_cad(back,A),mat=1,parent='spine_foot',note='Original bolted bracket returns the lower gas seat to the base foot')
                self.bolts('tray_gas_return_fix',['tray_gas_seat','tray_gas_return'],[A@T(u,28,84)@R('x',180) for u in (10,42)],r=2,length=22)
                self.bolts('tray_gas_foot_fix',['spine_foot','tray_gas_return'],[A@T(u,28,170) for u in (10,42)],r=2,length=25)
            else:
                shoe='spine_shoe_'+('-1' if side=='left' else '1')
                for u in (6,46):self.cut('spine',pose_cad(cyl(4,-3,-.5),A@T(u,0,bottom-3)@R('x',-90)))
                self.bolts(side+'_gas_seat_fix',['spine',shoe,side+'_gas_seat'],[A@T(u,-.5,bottom-3)@R('x',90) for u in (6,46)],r=2,length=20)
            # External required stud/nut interface, no guessed gas internals.
            for suffix,a,b in [('lower',shift-8,shift),('upper',shift+8,shift+16)]:
                self.component(side+'_lift_nut_'+suffix,hhex(19.63,a,b,5).translate((36,64,0)),g,2,'nominal M10 nut; required 24 mm threaded end',side+'_lift_pickup','The selected gas unit needs an approved end adapter for this original interface')
            self.guard(side)
        self.finish_base()
        # A removable nonstructural cap gives the shared spine a finished edge.
        cap=rounded(146,106,14,(0,0,1497),8).cut(box(140.3,100.3,12,(0,0,1494)))
        self.component('spine_top_cap',cap,mat=8,kind='printed nonstructural cap',parent='spine')
        from wiring import routes
        routed,_=routes(self,DEFAULT)
        for name,s in routed.items():self.component(name,s,mat=4,kind='analytic swept cable routing envelope',parent='spine',note='Installation corridor and slack study; actual cable/connector selection and flex validation pending')
        return self
    def finish_base(self):
        for sign in (-1,1):
            plate=rounded(180,140,6,(sign*250,-230,158),8)
            n=f'base_corner_{sign}';leg=f'base_outer_leg_{sign}'
            for axis in ['side','top']:
                for edge in (-1,1):
                    pad=box(1.9,100,29.8,(sign*280+edge*26,165,135)) if axis=='side' else box(49.8,100,1.9,(sign*280,165,135+edge*16))
                    pad=pad.cut(cyl(4.3,110,165).translate((sign*280,160,0)))
                    self.component(f'base_wear_{sign}_{axis}_{edge}',pad,mat=3,kind='1.9 mm cut polymer guide shim',parent=leg,note='0.05 mm nominal face clearance; fit must be set against the actual stock')
            self.component(n,plate,mat=1,parent='base_rear_crossmember')
            pts=[(sign*220,y) for y in (-240,-200)]
            self.bolts(f'base_corner_fix{sign}',[n,'base_rear_crossmember'],[T(x,y,161) for x,y in pts],r=3,length=20)
            for j,(x,y) in enumerate(pts):self.component(f'base_corner_nut{sign}_{j}',hhex(11.547,147,152,3).translate((x,y,0)),mat=2,kind='nominal M6 captive nut',parent=n)
            for prefix,g,cy,bot,top,seat in [('rear','base',-250,115,155,163),('front','base_extension',270,120,150,158)]:
                stem=prefix+'_'+str(sign);owner=leg if prefix=='rear' else f'base_inner_leg_{sign}'
                if prefix=='front':self.component(stem+'_top_clamp',rounded(100,100,6,(sign*280,cy,153),5),g,1,'original cut saddle clamp',owner)
                shim=box(60 if prefix=='rear' else 50,95,bot-111,(sign*280,cy,(111+bot)/2)).cut(cyl(6.2,110,bot+1).translate((sign*280,cy,0)))
                self.component(stem+'_saddle_shim',shim,g,1,'original full-width saddle shim',owner)
                self.cut(owner,cyl(6.2,bot-1,bot+4).translate((sign*280,cy,0)))
                self.component(stem+'_stem_washer',cyl(12,bot+3,bot+5,6.2).translate((sign*280,cy,0)),g,2,'M12 washer envelope',owner)
                self.component(stem+'_stem_nut',hhex(21.94,bot+5,bot+15,6).translate((sign*280,cy,0)),g,2,'nominal M12 stem nut',stem+'_fork_envelope')
                for j,(dx,dy) in enumerate([(a,b) for a in (-40,40) for b in (-40,40)]):
                    x,y=sign*280+dx,cy+dy
                    self.component(stem+f'_washer{j}',cyl(9,seat-2,seat,4.3).translate((x,y,0)),g,2,'M8 washer envelope',owner)
                    targets=[stem+'_adapter',n if prefix=='rear' else stem+'_top_clamp']+(['base_rear_crossmember'] if prefix=='rear' else [])
                    self.bolts(stem+f'_mount{j}',targets,[T(x,y,seat)],g,4,70 if prefix=='rear' else 65)
                    if prefix=='rear' and abs(x)<250 and y>-250:self.component(stem+f'_cross_crush{j}',cyl(6.5,118,152,4.3).translate((x,y,0)),g,2,'cut compression sleeve inside crossmember','base_rear_crossmember')
                    self.component(stem+f'_nut{j}',hhex(15.01,96.5,103,4).translate((x,y,0)),g,2,'nominal M8 nut',stem+'_adapter')
        for sign in (-1,1):
            for y in (-30,30):self.cut('spine_foot',cyl(4,160.9,163.3).translate((sign*73,y,0)))
            self.bolts(f'foot_shoe_fix{sign}',['spine_foot',f'spine_shoe_{sign}'],[T(sign*73,y,163.3)@R('x',180) for y in (-30,30)],r=2,length=25)
        self.bolts('foot_to_saddle',['spine_foot','base_saddle'],[T(x,y,170) for x in (-85,85) for y in (-55,50)],r=2.5,length=25)
        for i,(x,y) in enumerate([(a,b) for a in (-85,85) for b in (-55,50)]):self.component('foot_to_saddle_nut'+str(i),hhex(9.24,151,155,2.5).translate((x,y,0)),mat=2,kind='nominal M5 nut',parent='base_saddle')

    def world_shape(self,p,q,fs=None):
        fs=fs or self.poses(q)[0]
        n=p['name']
        if n.endswith('_harness_route') or n.endswith('_service_loop'):
            from wiring import routes
            side=n.split('_')[0];key=tuple(q[side])
            if not hasattr(self,'_route_keys'):self._route_keys={};self._route_shapes={};self._route_data={}
            if self._route_keys.get(side)!=key:
                s,d=routes(self,q,sides=[side]);self._route_shapes.update(s);self._route_data.update(d);self._route_keys[side]=key
            return self._route_shapes[n]
        if n.endswith('_gas_rod_envelope'):
            side=n.split('_')[0];cap=660 if side=='tray' else 850;shift=230 if side=='tray' else 80
            return pose_cad(cyl(5,cap,q[side][0]+shift+16).translate((36,64,0)),FACES[side])
        return pose_cad(p['shape'],fs[p['group']])

    def assembly(self,q,solve=False):
        f,receipts=self.poses(q,solve)
        def provenance(p):
            k=p['kind'].lower()
            if 'envelope' in k or 'nominal' in k or 'purchased spacer' in k or 'purchased screw' in k:return 'estimated-component-envelope'
            if 'gfm-' in k or 'dimensioned purchased bushing' in k or 'norelem' in k:return 'source-guided-concept'
            return 'designed-concept'
        parts=[cad_part(p['name'],self.world_shape(p,q,f),p['mat'],tolerance=.35 if p['name'].endswith(('_harness_route','_service_loop')) else .12,angular=.18,group=p['group'],role=p['note'] or p['kind'],provenance=provenance(p)) for p in self.parts]
        a=Assembly('ROAM / shared spine',parts,MAT,views={
            'hero':View(az=55,el=17,scale=1450,target=(0,70,800),focal_length_mm=52),
            'rear':View(az=-125,el=20,scale=1450,target=(0,50,800)),
            'joint':View(az=45,el=25,scale=170,target=(0,0,0))})
        return a,receipts


def fingerprint():return hashlib.sha256(Path(__file__).read_bytes()+(OUT/'wiring.py').read_bytes()).hexdigest()
def export():
    m=Model().build();(OUT/'cad').mkdir(exist_ok=True);(OUT/'verification').mkdir(exist_ok=True)
    print('Writing analytic bodies',flush=True)
    for p in m.parts:cq.exporters.export(p['shape'],str(OUT/'cad'/(p['name']+'.brep')))
    print('Solving and tessellating',flush=True)
    a,receipts=m.assembly(DEFAULT,True)
    export_step(a,OUT/'cad/workstation.step',individual=False);export_glb(a,OUT/'cad/workstation.glb');export_bom(a,OUT/'cad');save_cache(a,OUT/'cache')
    del a
    import gc;gc.collect()
    print('Binding export receipt',flush=True)
    current_names={p['name']+'.brep' for p in m.parts}
    for old in (OUT/'cad').glob('*.brep'):
        if old.name not in current_names:
            assert old.resolve().parent==(OUT/'cad').resolve()
            old.unlink() # obsolete generated candidate, never the R5/R6 baseline
    manifest={'schema':1,'cable_routes':m._route_data,'source_sha256':fingerprint(),'pivot_interface':PIVOT,'rail_interface':RAIL,'gas_interface':GAS,'link_lengths':LENGTHS,'fastener_interface':{'source':BUTTON_SOURCE,'ISO_7380_table_by_shaft_radius':BUTTON_HEAD,'head_diameter_clearance_mm':.5},'frames':m.frames,'default':DEFAULT,'ranges':RANGES,'axes':AXES,'poses':POSES,
              'parts':[{k:v for k,v in p.items() if k!='shape'} for p in m.parts],'connections':m.connections,'mates':receipts,
              'scope':'Original nominal design assembly; component envelopes explicitly labeled; physical qualification incomplete'}
    (OUT/'cad/assembly.json').write_text(json.dumps(manifest,indent=2))
    exported=[OUT/'cad'/name for name in current_names]+[OUT/'cad/workstation.step',OUT/'cad/workstation.glb',OUT/'cad/assembly.json',OUT/'cache/manifest.json',OUT/'cache/meshes.npz']
    receipt={'source_sha256':fingerprint(),'files':{str(p.relative_to(OUT)).replace('\\','/'):hashlib.file_digest(p.open('rb'),'sha256').hexdigest() for p in exported}}
    (OUT/'cad/export-receipt.json').write_text(json.dumps(receipt,indent=2))
    print(json.dumps({'components':len(m.parts),'driven_mates':len(receipts),'max_mate_residual_mm':max(x['error_mm'] for x in receipts),'STEP':str(OUT/'cad/workstation.step')}),flush=True)
if __name__=='__main__':
    try:export();os._exit(0)
    except BaseException:
        import traceback;traceback.print_exc();sys.stderr.flush();os._exit(1)
