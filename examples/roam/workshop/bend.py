"""Bench wire bender with a retained rolling follower and replaceable mandrel.
All dimensions are original nominal design dimensions, not a bend-force rating.
"""
from kernel import *

def wire_shape(angle):
 r=17.5;z=40.7;t=math.radians(angle);start=cq.Vector(r,-60,z);corner=cq.Vector(r,0,z)
 if abs(t)<1e-8:path=cq.Wire.assembleEdges([cq.Edge.makeLine(start,cq.Vector(r,40,z))])
 else:
  end=cq.Vector(r*math.cos(t),r*math.sin(t),z);mid=cq.Vector(r*math.cos(t/2),r*math.sin(t/2),z);tail=end+cq.Vector(-math.sin(t),math.cos(t),0)*(40-r*t)
  path=cq.Wire.assembleEdges([cq.Edge.makeLine(start,corner),cq.Edge.makeThreePointArc(corner,mid,end),cq.Edge.makeLine(end,tail)])
 return cq.Workplane('XZ',origin=start.toTuple()).circle(1.5).sweep(path,isFrenet=True).val()

def build():
 m=Machine('bend','ROAM / rolling wire bender');mo=['bend'];m.dof('bend','rotary','Z',0,90)
 base=m.add('B01_Bench_plate',box(190,160,12,(20,-15,6),8),1,role='Metal bench anchor plate',make='Cut and drill plate')
 for x in [-60,100]:
  for y in [-80,50]:m.cut(base,cyl(3.3,14,(x,y,-1)))
 m.add('P01_Lower_inner_spacer',ring(6,4.2,4,(0,0,12)),2,role='Fixed lower bearing inner-race spacer',make='Cut metal spacer',parent=base,fit=0)
 m.add('P02_Pivot_608',ring(11,4,7,(0,0,16)),2,role='608 bearing envelope: inner race retained, outer race carries arm',make='Purchase 608 bearing',parent='P01_Lower_inner_spacer',fit=0)
 hub=cyl(18,8,(0,0,15)).fuse(box(90,16,14,(60,0,22),4)).cut(cyl(11.1,7.1,(0,0,15.9))).cut(cyl(6.2,16,(0,0,14)))
 arm=m.add('A01_Rolling_arm',hub,0,mo,'Bearing-supported arm with shouldered pivot seat and raised follower pad','Print; force capacity unqualified','P02_Pivot_608',fit=.1)
 cap=cyl(18,6,(0,0,23)).cut(cyl(6.2,8,(0,0,22))).cut(box(40,60,10,(35,0,26)))
 m.add('A02_Pivot_cap',cap,0,mo,'Removable outer-race retention cap','Print',arm,fit=0)
 for angle in [90,180,270]:
  a=math.radians(angle);x=14*math.cos(a);y=14*math.sin(a);m.cut(arm,cyl(3.1,3.8,(x,y,15)));m.bolt(f'A03_{angle}_Cap_bolt',arm,'A02_Pivot_cap',(x,y,18.8),10.2,diam=3)
 m.add('P03_Upper_inner_spacer',ring(6,4.2,7,(0,0,23)),2,role='Upper inner-race spacer passes through rotating cap with clearance',make='Cut metal spacer',parent='P02_Pivot_608',fit=0)
 die=cyl(16,12,(0,0,37.2)).fuse(cyl(19,2,(0,0,37.2))).fuse(cyl(19,2,(0,0,47.2)))
 die=m.add('D01_32mm_Mandrel',die,0,role='Replaceable flanged bend mandrel with supported wire channel',make='Print; coupon testing required')
 m.cut(base,cyl(8.1,8.8,(0,0,0)));m.bolt('P10_Main_stud',base,die,(0,0,8.8),40.4,diam=8)
 m.add('P11_Lower_nut_washer',ring(8,4.2,.8,(0,0,30)),2,role='Lower clamping washer against inner-race spacer',make='Purchase washer',parent='P03_Upper_inner_spacer',fit=0)
 m.add('P12_Lower_locknut',prism(6,14.4,6.4,(0,0,30.8)).cut(cyl(3.28,8,(0,0,30))),2,role='Lower nut sets bearing retention independently of removable die',make='Purchase M8 nut',parent='P11_Lower_nut_washer',fit=0)
 m.threads.append(dict(a='P10_Main_stud',b='P12_Lower_locknut',mask=ring(4.001,3.279,6.402,(0,0,30.799)),motion=[],min_volume=.01,description='Bounded nominal M8 lower nut engagement'))
 m.connect(die,'P12_Lower_locknut','Mandrel lower landing',0)
 m.connect('P01_Lower_inner_spacer','P02_Pivot_608','Axial stop',0)
 m.connect('P03_Upper_inner_spacer','P02_Pivot_608','Axial stop',0)
 # Follower bearing is independently captured on a metal pin and inner spacers.
 m.add('R01_Lower_spacer',ring(6,4.2,11,(30,0,29)),2,mo,'Follower lower inner-race spacer','Cut metal spacer',arm,fit=0)
 m.add('R02_Follower_608',ring(11,4,7,(30,0,40)),2,mo,'608 outer diameter provides the rolling contact envelope','Purchase 608 bearing','R01_Lower_spacer',fit=0)
 m.add('R03_Upper_spacer',ring(6,4.2,3,(30,0,47)),2,mo,'Upper follower inner-race spacer','Cut metal spacer','R02_Follower_608',fit=0)
 m.cut(arm,cyl(8.1,8.8,(30,0,15)));m.bolt('R10_Follower_pin',arm,'R03_Upper_spacer',(30,0,23.8),26.2,diam=8)
 m.connect('R01_Lower_spacer','R02_Follower_608','Axial stop',0)
 # Through-bolt retains the tall input grip; no unsupported handle stub.
 grip=m.add('A10_Input_grip',handwheel(10,4.2,60,(95,0,29)),0,mo,'Hand grip captured between arm and metal washer/nut','Print',arm,fit=0)
 m.cut(arm,cyl(8.1,8.8,(95,0,15)));m.bolt('A11_Grip_pin',arm,grip,(95,0,23.8),65.2,diam=8)
 # Split metal feed clamp holds the straight leg. Its bore has actual contact.
 lower=m.add('W01_Feed_clamp_lower',box(14,16,28.7,(17.5,-40,26.35),1).cut(cyl(1.5,18,(17.5,-49,40.7),'Y')),1,role='Fixed workholding block with half-round wire seat',make='Cut/drill split metal block',parent=base,fit=0)
 upper=m.add('W02_Feed_clamp_cap',box(14,16,6,(17.5,-40,43.7),1).cut(cyl(1.5,18,(17.5,-49,40.7),'Y')),1,role='Separate metal clamp cap bears on the wire',make='Cut/drill split metal block',parent=lower,fit=0)
 for x in [12.5,22.5]:
  m.cut(base,cyl(4.1,4.8,(x,-40,0)));m.cut(lower,cyl(2.2,30,(x,-40,11)));m.bolt(f'W03_{x}_Feed_clamp',base,upper,(x,-40,4.8),41.9)
 m.add('W10_Wire',wire_shape(0),5,role='3 mm wire: prescribed constant-length bending shape, not a material simulation',make='Reference workpiece',parent=lower,fit=0)
 m.sources=[dict(url='https://www.emarketplace.in.skf.com/industrial/bearings?search=608-RSH',supports='608 nominal bore 8, outside diameter 22 and width 7 mm. Bearing internals are not modeled.')]
 m.requirements=dict(operation='Rotate the supported follower around a fixed die while the split feed clamp holds the straight wire leg',wire_diameter_mm=3,mandrel_diameter_mm=32,bend_angle_deg=[0,90],wire_length_mm=100,load_rating=None,process='Prescribed constant-length wire sweep; no springback, plasticity or force prediction')
 m.requirements['deformation']=dict(part='W10_Wire',control='bend',module='bend',function='wire_shape',conserve_volume=True)
 m.service=[dict(part=die,remove_first=['P10_Main_stud_nut','P10_Main_stud_nut_washer','W10_Wire'],axis='Z',distance_mm=35,step_mm=1,description='Remove workpiece and upper nut/washer, then lift die off the retained central stud.')]
 m.notes=['This model retains all pivot and follower hardware. Bending capacity in any wire alloy is unqualified.','The 608 external envelope is concentric; internal balls and race rotation are omitted.','The stock changes shape according to a prescribed constant-length CAD path; the geometry does not predict springback.','Metal feed clamp requires drilling/splitting; it cannot be made by the printer alone.']
 return m
if __name__=='__main__':
 try:build().export(ROOT/'outputs/roam-workshop/machines/bend')
 except Exception:
  import traceback;traceback.print_exc();sys.stderr.flush();os._exit(1)
 sys.stdout.flush();os._exit(0)
