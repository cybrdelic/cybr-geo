"""Upright machine frame, bolted feet and a removable tool-carriage interface."""
from slide import linear_stage
from composition import *
def upright(id,title):
 m=Machine(id,title);stage=linear_stage(length=200,travel=80);append(m,stage,'V_',offset=(0,0,120),rotation=('Y',-90))
 base=m.add('B00_Bench_plate',box(240,200,10,(-55,0,-5),5),1,role='Flat bench datum and anchor plate',make='Cut/drill aluminum plate')
 for x in [-155,45]:
  for y in [-80,80]:m.cut(base,cyl(3.3,12,(x,y,-11)))
 for y in [-62,62]:
  q=cq.Workplane('XZ',origin=(0,y+6,0)).polyline([(0,0),(-30,0),(-30,4),(-4,4),(-4,32),(0,32)]).close().extrude(12).val()
  name=f'B10_{y}_Angle_foot';m.add(name,q,1,role='Separate metal angle ties upright to bench plate',make='Cut angle stock',parent=base,fit=0)
  m.cut(base,cyl(4.1,4.8,(-20,y,-10)));m.bolt(f'B11_{y}_Foot',base,name,(-20,y,-5.2),9.2)
  m.bolt(f'B12_{y}_Back',name,'V_B01_Ribbed_base',(-4,y,18),12,'X')
 plate=m.add('H01_Carriage_adapter',box(16,70,60,(-52,0,120),3),0,['V_feed'],'Removable tool mounting plate with accessible through-fasteners','Print; specific head attached separately')
 fasteners=[]
 for y in [-16,16]:
  for z in [100,140]:
   m.cut('V_S02_Upper_carriage',cyl(2.2,56,(-61,y,z),'X'))
   before=set(m.parts);m.bolt(f'H10_{y}_{z}',plate,'V_S01_Lower_carriage',(-60,y,z),54,'X');fasteners+=list(set(m.parts)-before)
 m.connect(plate,'V_S02_Upper_carriage','Carriage landing face',0)
 m.requirements={'vertical_travel_mm':80,'bench_plate_mm':[240,200,10],'tool_interface':'Four M4 through-fasteners on 32 x 40 mm pattern; seated plane X=-44 mm','load_rating':None}
 m.notes=['Upright uses an independently retained feed and two ground guides.','Bench plate and angle feet are metal; printed carriage parts are not assigned a force rating.','Feed is reversible screw control, chosen instead of an unqualified spring-return lever.']
 return m,plate,fasteners

def workpiece(m,x=-110,hole=3.4):
 stock=m.add('W01_Pilot_coupon',box(30,30,22,(x,0,11)).cut(cyl(hole/2,24,(x,0,-1))),5,role='Metal coupon with existing pilot; action illustrates tool engagement, not chip formation',make='Reference stock',parent='B00_Bench_plate',fit=0)
 for side in [-1,1]:
  px=x+side*29;center=x+side*23
  post=m.add(f'W02_{side}_Heel',ring(5,2.2,22,(px,0,0)),1,role='Heel support for bolted work clamp',make='Cut spacer',parent='B00_Bench_plate',fit=0)
  toe=m.add(f'W03_{side}_Toe',box(20,12,4,(center,0,24),1),1,role='Metal hold-down toe bearing on coupon and heel',make='Cut/drill strap',parent=post,fit=0)
  m.connect(toe,stock,'Workholding contact face',0)
  m.cut('B00_Bench_plate',cyl(4.1,4.8,(px,0,-10)));m.bolt(f'W04_{side}_Clamp','B00_Bench_plate',toe,(px,0,-5.2),31.2)
 return stock
