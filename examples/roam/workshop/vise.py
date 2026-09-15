"""Screw-fed bench vise with separate metal jaws and a supported sample."""
from slide import *

def build():
 m=linear_stage('vise',length=180,travel=60);m.title='ROAM / guided bench vise';mo=['feed']
 upper='S02_Upper_carriage';fixed='E1_Guide_pedestal'
 def replace_bolt(name,a,b,start,grip):
  names={n for n in m.parts if n==name or n.startswith(name+'_')}
  for n in names:del m.parts[n]
  m.connections=[c for c in m.connections if c['a']not in names and c['b']not in names]
  m.threads=[t for t in m.threads if t['a']not in names and t['b']not in names]
  m.bolt(name,a,b,start,grip)
 m.fuse(upper,box(10,100,30,(22,0,59),2));m.fuse(fixed,box(17,100,30,(81.5,0,59),2))
 for y in [-20,20]:replace_bolt(f'F1_{y}_Foot_bolt','B01_Ribbed_base',fixed,(80,y,-8),82)
 for y in [-43,43]:replace_bolt(f'SB22_{y}','S01_Lower_carriage',upper,(22,y,6),68)
 moving=m.add('J01_Moving_metal_face',box(3,100,30,(28.5,0,59),.7),1,mo,'Replaceable metal jaw face with fasteners outside the work lane','Cut/drill jaw plate',upper,fit=0)
 stationary=m.add('J02_Fixed_metal_face',box(3,100,30,(71.5,0,59),.7),1,role='Replaceable stationary jaw face',make='Cut/drill jaw plate',parent=fixed,fit=0)
 moving_fasteners=[]
 for y in [-32,32]:
  before=set(m.parts);m.bolt(f'J10_{y}_Moving',upper,moving,(17,y,59),13,'X');moving_fasteners+=list(set(m.parts)-before)
  m.bolt(f'J11_{y}_Fixed',stationary,fixed,(70,y,59),20,'X')
 # Integral shelf actually carries the stock while the vise is open.
 m.fuse(fixed,box(10,35,4,(65,0,42),1))
 m.add('W01_10mm_Sample',box(10,30,25,(65,0,56.5)),5,role='10 mm reference coupon rests on the jaw shelf; closure stops at nominal contact',make='Reference workpiece',parent=fixed,fit=0)
 m.connect('W01_10mm_Sample',stationary,'Fixed jaw contact',0)
 m.service=[dict(parts=[upper,moving]+moving_fasteners,remove_first=[n for n in m.parts if n.startswith('SB')],axis='Z',distance_mm=65,step_mm=1,description='Remove four carriage fastener sets; lift the carriage cap and attached metal jaw as one supported unit.')]
 m.requirements.update(operation='Close the moving jaw on a shelf-supported coupon using the retained screw feed',jaw_opening_mm=[10,70],jaw_width_mm=100,jaw_height_mm=30,sample_thickness_mm=10,load_rating=None)
 m.notes+=['Clear PETG carries reaction between metal faces and guide bearings. No clamping-force or machining-workholding rating is assigned.','Fasteners sit outside the central work lane. The fixed jaw shelf supports the sample throughout opening and closure.','Metal jaw plates, ground guides, feed rod and fasteners must be purchased or fabricated.']
 return m
if __name__=='__main__':
 try:build().export(ROOT/'outputs/roam-workshop/machines/vise')
 except Exception:
  import traceback;traceback.print_exc();sys.stderr.flush();os._exit(1)
 sys.stdout.flush();os._exit(0)
