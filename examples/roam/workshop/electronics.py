"""Two-bearing PCB turnover fixture with separate metal contact shoes."""
from kernel import *

def build():
 m=Machine('electronics','ROAM / PCB turnover fixture');mo=['roll'];m.dof('roll','rotary','X',0,180,(0,0,55));base=m.add('B01_Bench_plate',box(180,110,8,(0,0,-4),6),1,role='Bench-anchored datum plate',make='Cut/drill plate')
 for x in [-78,78]:
  for y in [-42,42]:m.cut(base,cyl(3.3,10,(x,y,-9)))
 top_parts=[];top_bolts=[]
 for side in [-1,1]:
  x=side*60;pillar=f'E{side}_Bearing_pillar';q=box(10,60,76,(x,0,38),3).cut(cyl(4.2,12,(x-6,0,55),'X'));seat=-65 if side<0 else 58;q=q.cut(cyl(11.1,7.01,(seat,0,55),'X'))
  m.add(pillar,q,0,role='Shouldered bearing pillar with separate outside retainer',make='Print',parent=base,fit=0)
  for y in [-24,24]:m.bolt(f'F{side}_{y}_Foot',base,pillar,(x,y,-8),84)
  bearing=f'R{side}_608';m.add(bearing,ring(11,4,7,(seat,0,55),'X'),2,role='608 bearing envelope',make='Purchase 608 bearing',parent=pillar,fit=.1)
  cap=f'C{side}_Retainer';capx=-68 if side<0 else 65;m.add(cap,box(3,60,40,(capx+1.5,0,55),1).cut(cyl(4.2,5,(capx-1,0,55),'X')),0,role='Removable outside bearing cap',make='Print',parent=pillar,fit=0)
  cap_fasteners=[]
  for y in [-20,20]:
   for z in [43,67]:
    before=set(m.parts);m.bolt(f'K{side}_{y}_{z}',cap if side<0 else pillar,pillar if side<0 else cap,(-68 if side<0 else 55,y,z),13,'X',diam=3);cap_fasteners+=list(set(m.parts)-before)
  start=-74 if side<0 else 40.5;length=33.5 if side<0 else 41;shaft=cyl(4,length,(start,0,55),'X').cut(box(length+2,8,12,(start+length/2,-7.8,55)))
  axle=f'A{side}_Stub_axle';m.add(axle,shaft,2,mo,'Independent stub axle; no axle passes through the circuit board','Cut/file 8 mm shaft',bearing,fit=0)
  for name,wx,cx,cl,ro in [('inner',-55 if side<0 else 54.2,-54.1 if side<0 else 51.1,3,7),('outer',-68.8 if side<0 else 68,-72.9 if side<0 else 68.9,4 if side<0 else 10,7 if side<0 else 14)]:
   washer=f'T{side}_{name}_Washer';m.add(washer,ring(7,4.2,.8,(wx,0,55),'X'),2,role='Axial thrust washer against stationary bearing support',make='Purchase washer',parent=pillar if name=='inner' else cap,fit=0)
   stop=f'D{side}_{name}_Stop';shape=handwheel(ro,4.2,cl,(cx,0,55),'X') if side>0 and name=='outer' else ring(ro,4.2,cl,(cx,0,55),'X')
   m.add(stop,shape,0 if side>0 and name=='outer' else 2,mo,'Fluted turnover wheel' if side>0 and name=='outer' else 'Axial stop collar locked to filed axle flat','Print wheel / obtain metal collar',axle,fit=.21);m.socket(stop+'_Lock',stop,(cx+cl/2,-ro,55),'Y',diam=3,length=ro-3.8);m.connect(stop,washer,'Axial stop',.11)
   if name=='inner':
    p=(cx+cl/2,-6.8,55);m.parts[stop+'_Lock']['shape']=cyl(1.5,3,p,'Y').cut(prism(6,1.5,1.2,xyz(p,'Y',-.01),'Y'));m.parts[stop+'_Lock']['role']='Recessed M3 x 3 headless flat-point set screw';m.parts[stop+'_Lock']['make']='Purchase matching grub screw; nominal thread envelope'
  # Offset grips leave the board faces and most of its edge unobstructed.
  gx=side*45;lower=f'J{side}_Lower_grip';body=box(12,20,23.5,(gx,0,54.75),1).cut(cyl(4.2,14,(-52 if side<0 else 38,0,55),'X'))
  m.add(lower,body,0,mo,'Offset split-shoe carrier retained on axle flat','Print',axle,fit=.21);m.socket(f'J{side}_Axle_lock',lower,(side*42,-10,55),'Y',diam=3,length=6.2)
  p=(side*42,-9.8,55);m.parts[f'J{side}_Axle_lock']['shape']=cyl(1.5,6,p,'Y').cut(prism(6,1.5,1.2,xyz(p,'Y',-.01),'Y'));m.parts[f'J{side}_Axle_lock']['role']='Recessed M3 x 6 flat-point grub screw on axle flat';m.parts[f'J{side}_Axle_lock']['make']='Purchase matching grub screw; nominal thread envelope'
  shoes=[]
  for label,z in [('lower',66.5),('upper',68.6)]:
   name=f'J{side}_{label}_Metal_shoe';m.add(name,box(12,20,.5,(gx,0,z+.25)),2,mo,'Thin metal contact shoe between PCB and printed jaw','Cut/drill sheet',lower if label=='lower' else None,fit=0);shoes.append(name)
  upper=f'J{side}_Upper_grip';m.add(upper,box(12,20,4,(gx,0,71.1),1),0,mo,'Removable upper grip supports its metal contact shoe','Print',shoes[1],fit=0);top_parts += [upper,shoes[1]]
  for y in [-6,6]:
   spacer=f'J{side}_{y}_Gap_spacer';m.add(spacer,ring(3,1.7,1.6,(side*46,y,67)),2,mo,'Metal shim stack sets nominal 1.6 mm board gap','Measure and select shim thickness',shoes[0],fit=0);m.connect(spacer,shoes[1],'Upper shoe landing',0)
   for shoe in shoes:m.cut(shoe,cyl(1.7,9,(side*46,y,66)))
   before=set(m.parts);m.bolt(f'JB{side}_{y}',lower,upper,(side*46,y,43),30.1,diam=3);top_bolts+=list(set(m.parts)-before)
  outside=[f'D{side}_outer_Stop',f'D{side}_outer_Stop_Lock',f'T{side}_outer_Washer']
  m.service.append(dict(part=cap,remove_first=cap_fasteners+outside,axis='X',distance_mm=side*24,step_mm=1,description='Remove outer input/stop collar, thrust washer and cap fasteners; support and withdraw cap along the axle.'))
 pcb=box(80,60,1.6,(0,0,67.8),2)
 for x in [-30,30]:
  for y in [-20,20]:pcb=pcb.cut(cyl(1.6,3,(x,y,66.5)))
 m.add('W01_Reference_PCB',pcb,5,mo,'Nominal 80 x 60 x 1.6 mm board held between metal shoes','Reference workpiece','J-1_lower_Metal_shoe',fit=0)
 for side in [-1,1]:
  for label in ['lower','upper']:m.connect('W01_Reference_PCB',f'J{side}_{label}_Metal_shoe','Board contact',0)
 m.service.append(dict(part='W01_Reference_PCB',remove_first=top_parts+top_bolts,axis='Z',distance_mm=70,step_mm=2,description='Return to horizontal datum, support and remove upper jaws/shoes and fasteners, then lift the board.'))
 m.sources=[dict(url='https://www.emarketplace.in.skf.com/industrial/bearings?search=608-RSH',supports='608 envelope: 8 mm bore, 22 mm OD and 7 mm width. Internal bearing races and balls omitted.')]
 m.requirements=dict(operation='Turn the clamped board through 180 degrees while two retained stub axles leave the board center clear',board_mm=[80,60,1.6],rotation_deg=[0,180],thermal_rating=None,scope='PCB assembly fixture; soldering iron, heat extraction and powered test equipment are not included')
 m.notes=['Metal shoes separate the board from PETG but do not establish a soldering temperature rating.','The nominal 1.6 mm gap requires shims matched to the actual board; no automatic clamp-force model is implied.','The PCB is part of the rotating assembly. Neither shaft crosses its working area.']
 return m
if __name__=='__main__':
 try:build().export(ROOT/'outputs/roam-workshop/machines/electronics')
 except Exception:
  import traceback;traceback.print_exc();sys.stderr.flush();os._exit(1)
 sys.stdout.flush();os._exit(0)
