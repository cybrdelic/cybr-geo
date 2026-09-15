"""Two independent, retained linear feeds and a real, removable fixture deck."""
from slide import linear_stage
from composition import *
def build():
 m=Machine('table','ROAM / two-axis positioning table');x=linear_stage(length=180,travel=60);y=linear_stage(length=180,travel=60)
 append(m,x,'X_');upper=append(m,y,'Y_',offset=(0,0,68),rotation=('Z',90),inherited=['X_feed'])
 posts=[];fasteners=[]
 for xx in [-20,20]:
  for yy in [-16,16]:
   name=f'I_{xx}_{yy}_Spacer';m.add(name,ring(7,2.2,16,(xx,yy,44)),1,['X_feed'],'Spacer supporting upper stage; accessible through-bolt','Cut spacer stock','X_S02_Upper_carriage',fit=0);posts.append(name)
   m.cut('X_S02_Upper_carriage',cyl(2.2,64,(xx,yy,5)))
   before=set(m.parts);m.bolt(f'I_{xx}_{yy}_Bolt','X_S01_Lower_carriage','Y_B01_Ribbed_base',(xx,yy,6),62);fasteners+=list(set(m.parts)-before)
   m.connect(name,'Y_B01_Ribbed_base','Upper stage supporting face',0)
 plate,fp,ff=fixture_plate(m,'W_','Y_S01_Lower_carriage','Y_S02_Upper_carriage',128,['Y_feed','X_feed'],xy=(16,20))
 # The upper module lifts as a unit before the lower split housing is opened.
 m.service=[dict(parts=upper+[plate]+fp+ff,remove_first=fasteners,axis='Z',distance_mm=80,step_mm=2,description='Unbolt the four stage fasteners, support the upper module, and lift it off the four spacers.'),dict(part='X_S02_Upper_carriage',remove_first=upper+[plate]+fp+ff+posts+fasteners+[n for n in m.parts if n.startswith('X_SB')],axis='Z',distance_mm=45,step_mm=1,description='After upper module and loose spacers are on the bench, remove the lower carriage cap fasteners and lift its cap.')]
 m.requirements=dict(x_travel_mm=60,y_travel_mm=60,independent_controls=['X_feed','Y_feed'],work_plate_mm=[150,150],fixture_grid_pitch_mm=50,purpose='Drilling/assembly positioning, not a milling-load certificate')
 m.notes=['Both axes have their own captive nut, paired guides, axial stops and reversible manual feed.','Move one axis at a time for positioning; the viewer may demonstrate combined motion within checked samples.','This new machine is a complete two-axis mechanism, not two visually intersecting slide meshes.','Keep the metal-cutting vise separate until table stiffness, workholding and locking are qualified.']
 return m
if __name__=='__main__':
 try:build().export(ROOT/'outputs/roam-workshop/machines/table')
 except Exception:
  import traceback;traceback.print_exc();sys.stderr.flush();os._exit(1)
 sys.stdout.flush();os._exit(0)
