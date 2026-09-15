"""Screw-fed seating press with a retained steel tool and supported anvil."""
from vertical import *

def build():
 m,plate,attachment=upright('press','ROAM / light seating press');mo=['V_feed'];cx=-110
 m.fuse(plate,box(45,32,20,(-82.5,0,120),3));m.fuse(plate,cyl(16,35,(cx,0,95)));m.cut(plate,cyl(4.2,37,(cx,0,94)))
 spacer=m.add('P01_Ram_spacer',ring(7,4.2,35,(cx,0,60)),2,mo,'Metal sleeve carries the pressing-bolt shoulder against the carriage head','Cut metal tube; ends must be square',plate,fit=0)
 m.bolt('P02_Pressing_bolt',spacer,plate,(cx,0,60),70,diam=8)
 # Broad metal anvil is mounted to the bench rather than floating below ram.
 anvil=m.add('W01_Anvil',box(30,30,8.2,(cx,0,4.1),2),1,role='Bolted metal support anvil',make='Cut/drill metal plate',parent='B00_Bench_plate',fit=0)
 for dx in [-10,10]:
  for y in [-10,10]:
   m.cut('B00_Bench_plate',cyl(4.1,4.8,(cx+dx,y,-10)));m.bolt(f'W02_{dx}_{y}_Anvil','B00_Bench_plate',anvil,(cx+dx,y,-5.2),13.4)
 m.add('W10_Reference_washer',ring(6,3,3,(cx,0,8.2)),5,role='Supported 3 mm seating coupon; ram reaches nominal contact at minimum feed',make='Reference part',parent=anvil,fit=0)
 head=[n for n in m.parts if n==plate or n.startswith('P0')]
 m.service=[dict(parts=head,remove_first=attachment,axis='X',distance_mm=-120,step_mm=2,description='At datum height, remove four carriage bolts and support the complete head while withdrawing it.')]
 m.requirements.update(operation='Lower the retained steel tool onto a supported seating coupon; motion stops at nominal contact',ram_stroke_mm=80,contact_height_mm=11.2,load_rating=None,qualification='Measure force versus deflection, nut retention and creep before assigning any seating capacity')
 m.notes+=['This is a light seating-press CAD prototype. It has no bearing-installation, riveting or forming-force rating.','The metal ram reacts through the original PETG carriage and feed supports; those parts require a load/creep test.','The coupon does not deform in this demonstration. The tool stops at nominal contact.']
 return m
if __name__=='__main__':
 try:build().export(ROOT/'outputs/roam-workshop/machines/press')
 except Exception:
  import traceback;traceback.print_exc();sys.stderr.flush();os._exit(1)
 sys.stdout.flush();os._exit(0)
