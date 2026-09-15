"""Height-setting and layout fixture with a retained metal scriber."""
from vertical import *

def build():
 m,plate,attachment=upright('measure','ROAM / screw-fed height gauge');mo=['V_feed'];cx=-72
 m.fuse(plate,cyl(8,24,(-84,0,120),'X'));m.cut(plate,cyl(2.2,26,(-85,0,120),'X'))
 probe=cyl(2,70,(-132,0,120),'X').fuse(cq.Solid.makeCone(0,2,8,cq.Vector(-140,0,120),cq.Vector(1,0,0))).cut(box(15,6,6,(-72.5,-4.8,120)))
 m.add('P01_Metal_scriber',probe,2,mo,'Original 4 mm steel scriber with a ground conical point and locking flat','Cut/file steel rod; point and height require calibration',plate,fit=.21)
 m.socket('P02_Scriber_lock',plate,(cx,-8,120),'Y',diam=3,length=6.2)
 m.connect('P01_Metal_scriber','P02_Scriber_lock','Flat contact',0)
 block=m.add('W01_80mm_Reference',box(20,30,80,(-150,0,40)),5,role='Nominal 80 mm reference block; tip contacts its upper edge at minimum feed',make='Reference model; calibrate against a measured standard',parent='B00_Bench_plate',fit=0)
 for y in [-8,8]:
  m.cut('B00_Bench_plate',cyl(4.1,4.8,(-150,y,-10)));m.bolt(f'W02_{y}_Reference_mount','B00_Bench_plate',block,(-150,y,-5.2),85.2)
 # The printed graduation strip is a positioning aid, not a calibrated rule.
 scale=box(2,12,180,(-1,60,130),.5)
 for z in range(40,221,5):
  width=8 if z%10==0 else 4;scale=scale.cut(box(.4,width,.6,(-1.9,60,z)))
 rule=m.add('P10_Graduation_strip',scale,0,role='Five-millimetre engraved positioning marks; dimensions must be checked against a real reference',make='Print flat; mark grooves for readability',parent='V_B01_Ribbed_base',fit=0)
 for z in [55,205]:m.bolt(f'P11_{z}_Scale_mount',rule,'V_B01_Ribbed_base',(-2,60,z),10,'X',diam=3)
 head=[plate,'P01_Metal_scriber','P02_Scriber_lock']
 m.service=[dict(part='P01_Metal_scriber',remove_first=['P02_Scriber_lock'],axis='X',distance_mm=-90,step_mm=2,description='Raise to datum height 120 mm, remove locking screw, then withdraw scriber forward.'),dict(parts=head,remove_first=attachment,axis='X',distance_mm=-120,step_mm=2,description='At datum height, support and withdraw complete scriber head after removing four mounting fastener sets.')]
 m.requirements.update(operation='Set scriber height with the retained screw feed; the point reaches the reference block upper edge at 80 mm',tip_height_mm=[80,160],reference_height_mm=80,graduation_pitch_mm=5,accuracy_rating=None)
 m.notes+=['Printed marks are nominal position references. They do not turn PETG into a calibrated measuring instrument.','The metal point is an original ground-rod design, not a detailed model of an unselected commercial scriber.','Calibrate height, straightness and repeatability against external measuring equipment before use.']
 return m
if __name__=='__main__':
 try:build().export(ROOT/'outputs/roam-workshop/machines/measure')
 except Exception:
  import traceback;traceback.print_exc();sys.stderr.flush();os._exit(1)
 sys.stdout.flush();os._exit(0)
