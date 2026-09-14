"""Rebuild the delivered motor/drivetrain artifacts via the reusable public API."""
from pathlib import Path
import json,time
from mechanism_lab.registry import load
from mechanism_lab.core import project_root,validate
from mechanism_lab.exporters import export_glb,export_step,export_bom,export_animated_glb
from mechanism_lab.render import render_still
from mechanism_lab.whiteprint import whiteprint
from mechanism_lab.media import render_video,Shot,make_gif,catalogue

root=project_root();media=root/'media';start=time.time()
# Native CAD is retained during this build, not reconstructed from triangles.
a=load('m8325s',rebuild=True,analytic=True)
out=root/'outputs/m8325s';out.mkdir(exist_ok=True)
export_glb(a,out/'m8325s.glb');export_step(a,out/'m8325s_analytic.step');export_bom(a,out)
export_animated_glb(a,out/'m8325s_motion.glb',duration=10)
export_animated_glb(a,out/'m8325s_exploded.glb',duration=8,mode='explode')
whiteprint(a,out/'drawings/m8325s_whiteprint')
for view in ['hero','rear','internal','section','exploded','stator']:
 render_still(a,media/f'motor_{view}.png',view,size=(1600,1100));print('motor still',view,flush=True)
(root/'validation/motor_geometry.json').write_text(json.dumps(validate(a,expensive=True),indent=2))
print('motor assets built',time.time()-start,flush=True)
b=load('drivetrain',rebuild=True,analytic=True);out2=root/'outputs/drivetrain';out2.mkdir(exist_ok=True)
export_glb(b,out2/'drivetrain.glb');export_step(b,out2/'drivetrain_analytic.step',individual=False);export_bom(b,out2)
export_animated_glb(b,out2/'drivetrain_motion.glb',duration=8)
for view in ['hero','drive_face','internal','exploded']:
 render_still(b,media/f'drivetrain_{view}.png',view,size=(1800,1100));print('drivetrain still',view,flush=True)
for name in ['Drive_01_99p6_BCD_carrier_adapter','Drive_04_M3_face_adapter']:
 whiteprint(b,out2/'drawings'/name,part_names=[name],title=name.replace('_',' '))
(root/'validation/drivetrain_geometry.json').write_text(json.dumps(validate(b),indent=2))
print('drive assets built',time.time()-start,flush=True)
render_video(a,media/'motor_inspection.mp4',[Shot('hero',4,'orbit',45),Shot('exploded',6,'explode'),Shot('internal',5,'motion'),Shot('stator',3,'orbit',35),Shot('section',4,'motion')])
make_gif(media/'motor_inspection.mp4',media/'motor_exploded.gif',seconds=6,start=4)
make_gif(media/'motor_inspection.mp4',media/'motor_internal.gif',seconds=5,start=10)
render_video(b,media/'drivetrain_working.mp4',[Shot('drive_face',6,'motion'),Shot('internal',6,'motion'),Shot('hero',4,'orbit',25)])
make_gif(media/'drivetrain_working.mp4',media/'drivetrain_working.gif',seconds=6)
print('video assets built',time.time()-start,flush=True)
catalogue(a,out/'parts');print('catalogue complete',time.time()-start,flush=True)
