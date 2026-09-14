"""Native V9 stills and freshly traced print-operation frames.

The time-lapse and real-time segments are labeled separately. The geometry
contains only extrusion completed by each sampled G-code time.
"""
from __future__ import annotations
import os,sys,json,time,hashlib,subprocess,tempfile,shutil
from concurrent.futures import ThreadPoolExecutor,as_completed
from pathlib import Path
import numpy as np
from dataclasses import replace
from PIL import Image,ImageDraw,ImageFont
from printer import *
from toolpath import parse
from mechanism_lab.core import load_cache
from mechanism_lab.photoreal import render_photoreal,compile_renderer,_invoke
from mechanism_lab import finish_render as finish
from mechanism_lab.render_profiles import V9

OUT=Path('deliverables');WORK=Path('work');FPS=24
FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
BOLD='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'


def setup():
 a=load_cache(OUT/'cache');tp=parse(OUT/'FUSE_C220_calibration.gcode');tp.prepare_beads();return a,tp


def stills(a,tp,preview=False):
 tm=tp.deposition_end*.88;state,_,_=tp.state(tm)
 a=posed(a,state,tp.geometry(tm))
 shots=['hero','printing','drive'] if not preview else ['hero','printing']
 for view in shots:
  size=(1920,1440) if view!='printing' else (1920,1280)
  spp=256
  if preview:size=(960,720);spp=32
  name=(WORK/f'preview_{view}.png') if preview else OUT/f'FUSE_C220_{view}.png'
  print('RENDER',name,flush=True)
  r=render_photoreal(a,name,view,size=size,spp=spp,threads=8,depth=V9.reference_still_depth)
  print('DONE',view,'seconds',r['seconds'],flush=True)


def label(im,mode,layer,z,simtime):
 im=im.copy();d=ImageDraw.Draw(im)
 d.rounded_rectangle((16,14,im.width-16,66),radius=7,fill=(17,23,25))
 f=ImageFont.truetype(BOLD,17);s=ImageFont.truetype(FONT,12)
 d.text((30,23),'CYBR FUSE / C220',font=f,fill=(215,228,223))
 d.text((30,45),mode,font=s,fill=(102,208,182))
 text=f'Layer {layer+1:03d} / 180     Z {z:05.2f} mm'
 d.text((im.width-300,26),text,font=s,fill=(206,213,210))
 d.text((im.width-300,44),'G-code drives nozzle + bed + deposited beads',font=s,fill=(147,166,160))
 return im


def film(a,tp,only_frames=None):
 output=OUT/'FUSE_C220_printing.mp4';frames=WORK/'film_frames';frames.mkdir(exist_ok=True)
 ex=compile_renderer();logs=[]
 # 2 seconds of variable-speed 180-layer time-lapse, then 2 seconds at true modeled time.
 # 96 independent native renders. Camera geometry and sampled print states are recorded.
 schedule=[]
 start=next(m.t0 for m in tp.moves if m.deposits)
 for i in range(48):
  u=i/95 if i<24 else (23/95+(1-23/95)*(i-23)/24)
  tm=start+(tp.deposition_end-start)*u
  schedule.append((tm,'printing','180-layer time-lapse / compressed print time'))
 live_start=tp.deposition_end-3.0
 for i in range(48):schedule.append((live_start+i/FPS,'printing','1x motion / acceleration-limited extrusion'))
 def render_frame(job):
  i,(tm,viewname,mode)=job
  fpath=frames/f'{i:05d}.png';info=frames/f'{i:05d}.json'
  if fpath.exists() and info.exists():return json.loads(info.read_text())
  temporary=tempfile.TemporaryDirectory(prefix=f'fuse_c220_frame_{i:05d}_')
  scratch=Path(temporary.name);mesh=scratch/'scene.meshbin';ppm=scratch/'frame.ppm'
  state,mi,u=tp.state(tm);dep=tp.geometry(tm);assembly=posed(a,state,dep)
  view=assembly.views[viewname]
  # Height fixed so the complete growing print stays within the close shot.
  view=replace(view,scale=122,target=(0,-9,130),f_stop=11)
  begin=time.time()
  try:
   _invoke(ex,assembly,view,mesh,ppm,(960,720),48,2,10,2026,time_seconds=0,log=scratch/'native.log')
  finally:
   if (scratch/'native.log').exists():shutil.copyfile(scratch/'native.log',WORK/f'native_film_{i:05d}.log')
  hdr=finish.read_pfm(str(ppm)+'.pfm')
  with open(str(ppm)+'.guides','rb') as stream:
   w,h=np.fromfile(stream,'<u4',2);guides=np.fromfile(stream,'<f4').reshape(h,w,9)
  im=finish.finish_frame(hdr,guides,view.exposure,V9.filter_passes,view.tone_mapping)
  im=label(im,mode,tp.moves[mi].layer,state.z,tm);finish.save_png(im,fpath)
  item={'frame':i,'gcode_time_s':tm,'mode':mode,'state':state.__dict__,'layer':tp.moves[mi].layer,'move_index':mi,'move_fraction':u,'deposition_triangles':sum(len(p.faces) for p in dep),'seconds_to_render':time.time()-begin,'sha256':hashlib.sha256(fpath.read_bytes()).hexdigest()}
  info.write_text(json.dumps(item))
  temporary.cleanup()
  return item
 with ThreadPoolExecutor(max_workers=4) as pool:
  jobs=[job for job in enumerate(schedule) if only_frames is None or job[0] in only_frames]
  futures=[pool.submit(render_frame,job) for job in jobs]
  for future in as_completed(futures):
   item=future.result();logs.append(item)
   print('COMPLETE',len(logs),len(schedule),'frame',item['frame'],'render_s',round(item['seconds_to_render'],2),'z',round(item['state']['z'],2),flush=True)
 logs.sort(key=lambda r:r['frame'])
 if only_frames is not None:
  print('SELECTED FRAMES DONE',flush=True);return
 encode_film(output,frames,logs)


def encode_film(output,frames,logs):
 assert [r['frame'] for r in logs]==list(range(96)), 'All 96 frame records are required'
 subprocess.run(['ffmpeg','-y','-v','error','-framerate',str(FPS),'-i',str(frames/'%05d.png'),'-c:v','libx264','-preset','slow','-crf','16','-pix_fmt','yuv420p','-movflags','+faststart',str(output)],check=True)
 report={'renderer':'CYBR GEO native V9 thin-lens GGX/MIS path tracer','frames':len(logs),'fps':FPS,'duration_s':len(logs)/FPS,'resolution':[960,720],'spp':48,'bounce_limit':10,'filter_passes':V9.filter_passes,'fixed_sampling_seed':2026,'frame_interpolation':False,'denoiser':'3 native geometry-guided atrous passes; no generative imagery','time_lapse_seconds':2,'real_time_seconds':2,'frames_detail':logs}
 output.with_suffix('.video.json').write_text(json.dumps(report,indent=2))
 print('FILM DONE',flush=True)

if __name__=='__main__':
 mode=sys.argv[1] if len(sys.argv)>1 else 'stills'
 if mode=='encode':
  frames=WORK/'film_frames'
  logs=[json.loads((frames/f'{i:05d}.json').read_text()) for i in range(96)]
  encode_film(OUT/'FUSE_C220_printing.mp4',frames,logs);sys.exit(0)
 a,tp=setup()
 if mode=='preview':stills(a,tp,True)
 elif mode=='stills':stills(a,tp)
 elif mode=='film':film(a,tp)
 elif mode=='frames':film(a,tp,{int(i) for i in sys.argv[2:]})
