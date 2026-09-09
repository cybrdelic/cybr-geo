"""Frame-by-frame geometry animation, inspection catalogues and checksum receipts."""
from __future__ import annotations
from pathlib import Path
from typing import Callable
import json,math,shutil,subprocess
import numpy as np
from PIL import Image,ImageDraw
from .core import Assembly
from .render import Studio,labelled,font

def video(assembly:Assembly,path:Path,trajectory:Callable,seconds=6.,fps=24,size=(1280,720),
          title='ASSEMBLY INSPECTION',subtitle='',camera=(235,23,75,(0,0,0)),
          predicate=None,section=None,gif=True):
    """trajectory(t, normalized_time) -> {poses, explode, camera}; actual meshes move.

    Camera tuple: azimuth degrees, elevation degrees, orthographic half-height,
    target xyz mm. FFmpeg input is streamed: no giant intermediate frame directory.
    """
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    if shutil.which('ffmpeg') is None:raise RuntimeError('ffmpeg is required for videos')
    if seconds<=0 or fps<=0:raise ValueError('seconds and fps must be positive')
    if size[0]%2 or size[1]%2:raise ValueError('H.264 dimensions must be even')
    frames=max(1,round(seconds*fps))
    command=['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24',
        '-s',f'{size[0]}x{size[1]}','-r',str(fps),'-i','-','-an','-c:v','libx264',
        '-preset','medium','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(path)]
    proc=subprocess.Popen(command,stdin=subprocess.PIPE);first=None;last=None
    try:
        with Studio(assembly,size,section=section) as studio:
            if predicate:studio.visible(predicate)
            for i in range(frames):
                u=i/(frames-1) if frames>1 else 0;t=i/fps
                state=trajectory(t,u);studio.pose(state.get('poses'),state.get('explode',0))
                cam=state.get('camera',camera);studio.set_camera(*cam)
                raw=studio.render();im=labelled(raw,title,subtitle,
                   f'Frame {i+1:04d}/{frames:04d} | {fps:g} fps | Prescribed rigid-body motion, not a force solver')
                if first is None:first=np.asarray(raw).copy()
                last=np.asarray(raw).copy();proc.stdin.write(im.tobytes())
                if i%(fps*2)==0:print(path.name,i,frames,flush=True)
    finally:
        if proc.stdin:proc.stdin.close()
        ret=proc.wait()
    if ret:raise RuntimeError(f'ffmpeg returned {ret}')
    probe=subprocess.check_output(['ffprobe','-v','error','-count_frames','-select_streams','v:0',
            '-show_entries','stream=width,height,nb_read_frames,r_frame_rate','-of','json',str(path)],text=True)
    result={'frames_requested':frames,'fps':fps,'seconds':frames/fps,'renderer':'VTK EGL PBR / geometry frames',
            'ffprobe':json.loads(probe),'first_last_mean_absolute_change':float(np.mean(np.abs(first.astype(float)-last.astype(float))))}
    path.with_suffix('.json').write_text(json.dumps(result,indent=2))
    if gif:
        subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-i',str(path),'-filter_complex',
          '[0:v]fps=10,scale=640:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=96[p];[b][p]paletteuse=dither=sierra2_4a',
          '-loop','0',str(path.with_suffix('.gif'))],check=True)
    return result

def catalogue(assembly,folder,size=(640,480)):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True);rows=[]
    with Studio(assembly,size) as s:
        for i,p in enumerate(assembly.parts):
            s.visible(lambda q:q.name==p.name)
            radius=np.linalg.norm(p.bounds[1]-p.bounds[0])*.52
            s.set_camera(235,24,max(radius*1.1,1),p.bounds.mean(0))
            im=labelled(s.render(),p.name[:55],p.role[:85],f'{i+1} / {len(assembly.parts)}')
            im.save(folder/f'{p.name}.png')
            rows.append({'name':p.name,'image':f'{p.name}.png','group':p.group,
                         'dimensions_mm':np.ptp(p.bounds,axis=0).tolist(),'role':p.role})
    (folder/'index.json').write_text(json.dumps(rows,indent=2))
    cols=5;tile=(320,240);nrows=math.ceil(len(rows)/cols)
    atlas=Image.new('RGB',(cols*tile[0],nrows*tile[1]),(18,22,28))
    for i,row in enumerate(rows):atlas.paste(Image.open(folder/row['image']).resize(tile),(i%cols*320,i//cols*240))
    atlas.save(folder/'atlas.jpg',quality=90)
    return rows
