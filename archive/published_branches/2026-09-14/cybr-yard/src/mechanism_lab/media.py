"""Reusable shot lists, per-frame rigid animation, FFmpeg encoding and part catalogues."""
from __future__ import annotations
from dataclasses import dataclass,replace
from pathlib import Path
import hashlib,json,math,subprocess,time,html
import numpy as np
from PIL import Image,ImageDraw
from .core import Assembly,View
from .render import Studio,labelled,font,render_still
from .exporters import export_glb

@dataclass
class Shot:
    view: str='hero'
    duration: float=4.0
    action: str='motion' # motion, orbit, explode, still
    orbit_degrees: float=30.0
    title: str=''


def probe(path):
    result=subprocess.run(['ffprobe','-v','error','-count_frames','-show_entries','stream=width,height,nb_read_frames,r_frame_rate:format=duration','-of','json',str(path)],capture_output=True,text=True,check=True)
    return json.loads(result.stdout)

def make_gif(video,output,width=640,fps=10,seconds=6.,start=0.):
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    subprocess.run(['ffmpeg','-y','-v','error','-threads','2','-ss',str(start),'-t',str(seconds),'-i',str(video),
                    '-filter_complex',f'fps={fps},scale={width}:-1:flags=lanczos,split[a][b];[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=sierra2_4a',
                    '-loop','0',str(output)],check=True)
    return output

def render_video(assembly,output,shots=None,size=(1280,720),fps=24):
    if fps<=0 or any(n<=0 or n%2 for n in size):raise ValueError('fps must be positive and H.264 dimensions must be even')
    if shots is None:shots=[Shot('hero',8,'motion')]
    if any(s.duration<=0 or round(s.duration*fps)<1 or s.view not in assembly.views or s.action not in ('motion','orbit','explode','still') for s in shots):raise ValueError('Invalid shot duration, action or view')
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True);part=output.with_name(output.stem+'.partial.mp4')
    command=['ffmpeg','-y','-v','error','-threads','2','-f','rawvideo','-pix_fmt','rgb24','-s',f'{size[0]}x{size[1]}','-r',str(fps),'-i','-',
             '-an','-c:v','libx264','-threads','2','-preset','medium','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(part)]
    proc=subprocess.Popen(command,stdin=subprocess.PIPE);rows=[];globalframe=0;start=time.time()
    try:
        for si,shot in enumerate(shots):
            view=assembly.views[shot.view];n=round(shot.duration*fps);studio=Studio(assembly,size=size,section=view.section)
            try:
                studio.visible(lambda p:p.group not in view.hide)
                for f in range(n):
                    local=f/fps;t=globalframe/fps;u=f/max(1,n-1)
                    explosion=(.5-.5*math.cos(math.tau*u)) if shot.action=='explode' else view.explode
                    angle=view.az+shot.orbit_degrees*(u-.5) if shot.action=='orbit' else view.az
                    # Each frame is a new geometry render. Stationary parts stay stationary.
                    studio.pose(t if shot.action in ['motion','orbit'] else 0,explosion)
                    studio.set_camera(angle,view.el,view.scale,view.target)
                    im=studio.render();digest=hashlib.sha256(im.tobytes()).hexdigest()
                    footer=f'{t:05.2f} s  |  {fps} fps  |  {shot.action.upper()}  |  prescribed geometry motion, not a force simulation'
                    im=labelled(im,shot.title or view.title or assembly.name.upper(),view.note,footer)
                    proc.stdin.write(np.ascontiguousarray(im,dtype=np.uint8).tobytes())
                    rows.append(dict(frame=globalframe,shot=si,time=t,explosion=explosion,azimuth=angle,pixel_sha256_before_caption=digest))
                    globalframe+=1
                print(f'{assembly.name}: shot {si+1}/{len(shots)}, {globalframe} frames',flush=True)
            finally:studio.close()
        proc.stdin.close();code=proc.wait()
        if code:raise RuntimeError(f'FFmpeg failed with exit code {code}')
        part.replace(output)
    except BaseException:
        try:proc.stdin.close()
        except (BrokenPipeError,OSError):pass
        proc.kill();proc.wait();part.unlink(missing_ok=True);raise
    report=dict(model=assembly.name,frames=globalframe,unique_frames=len({r['pixel_sha256_before_caption'] for r in rows}),
                duration=globalframe/fps,resolution=list(size),fps=fps,seconds_to_render=time.time()-start,
                method='VTK EGL PBR, frame-by-frame geometry rendering; not path-traced',probe=probe(output),frames_log=rows)
    output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    return report

def catalogue(assembly,directory,size=(640,480)):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True);models=directory/'models';models.mkdir(exist_ok=True)
    studio=Studio(assembly,size=size);items=[];cards=[]
    try:
        for i,p in enumerate(assembly.parts):
            studio.visible(lambda q:q.name==p.name);studio.pose(0,0)
            T=assembly.pose(p);v=p.vertices@T[:3,:3].T+T[:3,3];lo,hi=v.min(0),v.max(0);target=(lo+hi)/2
            scale=max(6.,np.linalg.norm(hi-lo)*.52);studio.set_camera(45,26,scale,target)
            im=studio.render();card=labelled(im,f'{i+1:03} / '+p.name,'',p.provenance)
            filename=f'{i+1:03}_{p.name}.png';card.save(directory/filename)
            sub=replace(assembly,parts=[p]);glb=models/(p.name+'.glb');export_glb(sub,glb)
            items.append(dict(index=i+1,name=p.name,group=p.group,provenance=p.provenance,image=filename,geometry='models/'+glb.name,role=p.role))
            thumb=card.resize((320,240));cards.append(thumb)
        for page in range(math.ceil(len(cards)/20)):
            sheet=Image.new('RGB',(1280,1200),(13,18,24))
            for k,card in enumerate(cards[page*20:(page+1)*20]):sheet.paste(card,((k%4)*320,(k//4)*240))
            sheet.save(directory/f'atlas_{page+1:02}.jpg',quality=93)
    finally:studio.close()
    tiles='\n'.join(f'<article data-name="{html.escape(q["name"].lower())}"><a href="{q["geometry"]}"><img loading="lazy" src="{q["image"]}" alt="{html.escape(q["name"])}"></a><h3>{html.escape(q["name"])}</h3><p>{html.escape(q["provenance"])}</p><p>{html.escape(q["role"])}</p></article>' for q in items)
    (directory/'index.html').write_text('''<!doctype html><html lang="en"><meta charset="utf-8"><title>Component catalogue</title><style>body{background:#10161d;color:#d8e2eb;font:15px system-ui;margin:28px}main{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:20px}img{width:100%}h3{overflow-wrap:anywhere;font-size:14px}input{padding:12px;width:90%;margin:20px 0}p{color:#a6b6c2}</style><h1>'''+html.escape(assembly.name)+''' / component catalogue</h1><p>Click a rendering for its actual GLB geometry. All files work offline. Named visual components are not a manufacturing bill of materials.</p><input id="filter" placeholder="Filter part names"><main>'''+tiles+'''</main><script>document.querySelector('#filter').oninput=e=>document.querySelectorAll('article').forEach(a=>a.hidden=!a.dataset.name.includes(e.target.value.toLowerCase()))</script></html>''')
    (directory/'index.json').write_text(json.dumps(items,indent=2)+'\n');return items
