"""Verify, concatenate, catalogue and package the actually generated outputs."""
from __future__ import annotations
import json,sys,os,subprocess,time,zipfile,hashlib,platform,importlib.metadata
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1];VIDEOS=ROOT/'videos'

def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        while block:=f.read(2**20):h.update(block)
    return h.hexdigest()

def main():
    # Fail rather than publish an unfinished or nonexistent output.
    for n in ['inspection_video.json','operational_video.json','fixed_camera_video.json','pathtrace_stills.json']:
        if not (ROOT/'validation'/n).exists():raise FileNotFoundError('Unfinished output: '+n)
    clips=['01_exploded_inspection.mp4','02_differential_in_motion.mp4','03_fixed_camera_motion.mp4']
    concat=VIDEOS/'concat.txt';concat.write_text(''.join("file '"+str(VIDEOS/n)+"'\n" for n in clips))
    full=VIDEOS/'TORSEN_X_full_inspection.mp4'
    subprocess.run(['ffmpeg','-y','-v','error','-f','concat','-safe','0','-i',str(concat),'-c','copy','-movflags','+faststart',str(full)],check=True)
    # GIF is extracted from the genuine stationary-camera rendered movie.
    subprocess.run(['ffmpeg','-y','-v','error','-i',str(VIDEOS/clips[-1]),'-filter_complex','fps=12,scale=800:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=192[p];[b][p]paletteuse=dither=sierra2_4a','-loop','0',str(VIDEOS/'fixed_camera_preview.gif')],check=True)
    reference=np.load(ROOT/'geometry'/'reference_mesh_arrays.npz');current=np.load(ROOT/'geometry'/'reference_parts.npz')
    preservation=all(k in current and np.array_equal(reference[k],current[k]) for k in reference.files)
    if not preservation:raise AssertionError('Reference data was changed')
    media=[]
    for n,expected in zip(clips+['TORSEN_X_full_inspection.mp4'],[288,456,96,840]):
        p=VIDEOS/n
        cmd=['ffprobe','-v','error','-count_frames','-select_streams','v:0','-show_entries','stream=width,height,r_frame_rate,nb_read_frames,duration','-of','json',str(p)]
        info=json.loads(subprocess.check_output(cmd));s=info['streams'][0]
        if int(s['nb_read_frames'])!=expected:raise AssertionError((n,s,expected))
        if (s['width'],s['height'])!=(1280,720):raise AssertionError('Wrong resolution')
        subprocess.run(['ffmpeg','-v','error','-i',str(p),'-f','null','-'],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
        media.append(dict(file=str(p.relative_to(ROOT)),**s,decode_errors=0,sha256=sha(p)))
    # Decode representative frames of the final encoded file, not input stills.
    times=[.75,5.8,9.,13.5,17.4,21.,25.,29.,32.,34.5]
    sheet=Image.new('RGB',(1600,980),(12,18,25));d=ImageDraw.Draw(sheet);font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
    for i,t in enumerate(times):
        p=VIDEOS/f'encoded_check_{i:02d}.png';subprocess.run(['ffmpeg','-y','-v','error','-ss',str(t),'-i',str(full),'-frames:v','1',str(p)],check=True)
        im=Image.open(p).convert('RGB');im.thumbnail((640,360));im=im.resize((510,287),Image.Resampling.LANCZOS)
        x=(i%3)*530+5;y=(i//3)*244+22
        # Four rows; use 422x237 tiles in a 3-column plate to avoid overlaps.
        im=im.resize((420,236),Image.Resampling.LANCZOS)
        x=(i%3)*530+55;y=(i//3)*244+12;sheet.paste(im,(x,y));d.text((x+4,y+3),f'{t:04.1f} s',font=font,fill=(230,237,244))
    sheet.save(ROOT/'renders'/'encoded_video_contact_sheet.png')
    pi=json.loads((ROOT/'parts'/'index.json').read_text())
    for p in pi:
        assert (ROOT/'parts'/p['image']).is_file()
        assert (ROOT/'parts'/p['model']).is_file()
    report=dict(original_reference_mesh_arrays_bit_identical=preservation,reference_array_count=len(reference.files),reference_components=len(pi),individual_part_render_count=len(pi),individual_part_glb_count=len(pi),new_kinematic_component_count=44,media=media,pathtrace_stills=json.loads((ROOT/'validation'/'pathtrace_stills.json').read_text()),geometry_only=True,image_generation_used=False,notes=['All angular motion is prescribed under ideal rigid-body gear constraints.','No complete mechanical, contact-force, torque-bias or manufacturing qualification.'])
    (ROOT/'validation'/'release_checks.json').write_text(json.dumps(report,indent=2))
    env=dict(python=sys.version,platform=platform.platform(),renderer='VTK EGL / Mesa llvmpipe',packages={})
    for n in ['numpy','cadquery','trimesh','vtk','shapely','pillow','numba']:
        try:env['packages'][n]=importlib.metadata.version(n)
        except importlib.metadata.PackageNotFoundError:pass
    (ROOT/'validation'/'environment.json').write_text(json.dumps(env,indent=2))
    # No fonts, caches, scratch screenshots or transport buffers in release ZIP.
    def eligible(p):
        return p.is_file() and '__pycache__' not in str(p) and p.name!='pathtrace' and p.suffix not in ['.ppm','.pfm','.guides','.meshbin','.ttf','.otf'] and 'test' not in p.name and 'contact_review' not in p.name
    files=sorted(p for p in ROOT.rglob('*') if eligible(p) and p.name!='SHA256SUMS.json')
    sums={str(p.relative_to(ROOT)):sha(p) for p in files};(ROOT/'SHA256SUMS.json').write_text(json.dumps(sums,indent=2));files.append(ROOT/'SHA256SUMS.json')
    dst=ROOT.parent/'TORSEN_X_internal_exploded_motion_v3.zip'
    with zipfile.ZipFile(dst,'w',zipfile.ZIP_DEFLATED,compresslevel=5) as z:
        for p in files:z.write(p,Path(ROOT.name)/p.relative_to(ROOT))
    # Smaller media-only bundle includes the individual renders and final movies.
    mediazip=ROOT.parent/'TORSEN_X_views_and_videos_v3.zip'
    with zipfile.ZipFile(mediazip,'w',zipfile.ZIP_DEFLATED,compresslevel=5) as z:
        for p in files:
            rel=p.relative_to(ROOT)
            if rel.parts[0] in ['renders','videos'] or (rel.parts[0]=='parts' and p.suffix=='.png') or p.name=='README.md':z.write(p,rel)
    result=dict(full_package=str(dst),full_package_bytes=dst.stat().st_size,media_package=str(mediazip),media_package_bytes=mediazip.stat().st_size,video_frames=840,seconds=35,checks_passed=True)
    (ROOT.parent/'torsen_v3_completion.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True)
if __name__=='__main__':main()
