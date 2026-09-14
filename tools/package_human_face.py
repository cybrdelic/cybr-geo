"""Collect completed portrait renders, linear masters and the textured GLB."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys
import zipfile

import numpy as np
from PIL import Image
import mitsuba as mi

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
import orbit_mitsuba_photo as v9


def atomic_png(linear,path):
    temporary=path.with_name(path.stem+'.writing.png')
    v9.save_png(linear,temporary,1.)
    with Image.open(temporary) as im:im.load()
    temporary.replace(path)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,default=ROOT/'build/human_face_final')
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    masters=args.out/'linear_masters';masters.mkdir(exist_ok=True)
    receipts=[]
    for stem in ['CYBR_Face_portrait','CYBR_Face_profile','CYBR_Face_portrait_clay']:
        report=json.loads((args.input/(stem+'.json')).read_text())
        # Write directly from the saved linear master. This is exactly v9's
        # tone mapping, with no beauty repainting, sharpening, or resampling.
        linear=v9.read_pfm(args.input/(stem+'_denoised.pfm'))
        raw=v9.read_pfm(args.input/(stem+'_beauty.pfm'))
        if not np.isfinite(linear).all() or not np.isfinite(raw).all():
            raise ValueError(f'Invalid radiance in {stem}')
        final=args.out/(stem+'.png');atomic_png(linear,final)
        raw_file=args.out/(stem+'_raw.png');atomic_png(raw,raw_file)
        for p in [final,raw_file]:
            with Image.open(p) as im:
                im.load()
                if im.size!=tuple(report['resolution']):
                    raise ValueError('Rendered resolution does not match receipt')
        mi.Bitmap(raw).write(str(masters/(stem+'_beauty_linear.exr')))
        mi.Bitmap(linear).write(str(masters/(stem+'_denoised_linear.exr')))
        report['delivered_png_sha256']=hashlib.sha256(final.read_bytes()).hexdigest()
        report['delivered_png_pixels_sha256']=hashlib.sha256(np.asarray(Image.open(final)).tobytes()).hexdigest()
        receipts.append(report)
    shutil.copy2(args.input/'CYBR_Human_Face.glb',args.out/'CYBR_Human_Face.glb')
    shutil.copy2(ROOT/'assets/portrait/LeePerrySmith_License.txt',args.out/'SCAN_LICENSE.txt')
    shutil.copy2(ROOT/'assets/portrait/sources.json',args.out/'input_sources.json')
    shutil.copy2(ROOT/'docs/human_face_environment.json',args.out/'render_environment.json')
    (args.out/'render_evidence.json').write_text(json.dumps(receipts,indent=2)+'\n')
    (args.out/'README.txt').write_text(
        'CYBR GEO / HUMAN FACE / V9 PORTRAIT\n\n'
        'Beauty: 1440 x 1800, 512 spp. Profile: 1200 x 1500, 384 spp.\n'
        'Clay geometry proof: 960 x 1200, 128 spp. All views: 14-bounce limit.\n'
        'Actual ORBIT v9 Mitsuba/LLVM path tracing, guided OIDN, v9 tone mapping.\n'
        'All images are rendered from 3D geometry. No image generation.\n\n'
        'ANATOMY CREDIT\n'
        'Infinite, 3D Head Scan by Lee Perry-Smith. CC BY 3.0 Unported.\n'
        'The original scan has closed eyelids; these are retained.\n'
        'This is scan-derived anatomy, with subdivision and geometric relief.\n'
        'The skin material is an approximation; it has no multilayer BSSRDF.\n'
        'Source color/normal maps: 1024 px. Source displacement: 4096 px.\n'
        'Workshop environment: Poly Haven small_workshop, CC0.\n\n'
        'FILES\n'
        'PNG: final tone-mapped images and un-denoised comparisons.\n'
        'linear_masters/: linear float EXRs before tone mapping.\n'
        'CYBR_Human_Face.glb: 282,944 triangles, embedded color/normal textures.\n'
        'The model is in metres and Y-up for standard glTF viewers.\n'
        'render_evidence.json: actual render settings, timings and checksums.\n\n'
        'SOURCE AND REPRODUCTION\n'
        'https://github.com/cybrdelic/cybr-geo/tree/feat/human-face-v9\n'
        'See docs/HUMAN_FACE.md, examples/human_face.py and tools/render_human_face.py.\n'
        'Project code retains GPL-2.0; scan rights are separately attributed above.\n')
    archive=args.out/'CYBR_Face_Render_Package.zip'
    files=sorted(p for p in args.out.rglob('*') if p.is_file() and p!=archive)
    with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=5) as z:
        for path in files:z.write(path,path.relative_to(args.out))
    print(json.dumps({'files':[str(p) for p in files],
                      'archive':str(archive),'archive_bytes':archive.stat().st_size},indent=2))


if __name__=='__main__':
    main()
