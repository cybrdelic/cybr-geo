"""Create the delivery archive, excluding intermediate renders and caches."""
from pathlib import Path
import zipfile,json,hashlib
R=Path(__file__).resolve().parents[1]
required=['README.md','environment.json','validation.json','render_manifest.json','reference_hero_crop.png',
          'geometry/manifest.json','geometry/tessellation_validation.json','geometry/TORSEN_X_reference_rebuild.glb',
          'geometry/scene.meshbin','geometry/mesh_arrays.npz']
for name in ['hero','rear','front','side','internals']:
    required += [f'renders/{name}.png',f'renders/{name}_raw.png',f'logs/{name}.log']
required += ['renders/multiangle.png','renders/reference_comparison.png']
required += [str(p.relative_to(R)) for p in sorted((R/'geometry').glob('*.step'))]
for name in ['build_geometry.py','optimize_geometry.py','pathtrace.cpp','finish_render.py','render_all.py','validate.py','make_plates.py','package_release.py']:
    required.append('src/'+name)
for f in required:
    if not (R/f).is_file():raise FileNotFoundError(f)
checksums={f:hashlib.sha256((R/f).read_bytes()).hexdigest() for f in required}
(R/'SHA256SUMS.json').write_text(json.dumps(checksums,indent=2))
required.append('SHA256SUMS.json')
path=R.parent/'TORSEN_X_reference_rebuild.zip'
with zipfile.ZipFile(path,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for f in required:z.write(R/f,'TORSEN_X_reference_rebuild/'+f)
with zipfile.ZipFile(path) as z:
    failure=z.testzip()
    if failure:raise ValueError('Archive CRC failure: '+failure)
print(path,path.stat().st_size,'bytes',len(required),'files',flush=True)
