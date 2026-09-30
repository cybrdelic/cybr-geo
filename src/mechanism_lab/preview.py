"""A complete, offline first-run path using the in-house photographic renderer."""
from __future__ import annotations

import hashlib
import json
import shutil
from pathlib import Path

from PIL import Image

from .core import project_root
from .registry import cache_directory, load
from .truth import assert_renderable


def preview(args):
    if not shutil.which('g++'):
        raise RuntimeError('Native preview requires g++ with OpenMP. On Ubuntu/WSL: sudo apt install g++')
    if min(*args.size,args.spp,args.depth,args.threads)<1:
        raise ValueError('Image dimensions, samples, depth and threads must be positive')
    assembly=load(args.recipe,rebuild=args.rebuild,analytic=args.step)
    if args.view not in assembly.views:
        raise ValueError(f'Unknown view {args.view!r}; available: {", ".join(assembly.views)}')
    assert_renderable(assembly,args.intent,args.allow_estimates)
    if args.step and any(part.cad is None for part in assembly.parts):
        raise ValueError('STEP needs analytic CAD for every part; omit --step for mesh recipes')
    output=(args.out or project_root()/'outputs'/assembly.name/'preview').expanduser().resolve()
    output.mkdir(parents=True,exist_ok=True)
    # Remove an earlier success marker before attempting a replacement build.
    receipt=output/'preview.json';receipt.unlink(missing_ok=True)
    from .exporters import export_glb,export_bom,export_step
    from .photoreal import render_photoreal
    glb=export_glb(assembly,output/(assembly.name+'.glb'))
    export_bom(assembly,output)
    if args.step:export_step(assembly,output/(assembly.name+'.step'))
    shutil.copy2(cache_directory(args.recipe)/'validation.json',output/'validation.json')
    image=output/'hero.png'
    render=render_photoreal(assembly,image,args.view,args.size,args.spp,args.threads,args.depth,
                          args.intent,args.allow_estimates)
    with Image.open(image) as frame:
        frame.load()
        if frame.size!=tuple(args.size):raise RuntimeError('Rendered dimensions do not match the requested preview')
    # Reading the GLB checks the published artifact, not just in-memory arrays.
    import trimesh
    exported=trimesh.load_scene(glb,process=False)
    if not exported.geometry:raise RuntimeError('Exported GLB contains no geometry')
    files={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(output.iterdir()) if p.is_file()}
    report={'passed':True,'model':assembly.name,'view':args.view,'parts':len(assembly.parts),
            'renderer':render['renderer'],'resolution':list(args.size),'spp':args.spp,
            'output':str(output),'files_sha256':files,
            'scope':'Geometry, GLB readback and native image checks. Preview samples are not final quality.'}
    receipt.write_text(json.dumps(report,indent=2)+'\n')
    return report
