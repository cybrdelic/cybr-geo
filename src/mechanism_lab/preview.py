"""Offline geometry preview with explicit native or platform VTK rendering."""
from __future__ import annotations

import hashlib
import json
import shutil
import os
import subprocess
import tempfile
from pathlib import Path

from PIL import Image

from .core import project_root
from .registry import cache_directory, load
from .truth import assert_renderable


def native_unavailable_reason():
    """Probe the C++17/OpenMP capability required by the native renderer.

    A compiler named g++ can be Apple Clang without OpenMP support. Compile
    and execute a minimal probe before selecting a backend; renderer-source
    or rendering failures after that selection remain errors.
    """
    compiler = shutil.which('g++')
    if compiler is None:
        return 'A g++ compiler with C++17 and OpenMP support is not available on PATH.'
    with tempfile.TemporaryDirectory(prefix='cybrgeo-compiler-') as temporary:
        root = Path(temporary)
        source = root / 'openmp.cpp'
        executable = root / ('openmp.exe' if os.name == 'nt' else 'openmp')
        source.write_text('#include <omp.h>\nint main() { return omp_get_max_threads() > 0 ? 0 : 1; }\n', encoding='utf-8')
        try:
            compiled = subprocess.run([compiler, '-std=c++17', '-fopenmp', str(source), '-o', str(executable)],
                                      capture_output=True, text=True, timeout=30)
            if compiled.returncode:
                return f'The available g++ cannot compile a C++17/OpenMP program (exit {compiled.returncode}).'
            executed = subprocess.run([str(executable)], capture_output=True, text=True, timeout=10)
            if executed.returncode:
                return f'The compiled OpenMP probe cannot run successfully (exit {executed.returncode}).'
        except FileNotFoundError:
            return 'The native compiler or its required runtime could not be found.'
        except subprocess.TimeoutExpired:
            return 'The C++17/OpenMP capability probe exceeded its time limit.'
    return None


def select_renderer(requested):
    if requested not in ('auto', 'native', 'pbr'):
        raise ValueError('Preview renderer must be auto, native, or pbr')
    reason = native_unavailable_reason() if requested != 'pbr' else None
    if reason and requested == 'native':
        raise RuntimeError('Native preview was explicitly requested. ' + reason)
    return ('pbr' if requested == 'pbr' or reason else 'native'), reason


def preview(args):
    if min(*args.size,args.spp,args.depth,args.threads)<1:
        raise ValueError('Image dimensions, samples, depth and threads must be positive')
    assembly=load(args.recipe,rebuild=args.rebuild,analytic=args.step)
    if args.view not in assembly.views:
        raise ValueError(f'Unknown view {args.view!r}; available: {", ".join(assembly.views)}')
    assert_renderable(assembly,args.intent,args.allow_estimates)
    if args.step and any(part.cad is None for part in assembly.parts):
        raise ValueError('STEP needs analytic CAD for every part; omit --step for mesh recipes')
    from .exporters import artifact_stem,export_glb,export_bom,export_step
    stem=artifact_stem(assembly)
    output=(args.out or project_root()/'outputs'/stem/'preview').expanduser().resolve()
    output.mkdir(parents=True,exist_ok=True)
    # Remove an earlier success marker before attempting a replacement build.
    receipt=output/'preview.json';receipt.unlink(missing_ok=True)
    requested = getattr(args, 'renderer', 'auto')
    renderer, fallback_reason = select_renderer(requested)
    glb=export_glb(assembly,output/(stem+'.glb'))
    export_bom(assembly,output)
    if args.step:export_step(assembly,output/(stem+'.step'))
    shutil.copy2(cache_directory(args.recipe)/'validation.json',output/'validation.json')
    image=output/'hero.png'
    if renderer == 'native':
        from .photoreal import render_photoreal
        render=render_photoreal(assembly,image,args.view,args.size,args.spp,args.threads,args.depth,
                              args.intent,args.allow_estimates)
        render['backend'] = 'native-pathtrace'
    else:
        from .render import render_still
        render_still(assembly,image,args.view,args.size,captions=False,
                     intent=args.intent,allow_estimates=args.allow_estimates,supersample=2)
        render=json.loads(image.with_suffix('.render.json').read_text(encoding='utf-8'))
    with Image.open(image) as frame:
        frame.load()
        if frame.size!=tuple(args.size):raise RuntimeError('Rendered dimensions do not match the requested preview')
        if not any(low < high for low, high in frame.convert('RGB').getextrema()):
            raise RuntimeError('Rendered preview is a uniform image; check the graphics context and renderer')
    # Reading the GLB checks the published artifact, not just in-memory arrays.
    import trimesh
    exported=trimesh.load_scene(glb,process=False)
    if not exported.geometry:raise RuntimeError('Exported GLB contains no geometry')
    files={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(output.iterdir()) if p.is_file()}
    report={'passed':True,'model':assembly.name,'view':args.view,'parts':len(assembly.parts),
            'renderer':render['renderer'],'backend':render['backend'],'requested_renderer':requested,
            'fallback_reason':fallback_reason,'resolution':list(args.size),'spp':render.get('spp'),
            'render_window':render.get('render_window'),'supersample':render.get('supersample'),
            'output':str(output),'files_sha256':files,
            'scope':'Geometry, GLB readback and actual rendered image checks. Preview is not dynamics or hardware qualification.'}
    receipt.write_text(json.dumps(report,indent=2)+'\n')
    return report
