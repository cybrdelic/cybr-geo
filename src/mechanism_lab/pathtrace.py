"""Generic adapter for the preserved C++ BVH/GGX/MIS path tracer.

A material sidecar replaces the old hard-coded differential material palette.
Raw and classically filtered images are retained; no neural image generation.
Production renders are provenance-gated before the native renderer is invoked.
"""
from __future__ import annotations
from pathlib import Path
from dataclasses import replace
import subprocess,tempfile,json,time
import numpy as np
from PIL import Image
from .core import project_root
from .exporters import export_meshbin


def compile_renderer():
    root=project_root();build=root/'.build/native';exe=build/'mechanism_pathtrace'
    subprocess.run(['cmake','-S',str(root/'native'),'-B',str(build),'-DCMAKE_BUILD_TYPE=Release'],check=True)
    subprocess.run(['cmake','--build',str(build),'--parallel','4'],check=True)
    return exe


def render_pathtrace(assembly,output,view_name='hero',size=(1920,1080),spp=256,threads=4,depth=10,exposure=None,intent='auto',allow_estimates=False):
    from .render import labelled,clip_closed,polydata
    from .truth import assert_renderable,write_truth_report
    from vtk.util.numpy_support import vtk_to_numpy
    import vtk
    from . import finish_render as filt

    truth=assert_renderable(assembly,intent,allow_estimates)
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    view=assembly.views[view_name]
    if exposure is None:exposure=1.15*float(view.exposure)
    parts=[p for p in assembly.parts if p.group not in view.hide]
    if view.section:
        clipped=[]
        for p in parts:
            q=clip_closed(polydata(p),normal=view.section)
            tri=vtk.vtkTriangleFilter();tri.SetInputData(q);tri.Update();q=tri.GetOutput()
            if not q.GetNumberOfPoints():continue
            clipped.append(replace(p,vertices=vtk_to_numpy(q.GetPoints().GetData()).copy(),faces=vtk_to_numpy(q.GetPolys().GetConnectivityArray()).reshape(-1,3).copy(),normals=vtk_to_numpy(q.GetPointData().GetNormals()).copy(),cad=None))
        parts=clipped
    subset=replace(assembly,parts=parts);exe=compile_renderer();start=time.time()
    with tempfile.TemporaryDirectory(prefix='mechanism_pt_') as tmp:
        tmp=Path(tmp);mesh=tmp/'scene.meshbin';ppm=tmp/'render.ppm'
        export_meshbin(subset,mesh,explode=view.explode)
        cmd=[str(exe),str(mesh),str(ppm),'--materials',str(mesh.with_suffix('.materials')),
             '--w',str(size[0]),'--h',str(size[1]),'--spp',str(spp),'--depth',str(depth),'--threads',str(threads),
             '--az',str(view.az),'--el',str(view.el),'--scale',str(2*view.scale),
             '--tx',str(view.target[0]),'--ty',str(view.target[1]),'--tz',str(view.target[2]),
             '--camera-studio','--studio-scale',str(max(1.,view.scale/85)),'--exposure',str(exposure)]
        if view.projection=='orthographic':cmd.append('--ortho')
        elif view.projection!='perspective':raise ValueError(f'Unknown projection {view.projection!r}')
        if not view.floor:cmd.append('--no-floor')
        with output.with_suffix('.log').open('w') as log:subprocess.run(cmd,stdout=log,stderr=log,check=True)
        arr=filt.read_pfm(str(ppm)+'.pfm')
        with open(str(ppm)+'.guides','rb') as f:
            w,h=np.fromfile(f,'<u4',2);guides=np.fromfile(f,'<f4').reshape(h,w,9)
        Image.fromarray(filt.tonemap(arr,exposure=exposure)).save(output.with_name(output.stem+'_raw.png'))
        variance=guides[:,:,7].copy()
        # Higher sample counts need less denoising. Preserve small mechanical detail
        # rather than smearing it with the old fixed three-pass filter.
        filter_passes=2 if spp>=192 else 3
        for i in range(filter_passes):arr,variance=filt.atrous(arr,guides,variance,2**i,i)
        clean=Image.fromarray(filt.tonemap(arr,exposure=exposure));clean.save(output.with_name(output.stem+'_clean.png'))
        counts=truth['tier_counts'];truth_line=f"{truth['resolved_intent'].upper()} / "+', '.join(f'{k}:{v}' for k,v in counts.items())
        labelled(clean,view.title or assembly.name.upper(),view.note,
                 f'{spp} spp / {depth} bounces / {view.projection} / {truth_line}',
                 tag='CYBR MECHANISM LAB / TRUTH-GATED PATH TRACE').save(output)
    elapsed=time.time()-start
    report=dict(file=output.name,model=assembly.name,view=view_name,resolution=list(size),spp=spp,bounce_limit=depth,
                projection=view.projection,triangles=sum(len(p.faces) for p in parts),seconds=elapsed,exposure=exposure,
                lighting='finite-area procedural studio + optional physical floor',
                renderer='C++ BVH/GGX/MIS path tracer; no neural filtering',truth=truth)
    output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    write_truth_report(truth,output.with_suffix('.truth.json'))
    return report
