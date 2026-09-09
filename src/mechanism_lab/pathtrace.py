"""Generic adapter for the preserved C++ BVH/GGX/MIS path tracer.

A material sidecar replaces the old hard-coded differential material palette.
Raw and classically filtered images are retained; no neural image generation.
"""
from __future__ import annotations
from pathlib import Path
from dataclasses import replace
import subprocess,tempfile,json,time
import numpy as np
from PIL import Image
from .core import project_root,Part
from .exporters import export_meshbin

def compile_renderer():
    root=project_root();build=root/'.build/native';exe=build/'mechanism_pathtrace'
    subprocess.run(['cmake','-S',str(root/'native'),'-B',str(build),'-DCMAKE_BUILD_TYPE=Release'],check=True)
    subprocess.run(['cmake','--build',str(build),'--parallel','4'],check=True)
    return exe

def render_pathtrace(assembly,output,view_name='hero',size=(1600,1100),spp=64,threads=4,depth=7,exposure=1.3):
    from .render import labelled,clip_closed,polydata
    from vtk.util.numpy_support import vtk_to_numpy
    import vtk
    from . import finish_render as filt
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    view=assembly.views[view_name];parts=[p for p in assembly.parts if p.group not in view.hide]
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
             '--ortho','--az',str(view.az),'--el',str(view.el),'--scale',str(2*view.scale),
             '--tx',str(view.target[0]),'--ty',str(view.target[1]),'--tz',str(view.target[2]),
             '--camera-studio','--studio-scale',str(max(1.,view.scale/85)),'--no-floor']
        with output.with_suffix('.log').open('w') as log:subprocess.run(cmd,stdout=log,stderr=log,check=True)
        arr=filt.read_pfm(str(ppm)+'.pfm')
        with open(str(ppm)+'.guides','rb') as f:
            w,h=np.fromfile(f,'<u4',2);guides=np.fromfile(f,'<f4').reshape(h,w,9)
        Image.fromarray(filt.tonemap(arr,exposure=exposure)).save(output.with_name(output.stem+'_raw.png'))
        variance=guides[:,:,7].copy()
        for i in range(3):arr,variance=filt.atrous(arr,guides,variance,2**i,i)
        clean=Image.fromarray(filt.tonemap(arr,exposure=exposure));clean.save(output.with_name(output.stem+'_clean.png'))
        labelled(clean,view.title or assembly.name.upper(),view.note,
                 f'{spp} samples/pixel / {depth}-bounce limit / classical geometry-guided filtering',
                 tag='CYBR MECHANISM LAB / PATH-TRACED GEOMETRY').save(output)
    report=dict(file=output.name,model=assembly.name,view=view_name,resolution=list(size),spp=spp,bounce_limit=depth,
                triangles=sum(len(p.faces) for p in parts),seconds=time.time()-start,exposure=exposure,lighting='camera-relative procedural studio',renderer='C++ BVH/GGX/MIS path tracer; no neural filtering')
    output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n');return report
