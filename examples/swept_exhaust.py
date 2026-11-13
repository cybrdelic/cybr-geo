"""Four routed hollow exhaust runners. Every shape comes from the generic core.
No claim of thermal, flow, fabrication or collision validation is made.
Run from this directory with PYTHONPATH=.. python examples/swept_exhaust.py.
"""
from pathlib import Path
import json
import numpy as np
from cybrgeo import bezier, sample_curve, sweep_shell, export_obj

out=Path(__file__).resolve().parents[1]/'output'
out.mkdir(exist_ok=True)
angle=np.linspace(0,2*np.pi,48,endpoint=False)
outer=.24*np.column_stack((np.cos(angle),np.sin(angle)))
inner=.205*np.column_stack((np.cos(angle),np.sin(angle)))
steps=[]
for runner in range(4):
    x=(runner-1.5)*1.2
    # Parallel inlet tangents, independently routed midsections, axial outlets.
    controls=np.array([[x,0,2.7],[x,-1.5,2.7],[x*.6,-2.5,.3],[x*.3,-3.,-.6]])
    path=sample_curve(lambda t:bezier(controls,t),count=70,tolerance=1e-4)
    scale=np.linspace(1.,.9,len(path))
    mesh=sweep_shell(outer,inner,path,scale=scale)
    export_obj(mesh,out/f'runner_{runner}.obj')
    np.savez(out/f'runner_{runner}.npz',vertices=mesh.vertices,faces=mesh.faces,path=path,controls=controls,outer=outer,inner=inner)
    steps.append({'runner':runner,'controls':controls.tolist(),'path':path.tolist(),'outer':outer.tolist(),'inner':inner.tolist(),'mesh':f'runner_{runner}.npz'})
(out/'exhaust_steps.json').write_text(json.dumps(steps))
print('Exported four hollow runners, paths and profiles for progressive 3D construction.')
