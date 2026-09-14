"""Self-contained WebGL inspector made from the exported CYBR GEO geometry.

This fast interactive preview is explicitly not the native V9 offline renderer.
"""
from __future__ import annotations
import argparse,base64,json,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src'))
from mechanism_lab.core import load_cache

def create_viewer(out:Path):
    a=load_cache(out/'geometry');batches={}
    for p in a.parts:
        key=(p.group,tuple(np.asarray(p.explode,float)))
        batches.setdefault(key,[]).append(p)
    vertices=[];indices=[];groups=[];vertex_offset=0;index_offset=0
    for (group,explode),parts in batches.items():
        start=index_offset
        for p in parts:
            m=a.materials[p.material];count=len(p.vertices)
            v=np.empty((count,11),'<f4');v[:,:3]=p.vertices*.001;v[:,3:6]=p.normals
            v[:,6:9]=m.color;v[:,9]=m.metal;v[:,10]=m.rough
            f=(p.faces.astype(np.uint32)+vertex_offset).reshape(-1)
            vertices.append(v);indices.append(f);vertex_offset+=count;index_offset+=len(f)
        groups.append(dict(group=group,explode=[float(x)*.001 for x in explode],start=start,count=index_offset-start,parts=len(parts)))
    # A neutral ground surface used only by the inspector. It is not a model part.
    v=np.array([[-70,-70,-.275,0,0,1,.4,.41,.37,0,.98],[70,-70,-.275,0,0,1,.4,.41,.37,0,.98],
                [70,70,-.275,0,0,1,.4,.41,.37,0,.98],[-70,70,-.275,0,0,1,.4,.41,.37,0,.98]],'<f4')
    vertices.append(v);indices.append(np.array([0,1,2,0,2,3],np.uint32)+vertex_offset)
    groups.append(dict(group='inspector_ground',explode=[0,0,0],start=index_offset,count=6,parts=0))
    data=dict(vertices=base64.b64encode(np.concatenate(vertices).tobytes()).decode(),indices=base64.b64encode(np.concatenate(indices).astype('<u4').tobytes()).decode(),groups=groups)
    template=(Path(__file__).parent/'viewer_template.html').read_text()
    path=out/'CYBR_YARD_viewer.html';path.write_text(template.replace('__MODEL_DATA__',json.dumps(data,separators=(',',':'))))
    return path
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('out',type=Path);args=p.parse_args();print(create_viewer(args.out))
