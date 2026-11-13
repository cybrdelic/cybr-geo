"""Prototype generic sparse uniform block extraction for curve networks.
All blocks use the same world grid; shared surface vertices are welded explicitly.
No botanical assumptions and no radius inflation. Refuses failed topology.
"""
import numpy as np
from .field import field
from .model import Mesh, Attribute

def sparse_surface(network, spacing=.009, block_cells=12):
    if not np.isfinite(spacing) or spacing<=0: raise ValueError('positive finite spacing required')
    if not isinstance(block_cells,int) or block_cells<2: raise ValueError('block_cells must be integer >=2')
    if spacing > network.radii.min(): raise ValueError('spacing exceeds minimum radius')
    origin=np.floor((network.nodes.min(0)-network.radii.max()-spacing*2)/spacing)*spacing
    width=spacing*block_cells; blocks=set()
    for (a,b),ra,rb in zip(network.edges,network.radii[network.edges[:,0]],network.radii[network.edges[:,1]]):
        p,q=network.nodes[a],network.nodes[b];r=max(ra,rb)+spacing*2
        lo=np.floor((np.minimum(p,q)-r-origin)/width).astype(int)
        hi=np.floor((np.maximum(p,q)+r-origin)/width).astype(int)
        for x in range(lo[0],hi[0]+1):
            for y in range(lo[1],hi[1]+1):
                for z in range(lo[2],hi[2]+1):blocks.add((x,y,z))
    fn=network.evaluator();vertices=[];faces=[];lookup={}
    for count,b in enumerate(sorted(blocks)):
        low=origin+np.array(b)*width;high=low+width
        try:m=field(fn,(low,high),resolution=block_cells+1,closed=False,connected=False,root_iterations=8)
        except ValueError as e:
            if 'no sampled zero surface' in str(e):continue
            raise
        ids=[]
        for v in m.vertices:
            # Bisection coordinates are dyadic grid values. Round numerical noise only.
            key=tuple(np.rint((v-origin)/spacing*512).astype(np.int64))
            if key not in lookup:lookup[key]=len(vertices);vertices.append(v)
            ids.append(lookup[key])
        faces.extend(np.asarray(ids)[m.faces]);
        if count%50==0:print('blocks',count,len(blocks),'vertices',len(vertices),flush=True)
    m=Mesh(np.asarray(vertices),np.asarray(faces),closed=True,connected=True)
    _,source=fn.evaluate(m.vertices,return_edges=True)
    return Mesh(m.vertices,m.faces,{'source_edge':Attribute(source,interpolation='nearest')})
