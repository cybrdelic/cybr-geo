"""Explicit seam charts, harmonic disk parameterization and attribute baking.

Not automatic seam selection or animation retopology. Uniform harmonic mapping
uses a convex boundary and can distort strongly. Reject unsupported topology.
"""
from dataclasses import dataclass
import numpy as np
from scipy.sparse import lil_matrix
from scipy.sparse.linalg import spsolve
from .model import Mesh, Attribute, mesh_edges

@dataclass(frozen=True)
class UVChart:
    faces: np.ndarray
    vertices: np.ndarray  # source vertex IDs, duplicated at seam ends if needed
    triangles: np.ndarray
    uv: np.ndarray


def unwrap(mesh, seams=()):
    """Cut explicit undirected mesh edges; parameterize each resulting disk.

    Returns separate corner-topology charts, without modifying physical mesh.
    Closed charts/holes/pinched cuts are rejected, never silently projected.
    """
    seams={tuple(sorted(map(int,e))) for e in seams}
    available={tuple(e) for e in mesh_edges(mesh.faces)}
    if not seams <= available: raise ValueError('Seam contains nonexistent edge')
    f=mesh.faces; parents=np.arange(3*len(f)); adjacency=[set() for _ in f]
    def root(a):
        while parents[a]!=a:
            parents[a]=parents[parents[a]];a=parents[a]
        return a
    edges={}
    for i,face in enumerate(f):
        for j in range(3):
            a,b=int(face[j]),int(face[(j+1)%3]);edges.setdefault(tuple(sorted((a,b))),[]).append((i,j))
    for e,uses in edges.items():
        if len(uses)!=2 or e in seams: continue
        (a,aj),(b,bj)=uses;adjacency[a].add(b);adjacency[b].add(a)
        for vertex in e:
            ca=3*a+int(np.where(f[a]==vertex)[0][0]);cb=3*b+int(np.where(f[b]==vertex)[0][0])
            parents[root(cb)]=root(ca)
    charts=[];remaining=set(range(len(f)))
    while remaining:
        todo=[min(remaining)];ids=[]
        while todo:
            a=todo.pop()
            if a not in remaining:continue
            remaining.remove(a);ids.append(a);todo.extend(adjacency[a])
        ids=np.array(sorted(ids));keys=[root(3*i+j) for i in ids for j in range(3)]
        unique,inverse=np.unique(keys,return_inverse=True);tri=inverse.reshape(-1,3)
        source=np.array([f[k//3,k%3] for k in unique]);n=len(source)
        ed={};neighbors=[set() for _ in range(n)]
        for face in tri:
            for j in range(3):
                a,b=map(int,(face[j],face[(j+1)%3]));ed.setdefault(tuple(sorted((a,b))),[]).append((a,b));neighbors[a].add(b);neighbors[b].add(a)
        boundary=[uses[0] for uses in ed.values() if len(uses)==1]
        if n-len(ed)+len(tri)!=1 or len(boundary)<3:raise ValueError('Each cut chart must be a topological disk; add seams')
        following={a:b for a,b in boundary}
        if len(following)!=len(boundary):raise ValueError('Pinched boundary; revise seams')
        loop=[boundary[0][0]]
        while following[loop[-1]]!=loop[0]:
            loop.append(following[loop[-1]])
            if len(loop)>len(boundary):raise ValueError('Invalid boundary')
        if len(loop)!=len(boundary):raise ValueError('Multiple boundary loops; add seams')
        loop=np.array(loop);points=mesh.vertices[source[loop]]
        lengths=np.linalg.norm(np.roll(points,-1,axis=0)-points,axis=1)
        angles=2*np.pi*np.r_[0,np.cumsum(lengths[:-1])]/sum(lengths)
        uv=np.zeros((n,2));uv[loop]=np.c_[np.cos(angles),np.sin(angles)]
        interior=np.setdiff1d(np.arange(n),loop)
        if len(interior):
            slots={a:i for i,a in enumerate(interior)};matrix=lil_matrix((len(interior),len(interior)));rhs=np.zeros((len(interior),2))
            for a,i in slots.items():
                matrix[i,i]=len(neighbors[a])
                for b in neighbors[a]:
                    if b in slots:matrix[i,slots[b]]=-1
                    else:rhs[i]+=uv[b]
            uv[interior]=spsolve(matrix.tocsr(),rhs)
        a=uv[tri[:,1]]-uv[tri[:,0]];b=uv[tri[:,2]]-uv[tri[:,0]]
        area=a[:,0]*b[:,1]-a[:,1]*b[:,0]
        if np.any(area<=1e-12):raise ValueError('Degenerate/folded chart; revise seams or resolution')
        charts.append(UVChart(ids,source,tri,uv))
    return charts


def pack(mesh, charts, padding=0.02):
    """Deterministic one-row rectangle packing, common scale, normalized atlas.

    Deliberately conservative, not optimal packing or equal texel density.
    Padding is an atlas-space gap, including outer border.
    """
    if not charts or not 0<padding<1/(len(charts)+1):raise ValueError('Invalid padding or empty charts')
    all_faces=np.concatenate([c.faces for c in charts])
    if not np.array_equal(np.sort(all_faces),np.arange(len(mesh.faces))):raise ValueError('Charts must cover every face exactly once')
    spans=np.array([np.ptp(c.uv,axis=0) for c in charts]);scale=min((1-(len(charts)+1)*padding)/sum(spans[:,0]),(1-2*padding)/max(spans[:,1]))
    corners=np.zeros((len(mesh.faces),3,2));x=padding
    for c,span in zip(charts,spans):
        uv=(c.uv-c.uv.min(axis=0))*scale+[x,padding]
        corners[c.faces]=uv[c.triangles];x+=span[0]*scale+padding
    attrs=dict(mesh.attributes);attrs['uv']=Attribute(corners.reshape(-1,2),'corner')
    return Mesh(mesh.vertices,mesh.faces,attrs,mesh.closed,mesh.connected)


def bake(mesh, attribute, resolution=256, background=0.):
    """Rasterize a vertex attribute to pixel-center UV samples, barycentrically.

    Returns (float image, coverage mask). Empty gutters remain background; no
    dilation, ray projection, normal-space conversion or antialiasing implied.
    Multiple interior coverage raises instead of silently overwriting overlap.
    """
    if not isinstance(resolution,int) or resolution<2:raise ValueError('Resolution must be integer >=2')
    a=mesh.attributes[attribute]
    if a.domain!='vertex':raise ValueError('Baking currently requires vertex data')
    u=mesh.attributes['uv']
    if u.domain!='corner' or u.values.shape!=(3*len(mesh.faces),2):raise ValueError('Expected corner UV')
    uv=u.values.reshape(-1,3,2)
    if np.any(uv<0) or np.any(uv>1):raise ValueError('UV outside unit atlas')
    values=a.values.reshape(len(mesh.vertices),-1);image=np.full((resolution,resolution,values.shape[1]),background,dtype=float);mask=np.zeros((resolution,resolution),bool);deep=np.zeros_like(mask)
    for face,t in zip(mesh.faces,uv):
        matrix=np.column_stack((t[1]-t[0],t[2]-t[0]))
        if abs(np.linalg.det(matrix))<1e-14:raise ValueError('Degenerate UV triangle')
        lo=np.maximum(0,np.floor(t.min(axis=0)*resolution-.5).astype(int));hi=np.minimum(resolution-1,np.ceil(t.max(axis=0)*resolution-.5).astype(int))
        xx,yy=np.meshgrid(np.arange(lo[0],hi[0]+1),np.arange(lo[1],hi[1]+1));pixels=np.c_[xx.ravel(),yy.ravel()]
        bc=((pixels+.5)/resolution-t[0])@np.linalg.inv(matrix).T;weights=np.c_[1-bc.sum(axis=1),bc];inside=np.all(weights>=-1e-10,axis=1);strict=np.all(weights>1e-9,axis=1)
        for (x,y),w,s in zip(pixels[inside],weights[inside],strict[inside]):
            if s and deep[y,x]:raise ValueError('Overlapping UV interiors')
            image[y,x]=w@values[face];mask[y,x]=True;deep[y,x]|=s
    return image,mask
