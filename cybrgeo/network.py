"""General radius-bearing curve networks; no botanical assumptions.

Networks are editable geometry sources. Surface extraction is derived data.
The implicit function is a union of tapered segment distance approximations,
not an exact signed distance. Small features require explicit sample spacing.
"""
from dataclasses import dataclass
import numpy as np
from .model import Mesh, Attribute
from .field import field

@dataclass(frozen=True)
class CurveNetwork:
    nodes: np.ndarray
    edges: np.ndarray
    radii: np.ndarray

    def __post_init__(self):
        n=np.array(self.nodes,float,copy=True);e=np.array(self.edges,copy=True);r=np.array(self.radii,float,copy=True)
        if n.ndim!=2 or n.shape[1]!=3 or len(n)<2 or not np.isfinite(n).all(): raise ValueError('Finite Nx3 nodes required')
        if e.ndim!=2 or e.shape[1]!=2 or not len(e) or not np.issubdtype(e.dtype,np.integer) or e.min()<0 or e.max()>=len(n): raise ValueError('Integer Ex2 edges required')
        if r.shape!=(len(n),) or not np.isfinite(r).all() or np.any(r<=0): raise ValueError('Positive radius per node required')
        if np.any(np.linalg.norm(n[e[:,1]]-n[e[:,0]],axis=1)<1e-12): raise ValueError('Zero-length network edge')
        if len(np.unique(np.sort(e,axis=1),axis=0))!=len(e): raise ValueError('Duplicate network edge')
        if len(np.unique(e))!=len(n): raise ValueError('Unused network node')
        for a in (n,e,r):a.flags.writeable=False
        object.__setattr__(self,'nodes',n);object.__setattr__(self,'edges',e);object.__setattr__(self,'radii',r)

    def edit(self,*,nodes=None,radii=None):
        return CurveNetwork(self.nodes if nodes is None else nodes,self.edges,self.radii if radii is None else radii)

    def segments(self):
        return self.nodes[self.edges[:,0]],self.nodes[self.edges[:,1]]

    def evaluator(self):
        return NetworkField(self)

    def surface(self,spacing,*,padding=None):
        if not np.isfinite(spacing) or spacing<=0: raise ValueError('Positive finite spacing required')
        # Do not silently thicken geometry to make a chosen sampling budget pass.
        if spacing>self.radii.min(): raise ValueError('Spacing exceeds smallest radius; refine sampling, not the network')
        pad=self.radii.max()+3*spacing if padding is None else float(padding)
        if pad<self.radii.max()+spacing: raise ValueError('Padding must contain all material')
        low=self.nodes.min(axis=0)-pad;high=self.nodes.max(axis=0)+pad
        resolution=np.ceil((high-low)/spacing).astype(int)+1
        if np.prod(resolution)>30_000_000: raise ValueError('Uniform grid exceeds 30M samples; adaptive extraction needed')
        fn=self.evaluator()
        mesh=field(fn,(low,high),resolution=resolution,connected=True)
        _,ids=fn.evaluate(mesh.vertices,return_edges=True)
        attrs=dict(mesh.attributes);attrs['source_edge']=Attribute(ids,interpolation='nearest')
        return Mesh(mesh.vertices,mesh.faces,attrs)

class NetworkField:
    """Exact minimum over tapered segments, with conservative candidate pruning.

Unlike selecting an arbitrary 16 nearest segments, the midpoint lower bound
expands candidates until omitted segments provably cannot win. SciPy is an
optional accelerator; the NumPy fallback evaluates every segment.
"""
    def __init__(self,network):
        self.network=network;self.a,self.b=network.segments();self.ab=self.b-self.a
        self.length2=np.sum(self.ab*self.ab,axis=1)
        self.ra=network.radii[network.edges[:,0]];self.rb=network.radii[network.edges[:,1]]
        self.mid=(self.a+self.b)*.5
        self.reach=np.sqrt(self.length2)*.5+np.maximum(self.ra,self.rb)
        try:
            from scipy.spatial import cKDTree
            self.index=cKDTree(self.mid)
        except ImportError:self.index=None

    def __call__(self,points):return self.evaluate(points)

    def evaluate(self,points,return_edges=False):
        p=np.asarray(points,float)
        if p.ndim!=2 or p.shape[1]!=3 or not np.isfinite(p).all():raise ValueError('Finite Nx3 samples required')
        output=[];sources=[];count=len(self.a)
        for chunk in np.array_split(p,max(1,int(np.ceil(len(p)/8192)))):
            k=min(16,count) if self.index is not None else count
            while True:
                if self.index is None:ix=np.broadcast_to(np.arange(count),(len(chunk),count));dist=None
                else:
                    dist,ix=self.index.query(chunk,k=k)
                    if k==1:dist=dist[:,None];ix=ix[:,None]
                delta=chunk[:,None,:]-self.a[ix]
                t=np.clip(np.sum(delta*self.ab[ix],axis=2)/self.length2[ix],0,1)
                d=np.linalg.norm(delta-t[:,:,None]*self.ab[ix],axis=2)-(self.ra[ix]*(1-t)+self.rb[ix]*t)
                winner=np.argmin(d,axis=1);minimum=d[np.arange(len(chunk)),winner]
                if k==count or np.all(dist[:,-1]-self.reach.max()>minimum):break
                k=min(count,k*2)
            output.append(minimum);sources.append(ix[np.arange(len(chunk)),winner])
        values=np.concatenate(output) if output else np.empty(0)
        return (values,np.concatenate(sources)) if return_edges else values


def parametric_surface(fn,u,v):
    """Open connected surface from an arbitrary map R2→R3, with UV coordinates."""
    u=np.asarray(u,float);v=np.asarray(v,float)
    if u.ndim!=1 or v.ndim!=1 or len(u)<2 or len(v)<2 or np.any(np.diff(u)<=0) or np.any(np.diff(v)<=0):raise ValueError('Increasing parameter axes required')
    U,V=np.meshgrid(u,v,indexing='ij');points=np.asarray(fn(U,V),float)
    if points.shape!=U.shape+(3,):raise ValueError('Map must return (nu,nv,3)')
    ids=np.arange(U.size).reshape(U.shape);a=ids[:-1,:-1].ravel();b=ids[1:,:-1].ravel();c=ids[1:,1:].ravel();d=ids[:-1,1:].ravel()
    faces=np.concatenate([np.stack((a,b,c),axis=1),np.stack((a,c,d),axis=1)])
    uv=np.stack(((U-u[0])/(u[-1]-u[0]),(V-v[0])/(v[-1]-v[0])),axis=-1).reshape(-1,2)
    return Mesh(points.reshape(-1,3),faces,{'uv':Attribute(uv)},closed=False)

@dataclass(frozen=True)
class Instances:
    source: Mesh
    transforms: np.ndarray

    def __post_init__(self):
        t=np.array(self.transforms,float,copy=True)
        if t.ndim!=3 or t.shape[1:]!=(4,4) or not np.isfinite(t).all() or not np.allclose(t[:,3],[0,0,0,1]):raise ValueError('Finite affine instance transforms required')
        if np.any(np.linalg.det(t[:,:3,:3])<=0):raise ValueError('Instances require positive nonzero determinants')
        t.flags.writeable=False;object.__setattr__(self,'transforms',t)

    def realize(self):
        n=len(self.source.vertices);v=np.einsum('bij,nj->bni',self.transforms[:,:3,:3],self.source.vertices)+self.transforms[:,:3,3][:,None,:]
        f=self.source.faces[None,:,:]+(np.arange(len(v))*n)[:,None,None]
        attrs={name:Attribute(np.concatenate([a.values]*len(v)),a.domain,a.interpolation) for name,a in self.source.attributes.items()}
        return Mesh(v.reshape(-1,3),f.reshape(-1,3),attrs,closed=self.source.closed,connected=False)
