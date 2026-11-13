"""Generic connected surface construction and attribute-preserving edits."""
import numpy as np
from .model import Mesh, Attribute


def heightfield(x,y,heights):
    x,y=np.asarray(x,float),np.asarray(y,float)
    h=np.asarray(heights,float)
    if x.ndim!=1 or y.ndim!=1 or len(x)<2 or len(y)<2 or np.any(np.diff(x)<=0) or np.any(np.diff(y)<=0) or h.shape!=(len(x),len(y)):
        raise ValueError('Heightfield requires increasing axes and matching (nx,ny) heights')
    xx,yy=np.meshgrid(x,y,indexing='ij');v=np.stack((xx,yy,h),axis=-1).reshape(-1,3)
    ids=np.arange(len(v)).reshape(h.shape)
    a,b,c,d=[q.ravel() for q in (ids[:-1,:-1],ids[1:,:-1],ids[:-1,1:],ids[1:,1:])]
    f=np.concatenate((np.stack((a,b,d),axis=1),np.stack((a,d,c),axis=1)))
    uv=np.stack(((xx-x[0])/(x[-1]-x[0]),(yy-y[0])/(y[-1]-y[0])),axis=-1).reshape(-1,2)
    return Mesh(v,f,{'uv':Attribute(uv)},closed=False)


def _cap(points):
    """Ear-clip a planar simple polygon; reject ambiguous/nonplanar input."""
    normal=np.sum(np.cross(points,np.roll(points,-1,axis=0)),axis=0)
    if np.linalg.norm(normal)<1e-12: raise ValueError('Degenerate cap outline')
    normal/=np.linalg.norm(normal)
    if np.max(np.abs((points-points[0])@normal))>1e-7*max(np.ptp(points,axis=0).max(),1):
        raise ValueError('Loft end caps must be planar')
    axis=np.argmax(np.abs(normal));p=np.delete(points,axis,axis=1)
    cross=lambda a,b:a[0]*b[1]-a[1]*b[0]
    sign=np.sign(sum(cross(a,b) for a,b in zip(p,np.roll(p,-1,axis=0))))
    remaining=list(range(len(p)));triangles=[]
    while len(remaining)>3:
        for k,b in enumerate(remaining):
            a,c=remaining[k-1],remaining[(k+1)%len(remaining)]
            if sign*cross(p[b]-p[a],p[c]-p[b])<=1e-12: continue
            blocked=False
            for j in remaining:
                if j in (a,b,c): continue
                tests=[sign*cross(p[v]-p[u],p[j]-p[u]) for u,v in ((a,b),(b,c),(c,a))]
                if min(tests)>=-1e-12: blocked=True;break
            if not blocked:
                triangles.append((a,b,c));remaining.pop(k);break
        else: raise ValueError('Cap outline cannot be triangulated; check crossings or duplicate points')
    triangles.append(tuple(remaining));return triangles


def loft(sections,caps=True):
    """Connect ordered, corresponding closed section samples. No prefab shape."""
    s=np.asarray(sections,float)
    if s.ndim!=3 or s.shape[2]!=3 or s.shape[0]<2 or s.shape[1]<3:
        raise ValueError('Sections must be (rings,samples,3), with matching sample correspondence')
    rings,n,_=s.shape;faces=[]
    for i in range(rings-1):
        for j in range(n):
            a=i*n+j;b=i*n+(j+1)%n;c=(i+1)*n+j;d=(i+1)*n+(j+1)%n
            faces.extend(((a,b,d),(a,d,c)))
    if caps:
        faces.extend(tuple(reversed(t)) for t in _cap(s[0]))
        faces.extend(tuple((rings-1)*n+j for j in t) for t in _cap(s[-1]))
    return Mesh(s.reshape(-1,3),np.array(faces,dtype=np.int64),
                {'section_coordinate':Attribute(np.repeat(np.arange(rings,dtype=float),n))},closed=caps)


def region(mesh,name,weights):
    weights=np.asarray(weights(mesh.vertices) if callable(weights) else weights,float)
    if weights.shape!=(len(mesh.vertices),) or not np.isfinite(weights).all() or np.any((weights<0)|(weights>1)):
        raise ValueError('Region weights must be one finite [0,1] value per vertex')
    attributes=dict(mesh.attributes);attributes['region:'+name]=Attribute(weights)
    return Mesh(mesh.vertices,mesh.faces,attributes,mesh.closed,mesh.connected)


def sculpt(mesh,displacement,region=None):
    d=np.asarray(displacement(mesh.vertices) if callable(displacement) else displacement,float)
    if d.shape==(3,): d=np.broadcast_to(d,mesh.vertices.shape)
    if d.shape!=mesh.vertices.shape or not np.isfinite(d).all(): raise ValueError('Displacement must be a finite vector or Nx3 array')
    weights=mesh.attributes['region:'+region].values if region is not None else np.ones(len(d))
    return Mesh(mesh.vertices+d*weights[:,None],mesh.faces,mesh.attributes,mesh.closed,mesh.connected)


def subdivide(mesh,levels=1,smooth=True):
    """Loop subdivision, carrying linear attributes through the SAME stencils."""
    if not isinstance(levels,int) or levels<0 or levels>4: raise ValueError('Subdivision levels must be 0..4')
    for _ in range(levels):
        v,f=mesh.vertices,mesh.faces;N=len(v)
        pairs=np.concatenate((f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]))
        opposite=np.concatenate((f[:,2],f[:,0],f[:,1]))
        edges,inverse,counts=np.unique(np.sort(pairs,axis=1),axis=0,return_inverse=True,return_counts=True)
        neighbors=[set() for _ in v];boundary=[[] for _ in v]
        for (a,b),count in zip(edges,counts):
            neighbors[a].add(b);neighbors[b].add(a)
            if count==1: boundary[a].append(b);boundary[b].append(a)
        incident=[[] for _ in edges]
        for e,o in zip(inverse,opposite): incident[e].append(o)
        def transfer(values):
            shape=values.shape[1:];data=values.reshape(N,-1).astype(float)
            old=data.copy();mid=(data[edges[:,0]]+data[edges[:,1]])/2
            if smooth:
                for i,adj in enumerate(neighbors):
                    if boundary[i]:
                        if len(boundary[i])!=2: raise ValueError('Subdivision needs manifold boundaries')
                        old[i]=.75*data[i]+.125*data[boundary[i]].sum(axis=0)
                    else:
                        n=len(adj);beta=3/16 if n==3 else 3/(8*n)
                        old[i]=(1-n*beta)*data[i]+beta*data[list(adj)].sum(axis=0)
                for i,((a,b),others) in enumerate(zip(edges,incident)):
                    if len(others)==2: mid[i]=.375*(data[a]+data[b])+.125*data[others].sum(axis=0)
            return np.concatenate((old,mid)).reshape((N+len(edges),)+shape)
        ab,bc,ca=np.split(inverse+N,3);a,b,c=f.T
        faces=np.concatenate([np.stack(t,axis=1) for t in ((a,ab,ca),(b,bc,ab),(c,ca,bc),(ab,bc,ca))])
        attributes={}
        for name,attr in mesh.attributes.items():
            if attr.domain not in ('vertex','face'):
                raise ValueError('Subdivision supports vertex/face attributes only; corner/edge transfer is not implemented')
            if attr.domain=='face': values=np.concatenate([attr.values]*4)
            elif attr.interpolation=='linear': values=transfer(attr.values)
            else: values=np.concatenate((attr.values,attr.values[edges[:,0]]))
            attributes[name]=Attribute(values,attr.domain,attr.interpolation)
        mesh=Mesh(transfer(v),faces,attributes,mesh.closed,mesh.connected)
    return mesh
