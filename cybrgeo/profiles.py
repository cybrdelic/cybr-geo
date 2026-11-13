"""Exact straight-wall extrusion of authored planar boundaries.
SciPy triangulates only the cap vertices. Missing constrained boundaries are
rejected, never silently filled. This is not a general CAD Boolean kernel.
"""
import numpy as np
from .model import Mesh


def _inside(q,p):
    result=np.zeros(len(q),bool)
    for a,b in zip(p,np.roll(p,-1,axis=0)):
        if a[1]==b[1]:continue
        cross=((a[1]>q[:,1])!=(b[1]>q[:,1])) & (q[:,0]<(b[0]-a[0])*(q[:,1]-a[1])/(b[1]-a[1])+a[0])
        result^=cross
    return result

def validate_profile(outline, holes=()):
    """Validate simple disjoint planar rings, normalize outer/inner winding.

    Shared by mesh and optional analytic operations. No triangulation or SciPy
    dependency; rejects crossings, touching boundaries and nested holes.
    """
    rings=[]
    for i,ring in enumerate([outline,*holes]):
        p=np.asarray(ring,float)
        if p.ndim!=2 or p.shape[1]!=2 or len(p)<3 or not np.isfinite(p).all():
            raise ValueError('Each profile requires at least three finite 2D points')
        area=np.sum(p[:,0]*np.roll(p[:,1],-1)-p[:,1]*np.roll(p[:,0],-1))
        if abs(area)<1e-12:raise ValueError('Profile has zero area')
        if (area>0)!=(i==0):p=p[::-1]
        rings.append(p)
    if len(np.unique(np.concatenate(rings),axis=0))!=sum(map(len,rings)):
        raise ValueError('Profile vertices must be distinct')
    # Fail closed before triangulation: a topologically closed Mesh is not proof
    # that authored boundary rings describe a valid planar material region.
    scale=max(1.,np.ptp(np.concatenate(rings),axis=0).max())
    linear_tol=np.finfo(float).eps*scale*64
    area_tol=linear_tol*scale
    for ring in rings:
        incoming=ring-np.roll(ring,1,axis=0)
        outgoing=np.roll(ring,-1,axis=0)-ring
        turn=incoming[:,0]*outgoing[:,1]-incoming[:,1]*outgoing[:,0]
        if np.any((np.abs(turn)<=area_tol)&(np.sum(incoming*outgoing,axis=1)<-linear_tol**2)):
            raise ValueError('Profile has overlapping adjacent edges')
    starts=np.concatenate(rings)
    ends=np.concatenate([np.roll(r,-1,axis=0) for r in rings])
    ring_ids=np.concatenate([np.full(len(r),i) for i,r in enumerate(rings)])
    local_ids=np.concatenate([np.arange(len(r)) for r in rings])
    def cross(a,b):return a[...,0]*b[...,1]-a[...,1]*b[...,0]
    for i,(a,b) in enumerate(zip(starts,ends)):
        js=np.arange(i+1,len(starts))
        same=ring_ids[js]==ring_ids[i]
        adjacent=(np.abs(local_ids[js]-local_ids[i])==1)|(np.abs(local_ids[js]-local_ids[i])==len(rings[ring_ids[i]])-1)
        js=js[~(same&adjacent)]
        c,d=starts[js],ends[js]
        bbox=np.all(np.maximum(np.minimum(a,b),np.minimum(c,d))<=np.minimum(np.maximum(a,b),np.maximum(c,d))+linear_tol,axis=1)
        js=js[bbox];c,d=starts[js],ends[js]
        ab=b-a;cd=d-c
        p=cross(ab,c-a);q=cross(ab,d-a);r=cross(cd,a-c);t=cross(cd,b-c)
        hit=(((p<=area_tol)&(q>=-area_tol))|((q<=area_tol)&(p>=-area_tol))) & (((r<=area_tol)&(t>=-area_tol))|((t<=area_tol)&(r>=-area_tol)))
        if np.any(hit):raise ValueError('Profile rings cross, touch, or self-intersect')
    for i,hole in enumerate(rings[1:],1):
        if not _inside(hole[:1],rings[0])[0]:raise ValueError('Hole must lie strictly inside outer profile')
        for other in rings[1:i]:
            if _inside(hole[:1],other)[0] or _inside(other[:1],hole)[0]:
                raise ValueError('Hole regions must not overlap or contain one another')
    return rings



def validate_planar_ring(points):
    """Validate a simple planar 3D polygon without triangulating it.

    Returns its 2D projection only for validation consumers. Vertex order and
    correspondence in the producer remain untouched.
    """
    p=np.asarray(points,float)
    if p.ndim != 2 or p.shape[1] != 3 or len(p)<3 or not np.isfinite(p).all():
        raise ValueError('Planar ring needs at least three finite 3D points')
    centered=p-p[0]
    normal=np.sum(np.cross(centered,np.roll(centered,-1,axis=0)),axis=0)
    length=np.linalg.norm(normal)
    if length<1e-12: raise ValueError('Degenerate planar ring')
    normal/=length
    if np.max(np.abs(centered@normal))>1e-7*max(np.ptp(p,axis=0).max(),1):
        raise ValueError('Ring must be planar')
    projected=np.delete(centered,np.argmax(np.abs(normal)),axis=1)
    validate_profile(projected)
    return projected


def extrude(outline, height, holes=(), z=0., spacing=None):
    """Extrude a 2D outline and holes; reject unrecovered boundary constraints.

    Optional spacing seeds cap interiors for later deformation; it is a sampling
    spacing, not a guaranteed edge-length or surface-error bound. Boundary
    segments remain explicit and must all be recovered.

    Input rings must be simple and mutually nonintersecting. No duplicate final
    point. The result retains authored profile vertices without field sampling.
    SciPy is an optional dependency for this operation only.
    """
    from scipy.spatial import Delaunay
    if not np.isfinite(height) or height<=0 or not np.isfinite(z):
        raise ValueError('Extrusion height must be positive and finite')
    if spacing is not None and (not np.isfinite(spacing) or spacing <= 0):
        raise ValueError('spacing must be finite and positive')
    rings=validate_profile(outline, holes)
    interior = np.empty((0,2))
    if spacing is not None:
        lo,hi=rings[0].min(axis=0),rings[0].max(axis=0)
        dimensions=np.ceil((hi-lo)/spacing).astype(int)
        if int(dimensions[0])*int(dimensions[1]) > 1_000_000:
            raise ValueError('Profile sampling exceeds one million candidate points')
        x=np.arange(lo[0]+spacing*.5,hi[0],spacing)
        y=np.arange(lo[1]+spacing*.5,hi[1],spacing)
        xx,yy=np.meshgrid(x,y,indexing='ij'); candidates=np.c_[xx.ravel(),yy.ravel()]
        keep=_inside(candidates,rings[0])
        for hole in rings[1:]:keep&=~_inside(candidates,hole)
        candidates=candidates[keep]
        # Avoid exact boundary coincidences and extremely thin cap triangles.
        keep=np.ones(len(candidates),bool)
        for ring in rings:
            for a,b in zip(ring,np.roll(ring,-1,axis=0)):
                d=b-a; t=np.clip((candidates-a)@d/np.dot(d,d),0,1)
                keep &= np.linalg.norm(candidates-(a+t[:,None]*d),axis=1)>spacing*.1
        interior=candidates[keep]
    for refinement in range(7):
        points=np.concatenate([*rings,interior]);n=len(points)
        triangles=Delaunay(points).simplices
        centers=points[triangles].mean(axis=1)
        keep=_inside(centers,rings[0])
        for hole in rings[1:]:keep&=~_inside(centers,hole)
        triangles=triangles[keep]
        edges=np.sort(np.concatenate([triangles[:,[0,1]],triangles[:,[1,2]],triangles[:,[2,0]]]),axis=1)
        unique,counts=np.unique(edges,axis=0,return_counts=True)
        boundary={tuple(e) for e in unique[counts==1]}
        expected=set();offset=0;loops=[]
        for ring in rings:
            loop=np.arange(offset,offset+len(ring));loops.append(loop)
            expected.update(tuple(sorted((int(a),int(b)))) for a,b in zip(loop,np.roll(loop,-1)))
            offset+=len(ring)
        if boundary==expected:break
        if refinement==6:
            raise ValueError('Cap triangulation cannot recover every profile boundary within the refinement budget')
        # Split only unrecovered segments; midpoints preserve exact straight edges.
        refined=[]
        for ring,loop in zip(rings,loops):
            result=[]
            for j,(a,b) in enumerate(zip(loop,np.roll(loop,-1))):
                result.append(ring[j])
                if tuple(sorted((int(a),int(b)))) not in boundary:
                    result.append((ring[j]+ring[(j+1)%len(ring)])/2)
            refined.append(np.asarray(result))
        rings=refined
    a,b,c=points[triangles[:,0]],points[triangles[:,1]],points[triangles[:,2]]
    negative=((b[:,0]-a[:,0])*(c[:,1]-a[:,1])-(b[:,1]-a[:,1])*(c[:,0]-a[:,0]))<0
    triangles[negative]=triangles[negative][:,::-1]
    vertices=np.r_[np.c_[points,np.full(n,z)],np.c_[points,np.full(n,z+height)]]
    faces=[*triangles[:,::-1].tolist(),*(triangles+n).tolist()]
    for loop in loops:
        for a,b in zip(loop,np.roll(loop,-1)):faces.extend(((a,b,b+n),(a,b+n,a+n)))
    return Mesh(vertices,np.asarray(faces))
