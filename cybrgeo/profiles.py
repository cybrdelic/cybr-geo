"""Exact straight-wall extrusion of authored planar boundaries.
SciPy triangulates only the cap vertices. Missing constrained boundaries are
rejected, never silently filled. This is not a general CAD Boolean kernel.
"""
import numpy as np
from .model import Mesh


def extrude(outline, height, holes=(), z=0.):
    """Extrude a 2D outline and holes; reject unrecovered boundary constraints.

    Input rings must be simple and mutually nonintersecting. No duplicate final
    point. The result retains authored profile vertices without field sampling.
    SciPy is an optional dependency for this operation only.
    """
    from scipy.spatial import Delaunay
    if not np.isfinite(height) or height<=0 or not np.isfinite(z):
        raise ValueError('Extrusion height must be positive and finite')
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
    def inside(q,p):
        result=np.zeros(len(q),bool)
        for a,b in zip(p,np.roll(p,-1,axis=0)):
            if a[1]==b[1]:continue
            cross=((a[1]>q[:,1])!=(b[1]>q[:,1])) & (q[:,0]<(b[0]-a[0])*(q[:,1]-a[1])/(b[1]-a[1])+a[0])
            result^=cross
        return result
    for refinement in range(7):
        points=np.concatenate(rings);n=len(points)
        triangles=Delaunay(points).simplices
        centers=points[triangles].mean(axis=1)
        keep=inside(centers,rings[0])
        for hole in rings[1:]:keep&=~inside(centers,hole)
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
