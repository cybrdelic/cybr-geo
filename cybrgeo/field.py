"""Sample arbitrary signed fields and extract shared-edge triangular surfaces."""
import numpy as np
from .model import Mesh

def field(fn, bounds, resolution=48, closed=True, connected=True, root_iterations=12):
    if not isinstance(root_iterations,int) or not 0<=root_iterations<=24:
        raise ValueError('Root refinement iterations must be 0..24')
    low,high=np.asarray(bounds,dtype=float)
    if not np.issubdtype(np.asarray(resolution).dtype,np.integer):
        raise ValueError('Resolution must contain integer sample counts')
    n=np.broadcast_to(np.asarray(resolution,dtype=int),(3,))
    if low.shape!=(3,) or not np.isfinite([low,high]).all() or np.any(high<=low) or np.any(n<3):
        raise ValueError('Field needs finite increasing bounds and >=3 samples per axis')
    axes=[np.linspace(a,b,int(count)) for a,b,count in zip(low,high,n)]
    grid=np.stack(np.meshgrid(*axes,indexing='ij'),axis=-1)
    points=grid.reshape(-1,3)
    values=np.asarray(fn(points),dtype=float)
    if values.shape!=(len(points),) or not np.isfinite(values).all():
        raise ValueError('Field must return one finite signed value per sample')
    values=values.copy(); epsilon=max(np.max(np.abs(values))*1e-10,1e-12)
    values[values==0]=epsilon
    ids=np.arange(len(points)).reshape(tuple(n))
    boundary=np.zeros(tuple(n),bool)
    boundary[[0,-1],:,:]=True;boundary[:,[0,-1],:]=True;boundary[:,:,[0,-1]]=True
    if closed and np.any(values[boundary.ravel()]<0):
        raise ValueError('Closed field material reaches the sampling boundary; enlarge bounds')
    corners=np.stack([ids[a:n[0]-1+a,b:n[1]-1+b,c:n[2]-1+c].ravel()
                      for a,b,c in [(0,0,0),(1,0,0),(1,1,0),(0,1,0),(0,0,1),(1,0,1),(1,1,1),(0,1,1)]],axis=1)
    tetrahedra=[(0,1,2,6),(0,2,3,6),(0,3,7,6),(0,7,4,6),(0,4,5,6),(0,5,1,6)]
    vertices=[];faces=[];cache={}
    for pattern in tetrahedra:
        tets=corners[:,pattern]; signs=values[tets]<0
        tets=tets[(np.sum(signs,axis=1)>0)&(np.sum(signs,axis=1)<4)]
        for tet in tets:
            inside=values[tet]<0; polygon=[]
            for i in range(4):
                for j in range(i+1,4):
                    if inside[i]==inside[j]: continue
                    a,b=sorted((int(tet[i]),int(tet[j])));key=(a,b)
                    if key not in cache:
                        t=values[a]/(values[a]-values[b])
                        cache[key]=len(vertices);vertices.append(points[a]+t*(points[b]-points[a]))
                    polygon.append(cache[key])
            positions=np.array([vertices[i] for i in polygon])
            center=positions.mean(axis=0)
            normal=np.linalg.solve(points[tet[1:]]-points[tet[0]],values[tet[1:]]-values[tet[0]])
            normal/=np.linalg.norm(normal)
            u=positions[0]-center;u/=np.linalg.norm(u);v=np.cross(normal,u)
            angles=np.arctan2((positions-center)@v,(positions-center)@u)
            ordered=np.asarray(polygon)[np.argsort(angles)]
            for i in range(1,len(ordered)-1): faces.append((ordered[0],ordered[i],ordered[i+1]))
    if not faces: raise ValueError('Field has no sampled zero surface')
    vertices=np.array(vertices)
    if root_iterations:
        # Refine ALL crossings in batches. Linear interpolation is wrong at sharp
        # Boolean transitions and for signed fields that are not distance fields.
        edges=np.array(list(cache),dtype=np.int64)
        left,right=points[edges[:,0]].copy(),points[edges[:,1]].copy()
        left_inside=values[edges[:,0]]<0
        for _ in range(root_iterations):
            middle=(left+right)/2
            samples=np.asarray(fn(middle),dtype=float)
            if samples.shape!=(len(middle),) or not np.isfinite(samples).all():
                raise ValueError('Field returned invalid values during root refinement')
            same=(samples<0)==left_inside
            left[same]=middle[same];right[~same]=middle[~same]
        vertices=(left+right)/2
    return Mesh(vertices,np.array(faces,dtype=np.int64),closed=closed,connected=connected)
