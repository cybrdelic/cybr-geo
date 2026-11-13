"""Conservative-topology QEM simplification with an explicit sampled error budget.

No global Hausdorff guarantee: each accepted collapse is checked using finite
samples against continuous triangle surfaces, in both directions.
"""
from dataclasses import dataclass
import numpy as np
from .model import Mesh, Attribute, mesh_edges
from .spatial import MeshBVH
from .adaptive import reproject_anchor


def _samples(mesh):
    edges=mesh_edges(mesh.faces)
    return np.concatenate((mesh.vertices,mesh.vertices[mesh.faces].mean(1),
                           mesh.vertices[edges].mean(1)))


def sampled_surface_error(source, target):
    """Maximum two-way sampled distance (vertices, centroids, edge midpoints).

    Query samples against triangles, NOT nearest target vertices. This value
    is a lower bound on true Hausdorff distance, never an upper-bound guarantee.
    """
    def distance(a,b):
        index=MeshBVH(b)
        return max(index.closest_point(p).distance for p in _samples(a))
    return max(distance(source,target),distance(target,source))


@dataclass(frozen=True)
class LOD:
    mesh: Mesh
    sampled_error: float
    collapses: int
    reached_target: bool
    source_vertex_groups: tuple
    source_face_indices: np.ndarray

    def rebind(self, anchor, source, max_distance):
        """Explicit nearest-surface rebinding, not exact semantic ancestry."""
        return reproject_anchor(anchor,source,self.mesh,max_distance)


def simplify(mesh, target_faces, max_error, preserve_boundary=True,
             protected_vertices=()):
    """Greedy quadric-error edge collapses with sampled deviation rejection.

    Boundary vertices are pinned by default. Explicit protected vertices never
    move or collapse. Vertex attributes interpolate along collapsed edges; face
    attributes follow surviving source triangles. Edge/corner data is rejected
    rather than silently destroying seams. Preserve UV seams using vertex
    splits or supply an explicit seam-aware policy outside this operation.

    The implementation favors clarity/correctness for moderate meshes, not
    real-time reduction: each tentative collapse validates mesh topology and
    measures two-way source deviation. It may stop ABOVE target_faces when the
    error budget, pins, or topology prevent further admissible collapses.
    """
    if not isinstance(target_faces,int) or target_faces<1:
        raise ValueError('target_faces must be a positive integer')
    if not np.isfinite(max_error) or max_error<0:
        raise ValueError('A finite nonnegative sampled error budget is required')
    if any(a.domain not in ('vertex','face') for a in mesh.attributes.values()):
        raise ValueError('Simplification requires an explicit edge/corner seam policy')
    pins=set(protected_vertices)
    if any(not isinstance(i,(int,np.integer)) or i<0 or i>=len(mesh.vertices) for i in pins):
        raise ValueError('Protected vertices must exist')
    if preserve_boundary:
        f=mesh.faces
        e,c=np.unique(np.sort(np.concatenate((f[:,[0,1]],f[:,[1,2]],f[:,[2,0]])),1),axis=0,return_counts=True)
        pins.update(map(int,e[c==1].ravel()))
    # Original face plane quadrics, accumulated through every accepted collapse.
    tri=mesh.vertices[mesh.faces];normal=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
    normal/=np.linalg.norm(normal,axis=1)[:,None]
    planes=np.column_stack((normal,-np.einsum('ij,ij->i',normal,tri[:,0])))
    quadrics=np.zeros((len(mesh.vertices),4,4))
    for fi,face in enumerate(mesh.faces):
        for v in face:quadrics[v]+=np.outer(planes[fi],planes[fi])
    current=mesh;groups=[(i,) for i in range(len(mesh.vertices))]
    provenance=np.arange(len(mesh.faces));collapses=0;error=0.
    while len(current.faces)>target_faces:
        candidates=[]
        for a,b in mesh_edges(current.faces):
            if any(i in pins for i in groups[a]+groups[b]):continue
            q=quadrics[a]+quadrics[b];va,vb=current.vertices[[a,b]]
            # Restrict optimum to the edge segment so attribute interpolation
            # and source-vertex ancestry remain explicit and bounded.
            h=np.r_[va,1.];d=np.r_[vb-va,0.];den=d@q@d
            t=float(np.clip(-(d@q@h)/den,0,1)) if den>1e-14 else .5
            p=va+t*(vb-va);hp=np.r_[p,1.]
            candidates.append((float(hp@q@hp),int(a),int(b),t,p))
        accepted=False
        for _,a,b,t,p in sorted(candidates,key=lambda x:(x[0],x[1],x[2])):
            mapped=current.faces.copy();mapped[mapped==b]=a
            keep=np.array([len(set(f))==3 for f in mapped])
            if keep.sum()<target_faces:continue
            mapped=mapped[keep];v=current.vertices.copy();v[a]=p
            # Reject foldovers of any surviving incident triangle.
            old=current.vertices[current.faces[keep]];new=v[mapped]
            n0=np.cross(old[:,1]-old[:,0],old[:,2]-old[:,0]);n1=np.cross(new[:,1]-new[:,0],new[:,2]-new[:,0])
            if np.any(np.einsum('ij,ij->i',n0,n1)<=0):continue
            used=np.unique(mapped);lookup=np.full(len(v),-1,int);lookup[used]=np.arange(len(used))
            attrs={}
            for name,attr in current.attributes.items():
                if attr.domain=='face':values=attr.values[keep]
                else:
                    values=attr.values.astype(float).copy() if attr.interpolation=='linear' else attr.values.copy()
                    values[a]=(1-t)*values[a]+t*values[b] if attr.interpolation=='linear' else values[a if t<=.5 else b]
                    values=values[used]
                attrs[name]=Attribute(values,attr.domain,attr.interpolation)
            try:candidate=Mesh(v[used],lookup[mapped],attrs,current.closed,current.connected)
            except ValueError:continue
            deviation=sampled_surface_error(mesh,candidate)
            if deviation>max_error+1e-12:continue
            quadrics[a]+=quadrics[b];quadrics=quadrics[used]
            groups[a]=groups[a]+groups[b];groups=[groups[i] for i in used]
            provenance=provenance[keep];current=candidate
            error=deviation;collapses+=1;accepted=True;break
        if not accepted:break
    provenance.flags.writeable=False
    return LOD(current,error,collapses,len(current.faces)<=target_faces,
               tuple(groups),provenance)
