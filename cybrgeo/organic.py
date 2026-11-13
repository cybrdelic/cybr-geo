"""General organic edits. No anatomy, auto-retopology or collision guarantees."""
from dataclasses import dataclass
import numpy as np
from .model import Mesh, Attribute

def crease_subdivide(mesh, levels=1, creases=(), smooth=True):
    """Loop refinement with persistent hard creases supplied as vertex-index pairs.

    Two sharp neighbors use a crease stencil; three or more pin a corner.
    Boundary edges are hard. Linear vertex attributes use geometry stencils;
    face values replicate. No semi-sharp fractional crease weights yet.
    """
    if not isinstance(levels,int) or levels<0 or levels>4: raise ValueError('Subdivision levels must be 0..4')
    sharp = {tuple(sorted(map(int, e))) for e in creases}
    for _ in range(levels):
        v,f=mesh.vertices,mesh.faces;N=len(v)
        pairs=np.concatenate((f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]))
        opposite=np.concatenate((f[:,2],f[:,0],f[:,1]))
        edges,inverse,counts=np.unique(np.sort(pairs,axis=1),axis=0,return_inverse=True,return_counts=True)
        if not sharp.issubset(set(map(tuple, edges))):
            raise ValueError("Creases must reference existing mesh edges")
        neighbors=[set() for _ in v];boundary=[[] for _ in v]
        for (a,b),count in zip(edges,counts):
            neighbors[a].add(b);neighbors[b].add(a)
            if count==1 or (int(a),int(b)) in sharp: boundary[a].append(b);boundary[b].append(a)
        incident=[[] for _ in edges]
        for e,o in zip(inverse,opposite): incident[e].append(o)
        def transfer(values):
            shape=values.shape[1:];data=values.reshape(N,-1).astype(float)
            old=data.copy();mid=(data[edges[:,0]]+data[edges[:,1]])/2
            if smooth:
                for i,adj in enumerate(neighbors):
                    if boundary[i]:
                        if len(boundary[i])==2:
                            old[i]=.75*data[i]+.125*data[boundary[i]].sum(axis=0)
                        elif len(boundary[i])>2:
                            old[i]=data[i]  # Corner at a junction of three hard edges.
                        else:
                            n=len(adj); beta=3/16 if n==3 else 3/(8*n)
                            old[i]=(1-n*beta)*data[i]+beta*data[list(adj)].sum(axis=0)
                    else:
                        n=len(adj);beta=3/16 if n==3 else 3/(8*n)
                        old[i]=(1-n*beta)*data[i]+beta*data[list(adj)].sum(axis=0)
                for i,((a,b),others) in enumerate(zip(edges,incident)):
                    if len(others)==2 and (int(a),int(b)) not in sharp: mid[i]=.375*(data[a]+data[b])+.125*data[others].sum(axis=0)
            return np.concatenate((old,mid)).reshape((N+len(edges),)+shape)
        ab,bc,ca=np.split(inverse+N,3);a,b,c=f.T
        faces=np.concatenate([np.stack(t,axis=1) for t in ((a,ab,ca),(b,bc,ab),(c,ca,bc),(ab,bc,ca))])
        attributes={}
        for name,attr in mesh.attributes.items():
            if attr.domain not in ('face','vertex'):
                raise ValueError('Crease subdivision currently supports vertex and face attributes only')
            if attr.domain=='face': values=np.concatenate([attr.values]*4)
            elif attr.interpolation=='linear': values=transfer(attr.values)
            else: values=np.concatenate((attr.values,attr.values[edges[:,0]]))
            attributes[name]=Attribute(values,attr.domain,attr.interpolation)
        new_sharp={(int(a),N+i) for i,(a,b) in enumerate(edges) if (int(a),int(b)) in sharp} | {(int(b),N+i) for i,(a,b) in enumerate(edges) if (int(a),int(b)) in sharp}
        mesh=Mesh(transfer(v),faces,attributes,mesh.closed,mesh.connected)
        sharp={tuple(sorted(e)) for e in new_sharp}
    return mesh

@dataclass(frozen=True)
class LocalEdit:
    """Exact affected indices for downstream refit/rendering; not local remeshing."""
    mesh: Mesh
    vertices: np.ndarray
    faces: np.ndarray
    bounds: np.ndarray


def local_displace(mesh, indices, displacement):
    """Edit only selected vertices. Others stay bit-identical, topology unchanged.

    Evaluation and Mesh validation still allocate/check the whole output mesh.
    The returned affected faces can drive a spatial-index refit or preview update.
    """
    raw=np.asarray(indices)
    if raw.ndim!=1 or not np.issubdtype(raw.dtype,np.integer):
        raise ValueError('Indices must be an integer vector')
    ids=np.unique(raw)
    if len(ids)==0 or ids.min()<0 or ids.max()>=len(mesh.vertices):
        raise ValueError('Select at least one existing vertex')
    d=np.asarray(displacement(mesh.vertices[ids]) if callable(displacement) else displacement,float)
    if d.shape==(3,): d=np.broadcast_to(d,(len(ids),3))
    if d.shape!=(len(ids),3) or not np.isfinite(d).all():
        raise ValueError('Displacement must be finite selected-Nx3 or a vector')
    v=mesh.vertices.copy();v[ids]+=d
    affected=np.flatnonzero(np.any(np.isin(mesh.faces,ids),axis=1))
    bounds=np.stack((np.minimum(v[ids].min(0),mesh.vertices[ids].min(0)),
                     np.maximum(v[ids].max(0),mesh.vertices[ids].max(0))))
    for a in (ids,affected,bounds): a.flags.writeable=False
    return LocalEdit(Mesh(v,mesh.faces,mesh.attributes,mesh.closed,mesh.connected),ids,affected,bounds)


def lattice_deform(mesh, bounds, offsets):
    """Trilinear displacement lattice. Zero displacement outside its box.

    `offsets` is (nx,ny,nz,3), with nx,ny,nz >= 2. Continuity at the box
    boundary requires zero offsets on boundary control points. Affine offset
    fields are reproduced exactly inside. This is not a volume-preserving solver.
    """
    b=np.asarray(bounds,float);o=np.asarray(offsets,float)
    if b.shape!=(2,3) or not np.isfinite(b).all() or np.any(b[1]<=b[0]):
        raise ValueError('Bounds must be finite ordered 2x3')
    if o.ndim!=4 or o.shape[-1]!=3 or min(o.shape[:3])<2 or not np.isfinite(o).all():
        raise ValueError('Offsets must be a finite nx,ny,nz,3 lattice')
    inside=np.all((mesh.vertices>=b[0])&(mesh.vertices<=b[1]),axis=1)
    ids=np.flatnonzero(inside)
    if not len(ids): return mesh
    shape=np.array(o.shape[:3]);q=(mesh.vertices[ids]-b[0])/(b[1]-b[0])*(shape-1)
    cell=np.minimum(np.floor(q).astype(int),shape-2);t=q-cell;d=np.zeros((len(ids),3))
    for i in (0,1):
        for j in (0,1):
            for k in (0,1):
                corner=np.array([i,j,k]);w=np.prod(np.where(corner,t,1-t),axis=1)
                ix=cell+corner;d+=w[:,None]*o[ix[:,0],ix[:,1],ix[:,2]]
    return local_displace(mesh,ids,d).mesh


class Multires:
    """Subdivision pyramid storing explicit object-space residual detail.

    Editing a coarse level reconstructs all finer levels and adds their stored
    residuals. Detail survives translations and local coarse edits. Residuals
    do NOT rotate with large deformations: use before skinning, not as a
    tangent-space displacement or automatic retopology system.
    """
    def __init__(self, base, levels=2):
        if not isinstance(levels,int) or not 0<=levels<=4: raise ValueError('Levels must be 0..4')
        self._meshes=[base]; self._details=[]
        for _ in range(levels):
            fine=crease_subdivide(self._meshes[-1])
            self._meshes.append(fine); self._details.append(np.zeros_like(fine.vertices))

    def mesh(self, level=-1):
        return self._meshes[level]

    def edit(self, level, displacement):
        """Return a new pyramid; original remains unchanged."""
        if not isinstance(level,int) or not 0<=level<len(self._meshes): raise ValueError('Invalid level')
        from .surface import sculpt
        result=object.__new__(Multires)
        result._meshes=list(self._meshes);result._details=[d.copy() for d in self._details]
        edited=sculpt(self._meshes[level],displacement)
        result._meshes[level]=edited
        if level:
            prediction=crease_subdivide(result._meshes[level-1])
            result._details[level-1]=edited.vertices-prediction.vertices
        for i in range(level+1,len(result._meshes)):
            prediction=crease_subdivide(result._meshes[i-1])
            result._meshes[i]=sculpt(prediction,result._details[i-1])
        return result


def transfer_attributes(source, target, names=None, max_distance=np.inf):
    """Project target vertices onto source triangles and transfer selected attrs.

    Linear vertex data uses barycentric interpolation, nearest data uses the
    nearest corner of the hit triangle. Face data samples target-face centroids.
    A finite max_distance rejects unsafe distant transfers. This changes no
    topology and is not retopology; categorical seams need corner-domain support.
    """
    from .spatial import MeshBVH
    if np.isnan(max_distance) or max_distance<0: raise ValueError('Invalid distance limit')
    names=list(source.attributes) if names is None else list(names)
    attrs=dict(target.attributes); tree=MeshBVH(source)
    cache={}
    for name in names:
        attr=source.attributes[name]
        if attr.domain not in ('vertex','face'):
            raise ValueError('Transfer currently supports vertex/face attributes only')
        if attr.domain not in cache:
            points=target.vertices if attr.domain=='vertex' else target.vertices[target.faces].mean(axis=1)
            hits=[tree.closest_point(p) for p in points]
            if any(h.distance>max_distance for h in hits): raise ValueError('Transfer exceeds maximum projection distance')
            cache[attr.domain]=hits
        hits=cache[attr.domain];values=[]
        for h in hits:
            if attr.domain=='face': value=attr.values[h.face]
            else:
                data=attr.values[source.faces[h.face]]
                value=np.tensordot(h.barycentric,data,axes=1) if attr.interpolation=='linear' else data[np.argmin(np.linalg.norm(source.vertices[source.faces[h.face]]-h.point,axis=1))]
            values.append(value)
        attrs[name]=Attribute(np.asarray(values),attr.domain,attr.interpolation)
    return Mesh(target.vertices,target.faces,attrs,target.closed,target.connected)
