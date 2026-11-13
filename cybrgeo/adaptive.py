"""Local conforming refinement and explicit surface correspondence.

Not an automatic quad-retopology engine. Refinement keeps the input's piecewise
linear surface exactly unless a caller explicitly evaluates new samples.
"""
from dataclasses import dataclass
import hashlib
import numpy as np
from .model import Mesh, Attribute, mesh_edges
from .spatial import MeshBVH


def _key(mesh):
    h = hashlib.sha256()
    h.update(mesh.vertices.tobytes()); h.update(mesh.faces.tobytes())
    return h.hexdigest()


@dataclass(frozen=True)
class SurfaceAnchor:
    """Snapshot-bound surface coordinate, serializable with to_dict/from_dict."""
    snapshot: str
    face: int
    barycentric: tuple

    @classmethod
    def on(cls, mesh, face, barycentric):
        b = np.asarray(barycentric, float)
        if not isinstance(face, (int, np.integer)) or not 0 <= face < len(mesh.faces):
            raise ValueError('Anchor face must exist')
        if b.shape != (3,) or not np.isfinite(b).all() or b.min() < -1e-12 or abs(b.sum()-1) > 1e-10:
            raise ValueError('Anchor needs nonnegative barycentrics summing to one')
        return cls(_key(mesh), int(face), tuple(map(float, b)))

    def point(self, mesh):
        if self.snapshot != _key(mesh):
            raise ValueError('Anchor belongs to another snapshot; use an explicit correspondence')
        return np.asarray(self.barycentric) @ mesh.vertices[mesh.faces[self.face]]

    def to_dict(self):
        return dict(snapshot=self.snapshot, face=self.face, barycentric=list(self.barycentric))

    @classmethod
    def from_dict(cls, data):
        return cls(data['snapshot'], int(data['face']), tuple(data['barycentric']))


@dataclass(frozen=True)
class Refinement:
    mesh: Mesh
    source_key: str
    parent_faces: np.ndarray
    # Each child corner expressed in its source triangle's barycentrics.
    corner_coordinates: np.ndarray
    changed_faces: np.ndarray
    new_vertices: np.ndarray

    def follow(self, anchor):
        """Exact material-coordinate tracking, including displaced new samples."""
        if anchor.snapshot != self.source_key:
            raise ValueError('Anchor does not belong to refinement input')
        for face in np.flatnonzero(self.parent_faces == anchor.face):
            b = np.linalg.solve(self.corner_coordinates[face].T, anchor.barycentric)
            if b.min() >= -1e-10:
                b = np.maximum(b, 0); b /= b.sum()
                return SurfaceAnchor.on(self.mesh, int(face), b)
        raise ValueError('Anchor is outside refinement correspondence')


def refine(mesh, max_edge, faces=None, evaluate=None):
    """One conforming local pass, splitting selected edges longer than max_edge.

    Shared edges split on BOTH incident faces, avoiding T junctions. Unaffected
    faces/vertices remain exact. `evaluate(points)` is called only on new edge
    samples and returns positions, enabling local terrain/implicit evaluation.
    One pass halves selected original edges; repeat explicitly to meet a budget.
    Validation and array assembly remain whole-mesh work. Edge-domain attributes
    are rejected because internal child edges have no unambiguous source value.
    """
    if not np.isfinite(max_edge) or max_edge <= 0:
        raise ValueError('max_edge must be finite and positive')
    if any(a.domain == 'edge' for a in mesh.attributes.values()):
        raise ValueError('Refinement requires an explicit policy for edge attributes')
    selected = np.arange(len(mesh.faces)) if faces is None else np.asarray(faces)
    if selected.ndim != 1 or not np.issubdtype(selected.dtype, np.integer) or (len(selected) and (selected.min()<0 or selected.max()>=len(mesh.faces))):
        raise ValueError('faces must be existing integer face indices')
    edges = mesh_edges(mesh.faces[selected]) if len(selected) else np.empty((0,2),int)
    edges = edges[np.linalg.norm(mesh.vertices[edges[:,0]]-mesh.vertices[edges[:,1]],axis=1)>max_edge]
    n = len(mesh.vertices)
    mids = (mesh.vertices[edges[:,0]]+mesh.vertices[edges[:,1]])/2
    if evaluate is not None and len(mids):
        mids = np.asarray(evaluate(mids.copy()),float)
        if mids.shape != (len(edges),3) or not np.isfinite(mids).all():
            raise ValueError('evaluate must return finite new-Nx3 positions')
    split = {tuple(e): n+i for i,e in enumerate(edges)}
    output=[]; parents=[]; coordinates=[]; changed=[]
    for fi, face in enumerate(mesh.faces):
        points={int(v):np.eye(3)[i] for i,v in enumerate(face)}
        triangles=[tuple(map(int,face))]
        for i,j in ((0,1),(1,2),(2,0)):
            a,b=int(face[i]),int(face[j]); m=split.get(tuple(sorted((a,b))))
            if m is None: continue
            points[m]=(points[a]+points[b])/2
            next_triangles=[]
            for t in triangles:
                if a in t and b in t:
                    k=next(k for k in range(3) if {t[k],t[(k+1)%3]}=={a,b})
                    u,v,w=t[k],t[(k+1)%3],t[(k+2)%3]
                    next_triangles.extend(((u,m,w),(m,v,w)))
                else: next_triangles.append(t)
            triangles=next_triangles
        if len(triangles)>1: changed.append(fi)
        for t in triangles:
            output.append(t); parents.append(fi); coordinates.append([points[v] for v in t])
    parents=np.asarray(parents); coordinates=np.asarray(coordinates)
    attrs={}
    for name,a in mesh.attributes.items():
        if a.domain=='vertex':
            extra=(a.values[edges[:,0]]+a.values[edges[:,1]])/2 if a.interpolation=='linear' else a.values[edges[:,0]]
            values=np.concatenate((a.values,extra))
        elif a.domain=='face': values=a.values[parents]
        else:
            src=a.values.reshape((len(mesh.faces),3)+a.values.shape[1:])[parents]
            if a.interpolation=='linear': values=np.einsum('fij,fj...->fi...',coordinates,src)
            else: values=src[np.arange(len(src))[:,None],coordinates.argmax(axis=2)]
            values=values.reshape((3*len(output),)+a.values.shape[1:])
        attrs[name]=Attribute(values,a.domain,a.interpolation)
    result=Mesh(np.concatenate((mesh.vertices,mids)),np.asarray(output),attrs,mesh.closed,mesh.connected)
    arrays=[parents,coordinates,np.asarray(changed,dtype=int),np.arange(n,len(result.vertices))]
    for a in arrays: a.flags.writeable=False
    return Refinement(result,_key(mesh),*arrays)


@dataclass(frozen=True)
class Projection:
    mesh: Mesh
    source_faces: np.ndarray
    barycentric: np.ndarray
    distances: np.ndarray


def fit_surface(source, topology, max_distance, names=()):
    """Fit caller-authored replacement topology onto source; transfer vertex data.

    This is a retopology *fitting* primitive, not automatic topology design.
    The bound measures original cage vertex-to-source distances, NOT Hausdorff
    error of the resulting faces. Nearest projection can choose the wrong sheet;
    callers must partition/align ambiguous surfaces. No intersection guarantee.
    """
    if not np.isfinite(max_distance) or max_distance < 0:
        raise ValueError('A finite nonnegative projection limit is required')
    tree=MeshBVH(source); hits=[tree.closest_point(p) for p in topology.vertices]
    distances=np.asarray([h.distance for h in hits])
    if np.any(distances>max_distance): raise ValueError('Projection exceeds max_distance')
    ids=np.asarray([h.face for h in hits]); bary=np.asarray([h.barycentric for h in hits])
    attrs=dict(topology.attributes)
    for name in names:
        a=source.attributes[name]
        if a.domain!='vertex': raise ValueError('Fit transfers vertex data only; preserve cage UV seams explicitly')
        values=a.values[source.faces[ids]]
        values=np.einsum('ij,ij...->i...',bary,values) if a.interpolation=='linear' else values[np.arange(len(values)),bary.argmax(axis=1)]
        attrs[name]=Attribute(values,'vertex',a.interpolation)
    result=Mesh(np.asarray([h.point for h in hits]),topology.faces,attrs,topology.closed,topology.connected)
    for a in (ids,bary,distances): a.flags.writeable=False
    return Projection(result,ids,bary,distances)


def reproject_anchor(anchor, source, replacement, max_distance):
    """Explicit nearest-surface rebinding; not guaranteed semantic identity."""
    if not np.isfinite(max_distance) or max_distance<0: raise ValueError('Finite nonnegative limit required')
    h=MeshBVH(replacement).closest_point(anchor.point(source))
    if h.distance>max_distance: raise ValueError('Anchor rebind exceeds max_distance')
    return SurfaceAnchor.on(replacement,h.face,h.barycentric)
