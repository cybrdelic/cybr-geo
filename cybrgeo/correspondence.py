"""One explicit contract for surface references across geometry snapshots.

Exact means exact *supplied lineage*, not exact arithmetic or inferred design
intent. Projection is conditional geometric evidence, never semantic identity.
"""
from dataclasses import dataclass
import numpy as np
from .adaptive import SurfaceAnchor, _key
from .spatial import _closest


class CorrespondenceError(ValueError):
    """Missing, stale, ambiguous or out-of-budget reference. Never silently snap."""


@dataclass(frozen=True)
class Resolution:
    anchor: SurfaceAnchor
    method: str
    confidence: str
    distance: float
    lineage: tuple

    def to_dict(self):
        return dict(anchor=self.anchor.to_dict(), method=self.method,
                    confidence=self.confidence, distance=self.distance,
                    lineage=[list(x) for x in self.lineage])


@dataclass(frozen=True)
class Correspondence:
    """Explicit child triangle corners in source triangle barycentric coordinates.

    This same format describes refinement, topology-preserving deformation,
    reordering and caller-supplied CAD tessellation correspondence. New/unmapped
    faces use parent -1. CAD/remesh adapters must supply provenance; this module
    does not invent correspondence from face numbers or nearby positions.
    """
    source_key: str
    target: object
    parent_faces: np.ndarray
    corner_coordinates: np.ndarray
    operation: str

    @classmethod
    def from_lineage(cls, source, target, parent_faces, corner_coordinates,
                     operation='explicit-lineage'):
        p=np.asarray(parent_faces)
        c=np.asarray(corner_coordinates, float)
        if p.shape!=(len(target.faces),) or not np.issubdtype(p.dtype,np.integer):
            raise ValueError('One integer source face (or -1) per target face required')
        if np.any(p < -1) or np.any(p >= len(source.faces)):
            raise ValueError('Source face out of range')
        if c.shape!=(len(p),3,3) or not np.isfinite(c).all():
            raise ValueError('Finite target-corner source barycentrics required')
        mapped=c[p>=0]
        if len(mapped) and (mapped.min() < -1e-10 or not np.allclose(mapped.sum(2),1,rtol=0,atol=1e-10)):
            raise ValueError('Mapped corners must lie in their source triangle')
        if len(mapped) and np.any(np.abs(np.linalg.det(mapped)) < 1e-14):
            raise ValueError('Degenerate lineage coordinates')
        p=p.copy(); c=c.copy(); p.flags.writeable=False; c.flags.writeable=False
        return cls(_key(source),target,p,c,str(operation))

    @classmethod
    def deformation(cls, source, target):
        """Use only when vertex identities are preserved, not merely equal counts."""
        if not np.array_equal(source.faces,target.faces) or len(source.vertices)!=len(target.vertices):
            raise ValueError('Deformation requires unchanged vertex and triangle indexing')
        return cls.from_lineage(source,target,np.arange(len(source.faces)),
                                np.tile(np.eye(3),(len(source.faces),1,1)), 'deformation')

    @classmethod
    def refinement(cls, source, refinement):
        if refinement.source_key != _key(source):
            raise CorrespondenceError('Refinement belongs to another source')
        return cls.from_lineage(source,refinement.mesh,refinement.parent_faces,
                                refinement.corner_coordinates,'refinement')

    def follow(self, reference):
        previous=reference if isinstance(reference,Resolution) else None
        anchor=previous.anchor if previous else reference
        if anchor.snapshot != self.source_key:
            raise CorrespondenceError('Reference does not belong to correspondence source')
        candidates=[]
        for i in np.flatnonzero(self.parent_faces==anchor.face):
            b=np.linalg.solve(self.corner_coordinates[i].T,anchor.barycentric)
            if b.min() >= -1e-10:
                b=np.maximum(b,0); b/=b.sum()
                a=SurfaceAnchor.on(self.target,int(i),b)
                candidates.append((a,a.point(self.target)))
        if not candidates:
            raise CorrespondenceError('Reference has no surviving explicit lineage')
        # Shared child boundaries are OK only when all descendants agree in space.
        scale=max(1.,float(np.ptp(self.target.vertices,axis=0).max()))
        if any(np.linalg.norm(p-candidates[0][1])>scale*1e-9 for _,p in candidates[1:]):
            raise CorrespondenceError('Lineage branches into distinct descendant surfaces')
        a=candidates[0][0]
        history=previous.lineage if previous else ()
        confidence=previous.confidence if previous else 'exact-lineage'
        return Resolution(a,self.operation,confidence,previous.distance if previous else 0.,
                          history+((self.source_key,a.snapshot,self.operation),))


def project_reference(reference, source, target, max_distance, *,
                      source_regions=None, target_regions=None,
                      max_normal_angle=60., ambiguity_distance=None):
    """Conservative, bounded projection for remesh, LOD or CAD-converted meshes.

    All locally plausible disconnected triangle fans are examined, not only the
    nearest face. More than one fan fails closed, even if one is slightly nearer.
    Optional face-region labels are AUTHOR-SUPPLIED semantic correspondence.
    Normals are oriented; flipped targets fail unless explicitly allowed by the
    angle bound. Unlabelled connected surfaces can still change semantic meaning:
    successful output is 'conditional-projection', never 'exact-lineage'.

    O(F) candidate scan intentionally favors auditable behavior over throughput.
    ambiguity_distance is an absolute excess-distance window, defaulting to the
    full search budget. A narrower window weakens ambiguity detection explicitly.
    """
    previous=reference if isinstance(reference,Resolution) else None
    anchor=previous.anchor if previous else reference
    point=anchor.point(source)
    if not np.isfinite(max_distance) or max_distance<0:
        raise ValueError('Finite nonnegative distance budget required')
    margin=max_distance if ambiguity_distance is None else ambiguity_distance
    if not np.isfinite(margin) or margin<0 or not 0<=max_normal_angle<=180:
        raise ValueError('Invalid ambiguity or normal budget')
    ids=np.arange(len(target.faces))
    if (source_regions is None)!=(target_regions is None):
        raise ValueError('Supply BOTH source and target face-region labels')
    if source_regions is not None:
        sr=np.asarray(source_regions); tr=np.asarray(target_regions)
        if sr.shape!=(len(source.faces),) or tr.shape!=(len(target.faces),):
            raise ValueError('Region labels must match face counts')
        ids=ids[tr==sr[anchor.face]]
    t=source.vertices[source.faces[anchor.face]]
    normal=np.cross(t[1]-t[0],t[2]-t[0]); normal/=np.linalg.norm(normal)
    cosine=np.cos(np.deg2rad(max_normal_angle)); hits=[]
    for i in ids:
        tri=target.vertices[target.faces[i]]
        n=np.cross(tri[1]-tri[0],tri[2]-tri[0]); n/=np.linalg.norm(n)
        if np.dot(n,normal) < cosine-1e-12: continue
        q,b=_closest(point,tri); d=float(np.linalg.norm(q-point))
        if d<=max_distance+1e-12: hits.append((int(i),d,b))
    if not hits:
        raise CorrespondenceError('No compatible target within distance/region/normal bounds')
    best=min(x[1] for x in hits)
    hits=[h for h in hits if h[1]<=best+margin+1e-12]
    # Connected local fans, not whole-object components: two sides of a folded
    # connected sheet must still be treated as separate local candidates.
    remaining={i for i,_,_ in hits}; fans=[]
    vertices={i:set(map(int,target.faces[i])) for i in remaining}
    while remaining:
        seed=remaining.pop(); fan={seed}; frontier=[seed]
        while frontier:
            i=frontier.pop()
            neighbors={j for j in remaining if vertices[i]&vertices[j]}
            remaining-=neighbors; fan|=neighbors; frontier.extend(neighbors)
        fans.append(fan)
    if len(fans)!=1:
        raise CorrespondenceError(f'Ambiguous reference: {len(fans)} plausible surface fans; supply explicit lineage or region labels')
    i,d,b=min(hits,key=lambda h:(h[1],h[0]))
    a=SurfaceAnchor.on(target,i,b)
    method='region-projection' if source_regions is not None else 'bounded-projection'
    history=previous.lineage if previous else ()
    return Resolution(a,method,'conditional-projection',d,
                      history+((_key(source),a.snapshot,method),))
