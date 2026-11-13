"""Conservative closed-solid diagnostics over immutable triangle meshes.

Explicit valid-solid precondition, floating tolerances, no penetration-depth or
engineering certification. Surface contact is conservatively unresolved, never
silently called clear. Oppositely oriented cavity shells are supported by winding.
"""
from dataclasses import dataclass
import numpy as np
from .spatial import MeshBVH

@dataclass(frozen=True)
class Interference:
    state: str  # separated, contained_overlap, contact_or_overlap
    surface_gap: float
    witness: object
    inside_point: object
    inside_owner: object
    required_clearance: float
    allowed_reason: object = None
    gap_kind: str = 'exact_surface_distance'

    @property
    def violation(self):
        return self.allowed_reason is None and (self.state != 'separated' or self.surface_gap < self.required_clearance)

class SolidQuery:
    def __init__(self, mesh, *, assume_valid_solid=False, tolerance=None):
        if not assume_valid_solid or mesh.report()['boundary_edges']:
            raise ValueError('Explicit valid, non-self-intersecting closed solid assumption required')
        self.bvh = MeshBVH(mesh, tolerance=tolerance)
        self.mesh = mesh
        # One surface sample per connected shell suffices only after ruling out
        # surface contacts. This includes disconnected solids and cavity shells.
        unseen=set(range(len(mesh.vertices))); adjacency=[set() for _ in mesh.vertices]
        for a,b,c in mesh.faces:
            adjacency[a].update((int(b),int(c)));adjacency[b].update((int(a),int(c)));adjacency[c].update((int(a),int(b)))
        representatives=[]
        while unseen:
            seed=min(unseen);representatives.append(seed);pending=[seed];unseen.remove(seed)
            while pending:
                for j in adjacency[pending.pop()]:
                    if j in unseen:unseen.remove(j);pending.append(j)
        self.representatives=tuple(representatives)

    def signed_distance(self, point):
        return self.bvh.signed_distance(point, assume_valid_solid=True)

    def interference(self, other, *, clearance=0., allow_reason=None):
        clearance=float(clearance)
        if not np.isfinite(clearance) or clearance<0:raise ValueError('clearance must be finite and nonnegative')
        if allow_reason is not None and (not isinstance(allow_reason,str) or not allow_reason.strip()):
            raise ValueError('Intentional intersection requires a nonempty reason')
        hit=self.bvh.distance(other.bvh);tol=max(self.bvh.tolerance,other.bvh.tolerance)
        state='contact_or_overlap' if hit.distance<=tol else 'separated'
        inside=None;owner=None
        # No-contact shells have constant inside/outside classification. At
        # contact, do not claim absence of penetration from sampled vertices.
        if state=='separated':
            for source,target,label in ((self,other,'a'),(other,self,'b')):
                for index in source.representatives:
                    point=source.mesh.vertices[index]
                    if target.signed_distance(point)<-tol:
                        state='contained_overlap';inside=tuple(map(float,point));owner=label;break
                if inside is not None:break
        return Interference(state,hit.distance,hit,inside,owner,clearance,allow_reason)

    def check_clearance(self, other, *, clearance=0., allow_reason=None):
        """Bounded separation decision; exact distance only for near surfaces.

        Far surface pairs report a threshold lower bound, not a nearest witness.
        Closed-solid containment still runs: empty surface contacts alone never
        establish that one solid is not inside the other.
        """
        clearance = float(clearance)
        if not np.isfinite(clearance) or clearance < 0:
            raise ValueError('clearance must be finite and nonnegative')
        if allow_reason is not None:
            return self.interference(other, clearance=clearance, allow_reason=allow_reason)
        tolerance = max(self.bvh.tolerance, other.bvh.tolerance)
        threshold = clearance + tolerance
        hit = next(self.bvh._pairs(other.bvh, threshold), None)
        if hit is not None:
            return self.interference(other, clearance=clearance)
        for source, target, owner in ((self, other, 'a'), (other, self, 'b')):
            for index in source.representatives:
                point = source.mesh.vertices[index]
                if target.signed_distance(point) < -tolerance:
                    return Interference('contained_overlap', threshold, None,
                                        tuple(map(float,point)), owner, clearance,
                                        gap_kind='threshold_lower_bound')
        return Interference('separated', threshold, None, None, None, clearance,
                            gap_kind='threshold_lower_bound')

    def thickness(self, face_ids):
        """Normal-ray material chords at selected face centroids, NOT min thickness.

        Probe both normal directions; winding selects the material side. Thin
        material below tolerance or ambiguous orientation raises, not a pass.
        Returns (face_id, start, end, chord_length). May miss narrow features.
        """
        result=[];epsilon=max(self.bvh.tolerance*8,np.finfo(float).eps*np.max(np.abs(self.mesh.vertices))*16)
        for face in face_ids:
            if not isinstance(face,(int,np.integer)) or not 0<=face<len(self.mesh.faces):raise ValueError('Invalid face id')
            triangle=self.bvh.triangles[face];p=triangle.mean(axis=0)
            n=np.cross(triangle[1]-triangle[0],triangle[2]-triangle[0]);n/=np.linalg.norm(n)
            directions=[d for d in (n,-n) if self.signed_distance(p+epsilon*d)<0]
            if len(directions)!=1:raise ValueError('Unresolved material side at supplied tolerance')
            d=directions[0];hit=self.bvh.raycast(p+epsilon*d,d)
            if hit is None:raise ValueError('No exit surface found')
            result.append((int(face),tuple(p),tuple(hit.point),float(hit.distance+epsilon)))
        return tuple(result)


def sample_motion(fixed, moving_at, times, *, clearance=0., allow_reason=None):
    """Explicit discrete pose checks. Does NOT certify intervals between samples.

    moving_at(t) returns a validated SolidQuery. Retain every report, including
    intentional overlaps. Caller controls sampling; no hidden motion model.
    """
    times=tuple(float(t) for t in times)
    if not times or not np.isfinite(times).all() or any(a>=b for a,b in zip(times,times[1:])):
        raise ValueError('times must be finite and strictly increasing')
    return tuple((t,fixed.interference(moving_at(t),clearance=clearance,allow_reason=allow_reason)) for t in times)
