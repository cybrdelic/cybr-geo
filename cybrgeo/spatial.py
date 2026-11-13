"""Triangle BVH queries with geometric witnesses, not scene-specific collision rules.

Distances are unsigned *surface* distances. Nested solids can have positive
surface clearance: this is NOT a volume-penetration or engineering safety test.
Intersections use floating-point tolerances, not exact arithmetic predicates.
Face IDs refer to this immutable Mesh snapshot; refit after vertex edits, rebuild after topology edits.
"""
from dataclasses import dataclass
import heapq
import itertools
import numpy as np


@dataclass(frozen=True)
class PointHit:
    distance: float
    point: np.ndarray
    face: int
    barycentric: np.ndarray


@dataclass(frozen=True)
class PairHit:
    distance: float
    face_a: int
    face_b: int
    point_a: np.ndarray
    point_b: np.ndarray


@dataclass(frozen=True)
class ClearanceReport:
    minimum: float
    nearest: object
    violations: tuple
    allowed_pairs: tuple
    # This report intentionally makes no claim about enclosed volume overlap.
    surface_only: bool = True


@dataclass
class _Node:
    low: np.ndarray
    high: np.ndarray
    ids: object = None
    left: object = None
    right: object = None


def _vec(value):
    x = np.asarray(value, dtype=float)
    if x.shape != (3,) or not np.isfinite(x).all():
        raise ValueError('Expected a finite 3-vector')
    return x


def _box_distance(a, b, c, d):
    return float(np.linalg.norm(np.maximum(np.maximum(a-d, c-b), 0)))


def _closest(p, t):
    # Orthogonal plane projection, followed by clamped edge candidates.
    a,b,c=t; u=b-a; v=c-a
    cross=np.cross(u,v); n2=np.dot(cross,cross)
    q=p-cross*np.dot(p-a,cross)/n2
    uv=np.column_stack((u,v))
    s,r=np.linalg.lstsq(uv,q-a,rcond=None)[0]
    if s>=0 and r>=0 and s+r<=1:
        return q,np.array([1-s-r,s,r])
    best=None
    for i,j in ((0,1),(1,2),(2,0)):
        e=t[j]-t[i]; f=np.clip(np.dot(p-t[i],e)/np.dot(e,e),0,1)
        q=t[i]+f*e; w=np.zeros(3); w[i]=1-f; w[j]=f
        item=(float(np.dot(p-q,p-q)),q,w)
        if best is None or item[0]<best[0]: best=item
    return best[1:]


def _ray(o,d,t,tolerance):
    e1=t[1]-t[0];e2=t[2]-t[0];p=np.cross(d,e2);det=np.dot(e1,p)
    # Angular degeneracy threshold, independent of triangle scale.
    if abs(det)<=1e-14*np.linalg.norm(e1)*np.linalg.norm(e2): return None
    inv=1/det;s=o-t[0];u=np.dot(s,p)*inv;q=np.cross(s,e1)
    v=np.dot(d,q)*inv;distance=np.dot(e2,q)*inv
    bt=tolerance/max(np.linalg.norm(e1),np.linalg.norm(e2))
    if u < -bt or v < -bt or u+v>1+bt or distance < -tolerance:return None
    return max(0.,float(distance)),np.array([1-u-v,u,v])


def _segments(a,b,c,d):
    # Closest points on bounded segments, including parallel segments.
    u=b-a;v=d-c;w=a-c
    aa=np.dot(u,u);bb=np.dot(u,v);cc=np.dot(v,v)
    dd=np.dot(u,w);ee=np.dot(v,w);den=aa*cc-bb*bb
    s=np.clip((bb*ee-cc*dd)/den,0,1) if den>1e-14*aa*cc else 0.
    t=(bb*s+ee)/cc
    if t<0:t=0.;s=np.clip(-dd/aa,0,1)
    elif t>1:t=1.;s=np.clip((bb-dd)/aa,0,1)
    return a+s*u,c+t*v


def _triangle_pair(a,b,tolerance):
    candidates=[]
    # Edge-face tests catch transverse crossings that edge/edge distance misses.
    for first,second,swap in ((a,b,False),(b,a,True)):
        for i,j in ((0,1),(1,2),(2,0)):
            delta=first[j]-first[i];length=np.linalg.norm(delta)
            hit=_ray(first[i],delta/length,second,tolerance)
            if hit is not None and hit[0]<=length+tolerance:
                p=first[i]+np.clip(hit[0],0,length)*delta/length
                return 0.,p,p.copy()
        for p in first:
            q,_=_closest(p,second)
            candidates.append((q,p) if swap else (p,q))
    for i,j in ((0,1),(1,2),(2,0)):
        for k,l in ((0,1),(1,2),(2,0)):
            candidates.append(_segments(a[i],a[j],b[k],b[l]))
    p,q=min(candidates,key=lambda pair:np.dot(pair[0]-pair[1],pair[0]-pair[1]))
    return float(np.linalg.norm(p-q)),p.copy(),q.copy()


class MeshBVH:
    """Median-split immutable triangle index; no optional third-party dependency."""
    def __init__(self,mesh,leaf_size=8,tolerance=None):
        if not isinstance(leaf_size,int) or leaf_size<1:raise ValueError('leaf_size must be positive')
        self.mesh=mesh;self.triangles=mesh.vertices[mesh.faces]
        scale=float(np.linalg.norm(np.ptp(mesh.vertices,axis=0)))
        self.tolerance=scale*1e-9 if tolerance is None else float(tolerance)
        if not np.isfinite(self.tolerance) or self.tolerance<0:raise ValueError('Invalid tolerance')
        self.low=self.triangles.min(axis=1);self.high=self.triangles.max(axis=1)
        centers=(self.low+self.high)/2
        def build(ids):
            node=_Node(self.low[ids].min(axis=0),self.high[ids].max(axis=0))
            if len(ids)<=leaf_size:node.ids=ids;return node
            axis=np.argmax(np.ptp(centers[ids],axis=0));ids=ids[np.argsort(centers[ids,axis])]
            middle=len(ids)//2;node.left=build(ids[:middle]);node.right=build(ids[middle:]);return node
        self.root=build(np.arange(len(self.triangles)))

    def refit(self, mesh):
        """Return a new index after vertex edits, sharing untouched BVH branches.

        Connectivity must be identical. Changed triangles are detected rather
        than trusting a caller's dirty list. Scanning/copying arrays is O(F);
        only affected leaf boxes and their ancestors are geometrically refit.
        Large deformations may degrade traversal quality: rebuild then.
        """
        if mesh.vertices.shape != self.mesh.vertices.shape or not np.array_equal(mesh.faces,self.mesh.faces):
            raise ValueError('Refit requires unchanged topology; rebuild after remeshing')
        result=object.__new__(MeshBVH)
        result.mesh=mesh;result.tolerance=self.tolerance
        moved=np.any(mesh.vertices!=self.mesh.vertices,axis=1)
        dirty=np.any(moved[mesh.faces],axis=1)
        result.triangles=self.triangles.copy()
        result.triangles[dirty]=mesh.vertices[mesh.faces[dirty]]
        result.low=self.low.copy();result.high=self.high.copy()
        result.low[dirty]=result.triangles[dirty].min(axis=1)
        result.high[dirty]=result.triangles[dirty].max(axis=1)
        def update(node):
            if node.ids is not None:
                if not np.any(dirty[node.ids]):return node
                return _Node(result.low[node.ids].min(axis=0),result.high[node.ids].max(axis=0),node.ids)
            left=update(node.left);right=update(node.right)
            if left is node.left and right is node.right:return node
            return _Node(np.minimum(left.low,right.low),np.maximum(left.high,right.high),None,left,right)
        result.root=update(self.root)
        return result

    def closest_point(self,point):
        point=_vec(point);best=None;queue=[];counter=itertools.count()
        def add(node):heapq.heappush(queue,(_box_distance(point,point,node.low,node.high),next(counter),node))
        add(self.root)
        while queue:
            lower,_,node=heapq.heappop(queue)
            if best is not None and lower>best.distance:break
            if node.ids is None:add(node.left);add(node.right);continue
            for face in node.ids:
                q,w=_closest(point,self.triangles[face]);distance=float(np.linalg.norm(q-point))
                if best is None or distance<best.distance:best=PointHit(distance,q.copy(),int(face),w)
        return best

    def raycast(self,origin,direction,max_distance=np.inf):
        """Nearest forward hit. Distance is world units, independent of |direction|."""
        o=_vec(origin);d=_vec(direction);length=np.linalg.norm(d)
        if length==0 or np.isnan(max_distance) or max_distance<0:raise ValueError('Invalid ray')
        d=d/length;best=None;pending=[self.root];limit=float(max_distance)
        def intersects(node):
            near,far=0.,limit
            for axis in range(3):
                if d[axis]==0:
                    if o[axis]<node.low[axis]-self.tolerance or o[axis]>node.high[axis]+self.tolerance:return False
                else:
                    a=(node.low[axis]-self.tolerance-o[axis])/d[axis];b=(node.high[axis]+self.tolerance-o[axis])/d[axis]
                    near=max(near,min(a,b));far=min(far,max(a,b))
            return near<=far
        while pending:
            node=pending.pop()
            if not intersects(node):continue
            if node.ids is None:pending.extend((node.left,node.right));continue
            for face in node.ids:
                hit=_ray(o,d,self.triangles[face],self.tolerance)
                if hit is not None and hit[0]<=limit:
                    limit=hit[0];best=PointHit(limit,o+d*limit,int(face),hit[1])
        return best

    def _pairs(self,other,limit,allowed=(),same=False):
        allowed={tuple(map(int,p)) for p in allowed};pending=[(self.root,other.root)];seen=set()
        while pending:
            a,b=pending.pop()
            if _box_distance(a.low,a.high,b.low,b.high)>limit:continue
            if a.ids is not None and b.ids is not None:
                for i in a.ids:
                    for j in b.ids:
                        i,j=int(i),int(j)
                        if same and i>=j:continue
                        if (i,j) in allowed or (same and (j,i) in allowed):continue
                        if (i,j) in seen:continue
                        seen.add((i,j))
                        if _box_distance(self.low[i],self.high[i],other.low[j],other.high[j])>limit:continue
                        distance,p,q=_triangle_pair(self.triangles[i],other.triangles[j],max(self.tolerance,other.tolerance))
                        if distance<=limit:yield PairHit(distance,i,j,p,q)
            elif a.ids is None and (b.ids is not None or np.prod(a.high-a.low)>=np.prod(b.high-b.low)):
                pending.extend(((a.left,b),(a.right,b)))
            else:pending.extend(((a,b.left),(a,b.right)))

    def distance(self,other,allowed_pairs=()):
        """Nearest unexcluded triangle pair; None if every pair was excluded."""
        allowed=set(map(tuple,allowed_pairs));best=None
        queue=[];counter=itertools.count()
        def add(a,b):
            lower=_box_distance(a.low,a.high,b.low,b.high)
            if best is None or lower<=best.distance:
                heapq.heappush(queue,(lower,next(counter),a,b))
        add(self.root,other.root)
        while queue:
            lower,_,a,b=heapq.heappop(queue)
            if best is not None and lower>best.distance:break
            if a.ids is not None and b.ids is not None:
                for i in a.ids:
                    for j in b.ids:
                        i,j=int(i),int(j)
                        if (i,j) in allowed:continue
                        if best is not None and _box_distance(self.low[i],self.high[i],other.low[j],other.high[j])>best.distance:continue
                        d,p,q=_triangle_pair(self.triangles[i],other.triangles[j],max(self.tolerance,other.tolerance))
                        if best is None or d<best.distance:best=PairHit(d,i,j,p,q)
                        if best.distance==0:return best
            elif a.ids is None and (b.ids is not None or np.linalg.norm(a.high-a.low)>=np.linalg.norm(b.high-b.low)):
                add(a.left,b);add(a.right,b)
            else:add(a,b.left);add(a,b.right)
        return best

    def contacts(self,other,distance=None,allowed_pairs=()):
        """All face pairs within distance; exclusions are explicit face-ID pairs."""
        limit=max(self.tolerance,other.tolerance) if distance is None else float(distance)
        if not np.isfinite(limit) or limit<0:raise ValueError('Invalid contact distance')
        return tuple(self._pairs(other,limit,allowed_pairs))

    def self_intersections(self,ignore_adjacent=True):
        """Near-zero face contacts. Default omits pairs sharing ANY vertex.

        Consequently adjacent-fold overlap is NOT certified absent. Set
        ignore_adjacent=False for all contacts, including ordinary manifold
        seams. This is a diagnostic, not an exact self-intersection proof.
        """
        result=[]
        for hit in self._pairs(self,self.tolerance,same=True):
            if ignore_adjacent and np.intersect1d(self.mesh.faces[hit.face_a],self.mesh.faces[hit.face_b]).size:continue
            result.append(hit)
        return tuple(result)

    def clearance(self,other,minimum,allowed_pairs=()):
        """Surface gap diagnostics. Allowed contact is explicit, never inferred."""
        minimum=float(minimum)
        if not np.isfinite(minimum) or minimum<0:raise ValueError('Invalid minimum clearance')
        allowed=tuple(tuple(map(int,p)) for p in allowed_pairs)
        hits=self.contacts(other,minimum,allowed)
        return ClearanceReport(minimum,self.distance(other,allowed),
                               tuple(h for h in hits if h.distance<minimum),allowed)

    def signed_distance(self,point,*,assume_valid_solid=False):
        """Negative inside using generalized winding number.

        Caller must explicitly attest a non-self-intersecting closed solid.
        Topological closure alone cannot establish that precondition. Reversed
        winding is accepted. Near-boundary points return zero.
        """
        if not assume_valid_solid or self.mesh.report()['boundary_edges']:
            raise ValueError('signed_distance requires an explicitly assumed valid closed solid')
        point=_vec(point);hit=self.closest_point(point)
        if hit.distance<=self.tolerance:return 0.
        a,b,c=(self.triangles-point).transpose(1,0,2)
        la=np.linalg.norm(a,axis=1);lb=np.linalg.norm(b,axis=1);lc=np.linalg.norm(c,axis=1)
        numerator=np.einsum('ij,ij->i',a,np.cross(b,c))
        denominator=(la*lb*lc+np.einsum('ij,ij->i',a,b)*lc+
                     np.einsum('ij,ij->i',b,c)*la+np.einsum('ij,ij->i',c,a)*lb)
        winding=np.sum(2*np.arctan2(numerator,denominator))/(4*np.pi)
        return -hit.distance if abs(winding)>.5 else hit.distance
