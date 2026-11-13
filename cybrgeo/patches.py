"""Polynomial patches: explicit parameter domains, continuity and mesh diagnostics.

Not NURBS or a CAD trimming kernel. Rectangular trims are exact in parameter
space. Continuity constraints are exact algebraically; reported geometric
errors are sampled. Intersection diagnostics concern tessellated surfaces only.
"""
from dataclasses import dataclass
import numpy as np
from .curves import bezier, sweep
from .model import Mesh
from .spatial import MeshBVH


def _split(p, t):
    rows=[np.array(p,float)]
    while len(rows[-1])>1: rows.append((1-t)*rows[-1][:-1]+t*rows[-1][1:])
    return np.array([r[0] for r in rows]),np.array([r[-1] for r in rows[::-1]])


def _trim(p, a, b):
    left,_=_split(p,b)
    return _split(left,a/b)[1]


@dataclass(frozen=True)
class BezierPatch:
    """Tensor-product polynomial patch. Control shape (u_degree+1,v_degree+1,3)."""
    control: np.ndarray
    def __post_init__(self):
        p=np.array(self.control,float,copy=True)
        if p.ndim!=3 or p.shape[-1]!=3 or min(p.shape[:2])<2 or not np.isfinite(p).all():
            raise ValueError('Need finite UxVx3 control net, U,V >=2')
        p.flags.writeable=False
        object.__setattr__(self,'control',p)
    def point(self,u,v,du=0,dv=0):
        p=self.control.copy()
        for axis,count in ((0,du),(1,dv)):
            if not isinstance(count,int) or count<0: raise ValueError('Derivative order must be nonnegative integer')
            for _ in range(count):
                degree=p.shape[axis]-1
                if degree==0:return np.zeros(3)
                p=degree*np.diff(p,axis=axis)
        return bezier(np.array([bezier(row,v) for row in p]),u)
    def trim(self,u=(0.,1.),v=(0.,1.)):
        """Exact rectangular restriction, renormalized to [0,1]^2. No polygon holes."""
        for interval in (u,v):
            if len(interval)!=2 or not 0<=interval[0]<interval[1]<=1:raise ValueError('Trim interval must satisfy 0 <= a < b <= 1')
        p=np.stack([_trim(self.control[:,j],*u) for j in range(self.control.shape[1])],axis=1)
        p=np.stack([_trim(row,*v) for row in p])
        return BezierPatch(p)
    def mesh(self,nu=24,nv=24):
        if nu<2 or nv<2:raise ValueError('Need at least 2x2 samples')
        p=np.array([[self.point(u,v) for v in np.linspace(0,1,nv)] for u in np.linspace(0,1,nu)])
        ids=np.arange(nu*nv).reshape(nu,nv);a=ids[:-1,:-1].ravel();b=ids[1:,:-1].ravel();c=ids[1:,1:].ravel();d=ids[:-1,1:].ravel()
        return Mesh(p.reshape(-1,3),np.vstack((np.c_[a,b,c],np.c_[a,c,d])),closed=False)


def join_patch(left,right,order=1):
    """Constrain right u=0 to left u=1 in the SAME v orientation.

C0/C1/C2 in normalized parameter coordinates. C1 implies G1 and C2 implies G2
only on regular patches. Requires matching v degree; preserves left entirely.
Modifies first order+1 right control rows, not a global fairing solve.
"""
    a=left.control;b=right.control.copy();m=len(a)-1;n=len(b)-1
    if order not in (0,1,2) or min(m,n)<order or a.shape[1]!=b.shape[1]:raise ValueError('Unsupported continuity order or incompatible degrees')
    b[0]=a[-1]
    if order>=1:b[1]=b[0]+m/n*(a[-1]-a[-2])
    if order>=2:b[2]=2*b[1]-b[0]+m*(m-1)/(n*(n-1))*(a[-1]-2*a[-2]+a[-3])
    return BezierPatch(b)


def join_metrics(left,right,samples=65):
    """Sampled gap, oriented normal angle and normal-curvature mismatch.

Curvature uses the shared physical tangent plane, comparing second fundamental
forms in an orthonormal tangent basis, rather than raw parameter derivatives.
Degenerate samples fail explicitly. Not a continuous-domain proof.
"""
    if samples<2:raise ValueError('Need at least two seam samples')
    gaps=[];angles=[];curvatures=[]
    for v in np.linspace(0,1,samples):
        jets=[]
        for patch,u in ((left,1.),(right,0.)):
            a=patch.point(u,v,1,0);b=patch.point(u,v,0,1);normal=np.cross(a,b);norm=np.linalg.norm(normal)
            if norm<1e-12:raise ValueError('Singular seam: geometric continuity undefined')
            normal/=norm
            jets.append((a,b,normal,patch.point(u,v,2,0),patch.point(u,v,1,1),patch.point(u,v,0,2)))
        A,B=jets;gaps.append(np.linalg.norm(left.point(1,v)-right.point(0,v)))
        angles.append(np.arccos(np.clip(np.dot(A[2],B[2]),-1,1)))
        e=A[1]/np.linalg.norm(A[1]);f=np.cross(A[2],e);basis=np.stack((e,f),axis=1)
        forms=[]
        for a,b,n,uu,uv,vv in jets:
            J=np.stack((a,b),axis=1);coords=np.linalg.lstsq(J,basis,rcond=None)[0]
            II=np.array([[np.dot(n,uu),np.dot(n,uv)],[np.dot(n,uv),np.dot(n,vv)]])
            forms.append(coords.T@II@coords)
        curvatures.append(np.linalg.norm(forms[0]-forms[1],ord=2))
    return {'samples':samples,'max_gap':float(max(gaps)),'max_normal_angle_radians':float(max(angles)),
            'max_normal_curvature_difference':float(max(curvatures)), 'scope':'sampled seam, regular oriented patches'}


def stitch_patches(left,right,nu=24,nv=24,tolerance=1e-8):
    """Weld corresponding tessellated seam vertices. No proximity-based rematching."""
    if join_metrics(left,right,nv)['max_gap']>tolerance:raise ValueError('Seams disagree; constrain them before stitching')
    a=left.mesh(nu,nv);b=right.mesh(nu,nv)
    mapping=np.r_[np.arange(len(a.vertices)-nv,len(a.vertices)),np.arange(len(a.vertices),len(a.vertices)+len(b.vertices)-nv)]
    return Mesh(np.vstack((a.vertices,b.vertices[nv:])),np.vstack((a.faces,mapping[b.faces])),closed=False)


def surface_safety(mesh):
    """Tessellated non-adjacent contacts; adjacent fold and subtriangle behavior unproven.

The explicit verdict is never 'safe'. Pairs sharing a vertex are excluded by
this BVH query, so callers must not interpret no contacts as certification.
"""
    hits=MeshBVH(mesh).self_intersections()
    return {'nonadjacent_contacts':[(h.face_a,h.face_b) for h in hits],
            'verdict':'intersection_detected' if hits else 'no_nonadjacent_contact_detected',
            'certified':False,'scope':'triangulated surface; adjacent foldovers and between-sample geometry not certified'}


def checked_sweep(profile,path,**kwargs):
    """Build and reject detected non-adjacent intersections. Returns mesh and report."""
    mesh=sweep(profile,path,**kwargs);report=surface_safety(mesh)
    if report['nonadjacent_contacts']:raise ValueError('Sweep has non-adjacent surface intersections')
    return mesh,report


def offset_surface(mesh,distance):
    """Vertex-normal mesh offset with local inversion and nonadjacent contact checks.

Not an exact constant-distance solid shell. Boundaries stay open; no topology
repair is attempted. Rejects detected failures, not a proof of injectivity.
"""
    if not np.isfinite(distance):raise ValueError('Offset must be finite')
    v=mesh.vertices;f=mesh.faces;normals=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
    n=np.zeros_like(v)
    for k in range(3):np.add.at(n,f[:,k],normals)
    length=np.linalg.norm(n,axis=1)
    if np.any(length<1e-12):raise ValueError('Undefined vertex normal')
    p=v+float(distance)*n/length[:,None];new=np.cross(p[f[:,1]]-p[f[:,0]],p[f[:,2]]-p[f[:,0]])
    if np.any(np.sum(new*normals,axis=1)<=0):raise ValueError('Offset collapsed or reversed a face')
    result=Mesh(p,f,mesh.attributes,closed=mesh.closed,connected=mesh.connected);report=surface_safety(result)
    if report['nonadjacent_contacts']:raise ValueError('Offset has non-adjacent surface intersections')
    return result,report
