"""General polynomial surface construction with explicit boundary contracts.

Continuity is parametric C0/C1/C2, not arbitrary geometric G2 reparameterization.
Polygon trims are in UV, and tessellation is approximate. No collision guarantee.
"""
from dataclasses import dataclass
from types import MappingProxyType
import numpy as np
from .patches import BezierPatch
from .model import Mesh, Attribute


def trim_patch(patch, outline, holes=(), *, spacing=.05):
    """Tessellate arbitrary simple polygon UV outline/holes on a Bezier patch.

    Curved trim boundaries must be sampled explicitly. Uses existing constrained
    boundary recovery and fails when recovery fails, never fills missing holes.
    Spacing controls UV sampling, not world-space error or triangle quality.
    """
    from .profiles import extrude, validate_profile
    rings=validate_profile(outline,holes)
    if not np.isfinite(spacing) or spacing<=0:
        raise ValueError('Trim spacing must be positive')
    if any(np.any((r<0)|(r>1)) for r in rings):
        raise ValueError('Trim coordinates must lie in [0,1]^2')
    # Sample the boundary too: an affine UV edge maps to a curved spatial edge.
    # Four corner samples are not sufficient even for an untrimmed patch.
    counts=[np.maximum(1,np.ceil(np.linalg.norm(np.roll(r,-1,axis=0)-r,axis=1)/spacing).astype(int)) for r in rings]
    if sum(int(c.sum()) for c in counts)>100_000:
        raise ValueError('Trim boundary exceeds sampling budget')
    rings=[np.concatenate([a+(b-a)*np.arange(n)[:,None]/n for a,b,n in zip(r,np.roll(r,-1,axis=0),c)]) for r,c in zip(rings,counts)]
    cap=extrude(rings[0],1,holes=rings[1:],spacing=spacing)
    # Extract lower planar cap and retain its exact UV connectivity.
    f=cap.faces[np.all(cap.vertices[cap.faces,2]==0,axis=1)][:,::-1]
    used=np.unique(f); remap=np.full(len(cap.vertices),-1,int);remap[used]=np.arange(len(used))
    uv=cap.vertices[used,:2]
    points=np.array([patch.point(u,v) for u,v in uv])
    return Mesh(points,remap[f],{'uv':Attribute(uv), 'normal':Attribute(_normals(patch,uv))},closed=False)


@dataclass(frozen=True)
class Seam:
    a: str
    edge_a: str
    b: str
    edge_b: str
    order: int = 1
    reverse: bool = False
    scale: float = 1.
    def __post_init__(self):
        if self.edge_a not in ('u0','u1','v0','v1') or self.edge_b not in ('u0','u1','v0','v1'):
            raise ValueError('Edge must be u0/u1/v0/v1')
        if self.order not in (0,1,2) or not np.isfinite(self.scale) or self.scale<=0:
            raise ValueError('Seam needs order 0..2 and positive derivative scale')


def _edge_rows(shape,edge,order):
    """Coefficient stencils for inward cross-edge derivatives, control by control."""
    ids=np.arange(np.prod(shape)).reshape(shape)
    rows=ids if edge[0]=='u' else ids.T
    if edge[1]=='1':rows=rows[::-1]
    degree=len(rows)-1
    if degree<order:raise ValueError('Insufficient cross-edge degree')
    out=[]
    for k in range(order+1):
        if k==0:coeff=[1.]
        elif k==1:coeff=[-degree,degree]
        else:coeff=[degree*(degree-1),-2*degree*(degree-1),degree*(degree-1)]
        M=np.zeros((rows.shape[1],np.prod(shape)))
        for i,c in enumerate(coeff):M[np.arange(len(M)),rows[i]]=c
        out.append(M)
    return out


def _system(patches,seams):
    names=list(patches);sizes=[p.control.shape[0]*p.control.shape[1] for p in patches.values()]
    starts=dict(zip(names,np.r_[0,np.cumsum(sizes)[:-1]]));total=sum(sizes);rows=[]
    for seam in seams:
        if seam.a not in patches or seam.b not in patches or seam.a==seam.b:raise ValueError('Seam names must be distinct existing patches')
        a=_edge_rows(patches[seam.a].control.shape[:2],seam.edge_a,seam.order)
        b=_edge_rows(patches[seam.b].control.shape[:2],seam.edge_b,seam.order)
        if a[0].shape[0]!=b[0].shape[0]:raise ValueError('Seam edge degrees must match; elevate degree explicitly')
        for k,(aa,bb) in enumerate(zip(a,b)):
            if seam.reverse:bb=bb[::-1]
            row=np.zeros((len(aa),total));i=starts[seam.a];j=starts[seam.b]
            row[:,i:i+aa.shape[1]]=aa
            # Inward derivative directions oppose across a join.
            row[:,j:j+bb.shape[1]]-=(-1)**k*seam.scale**k*bb
            rows.append(row)
    return np.vstack(rows) if rows else np.zeros((0,total)), starts


def _fairness(patches):
    total=sum(p.control.shape[0]*p.control.shape[1] for p in patches.values());rows=[];offset=0
    for p in patches.values():
        ids=np.arange(p.control.shape[0]*p.control.shape[1]).reshape(p.control.shape[:2])+offset
        for axis in (ids,ids.T):
            for i in range(1,len(axis)-1):
                for j in range(axis.shape[1]):
                    row=np.zeros(total);row[axis[i-1,j]]=1;row[axis[i,j]]=-2;row[axis[i+1,j]]=1;rows.append(row)
        offset+=ids.size
    return np.asarray(rows) if rows else np.zeros((0,total))


def fit_network(patches,seams,*,fixed=(),fairness=0.,tolerance=1e-8):
    """Constrained global least-squares fairing; original controls are soft targets.

    Fixed named patches are exact constraints. Incompatible cycles/fixed targets
    fail without publishing partial geometry. Fairness is control-net second
    differences, not an invariant curvature energy. Dense solver suits small nets.
    """
    patches=dict(patches);seams=tuple(seams)
    if not patches or any(not isinstance(p,BezierPatch) for p in patches.values()):raise ValueError('Need named Bezier patches')
    if not np.isfinite(fairness) or fairness<0:raise ValueError('Fairness must be nonnegative')
    C,starts=_system(patches,seams);target=np.vstack([p.control.reshape(-1,3) for p in patches.values()]);rhs=np.zeros((len(C),3));n=len(target)
    for name in fixed:
        if name not in patches:raise ValueError('Unknown fixed patch')
        count=patches[name].control.shape[0]*patches[name].control.shape[1];ids=np.arange(starts[name],starts[name]+count)
        C=np.vstack((C,np.eye(n)[ids]));rhs=np.vstack((rhs,target[ids]))
    if len(C):
        U,s,Vh=np.linalg.svd(C,full_matrices=True);rank=np.sum(s>max(C.shape)*np.finfo(float).eps*(s[0] if len(s) else 1))
        x0=np.linalg.lstsq(C,rhs,rcond=None)[0]
        if np.max(np.abs(C@x0-rhs))>tolerance:raise ValueError('Inconsistent fixed/seam constraints')
        N=Vh[rank:].T
    else:x0=np.zeros_like(target);N=np.eye(n)
    D=_fairness(patches);A=np.vstack((np.eye(n),np.sqrt(fairness)*D));b=np.vstack((target,np.zeros((len(D),3))))
    fitted=x0+N@np.linalg.lstsq(A@N,b-A@x0,rcond=None)[0] if N.shape[1] else x0
    if len(C) and np.max(np.abs(C@fitted-rhs))>tolerance:raise ValueError('Solver failed constraint tolerance')
    result={name:BezierPatch(fitted[starts[name]:starts[name]+p.control.shape[0]*p.control.shape[1]].reshape(p.control.shape)) for name,p in patches.items()}
    return SurfaceNetwork(result,seams,tolerance=tolerance)


@dataclass(frozen=True)
class SurfaceNetwork:
    """Immutable published surface state; declared seams are construction invariants.

    Raw BezierPatch/Mesh remain available for intentional disconnected surfaces.
    This wrapper enforces only declared seams; it does not infer joins, collision,
    arbitrary trim correspondence, or whether a car has a convincing design.
    """
    patches: dict
    seams: tuple = ()
    tolerance: float = 1e-8
    def __post_init__(self):
        p=dict(self.patches);s=tuple(self.seams)
        if not p or any(not isinstance(v,BezierPatch) for v in p.values()):raise ValueError('Need named Bezier patches')
        if not np.isfinite(self.tolerance) or self.tolerance<=0:raise ValueError('Tolerance must be positive')
        C,_=_system(p,s);x=np.vstack([v.control.reshape(-1,3) for v in p.values()])
        if len(C) and np.max(np.abs(C@x))>self.tolerance:raise ValueError('Boundary contract violated; use fit_network or correct controls')
        # Parametric algebra alone permits collapsed seams. Reject sampled singularities.
        for seam in s:
            for name,edge in ((seam.a,seam.edge_a),(seam.b,seam.edge_b)):
                patch=p[name]
                for t in np.linspace(0,1,17):
                    u,v=(float(edge[1]),t) if edge[0]=='u' else (t,float(edge[1]))
                    cross=np.cross(patch.point(u,v,1,0),patch.point(u,v,0,1))
                    if np.linalg.norm(cross)<1e-12:raise ValueError('Singular seam; continuity undefined')
        object.__setattr__(self,'patches',MappingProxyType(p));object.__setattr__(self,'seams',s)
    def with_patch(self,name,patch,*,refit=False,fixed=(),fairness=0.):
        if name not in self.patches:raise ValueError('Unknown patch')
        p=dict(self.patches);p[name]=patch
        return fit_network(p,self.seams,fixed=fixed,fairness=fairness,tolerance=self.tolerance) if refit else SurfaceNetwork(p,self.seams,self.tolerance)
    def meshes(self,nu=24,nv=24):
        """Separate patch meshes. Shared boundaries coincide; not a welded solid."""
        return {name:p.mesh(nu,nv) for name,p in self.patches.items()}
    def report(self):
        C,_=_system(self.patches,self.seams);x=np.vstack([p.control.reshape(-1,3) for p in self.patches.values()])
        return {'patches':len(self.patches),'seams':len(self.seams),'max_control_constraint_residual':float(np.max(np.abs(C@x))) if len(C) else 0.,'global_intersection_certified':False}


def _normals(patch, uv):
    normals=np.array([np.cross(patch.point(u,v,1,0),patch.point(u,v,0,1)) for u,v in uv])
    size=np.linalg.norm(normals,axis=1)
    if np.any(size<1e-12):raise ValueError('Singular sampled surface; normal undefined')
    return normals/size[:,None]


def tessellate_network(network, samples=24):
    """Synchronized full-patch tessellation with welded declared edges."""
    if isinstance(samples,bool) or not isinstance(samples,int) or samples<2:
        raise ValueError('samples must be an integer >=2')
    uv=np.array([[u,v] for u in np.linspace(0,1,samples) for v in np.linspace(0,1,samples)])
    meshes={}
    for name,p in network.patches.items():
        m=p.mesh(samples,samples)
        meshes[name]=Mesh(m.vertices,m.faces,{'uv':Attribute(uv)},closed=False)
    return stitch_network_meshes(network,meshes)


def stitch_network_meshes(network, meshes):
    """Weld declared edges of UV-tagged full or trimmed patch meshes.

    Matching retained seam segments and sample coordinates are REQUIRED. A
    mismatch fails; this does not remesh or silently bridge a trim opening.
    Returns analytic corner normals, corner UVs and face patch_index. This is
    an open design surface, not automatic shelling or collision certification.
    """
    if set(meshes)!=set(network.patches):raise ValueError('Provide one mesh for every network patch')
    names=list(network.patches);starts={};offset=0;uvs={}
    for name in names:
        m=meshes[name];attr=m.attributes.get('uv')
        if attr is None or attr.domain!='vertex' or attr.values.shape!=(len(m.vertices),2):
            raise ValueError('Network stitching requires vertex UV correspondence')
        uv=attr.values
        if np.any((uv<0)|(uv>1)):raise ValueError('UV outside patch')
        expected=np.array([network.patches[name].point(u,v) for u,v in uv])
        if np.max(np.linalg.norm(expected-m.vertices,axis=1))>network.tolerance:
            raise ValueError('Mesh no longer corresponds to its source patch')
        uvs[name]=uv;starts[name]=offset;offset+=len(m.vertices)
    parent=np.arange(offset)
    def root(a):
        while parent[a]!=a:
            parent[a]=parent[parent[a]];a=parent[a]
        return a
    def boundary(name,edge,reverse=False):
        uv=uvs[name];axis=0 if edge[0]=='u' else 1;side=int(edge[1]);ids=np.flatnonzero(np.abs(uv[:,axis]-side)<1e-12)
        t=uv[ids,1-axis];t=1-t if reverse else t;order=np.argsort(t);ids=ids[order];t=t[order]
        if len(t)>1 and np.any(np.diff(t)<1e-12):raise ValueError('Duplicate seam samples')
        m=meshes[name];f=m.faces;edges=np.concatenate([f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]]);keys=np.sort(edges,axis=1)
        unique,count=np.unique(keys,axis=0,return_counts=True);boundary=unique[count==1];lookup={int(i):j for j,i in enumerate(ids)}
        segments=sorted(tuple(sorted((lookup[int(a)],lookup[int(b)]))) for a,b in boundary if int(a) in lookup and int(b) in lookup)
        return ids+starts[name],t,segments
    direction={'u0':-1,'u1':1,'v0':1,'v1':-1};adj={k:[] for k in names}
    for seam in network.seams:
        parity=-direction[seam.edge_a]*direction[seam.edge_b]*(-1 if seam.reverse else 1)
        adj[seam.a].append((seam.b,parity));adj[seam.b].append((seam.a,parity))
        a,ta,sa=boundary(seam.a,seam.edge_a);b,tb,sb=boundary(seam.b,seam.edge_b,seam.reverse)
        if len(ta)!=len(tb) or sa!=sb or not np.allclose(ta,tb,rtol=0,atol=1e-12):
            raise ValueError('Retained seam tessellation differs; share boundary samples before stitching')
        for x,y in zip(a,b):parent[root(y)]=root(x)
    orientation={}
    for name in names:
        if name in orientation:continue
        orientation[name]=1;queue=[name]
        while queue:
            a=queue.pop()
            for b,p in adj[a]:
                value=orientation[a]*p
                if b in orientation:
                    if orientation[b]!=value:raise ValueError('Nonorientable seam network')
                else:orientation[b]=value;queue.append(b)
    points=[];faces=[];normals=[];uvcorners=[];labels=[]
    for index,name in enumerate(names):
        patch=network.patches[name];m=meshes[name];f=m.faces.copy()
        if orientation[name]<0:f=f[:,::-1]
        points.append(m.vertices);faces.append(f+starts[name])
        normals.append((_normals(patch,uvs[name])*orientation[name])[f].reshape(-1,3))
        uvcorners.append(uvs[name][f].reshape(-1,2));labels.extend([index]*len(f))
    points=np.vstack(points);faces=np.vstack(faces);roots=np.array([root(i) for i in range(offset)])
    unique,inverse=np.unique(roots,return_inverse=True)
    if np.max(np.linalg.norm(points-points[roots],axis=1))>network.tolerance:
        raise ValueError('Sampled boundary contract violated')
    return Mesh(points[unique],inverse[faces],
        {'normal':Attribute(np.vstack(normals),'corner'), 'uv':Attribute(np.vstack(uvcorners),'corner'),
         'patch_index':Attribute(np.asarray(labels),'face','nearest')},closed=False,connected=False)
