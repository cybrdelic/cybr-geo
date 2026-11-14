"""Curves and surface construction. No named objects or asset recipes.

Sampling tolerance concerns the polyline approximation only; these constructors
validate topology, not geometric self-intersection or engineering suitability.
"""
import numpy as np
from .model import Mesh, Attribute
from .surface import loft


def bezier(control, t):
    """Evaluate a Bézier curve of any degree using stable de Casteljau steps."""
    p = np.asarray(control, float)
    t = np.asarray(t, float)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) < 1 or not np.isfinite(p).all():
        raise ValueError('Control points must be finite Nx3, N >= 1')
    if not np.isfinite(t).all() or np.any((t < 0) | (t > 1)):
        raise ValueError('Curve parameters must be in [0,1]')
    q = np.broadcast_to(p, t.shape + p.shape).copy()
    u = t[..., None, None]
    for n in range(len(p)-1, 0, -1):
        q = (1-u)*q[..., :n, :] + u*q[..., 1:n+1, :]
    return q[..., 0, :]


def sample_curve(curve, count=64, tolerance=1e-4, max_depth=16):
    """Adaptively flatten callable curve, then resample by polyline arc length.

Quarter-point and midpoint tests catch curvature missed by midpoint alone.
The tolerance is a local chord-deviation criterion, not a global error proof
for an arbitrary callable. A depth limit fails rather than silently accepting.
"""
    if count < 2 or tolerance <= 0 or max_depth < 1:
        raise ValueError('Need count >= 2, tolerance > 0 and max_depth >= 1')
    def point(t):
        p = np.asarray(curve(t), float)
        if p.shape != (3,) or not np.isfinite(p).all():
            raise ValueError('Curve must return a finite 3-vector')
        return p
    points = [point(0.)]
    def split(a, b, pa, pb, depth):
        ts = np.array([.25,.5,.75])
        ps = np.array([point(a+(b-a)*t) for t in ts])
        error = np.linalg.norm(ps - (pa[None,:]*(1-ts[:,None])+pb*ts[:,None]),axis=1).max()
        if error <= tolerance:
            points.append(pb)
        elif depth >= max_depth:
            raise ValueError('Curve sampling tolerance not reached at max_depth')
        else:
            mid=(a+b)/2; split(a,mid,pa,ps[1],depth+1); split(mid,b,ps[1],pb,depth+1)
    split(0.,1.,points[0],point(1.),0)
    return resample_polyline(points,count)


def resample_polyline(points, count, closed=False):
    """Equal-distance samples; closed output never repeats its first vertex."""
    p=np.asarray(points,float)
    if p.ndim!=2 or p.shape[1] not in (2,3) or len(p)<2 or not np.isfinite(p).all() or count<2:
        raise ValueError('Need finite Nx2/Nx3 points and count >= 2')
    if closed and np.allclose(p[0],p[-1],rtol=0,atol=1e-12): p=p[:-1]
    if closed: p=np.vstack((p,p[0]))
    d=np.linalg.norm(np.diff(p,axis=0),axis=1)
    if np.any(d<=1e-12): raise ValueError('Consecutive duplicate curve points')
    s=np.r_[0.,np.cumsum(d)]
    x=np.linspace(0,s[-1],count,endpoint=not closed)
    return np.column_stack([np.interp(x,s,p[:,k]) for k in range(p.shape[1])])


def transport_frames(path, initial_normal=None, twist=0., *, closed=False):
    """Discrete rotation-minimizing frames (tangent, normal, binormal).

Twist is radians per sample or a scalar total angle, ramped by arc length.
A 180-degree reversal is ambiguous and rejected. For closed=True, the path
has no duplicate endpoint. Closure holonomy is spread continuously by arc length,
not forced into the last segment. Closed twist must be a scalar whole turn;
arbitrary per-point twists have no unambiguous periodic endpoint.
"""
    p=np.asarray(path,float)
    if p.ndim!=2 or p.shape[1]!=3 or len(p)<2 or not np.isfinite(p).all():
        raise ValueError('Path must be finite Nx3')
    if closed and len(p)<3: raise ValueError('Closed path needs at least three points')
    d=np.diff(np.vstack((p,p[0])) if closed else p,axis=0); length=np.linalg.norm(d,axis=1)
    if np.any(length<=1e-12): raise ValueError('Path has duplicate adjacent points')
    edges=d/length[:,None]
    t=edges+np.roll(edges,1,axis=0) if closed else np.vstack((edges[0],edges[:-1]+edges[1:],edges[-1]))
    norm=np.linalg.norm(t,axis=1)
    if np.any(norm<1e-8): raise ValueError('Path reverses direction by 180 degrees')
    t/=norm[:,None]
    n=np.empty_like(t)
    seed=np.eye(3)[np.argmin(abs(t[0]))] if initial_normal is None else np.asarray(initial_normal,float)
    if seed.shape!=(3,) or not np.isfinite(seed).all(): raise ValueError('Invalid initial normal')
    n[0]=seed-t[0]*np.dot(seed,t[0])
    if np.linalg.norm(n[0])<1e-10: raise ValueError('Initial normal parallel to tangent')
    n[0]/=np.linalg.norm(n[0])
    def carry(vector, a, b):
        axis=np.cross(a,b); c=np.dot(a,b); sine=np.linalg.norm(axis)
        if c < -1+1e-8: raise ValueError('Frame reversal is ambiguous')
        if sine < 1e-12: result=vector.copy()
        else:
            axis/=sine
            result=vector*c+np.cross(axis,vector)*sine+axis*np.dot(axis,vector)*(1-c)
        result-=b*np.dot(result,b)
        return result/np.linalg.norm(result)
    for i in range(1,len(t)):
        n[i]=carry(n[i-1],t[i-1],t[i])
    distance=np.r_[0,np.cumsum(length)]
    if closed: distance=distance[:-1]
    fraction=distance/length.sum()
    b=np.cross(t,n)
    angle=np.asarray(twist,float)
    if closed:
        if angle.ndim!=0 or not np.isfinite(angle):
            raise ValueError('Closed twist must be a scalar whole turn')
        if not np.isclose(float(angle)/(2*np.pi),round(float(angle)/(2*np.pi)),rtol=0,atol=1e-10):
            raise ValueError('Closed twist must be a whole number of turns')
        end=carry(n[-1],t[-1],t[0])
        correction=np.arctan2(np.dot(t[0],np.cross(end,n[0])),np.dot(end,n[0]))
        angle=(float(angle)+correction)*fraction
    elif angle.ndim==0: angle=float(angle)*fraction
    if angle.shape!=(len(p),) or not np.isfinite(angle).all(): raise ValueError('Twist must be scalar or one angle per path point')
    rotated=n*np.cos(angle)[:,None]+b*np.sin(angle)[:,None]
    return t,rotated,np.cross(t,rotated)


def sweep(profile, path, scale=1., twist=0., initial_normal=None, caps=None, *, closed=False):
    """Sweep a closed 2D profile along a 3D path with transported frames.

Scale is scalar, one isotropic scale per ring, or (rings,2) anisotropic scales.
Input profile is ordered counterclockwise; caller controls seam/correspondence.
closed=True connects the last ring to the first with periodic frames, never
coincident duplicate endpoint vertices. Caps default on for open paths and off
for closed paths; explicit caps=True on a closed path is an error. This does not
certify self-intersection freedom for thick profiles or tight path curvature.
"""
    q=np.asarray(profile,float); p=np.asarray(path,float)
    if q.ndim!=2 or q.shape[1]!=2 or len(q)<3 or not np.isfinite(q).all(): raise ValueError('Profile must be finite Nx2')
    if np.linalg.norm(q[0]-q[-1])<1e-12: raise ValueError('Do not repeat the closed profile endpoint')
    from .profiles import validate_profile
    validate_profile(q)  # Validate without changing seam order or winding.
    if caps is None: caps=not closed
    if closed and caps: raise ValueError('A closed sweep has no end caps')
    t,n,b=transport_frames(p,initial_normal,twist,closed=closed)
    s=np.asarray(scale,float)
    if s.ndim==0: s=np.full((len(p),2),s)
    elif s.shape==(len(p),): s=np.repeat(s[:,None],2,axis=1)
    if s.shape!=(len(p),2) or not np.isfinite(s).all() or np.any(s<=0): raise ValueError('Scale must be positive scalar, N, or Nx2')
    sections=p[:,None,:]+q[None,:,0,None]*s[:,None,0,None]*n[:,None,:]+q[None,:,1,None]*s[:,None,1,None]*b[:,None,:]
    if not closed: return loft(sections,caps=caps)
    rings,samples=len(p),len(q)
    ids=np.arange(rings*samples).reshape(rings,samples)
    a=ids.ravel(); b=np.roll(ids,-1,axis=1).ravel()
    c=np.roll(ids,-1,axis=0).ravel(); d=np.roll(np.roll(ids,-1,axis=0),-1,axis=1).ravel()
    faces=np.stack((np.column_stack((a,b,d)),np.column_stack((a,d,c))),axis=1).reshape(-1,3)
    return Mesh(sections.reshape(-1,3),faces,
                {'section_coordinate':Attribute(np.repeat(np.arange(rings,dtype=float),samples))})


def matched_loft(sections, samples=64, align=True, caps=True):
    """Resample unequal closed rings and optionally find cyclic correspondence.

Alignment minimizes centered, scale-normalized point distance against the prior
ring, considering both directions. Not a semantic feature-matching guarantee:
turn align off and provide corresponding samples when seam identity matters.
"""
    rings=[resample_polyline(s,samples,closed=True) for s in sections]
    if len(rings)<2 or any(r.shape[1]!=3 for r in rings): raise ValueError('Need at least two 3D sections')
    def normalized(r):
        r=r-r.mean(axis=0); return r/max(np.linalg.norm(r),1e-12)
    if align:
        for i in range(1,len(rings)):
            target=normalized(rings[i-1]); best=None
            for candidate in (rings[i],rings[i][::-1]):
                for shift in range(samples):
                    r=np.roll(candidate,shift,axis=0); cost=np.sum((normalized(r)-target)**2)
                    if best is None or cost<best[0]: best=(cost,r)
            rings[i]=best[1]
    return loft(rings,caps=caps)


def coons_patch(bottom, top, left, right, nu=32, nv=32, corner_tolerance=1e-8):
    """C0 transfinite patch from four boundary functions returning 3-vectors.

bottom/top run left-to-right; left/right run bottom-to-top. Interpolates all
boundaries exactly at sampled points. Does not promise tangent continuity.
"""
    if nu<2 or nv<2: raise ValueError('Patch needs at least 2x2 samples')
    u=np.linspace(0,1,nu); v=np.linspace(0,1,nv)
    B=np.array([bottom(x) for x in u],float); T=np.array([top(x) for x in u],float)
    L=np.array([left(x) for x in v],float); R=np.array([right(x) for x in v],float)
    if B.shape!=(nu,3) or T.shape!=(nu,3) or L.shape!=(nv,3) or R.shape!=(nv,3) or not all(np.isfinite(x).all() for x in (B,T,L,R)):
        raise ValueError('Boundaries must return finite 3-vectors')
    if max(np.linalg.norm(a-b) for a,b in ((B[0],L[0]),(B[-1],R[0]),(T[0],L[-1]),(T[-1],R[-1])))>corner_tolerance:
        raise ValueError('Patch boundary corners disagree')
    U=u[:,None,None]; V=v[None,:,None]
    bilinear=(1-U)*(1-V)*B[0]+U*(1-V)*B[-1]+(1-U)*V*T[0]+U*V*T[-1]
    points=(1-V)*B[:,None,:]+V*T[:,None,:]+(1-U)*L[None,:,:]+U*R[None,:,:]-bilinear
    ids=np.arange(nu*nv).reshape(nu,nv); a=ids[:-1,:-1].ravel(); b=ids[1:,:-1].ravel(); c=ids[1:,1:].ravel(); d=ids[:-1,1:].ravel()
    faces=np.vstack((np.column_stack((a,b,c)),np.column_stack((a,c,d))))
    uv=np.stack(np.meshgrid(u,v,indexing='ij'),axis=-1).reshape(-1,2)
    return Mesh(points.reshape(-1,3),faces,{'uv':Attribute(uv)},closed=False)


def sweep_shell(outer, inner, path, scale=1., twist=0., initial_normal=None):
    """Sweep corresponding nested closed profiles into an open-ended solid tube.

Both profiles use the same orientation and vertex count. End annuli connect
outer and inner walls. Profile nesting is validated before sweeping; no
constant physical wall thickness is implied by scaled sections or curvature.
"""
    outer=np.asarray(outer,float); inner=np.asarray(inner,float)
    if outer.shape!=inner.shape: raise ValueError('Inner and outer profiles need corresponding samples')
    from .profiles import validate_profile
    validate_profile(outer, [inner])  # Preserve authored seam indices/correspondence.
    area=lambda q: np.sum(q[:,0]*np.roll(q[:,1],-1)-q[:,1]*np.roll(q[:,0],-1))
    if np.sign(area(outer))!=np.sign(area(inner)):
        raise ValueError('Inner and outer sweep profiles must have the same orientation')
    a=sweep(outer,path,scale,twist,initial_normal,caps=False)
    b=sweep(inner,path,scale,twist,initial_normal,caps=False)
    offset=len(a.vertices); n=len(outer); count=len(path)
    faces=[*a.faces,*(b.faces[:,::-1]+offset)]
    for ring,reverse in ((0,False),(count-1,True)):
        for j in range(n):
            x=ring*n+j; y=ring*n+(j+1)%n
            pair=((x,x+offset,y+offset),(x,y+offset,y))
            # Boundary rim orientation opposite outer wall boundary.
            faces.extend(tuple(reversed(f)) if reverse else f for f in pair)
    return Mesh(np.vstack((a.vertices,b.vertices)),np.array(faces,dtype=np.int64))
