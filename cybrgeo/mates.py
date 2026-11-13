"""Geometry-bound frames and rigid mates; no guessed nearest-surface identity.

Feature labels are numeric vertex attributes on actual mesh vertices. Producers
must preserve/recreate their semantic labels after edits. Missing, ambiguous or
geometrically invalid features fail. This is not universal CAD face naming.
"""
from dataclasses import dataclass, replace
import numpy as np
from .deform import transform


def _unit(v):
    v = np.asarray(v, float)
    if v.shape != (3,) or not np.isfinite(v).all() or np.linalg.norm(v) < 1e-12:
        raise ValueError('Expected a finite nonzero direction')
    return v / np.linalg.norm(v)


@dataclass(frozen=True)
class Frame:
    matrix: np.ndarray
    radius: float | None = None

    def __post_init__(self):
        m = np.array(self.matrix, float, copy=True)
        if m.shape != (4, 4) or not np.isfinite(m).all() or not np.allclose(m[3], [0,0,0,1]):
            raise ValueError('Invalid frame')
        if not np.allclose(m[:3,:3].T @ m[:3,:3], np.eye(3), atol=1e-8) or np.linalg.det(m[:3,:3]) < 0:
            raise ValueError('Frame must be a proper rigid transform')
        if self.radius is not None and (not np.isfinite(self.radius) or self.radius <= 0):
            raise ValueError('Radius must be positive and finite')
        m.flags.writeable = False
        object.__setattr__(self, 'matrix', m)

    @property
    def origin(self): return self.matrix[:3,3]

    @property
    def axis(self): return self.matrix[:3,2]

    def fitted_radius(self, radial_clearance):
        if self.radius is None: raise ValueError('Feature is not circular')
        c = float(radial_clearance)
        if not np.isfinite(c) or c < 0 or c >= self.radius - 1e-12 * max(1.,self.radius):
            raise ValueError('Radial clearance must be nonnegative and smaller than bore')
        return self.radius - c


@dataclass(frozen=True)
class Feature:
    """Resolve a named vertex label to a measured circle or planar patch.

    normal_hint chooses the normal sign, not its geometry. x_hint sets a phase
    convention. Both are explicit because a symmetric circle has no unique phase.
    Circle selections must sample the complete rim (maximum angular gap < pi).
    Tolerance is absolute and uses the mesh's coordinate units.
    """
    label: str
    kind: str = 'circle'
    normal_hint: tuple = (0,0,1)
    x_hint: tuple = (1,0,0)
    tolerance: float = 1e-6
    anchor: object = None
    offset: float = 0.

    @classmethod
    def at(cls, anchor, *, x_hint=(1,0,0), offset=0.):
        """An explicit point on a curved triangle surface, not a nearest snap.

        Uses the existing snapshot-bound SurfaceAnchor. The triangle winding
        defines the normal; x_hint projected into its plane defines phase.
        Normal offset does not certify clearance or conform a whole footprint.
        """
        from .adaptive import SurfaceAnchor
        if not isinstance(anchor, SurfaceAnchor):
            raise TypeError('Surface feature requires a SurfaceAnchor')
        if not np.isfinite(offset): raise ValueError('Offset must be finite')
        _unit(x_hint)
        return cls('surface:'+str(anchor.face), kind='surface',
                   x_hint=tuple(x_hint), anchor=anchor, offset=float(offset))

    def follow(self, correspondence):
        """Follow explicit existing lineage; never infer identity by proximity."""
        from .correspondence import Correspondence
        if self.kind != 'surface' or not isinstance(correspondence, Correspondence):
            raise TypeError('Only surface features follow explicit Correspondence')
        result = correspondence.follow(self.anchor)
        return replace(self, label='surface:'+str(result.anchor.face), anchor=result.anchor)

    def resolve(self, mesh):
        if self.kind == 'surface':
            from .adaptive import SurfaceAnchor
            if not isinstance(self.anchor, SurfaceAnchor):
                raise ValueError('Surface feature requires an anchor')
            if not np.isfinite(self.offset): raise ValueError('Offset must be finite')
            point = self.anchor.point(mesh)
            triangle = mesh.vertices[mesh.faces[self.anchor.face]]
            n = _unit(np.cross(triangle[1]-triangle[0], triangle[2]-triangle[0]))
            x = _unit(self.x_hint); x = x - n*(x@n)
            if np.linalg.norm(x) < 1e-6:
                raise ValueError('Phase direction is parallel to surface normal')
            x = _unit(x)
            m = np.eye(4)
            m[:3,:3] = np.column_stack([x, np.cross(n,x), n])
            m[:3,3] = point + self.offset*n
            return Frame(m)

        if self.kind not in ('circle','plane'): raise ValueError('Unknown feature kind')
        if not np.isfinite(self.tolerance) or self.tolerance <= 0: raise ValueError('Invalid tolerance')
        if self.label not in mesh.attributes: raise ValueError('Feature label lost: '+self.label)
        attr = mesh.attributes[self.label]
        if attr.domain != 'vertex' or attr.values.ndim != 1: raise ValueError('Feature needs a scalar vertex label')
        p = mesh.vertices[attr.values > 0.5]
        if len(p) < 3: raise ValueError('Feature needs at least three vertices')
        center = p.mean(axis=0)
        _, s, vt = np.linalg.svd(p-center, full_matrices=False)
        if s[1] <= self.tolerance: raise ValueError('Feature is collinear')
        n = vt[-1]
        hint = _unit(self.normal_hint)
        if abs(n @ hint) < 1e-6: raise ValueError('Normal orientation is ambiguous')
        if n @ hint < 0: n = -n
        if np.max(np.abs((p-center) @ n)) > self.tolerance: raise ValueError('Feature is not planar')
        x = _unit(self.x_hint); x = x - n*(x@n)
        if np.linalg.norm(x) < 1e-6: raise ValueError('Phase direction is parallel to normal')
        x = _unit(x); y = np.cross(n,x)
        uv = (p-center) @ np.stack([x,y],axis=1)
        radius = None
        if self.kind == 'circle':
            a = np.column_stack([2*uv, np.ones(len(uv))])
            fit = np.linalg.lstsq(a, np.sum(uv*uv,axis=1), rcond=None)[0]
            radius = float(np.sqrt(max(0, fit[2]+fit[:2]@fit[:2])))
            lengths = np.linalg.norm(uv-fit[:2],axis=1)
            if radius <= self.tolerance or np.max(np.abs(lengths-radius)) > self.tolerance:
                raise ValueError('Feature is not circular or has multiple rims')
            angles = np.sort(np.arctan2(uv[:,1]-fit[1],uv[:,0]-fit[0]))
            if np.max(np.diff(np.r_[angles,angles[0]+2*np.pi])) >= np.pi:
                raise ValueError('Circle feature must cover the complete rim')
            center = center + fit[0]*x + fit[1]*y
        m = np.eye(4); m[:3,:3] = np.column_stack([x,y,n]); m[:3,3] = center
        return Frame(m, radius)


def mate(source, target, *, angle=0., slide=0., opposed=False):
    """Rigid transform mapping source frame to target, then axial motion.

    Angle is degrees, slide uses geometry units. This permits revolute/prismatic
    movement but is not a collision solver or automatic closed-loop assembly.
    """
    if not np.isfinite([angle,slide]).all(): raise ValueError('Nonfinite mate motion')
    a = np.deg2rad(angle); c,s = np.cos(a),np.sin(a)
    motion = np.eye(4); motion[:3,:3] = [[c,-s,0],[s,c,0],[0,0,1]]
    motion[2,3] = slide
    if opposed: motion[:3,:3] = motion[:3,:3] @ np.diag([1,-1,-1])
    return target.matrix @ motion @ np.linalg.inv(source.matrix)


def place(mesh, source, target, **motion):
    return transform(mesh, mate(source,target,**motion))
