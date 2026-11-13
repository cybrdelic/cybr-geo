"""Optional analytic-solid construction, backed by CadQuery/OpenCascade.

This is NOT a triangle-mesh Boolean. Keep analytic Solid values until export.
No materials/UVs are silently transferred across topology-changing operations.
Selectors are evaluated on the current solid, not persistent feature IDs.
"""
from dataclasses import dataclass
import math
import numpy as np
from .model import Mesh


def _cq():
    try:
        import cadquery
    except ImportError as error:
        raise ImportError('Analytic solids require the optional cadquery>=2.7 dependency') from error
    return cadquery


def _positive(value, name):
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and positive')


@dataclass(frozen=True)
class Solid:
    """An immutable-operation wrapper around a validated analytic B-rep.

    Coordinates use caller-selected consistent units. Empty intersections are
    explicit valid values (is_empty); they cannot be exported as a Mesh.
    The underlying shape is exposed for interoperability: do not mutate it.
    """
    shape: object

    def __post_init__(self):
        if not self.shape.isValid():
            raise ValueError('OpenCascade rejected this B-rep as invalid')
        if not self.is_empty and not self.shape.Solids():
            raise ValueError('Expected volumetric solids, not a sheet or wire')

    @property
    def is_empty(self):
        return len(self.shape.Solids()) == 0 and len(self.shape.Faces()) == 0

    @property
    def volume(self):
        return 0.0 if self.is_empty else float(self.shape.Volume())

    @property
    def solid_count(self):
        return len(self.shape.Solids())

    @classmethod
    def box(cls, size):
        for d in size: _positive(d, 'box dimension')
        return cls(_cq().Workplane('XY').box(*size).val())

    @classmethod
    def cylinder(cls, radius, height):
        _positive(radius, 'radius'); _positive(height, 'height')
        return cls(_cq().Workplane('XY').circle(radius).extrude(height).val())

    @classmethod
    def extrude(cls, outline, height):
        """Extrude a planar closed polygon; Boolean subtraction adds holes."""
        _positive(height, 'height')
        p=np.asarray(outline,dtype=float)
        if p.ndim!=2 or p.shape[1]!=2 or len(p)<3 or not np.isfinite(p).all():
            raise ValueError('outline must be finite Nx2, at least three points')
        return cls(_cq().Workplane('XY').polyline(p.tolist()).close().extrude(height).val())

    def move(self, xyz):
        if len(xyz)!=3 or not np.isfinite(xyz).all(): raise ValueError('finite xyz required')
        return Solid(self.shape.translate(tuple(xyz)))

    def rotate(self, axis_start, axis_end, degrees):
        if not np.isfinite([*axis_start,*axis_end,degrees]).all() or np.allclose(axis_start,axis_end):
            raise ValueError('finite nonzero axis and angle required')
        return Solid(self.shape.rotate(tuple(axis_start),tuple(axis_end),degrees))

    def union(self, *others):
        return Solid(self.shape.fuse(*(s.shape for s in others)).clean())

    def subtract(self, *others):
        return Solid(self.shape.cut(*(s.shape for s in others)).clean())

    def intersect(self, other):
        return Solid(self.shape.intersect(other.shape).clean())

    def _workplane(self):
        return _cq().Workplane('XY').newObject([self.shape])

    def shell(self, thickness, opening=None):
        """Offset walls; negative=inward, positive=outward.

        opening is an optional CadQuery face selector, e.g. '>Z'. None gives
        a sealed hollow solid. Fails rather than quietly clipping bad offsets.
        """
        if not math.isfinite(thickness) or thickness==0: raise ValueError('nonzero finite thickness required')
        w=self._workplane()
        if opening is not None:
            w=w.faces(opening)
            if not w.size(): raise ValueError('opening selector matched no faces')
        try: return Solid(w.shell(thickness).val())
        except Exception as e: raise ValueError('Shell failed; offset may cross itself or exceed local feature size') from e

    def fillet(self, radius, edges):
        """Round selected analytic edges. Radius must fit surrounding features."""
        _positive(radius,'radius')
        w=self._workplane().edges(edges)
        if not w.size(): raise ValueError('edge selector matched no edges')
        try: return Solid(w.fillet(radius).val())
        except Exception as e: raise ValueError('Fillet failed; reduce radius or choose compatible edges') from e

    def chamfer(self, distance, edges):
        _positive(distance,'distance')
        w=self._workplane().edges(edges)
        if not w.size(): raise ValueError('edge selector matched no edges')
        try: return Solid(w.chamfer(distance).val())
        except Exception as e: raise ValueError('Chamfer failed; reduce distance or choose compatible edges') from e

    def mesh(self, tolerance=0.05, angular_tolerance=0.1, weld_tolerance=1e-8):
        """Approximate B-rep by triangles, weld face seams, then validate Mesh.

        tolerance is chordal deviation in model units; angular_tolerance radians.
        weld_tolerance MUST be much smaller than the narrowest intentional gap.
        A thin gap below the weld tolerance can invalidate conversion: no repair
        or topology changes are silently performed. Export has no UV attributes.
        """
        for name,value in [('tolerance',tolerance),('angular_tolerance',angular_tolerance),('weld_tolerance',weld_tolerance)]: _positive(value,name)
        if self.is_empty: raise ValueError('An empty solid has no mesh')
        vertices,faces=self.shape.tessellate(tolerance,angular_tolerance)
        v=np.array([p.toTuple() for p in vertices]); f=np.asarray(faces,dtype=np.int64)
        from scipy.spatial import cKDTree
        parent=np.arange(len(v))
        def root(i):
            while parent[i]!=i:
                parent[i]=parent[parent[i]];i=parent[i]
            return i
        for a,b in cKDTree(v).query_pairs(weld_tolerance):
            a,b=root(a),root(b)
            if a!=b: parent[b]=a
        roots=np.array([root(i) for i in range(len(v))])
        unique,inverse=np.unique(roots,return_inverse=True)
        # Validation deliberately rejects collapsed or nonmanifold triangles.
        return Mesh(v[unique],inverse[f],closed=True,connected=False)
