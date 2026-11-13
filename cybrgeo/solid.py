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
    def extrude(cls, outline, height, *, offset=0.):
        """Extrude a planar polygon with an optional signed planar offset.

        Negative offset insets the region. The CAD kernel resolves corner joins;
        offset collapse or invalid construction fails instead of rescaling the
        polygon. Boolean subtraction adds holes.
        """
        _positive(height, 'height')
        from .profiles import validate_profile
        p=validate_profile(outline)[0]
        if not math.isfinite(offset): raise ValueError('offset must be finite')
        profile = _cq().Workplane('XY').polyline(p.tolist()).close()
        if offset:
            profile = profile.offset2D(offset)
        return cls(profile.extrude(height).val())

    @classmethod
    def loft(cls, sections, *, ruled=True):
        """Capped analytic loft through corresponding planar polygon sections.

        Uses OpenCascade via CadQuery, not a new geometry kernel. Sections are
        ordered NxMx3 closed polygon samples (do not repeat the first point).
        Ruled side faces are the default; smooth mode is backend interpolation,
        not a promised curvature or self-intersection certificate. No holes,
        topology/name preservation, or automatic section correspondence.
        """
        from .profiles import validate_planar_ring
        s = np.asarray(sections, dtype=float)
        if (s.ndim != 3 or s.shape[0] < 2 or s.shape[1] < 3 or s.shape[2] != 3
                or not np.isfinite(s).all() or not isinstance(ruled, bool)):
            raise ValueError('Solid loft needs finite corresponding NxMx3 sections and boolean ruled')
        for ring in s:
            if np.any(np.linalg.norm(ring-np.roll(ring, -1, axis=0), axis=1) <= 1e-10):
                raise ValueError('Solid loft sections contain repeated adjacent points')
            validate_planar_ring(ring)
        if np.any(np.linalg.norm(np.diff(s, axis=0).reshape(len(s)-1, -1), axis=1) <= 1e-10):
            raise ValueError('Solid loft sections must not repeat')
        cq = _cq()
        try:
            wires = [cq.Wire.makePolygon([tuple(p) for p in ring], close=True) for ring in s]
            result = cls(cq.Solid.makeLoft(wires, ruled=ruled))
            if result.solid_count != 1 or result.volume <= 0:
                raise ValueError('Loft did not produce one positive-volume solid')
            return result
        except Exception as error:
            raise ValueError('Analytic loft failed; check planar section order and shape compatibility') from error

    def move(self, xyz):
        if len(xyz)!=3 or not np.isfinite(xyz).all(): raise ValueError('finite xyz required')
        return Solid(self.shape.translate(tuple(xyz)))

    def scale(self, factor):
        """Return a uniformly scaled solid about the origin; factor must be positive.

        Delegates to the analytic backend. Nonuniform scaling, reflection and
        feature-name preservation are not promised by this operation.
        """
        _positive(factor, 'scale factor')
        return Solid(self.shape.scale(float(factor)))

    def rotate(self, axis_start, axis_end, degrees):
        if not np.isfinite([*axis_start,*axis_end,degrees]).all() or np.allclose(axis_start,axis_end):
            raise ValueError('finite nonzero axis and angle required')
        return Solid(self.shape.rotate(tuple(axis_start),tuple(axis_end),degrees))

    def union(self, *others):
        return Solid(self.shape.fuse(*(s.shape for s in others)).clean())

    def subtract(self, *others):
        return Solid(self.shape.cut(*(s.shape for s in others)).clean())

    def cut_through(self, outline, *more_outlines, frame=None):
        """Subtract planar polygon prisms along a rigid frame's local Z axis.

        With no frame, existing XY/Z behavior is unchanged. Profiles are local
        XY coordinates in the supplied Frame. Cutter depth is derived from the
        finished host bounds projected onto the cutting axis, not guessed by the
        caller. Batch cutting delegates to OpenCascade. No curved passages or
        automatic machining plan is implied; the batch must remove material.
        """
        from .mates import Frame
        from .profiles import validate_profile
        if self.is_empty: raise ValueError('Cannot cut an empty host')
        if frame is None: frame=Frame(np.eye(4))
        if not isinstance(frame, Frame): raise TypeError('cut_through frame must be a rigid Frame')
        bounds=self.shape.BoundingBox()
        corners=np.array([[x,y,z] for x in (bounds.xmin,bounds.xmax)
                         for y in (bounds.ymin,bounds.ymax)
                         for z in (bounds.zmin,bounds.zmax)])
        depths=(corners-frame.origin)@frame.axis
        low,high=float(depths.min()),float(depths.max());span=high-low
        if not math.isfinite(span) or span<=0: raise ValueError('Invalid host extent')
        margin=max(span*1e-6,1e-6)
        start=frame.origin+frame.axis*(low-margin)
        cq=_cq();plane=cq.Plane(origin=tuple(start),xDir=tuple(frame.matrix[:3,0]),normal=tuple(frame.axis))
        cutters=[]
        for profile in (outline,*more_outlines):
            ring=validate_profile(profile)[0]
            cutters.append(Solid(cq.Workplane(plane).polyline(ring.tolist()).close().extrude(span+2*margin).val()))
        result=self.subtract(*cutters)
        if self.volume-result.volume<=max(1e-12,self.volume*1e-12):
            raise ValueError('Through-cut profile removes no resolved host material')
        return result

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

    def mesh(self, tolerance=0.05, angular_tolerance=0.1, weld_tolerance=1e-8, *, canonicalize_degenerate=False):
        """Approximate B-rep by triangles, weld face seams, then validate Mesh.

        tolerance is chordal deviation in model units; angular_tolerance radians.
        weld_tolerance MUST be much smaller than the narrowest intentional gap.
        A thin gap below the weld tolerance can invalidate conversion: no repair
        or topology changes are silently performed. Export has no UV attributes.

        Opt-in canonicalize_degenerate removes only CAD triangles whose original
        coordinates already contain an exactly repeated point and have exactly
        zero area, after welding confirms repeated indices. This handles CAD
        parameter poles without deleting any finite-area triangles. It emits a
        warning with the count and still requires full closed-manifold validation.
        Finite-area faces collapsed by excessive weld tolerance remain errors.
        """
        for name,value in [('tolerance',tolerance),('angular_tolerance',angular_tolerance),('weld_tolerance',weld_tolerance)]: _positive(value,name)
        if not isinstance(canonicalize_degenerate, bool):
            raise ValueError('canonicalize_degenerate must be boolean')
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
        welded_faces=inverse[f]
        if canonicalize_degenerate:
            original=v[f]
            exact_repeat=(np.all(original[:,0]==original[:,1],axis=1)
                          | np.all(original[:,1]==original[:,2],axis=1)
                          | np.all(original[:,2]==original[:,0],axis=1))
            exact_zero=np.all(np.cross(original[:,1]-original[:,0],original[:,2]-original[:,0])==0.,axis=1)
            collapsed=(welded_faces[:,0]==welded_faces[:,1]) | (welded_faces[:,1]==welded_faces[:,2]) | (welded_faces[:,2]==welded_faces[:,0])
            remove=exact_repeat & exact_zero & collapsed
            if np.any(remove):
                import warnings
                warnings.warn(f'Canonicalized {int(remove.sum())} exactly zero-area CAD tessellation faces; no finite-area faces removed', RuntimeWarning, stacklevel=2)
                welded_faces=welded_faces[~remove]
        # Every other topology defect still fails. No gap filling or mesh repair.
        return Mesh(v[unique],welded_faces,closed=True,connected=False)
