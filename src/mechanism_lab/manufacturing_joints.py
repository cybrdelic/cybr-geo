"""Native hollow spans and sleeved lap interfaces in body-local millimetres.

Construction is adapted from ``build_structural_prototypes.py`` in
cybrdelic/cybr-arx6-compact, commit 85bd6d2ce5b7c567d08b3ffccf2a9be46005aabf.
The source's actual fillets, split surfaces, pockets and bores are retained.
Parameter checks describe nominal CAD fits, not printing or load capacity.
CYBR GEO is GPL-2.0-only; source attribution is retained in NOTICE-SOURCES.md.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math

from .native_construction import NativeInterfaceRequirements


SOURCE = {
    'repository': 'https://github.com/cybrdelic/cybr-arx6-compact',
    'commit': '85bd6d2ce5b7c567d08b3ffccf2a9be46005aabf',
    'path': 'revisions/p6b-print-first/source/build_structural_prototypes.py',
    'git_blob_sha1': 'aef765032a8e1779078acd0ef763413686989719',
    'sha256': '8e4c76d215d1f2c1e882f7756e6415ec5dd7bb1ab866b9c341aa0187b7065520',
}


def _number(value, label, *, positive=False, nonnegative=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{label} must be a finite number')
    if positive and value <= 0 or nonnegative and value < 0:
        raise ValueError(f'{label} must be {"positive" if positive else "nonnegative"}')
    return float(value)


def _positions(values, label):
    try:
        result = tuple(_number(value, label) for value in values)
    except TypeError as error:
        raise ValueError(f'{label} must contain finite positions') from error
    if not result or len(set(result)) != len(result):
        raise ValueError(f'{label} must contain unique positions')
    return result


def centered_box(width_mm, length_mm, height_mm, center):
    """A native box centered on an explicitly supplied millimetre frame."""
    import cadquery as cq
    dimensions = [_number(value, 'Box dimension', positive=True)
                  for value in (width_mm, length_mm, height_mm)]
    position = tuple(_number(value, 'Box center') for value in center)
    if len(position) != 3:
        raise ValueError('Box center must have three coordinates')
    return cq.Solid.makeBox(*dimensions, cq.Vector(*(p-d/2 for p, d in zip(position, dimensions))))


def _valid_single(shape, label):
    if not shape.isValid() or len(shape.Solids()) != 1 or shape.Volume() <= 0:
        raise ValueError(f'{label} must be a valid positive-volume single solid')
    return shape


def native_bounds(shape):
    """Analytic BREP bounds, independent of any cached display tessellation."""
    import cadquery as cq
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    bounds = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape.wrapped, bounds, False, False)
    return cq.BoundBox(bounds)


def _axial_edges(shape, length):
    # Selecting direction as well as length avoids accidentally rounding the
    # transverse edges when an editable span length equals its width/height.
    return [edge for edge in shape.Edges()
            if abs(edge.BoundingBox().ylen-length) < 1e-5
            and max(edge.BoundingBox().xlen, edge.BoundingBox().zlen) < 1e-5]


@dataclass(frozen=True)
class RoundedBoxTube:
    """A Y-directed hollow span with real axial-edge fillets."""

    width_mm: float
    height_mm: float
    length_mm: float
    wall_mm: float
    center_y_mm: float = 0.
    inner_radius_mm: float = 1.2
    outer_radius_mm: float = 0.

    def __post_init__(self):
        for name in ('width_mm', 'height_mm', 'length_mm', 'wall_mm'):
            _number(getattr(self, name), name, positive=True)
        _number(self.center_y_mm, 'center_y_mm')
        for name in ('inner_radius_mm', 'outer_radius_mm'):
            _number(getattr(self, name), name, nonnegative=True)
        if min(self.inner_width_mm, self.inner_height_mm) <= 0:
            raise ValueError('Tube wall leaves no hollow section')
        if self.inner_radius_mm >= min(self.inner_width_mm, self.inner_height_mm)/2:
            raise ValueError('Inner fillet consumes the hollow section')
        if self.outer_radius_mm >= min(self.width_mm, self.height_mm)/2:
            raise ValueError('Outer fillet consumes the outer section')
        if self.outer_radius_mm > self.wall_mm+self.inner_radius_mm:
            raise ValueError('Outer fillet can cut through the nominal wall')

    @property
    def inner_width_mm(self):
        return self.width_mm-2*self.wall_mm

    @property
    def inner_height_mm(self):
        return self.height_mm-2*self.wall_mm

    def native_shape(self):
        outer = centered_box(self.width_mm, self.length_mm, self.height_mm, (0, self.center_y_mm, 0))
        inner = centered_box(self.inner_width_mm, self.length_mm+2, self.inner_height_mm,
                             (0, self.center_y_mm, 0))
        if self.outer_radius_mm:
            outer = outer.fillet(self.outer_radius_mm, _axial_edges(outer, self.length_mm))
        if self.inner_radius_mm:
            inner = inner.fillet(self.inner_radius_mm, _axial_edges(inner, self.length_mm+2))
        return _valid_single(outer.cut(inner), 'Hollow span')


@dataclass(frozen=True)
class CaptiveNutPattern:
    """Opposed roof/floor pockets that open into the actual hollow span.

    The positive opening overlap cuts through the inner wall face. Remaining
    wall and hole/pocket dimensions are nominal geometry, not insert strength.
    """

    positions_y_mm: tuple[float, ...]
    positions_x_mm: tuple[float, ...]
    hole_radius_mm: float
    pocket_width_mm: float
    pocket_depth_mm: float
    opening_overlap_mm: float = .05

    def __post_init__(self):
        object.__setattr__(self, 'positions_y_mm', _positions(self.positions_y_mm, 'Pocket Y positions'))
        object.__setattr__(self, 'positions_x_mm', _positions(self.positions_x_mm, 'Pocket X positions'))
        for name in ('hole_radius_mm', 'pocket_width_mm', 'pocket_depth_mm', 'opening_overlap_mm'):
            _number(getattr(self, name), name, positive=True)
        if self.pocket_width_mm <= 2*self.hole_radius_mm:
            raise ValueError('Captive pocket must surround its clearance hole')
        if self.opening_overlap_mm >= self.pocket_depth_mm:
            raise ValueError('Pocket must leave material behind its opening')
        for positions in (self.positions_x_mm, self.positions_y_mm):
            if any(abs(a-b) <= self.pocket_width_mm for i, a in enumerate(positions) for b in positions[i+1:]):
                raise ValueError('Captive pockets overlap or leave no separating material')

    def validate_for(self, tube):
        if self.pocket_depth_mm-self.opening_overlap_mm >= tube.wall_mm:
            raise ValueError('Pocket breaches the outside wall')
        flat_roof_half_width = tube.inner_width_mm/2-tube.inner_radius_mm
        if any(abs(x)+self.pocket_width_mm/2 >= flat_roof_half_width for x in self.positions_x_mm):
            raise ValueError('Pocket footprint extends beyond the flat inner roof; rounded-corner pocket seating is unsupported')
        if any(abs(y-tube.center_y_mm)+self.pocket_width_mm/2 >= tube.length_mm/2 for y in self.positions_y_mm):
            raise ValueError('Pocket lies beyond the reinforced span')

    def cut(self, shape, tube):
        import cadquery as cq
        self.validate_for(tube)
        outer_z = tube.height_mm/2
        inner_z = outer_z-tube.wall_mm
        pocket_center = inner_z+self.pocket_depth_mm/2-self.opening_overlap_mm
        for y in self.positions_y_mm:
            for x in self.positions_x_mm:
                shape = shape.cut(cq.Solid.makeCylinder(self.hole_radius_mm, 2*outer_z+4,
                    cq.Vector(x, y, -outer_z-2), cq.Vector(0, 0, 1)))
                for sign in (-1, 1):
                    shape = shape.cut(centered_box(self.pocket_width_mm, self.pocket_width_mm,
                        self.pocket_depth_mm, (x, y, sign*pocket_center)))
        return _valid_single(shape.clean(), 'Pocketed span')


@dataclass(frozen=True)
class SleevedCrossBore:
    """Nominal purchased-hardware candidate fit; no hardware solid or rating.

    The bore follows the sleeve diameter and clearance, so a change of sleeve
    updates both mating components' drilled pattern through SplitLapSplice.
    Length equality describes flush nominal seating, not a tolerance stack.
    """

    sleeve_outer_diameter_mm: float
    sleeve_inner_diameter_mm: float
    bolt_diameter_mm: float
    radial_clearance_mm: float
    sleeve_length_mm: float

    def __post_init__(self):
        for name, value in asdict(self).items():
            _number(value, name, positive=True)
        if self.sleeve_inner_diameter_mm >= self.sleeve_outer_diameter_mm:
            raise ValueError('Sleeve candidate has no positive wall thickness')
        if self.bolt_diameter_mm >= self.sleeve_inner_diameter_mm:
            raise ValueError('Bolt candidate has no clearance inside sleeve')

    @property
    def hole_radius_mm(self):
        return self.sleeve_outer_diameter_mm/2+self.radial_clearance_mm

    def validate_for(self, beam):
        if self.sleeve_length_mm != beam.width_mm:
            raise ValueError('Sleeve candidate must match nominal flush span width; recessed seating is unsupported')

    def to_source(self):
        return dict(parameters=asdict(self), hole_radius_mm=self.hole_radius_mm,
                    bolt_radial_clearance_mm=(self.sleeve_inner_diameter_mm-self.bolt_diameter_mm)/2,
                    status='Hardware candidates specified only; not modeled, sourced, tolerance-qualified or load-rated')


@dataclass(frozen=True)
class SplitLapSplice:
    """A coupled split and sleeve interface, retaining one source of dimensions.

    ``beam`` defines the reinforced outer span. The coupler outside dimensions
    derive from its bore and per-side clearance. Cross-hole offsets are relative
    to the split center and must stay inside both overlap regions.
    """

    beam: RoundedBoxTube
    overlap_length_mm: float
    gap_mm: float
    radial_clearance_mm: float
    coupler_wall_mm: float
    hole_offsets_y_mm: tuple[float, ...]
    hole_radius_mm: float
    coupler_inner_radius_mm: float = 1.2
    coupler_outer_radius_mm: float = 1.4

    def __post_init__(self):
        if not isinstance(self.beam, RoundedBoxTube):
            raise ValueError('Splice requires a RoundedBoxTube beam specification')
        for name in ('overlap_length_mm', 'gap_mm', 'radial_clearance_mm', 'coupler_wall_mm', 'hole_radius_mm'):
            _number(getattr(self, name), name, positive=True)
        for name in ('coupler_inner_radius_mm', 'coupler_outer_radius_mm'):
            _number(getattr(self, name), name, nonnegative=True)
        offsets = _positions(self.hole_offsets_y_mm, 'Cross-hole offsets')
        object.__setattr__(self, 'hole_offsets_y_mm', offsets)
        if self.overlap_length_mm > self.beam.length_mm:
            raise ValueError('Coupler extends outside the reinforced span')
        if self.gap_mm >= self.overlap_length_mm:
            raise ValueError('Split gap leaves no lap overlap')
        if not any(y < 0 for y in offsets) or not any(y > 0 for y in offsets):
            raise ValueError('Both splice halves need cross-hole positions')
        if any(abs(y)-self.hole_radius_mm <= self.gap_mm/2 for y in offsets):
            raise ValueError('A cross hole breaches the split face')
        if any(abs(y)+self.hole_radius_mm >= self.overlap_length_mm/2 for y in offsets):
            raise ValueError('A cross hole breaches the coupler end')
        if any(abs(a-b) <= 2*self.hole_radius_mm for i, a in enumerate(offsets) for b in offsets[i+1:]):
            raise ValueError('Cross holes overlap')
        # Constructing this immutable spec validates the coupled hollow section
        # before any CAD operation, including the two wall/fillet constraints.
        coupler = self.coupler
        if self.hole_radius_mm >= coupler.inner_height_mm/2:
            raise ValueError('Cross hole does not remain in the sleeve side wall')

    @property
    def coupler(self):
        return RoundedBoxTube(self.beam.inner_width_mm-2*self.radial_clearance_mm,
            self.beam.inner_height_mm-2*self.radial_clearance_mm, self.overlap_length_mm,
            self.coupler_wall_mm, self.beam.center_y_mm, self.coupler_inner_radius_mm,
            self.coupler_outer_radius_mm)

    @property
    def hole_positions_y_mm(self):
        return tuple(self.beam.center_y_mm+offset for offset in self.hole_offsets_y_mm)

    def _cross_holes(self, shape, span):
        import cadquery as cq
        for y in self.hole_positions_y_mm:
            shape = shape.cut(cq.Solid.makeCylinder(self.hole_radius_mm, span,
                cq.Vector(-span/2, y, 0), cq.Vector(1, 0, 0)))
        return shape.clean()

    def native_shapes(self, reinforced_shape):
        """Cut the same cross-hole pattern through both halves and the coupler."""
        _valid_single(reinforced_shape, 'Reinforced source span')
        span = self._cross_holes(reinforced_shape, self.beam.width_mm+10)
        bounds = native_bounds(span)
        center = self.beam.center_y_mm
        lo, hi = center-self.gap_mm/2, center+self.gap_mm/2
        if bounds.ymin >= lo or bounds.ymax <= hi:
            raise ValueError('Source span does not extend to both sides of the split')
        # Bounding-box padding only defines an unbounded-side clipping tool;
        # it changes none of the retained source joint/end geometry.
        xlen, zlen = bounds.xlen+2, bounds.zlen+2
        cx, cz = (bounds.xmin+bounds.xmax)/2, (bounds.zmin+bounds.zmax)/2
        lower = span.intersect(centered_box(xlen, lo-bounds.ymin+1, zlen,
            (cx, (lo+bounds.ymin-1)/2, cz))).clean()
        upper = span.intersect(centered_box(xlen, bounds.ymax-hi+1, zlen,
            (cx, (bounds.ymax+hi+1)/2, cz))).clean()
        coupler = self._cross_holes(self.coupler.native_shape(), self.beam.width_mm)
        parts = dict(proximal=lower, distal=upper, coupler=coupler)
        for name, shape in parts.items():
            _valid_single(shape, 'Splice '+name)
        # Construction is not accepted without these measured nominal mates.
        report = self.interface_requirements(parts).report()
        if not report['passed']:
            raise ValueError('Constructed lap interface fails its declared nominal clearance')
        return parts

    def interface_requirements(self, parts, *, allowed_volume_mm3=.01, kernel_allowance_mm=1e-6):
        _number(kernel_allowance_mm, 'CAD distance allowance', nonnegative=True)
        requirements = NativeInterfaceRequirements(dict(source=SOURCE, specification=self.to_source()))
        for a, b, distance in (('proximal', 'distal', self.gap_mm),
                                ('proximal', 'coupler', self.radial_clearance_mm),
                                ('distal', 'coupler', self.radial_clearance_mm)):
            row = requirements.noninterference(a+'_vs_'+b, parts[a], parts[b],
                                              allowed_volume_mm3=allowed_volume_mm3)
            row.update(required_minimum_clearance_mm=distance,
                       distance_kernel_allowance_mm=kernel_allowance_mm,
                       clearance_passed=row['minimum_distance_mm']+kernel_allowance_mm >= distance)
            row['pass'] = row['pass'] and row['clearance_passed']
        return requirements

    def to_source(self):
        return dict(schema='cybrgeo-split-lap-splice/1', parameters=asdict(self),
                    derived_coupler=asdict(self.coupler), hole_positions_y_mm=list(self.hole_positions_y_mm),
                    overlap_per_side_mm=(self.overlap_length_mm-self.gap_mm)/2,
                    scope='Nominal analytic CAD interfaces; no hardware, printing, contact dynamics or load qualification')
