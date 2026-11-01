"""Nominal, sleeved through-fastener construction in local millimetres.

One specification derives the sleeve, both washers, bolt and nut seats.
These are geometric thread envelopes, not selected/catalog-rated hardware.
Preload, material allowables, thread tolerances and physical tests remain
external qualification evidence. Based on the October 31 ARX repair recipe.
"""
from dataclasses import dataclass
import math


def cylinder(radius, z0, z1, center=(0, 0)):
    import cadquery as cq
    return cq.Solid.makeCylinder(radius, z1-z0, cq.Vector(*center, z0))


def ring(outer_radius, inner_radius, z0, z1, center=(0, 0)):
    return cylinder(outer_radius, z0, z1, center).cut(
        cylinder(inner_radius, z0-1, z1+1, center))


def hexagonal(across_flats, z0, z1, center=(0, 0)):
    import cadquery as cq
    radius = across_flats/math.sqrt(3)
    points = [cq.Vector(center[0]+radius*math.cos(k*math.pi/3+math.pi/6),
                        center[1]+radius*math.sin(k*math.pi/3+math.pi/6), z0)
              for k in range(6)]
    return cq.Solid.extrudeLinear(cq.Wire.makePolygon(points, close=True), [],
                                  cq.Vector(0, 0, z1-z0))


def socket_screw(diameter, length, seat, head_height=None, center=(0, 0), axis_sign=-1):
    h = diameter if head_height is None else head_height
    if axis_sign == 1:
        shape = cylinder(diameter/2, seat, seat+length, center).fuse(
            cylinder(.95*diameter, seat-h, seat, center))
        socket = hexagonal(.78*diameter, seat-h-.01, seat-h+.65*h, center)
    elif axis_sign == -1:
        shape = cylinder(diameter/2, seat-length, seat, center).fuse(
            cylinder(.95*diameter, seat, seat+h, center))
        socket = hexagonal(.78*diameter, seat+.35*h, seat+h+.01, center)
    else:
        raise ValueError('Screw direction must be +1 or -1')
    return shape.cut(socket).clean()


def hex_nut(diameter, across_flats, height, z0, center=(0, 0)):
    return hexagonal(across_flats, z0, z0+height, center).cut(
        cylinder(diameter/2, z0-1, z0+height+1, center)).clean()


@dataclass(frozen=True)
class SleevedFastenerStack:
    """Construct the complete five-member stack; partial stacks are not output.

    Sleeve endpoints are the two clamped seats. Washer and nut positions are
    derived, never supplied independently. Positive direction puts the head at
    the low endpoint; negative direction puts it at the high endpoint.
    """
    diameter_mm: float
    length_mm: float
    sleeve_outer_radius_mm: float
    bore_radius_mm: float
    low_mm: float
    high_mm: float
    washer_radius_mm: float
    washer_thickness_mm: float
    nut_across_flats_mm: float
    nut_height_mm: float
    direction: int = 1

    def __post_init__(self):
        for name, value in vars(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
            if name not in ('low_mm', 'high_mm', 'direction') and value <= 0:
                raise ValueError(f'{name} must be positive')
        if self.direction not in (-1, 1) or self.high_mm <= self.low_mm:
            raise ValueError('Invalid stack direction or sleeve span')
        if not self.diameter_mm/2 < self.bore_radius_mm < self.sleeve_outer_radius_mm < self.washer_radius_mm:
            raise ValueError('Bolt clearance, sleeve wall and washer seat must be positive')
        if self.nut_across_flats_mm <= 2*self.bore_radius_mm:
            raise ValueError('Nut cannot seat on washer bore')
        if self.engagement_mm < self.nut_height_mm-1e-10:
            raise ValueError('Bolt cannot engage the complete nominal nut thickness')

    @property
    def engagement_mm(self):
        return min(self.nut_height_mm, self.length_mm-(self.high_mm-self.low_mm)-2*self.washer_thickness_mm)

    def native_shapes(self, center=(0, 0)):
        d, lo, hi, t = self.diameter_mm, self.low_mm, self.high_mm, self.washer_thickness_mm
        seat = lo-t if self.direction == 1 else hi+t
        nut_z = hi+t if self.direction == 1 else lo-t-self.nut_height_mm
        return {
            'sleeve': ring(self.sleeve_outer_radius_mm, self.bore_radius_mm, lo, hi, center),
            'lower_washer': ring(self.washer_radius_mm, self.bore_radius_mm, lo-t, lo, center),
            'upper_washer': ring(self.washer_radius_mm, self.bore_radius_mm, hi, hi+t, center),
            'bolt': socket_screw(d, self.length_mm, seat, d, center, self.direction),
            'nut': hex_nut(d, self.nut_across_flats_mm, self.nut_height_mm, nut_z, center),
        }
