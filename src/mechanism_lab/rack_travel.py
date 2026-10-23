"""Construct a rack position that retains nominal tooth span through travel.

This is a geometric pitch-point support bound. It does not establish rolling
contact, gear strength, backlash, friction, or a manufacturable tooth form.
"""
from __future__ import annotations

from dataclasses import dataclass
import math


def _finite(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{label} must be a finite number')
    try:
        result = float(value)
    except OverflowError as error:
        raise ValueError(f'{label} must be representable') from error
    if not math.isfinite(result):
        raise ValueError(f'{label} must be a finite number')
    return result


def _positive(value, label):
    result = _finite(value, label)
    if result <= 0:
        raise ValueError(f'{label} must be a positive finite number')
    return result


@dataclass(frozen=True)
class RackPinionTravel:
    """One symmetric opposed rack pair about a fixed pinion pitch point.

    The rack blank spans [-L/2, L/2] before placement. Its sampled tooth
    polygon has first and last pitch-line flanks inset by pitch/4, so only
    [-L/2+pitch/4, L/2-pitch/4] is credited for engagement. The right
    jaw translates +q and the mirrored left jaw -q, both for q in the given
    physical slide interval. ``toothed_end_local_x_mm`` is the right rack's
    nominal blank/tooth-array end after placement at q=0; the usable pitch
    flank ends pitch/4 earlier. The left rack mirrors it.
    The constructor rejects a span with insufficient authored pitch-point
    edge margin anywhere in the closed full travel interval.
    """

    module_mm: float
    pinion_teeth: int
    rack_teeth: int
    toothed_end_local_x_mm: float
    slide_min_mm: float
    slide_max_mm: float
    required_edge_pitch_count: float
    design_reserve_pitch_count: float

    def __post_init__(self):
        for name in ('module_mm', 'slide_min_mm', 'slide_max_mm',
                     'required_edge_pitch_count', 'design_reserve_pitch_count'):
            _positive(getattr(self, name), name)
        for name in ('pinion_teeth', 'rack_teeth'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 3:
                raise ValueError(f'{name} must be an integer >= 3')
        _finite(self.toothed_end_local_x_mm, 'toothed_end_local_x_mm')
        if self.slide_min_mm >= self.slide_max_mm:
            raise ValueError('Rack slide limits must be increasing')
        try:
            derived = (self.pitch_mm, self.pitch_radius_m, self.toothed_length_mm,
                       self.right_rack_x_offset_mm, self.required_edge_margin_mm)
        except OverflowError as error:
            raise ValueError('Derived rack geometry must be representable') from error
        if not all(math.isfinite(value) for value in derived):
            raise ValueError('Derived rack geometry must be finite')
        if self.minimum_edge_margin_mm < self.required_edge_margin_mm:
            raise ValueError('Rack tooth span cannot retain the required pitch-point margin through full slide travel')

    @classmethod
    def from_document(cls, document, slide_range_m):
        fields = {'module_mm', 'pinion_teeth', 'rack_teeth',
                  'toothed_end_local_x_mm', 'required_edge_pitch_count',
                  'design_reserve_pitch_count'}
        if not isinstance(document, dict) or set(document) != fields:
            raise ValueError('Rack travel needs one complete source geometry and margin document')
        if not isinstance(slide_range_m, (tuple, list)) or len(slide_range_m) != 2:
            raise ValueError('Physical rack slide interval must have finite endpoints')
        lower = _finite(slide_range_m[0], 'Physical slide minimum')
        upper = _finite(slide_range_m[1], 'Physical slide maximum')
        return cls(slide_min_mm=lower*1000., slide_max_mm=upper*1000., **document)

    @property
    def pitch_mm(self):
        return math.pi * self.module_mm

    @property
    def pitch_radius_m(self):
        return self.pitch_radius_mm / 1000.

    @property
    def pitch_radius_mm(self):
        return self.module_mm * self.pinion_teeth / 2.

    @property
    def toothed_length_mm(self):
        return self.rack_teeth * self.pitch_mm

    @property
    def pitch_line_flank_inset_mm(self):
        # P5 rack(): the first centre is -L/2+p/2 and each pitch-line
        # flank is p/4 from its centre. Root/tip pressure-angle offsets cancel
        # at the pitch line, so the usable flank starts at -L/2+p/4.
        return self.pitch_mm/4.

    @property
    def right_rack_x_offset_mm(self):
        return self.toothed_end_local_x_mm - self.toothed_length_mm/2.

    @property
    def left_rack_x_offset_mm(self):
        return -self.right_rack_x_offset_mm

    def opposed_pitch_line_y(self, pinion_center_y_mm, jaw_origin_y_mm):
        """Local right and pre-offset left CAD pitch-line Y for opposed jaws."""
        center = _finite(pinion_center_y_mm, 'Pinion center Y')
        origin = _finite(jaw_origin_y_mm, 'Jaw body origin Y')
        right = center + self.pitch_radius_mm - origin
        left = center - self.pitch_radius_mm
        if not math.isfinite(right) or not math.isfinite(left):
            raise ValueError('Derived rack pitch-line placement must be finite')
        return dict(right_local_y_mm=right, left_preoffset_y_mm=left,
                    pinion_center_y_mm=center, jaw_origin_y_mm=origin,
                    pitch_radius_mm=self.pitch_radius_mm)

    @property
    def minimum_edge_margin_mm(self):
        inset = self.pitch_line_flank_inset_mm
        return min(self.slide_min_mm + self.toothed_end_local_x_mm - inset,
                   self.toothed_length_mm - self.slide_max_mm
                   - self.toothed_end_local_x_mm - inset)

    @property
    def required_edge_margin_mm(self):
        return (self.required_edge_pitch_count + self.design_reserve_pitch_count) * self.pitch_mm

    def report(self):
        return dict(schema='cybrgeo-rack-pinion-travel/1',
                    module_mm=self.module_mm, pinion_teeth=self.pinion_teeth,
                    rack_teeth=self.rack_teeth, pitch_mm=self.pitch_mm,
                    pitch_radius_mm=self.pitch_radius_mm,
                    pitch_radius_m=self.pitch_radius_m,
                    toothed_length_mm=self.toothed_length_mm,
                    pitch_line_flank_inset_mm=self.pitch_line_flank_inset_mm,
                    slide_range_mm=[self.slide_min_mm, self.slide_max_mm],
                    toothed_end_local_x_mm=self.toothed_end_local_x_mm,
                    right_rack_x_offset_mm=self.right_rack_x_offset_mm,
                    left_rack_x_offset_mm=self.left_rack_x_offset_mm,
                    required_edge_pitch_count=self.required_edge_pitch_count,
                    design_reserve_pitch_count=self.design_reserve_pitch_count,
                    minimum_edge_margin_mm=self.minimum_edge_margin_mm,
                    required_edge_margin_mm=self.required_edge_margin_mm,
                    geometry_margin_mm=self.minimum_edge_margin_mm - self.required_edge_margin_mm,
                    full_nominal_travel_supported=True,
                    scope='Nominal straight rack tooth-span containment at fixed pinion pitch point over the complete opposed-slide interval; no tooth contact or load capacity')
