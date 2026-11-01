"""Analytic planar four-bar closure and conservative angular motion bounds.

This is a rigid kinematic construction tool. It neither applies joint forces
nor establishes strength, fit, continuous CAD clearance, or hardware quality.
The bound applies to one continuous circle-intersection branch over the exact
declared input interval, including its endpoints; change-point intervals fail.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math


def _finite(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{label} must be finite')
    return float(value)


def _positive(value, label):
    value = _finite(value, label)
    if value <= 0:
        raise ValueError(f'{label} must be positive')
    return value


def _point(value, label):
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise ValueError(f'{label} must be a planar millimetre pair')
    return tuple(_finite(component, label) for component in value)


def _cos_extrema(low, high):
    """Enclose cos over a closed interval, including every stationary point."""
    if high - low >= math.tau:
        return -1., 1.
    values = [math.cos(low), math.cos(high)]
    first = math.ceil(low / math.pi)
    last = math.floor(high / math.pi)
    for integer in range(first, last + 1):
        values.append(1. if integer % 2 == 0 else -1.)
    # Trigonometric and interval arithmetic is floating-point; enlarge the
    # enclosure rather than accidentally claiming a tighter speed bound.
    return max(-1., min(values) - 1e-14), min(1., max(values) + 1e-14)


@dataclass(frozen=True)
class PlanarFourBar:
    """Two fixed pivots, a rotating crank, one coupler, and a passive rocker.

    ``input_angle_rad`` is relative to the optional phase, while the phase and
    both pivot coordinates use one common fixed XY frame. Branch +1 means the
    coupler/rocker pin lies left of the ray from crank pin to output pivot.
    """

    input_pivot_mm: tuple[float, float]
    output_pivot_mm: tuple[float, float]
    crank_length_mm: float
    coupler_length_mm: float
    rocker_length_mm: float
    input_phase_rad: float = 0.
    branch: int = 1

    def __post_init__(self):
        for name in ('input_pivot_mm', 'output_pivot_mm'):
            object.__setattr__(self, name, _point(getattr(self, name), name))
        for name in ('crank_length_mm', 'coupler_length_mm', 'rocker_length_mm'):
            object.__setattr__(self, name, _positive(getattr(self, name), name))
        object.__setattr__(self, 'input_phase_rad', _finite(self.input_phase_rad, 'Input phase'))
        if isinstance(self.branch, bool) or self.branch not in (-1, 1):
            raise ValueError('Circle-intersection branch must be +1 or -1')
        if self.ground_length_mm <= 0:
            raise ValueError('Fixed planar pivots must be distinct')

    @property
    def ground_length_mm(self):
        x0, y0 = self.input_pivot_mm
        x1, y1 = self.output_pivot_mm
        return math.hypot(x1 - x0, y1 - y0)

    @property
    def source_sha256(self):
        return hashlib.sha256(json.dumps(self.source(), sort_keys=True,
                                         separators=(',', ':'), allow_nan=False).encode()).hexdigest()

    def source(self):
        return dict(schema='cybrgeo-planar-fourbar/1', input_pivot_mm=self.input_pivot_mm,
                    output_pivot_mm=self.output_pivot_mm,
                    crank_length_mm=self.crank_length_mm,
                    coupler_length_mm=self.coupler_length_mm,
                    rocker_length_mm=self.rocker_length_mm,
                    input_phase_rad=self.input_phase_rad, branch=self.branch,
                    units='mm, rad; absolute XY fixed frame')

    def solve(self, input_angle_rad):
        """Return nominal pin positions and principal follower angles.

        Principal angles are wrapped by atan2; a consumer tracking an entire
        revolution should unwrap them while retaining the branch identity.
        """
        theta = _finite(input_angle_rad, 'Input angle') + self.input_phase_rad
        ox, oy = self.input_pivot_mm
        px, py = self.output_pivot_mm
        a, b, c = self.crank_length_mm, self.coupler_length_mm, self.rocker_length_mm
        ax, ay = ox + a * math.cos(theta), oy + a * math.sin(theta)
        dx, dy = px - ax, py - ay
        distance = math.hypot(dx, dy)
        if distance <= abs(b - c) or distance >= b + c:
            raise ValueError('Four-bar is unreachable or at a change point at this input angle')
        along = (b*b - c*c + distance*distance) / (2 * distance)
        height_squared = b*b - along*along
        if height_squared <= 0:
            raise ValueError('Four-bar circle intersection is degenerate')
        height = math.sqrt(height_squared)
        ux, uy = dx / distance, dy / distance
        bx = ax + along*ux - self.branch*height*uy
        by = ay + along*uy + self.branch*height*ux
        return dict(input_angle_rad=float(input_angle_rad),
                    crank_angle_rad=theta,
                    input_pivot_mm=self.input_pivot_mm,
                    output_pivot_mm=self.output_pivot_mm,
                    crank_pin_mm=(ax, ay), rocker_pin_mm=(bx, by),
                    coupler_angle_rad=math.atan2(by - ay, bx - ax),
                    rocker_angle_rad=math.atan2(by - py, bx - px),
                    branch=self.branch, source_sha256=self.source_sha256)

    def angular_rate_bounds(self, input_start_rad, input_end_rad):
        """Bound follower angular travel per input radian on a full interval.

        If the crank pin's distance d from the output pivot stays strictly
        between |b-c| and b+c, the same branch exists continuously. The loop
        Jacobian determinant has magnitude b*c*|sin(gamma)|, where gamma is
        the coupler-rocker angle. Distance extrema follow directly from the
        fixed-pivot separation and extrema of cos(theta-relative). Thus
        |d rocker/d input| <= a/(c*sin_min) and
        |d coupler/d input| <= a/(b*sin_min) everywhere on the interval.
        """
        start = _finite(input_start_rad, 'Input interval start')
        end = _finite(input_end_rad, 'Input interval end')
        if end < start:
            raise ValueError('Input interval end must not precede its start')
        ox, oy = self.input_pivot_mm
        px, py = self.output_pivot_mm
        a, b, c, g = (self.crank_length_mm, self.coupler_length_mm,
                      self.rocker_length_mm, self.ground_length_mm)
        relative = math.atan2(py - oy, px - ox)
        cos_low, cos_high = _cos_extrema(start + self.input_phase_rad - relative,
                                         end + self.input_phase_rad - relative)
        # d^2 = g^2+a^2-2ga cos(theta-relative).
        distance_min = math.sqrt(max(0., g*g+a*a-2*g*a*cos_high))
        distance_max = math.sqrt(max(0., g*g+a*a-2*g*a*cos_low))
        inner, outer = abs(b-c), b+c
        scale = max(a, b, c, g, 1.)
        geometry_guard = 1e-12 * scale
        if distance_min <= inner + geometry_guard or distance_max >= outer - geometry_guard:
            raise ValueError('Full input interval reaches an unreachable or change-point configuration')
        cos_at_min = (b*b+c*c-distance_min*distance_min)/(2*b*c)
        cos_at_max = (b*b+c*c-distance_max*distance_max)/(2*b*c)
        cos_magnitude_upper = max(abs(cos_at_min), abs(cos_at_max)) + 1e-12
        if cos_magnitude_upper >= 1:
            raise ValueError('Four-bar Jacobian cannot be bounded away from a change point')
        sin_lower = math.sqrt(1 - cos_magnitude_upper*cos_magnitude_upper)
        # Widen the bound for floating arithmetic; not a manufacturing margin.
        sin_lower *= 1 - 1e-12
        return dict(schema='cybrgeo-planar-fourbar-motion-bound/1',
                    source_sha256=self.source_sha256,
                    input_interval_rad=[start, end], branch=self.branch,
                    output_pivot_distance_interval_mm=[distance_min, distance_max],
                    minimum_coupler_rocker_sine_lower_bound=sin_lower,
                    maximum_rocker_angular_rate_rad_per_input_rad=a/(c*sin_lower),
                    maximum_coupler_angular_rate_rad_per_input_rad=a/(b*sin_lower),
                    scope='Continuous rigid planar circle-intersection branch over the entire declared input interval; no dynamics, CAD clearance, process tolerance or hardware qualification')

    def point_motion_bound(self, body, maximum_point_radius_mm,
                           input_start_rad, input_end_rad):
        """Bound a rigid body's point travel per input radian over an interval.

        The radius encloses every queried point measured from the crank's
        fixed input pivot, the coupler's *moving crank pin*, or the rocker's
        fixed output pivot. The coupler adds the crank pin's translation.
        This is only a kinematic Lipschitz bound; a pose checker must still
        test actual CAD geometry, tolerances, and possible intersections.
        """
        if body not in ('crank', 'coupler', 'rocker'):
            raise ValueError('Body must be crank, coupler, or rocker')
        radius = _finite(maximum_point_radius_mm, 'Maximum point radius')
        if radius < 0:
            raise ValueError('Maximum point radius must be nonnegative')
        angular = self.angular_rate_bounds(input_start_rad, input_end_rad)
        if body == 'crank':
            speed = radius
            pivot = 'input_pivot_mm'
        elif body == 'coupler':
            speed = self.crank_length_mm + radius * angular[
                'maximum_coupler_angular_rate_rad_per_input_rad']
            pivot = 'crank_pin_mm'
        else:
            speed = radius * angular[
                'maximum_rocker_angular_rate_rad_per_input_rad']
            pivot = 'output_pivot_mm'
        if not math.isfinite(speed):
            raise ValueError('Point-motion bound overflows the supported numerical range')
        return dict(schema='cybrgeo-planar-fourbar-point-motion-bound/1',
                    source_sha256=self.source_sha256,
                    input_interval_rad=angular['input_interval_rad'],
                    body=body, attachment_pivot=pivot,
                    maximum_point_radius_mm=radius,
                    maximum_point_speed_mm_per_input_rad=speed,
                    scope='Nominal rigid planar point displacement Lipschitz bound only; no CAD clearance or hardware qualification')
