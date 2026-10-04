"""Parametric transverse involute spur/internal-ring gear geometry.

The involute flanks are analytic. Root transitions are smooth visualization/design
transitions rather than a generated hob/shaper cutter envelope, so these solids are
engineering concept CAD rather than manufacturing-certified gear geometry.
Coordinates follow Mechanism Lab convention: X is the shaft axis and YZ is the
transverse gear plane; dimensions are millimetres.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np
import cadquery as cq


@dataclass(frozen=True)
class GearRadii:
    pitch: float
    base: float
    root: float
    addendum: float


def gear_radii(teeth: int, module: float, pressure_angle_deg: float = 20.0, *, internal: bool = False) -> GearRadii:
    if teeth < 8 or module <= 0:
        raise ValueError('teeth must be >= 8 and module must be positive')
    if not 10.0 <= pressure_angle_deg <= 35.0:
        raise ValueError('pressure angle must be between 10 and 35 degrees')
    rp = teeth * module / 2.0
    rb = rp * math.cos(math.radians(pressure_angle_deg))
    if internal:
        # Internal tooth tip is inward of pitch; root is outward.
        root = rp + 1.25 * module
        addendum = rp - module
        if addendum <= rb:
            raise ValueError('internal addendum lies at/below base circle; use more teeth or profile shift')
    else:
        root = max(0.05, rp - 1.25 * module)
        addendum = rp + module
    return GearRadii(rp, rb, root, addendum)


def involute_function(base_radius: float, radius: float) -> float:
    """Return inv(alpha)=tan(alpha)-alpha at a radius on an involute."""
    if base_radius <= 0 or radius <= 0:
        raise ValueError('radii must be positive')
    if radius <= base_radius:
        return 0.0
    q = math.sqrt((radius / base_radius) ** 2 - 1.0)
    return q - math.atan(q)


def _polar_to_yz(profile: np.ndarray, phase: float = 0.0) -> np.ndarray:
    r = profile[:, 0]
    a = profile[:, 1] + phase
    return np.column_stack([r * np.cos(a), r * np.sin(a)])


def external_profile(
    teeth: int,
    module: float,
    pressure_angle_deg: float = 20.0,
    backlash: float = 0.10,
    flank_samples: int = 14,
    tip_samples: int = 6,
    root_samples: int = 5,
) -> np.ndarray:
    """Closed polar profile for an external involute spur gear.

    `backlash` is a circumferential tooth-thickness reduction at the pitch circle.
    The root connection is a compact radial/arc transition, not a trochoidal hob root.
    """
    if backlash < 0 or flank_samples < 4 or tip_samples < 3 or root_samples < 3:
        raise ValueError('invalid profile sampling/backlash')
    g = gear_radii(teeth, module, pressure_angle_deg)
    if backlash >= math.pi * module / 2:
        raise ValueError('backlash is too large for the tooth pitch')
    rp, rb, rr, ra = g.pitch, g.base, g.root, g.addendum
    ip = involute_function(rb, rp)
    half_pitch_tooth = math.pi / (2.0 * teeth) - backlash / (2.0 * rp)
    pts: list[tuple[float, float]] = []
    start_r = max(rb, rr + min(0.12 * module, 0.10))
    for j in range(teeth):
        c = 2.0 * math.pi * j / teeth
        root_half = half_pitch_tooth + ip
        # Left root transition into the involute flank.
        pts.append((rr, c - root_half - 0.008))
        for u in np.linspace(0.0, 1.0, root_samples)[1:]:
            r = rr + (start_r - rr) * u
            pts.append((r, c - root_half * (1.0 - 0.04 * u)))
        for r in np.linspace(start_r, ra, flank_samples):
            half = half_pitch_tooth + ip - involute_function(rb, r)
            pts.append((float(r), c - half))
        tip_half = half_pitch_tooth + ip - involute_function(rb, ra)
        for a in np.linspace(c - tip_half, c + tip_half, tip_samples)[1:]:
            pts.append((ra, float(a)))
        for r in np.linspace(ra, start_r, flank_samples)[1:]:
            half = half_pitch_tooth + ip - involute_function(rb, r)
            pts.append((float(r), c + half))
        for u in np.linspace(1.0, 0.0, root_samples)[1:]:
            r = rr + (start_r - rr) * u
            pts.append((r, c + root_half * (1.0 - 0.04 * u)))
        right = c + root_half + 0.008
        next_left = c + 2.0 * math.pi / teeth - root_half - 0.008
        pts.append((rr, right))
        for a in np.linspace(right, next_left, root_samples + 1)[1:-1]:
            pts.append((rr, float(a)))
    return np.asarray(pts, float)


def internal_space_profile(
    teeth: int,
    module: float,
    pressure_angle_deg: float = 20.0,
    backlash: float = 0.10,
    flank_samples: int = 14,
    tip_samples: int = 6,
    root_samples: int = 5,
) -> np.ndarray:
    """Closed polar profile of the *void spaces* of an internal ring gear.

    Cutting this profile from an annulus leaves internal involute teeth. Backlash
    widens the pitch-circle space rather than silently shrinking an unrelated part.
    As with the external profile, the root transition is not a cutter-envelope root.
    """
    if backlash < 0 or flank_samples < 4 or tip_samples < 3 or root_samples < 3:
        raise ValueError('invalid profile sampling/backlash')
    g = gear_radii(teeth, module, pressure_angle_deg, internal=True)
    rp, rb = g.pitch, g.base
    inner_tip = g.addendum
    outer_root = g.root
    ip = involute_function(rb, rp)
    # This is half the INTERNAL SPACE width at pitch, not tooth thickness.
    half_space = math.pi / (2.0 * teeth) + backlash / (2.0 * rp)
    pts: list[tuple[float, float]] = []
    start_r = max(rb, inner_tip)
    for j in range(teeth):
        c = 2.0 * math.pi * j / teeth
        inner_half = half_space + ip - involute_function(rb, start_r)
        pts.append((inner_tip, c - inner_half - 0.006))
        if start_r > inner_tip + 1e-9:
            for u in np.linspace(0.0, 1.0, root_samples)[1:]:
                r = inner_tip + (start_r - inner_tip) * u
                pts.append((r, c - inner_half))
        for r in np.linspace(start_r, outer_root, flank_samples):
            half = half_space + ip - involute_function(rb, r)
            pts.append((float(r), c - half))
        outer_half = half_space + ip - involute_function(rb, outer_root)
        for a in np.linspace(c - outer_half, c + outer_half, tip_samples)[1:]:
            pts.append((outer_root, float(a)))
        for r in np.linspace(outer_root, start_r, flank_samples)[1:]:
            half = half_space + ip - involute_function(rb, r)
            pts.append((float(r), c + half))
        if start_r > inner_tip + 1e-9:
            for u in np.linspace(1.0, 0.0, root_samples)[1:]:
                r = inner_tip + (start_r - inner_tip) * u
                pts.append((r, c + inner_half))
        right = c + inner_half + 0.006
        next_left = c + 2.0 * math.pi / teeth - inner_half - 0.006
        pts.append((inner_tip, right))
        for a in np.linspace(right, next_left, root_samples + 1)[1:-1]:
            pts.append((inner_tip, float(a)))
    return np.asarray(pts, float)


def external_spur_gear(
    teeth: int,
    module: float,
    width: float,
    *,
    bore: float = 0.0,
    origin: tuple[float, float, float] = (0.0, 0.0, 0.0),
    phase: float = 0.0,
    pressure_angle_deg: float = 20.0,
    backlash: float = 0.10,
):
    """Create a CadQuery external spur gear with X-axis face width."""
    if width <= 0 or bore < 0:
        raise ValueError('width must be positive and bore nonnegative')
    profile = external_profile(teeth, module, pressure_angle_deg, backlash)
    yz = _polar_to_yz(profile, phase)
    work = cq.Workplane('YZ', origin=origin).polyline(yz.tolist()).close().extrude(width)
    if bore:
        cutter_origin = (origin[0] - 0.5, origin[1], origin[2])
        work = work.cut(cq.Workplane('YZ', origin=cutter_origin).circle(bore / 2.0).extrude(width + 1.0))
    return work.val()


def internal_ring_gear(
    teeth: int,
    module: float,
    width: float,
    *,
    outer_radius: float,
    origin: tuple[float, float, float] = (0.0, 0.0, 0.0),
    phase: float = 0.0,
    pressure_angle_deg: float = 20.0,
    backlash: float = 0.10,
):
    """Create a CadQuery internal ring gear by cutting an involute space profile."""
    if width <= 0 or outer_radius <= 0:
        raise ValueError('width and outer_radius must be positive')
    g = gear_radii(teeth, module, pressure_angle_deg, internal=True)
    if outer_radius <= g.root + max(1.5 * module, 1.0):
        raise ValueError('outer radius leaves insufficient material outside internal tooth roots')
    blank = cq.Workplane('YZ', origin=origin).circle(outer_radius).extrude(width)
    profile = internal_space_profile(teeth, module, pressure_angle_deg, backlash)
    yz = _polar_to_yz(profile, phase)
    cutter_origin = (origin[0] - 0.5, origin[1], origin[2])
    void = cq.Workplane('YZ', origin=cutter_origin).polyline(yz.tolist()).close().extrude(width + 1.0)
    result = blank.cut(void)
    if not result.val().isValid():
        raise ValueError('OpenCascade produced an invalid internal ring gear')
    return result.val()
