"""Exact simple-planetary geometry and rigid-body kinematics.

The supported topology is a conventional simple planetary set with external sun,
external planets, an internal ring and equally spaced planets. Kinematics use the
Willis relation and exact external/internal mesh phase constraints; no visual
rotation ratios are hand-tuned.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np


TAU = 2.0 * math.pi


def _wrap_pi(value: float) -> float:
    return (value + math.pi) % TAU - math.pi


@dataclass(frozen=True)
class PlanetarySpec:
    sun_teeth: int = 18
    planet_teeth: int = 27
    ring_teeth: int = 72
    planets: int = 3
    module: float = 0.8
    pressure_angle_deg: float = 20.0
    backlash_mm: float = 0.10
    face_width_mm: float = 10.0
    input_speed_rps: float = 0.45

    def validate(self) -> None:
        if min(self.sun_teeth, self.planet_teeth, self.ring_teeth) < 8:
            raise ValueError('all gear tooth counts must be >= 8')
        if self.ring_teeth != self.sun_teeth + 2 * self.planet_teeth:
            raise ValueError('simple planetary requires ring_teeth = sun_teeth + 2*planet_teeth')
        if self.planets < 2:
            raise ValueError('at least two planets required')
        # Equal-spacing assembly condition. If not integral, nominally equal planet
        # angles do not all share the same tooth phase relationship.
        if (self.sun_teeth + self.ring_teeth) % self.planets:
            raise ValueError('(sun_teeth + ring_teeth) must be divisible by planet count for equal spacing')
        if self.module <= 0 or self.face_width_mm <= 0:
            raise ValueError('module and face width must be positive')
        if not 10.0 <= self.pressure_angle_deg <= 35.0:
            raise ValueError('pressure angle out of supported range')
        if self.backlash_mm < 0 or self.backlash_mm >= math.pi * self.module / 2:
            raise ValueError('invalid circumferential backlash')
        if self.input_speed_rps <= 0:
            raise ValueError('input speed must be positive')
        # Standard unshifted 20 degree spur gears undercut below roughly 17 teeth.
        # Keep the concept away from that region instead of silently pretending a
        # profile-shift/cutter solution exists.
        if self.pressure_angle_deg <= 20.5 and self.sun_teeth < 17:
            raise ValueError('sun tooth count enters standard unshifted undercut region')

    @property
    def sun_pitch_radius(self) -> float:
        return self.sun_teeth * self.module / 2.0

    @property
    def planet_pitch_radius(self) -> float:
        return self.planet_teeth * self.module / 2.0

    @property
    def ring_pitch_radius(self) -> float:
        return self.ring_teeth * self.module / 2.0

    @property
    def planet_center_radius(self) -> float:
        return self.sun_pitch_radius + self.planet_pitch_radius

    @property
    def reduction(self) -> float:
        """Sun input / fixed ring / carrier output speed reduction."""
        return 1.0 + self.ring_teeth / self.sun_teeth

    @property
    def carrier_speed_rps(self) -> float:
        return self.input_speed_rps / self.reduction

    @property
    def planet_absolute_speed_rps(self) -> float:
        # omega_p = omega_c - Ns/Np * (omega_s - omega_c)
        return self.carrier_speed_rps - (self.sun_teeth / self.planet_teeth) * (
            self.input_speed_rps - self.carrier_speed_rps
        )


def planet_base_angles(spec: PlanetarySpec) -> tuple[float, ...]:
    spec.validate()
    return tuple(TAU * i / spec.planets for i in range(spec.planets))


def planet_centers(spec: PlanetarySpec, carrier_angle: float = 0.0) -> tuple[tuple[float, float], ...]:
    """Return planet-center YZ coordinates at a carrier angle."""
    r = spec.planet_center_radius
    return tuple(
        (r * math.cos(alpha + carrier_angle), r * math.sin(alpha + carrier_angle))
        for alpha in planet_base_angles(spec)
    )


def planet_initial_phases(spec: PlanetarySpec) -> tuple[float, ...]:
    """Planet absolute tooth phases that satisfy sun external mesh at t=0.

    The added pi is a tooth-to-space half-pitch condition. The equal-spacing
    assembly criterion guarantees the same phases also satisfy the internal ring
    mesh modulo whole tooth pitches.
    """
    spec.validate()
    return tuple(
        (math.pi + (spec.sun_teeth + spec.planet_teeth) * alpha) / spec.planet_teeth
        for alpha in planet_base_angles(spec)
    )


def planetary_angles(spec: PlanetarySpec, time_seconds: float) -> dict:
    """Return exact absolute sun/carrier/planet angles for a fixed ring."""
    spec.validate()
    theta_s = TAU * spec.input_speed_rps * float(time_seconds)
    theta_c = theta_s / spec.reduction
    phases = planet_initial_phases(spec)
    base = planet_base_angles(spec)
    planet_angles = []
    center_angles = []
    for alpha0, phase0 in zip(base, phases):
        alpha = alpha0 + theta_c
        # Exact external-mesh velocity relation in the carrier frame.
        theta_p = phase0 + theta_c - (spec.sun_teeth / spec.planet_teeth) * (theta_s - theta_c)
        center_angles.append(alpha)
        planet_angles.append(theta_p)
    return {
        'sun': theta_s,
        'carrier': theta_c,
        'ring': 0.0,
        'planet_centers': tuple(center_angles),
        'planets': tuple(planet_angles),
    }


def mesh_residuals(spec: PlanetarySpec, time_seconds: float) -> dict:
    """Return wrapped angular mesh-constraint residuals in radians.

    Both arrays should remain near zero for all time. This directly checks the
    external sun/planet and internal ring/planet phase equations used by motion.
    """
    q = planetary_angles(spec, time_seconds)
    ext = []
    internal = []
    for alpha, theta_p in zip(q['planet_centers'], q['planets']):
        e = (
            spec.sun_teeth * q['sun']
            + spec.planet_teeth * theta_p
            - (spec.sun_teeth + spec.planet_teeth) * alpha
            - math.pi
        )
        r = (
            spec.ring_teeth * q['ring']
            - spec.planet_teeth * theta_p
            - (spec.ring_teeth - spec.planet_teeth) * alpha
            + math.pi
        )
        ext.append(_wrap_pi(e))
        internal.append(_wrap_pi(r))
    return {'sun_planet': tuple(ext), 'ring_planet': tuple(internal)}


def rotation_x4(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    T = np.eye(4)
    T[:3, :3] = np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)
    return T


def orbit_and_spin(center_yz: tuple[float, float], carrier_angle: float, relative_spin: float) -> np.ndarray:
    """Transform bind geometry at an initial planet center by orbit + relative spin."""
    y, z = center_yz
    to_center = np.eye(4); to_center[:3, 3] = [0.0, y, z]
    from_center = np.eye(4); from_center[:3, 3] = [0.0, -y, -z]
    return rotation_x4(carrier_angle) @ to_center @ rotation_x4(relative_spin) @ from_center
