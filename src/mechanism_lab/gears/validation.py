"""Numerical validation helpers for simple planetary gear concepts."""
from __future__ import annotations

import math
from .planetary import PlanetarySpec, mesh_residuals, planet_centers


def planetary_validation_report(spec: PlanetarySpec, sample_times=(0.0, 0.17, 0.73, 1.31, 2.4)) -> dict:
    spec.validate()
    center = spec.planet_center_radius
    sun_tangent = spec.sun_pitch_radius + spec.planet_pitch_radius
    ring_tangent = spec.ring_pitch_radius - spec.planet_pitch_radius
    center_errors = [abs(math.hypot(y, z) - center) for y, z in planet_centers(spec)]
    ext_res = []
    int_res = []
    for t in sample_times:
        r = mesh_residuals(spec, t)
        ext_res.extend(abs(v) for v in r['sun_planet'])
        int_res.extend(abs(v) for v in r['ring_planet'])
    return {
        'topology': {
            'sun_teeth': spec.sun_teeth,
            'planet_teeth': spec.planet_teeth,
            'ring_teeth': spec.ring_teeth,
            'planet_count': spec.planets,
            'ring_relation_exact': spec.ring_teeth == spec.sun_teeth + 2 * spec.planet_teeth,
            'equal_spacing_phase_condition': (spec.sun_teeth + spec.ring_teeth) % spec.planets == 0,
        },
        'pitch_geometry_mm': {
            'sun_radius': spec.sun_pitch_radius,
            'planet_radius': spec.planet_pitch_radius,
            'ring_radius': spec.ring_pitch_radius,
            'planet_center_radius': center,
            'sun_planet_tangency_error': abs(center - sun_tangent),
            'ring_planet_tangency_error': abs(center - ring_tangent),
            'max_center_radius_error': max(center_errors, default=0.0),
        },
        'kinematics': {
            'fixed_ring_reduction': spec.reduction,
            'input_speed_rps': spec.input_speed_rps,
            'carrier_speed_rps': spec.carrier_speed_rps,
            'planet_absolute_speed_rps': spec.planet_absolute_speed_rps,
            'max_external_mesh_phase_residual_rad': max(ext_res, default=0.0),
            'max_internal_mesh_phase_residual_rad': max(int_res, default=0.0),
        },
        'claims': {
            'flanks': 'analytic transverse involute profiles',
            'roots': 'concept transitions; not hob/shaper cutter-envelope certification',
            'backlash': 'explicit circumferential pitch-circle design allowance',
            'qualification': 'no loaded contact, stress, fatigue, lubrication, thermal or manufacturing qualification',
        },
    }
