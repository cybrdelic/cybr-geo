"""ARX belt example's source-derived reference task; SI units, not dynamics.

Port of ``reference`` and ``referenceDerivatives`` in the pinned source's
``ARX-6-P5-Engineering/integration/controller.mjs`` at commit
``aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2``. Supply its parameters.json mapping.

The reference is a target for a force/torque controller. It does not prescribe
the actual body state, attach a workpiece, or prove contact/placement success.
The original finite-difference stencil and angle branch choices are retained,
including across stage boundaries. The optional lateral-tilt coefficient
reduces only the authored transfer posture; its default preserves the source.
This example-specific task is not a core trajectory policy.
"""
from __future__ import annotations

import math
from collections.abc import Mapping

SOURCE_COMMIT = "aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2"
PERIOD_S = 10.0
DERIVATIVE_STEP_S = 0.00025
HALF_CYCLE_BOUNDARIES_S = (0.0, 0.40, 0.65, 1.0, 1.65, 2.75, 3.4, 3.75, 4.45, 5.0)


def _clip(value, low, high):
    return max(low, min(high, value))


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _scale(a, value):
    return [x * value for x in a]


def _sub(a, b):
    return [x - b[index] for index, x in enumerate(a)]


def _mix(a, b, value):
    return [x + (b[index] - x) * value for index, x in enumerate(a)]


def _ease(value):
    value = _clip(value, 0, 1)
    return value * value * value * (value * (value * 6 - 15) + 10)


def _phase(time, start, end):
    return _ease((time - start) / (end - start))


def _ry(angle):
    c, s = math.cos(angle), math.sin(angle)
    return [c, 0, s, 0, 1, 0, -s, 0, c]


def _rz(angle):
    c, s = math.cos(angle), math.sin(angle)
    return [c, -s, 0, s, c, 0, 0, 0, 1]


def _mm(a, b):
    result = [0.0] * 9
    for row in range(3):
        for col in range(3):
            for index in range(3):
                result[3 * row + col] += a[3 * row + index] * b[3 * index + col]
    return result


def _transpose_mul(a, vector):
    return [a[0] * vector[0] + a[3] * vector[1] + a[6] * vector[2],
            a[1] * vector[0] + a[4] * vector[1] + a[7] * vector[2],
            a[2] * vector[0] + a[5] * vector[1] + a[8] * vector[2]]


def _position(time, parameters, lateral_tilt_scale=1.0):
    t = math.fmod(time, PERIOD_S)
    if t < 0:
        t += PERIOD_S
    returning = t >= 5
    h, direction_sign = (t - 5, -1) if returning else (t, 1)
    scale = parameters['scene_units_to_meters']
    a, b = parameters['workpiece_targets_m']['A'], parameters['workpiece_targets_m']['B']
    start, end = (b, a) if returning else (a, b)
    high_start, high_end = [start[0], .92 * scale, start[2]], [end[0], .92 * scale, end[2]]
    travel = _phase(h, 1.65, 2.75)
    hump = 16 * travel * travel * (1 - travel) * (1 - travel)
    if h < .65:
        tcp, stage = _mix(high_start, start, _phase(h, 0, .65)), 0
    elif h < 1:
        tcp, stage = list(start), 1
    elif h < 1.65:
        tcp, stage = _mix(start, high_start, _phase(h, 1, 1.65)), 2
    elif h < 2.75:
        tcp, stage = _mix(high_start, high_end, travel), 3
        tcp[1] += .16 * scale * hump
    elif h < 3.4:
        tcp, stage = _mix(high_end, end, _phase(h, 2.75, 3.4)), 4
    elif h < 3.75:
        tcp, stage = list(end), 5
    elif h < 4.45:
        tcp, stage = _mix(end, high_end, _phase(h, 3.75, 4.45)), 6
    else:
        tcp, stage = list(high_end), 7
    cfg = parameters['controller']
    gap = cfg['jaw_closed_target_half_gap_m'] if .40 <= h < 3.4 else cfg['jaw_open_half_gap_m']
    direction = [.15 * hump, -1, direction_sign * .34 * hump * lateral_tilt_scale]
    direction = _scale(direction, 1 / math.hypot(*direction))
    wrist = _sub(tcp, _scale(direction, parameters['tool_length_m']))
    delta = _sub(wrist, [0, parameters['shoulder_height_m'], 0])
    radial = math.hypot(delta[0], delta[2])
    q1 = math.atan2(delta[2], delta[0])
    base = _ry(-q1)
    l1, l2 = parameters['upper_length_m'], parameters['fore_length_m']
    q3 = math.acos(_clip((_dot(delta, delta) - l1 * l1 - l2 * l2) / (2 * l1 * l2), -1, 1))
    elevation = math.atan2(delta[1], radial) + math.atan2(l2 * math.sin(q3), l1 + l2 * math.cos(q3))
    q2 = math.pi / 2 - elevation
    fore = _mm(_mm(base, _rz(-q2)), _rz(-q3))
    local = _transpose_mul(fore, direction)
    q4 = math.atan2(-local[2], local[0])
    q5 = math.acos(_clip(local[1], -1, 1))
    bend = _mm(_mm(fore, _ry(q4)), _rz(-q5))
    reference_x = _sub([1, 0, 0], _scale(direction, direction[0]))
    reference_x = _scale(reference_x, 1 / math.hypot(*reference_x))
    reference_local = _transpose_mul(bend, reference_x)
    q6 = math.atan2(-reference_local[2], reference_local[0])
    if q6 < 0:
        q6 += 2 * math.pi
    q6 += direction_sign * .5 * hump
    return dict(q=[q1, q2, q3, q4, q5, q6], gap=gap, stage=stage)


def reference(time: float, parameters: Mapping, *, lateral_tilt_scale: float = 1.0) -> dict:
    """Return q [rad], qd [rad/s], qdd [rad/s²], half-gap [m], and stage.

    Accepts arbitrary finite times, including negative times and later cycles.
    Derivatives use the original centred 0.00025-second stencil without angle
    unwrapping or smoothing; gap remains the source's switched half-gap target.
    ``lateral_tilt_scale=1`` preserves the pinned source posture. Values in
    [0, 1] reduce the sideways transfer tilt while retaining the forward tilt,
    source roll, TCP targets and IK equations. This changes the desired joint
    path, not the physical model. Parameters are read only; no body state is
    imposed. Contact, braking and task success require separate simulation.
    """
    if isinstance(time, bool) or not isinstance(time, (int, float)) or not math.isfinite(time):
        raise ValueError('Reference time must be a finite number of seconds')
    if not isinstance(parameters, Mapping):
        raise TypeError('Reference parameters must be the source parameter mapping')
    if (isinstance(lateral_tilt_scale, bool) or not isinstance(lateral_tilt_scale, (int, float))
            or not math.isfinite(lateral_tilt_scale) or not 0 <= lateral_tilt_scale <= 1):
        raise ValueError('lateral_tilt_scale must be finite and within [0, 1]')
    result = _position(time, parameters, lateral_tilt_scale)
    before = _position(time - DERIVATIVE_STEP_S, parameters, lateral_tilt_scale)['q']
    after = _position(time + DERIVATIVE_STEP_S, parameters, lateral_tilt_scale)['q']
    result['qd'] = [(b - a) / (2 * DERIVATIVE_STEP_S) for a, b in zip(before, after)]
    result['qdd'] = [(b - 2 * q + a) / (DERIVATIVE_STEP_S * DERIVATIVE_STEP_S)
                     for q, a, b in zip(result['q'], before, after)]
    return result
