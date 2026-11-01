"""One bounded, periodic motion task for prescribed playback and simulation.

Controls use the generated joint's native unit: degrees or millimetres. The
cosine blend travels between two waypoints and returns once per period. A run's
duration never changes that period or the requested speed/acceleration.
"""
from __future__ import annotations

import math

SCHEMA = 'cybrgeo-motion/1'
INTERPOLATION = 'cosine_out_and_back'
DEFAULT_SOURCE = dict(period_s=8., policy='bounded_excursion')


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return float(value)


def validate_motion_source(source, actuator_ids):
    if not isinstance(source, dict) or 'period_s' not in source:
        raise ValueError('motion requires period_s and a bounded policy or two actuator waypoints')
    period = _finite(source['period_s'], 'motion.period_s')
    if not .1 <= period <= 3600:
        raise ValueError('motion.period_s must be within [0.1, 3600] seconds')
    if set(source) == {'period_s', 'policy'}:
        if source['policy'] != 'bounded_excursion':
            raise ValueError('Unsupported motion policy')
        return dict(period_s=period, policy=source['policy'])
    if set(source) != {'period_s', 'waypoints'}:
        raise ValueError('motion must specify exactly one policy or explicit waypoint pair')
    points = source['waypoints']
    if not isinstance(points, list) or len(points) != 2:
        raise ValueError('motion requires exactly two named actuator waypoints')
    expected = set(actuator_ids)
    normalized = []
    for point in points:
        if not isinstance(point, dict) or set(point) != expected:
            raise ValueError('Each motion waypoint must name every source actuator exactly once')
        normalized.append({name: _finite(value, 'motion.'+name) for name, value in point.items()})
    return dict(period_s=period, waypoints=normalized)


def derive_motion(joints, motion_source=None, actuator_bindings=None):
    """Resolve source actuator names once, then expose a backend-neutral task."""
    controls = [joint for joint in joints if joint['type'] != 'fixed' and not joint.get('drive')]
    by_name = {joint['name']: joint for joint in controls}
    bindings = dict(actuator_bindings if actuator_bindings is not None else {name: name for name in by_name})
    if len(set(bindings.values())) != len(bindings) or set(bindings.values()) != set(by_name):
        raise ValueError('Motion actuator bindings must cover each independent joint once')
    source = validate_motion_source(DEFAULT_SOURCE if motion_source is None else motion_source, bindings)
    if 'waypoints' in source:
        points = [{bindings[name]: value for name, value in point.items()} for point in source['waypoints']]
    else:
        first, second = {}, {}
        for index, joint in enumerate(controls):
            reference = joint['value']
            left, right = reference-joint['min'], joint['max']-reference
            direction = -1 if left > right or (left == right and index % 2) else 1
            available = left if direction < 0 else right
            amplitude = min(.7*available, 22. if joint['type'] == 'revolute' else .7*available)
            first[joint['name']] = reference
            second[joint['name']] = reference + direction*amplitude
        points = [first, second]
    for point in points:
        for name, value in point.items():
            if not by_name[name]['min'] <= value <= by_name[name]['max']:
                raise ValueError(f'Motion waypoint for {name} is outside its constructed joint limits')
    return MotionTask.from_dict(dict(schema=SCHEMA, period_s=source['period_s'],
                                     interpolation=INTERPOLATION, waypoints=points,
                                     actuator_bindings=bindings)).to_dict()


class MotionTask:
    """Analytic position, velocity, acceleration and peak-rate evaluator."""

    @classmethod
    def from_dict(cls, document):
        if (not isinstance(document, dict)
                or set(document) != {'schema', 'period_s', 'interpolation', 'waypoints', 'actuator_bindings'}
                or document['schema'] != SCHEMA or document['interpolation'] != INTERPOLATION):
            raise ValueError('Unsupported lowered motion task')
        bindings = document['actuator_bindings']
        if not isinstance(bindings, dict) or any(not isinstance(k, str) or not isinstance(v, str) for k, v in bindings.items()):
            raise ValueError('Motion actuator bindings must map string names')
        if len(set(bindings.values())) != len(bindings):
            raise ValueError('Motion actuator bindings must be unique')
        normalized = validate_motion_source(dict(period_s=document['period_s'], waypoints=document['waypoints']), bindings.values())
        task = cls()
        task.period_s = normalized['period_s']
        task._points = normalized['waypoints']
        task._bindings = dict(bindings)
        return task

    @classmethod
    def from_joints(cls, joints, period_s=8.):
        """Legacy plan fallback through the same bounded policy and evaluator."""
        return cls.from_dict(derive_motion(joints, dict(period_s=period_s, policy='bounded_excursion')))

    @property
    def waypoints(self):
        return [dict(point) for point in self._points]

    def to_dict(self):
        return dict(schema=SCHEMA, period_s=self.period_s, interpolation=INTERPOLATION,
                    waypoints=self.waypoints, actuator_bindings=dict(self._bindings))

    def sample(self, time_seconds):
        time = _finite(time_seconds, 'motion time')
        omega = math.tau/self.period_s
        phase = omega*time
        blend, speed, acceleration = .5*(1-math.cos(phase)), .5*omega*math.sin(phase), .5*omega*omega*math.cos(phase)
        values, velocities, accelerations = {}, {}, {}
        for name, start in self._points[0].items():
            delta = self._points[1][name]-start
            values[name] = start+delta*blend
            velocities[name] = delta*speed
            accelerations[name] = delta*acceleration
        return dict(values=values, velocities=velocities, accelerations=accelerations)

    def peak_rates(self):
        omega = math.tau/self.period_s
        return {name: dict(speed=abs(self._points[1][name]-start)*omega/2,
                           acceleration=abs(self._points[1][name]-start)*omega*omega/2)
                for name, start in self._points[0].items()}
