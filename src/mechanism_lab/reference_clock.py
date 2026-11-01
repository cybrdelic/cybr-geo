"""Periodic phase timing for an authored reference, independent of its mechanism.

Only time is changed: within each interval, q is unchanged, qd is multiplied
by d(source time)/d(physical time), and qdd by its square. A discontinuous rate
is suitable only at stationary joins. ``bind`` checks those joins once before
returning a callable; there is deliberately no unchecked retiming entry point.

This is a bounded numerical continuity screen, not an analytic path certificate
or dynamics result. Authors specify probe size and coordinate-unit tolerances,
and retain the returned measurements. Switched auxiliary fields such as grip
commands are passed through unchanged and are not certified as smooth motion.
"""
from __future__ import annotations

from bisect import bisect_right
from collections.abc import Mapping
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
import math
from numbers import Real
from pathlib import Path


_IMPLEMENTATION_SHA256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def _number(value, label, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f'{label} must be a finite number')
    value = float(value)
    if positive and value <= 0:
        raise ValueError(f'{label} must be positive')
    return value


def _vector(value, label):
    if isinstance(value, (str, bytes, Mapping)):
        raise ValueError(f'{label} must be a nonempty finite coordinate vector')
    try:
        values = tuple(_number(item, label) for item in value)
    except TypeError as error:
        raise ValueError(f'{label} must be a nonempty finite coordinate vector') from error
    if not values:
        raise ValueError(f'{label} must be a nonempty finite coordinate vector')
    return values


def _sample(reference, time_s, parameters):
    row = reference(time_s, parameters)
    if not isinstance(row, Mapping):
        raise ValueError('Reference must return a mapping containing q, qd and qdd')
    try:
        vectors = {name: _vector(row[name], name) for name in ('q', 'qd', 'qdd')}
    except KeyError as error:
        raise ValueError('Reference must contain q, qd and qdd') from error
    if len({len(value) for value in vectors.values()}) != 1:
        raise ValueError('Reference q, qd and qdd must have identical coordinate counts')
    return row, vectors


def _maximum(values):
    return max(abs(value) for value in values)


@dataclass(frozen=True)
class PhaseClock:
    """Explicit source and physical knot times for a repeating task.

    Both arrays start at zero, end at their respective positive periods, and
    contain the same number of strictly increasing finite knots. Landmark
    conversion retains cycle counts, including negative and later cycles.
    The bound callback itself receives times in the source's first cycle.
    """
    source_times_s: tuple
    physical_times_s: tuple

    def __post_init__(self):
        source = _vector(self.source_times_s, 'Source knots')
        physical = _vector(self.physical_times_s, 'Physical knots')
        if len(source) != len(physical) or len(source) < 2:
            raise ValueError('Source and physical knots require equal lengths of at least two')
        for label, knots in (('Source', source), ('Physical', physical)):
            if knots[0] != 0. or any(b <= a for a, b in zip(knots, knots[1:])):
                raise ValueError(f'{label} knots must start at zero and strictly increase')
        object.__setattr__(self, 'source_times_s', source)
        object.__setattr__(self, 'physical_times_s', physical)
        # Validate both directions before accepting an unrepresentable map.
        for a, b, x, y in zip(source, source[1:], physical, physical[1:]):
            rate = _number((b-a)/(y-x), 'Clock rate', positive=True)
            _number(rate*rate, 'Squared clock rate', positive=True)
            _number((y-x)/(b-a), 'Inverse clock rate', positive=True)

    @property
    def source_period_s(self):
        return self.source_times_s[-1]

    @property
    def physical_period_s(self):
        return self.physical_times_s[-1]

    @property
    def rates(self):
        return tuple((b-a)/(y-x) for a, b, x, y in zip(
            self.source_times_s, self.source_times_s[1:],
            self.physical_times_s, self.physical_times_s[1:]))

    @staticmethod
    def _locate(time_s, knots):
        time_s = _number(time_s, 'Time')
        cycle, local = divmod(time_s, knots[-1])
        if not math.isfinite(cycle):
            raise ValueError('Time exceeds the representable clock cycle range')
        # Float divmod can round a tiny negative remainder to exactly the period.
        if local >= knots[-1]:
            cycle, local = cycle + 1, 0.
        index = min(bisect_right(knots, local)-1, len(knots)-2)
        return cycle, local, index

    @classmethod
    def _map(cls, time_s, origin, target):
        cycle, local, index = cls._locate(time_s, origin)
        value = (cycle*target[-1] + target[index]
                 + (local-origin[index])*(target[index+1]-target[index])/(origin[index+1]-origin[index]))
        return _number(value, 'Mapped time')

    def source_time(self, physical_time_s):
        """Convert a physical timestamp to an unwrapped source timestamp."""
        return self._map(physical_time_s, self.physical_times_s, self.source_times_s)

    def physical_time(self, source_time_s):
        """Convert a source landmark to the same phase in physical time."""
        return self._map(source_time_s, self.source_times_s, self.physical_times_s)

    def rate(self, physical_time_s):
        """Right-hand source-time rate at the given physical timestamp."""
        return self.rates[self._locate(physical_time_s, self.physical_times_s)[2]]

    def source(self):
        """Portable exact timing and implementation identity for source receipts."""
        return dict(schema='cybrgeo-reference-clock/1',
            source_times_s=list(self.source_times_s), physical_times_s=list(self.physical_times_s),
            source_period_s=self.source_period_s, physical_period_s=self.physical_period_s,
            rates=list(self.rates),
            implementation_sha256=_IMPLEMENTATION_SHA256)

    def identity(self):
        return hashlib.sha256(json.dumps(self.source(), sort_keys=True,
            separators=(',', ':'), allow_nan=False).encode()).hexdigest()

    @classmethod
    def from_source(cls, source):
        """Restore only a matching complete clock/implementation receipt."""
        if not isinstance(source, Mapping):
            raise ValueError('Clock source must be its complete source receipt')
        try:
            clock = cls(source['source_times_s'], source['physical_times_s'])
        except KeyError as error:
            raise ValueError('Clock source is missing its exact knot arrays') from error
        if dict(source) != clock.source():
            raise ValueError('Clock source schema, timing or implementation identity differs')
        return clock

    def bind(self, reference, parameters, *, probe_s, position_tolerance,
             velocity_tolerance, acceleration_tolerance):
        """Validate once, then return a callable of physical time only.

        The callback must be pure and return finite, equal-length q/qd/qdd
        vectors on [0, source period]. Parameters are copied once; neither the
        caller's mapping nor callback-owned output vectors are modified.

        Tolerances use the source coordinates, coordinate/s and coordinate/s².
        They bound sampled q continuity and qd/qdd continuity, and bound qd/qdd
        near zero wherever the clock rate changes. Two one-sided probe sizes
        plus the exact join are measured. Small finite-difference residuals are
        retained rather than silently rounded to zero. Large speed-up rates
        amplify these residuals; the report includes mapped derivative jumps.
        Auxiliary command fields are outside this smooth-coordinate contract.
        """
        if not callable(reference):
            raise TypeError('Reference must be callable')
        if not isinstance(parameters, Mapping):
            raise TypeError('Reference parameters must be a mapping')
        probe_s = _number(probe_s, 'Probe size', positive=True)
        tolerances = dict(position=_number(position_tolerance, 'Position tolerance', positive=True),
            velocity=_number(velocity_tolerance, 'Velocity tolerance', positive=True),
            acceleration=_number(acceleration_tolerance, 'Acceleration tolerance', positive=True))
        if probe_s >= min(b-a for a, b in zip(self.source_times_s, self.source_times_s[1:]))/4:
            raise ValueError('Probe size must stay within each source interval')
        snapshot = deepcopy(dict(parameters))
        report = self._validate(reference, snapshot, probe_s, tolerances)
        return _BoundReference(self, reference, snapshot, report)

    def _validate(self, reference, parameters, probe_s, tolerances):
        joins, coordinate_count = [], None
        for index, time_s in enumerate(self.source_times_s[:-1]):
            previous_rate, next_rate = self.rates[index-1], self.rates[index]
            changed = previous_rate != next_rate
            _, center = _sample(reference, time_s, parameters)
            if coordinate_count is None:
                coordinate_count = len(center['q'])
            if len(center['q']) != coordinate_count:
                raise ValueError('Reference coordinate count changes across the source cycle')
            measurements = []
            for step in (probe_s, probe_s/2):
                _, left = _sample(reference, (time_s-step) % self.source_period_s, parameters)
                _, right = _sample(reference, (time_s+step) % self.source_period_s, parameters)
                if any(len(row['q']) != coordinate_count for row in (left, right)):
                    raise ValueError('Reference coordinate count changes across a join')
                # Extrapolate both short one-sided probes to the join. This
                # permits continuous moving joins when the rate is unchanged.
                left_q = [q + v*step + .5*a*step*step for q, v, a in zip(left['q'], left['qd'], left['qdd'])]
                right_q = [q - v*step + .5*a*step*step for q, v, a in zip(right['q'], right['qd'], right['qdd'])]
                position_jump = max(_maximum(a-b for a, b in zip(left_q, right_q)),
                    _maximum(a-b for a, b in zip(left_q, center['q'])),
                    _maximum(a-b for a, b in zip(right_q, center['q'])))
                velocity_jump = max(_maximum(a-b for a, b in zip(left['qd'], right['qd'])),
                    _maximum(a-b for a, b in zip(left['qd'], center['qd'])),
                    _maximum(a-b for a, b in zip(right['qd'], center['qd'])))
                acceleration_jump = max(_maximum(a-b for a, b in zip(left['qdd'], right['qdd'])),
                    _maximum(a-b for a, b in zip(left['qdd'], center['qdd'])),
                    _maximum(a-b for a, b in zip(right['qdd'], center['qdd'])))
                velocity_peak = _maximum(value for row in (left, center, right) for value in row['qd'])
                acceleration_peak = _maximum(value for row in (left, center, right) for value in row['qdd'])
                passed = (position_jump <= tolerances['position']
                    and velocity_jump <= tolerances['velocity']
                    and acceleration_jump <= tolerances['acceleration']
                    and (not changed or (velocity_peak <= tolerances['velocity']
                                         and acceleration_peak <= tolerances['acceleration'])))
                measurements.append(dict(probe_s=step, passed=passed,
                    position_jump=position_jump, velocity_jump=velocity_jump,
                    acceleration_jump=acceleration_jump,
                    maximum_absolute_velocity=velocity_peak, maximum_absolute_acceleration=acceleration_peak,
                    mapped_velocity_jump=_maximum(a*previous_rate-b*next_rate for a, b in zip(left['qd'], right['qd'])),
                    mapped_acceleration_jump=_maximum(a*previous_rate**2-b*next_rate**2 for a, b in zip(left['qdd'], right['qdd']))))
            if index == 0:
                _, end = _sample(reference, self.source_period_s, parameters)
                if len(end['q']) != coordinate_count:
                    raise ValueError('Reference coordinate count changes at the period endpoint')
                endpoint = {label: _maximum(a-b for a, b in zip(center[key], end[key]))
                            for label, key in (('position', 'q'), ('velocity', 'qd'), ('acceleration', 'qdd'))}
                if any(endpoint[key] > tolerances[key] for key in endpoint):
                    raise ValueError(f'Reference period endpoints are discontinuous: {endpoint}')
            joins.append(dict(source_time_s=time_s, physical_time_s=self.physical_time(time_s),
                previous_rate=previous_rate, next_rate=next_rate, rate_changes=changed,
                measurements=measurements, passed=all(row['passed'] for row in measurements)))
        failed = [row['source_time_s'] for row in joins if not row['passed']]
        if failed:
            raise ValueError(f'Reference has a discontinuous or nonstationary clock join at source seconds {failed}')
        return dict(schema='cybrgeo-reference-clock-validation/1', passed=True,
            source_coordinates=coordinate_count, probe_s=probe_s, tolerances=tolerances,
            period_endpoint_differences=endpoint, joins=joins,
            scope='Two one-sided numerical probes and exact join values; not an analytic continuity proof. Auxiliary command fields are not certified smooth.')


class _BoundReference:
    """Validated callback wrapper; instantiate through PhaseClock.bind."""
    def __init__(self, clock, reference, parameters, validation):
        self._clock, self._reference, self._parameters = clock, reference, parameters
        self._validation = validation

    @property
    def validation_report(self):
        return deepcopy(self._validation)

    def source(self):
        return dict(clock=self._clock.source(), validation=self.validation_report)

    def __call__(self, physical_time_s):
        _, local, index = self._clock._locate(physical_time_s, self._clock.physical_times_s)
        rate = self._clock.rates[index]
        source_time = self._clock.source_times_s[index] + (local-self._clock.physical_times_s[index])*rate
        row, vectors = _sample(self._reference, source_time, self._parameters)
        if len(vectors['q']) != self._validation['source_coordinates']:
            raise ValueError('Reference coordinate count changed after clock binding')
        result = deepcopy(dict(row))
        result['q'] = list(vectors['q'])
        result['qd'] = [_number(value*rate, 'Mapped velocity') for value in vectors['qd']]
        result['qdd'] = [_number(value*rate*rate, 'Mapped acceleration') for value in vectors['qdd']]
        return result
