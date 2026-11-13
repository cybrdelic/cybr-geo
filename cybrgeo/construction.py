"""Commit-time constraints on explicitly registered construction bodies.

Modeling intermediates are not assembly bodies. Every pair of registered bodies
is separate by default; exceptions must be named. Valid-solid assumptions remain
explicit: this is not a self-intersection certifier or a placement solver.
"""
from dataclasses import dataclass
from itertools import combinations
import math
from .interference import SolidQuery, Interference
import numpy as np


@dataclass(frozen=True)
class Body:
    clearance: float
    tolerance: object


@dataclass(frozen=True)
class PairConstraint:
    clearance: float
    allow_reason: object = None


class ConstructionError(ValueError):
    def __init__(self, conflicts):
        self.conflicts = tuple(conflicts)
        names = ', '.join(f'{a} / {b}: {result.state}' for a, b, result in conflicts)
        super().__init__('Construction rejected: ' + names)


def _clearance(value):
    value = float(value)
    if not math.isfinite(value) or value < 0:
        raise ValueError('clearance must be finite and nonnegative')
    return value


class Construction:
    """Internal Graph policy and committed-query cache, not an extra test pass."""
    def __init__(self):
        self.bodies = {}
        self.pairs = {}
        self._queries = {}
        self._checks = {}

    def body(self, name, *, assume_valid_solid=False, clearance=0., tolerance=None):
        if assume_valid_solid is not True:
            raise ValueError('Registering a body requires an explicit valid-solid assumption')
        if tolerance is not None:
            tolerance = float(tolerance)
            if not math.isfinite(tolerance) or tolerance <= 0:
                raise ValueError('tolerance must be finite and positive')
        self.bodies[name] = Body(_clearance(clearance), tolerance)

    def pair(self, a, b, *, clearance=0., allow_reason=None):
        if a == b or a not in self.bodies or b not in self.bodies:
            raise ValueError('Pair requires two different registered bodies')
        if allow_reason is not None and (not isinstance(allow_reason, str) or not allow_reason.strip()):
            raise ValueError('Intentional intersection requires a nonempty reason')
        self.pairs[tuple(sorted((a,b)))] = PairConstraint(_clearance(clearance), allow_reason)

    def stage(self, evaluate, report, *, force=False):
        queries = {}; checks = {}; results = []; conflicts = []
        report['construction'] = dict(bodies=list(self.bodies), checked=[], reused=[], results=results,
                                      scope='registered closed solid pairs at the built pose',
                                      continuous_motion_checked=False, self_intersections_checked=False)
        feedback = report['construction']
        for name, policy in self.bodies.items():
            mesh = evaluate(name)
            old = self._queries.get(name)
            if not force and old is not None and old[0] is mesh and old[1] == policy:
                query = old[2]
            else:
                query = SolidQuery(mesh, assume_valid_solid=True, tolerance=policy.tolerance)
            queries[name] = (mesh, policy, query)
        for a, b in combinations(sorted(self.bodies), 2):
            policy = self.pairs.get((a,b), PairConstraint(max(self.bodies[a].clearance, self.bodies[b].clearance)))
            qa, qb = queries[a][2], queries[b][2]
            old = self._checks.get((a,b))
            if not force and old is not None and old[0] is qa and old[1] is qb and old[2] == policy:
                result = old[3]; feedback['reused'].append((a,b))
            else:
                # A disjoint bounding box is a conservative separation proof,
                # not an exact nearest-surface distance or witness.
                ra, rb = qa.bvh.root, qb.bvh.root
                lower = float(np.linalg.norm(np.maximum(0., np.maximum(ra.low-rb.high, rb.low-ra.high))))
                tolerance = max(qa.bvh.tolerance, qb.bvh.tolerance)
                if lower > policy.clearance + tolerance:
                    result = Interference('separated', lower, None, None, None,
                                          policy.clearance, policy.allow_reason, 'aabb_lower_bound')
                else:
                    result = qa.check_clearance(qb, clearance=policy.clearance, allow_reason=policy.allow_reason)
                feedback['checked'].append((a,b))
            checks[a,b] = (qa,qb,policy,result)
            results.append((a,b,result))
            if result.violation: conflicts.append((a,b,result))
        if conflicts:
            raise ConstructionError(conflicts)
        return queries, checks

    def commit(self, staged):
        self._queries, self._checks = staged
