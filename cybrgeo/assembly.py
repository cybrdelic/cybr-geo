"""Final physical assemblies, not temporary modeling geometry.

OpenCascade supplies the solid validity, Boolean and distance evidence. This is
an all-part, single-pose contract at a declared linear tolerance, not a theorem
about arbitrary meshes, continuous motion, strength or design quality.
"""
from dataclasses import dataclass
from itertools import combinations
from types import MappingProxyType
from pathlib import Path
import json
import math
import re
import hashlib
import io
from copy import deepcopy
from .solid import Solid


class AssemblyError(ValueError):
    def __init__(self, report):
        self.report = report
        super().__init__('Assembly rejected: ' + '; '.join(report['errors']))


def _fingerprint(part):
    """Serialize actual B-rep content: identity, bounds and volume are not enough."""
    stream = io.BytesIO()
    if not part.shape.exportBrep(stream):
        raise ValueError('B-rep fingerprint serialization failed')
    return hashlib.sha256(stream.getvalue()).hexdigest()


def _check(parts, contacts, tolerance, clearance, cache=None):
    previous_bodies, previous_pairs = cache or ({}, {})
    bodies, pairs, fingerprints, shapes = {}, {}, {}, {}

    report = dict(scope='all declared final analytic solids at one pose',
                  participants=list(parts), pair_count=0, pairs=[], errors=[],
                  reused_bodies=[], evaluated_bodies=[], reused_pairs=[], evaluated_pairs=[],
                  tolerance=tolerance, clearance=clearance,
                  backend='CadQuery/OpenCascade', committed=False,
                  validity='B-rep validity, closure, positive volume and self-interference analysis',
                  continuous_motion_checked=False,
                  triangle_mesh_self_intersections_certified=False)
    for name, part in parts.items():
        if not isinstance(part, Solid):
            report['errors'].append(f'{name}: unresolved solid validity; retain an analytic Solid')
            continue
        try:
            fingerprint = _fingerprint(part)
            fingerprints[name] = fingerprint
            bodies[name] = fingerprint
            # OCCT may attach analysis data/p-curves during Boolean queries.
            # Query private copies so evidence cannot mutate author inputs.
            shapes[name] = part.shape.copy()
            if previous_bodies.get(name) == fingerprint:
                report['reused_bodies'].append(name)
                continue
            report['evaluated_bodies'].append(name)
            shape = shapes[name]
            from OCP.BOPAlgo import BOPAlgo_ArgumentAnalyzer
            analyzer = BOPAlgo_ArgumentAnalyzer()
            analyzer.SetShape1(shape.wrapped)
            analyzer.SelfInterMode = True
            analyzer.Perform()
            if analyzer.HasErrors() or analyzer.HasWarnings():
                raise ValueError("unresolved self-interference analysis")
            if analyzer.HasFaulty():
                report['errors'].append(f'{name}: B-rep self-interference or invalid argument')
            if (not shape.isValid() or len(shape.Solids()) != 1 or
                    not shape.Shells() or not all(s.Closed() for s in shape.Shells()) or
                    not math.isfinite(part.volume) or part.volume <= 0):
                report['errors'].append(f'{name}: invalid, open, empty or multiple solids')
        except Exception as exc:
            report['errors'].append(f'{name}: unresolved validity ({type(exc).__name__})')
    if report['errors']:
        raise AssemblyError(report)
    for a, b in combinations(parts, 2):
        key = tuple(sorted((a, b)))
        signature = (fingerprints[a], fingerprints[b], key in contacts, tolerance, clearance)
        old = previous_pairs.get((a, b))
        if old is not None and old[0] == signature:
            record = deepcopy(old[1])
            report['pairs'].append(record)
            report['pair_count'] += 1
            report['reused_pairs'].append([a, b])
            pairs[a, b] = old
            continue
        report['evaluated_pairs'].append([a, b])
        record = dict(parts=[a, b], intended_contact=key in contacts)
        try:
            sa, sb = shapes[a], shapes[b]
            # A box lower bound avoids expensive Booleans only when separation
            # is already proved. Containment never takes this fast path.
            aa, bb = sa.BoundingBox(), sb.BoundingBox()
            gap_bound = math.sqrt(sum(max(0., getattr(aa, k+'min')-getattr(bb, k+'max'),
                                          getattr(bb, k+'min')-getattr(aa, k+'max'))**2
                                      for k in ('x', 'y', 'z')))
            if gap_bound > max(clearance, tolerance) and key not in contacts:
                record.update(state='separated', gap=gap_bound, evidence='aabb_lower_bound')
            else:
                common = sa.intersect(sb)
                volume = float(common.Volume())
                gap = float(sa.distance(sb))
                if not common.isValid() or not math.isfinite(volume+gap) or volume < 0 or gap < 0:
                    raise ValueError('invalid Boolean/distance result')
                record.update(common_volume=volume, gap=gap, evidence='brep_boolean_and_distance')
                # No positive-volume exception, even for explicitly named contact.
                if volume > 0:
                    record['state'] = 'penetration'
                elif key in contacts:
                    record['state'] = 'contact' if gap <= tolerance else 'missing_contact'
                elif gap <= tolerance:
                    record['state'] = 'undeclared_contact'
                elif gap < clearance:
                    record['state'] = 'insufficient_clearance'
                else:
                    record['state'] = 'separated'
        except Exception as exc:
            record.update(state='unresolved', detail=type(exc).__name__ + ': ' + str(exc))
        pairs[a, b] = (signature, deepcopy(record))
        report['pairs'].append(record)
        report['pair_count'] += 1
        if record['state'] not in ('separated', 'contact'):
            report['errors'].append(f"{a} / {b}: {record['state']}")
    if report['errors']:
        raise AssemblyError(report)
    report['committed'] = True
    return report, (bodies, pairs)


@dataclass(frozen=True)
class AssemblyBuild:
    parts: object
    report: object
    _contacts: frozenset
    _tolerance: float
    _clearance: float
    _cache: object

    def export(self, directory, *, mesh_tolerance=.05, angular_tolerance=.1):
        """Recheck accepted B-reps; export all participants, never a subset.

        Mesh tessellation is a representation, not a new solid-clearance proof.
        A manifest records that distinction. Existing directories are refused so
        a failed export cannot overwrite a previous good package.
        """
        from .exchange import export_obj
        import tempfile
        import shutil
        destination = Path(directory)
        if destination.exists():
            raise FileExistsError(destination)
        report, _ = _check(self.parts, self._contacts, self._tolerance, self._clearance, self._cache)
        destination.parent.mkdir(parents=True, exist_ok=True)
        staging = Path(tempfile.mkdtemp(prefix='.assembly-', dir=destination.parent))
        try:
            for name, solid in self.parts.items():
                mesh = solid.mesh(tolerance=mesh_tolerance, angular_tolerance=angular_tolerance)
                export_obj(mesh, staging / (name+'.obj'))
            report['mesh_export'] = dict(tolerance=mesh_tolerance,
                angular_tolerance=angular_tolerance,
                scope='tessellations of checked B-reps; mesh interference not certified')
            (staging/'assembly.json').write_text(json.dumps(report, indent=2))
            staging.rename(destination)
        except Exception:
            shutil.rmtree(staging)
            raise
        return destination/'assembly.json'


class Assembly:
    """Finalization boundary for authored analytic physical parts.

    ``Assembly({'housing': housing, 'shaft': shaft}, contacts=[...]).build()``.
    Contacts name intended zero-volume touching pairs, never overlap waivers.
    Every supplied part participates, regardless of visibility. A failed edit
    preserves both the prior inputs and accepted build. Factory dependencies can
    be computed by Graph before this boundary; no object-specific factories live
    here. Mesh-only inputs are rejected as unresolved, not silently trusted.
    """
    def __init__(self, parts, *, contacts=(), tolerance=1e-6, clearance=0.):
        self._parts = dict(parts)
        if not self._parts or any(not isinstance(n, str) or not re.fullmatch(r'[A-Za-z0-9_-]+', n)
                                  for n in self._parts):
            raise ValueError('Provide nonempty named parts (letters, numbers, underscore, hyphen)')
        self._contacts = frozenset(tuple(sorted(pair)) for pair in contacts)
        if any(len(pair) != 2 or pair[0] == pair[1] or not set(pair) <= self._parts.keys()
               for pair in self._contacts):
            raise ValueError('Contacts require two different declared part names')
        self._tolerance, self._clearance = float(tolerance), float(clearance)
        if not math.isfinite(self._tolerance) or self._tolerance <= 0:
            raise ValueError('tolerance must be finite and positive')
        if not math.isfinite(self._clearance) or self._clearance < 0:
            raise ValueError('clearance must be finite and nonnegative')
        self.last = None
        self._cache = ({}, {})

    def build(self):
        report, staged_cache = _check(self._parts, self._contacts, self._tolerance, self._clearance, self._cache)
        # Snapshot CAD shapes so later caller mutation cannot change this build.
        parts = MappingProxyType({name: Solid(part.shape.copy()) for name, part in self._parts.items()})
        result = AssemblyBuild(parts, report, self._contacts, self._tolerance, self._clearance, staged_cache)
        self._cache = staged_cache
        self.last = result
        return result

    def edit(self, **parts):
        if not parts.keys() <= self._parts.keys():
            raise KeyError('Edits replace declared parts only; create a new Assembly for new membership')
        previous = self._parts
        self._parts = {**previous, **parts}
        try:
            return self.build()
        except Exception:
            self._parts = previous
            raise
