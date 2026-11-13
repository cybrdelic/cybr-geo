"""Explicit dependency evaluation for pure geometry operations.

Cache scope is an operation, not an inferred spatial patch. Functions must not
read mutable globals or mutate inputs. Replace the operation to change its code.
"""
from dataclasses import dataclass
from types import MappingProxyType
from time import perf_counter
import copy
import hashlib
import numpy as np
from .construction import Construction
from .model import Mesh


def _fingerprint(value):
    if isinstance(value, np.ndarray):
        if value.dtype.hasobject: raise TypeError('Object arrays are not supported graph inputs')
        return ('array',value.dtype.str,value.shape,hashlib.sha256(value.tobytes()).digest())
    if isinstance(value,(str,int,float,bool,type(None))): return (type(value).__name__,value)
    if isinstance(value,(tuple,list)): return (type(value).__name__,tuple(_fingerprint(v) for v in value))
    if isinstance(value,dict): return ('dict',tuple(sorted((k,_fingerprint(v)) for k,v in value.items())))
    raise TypeError('Graph inputs must be numbers, strings, arrays, lists, tuples or dictionaries')


def _freeze(value):
    if isinstance(value,np.ndarray):
        value=value.copy();value.flags.writeable=False;return value
    if isinstance(value,(tuple,list)):return tuple(_freeze(v) for v in value)
    if isinstance(value,dict):return MappingProxyType({k:_freeze(v) for k,v in value.items()})
    return copy.deepcopy(value)


@dataclass(frozen=True)
class Build:
    outputs: object
    report: dict


class BuildError(RuntimeError):
    def __init__(self,node,cause,report):
        super().__init__(f'{node}: {cause}')
        self.node,self.cause,self.report=node,cause,report


class Graph:
    """Named inputs and pure operations. Failed builds preserve last good cache.

    physical=True requires final part declarations, protected output coverage,
    connected closed solids and no blanket overlap exceptions. It does not turn
    geometric factories into a universal contact or structural solver.
    """
    def __init__(self, *, physical=False):
        if not isinstance(physical, bool): raise TypeError('physical must be boolean')
        self._physical=physical
        self._inputs={};self._ops={};self._cache={};self._revision={};self.last=None
        self._construction=Construction()
        self._parts=[]
        self._fits={}

    def input(self,name,value):
        if name in self._inputs or name in self._ops:raise ValueError(f'Duplicate node: {name}')
        self._inputs[name]=(_fingerprint(value),_freeze(value));self._revision[name]=1
        return name

    def op(self,name,function,*dependencies):
        if name in self._inputs or name in self._ops:raise ValueError(f'Duplicate node: {name}')
        if not callable(function):raise TypeError('Operation must be callable')
        unknown=set(dependencies)-set(self._inputs)-set(self._ops)
        if unknown:raise KeyError(f'Unknown dependencies: {sorted(unknown)}')
        self._ops[name]=(function,tuple(dependencies),1)
        return name

    def replace(self,name,function):
        if self._physical and name in self._fits:
            raise ValueError('Cannot replace a physical fit with an unbound operation; rebuild its feature-driven declaration')
        old,deps,version=self._ops[name]
        if not callable(function):raise TypeError('Operation must be callable')
        self._ops[name]=(function,deps,version+1)
        self._fits.pop(name,None)

    def set(self,name,value):
        if name not in self._inputs:raise KeyError(f'Not an input: {name}')
        fingerprint=_fingerprint(value)
        if fingerprint!=self._inputs[name][0]:
            self._inputs[name]=(fingerprint,_freeze(value));self._revision[name]+=1
        return self

    def part(self, name, function, *dependencies, assume_valid_solid=False,
             clearance=0., tolerance=None):
        """Create a final mesh operation and register it in one call.

        New code uses part() for final bodies and op() for intermediate work.
        A graph using part() defaults to returning these final parts; legacy
        op()/body() graphs retain their existing build target behavior.
        """
        # Validate policy before modifying the graph, including duplicates.
        policy = Construction()
        policy.body(name, assume_valid_solid=assume_valid_solid,
                    clearance=clearance, tolerance=tolerance)
        self.op(name, function, *dependencies)
        self._construction.bodies[name] = policy.bodies[name]
        self._parts.append(name)
        return name

    def fit(self, name, make, host, *, target, source, dependencies=(),
            assume_valid_solid=False, clearance=0., slide=0., angle=0.):
        """Construct from a measured host feature, then align actual features.

        make(target_frame, *dependencies) returns an unplaced Mesh. The factory
        derives dimensions from that frame (e.g. frame.fitted_radius(gap)).
        Missing/ambiguous feature labels fail; there is no nearest-face guessing.
        target can also name an operation returning a Feature, allowing explicit
        surface correspondence to rebuild with its host. This is a one-way
        dependency, not a closed-loop assembly solver.
        """
        from .mates import Feature, place
        if not isinstance(target, (Feature, str)) or not isinstance(source, Feature):
            raise TypeError('fit needs a target Feature or feature-node name, and a source Feature')
        dynamic_target = isinstance(target, str)
        if not callable(make): raise TypeError('Fit factory must be callable')
        def construct(host_mesh, *args):
            feature = args[0] if dynamic_target else target
            if not isinstance(feature, Feature):
                raise TypeError('Target feature node did not produce a Feature')
            frame = feature.resolve(host_mesh)
            mesh = make(frame, *(args[1:] if dynamic_target else args))
            return place(mesh, source.resolve(mesh), frame, slide=slide, angle=angle)
        feature_dependencies = (target,) if dynamic_target else ()
        node = self.part(name, construct, host, *feature_dependencies, *dependencies,
                         assume_valid_solid=assume_valid_solid, clearance=clearance)
        self._fits[name] = dict(host=host, target=target if dynamic_target else target.label, source=source.label,
                               mode='feature-driven reconstruction and rigid alignment')
        return node

    def edit(self, **inputs):
        """Apply input changes and build atomically; restore draft on failure.

        Operation functions must be pure. External side effects cannot be undone.
        For arbitrary node names pass a mapping: edit(**{'shaft.radius': 6.0}).
        """
        previous_inputs = self._inputs.copy()
        previous_revisions = self._revision.copy()
        try:
            for name, value in inputs.items(): self.set(name, value)
            return self.build()
        except Exception:
            self._inputs = previous_inputs
            self._revision = previous_revisions
            raise

    def body(self, node, *, assume_valid_solid=False, clearance=0., tolerance=None):
        """Register a final solid. Every registered pair must remain separate.

        Intermediates (profiles, cutters, fields) stay ordinary graph nodes.
        All bodies are evaluated even for a partial-target build, preventing a
        target selection from bypassing an assembly constraint.
        """
        if self._physical: raise ValueError('Physical assemblies declare final bodies with part(), not late body() registration')
        if node not in self._ops: raise KeyError(f'Unknown geometry operation: {node}')
        self._construction.body(node, assume_valid_solid=assume_valid_solid,
                                clearance=clearance, tolerance=tolerance)
        return node

    def separate(self, a, b, *, clearance=0.):
        """Set a pair clearance, or revoke an intentional-overlap exception."""
        self._construction.pair(a,b,clearance=clearance)
        return self

    def allow_overlap(self, a, b, *, reason):
        """Named intentional overlap. Its measured evidence remains in reports."""
        if self._physical:
            raise ValueError('Physical assemblies do not permit blanket overlap exceptions; construct a real joined body or preserve clearance')
        self._construction.pair(a,b,allow_reason=reason)
        return self

    def build(self,targets=None,*,force=False,require_checked=False):
        targets=list(self._parts or self._ops) if targets is None else ([targets] if isinstance(targets,str) else list(targets))
        start=perf_counter();staged={};versions={};seen={}
        report=dict(evaluated=[],reused=[],inputs=[],seconds_by_node={},invalidated=[],
                    relationships={name: dict(value) for name,value in self._fits.items()})
        def evaluate(name):
            if name in seen:return seen[name]
            if name in self._inputs:
                report['inputs'].append(name);versions[name]=self._revision[name]
                seen[name]=self._inputs[name][1];return seen[name]
            if name not in self._ops:raise KeyError(f'Unknown target: {name}')
            fn,deps,version=self._ops[name]
            args=[evaluate(d) for d in deps]
            signature=(version,tuple(versions[d] for d in deps))
            old=self._cache.get(name)
            if not force and old is not None and old[0]==signature:
                report['reused'].append(name);seen[name]=old[1];versions[name]=old[2];return old[1]
            report['invalidated'].append(name)
            tick=perf_counter()
            try: value=fn(*args)
            except Exception as exc:
                report['seconds_by_node'][name]=perf_counter()-tick
                report['seconds']=perf_counter()-start;report['committed']=False
                raise BuildError(name,exc,report) from exc
            report['seconds_by_node'][name]=perf_counter()-tick
            report['evaluated'].append(name)
            revision=(old[2]+1) if old else 1
            staged[name]=(signature,value,revision);versions[name]=revision;seen[name]=value
            return value
        outputs={name:evaluate(name) for name in targets}
        try:
            if self._physical: outputs=_freeze_output(outputs)
            if self._physical:
                for name in self._parts:
                    mesh=evaluate(name)
                    if not isinstance(mesh, Mesh) or not mesh.closed or mesh.report()['components'] != 1:
                        raise ValueError('Physical part '+name+' must be one connected closed mesh; combine joined material with a real union, not concatenation')
            construction = self._construction.stage(evaluate, report, force=force)
            coverage = _coverage(outputs, construction, report['construction'])
            report['coverage'] = coverage
            report['physical_assembly'] = self._physical
            if self._physical:
                # Typed final parts are mandatory; intermediates cannot be
                # exported as finished geometry to bypass the assembly policy.
                if coverage['status'] not in ('pair_constraints_passed','registered_only'):
                    raise ValueError('Physical assembly cannot export ' + coverage['status'])
                if coverage['allowed_pairs']:
                    raise ValueError('Physical assembly contains blanket overlap exceptions')
                if not self._parts:
                    raise ValueError('Physical assembly requires final part() declarations; op() values are intermediates')
            if require_checked and coverage['status'] != 'pair_constraints_passed':
                raise ValueError('Requested checked output, but coverage is ' + coverage['status'])
        except BuildError:
            raise
        except Exception as exc:
            report['seconds']=perf_counter()-start;report['committed']=False
            raise BuildError('construction',exc,report) from exc
        self._cache.update(staged)
        self._construction.commit(construction)
        report['seconds']=perf_counter()-start;report['committed']=True
        result=Build(MappingProxyType(outputs),report);self.last=result;return result


def _freeze_output(value, identities=None):
    """Freeze exported containers; physical outputs have unique Mesh identities."""
    if identities is None: identities=set()
    if isinstance(value, Mesh):
        if id(value) in identities:
            raise ValueError('Physical output repeats a Mesh reference; explicit instances must be separate registered parts')
        identities.add(id(value))
    if isinstance(value, (tuple,list)):
        return tuple(_freeze_output(item,identities) for item in value)
    if isinstance(value, (dict,MappingProxyType)):
        return MappingProxyType({key:_freeze_output(item,identities) for key,item in value.items()})
    return value


def _coverage(outputs, construction, feedback):
    """Conservative typed-Mesh output coverage, not semantic decoding of serialized data."""
    registered = {id(entry[0]): name for name, entry in construction[0].items()}
    enforced = {name for a,b,r in feedback['results'] if not r.allowed_reason for name in (a,b)}
    checked=[]; unchecked=[]; unknown=[]; unconstrained=[]
    def visit(value, path):
        if isinstance(value, Mesh):
            (checked if id(value) in registered else unchecked).append(path)
            if id(value) in registered and registered[id(value)] not in enforced:
                unconstrained.append(path)
        elif isinstance(value, (tuple, list)):
            for i, item in enumerate(value): visit(item, f'{path}[{i}]')
        elif isinstance(value, (dict, MappingProxyType)):
            for key, item in value.items(): visit(item, f'{path}[{key!r}]')
        elif isinstance(value, np.ndarray):
            unknown.append(path)
        elif not isinstance(value, (str, int, float, bool, type(None), np.ndarray, np.number)):
            unknown.append(path)
    for name, value in outputs.items(): visit(value, name)
    pairs = feedback['results']
    if unchecked or unknown: status='unchecked_outputs'
    elif not checked: status='no_geometry_outputs'
    elif not pairs: status='registered_only'
    elif all(r.allowed_reason for a,b,r in pairs): status='all_pairs_exempt'
    elif unconstrained: status='unconstrained_outputs'
    else: status='pair_constraints_passed'
    return dict(status=status, registered_mesh_outputs=checked,
                unchecked_mesh_outputs=unchecked, unknown_outputs=unknown,
                unconstrained_mesh_outputs=unconstrained,
                pair_count=len(pairs), allowed_pairs=[(a,b) for a,b,r in pairs if r.allowed_reason],
                solid_validity='caller_asserted', self_intersections_checked=False,
                continuous_motion_checked=False)


def inspect(mesh,*,region=None):
    """Cheap targeted feedback; does not claim collision or self-intersection checks."""
    vertices=mesh.vertices
    if region is not None:
        key='region:'+region
        if key not in mesh.attributes:raise KeyError(f'Unknown region: {region}')
        vertices=vertices[mesh.attributes[key].values>0]
    return dict(vertices=len(vertices),faces=len(mesh.faces) if region is None else None,
                bounds=None if not len(vertices) else [vertices.min(axis=0).tolist(),vertices.max(axis=0).tolist()],
                finite=bool(np.isfinite(vertices).all()),self_intersections_checked=False,
                region=region,topology='validated when Mesh constructed')
