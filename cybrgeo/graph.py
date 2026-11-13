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
    """Named inputs and pure operations. Failed builds preserve last good cache."""
    def __init__(self):
        self._inputs={};self._ops={};self._cache={};self._revision={};self.last=None

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
        old,deps,version=self._ops[name]
        if not callable(function):raise TypeError('Operation must be callable')
        self._ops[name]=(function,deps,version+1)

    def set(self,name,value):
        if name not in self._inputs:raise KeyError(f'Not an input: {name}')
        fingerprint=_fingerprint(value)
        if fingerprint!=self._inputs[name][0]:
            self._inputs[name]=(fingerprint,_freeze(value));self._revision[name]+=1
        return self

    def build(self,targets=None,*,force=False):
        targets=list(self._ops) if targets is None else ([targets] if isinstance(targets,str) else list(targets))
        start=perf_counter();staged={};versions={};seen={}
        report=dict(evaluated=[],reused=[],inputs=[],seconds_by_node={},invalidated=[])
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
        self._cache.update(staged)
        report['seconds']=perf_counter()-start;report['committed']=True
        result=Build(MappingProxyType(outputs),report);self.last=result;return result


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
