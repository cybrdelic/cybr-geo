# Incremental construction and feedback

This experimental version preserves the previous core and adds explicit dependency caching. It is not a GPU engine, automatic geometric partitioner, production kernel, or realism generator.

## Actual interface

```python
import cybrgeo as geo
import numpy as np

g = geo.Graph()
g.input('left.height', 0.1)
g.input('right.height', 0.2)
g.op('left', lambda h: geo.heightfield([0,1], [0,1], np.full((2,2),h)), 'left.height')
g.op('right', lambda h: geo.heightfield([1,2], [0,1], np.full((2,2),h)), 'right.height')
first = g.build(['left','right'])
g.set('left.height', 0.3)
edited = g.build(['left','right'])
assert edited.report['evaluated'] == ['left']
assert edited.report['reused'] == ['right']
print(geo.inspect(edited.outputs['left']))
```

Operations are pure functions: all changing inputs must be declared dependencies, not mutable globals. Their returned values must be immutable (core Mesh is immutable). Named regions can have distinct input nodes and construction branches; this provides regional invalidation only where the author defines those dependencies. It does **not** infer how to partially remesh a monolithic mesh, automatically split a sculpture, or stitch independent tile boundaries. Shared boundary inputs must be declared in every dependent patch.

`Graph.input(name,value)` captures copied data. `Graph.set` only revises the input when its content changes. `Graph.op(name,callable,*dependencies)` declares a node; dependencies must already exist, preventing cycles. `Graph.replace(name,callable)` changes an operation and invalidates its consumers. `Graph.build(targets,force=True)` recomputes the selected graph for comparison.

Build results contain output references plus evaluated/reused node names and per-node wall-clock times. An operation failure raises BuildError with node, cause and partial report; candidate cache entries are discarded, and `graph.last` remains the previous successful snapshot. Pending inputs remain available for correction. Dependencies must not mutate other output objects or external state, which cannot be rolled back.

## Geometry checks

The original core repeated all connectivity analysis after every same-topology deformation and region assignment. The new core caches exact connectivity reports keyed by vertex count and face bytes. Vertex area/degeneracy tests still run on every newly constructed Mesh. A collapse test proves topology reuse does not hide collapsed faces. No collision or self-intersection detection is added.

`geo.inspect(mesh,region=...)` gives inexpensive finite-state/bounds/count diagnostics and explicitly discloses absent self-intersection checking. It does not silently replace expensive quality inspection or render analysis.

## Measured results

Run `PYTHONPATH=. python benchmarks/edit.py`. Nine independent 55×55 terrain patches (27,225 vertices, 52,488 triangles) were built; editing one patch reused eight. On this machine, seven repeats gave median 1.72 ms incremental versus 14.24 ms full recomputation: 8.30× for this workload. Full and incremental vertices/faces were bitwise equal. No-op median 0.024 ms. These are CPU construction times, not rendering or agent completion times.

Separate `benchmarks/topology.py` comparison on the original versus new package: same-topology sculpt of 10,000 vertices, 19,602 triangles took 156.1 ms versus 2.31 ms median. This narrow improvement avoids repeated Python topology analysis; it is **not** a claim that arbitrary geometry generation is 67× faster. Initial topology, changed connectivity and deformation degeneracy checks still cost work. Results are machine-dependent; JSON data included.

## Verification

`PYTHONPATH=. python -m unittest discover -s tests -q`: 37 tests pass including eight new cache/feedback tests. No-op reuse, selective dependencies, exact full rebuild equivalence, input copies, operation replacement, missing-region diagnostics and failed build preservation are covered.
