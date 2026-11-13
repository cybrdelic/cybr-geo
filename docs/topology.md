# Editable topology, without an object catalogue

`Editable` wraps an immutable `Mesh`. Each operation returns a new evaluated
value. It does not introduce special motor, tree or building classes.

```python
import numpy as np
from cybrgeo import extrude, Editable

# Existing general profile extrusion creates a closed block.
base = extrude([(-1,-1), (1,-1), (1,1), (-1,1)], 2)
model = Editable(base)
top = np.where(np.all(base.vertices[base.faces][...,2] == 2, axis=1))[0]
cap_reference = model.ref('face', int(top[0]))
model = model.inset_faces(top, .25)  # real annular rim, not overlapping planes
model = model.extrude_faces(top, [0,0,1])  # retain complete existing body
cap_descendants = model.descendants(cap_reference)
result = model.mesh
```

The cap face indices remain in their original positions in these two operations;
added wall triangles follow them. References are preferable if later edits may
replace/reorder faces. `resolve(ref)` raises when the exact feature was replaced;
`descendants(ref)` deliberately returns a potentially empty or one-to-many tuple.
A face can have both cap and wall descendants. This is provenance, not semantic
recognition of which new face is "the top".

## Operations

- `split_edge(a,b,t=.5)` splits all incident triangles consistently.
- `extrude_faces(indices, offset)` duplicates a patch and closes its boundary
  with side triangles. Offset is a single vector, not a face-normal field.
- `inset_faces(indices,fraction)` scales toward the selected patch's centroid
  and constructs its rim. This is **not constant-distance inset**, particularly
  on concave, disconnected or nonplanar patches.
- `boundaries()` returns oriented boundary loops.
- `bridge(loop_a,loop_b)` stitches distinct equal-length boundary loops. Inputs
  follow their existing face windings; the second loop is reversed internally,
  retaining its first vertex as correspondence anchor. New face/corner attributes
  have no automatic source: currently such attributed bridges reject explicitly.
- `weld([(a,b),...], tolerance=1e-8)` explicitly merges coincident paired vertices,
  dropping collapsed triangles. It keeps the first position and averages source
  vertex attributes; invalid resulting topology raises. This is not global
  distance-based welding or an intersection Boolean.

All operations run normal `Mesh` validation. They do not prove geometric
nonintersection, dimensional fit, wall thickness, physical correctness or
structural safety. An operation may reject degenerate/nonmanifold outcomes.

## Attribute contracts

`Attribute` accepts four domains:

- `vertex`: one entry per vertex;
- `face`: one entry per triangle;
- `edge`: one per `mesh_edges(mesh.faces)`, sorted undirected vertex pairs;
- `corner`: flattened face-major order, index `3*face + local_corner`.

Vertex and corner interpolation honors `linear`/`nearest`; face values inherit
from their source triangle. Corner transfer is per source face and therefore
does not weld UV seams. Side walls inherit source corners; usable UV layout is
not automatically generated. New edges with no source require explicit defaults,
e.g. `model.extrude_faces(top,[0,0,1],edge_defaults={'crease':0})`.
Unsupported new attributes fail rather than disappearing silently.
Snapshot save/load supports all four domains. Editable reference identity and
lineage persist **through an in-memory edit chain**, not through plain Mesh
snapshot serialization, remeshing, external geometry or unrelated operations.
No correspondence is invented across those boundaries.
