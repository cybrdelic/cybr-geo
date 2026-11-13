# Organic editing additions

These are general numerical operations, not object generators or automatic art direction.

```python
from cybrgeo.organic import Multires, lattice_deform, local_displace

pyramid = Multires(mesh, levels=2)
# Sculpt a fine level without destructively changing the coarse input.
pyramid = pyramid.edit(2, fine_displacements)
# Fine object-space detail is retained when a coarse level changes.
pyramid = pyramid.edit(0, coarse_displacements)
mesh = pyramid.mesh()

# Arbitrary regular control lattice; NxMxK control vectors, not a prefab shape.
mesh = lattice_deform(mesh, bounds, control_offsets)
# This reports exactly which existing vertices/faces changed.
edit = local_displace(mesh, selected_vertex_indices, displacement_vectors)
```

`crease_subdivide` adds explicit permanent hard edges to Loop subdivision. Edges are vertex-index pairs, survive each refinement and are validated against current topology. Boundary edges are treated as sharp. Three-edge corners stay fixed. Vertex linear data uses the geometric stencil and face data is replicated. Unsupported corner/edge attributes fail explicitly.

`transfer_attributes` projects to source triangles using `MeshBVH`, interpolates linear vertex data barycentrically, selects the nearest triangle corner for nearest data, and samples face data at target face centroids. A finite `max_distance` prevents transfer across distant surfaces. Attribute transfer does not create a new topology or establish semantic correspondence between nearby separate surfaces.

## Limits

- Multires stores object-space residuals. It preserves additive detail, not rotating tangent-space detail; apply it before skinning.
- Lattice deformation is piecewise trilinear displacement, not smooth B-spline FFD or volume-preserving deformation. The mesh must resolve lattice variation for visual accuracy. Zero boundary controls avoid discontinuity with points outside the box.
- `local_displace` limits numerical edits and returns affected indices. Mesh copying and validation still operate globally; no claim of automatic local remeshing or sublinear total runtime.
- No remesher, auto-retopology, simplifier, UV-seam transfer, collision prevention or material/physical validity is added here.
- Hard, not fractional/semi-sharp, crease weights.

## Verification

Six tests cover hard-crease propagation, exact unchanged vertices/topology, affine lattice deformation and attribute preservation, retained high-resolution detail under coarse translation, barycentric affine attribute reproduction and transfer distance rejection, and malformed input rejection. All 43 tests (37 baseline + 6 additions) passed on this working copy.
