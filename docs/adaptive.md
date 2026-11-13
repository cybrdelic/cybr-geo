# Local refinement, replacement topology, and surface references

These are general operations, not named parts or scene workarounds.

```python
from cybrgeo import SurfaceAnchor, refine, fit_surface, reproject_anchor, MeshBVH

# Record a material point on a particular input triangle.
anchor = SurfaceAnchor.on(mesh, face=12, barycentric=(.2, .3, .5))

# Split long edges only around selected faces; neighbors conform automatically.
step = refine(mesh, max_edge=.05, faces=[12, 13])
anchor = step.follow(anchor)
mesh = step.mesh

# Fit deliberately authored new connectivity to an existing detailed surface.
fit = fit_surface(detailed_mesh, replacement_cage, max_distance=.02,
                  names=['color'])
new_mesh = fit.mesh

# Remeshing lacks exact ancestry. Make nearest-surface rebinding explicit.
rebound = reproject_anchor(anchor, mesh, new_mesh, max_distance=.01)
```

## Guarantees and limits

- `refine` creates a conforming triangle subdivision: every selected shared edge
  splits in all incident faces. Original vertices and unmodified triangles stay
  bit-identical. Without `evaluate`, the piecewise-linear surface is unchanged.
- Vertex and face data transfer; corner data interpolates *within its original
  face*, preserving UV/material seams. Edge data requires a policy and is rejected.
- `evaluate(new_positions)` evaluates only newly introduced vertices, not the
  whole surface. This can sample terrain height, for example. It can introduce
  geometric self-intersections; those are not automatically ruled out.
- One pass is one refinement level, not an unbounded convergence loop. It does
  not coarsen, flip edges, optimize triangle quality, or automatically estimate
  curvature error. Selected edges exceeding the limit are split once.
- `Refinement.follow` tracks exact material coordinates using child-to-parent
  barycentrics. Anchors serialize as plain dictionaries and recognize matching
  mesh snapshots after save/load. They deliberately reject unrelated snapshots.
- `fit_surface` fits **user-authored topology**, preserving that connectivity and
  its own attributes while optionally sampling source vertex data. It is not an
  automatic quad flow/retopology system and does not preserve detail finer than
  the replacement cage sampling. Subdivide/refine the cage first if needed.
- Projection has a required finite distance bound. That is a per-input-vertex
  bound, not a whole-surface Hausdorff bound or a guarantee of the correct sheet.
- `reproject_anchor` explicitly relinquishes ancestry and uses nearest surface.
  Nearby overlapping sheets may be ambiguous; partition/align inputs first.

## Downstream local update

```python
from cybrgeo import local_displace
index = MeshBVH(mesh)
edit = local_displace(mesh, [5, 6], [0, 0, .01])
updated_index = index.refit(edit.mesh)
```

Refit checks connectivity and discovers changes itself. It recalculates changed
triangle boxes and affected BVH leaf/ancestor boxes, sharing untouched branches.
It still scans/copies mesh arrays and traverses the tree. This is **not O(k)
end-to-end updating** or a GPU implementation. The partition stays unchanged;
rebuild after large deformations for better query performance. Topology edits
must rebuild. Tests compare refit query results to a fresh BVH and verify branch
reuse, closed/manifold refinement, anchors across two refinement levels and
snapshot reload, projection rejection, and UV/categorical transfer.
