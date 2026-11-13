# Mesh LOD: explicit reduction, budget, and correspondence

```python
from cybrgeo import simplify, SurfaceAnchor

anchor = SurfaceAnchor.on(mesh, 12, (.2, .3, .5))
coarse = simplify(mesh, target_faces=300, max_error=.002,
                  preserve_boundary=True, protected_vertices=[7, 8])
print(coarse.reached_target, coarse.sampled_error)
new_anchor = coarse.rebind(anchor, mesh, max_distance=.003)
```

`max_error` uses mesh coordinate units. It is required, finite, and nonnegative.
The method can stop before reaching the requested face count. Check
`reached_target` instead of treating the target as a promise.

## What happens

1. Original triangle planes form per-vertex quadric error matrices.
2. Adjacent vertex pairs propose collapses, ordered by quadric error. The
   minimizer lies on the edge segment, making attribute interpolation explicit.
3. Boundary and protected vertices cannot participate in collapses.
4. Surviving triangle normal reversals and invalid topology are rejected.
5. Each candidate is compared against the original surface, not just its prior
   level. Samples comprise all vertices, face centroids, and edge midpoints.
   Both directions are checked against continuous triangles using the BVH.
6. Only candidates within the sampled budget (plus 1e-12 numeric slack) survive.

Output includes actual reduction count, measured error, surviving source-face
indices, and source-vertex ancestry groups. Vertex attributes interpolate;
nearest categories pick one endpoint. Face values follow surviving faces.
Corner/edge attributes are rejected without a seam policy, not discarded.

## Deliberate limits

**The sampled metric is not a guaranteed Hausdorff bound.** It can miss a
between-sample deviation. The QEM objective is an ordering heuristic, not the
error guarantee. No volume, silhouette, curvature, or UV-distortion guarantee
is implied. Manifold checks and local normal tests do not establish absence of
all nonlocal self-intersections.

This implementation is for moderate meshes, not real-time million-face LOD.
It recomputes candidates and checks full mesh deviation for each tentative
collapse. Boundary preservation is conservative: it pins all boundary vertices,
which can prevent reduction. There is no seam-aware material/UV optimization,
automatic quad retopology, progressive-mesh encoder, or GPU path.

Rebinding uses explicit nearest-surface projection with its own required
finite distance limit. It preserves a nearby surface location, **not exact
semantic/material identity** after topology changes. The original snapshot
must match the supplied anchor; unrelated snapshots are rejected.
