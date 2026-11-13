# Expanded core candidate, 13 November 2026

This is a tested local integration candidate, **not a GitHub push**. It extends
the published small core without deleting its prior operations.

Run `python -m pip install -e '.[profiles,solid]'`, then
`python -m unittest discover -s tests -v`.
Observed integration result: **91 tests pass**. These are focused correctness
checks, not evidence of production parity with a commercial DCC.

| Area | Implemented | Still not implemented |
|---|---|---|
| Topology | Edge split, patch inset/extrusion, equal-loop bridge, explicit weld, edge/corner domains, in-memory references and lineage | General retopology, durable semantic IDs across arbitrary operations, constant-distance inset |
| Solids | Optional analytic CadQuery Boolean, shell, fillet/chamfer and mesh handoff | Arbitrary imported-mesh Boolean, universal tolerance guarantees, automatic material transfer |
| Curves | Bézier, adaptive sampling, arc-length redistribution, transported sweeps, matched loft, Coons patch, hollow sweep | General NURBS trimming, G1/G2 stitched patch network, universal self-intersection prevention |
| Spatial | Mesh BVH, closest point, ray, overlap/clearance queries and witnesses | Complete construction constraint/mate solver, robust arbitrary engineering certification |
| Organic/detail | Hard-crease refinement, lattice, local displacement, multires detail, bounded projection-based attribute transfer | Automatic anatomy, production skinning/retopology, seam-safe arbitrary attribute transfer, error-bounded LOD |
| Incremental | Local conforming edge refinement, new-sample-only evaluation, BVH box refit/branch reuse, local edit footprints, operation graph cache | Coarsening, error-estimator-driven adaptation, fully local dependency evaluation and O(k) end-to-end processing |
| Correspondence | Serializable snapshot-bound surface anchors, exact refinement ancestry, distance-limited replacement-topology fitting and explicit nearest rebinding | Automatic quad retopology, guaranteed semantic correspondence across arbitrary remeshing, whole-surface projection error bounds |

Each module documents its contracts and limitations. Geometry remains generated
by executable CYBR GEO operations; previews are rasterized geometry, not generated
images. Renderer animation interpolates an explanatory construction operation,
not a dynamics simulation or solver convergence history.

The new topology example's 25 authored lines are all represented in its video.
Import, selection and reference lines are illustrated as guides/highlights in the
3D viewport rather than falsely claimed to create geometry. Its final scalloped
surface is a demonstration of general editing, not a photorealistic asset.

The ZIP includes source/tests/docs and previously published example media.
The new topology video is delivered separately to keep downloads small.

## Added mesh LOD

Quadric-error edge collapse with two-way sampled source-deviation rejection, protected/boundary vertices, attribute transfer and explicit reference rebinding is now implemented. See [LOD limits](lod.md). This is NOT a guaranteed Hausdorff/error-bounded LOD system; finite samples can miss between-sample deviations.
