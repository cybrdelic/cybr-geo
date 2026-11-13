# Six-area integration candidate

All six additions are integrated into one package, but **the complete six original gaps are not solved**. Existing docs are retained; this page is the current scope summary for these additions.

| Area | Implemented | Still missing / bounds |
|---|---|---|
| Geometry-bound relationships | Labeled real mesh circle/plane features derive frames and radii; existing Graph rebuilds fitted geometry and rigid mates. Invalid labels fail with rollback. | Producer must preserve labels; not universal CAD naming or a closed-loop assembly constraint solver. |
| Solid interference | Closed-shell containment (including cavities), conservative overlap/contact, clearance witnesses, allowed-intersection reasons, selected-face thickness chords, discrete motion samples. | Requires explicitly valid solids; no exact penetration depth, global minimum thickness, continuous collision or structural certification. |
| Correspondence | Explicit triangle lineage through refinement/deformation/reordering; conditional projection rejects nearby-sheet ambiguity; old rebind adapters use fail-closed policy. | No automatic semantic mapping through arbitrary remesh or CAD conversion; labels/correspondence supplied by author when needed. |
| Surface design | Tensor Bezier patches, rectangular parameter trims, matching-edge C0/C1/C2 constraints, welded tessellation, sampled continuity/offset/intersection diagnostics. | No arbitrary trim loops, exact NURBS surface kernel, general sewing or global continuous self-intersection proof. |
| UV | Explicit seam cuts, disk-chart unwrap, padded packing, corner UVs, vertex-attribute bake with overlap rejection. | No automatic animation-ready quads, complete production baking, seam-preserving QEM simplification. LOD intentionally rejects unsupported seam-bearing data. |
| Local updates | Persistent tiled heightfield edits, shared unmodified tiles, cached bounds, exact vertical deviation for nested sampled PL heightfield LOD. | Specialization, not arbitrary-mesh acceleration. Full mesh/materialization remains O(N). Mesh QEM error remains sampled. |

## Validation

`python -m unittest discover -s tests` passes **140 tests** in this combined candidate with CadQuery installed. This includes five additional integration tests: patch/UV/refinement, tiled geometry/reference lineage, mates/interference before and after edits, CAD conversion rejecting absent feature identity, and a simulated missing-CadQuery path that preserves non-CAD use.

CAD operations are supplied by CadQuery/OpenCascade, not a newly authored analytic kernel. New solid diagnostics operate on tessellated meshes. Tests exercise supported cases and failure modes; they are not proof of production completeness.

## New source examples

- `examples/bound_cartridge.py`: geometry-derived bearing seat and dependent shaft.
- `examples/solid_interference.py`: containment-aware clearance and motion samples.
- `examples/reference_lineage.py`: exact lineage, ambiguous sheets and explicit regions.
- `examples/patch_body.py`: trim, continuity, stitch and offset.
- `examples/uv_demo.py`: seam-based chart construction, atlas and bake.
- `examples/local_terrain_edits.py`: persistent terrain cut, bench and embankment.

New videos are delivered separately; these source examples use the integrated core. Some historical rendering/benchmark recipes retain their original workspace layouts; see source imports before running those support scripts. No remote push was made by this integration task.
