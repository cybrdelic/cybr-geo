# Surface identity is a contract, not nearest distance

`SurfaceAnchor` identifies one triangle coordinate in one immutable mesh
snapshot. `Correspondence` carries explicit triangle ancestry between snapshots.
`Resolution` adds a serializable history, method, confidence class, and distance.

```python
anchor = SurfaceAnchor.on(mesh, face=4, barycentric=(.2, .3, .5))
refined = refine(mesh, max_edge=.1)
tracked = Correspondence.refinement(mesh, refined).follow(anchor)
deformed = Mesh(new_positions, refined.mesh.faces)
tracked = Correspondence.deformation(refined.mesh, deformed).follow(tracked)
```

Deformation assumes the caller preserves vertex identities. Same face indices
are necessary but cannot prove that the author didn't secretly reorder positions.
Explicit mappings use `from_lineage(source, target, parent_faces,
corner_coordinates, operation=...)`. Each target triangle's corners specify
barycentrics within its parent triangle. Unmapped faces use -1. This supports
face reorder and supplied tessellation ancestry, not arbitrary CAD edits.
Overlapping lineage that sends one point to different surfaces raises an error.

For remesh, simplification, or CAD-converted meshes WITHOUT explicit ancestry:

```python
tracked = project_reference(anchor, mesh, replacement, max_distance=.02,
                            source_regions=old_region_ids,
                            target_regions=new_region_ids)
```

Both label arrays are author-supplied face partitions with shared meanings.
Do not manufacture labels from nearest-point guesses. Without labels, every
compatible local triangle fan in the distance budget is considered; multiple
plausible fans fail closed. This catches two similarly oriented parallel sheets
even when one is closer. A connected folded surface is assessed locally rather
than assuming one global connected component means one semantic surface.

`confidence` is categorical evidence, NOT a probability. `exact-lineage` means
following an explicit material-coordinate mapping; `conditional-projection`
means bounded geometric matching. The latter never becomes exact simply because
subsequent deformation has exact lineage. Distances are geometric witness
measurements, not certified errors. Normals, triangle coordinates and tolerance
checks use ordinary floating point.

`reproject_anchor` and `LOD.rebind` now route through this fail-closed policy;
legacy code can therefore raise on ambiguity that formerly silently snapped.
Use `project_reference` directly to retain the evidence/history. Existing
`fit_surface` remains a caller-authored cage fitting utility, not a robust
identity-preserving remesher. No automatic CAD naming/trim correspondence,
universal semantic correspondence, global self-intersection proof or guaranteed
Hausdorff bound is claimed. Candidate scans are currently O(F); this is not a
million-face throughput implementation.

Run `/usr/bin/python3 -m unittest discover -s tests -v` from the source root.
