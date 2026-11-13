# Achieved edge-size refinement

`refine(mesh, max_edge)` remains one lineage-preserving conforming pass.
`refine_to_size(mesh, max_edge)` repeats that operation until every output edge
meets the requested limit. It fails explicitly on a pass or vertex budget.

```python
from cybrgeo import extrude, refine_to_size, sculpt
blank = extrude(outline, height=3, holes=holes)
dense = refine_to_size(blank, max_edge=3)
plate = sculpt(dense, displacement)
```

This is a simple generic way to prepare an arbitrary perforated profile for a
smoothly varying displacement. **Refinement itself does not add curvature, round
a block, correct poor source profiles, or repair intersections.** The authored
`displacement` must specify the intended shape, and source loops still have to
be valid. After deformation the edge-size bound may no longer hold.

The bound applies to mesh edge lengths, not deviation from an analytic surface.
This is not isotropic remeshing. It preserves the underlying piecewise-linear
surface and supported attributes. The return value is a Mesh, not a composed
persistent-reference correspondence. Every pass still scans/copies the mesh.

Defaults: 12 passes, 1,000,000 vertices. Existing valid meshes below the bound
return by identity. Limits are checked before refinement allocation.

For perforated sheets, `extrude(outline, height, holes=holes, spacing=5)` now seeds
cap interiors before triangulation. This reduces skinny boundary-only fans.
`spacing` is a sampling spacing, not a strict edge bound; call `refine_to_size`
when a strict bound is required. Invalid crossing/touching rings, holes outside
the outer ring, and nested/overlapping holes now fail before triangulation.

The analytic `Solid.extrude(outline, height, offset=-2)` can inset a planar
profile by a distance using OpenCascade's existing offset operator. This is not
uniform scaling: concave and convex corner treatment is delegated to the CAD
kernel and offset collapse fails. CYBR GEO does not claim to implement that
kernel itself.
