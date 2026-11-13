# Surface attachments without nearest-face guessing

`Feature.at(anchor)` reuses `SurfaceAnchor` and the same `Feature`/`Graph.fit`
interface. It resolves an explicit barycentric point on any valid triangle mesh,
including a curved mesh, into a rigid frame. No subject-specific attachment type
or parallel scene graph is introduced.

```python
anchor = SurfaceAnchor.on(surface, face=42, barycentric=(0.2, 0.3, 0.5))
contact = Feature.at(anchor, x_hint=(1, 0, 0), offset=0.1)
frame = contact.resolve(surface)

g.fit('detail', make_detail, 'surface',
      target=contact, source=Feature('base'),
      assume_valid_solid=True)
```

This snippet assumes an existing surface, graph host, and factory. The factory
receives the resolved target frame as usual. The frame has no radius because a
triangle point does not imply a circular opening or clearance corridor.

## Precisely what follows an edit

The anchor remains **snapshot-bound**. Changing the mesh does not authorize reuse
of the same face number. A stale reference fails before constructing the dependent
part. To preserve it, supply the existing explicit correspondence:

```python
mapping = Correspondence.deformation(old_surface, edited_surface)
updated_contact = contact.follow(mapping)
frame = updated_contact.resolve(edited_surface)
```

Deformation correspondence assumes actual vertex identity is preserved. For a
refinement, use `Correspondence.refinement(old_surface, refinement)`. Arbitrary
remeshing needs explicit supplied lineage or the existing bounded projection
workflow, whose result remains conditional. No closest-surface fallback is hidden
inside the attachment. A graph whose target is a fixed snapshot-bound feature
will reject host edits until the author updates that reference or explicitly
resolves correspondence in a dependency operation.

## Geometry and limits

- Position is the exact barycentric point on the **piecewise-linear** triangle.
- Normal is that triangle's actual oriented normal, not a smoothed or analytic
  surface normal. It can jump between triangles at a crease.
- Phase is the projection of the explicit world-space `x_hint` into the tangent
  plane. If it is parallel to the normal, resolution fails instead of guessing.
  Following correspondence preserves that hint, not a material tangent field.
- Offset moves along the normal. It does not establish a safe insertion depth,
  conform an entire patch/footprint, certify contact, or guarantee clearance.
- SurfaceAnchor deserialization now rejects nonfinite barycentrics, noninteger or
  negative face indices, and empty snapshot identity. Point evaluation rejects
  an out-of-range face even if a supplied snapshot hash matches.

The existing circular and planar named features retain their behavior. No new
root export is required. Tests cover curved placement, refinement and deformation
lineage, missing/stale references, graph failure rollback, and invalid anchors.

## Rebuild the reference as a dependency

A Feature-producing graph operation can be supplied by name to the existing
`target` argument. This keeps explicit lineage inside the dependency graph:

```python
g.op('contact', lambda edited: contact.follow(
    Correspondence.deformation(original, edited)), 'host')
g.fit('detail', make_detail, 'host', target='contact',
      source=Feature('base'), assume_valid_solid=True)
```

A host edit now rebuilds `contact` and `detail`; invalid correspondence still
fails. This example requires vertex identity preserved by the deformation. A
remesher must provide real correspondence instead, not reuse this declaration.
The host can be an intermediate open surface; only registered closed solids
participate in solid-interference checking. Attachment alone does not imply
whole-footprint contact or assembly validation.
