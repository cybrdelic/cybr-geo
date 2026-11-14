# Surface construction contracts

`BezierPatch` remains the geometric value. `SurfaceNetwork` is the immutable,
validated construction value. `Seam` declares the intended connection. Raw Mesh
and independent patches remain available for intentionally separate surfaces.

```python
net = fit_network(
    {'hood': hood, 'shoulder': shoulder},
    [Seam('hood', 'u1', 'shoulder', 'v0', order=1, reverse=True)],
    fixed=['hood'], fairness=0.01,
)
mesh = trim_patch(net.patches['shoulder'], outline_uv, holes=[window_uv])
net = net.with_patch('hood', edited_hood, refit=True, fixed=['hood'])
```

- All four edges are supported, including reversed edge orientation.
- Orders 0/1/2 constrain position / first / second parametric derivatives.
- `scale` expresses a positive constant cross-boundary parameter scale.
- Edge degrees must match; degree elevation is not silently invented.
- Global constrained least squares preserves fixed control nets and enforces all
  seams simultaneously. Incompatible fixed constraints fail. Control-net
  second-difference fairing is optional, not a claim of exact curvature minimization.
- Boundary contracts are checked before returning construction state. An invalid
  edit never mutates or replaces the prior state.
- Simple arbitrary polygon UV outlines/holes retain exact polygon boundaries in
  parameter space. Curved trim paths must be sampled by callers. The existing
  profile triangulator rejects unrecovered boundary constraints.
- `meshes` returns separate patch meshes, not automatically welded closed solids.
- Trimmed-away joins are not tracked by SurfaceNetwork; trim and assembly topology
  are separate. Do not claim a network has an opening/solid it has not constructed.
- No automatic automotive design, analytic NURBS trim intersections, arbitrary G2
  reparameterization, aspect-ratio guarantee, global injectivity, or collision
  certification is provided. Seam regularity checks are sampled.
- Dense control-net solve is intended for small design networks, not millions of
  vertices. Tessellation remains separate from control resolution.

## Joined tessellation and normals

`tessellate_network(net, samples=49)` samples all full patch edges consistently,
welds declared seams and propagates winding. `stitch_network_meshes(net, meshes)`
does the same for supplied UV-tagged patch meshes (including trims), but requires
matching retained edge segments and sample parameters. It rejects mismatches
instead of silently bridging an opening. It verifies actual vertices against
source patch evaluation before welding.

Both return `normal` and `uv` **corner** attributes plus `patch_index` face IDs.
Analytic normals preserve sharp C0 joins and avoid independent triangle-averaged
normal errors at smooth joins. Renderers must consume these attributes rather
than discard them. `trim_patch` now supplies analytic vertex normals.

This does not generate a network-wide trim across charts, automatically
synchronize arbitrarily different trim samples, or shell the result. Independent
thickening of each patch is still not a joined shell. Use the joined surface as
input to the intended shelling/backend operation; do not claim manifold solid
properties for the returned open design surface.
