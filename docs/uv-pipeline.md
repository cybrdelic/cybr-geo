# CYBR GEO: explicit UV charts and baking

New files: `cybrgeo/uv.py`, `tests/test_uv.py`, `examples/uv_demo.py`, `examples/render_uv.py`.

```python
charts = unwrap(mesh, seams=[(vertex_a, vertex_b), ...])
atlas = pack(mesh, charts, padding=0.04)
texture, coverage = bake(atlas, 'color', resolution=256)
```

`unwrap` cuts face corners along specified physical edges. Separate charts can
share physical vertices but retain independent UV coordinates. Each chart must
be a disk. Its boundary maps to a convex circle proportional to boundary edge
length. Interior vertices solve a sparse uniform graph Laplacian. Degenerate
or flipped triangles are rejected. Closed charts and multiple boundary loops
require additional user seams, not silent approximate projection.

`pack` uses deliberately conservative horizontal rectangle packing. Charts
have a common UV scale with a guaranteed bounding-box gap. This is not optimal
packing, and does not equalize texel density relative to physical surface area.
It returns a new Mesh with an actual corner-domain `uv` attribute, usable by
existing OBJ export. Physical vertex positions/topology remain unchanged.

`bake` rasterizes existing vertex attributes via barycentric interpolation at
pixel centers. It outputs the numeric texture plus coverage mask. It rejects
sampled overlapping interiors; this pixel test is NOT an exact continuous
triangle-overlap theorem for arbitrary externally supplied UVs. The unwrap's
convex harmonic charts and disjoint packing avoid such overlaps for this path.
No texture dilation, AA, normal-space baking, raycasting, or imported texture
transfer is claimed.

This implements a useful subset, not the entire production UV/retopology gap.
There is NO automatic animation-friendly quad topology, automatic seam finding,
seam-preserving simplification, conformal distortion optimization or paint UI.
The simplifier's existing rejection of corner attributes remains intentional.

Video: actual curved source shell, two cut charts continuously interpolated into
their actual packed UV coordinates, then actual baked color atlas shown alongside.
The unfold is explanatory interpolation, not physical material deformation.
The source listing is displayed with long lines abbreviated; complete runnable source is bundled. Imports and data setup are shown as code, NOT
misrepresented as complete per-line geometric animation. This focused pipeline
video is not yet the user's full every-line semantic tracer requirement.

Reproduce from this directory:

    PYTHONPATH=. python examples/render_uv.py
    python -m unittest discover -s tests

Requirements: existing CYBR GEO dependencies, numpy, scipy, matplotlib, ffmpeg.
