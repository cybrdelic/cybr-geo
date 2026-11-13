# Curve and patch extension

This is an extension to the published CYBR GEO core, not a replacement asset library.

- `bezier(control, t)` evaluates arbitrary-degree Bézier control polygons with de Casteljau.
- `sample_curve(curve, count, tolerance, max_depth)` adaptively flattens a callable curve and distributes output by approximate arc length. It checks quarter points and midpoint and rejects unresolved depth limits. A finite sample check cannot guarantee error for every arbitrary function.
- `resample_polyline(points, count, closed=False)` resamples open paths or closed profile boundaries. Closed samples omit a duplicate endpoint.
- `transport_frames(path, initial_normal=None, twist=0)` returns tangent/normal/binormal arrays. Discrete parallel transport minimizes incremental frame rotation. Scalar twist is total radians distributed by arc length; an array explicitly controls angle at every station. Ambiguous reversals are rejected.
- `sweep(profile, path, scale=1, twist=0, initial_normal=None, caps=True)` supports variable isotropic/anisotropic scale and controlled twist. End caps use the existing core's ear clipping, not triangle fans.
- `sweep_shell(outer, inner, path, ...)` retains both walls and connects annular end rims. This is real closed shell topology with open bore, not a capped solid cylinder.
- `matched_loft(sections, samples, align=True, caps=True)` resamples differently sampled profiles, and optionally optimizes cyclic shift and winding correspondence. Turn off automatic matching for semantically pinned seams; this is not feature recognition.
- `coons_patch(bottom, top, left, right, nu, nv)` interpolates four compatible boundary curves and preserves their sampled positions. It checks corner agreement and emits UV coordinates.

## Limits, not hidden claims

Constructors validate connectivity through `Mesh`. They do not detect arbitrary self-intersections, ensure profile nesting, solve mechanical loads, certify flow paths, or guarantee constant physical wall thickness under curvature/taper. Patches give C0 boundary position interpolation, not G1 tangent stitching, and are not NURBS. Frames are for open paths; closed-loop holonomy correction is not implemented. No unrelated solver is mislabeled as a geometry feature.

`examples/construction.py` is the complete 21-line authored assembly. Its resulting 20 components are individually coherent closed shells, not one welded assembly. Reinforcement collars intentionally overlap their receiving wall to represent attachment; assembly-level connection physics is not claimed. The video shows every authored line, actual construction inputs, and progressively swept geometry. It is explanatory playback, not physical time. Library implementation is included in `cybrgeo/curves.py` but is not expanded line-by-line in this example video.

## Verification

```
PYTHONPATH=. python -m unittest discover -s tests -v
PYTHONPATH=. python examples/swept_exhaust.py
PYTHONPATH=. python animate_construction.py
```

Nine tests cover Bézier evaluation, arc-length spacing, frame orthogonality and continuity, total twist, closed sweep topology, shell signed volume and winding, loft alignment, exact patch boundaries, and rejected malformed input.
