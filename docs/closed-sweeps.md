# Continuous closed sweeps

Use the existing `sweep` operator for a rim, loop, seam, cable or collar rather
than joining two coincident end rings:

```python
import numpy as np
from cybrgeo import sweep

u = np.linspace(0, 2*np.pi, 100, endpoint=False)
path = np.column_stack((3*np.cos(u), 2*np.sin(u), .2*np.sin(2*u)))
a = np.linspace(0, 2*np.pi, 20, endpoint=False)
profile = .15*np.column_stack((np.cos(a), np.sin(a)))
rim = sweep(profile, path, closed=True)
```

The path and profile do not repeat their starting point. The final ring is
connected to the first using the same vertices, so there is no coincident
boundary or additional seam weld. Periodic tangent frames include the closure
edge. Residual transport rotation (holonomy) is distributed over the entire
loop by arc length, not concentrated in a last-ring jump. The resulting mesh
still approximates the authored smooth path with straight edges; this is a
periodic discrete frame, not an analytic C1 surface claim.

`caps` now defaults to `None`: open paths retain the previous capped behavior;
closed paths have no caps. Passing `caps=True` with a closed path is an error.
All earlier positional calls remain accepted. `transport_frames` also accepts
`closed=True` for callers constructing their own sections.

Closed-loop `twist` is a scalar whole number of turns in radians, for example
`2*np.pi`. Fractional turns and per-point arrays are rejected because this API
cannot infer a compatible seam for an arbitrary profile. Positive ring scales
remain supported. Only the existing `section_coordinate` vertex attribute is
produced; profile seam UVs are not synthesized.

## Construction limits

A continuous frame and closed manifold mesh do **not** prove a valid solid:
a thick profile can intersect itself on a tight bend, the authored path can
cross itself, or an excessive full-turn twist can fold the surface. These are
not silently repaired or certified by this operation.

`sweep_shell` now validates nested simple 2D boundary rings before construction
and rejects touching/crossing/outside inner profiles. It preserves authored
profile indexing and requires matching winding. It still does not certify
wall thickness or nonintersection after a curved sweep.

The rotation helper now copies a caller-supplied axis before normalization, so
using an axis array does not mutate the caller's geometry inputs.
