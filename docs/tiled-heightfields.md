# CYBR GEO: local terrain edits and bounded heightfield LOD

This addition is deliberately small: `cybrgeo/tiled.py`, one specialized geometry
representation, not a second scene graph or an arbitrary-mesh replacement.

## New public interface

```python
from cybrgeo.tiled import TiledHeightfield, height_lod

terrain = TiledHeightfield.sample(
    lambda x, y: 3 * np.sin(x / 20),
    tiles=(64, 64), cells=32, spacing=1.0,
)
changed, cost = terrain.edit(
    (997, 997, 1013, 1013),
    lambda x, y, z: z + 2,
)
lod = height_lod(changed.heights(), stride=4, budget=0.25)
```

An inclusive sample rectangle, not a whole mesh, defines edit scope. A callback
receives world x/y and the old heights for the unique edited samples. It returns
replacement heights. All tiles owning a seam sample receive the same value.
Untouched tile arrays and tree branches are shared with the old snapshot. Height
bounds are cached and updated only along changed branches. Invalid nonfinite or
wrong-shaped edits raise before a new snapshot is returned. Sampling callbacks
should be pure, deterministic functions.

`tile(x,y)` gives a read-only tile view. `bounds` uses cached extrema.
`heights()` explicitly materializes the full array; `mesh()` does that and performs
ordinary Mesh validation. Those two operations remain O(total geometry). Local
updates are O(edited samples + dirty tiles * tile area + dirty tiles * log tiles).
The tile tree stores all existing tiles, not an unbounded sparse world.

## Measured benchmark (this machine, not a GPU claim)

See `benchmark.json`. A 2049 by 2049 sample terrain, 4096 tiles, 32 cells per tile:
- 17 by 17 sample edit: median **0.055 ms** over 15 trials.
- Complete regeneration: median **391 ms** over 3 trials.
- One changed tile, 13 new tree nodes, 8,712 height-array bytes copied.
- 289 unique callback samples evaluated, versus 4,198,401 unique full samples.
- Edited heights matched full regeneration exactly.

These timings compare sample update and regeneration only, not rendering,
export, arbitrary topology edits, total process memory, or end-to-end application
speed. The reported copied bytes count persistent height arrays, not callback
scratch arrays or Python object allocations. Re-run `benchmark.py` for your PC.

## Error guarantee, precisely scoped

`height_lod` evaluates deviation from the original piecewise-linear sampled
heightfield, not from the original analytic terrain function. Fine and coarse
grids must cover the same domain and split cells using the same lower-left to
upper-right diagonal. The coarse grid is nested; its stride divides both cell
counts. Every coarse triangle is a union of fine triangles. Surface difference
is affine within each fine triangle, hence its absolute maximum occurs at a fine
vertex. Testing every original vertex therefore gives the maximum vertical
surface deviation, subject to floating-point arithmetic. This also supplies an
upper bound on Hausdorff distance, but not the exact Hausdorff distance.

`within_budget` reports acceptance; it does not silently claim an over-budget
result is valid. The returned surface is still available for inspection. This is
not a normal, curvature, silhouette, thickness, structural or collision guarantee.
It does not change the existing arbitrary mesh simplifier's **sampled** error
estimate. Independent neighboring tile LODs would require seam stitching, which
is not implemented. The demo applies LOD to the whole common domain.

## Video

`Local_Terrain_Edit_Construction.mp4` runs all 14 executable lines of `site.py`,
including imports. It forms a winding cut, a level construction bench and an
embankment in one persistent terrain. Changed regions move through actual
before/after heights; amber highlights edited samples. Nonspatial imports and
function definitions explicitly leave geometry unchanged instead of inventing
fake effects. Coarse and fine LODs visibly morph the surface, with error/budget
readouts. These are explanatory interpolations, not an erosion/soil simulation.
The source has no hidden asset class or model catalogue. Rendering uses real
GEO sample output and an independent triangle rasterizer, not image generation.

To reproduce from this package, install NumPy and Pillow, provide ffmpeg and g++,
then run:

```sh
g++ -O3 -shared -fPIC geo_motor/raster.cpp -o geo_motor/raster.so
python geo_large_edits_demo/render.py
python geo_large_edits_demo/benchmark.py
PYTHONPATH=geo_large_edits_work python -m unittest discover -s geo_large_edits_work/tests
```

The supplied test report is 97 passing tests, including six new focused tests:
local versus full reconstruction; seam/corner/outer edge edits; failed-edit
rollback and read-only views; exact PL error/budget cases; full mesh export; edit allocation scaling with tree depth.
