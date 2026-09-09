# TORSEN-X — reference-led geometry rebuild

This replaces the earlier generic differential visualization. The large hero
image in the supplied concept board is the primary shape reference. The smaller
views and printed dimensions in that generated board are not mutually consistent;
this package builds one coherent object rather than claiming to reproduce all
of those contradictory details at once.

## What is different

The added center mounting ring and the long solid front shaft are gone. The
front interface is a short, stepped, **hollow internal-spline socket**. The cage
is a continuous waisted shell with filleted aperture edges, a separate narrow
top opening and a larger rear shoulder. The rear flange mounting holes continue
through the cage lip. Larger helical gear forms fill the main visible opening.
Front and rear flanges, hub transitions, collars, bearing components and the
clutch stack are separately modeled. The lettering is vector geometry wrapped
onto the outer rail, not a photograph mapped onto it.

## Files

- `renders/hero.png`: final rebuilt hero view.
- `renders/rear.png`, `front.png`, `side.png`, `internals.png`: actual renders of
  the same component geometry from other viewpoints / with outer parts removed.
- `renders/*_raw.png`: unfiltered Monte Carlo renders, before denoising.
- `geometry/TORSEN_X_reference_rebuild.glb`: named component meshes and PBR materials.
- `geometry/*.step`: the six main analytic CAD components. This is **not** a full
  STEP assembly of every gear, ball and spring; those additional parts are meshes.
- `geometry/scene.meshbin`: the geometry used by the renderer, including normals,
  materials and component groups.
- `geometry/mesh_arrays.npz`: the corresponding editable named NumPy arrays.
- `src/build_geometry.py`: complete procedural CAD / mesh construction.
- `src/optimize_geometry.py`: optional normal-aware tessellation reduction.
- `src/pathtrace.cpp`: complete standalone offline renderer.
- `src/finish_render.py`: non-neural, normal/depth/material/variance-guided filtering.
- `src/render_all.py`: render reproducibility commands and checksums.
- `src/validate.py`, `validation.json`: mesh / CAD import checks.
- `render_manifest.json`: resolution, sample counts, elapsed times and hashes.

The STEP CAD uses **millimeters, shaft along Z**. The custom rendering mesh
uses **millimeters, shaft along X, Z up**. The GLB has an explicit
**meters / Y-up** export transform. It should not be imported as a
millimeter-valued glTF scene.

## Render method

Triangle ray tracing with a binned SAH BVH, smooth shading normals, GGX
visible-normal sampling, area-light next-event estimation with multiple
importance sampling, a studio floor and multiple reflection cards. Paths have
up to seven surface interactions with Russian roulette. Small machining
variations are procedural shader variations. Nothing in the render uses the
reference picture as a surface or a background; that picture is only a visual
modeling reference and appears separately on the comparison sheet.

The PNG finishing step is a classical edge-aware variance filter, not an image
synthesis model. The unfiltered PNGs are retained for comparison.

## Rebuild

Python 3.11+ with NumPy, CadQuery, VTK, trimesh, SciPy, Pillow and numba; g++ with
C++17 and OpenMP. Exact versions used for this build are recorded in
`environment.json`.

```bash
python src/build_geometry.py
python src/optimize_geometry.py   # optional; analytic STEP geometry is unchanged

g++ -O3 -march=native -fno-math-errno -fopenmp -std=c++17 \
  src/pathtrace.cpp -o src/pathtrace
python src/render_all.py --threads 5
python src/validate.py
```

A quick render pass is available with `python src/render_all.py --quick`.
The renderer also accepts `--az` and `--el` camera angles, and `--view exploded`
for an axially separated assembly visualization.

## Engineering status

This is a **visual reconstruction of a generated concept**, not a functioning
or production-qualified torque-biasing differential. The gear meshes use sampled
transverse involute profiles and helical sweeps, but no conjugate mating system,
backlash specification, internal differential action, stress analysis, lubrication
scheme, bearing fit or torque-bias ratio has been established. The source image
alone cannot establish the hidden assembly design. The dark surface marking is
applied vector geometry, not a qualified engraving operation in the CAD.

The CAD files and meshes are real, editable geometry. Their presence must not be
confused with mechanical validation or an exact reverse engineering of a
commercial product.
