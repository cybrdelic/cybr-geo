# Working with CYBR GEO

## A new recipe

For an articulated mechanism, start with `lab mechanism --step --media`.
The default is the actual ARX belt project, built through shared native CAD,
material/mass, contact, electrical-control, MuJoCo and recorded-replay services.
Authored projects supply their specification, geometry and task through one
protocol. Earlier bounded component documents remain compatibility examples.
[Construction guide](CONSTRUCTIVE_MECHANISMS.md) and
[local browser Studio](../studio/web-auth/README.md).

For independent static geometry, use the Python recipe interface below.

Start from [custom_flange.py](../examples/custom_flange.py). Its `build()` returns a `mechanism_lab.Assembly` with uniquely named parts, materials and views. `lab preview path/to/recipe.py` performs the normal geometry checks, exports and native render in one invocation. No separate validation command is needed for ordinary iteration.

Use `--view exploded`, `--size 960x720` and `--spp 64` to inspect a different composition. A local Python recipe is trusted executable code. A mesh recipe supports GLB and rendering; `--step` requires analytic CAD for every part. `lab build --step` retains its existing mixed-assembly coverage behavior.

`lab preview` probes the in-house C++ toolchain and uses the existing VTK renderer when that toolchain is unavailable. It checks image decoding/dimensions and GLB readback, and writes `preview.json` last with the actual backend. This establishes usable outputs, not image convergence or physical qualification. The geometry validation report is preserved in the build cache and preview directory. See [platform prerequisites](PLATFORMS.md).

## Rendering backends

| Backend | Command | Requirements |
| --- | --- | --- |
| Automatic geometry preview | `lab preview recipe.py` | C++17/OpenMP, or VTK with a working OpenGL context |
| Required native photographic preview | `lab preview recipe.py --renderer native` | g++/OpenMP and installed Python dependencies |
| Native photographic still | `lab render recipe.py --renderer photoreal` | Same; no V9 studio asset download |
| V9 photographic output | `lab render recipe.py` | Mitsuba, OIDN, external CC0 studio assets; [setup](SHARED_PHOTOGRAPHY.md) |
| Raster inspection | `lab render recipe.py --renderer pbr` | VTK, EGL/OpenGL/Mesa |

Use `lab doctor` for the complete toolkit dependency inventory. It reports video and V9 prerequisites too; those are broader than the offline preview's requirements. FFmpeg is needed for video commands. The default `lab mechanism --media` renders native CAD meshes at recorded solver poses with VTK and encodes its PNG/GIF with Pillow; it needs a supported graphics context but no FFmpeg or C++ renderer.

## Asset-backed models

The independent flange, procedural motor and Nitinol actuator do not require the historical differential archive. The differential and combined drivetrain need recovered or regenerated arrays:

```bash
python tools/rebuild_differential_inputs.py
lab build differential_core
lab build drivetrain --step
```

The regenerated inputs are reconstructed from retained source and identified as regenerated; they do not impersonate original binary deliveries. They remain untracked build inputs.

The manufacturer NEO Vortex study has a separate source preparation path:

```bash
python tools/fetch_references.py
python tools/import_motor.py
python -c 'from tools.build_motor_study import prepare_motor; prepare_motor()'
python tools/render_motor_study.py --motor-only
```

This imports vendor mechanical CAD. [Source hashes and rights](../references/neo_vortex/pinned_sources.json), [study notes](../examples/neo_vortex/README.md).

## Tests

`make test-core` exercises independent CAD, cache/units/import/export contracts, analytic normals, photographic adapters, render provenance and the preview CLI. It does not require historical delivery artifacts or a full production render.

`make test` runs active and legacy tests. Rebuild the differential inputs first. Some tests inspect original release catalogues or videos and explicitly skip when those deliveries are not installed. Render tests additionally need EGL/Mesa and FFmpeg. The complete CI workflow prepares differential inputs and records each test partition; it distinguishes skips from passes.

The README quickstart is a separate integration gate: a native image, GLB readback and analytic STEP export are exercised on all supported Python versions. The expensive existing engineering/media workflows remain available for individual showcases.

## Project layout

`src/mechanism_lab/` is the primary recipe/output toolkit. `src/cybrgeo/` retains the earlier API. `examples/` contains current studies. `archive/` preserves earlier design/source revisions; it is not a second installation target. `outputs/`, `build/` and `dist/` are generated and ignored.

The retained README images are existing renders. They can be viewed without installing anything; reproducing every historical movie and per-part catalogue requires the assets and workload documented for that study.
