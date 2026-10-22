# Windows, Linux and macOS

Use Python 3.12 for the pinned environment in `requirements-tested.txt`. The
engineering CI matrix targets Windows x64, Ubuntu 24.04 x64 and Apple Silicon
macOS. Each platform job installs its actual wheels, checks dependencies,
constructs the complete default native assembly through the public CLI, runs
its nominal MuJoCo task, independently reopens its exported STEP, tests shared
CAD/controller components, installs a real wheel, and renders actual geometry.
Linux also runs Quickstart and the full Recovery suite. A failed or pending job
does not establish support.

## Install

From a checkout on Windows PowerShell:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-tested.txt
.\.venv\Scripts\python.exe -m pip install --no-deps --no-build-isolation -e .
.\.venv\Scripts\python.exe -m mechanism_lab doctor
.\.venv\Scripts\python.exe -m mechanism_lab mechanism --step --out outputs/default
.\.venv\Scripts\python.exe -m mechanism_lab preview example_flange --step
```

On macOS or Linux:

```sh
python3.12 -m venv .venv
.venv/bin/python -m pip install -r requirements-tested.txt
.venv/bin/python -m pip install --no-deps --no-build-isolation -e .
.venv/bin/python -m mechanism_lab doctor
.venv/bin/python -m mechanism_lab mechanism --step --out outputs/default
.venv/bin/python -m mechanism_lab preview example_flange --step
```

The model-omitted `mechanism --step` command constructs the default ARX belt
assembly, checks its nominal interfaces, runs ten seconds of contact-driven
simulation, and exports the aggregate native STEP and GLB. It takes several
minutes. The separately named flange preview is a small graphics setup smoke;
it does not stand in for complete-default validation. Omit `example_flange`
from `preview` to construct and preview the full default too. Source-installed
and wheel-installed examples use the same maintained files; no
repository-specific absolute path is required.

On every matrix platform, the full default has a 1200-second total budget for
construction, simulation, aggregate STEP export and separate-process readback.
CI requires the passing engineering report, all 118 native parts and 375
placements, current project/shared-source and runtime binding, matching native
artifact hashes, and valid positive-volume solids reopened from aggregate STEP.
It also reopens the GLB and matches its 385 scene components against the STEP
coverage, including all native placements and the jaw/puck/nest additions.
Fatal-signal diagnostics, stage durations and completion markers are
retained on failure. This checks
the nominal software construction and task; it does not qualify production
tolerances, unrestricted motion, fatigue or hardware safety.

## Preview backends

`lab preview` defaults to `--renderer auto`. It compiles and executes a small
C++17/OpenMP capability probe. When that toolchain is unavailable, preview uses
the existing VTK PBR renderer with actual meshes, materials, camera and studio
lighting. Windows and macOS use VTK's native window factory; headless Linux
uses EGL when available. An explicit `VTK_DEFAULT_OPENGL_WINDOW` setting is
honored by the VTK factory.

```sh
lab preview example_flange --renderer pbr --step
lab preview example_flange --renderer native --step
```

`native` requires a working GCC-compatible C++17/OpenMP toolchain and fails if
it cannot run. Apple Clang can be named `g++` while lacking OpenMP, so its name
alone is not treated as support. Renderer failures after a successful compiler
probe also remain failures; they are not hidden by an image substitution.
`render`, `video` and `film` retain their separately selected rendering engines.

`preview.json` records the selected backend and any toolchain fallback reason.
VTK writes `hero.render.json` with its actual window class, version and graphics
capabilities. PBR reports raster supersampling and does not claim path-tracing
samples or simulated mechanics. A real OpenGL context is required; a missing
graphics driver is an error. This project does not download unverified graphics
drivers or replace unavailable renders with static images.

For headless Ubuntu, the Quickstart workflow installs Mesa/EGL/OpenGL runtime
libraries and GCC. Other Linux distributions need their corresponding runtime
packages. Windows/macOS do not run the Ubuntu APT helper.

The Windows hosted runner does not provide the desktop's graphics driver. CI
therefore uses Mesa 26.2.4 `llvmpipe` for genuine CPU OpenGL rendering. The
[setup helper](../.github/scripts/setup_windows_software_gl.py) verifies the
published SHA256 of a pinned [mesa-dist-win release](https://github.com/pal1000/mesa-dist-win/releases/tag/26.2.4),
extracts only its two x64 WGL libraries into the job's temporary directory, and
preloads them before VTK imports. It changes neither Windows drivers nor the
Python installation. VTK's own [Windows software rendering guidance](https://github.com/Kitware/VTK/blob/v9.6.2/Rendering/OpenGL2/vtkOSOpenGLRenderWindow.cxx)
identifies this distributor. A separate real sphere render must report
`llvmpipe` and pass the image check before CI continues; the final assembly
flange preview must report the same software backend. Normal application startup does
not download or install these libraries.

PBR writes `hero.render-progress.json` before initialization and updates it at
rendering, image saving and completion. Its completed receipt includes stage
durations. CI captures fatal native Python stacks and retains stage receipts on
a timeout. It avoids periodic `dump_traceback_later` watchdogs because of the
upstream [CPython stack-walker race](https://github.com/python/cpython/issues/140815).
One Ubuntu run crashed during a truncated periodic dump after native construction
and simulation; the watchdog is a plausible instrumentation cause, not a proven
attribution. The full default must still pass with fatal-only diagnostics. The public
CLI setup smoke requests 128×96 pixels with the normal PBR lighting and 2×
supersampling; this CI budget does not change production rendering defaults.

## Feature-specific prerequisites and current scope

- The pinned MuJoCo 3.16.0 distribution supplies Windows x64 and macOS ARM64
  wheels. It does not supply an Intel macOS wheel; Intel Mac simulation needs a
  separately qualified source build and is not covered by this binary matrix.
  [MuJoCo release files](https://pypi.org/project/mujoco/3.16.0/#files)
- SVG/PDF conversion through CairoSVG additionally needs system Cairo. Its
  Python wheel alone does not establish PDF support; follow the platform
  prerequisites in the [CairoSVG installation documentation](https://cairosvg.org/documentation/).
  STEP, GLB, MuJoCo and PBR preview do not require this conversion backend.
- Video and GIF export need FFmpeg on `PATH`. The engineering platform smoke
  checks still rendering and does not claim to qualify every video encoder or
  photographic backend. `lab doctor` reports detected tools.
- Node is required for the tests that compare the adapted controller against
  the retained original JavaScript. CI installs it explicitly; a skipped local
  parity test is not a pass.

The [platform workflow](../.github/workflows/engineering-portability.yml) retains
dependency reports, actual versions, test logs, JUnit results, rendered
artifacts, full-default acceptance/source receipts and aggregate STEP/GLB.
It excludes the duplicated individual CAD/mesh files from artifact upload.
Its Mac runner is explicitly ARM64 according to GitHub's
[runner labels](https://docs.github.com/en/actions/how-tos/write-workflows/choose-where-workflows-run/choose-the-runner-for-a-job).
Wheel availability and successful tests are distinct evidence; hardware
qualification remains outside these software checks.
