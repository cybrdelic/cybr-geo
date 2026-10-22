# Windows, Linux and macOS

Use Python 3.12 for the pinned environment in `requirements-tested.txt`. The
engineering CI matrix targets Windows x64, Ubuntu 24.04 x64 and Apple Silicon
macOS. Each platform job installs its actual wheels, checks dependencies,
constructs the complete default native assembly through the public CLI, runs
its nominal MuJoCo task, independently reopens its exported STEP, tests shared
CAD/controller components, installs a real wheel, and renders actual geometry.
The same matrix separately constructs and simulates the full named `iris09`
client, then independently reopens its STEP and GLB.
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
.\.venv\Scripts\python.exe -m mechanism_lab mechanism iris09 --step --out outputs/iris09
.\.venv\Scripts\python.exe -m mechanism_lab preview example_flange --step
```

On macOS or Linux:

```sh
python3.12 -m venv .venv
.venv/bin/python -m pip install -r requirements-tested.txt
.venv/bin/python -m pip install --no-deps --no-build-isolation -e .
.venv/bin/python -m mechanism_lab doctor
.venv/bin/python -m mechanism_lab mechanism --step --out outputs/default
.venv/bin/python -m mechanism_lab mechanism iris09 --step --out outputs/iris09
.venv/bin/python -m mechanism_lab preview example_flange --step
```

The model-omitted `mechanism --step` command constructs the default ARX belt
assembly, checks its nominal interfaces, runs its declared complete contact-driven
task, and exports the aggregate native STEP and GLB. It takes several
minutes. The separately named flange preview is a small graphics setup smoke;
it does not stand in for complete-default validation. Omit `example_flange`
from `preview` to construct and preview the full default too. Source-installed
and wheel-installed examples use the same maintained files; no
repository-specific absolute path is required.

On every matrix platform, the full default has an 1800-second total budget for
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

The separate IRIS check has its own 1800-second construction, simulation,
export and readback budget. It runs the newly derived linkage's eight-second
cycle (40,000 integration steps), with one electrical motor, nine passive
leaves, nine passive rods and nine native connect constraints. The verifier
re-evaluates the task from the saved trace, requires complete every-step joint
and closure monitors, checks CAD-derived masses and source binding, and matches
every native placement against reopened aggregate STEP and GLB. Its receipt
retains actual extrema, motor measurements, nominal interface results and the
authored geometry assumptions. Individual command deadlines total 105 minutes:
two 30-minute native projects, 15 minutes for the latest source geometry,
15 minutes for the fault subset, 12 minutes for required tests and three
minutes for graphics. The 115-minute matrix job budget
leaves ten minutes for dependency setup and artifact upload without shortening
any of those command deadlines. These are upper bounds, not expected runtimes.
The bounds include the observed 1253-second ARX build and independent readback
under concurrent local load; they do not change any physical acceptance limit.

The additional geometry step reconstructs the actual Refined source at
`85bd6d2c`: 278 native parts with 123 explicit BREP replacements. A separate
process reopens every local STEP and reconciles changed-part properties.
It also reconstructs all six P6B print-first articles, reopens their native and
plate-local STEP, verifies mass integrals, hashes and nominal splice mates.
This step has one 900-second deadline for both fresh clients and readbacks.
Its results do not rerun compact dynamics or establish printed-arm load
capacity. Required tests include actual upstream-builder parity and installed
wheel inputs, with no skips accepted on these matrix platforms.

Using that job's fresh ARX CAD, CI also runs the three inherited failing
ordinary fault cases and the later `power_loss_7.2` regression at the
corresponding authored task phases, plus friction loss and weak-grip adverse
controls. This six-case subset uses the maintained
`tools/check_arx_faults.py` runner with two workers and a 900-second deadline.
Ordinary cases must meet their unchanged source requirements. The friction and
grip controls must fail the source's 1.5 mm payload-placement task while retaining
valid scenario and measurement coverage. Their original numerical-limit result
is retained separately and may still pass. Missing measurements cannot turn an
adverse control into a pass.
The full source scenario suite is separate from this bounded platform check.

IRIS currently uses ideal pin constraints without collision geometry or leaf,
bearing and housing contact-force modeling. Its selected nominal CAD fits,
leaf keepouts, pin/lower-lip envelope and keeper/radial-bearing envelope do not
prove a continuous clearance envelope for the entire assembly. The assumed
motor, foil thickness, materials, production tolerances,
retainer strength, fatigue and wear remain unqualified. This derived mechanism
is separate from the original source's prescribed optical animation.

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
artifacts, ARX and IRIS acceptance/source receipts and aggregate STEP/GLB.
It excludes the duplicated individual CAD/mesh files from artifact upload.
Its Mac runner is explicitly ARM64 according to GitHub's
[runner labels](https://docs.github.com/en/actions/how-tos/write-workflows/choose-where-workflows-run/choose-the-runner-for-a-job).
Wheel availability and successful tests are distinct evidence; hardware
qualification remains outside these software checks.
