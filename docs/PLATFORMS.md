# Windows, Linux and macOS

Use Python 3.12 for the pinned environment in `requirements-tested.txt`. The
engineering CI matrix targets Windows x64 and Apple Silicon macOS; Linux is
covered by Quickstart and the full Recovery suite. Each platform job installs
its actual wheels, checks dependencies, builds native CAD, runs MuJoCo contact
and motor tests, installs a real wheel, and renders actual geometry. A failed
or pending job does not establish support.

## Install

From a checkout on Windows PowerShell:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-tested.txt
.\.venv\Scripts\python.exe -m pip install --no-deps --no-build-isolation -e .
.\.venv\Scripts\python.exe -m mechanism_lab doctor
.\.venv\Scripts\python.exe -m mechanism_lab preview example_flange --step
```

On macOS or Linux:

```sh
python3.12 -m venv .venv
.venv/bin/python -m pip install -r requirements-tested.txt
.venv/bin/python -m pip install --no-deps --no-build-isolation -e .
.venv/bin/python -m mechanism_lab doctor
.venv/bin/python -m mechanism_lab preview example_flange --step
```

Omit `example_flange` to run the default ARX belt engineering construction.
The default creates the full authored native assembly and checks its nominal
interfaces before previewing it, so it takes longer than this small setup
smoke. Source-installed and wheel-installed examples use the same maintained
files; no repository-specific absolute path is required.

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
dependency reports, actual versions, test logs, JUnit results and rendered
artifacts. Its Mac runner is explicitly ARM64 according to GitHub's
[runner labels](https://docs.github.com/en/actions/how-tos/write-workflows/choose-where-workflows-run/choose-the-runner-for-a-job).
Wheel availability and successful tests are distinct evidence; hardware
qualification remains outside these software checks.
