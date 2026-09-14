# Reproduce, extend and package

## Tested environment and installation

This release was exercised locally on Linux with Python 3.13.5, CadQuery/OCP, VTK 9.6.2 offscreen Mesa EGL, NumPy, trimesh, Shapely, SciPy, Numba, Pillow, CairoSVG, ezdxf, FFmpeg and the included C++17/OpenMP renderer. Exact installed Python versions are in `validation/environment.json` and `requirements-tested.txt`. Local pytest and smoke outputs are recorded; the provided Dockerfile and GitHub workflow have not been executed on their remote services.

Install system dependencies before the Python package. On Debian/Ubuntu, the supplied Dockerfile shows the required package set, including `ffmpeg`, `g++`, `cmake`, `libcairo2`, `libegl1`, Mesa GL drivers and `libglib2.0-0`. The command-line package's pinned major rendering/CAD dependencies match the tested environment. The source tree and its `assets/` directory must stay together. A standalone Python wheel does not contain the large historical model assets or native tracer.

```bash
python -m venv .venv
. .venv/bin/activate
python -m pip install -e '.[dev]'
lab doctor
python -m pytest -q
```

Windows/macOS native installations were not exercised in this release. Linux/WSL or the supplied Linux container definition is the documented reproduction path, not a claim of tested native-platform parity. Set `OMP_NUM_THREADS=4`, `LP_NUM_THREADS=4` and `OPENBLAS_NUM_THREADS=1` to limit oversubscription on a workstation.

## Build exact reference inputs and new models

```bash
lab build differential_reference
lab build differential_core
lab build differential_working
lab build m8325s --step
lab build drivetrain --step
```

The three differential recipes load preserved arrays; they do not approximate the old object by making a generic replacement. The motor is built procedurally from declared parameters. Its STEP contains only its 141 analytic components; its complete 181-component mesh is in GLB. The combined drivetrain STEP is also partial, with its own explicit coverage JSON. Historical CAD remains available under `assets/`.

## Render, animate and make catalogues

```bash
lab render m8325s --view hero
lab render m8325s --view stator --renderer pathtrace --spp 64 --threads 4
lab render differential_reference --view exploded
lab render drivetrain --view internal --time 1.2
lab video m8325s --shots examples/inspection_shots.json
lab video differential_reference --view exploded --action explode --seconds 12
lab video drivetrain --view drive_face --action motion --seconds 8
lab animate drivetrain --seconds 8
lab animate m8325s --mode explode --seconds 8
lab catalogue m8325s
lab gif outputs/drivetrain/videos/drivetrain.mp4 --out outputs/drivetrain/preview.gif
```

`python tools/build_release_media.py` runs the delivered motor/drivetrain sequence, including geometry exports, partial STEP, stills, 22/16-second films, GIFs and all 181 motor-part renders. Path-traced stills can then be regenerated with `lab render --renderer pathtrace`; their shared renderer is compiled automatically with CMake. The movie renderer remains PBR rasterization, not offline path tracing.

Exact pixel identity across GPU/driver/library versions is not promised. Model coordinates, parameters and source provenance are deterministic inputs; each rendering records its selected settings. Geometry caches are safe to delete and rebuild, while the exact original differential input arrays in `assets/differential_v3/geometry/` must be retained.

## New geometry and import

```bash
lab build examples/custom_flange.py --step --stl
lab render examples/custom_flange.py
lab blueprint examples/custom_flange.py
lab import model.step --name imported_body
lab render examples/imported/imported_body/imported_body.json
lab import model.glb --name imported_assembly
lab import model.stl --name imported_mesh --units mm --up-axis Z
```

GLB/glTF units are metres and up is Y; these values are not guesses. OBJ/STL/PLY requires an explicit unit choice. STEP imports keep separate solid bodies but do not promise retention of every proprietary CAD feature/history/name. The CAD importer normalizes STEP units to millimetres. Imported drawings can only use analytic hidden-line removal when actual CAD is present. See `docs/WHITEPRINTS.md` for the mesh fallback.

## Offline viewing and archives

Open the root `index.html` for an entirely local image/video gallery. Open `outputs/m8325s/parts/index.html` for all named motor components and GLB download links. A local HTTP server is optional for browsers that restrict file URLs:

```bash
python -m http.server 8000
# Open http://localhost:8000/
lab pack --out dist/cybr-mechanism-lab-full.zip
lab pack --source-only --out dist/cybr-mechanism-lab-regeneration-kit.zip
```

The full archive preserves old and new media, source and geometry. The smaller regeneration kit contains toolkit source, documents/tests and exact differential mesh inputs, but omits already rendered media and archived historical CAD. It has its own README explaining those omissions. Neither archive includes private environment files, credentials, font files, virtual environments or generated native build directories. The full manifest records paths, byte sizes and SHA-256 hashes; it explicitly excludes its own checksum.

Git publication is a separate explicit step. See `docs/PUBLICATION.md`. An archive, local commit or prepared README is not a remote upload.
