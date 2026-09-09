# CYBR GEO

A reusable mechanical-geometry workshop: named CAD and meshes, studio and CPU path-traced images, real geometry animation, part catalogues, and CAD-derived whiteprint drawings.

![Differential in motion](media/differential_preview.gif)

![Exploded differential](media/differential_exploded.jpg)

## Differential studies

The complete delivery retains the **81-component reference reconstruction**, its **44-component alternative kinematic core**, and the two earlier modeling stages. These remain explicitly separate designs. The alternative core obeys ideal rigid-body gear constraints; it is not a torque-bias/contact-force simulation.

The full-resolution delivery has the 35-second original inspection film, all individual parts, STEP/GLB files, animated GLBs, original source, and verification data. See [the preserved differential documentation](examples/differential/README.md) and [the asset inventory](docs/DELIVERY.md). Publication status is stated in that inventory rather than inferred from a workflow's success flag.

## Motor and carrier-drive study

![Motor inspection](media/motor_preview.gif)

![NEO Vortex mechanical model](media/motor_hero.jpg)

The motor study uses **REV Robotics' supplied NEO Vortex + SPARK Flex + 8 mm shaft CAD**, retaining **75 solids and one surface body**. All 76 bodies have separate model and image exports in the complete delivery. This is an import, inspection, material assignment and animation of manufacturer mechanical CAD, not a claim to have reverse-engineered the undisclosed windings, magnets or control circuitry.

A separately authored **20T / 80T, 4:1 spur-drive interface** connects the motor to the differential's carrier flange. It is a custom prototype interface, **not a bolt-on or load-qualified compatibility claim**. There is no verified torque rating for the generated differential. See [the motor study](examples/neo_vortex/README.md) and [drive-interface qualifications](examples/motor_drive/README.md).

![Motor-driven differential](media/drive_preview.gif)

## Whiteprints

![CAD-derived motor whiteprint](media/whiteprint_preview.jpg)

The drawing tool uses OpenCascade hidden-line removal on analytic B-reps. It produces **A3 SVG, vector PDF, DXF, PNG and a JSON receipt**. Views, paper scale and REF dimensions are explicit. No tolerances, fits or certification are invented. Mesh-only scenes are rejected instead of being disguised as analytic production CAD.

## Reuse the pipeline

Python 3.11 or newer; working VTK EGL/OpenGL for the preview renderer; FFmpeg for video; a C++17/OpenMP compiler for offline path tracing. See [architecture and units](docs/ARCHITECTURE.md), [tested environment](docs/environment.json), and [reproduction](docs/REPRODUCE.md).

```bash
python -m venv .venv
# Linux/macOS: source .venv/bin/activate
# Windows: .venv\Scripts\activate
python -m pip install -e '.[test]'

cybrgeo build examples/template_part.py --out build/template
cybrgeo render build/template --out build/template/hero.png
cybrgeo render build/template --backend pathtrace --samples 64 --out build/template/pathtraced.png
cybrgeo video build/template --out build/template/orbit.mp4
cybrgeo video build/template --mode explode --out build/template/exploded.mp4
cybrgeo parts build/template --out build/template/parts
cybrgeo whiteprint build/template --out build/template/drawing --title 'MOUNTING PLATE'
cybrgeo validate build/template
python -m pytest -q
```

A new project supplies `build() -> Assembly`. It reuses the same exporters, renderer, camera paths, video encoder, part catalogue and drawing tool. Custom animation supplies per-part transform matrices; it does not require a new renderer. This is a geometry-programming framework, not a claim that arbitrary engineering design is solved automatically.

## Provenance and scope

All presented new images and videos come from geometry rendering. The earlier generated concept poster is retained only as a labeled visual reference, not as a render texture. GLB uses metres/Y-up; the new CAD, meshes and STL use millimetres/Z-up. Archived projects retain their own documented conventions.

The repository's GPL-2.0 license remains in force for the project software. Manufacturer CAD, photography, drawings and trademarks retain their respective rights; see [provenance](docs/PROVENANCE.md). These studies are not manufacturing releases.
