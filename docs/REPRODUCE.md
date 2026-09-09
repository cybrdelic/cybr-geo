# Reproduction

Install FFmpeg, a C++17 compiler with OpenMP, OpenGL/EGL libraries and the Python package. An Ubuntu example is in `Dockerfile`; local tests were performed in the environment recorded in `environment.json`. Platform GPU drivers may differ, so the container recipe is a deployment aid, not a claim that every OS was tested.

## New CAD model

Implement `build() -> cybrgeo.Assembly`; keep model parameters and provenance in its metadata. Run the CLI sequence in the root README. `examples/template_part.py` is a complete working example, not pseudocode.

## Manufacturer motor

Place the official STEP at `references/neo_vortex/NEO_Vortex_SPARK_Flex_8mm.STEP`. `tools/fetch_references.py` downloads only the recorded manufacturer URLs and checks HTTP failures. Original manufacturer rights remain with REV Robotics.

```bash
python tools/fetch_references.py
python tools/import_motor.py
python -c "from tools.build_motor_study import prepare_motor; prepare_motor()"
```

The raw import is retained in `examples/neo_vortex/geometry`; the styled, semantically named study is a separate scene at `examples/neo_vortex/study`. All raw vertex arrays are checked unchanged during preparation.

## Drive and full study outputs

The full delivery includes the exact v3 differential input caches, which must be present before building this mixed assembly. A source-only checkout cannot silently replace those missing caches with a generic differential.

```bash
python tools/build_motor_study.py
python tools/validate_drive.py
python tools/render_motor_study.py
```

`--resume` skips the motor's already-completed still/video/part-catalogue stage; it rebuilds drawings, drive footage and path-traced motor stills. Remove that switch for a complete run.

## Earlier projects

Their source and existing assets are preserved under `archive` and `examples/differential`; use each stage's own README. The v3 baseline is loaded from exact saved arrays. Its historical procedural generator is available, but rebuilding CAD need not reproduce old triangle ordering bit for bit.

## Verify a downloaded full delivery

```bash
python tools/audit_delivery.py
```

That audit checks the inventory's file sizes and SHA256 hashes. It is a packaging check, not mechanical certification. `tools/publish.py` can merge the complete extracted workspace into the existing repository with normal local Git credentials; it never force-pushes.
