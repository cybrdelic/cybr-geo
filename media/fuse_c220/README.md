# FUSE C220 — corrected feed path and operation replay

![Corrected FUSE C220](hero.jpg)

The hotend now has a continuous feed passage, separate heater and sensor bores, non-overlapping nominal interfaces, and two open cooling ducts. G-code replay preserves extrusion-only moves and physical feed across logical E resets.

## Deliverables

- [Operation film, 4 seconds / 24 fps](printing.mp4) and [GIF preview](printing.gif)
- Full-resolution renders: [hero](hero.png), [printing](printing.png), [drive](drive.png), [hotend CAD section](feed_section.png)
- [Analytic STEP assembly, ZIP](cad/FUSE_C220_analytic.step.zip)
- [Complete visual assembly, GLB](cad/FUSE_C220.glb)
- [Calibration G-code](FUSE_C220_calibration.gcode)
- [Parametric source and reproduction instructions](../../examples/fuse_c220/README.md)

The section view removes the negative-Y half and material above nozzle Z+55 from selected hotend bodies. It is an inspection view, not the assembled geometry. STEP retains analytic bodies; GLB also retains mesh-only components.

## Execution evidence

The 54 printer, adversarial G-code, CAD-passage and V9-contract tests pass in [the functional validation run](https://github.com/cybrdelic/cybr-geo/actions/runs/34855721027). Separate reports cover 13 mechanism checks, 9 edge checks, 6 toolpath checks and 5 hotend checks. These are selected nominal checks rather than an exhaustive collision or manufacturing certification.

All imagery was rendered from the actual geometry through the shared CYBR GEO V9 Mitsuba/OIDN pipeline. Stills use 256 samples per pixel and depth 14. The 96 film frames use 128 samples per pixel and depth 12 at 960×720. [The completed film run](https://github.com/cybrdelic/cybr-geo/actions/runs/34855918755) restores checksum-valid checkpoints from [the original render run](https://github.com/cybrdelic/cybr-geo/actions/runs/34850963896), then verifies all 96 frames and the decoded video. There is no image generation or frame interpolation.

The first two seconds show a variable-speed 180-layer time-lapse; the next two seconds replay final extrusion at modeled real time. Beads are imposed geometry driven by the parsed G-code, not molten-polymer simulation.

Exact source commits, hashes and settings are retained in [execution.json](execution.json), [model_stills_execution.json](model_stills_execution.json), [the CAD manifest](cad/manifest.json), per-view JSON, [the video receipt](printing.video.json) and [the decoded video probe](printing.probe.json).

## Physical engineering still required

Select real purchased components and resolve mating threads, retention, tolerance stacks, belt tensioning, endstops, wiring and controller configuration. Thermal, airflow, stiffness, backlash and physical printing performance remain unqualified. These files are a digital prototype, not a manufacturing release.
