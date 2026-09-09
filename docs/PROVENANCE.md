# Provenance and license scope

## Preserved project assets

`assets/legacy_v1/torquebias_diff/` contains the first modeled differential package. `assets/reference_v2/` contains the subsequent reference-fidelity rebuild. `assets/differential_v3/` contains the unchanged reference, separately labeled kinematic core, original internal/exploded/individual views, films and source. Original extracted payload files are preserved; the full release does not repeat the redundant ZIP wrappers because their contents are already included.

The earliest generic mounting-bracket concept image and later TORSEN-X concept poster are retained as clearly labeled reference images. These were generated earlier in the conversation, not by this release's geometry renderer. They are not used as a rendering texture or video frame. Every newly labeled motor/drive render is computed from actual geometry.

Historical sources may contain original-session absolute paths. They are archival evidence, not the new cross-model API. The reusable loader reads the preserved exact mesh arrays by project-relative paths and the current toolkit source avoids session-specific absolute paths. The original image's invented specifications are not adopted as validated mechanical data.

## New project code and models

The `mechanism_lab` package, source-based M8325s reconstruction, custom adapter/drive study, generic command line, import/export, media, drawing and packaging tools are part of this project's development. The new native tracer adaptation retains and generalizes the earlier project tracer rather than replacing it with an image service. Geometric invariants and export conventions are tested in the supplied suite.

No new project-wide redistribution license has been selected on the owner's behalf. Existing original notices are retained with their files. Publishing the repository does not itself turn unlabeled files into MIT/GPL-licensed material. The owner should explicitly choose license scope before claiming an open-source license.

Third-party Python/system dependencies are installed separately, not copied wholesale into this repository. They retain their own licenses. No font files, credential files, environment secrets or private account records are bundled. PDF font embedding performed by the renderer is not a redistributed standalone font file.

## Manufacturer boundaries

The motor's published external interface and inspected photos are attributed in `MOTOR_SOURCES.md`. Manufacturer photos and native Onshape CAD are not vendored. The new model is not an official ODrive CAD release, and the modeled internals should not be represented as a teardown-derived or factory-verified assembly.

ODrive and Torsen names are used descriptively. There is no manufacturer endorsement, trademark grant, certified load rating or implied approval for production hardware.
