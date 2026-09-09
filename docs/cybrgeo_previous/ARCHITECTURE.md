# Architecture

## Model boundary

`cybrgeo.core.Material`, `Part` and `Assembly` are the shared data model. A `Part` has a unique safe name, explicit material index, triangle arrays and normals, group, motion label, pivot, exploded-view offset, role and provenance metadata. `Assembly.cad` retains optional analytic B-reps independently of mesh arrays. A model plugin exports a Python `build()` function returning one Assembly. The example mounting plate exercises this path end to end.

`features.py` supplies involute spur-gear profiles, annuli and bolt circles. These are constructive geometry helpers, not a strength-design or automatic-CAD solver. Root transitions are approximate rather than cutter-generated trochoids.

## Persistence and units

`Assembly.save()` emits a versioned JSON scene, compressed NumPy arrays, SHA256 cache checksum, per-part BREP files and a STEP assembly when analytic shapes are available. Loading disallows NumPy pickle data and rejects checksum mismatches. Unique part names, finite vertices, normal counts, triangle indices and palette bounds are checked. Stored mesh data is millimetre-native and Z-up.

GLB export performs one right-handed mm/Z-up to metre/Y-up conversion. STEP, STL and BREP remain in millimetres. Materials are explicit PBR properties, not pre-baked image textures. Source CAD and tessellated meshes remain distinguishable. A mixed assembly's STEP contains only its analytic shapes, never secretly faceted substitutes for the mesh-only parts.

## Renderers

`render.Studio` is a VTK EGL PBR preview renderer with procedural studio reflections and geometric sections. A pose dictionary maps part names to 4x4 matrices. `media.video()` invokes a trajectory callback for every frame, renders the meshes and streams RGB pixels to FFmpeg. H.264 MP4 and palette-generated GIF outputs receive ffprobe receipts. A moving camera alone is not used as a substitute for mechanism motion.

`offline.render()` uses the retained C++ Monte Carlo path tracer. Compiled binaries are cached by source plus material palette, so new models do not need rewritten shaders. Raw output and a separately identified geometric guide-aware atrous filter result are both retained. No neural or image-generation model supplies image content.

`whiteprint.write_whiteprint()` performs OpenCascade hidden-line removal on the analytic B-rep. Four independently labeled projections are composed on A3. Curve sampling is recorded; PDF and SVG retain vector paths. DXF uses separate visible/hidden/annotation layers in sheet millimetres. Current dimension labels are model-envelope references, not a complete GD&T or manufacturing-dimension system.

## What is not generalized yet

Automatic feature recognition, dimension-stack design, associative GD&T, commercial CAM, solver-derived torque/thermal response, universal watertight mesh-to-solid conversion, and automatic certification are not implemented. The motor's hidden electromagnetic/electronic details are not reverse-engineered. The tools accept new geometry; they do not make every design safe or manufacturable.
