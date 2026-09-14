# Architecture and extension contract

The central object is `Assembly(parts, materials, views, metadata, motion_function)`. A `Part` holds immutable bind-space triangle coordinates, indices and normals; optional analytic CadQuery/OCP geometry; material/group identifiers; a rotation center; an explosion offset; provenance and role. `Assembly.pose(part, time_seconds, explosion)` returns a rigid homogeneous transform. The same transform drives rendering, static exports, animated glTF and initial world bounds.

Internal coordinates are **millimetres / shaft X / up Z**. Standard GLB exports are **metres / Y-up**. The native-to-glTF mapping is `(x,y,z) -> (x,z,-y)/1000`. Animated GLBs put that conversion on a world root and retain native-unit child TRS animations. The tests decode the actual buffer/accessor values, accumulate the hierarchy and compare transformed vertices against the model at several sample times. This prevents a superficially passing source-equation test from missing a missing scale or basis conversion.

## Module boundaries

`core.py` defines types, tessellation, validation, rigid transforms, cache IO and project-root resolution. `geometry.py` supplies shared rings, drilled bolt circles, sectors, transported tube meshes and other construction helpers. `models/` defines only model-specific geometry, source anchors, annotations, views and motion. `registry.py` resolves built-ins, a trusted local Python file, a module factory, or a static JSON import recipe.

`exporters.py` consumes the same objects to produce GLB, animated GLB, analytic STEP with an explicit coverage manifest, per-part STL, component lists and the native tracer's triangle/material interchange. STEP never substitutes faceted triangle shells for missing analytic CAD. STL contains the initial assembled world pose and an adjacent unit notice.

`render.py` sets up VTK EGL, a procedurally calculated floating-point studio reflection environment, direct lights, physically based materials, SSAO and filmic tone mapping. It renders actual geometry each frame. `pathtrace.py` compiles/adapts the retained full C++ tracer using material sidecars, camera-relative studio lighting and classically guided filtering. No source photograph is part of either render pipeline.

`media.py` takes a list of `Shot` records and writes frames to FFmpeg. It supports rigid motion, camera orbit, explosion cycles and static inspection, then creates GIFs and full per-part HTML/GLB catalogues. Video frame hashes are recorded before captions. An orbit is actual camera motion around a 3D assembly, not a pan across a still image.

`whiteprint.py` consumes analytic solids or a disclosed mesh fallback. Projection and annotation rendering are shared. `importers.py` imports real geometry with explicit unit contracts. `packaging.py` creates hash inventories and portable full/source-only archives. `cli.py` exposes all these functions.

## Add a new part without rewriting infrastructure

Copy `examples/custom_flange.py`; change `build()` to construct the desired solids/meshes. Give every part a unique stable name, material, group and provenance. Return named views with a sensible orthographic half-height and target. Add a module-level `pose(part,t,e)` only when the assembly needs functional rigid motion. A static import has no invented mechanism constraints.

Use analytic CadQuery solids when accurate edge projection and editable STEP are required. Use procedural meshes for shapes that are naturally mesh-based, such as individual winding conductors. A mixed assembly is valid; its STEP coverage will state the omissions. Supply meaningful dimensions and datums in `metadata['drawings']` instead of modifying the whiteprint renderer.

A recipe file is executable Python: load only trusted code. A JSON import descriptor is non-executable and locates its input relative to itself. Imported GLB/glTF is interpreted as standard metres/Y-up; OBJ/STL/PLY requires explicit units. STEP lengths are normalized by the CAD importer. Standard source texture images, skins and source animation channels are not imported in this release.

## Cache and reproducibility

The cache is an NPZ file plus a JSON manifest. It retains exact triangle arrays, names, materials, view definitions and provenance, not BRep objects. `--step` and whiteprint commands request an analytic rebuild when necessary. Cache fingerprints conservatively include the toolkit Python source, the recipe and preserved differential inputs; editing tools may invalidate more than just geometry. A new recipe cannot accidentally reuse another same-named file's cache because external recipe paths participate in the cache key.

Paths resolve from `MECHANISM_LAB_ROOT`, the extracted source tree, or a working directory containing `assets/`. No new source depends on `/mnt/data` or another session-specific absolute path. Legacy archive files are retained unchanged even where they contain original-session paths; the shared loader replaces their role for current rendering workflows.

A model's successful geometry check does not prove clearances, conjugate loaded contact, bearing suitability, stress, fatigue or function. Provenance is data, not a disclaimer to erase later: preserve it when adding models.
