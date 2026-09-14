# CYBR YARD — DIY skatepark

A 22 × 16 metre original design study built with CYBR GEO / Mechanism Lab. All visible features are analytic CAD or explicitly identified procedural mesh geometry. The scene contains **1,404 named parts**, **1,401 valid analytic CAD parts**, and **157,034 triangles** in the pinned environment. There are no image-generation calls, photographic backgrounds, hidden downloaded meshes, or billboard substitute geometry.

## Layout

| Element | Nominal dimensions |
|---|---|
| Mini ramp | 4.80 m wide; 3.60 m flat; 1.00 m rise; 2.20 m transition radius |
| Mini-ramp total envelope | 9.29 × 4.80 m, excluding access stairs |
| Bank | 3.00 m wide; 2.35 m slope run; 0.65 m rise; 0.85 m deck |
| Manual pad | 2.40 × 1.20 × 0.20 m |
| Grind ledge | 3.20 × 0.62 × 0.36 m |
| Flat bar | 3.00 m long; 0.34 m top; 50 × 50 × 3 mm nominal hollow section |
| Slappy curb | 3.20 × 0.33 × 0.12 m |
| Quarter pipe | 3.60 m wide; 0.84 m rise; 2.10 m radius; 0.76 m deck |

The mini-ramp flat is 165 mm above grade. Its deck elevation is therefore 1,165 mm above grade, not 1,000 mm. The transitions use separate 6 + 9 + 9 mm skins, accurately offset support-rib curves, timber joists, separate 60.3 mm OD coping with 3.2 mm wall, nominal fasteners, deck framing, guard geometry, and six access treads. Coping reveal is 6 mm relative to the deck plane and terminal transition tangent. These are design choices, not approved construction specifications.

The freestanding quarter pipe has a closed, procedurally meshed steel entry joining grade to the circular skin. The two planters use open leaf meshes. Those three mesh-only objects are explicitly omitted from the analytic STEP export; the full assembly is present in GLB.

## Reproduce

Use Python 3.13 and the repository's `requirements-tested.txt` for the pinned numerical/CAD environment. System requirements include a C++17/OpenMP compiler, Cairo and the toolkit's normal OpenGL/EGL libraries. Install into a virtual environment.

```bash
python -m pip install -r requirements-tested.txt
python -m pip install --no-deps -e .

# Build real CAD, export GLB/STEP, audit geometry and render all named views.
PYTHONPATH=src python examples/diy_skatepark/build_render.py \
  --out build/diy_skatepark --step --width 1920 --height 1280 --spp 192 --depth 12

# Regenerate one view from the saved mesh cache without rebuilding CAD.
PYTHONPATH=src python examples/diy_skatepark/build_render.py \
  --out build/diy_skatepark --cache --views mini --width 1920 --height 1280 --spp 128

# Dimensioned vector plan, PNG and nominal developed curved-skin schedule.
python examples/diy_skatepark/make_plan.py build/diy_skatepark

# Self-contained interactive geometry inspector; no CDN or server required.
PYTHONPATH=src python examples/diy_skatepark/make_viewer.py build/diy_skatepark

# Fresh verification, including the existing V9 default regression gates.
PYTHONPATH=src python -m pytest -q \
  tests/test_diy_skatepark.py tests/test_v9_render_profile.py \
  tests/test_photo_material_export.py tests/test_photoreal_contract.py \
  tests/test_analytic_normals.py tests/test_truth_rendering.py
```

`lab build examples/diy_skatepark/recipe.py --step` and `lab render examples/diy_skatepark/recipe.py` also consume the standard `build() -> Assembly` recipe. Edit `ParkConfig` and the named recipe functions to change the design, then rebuild without `--cache`.

## Rendering and inspection

The stills use the toolkit's **V9 native BVH/GGX/MIS path tracer**, its thin-lens camera and three geometry-guided linear-light filtering passes. Final delivery settings are recorded alongside every image: 1920 × 1280 / 192 spp for hero and coping; 1920 × 1280 / 128 spp for mini and street; 1600 × 1066 / 128 spp for framing, all at a 12-bounce limit. Raw tone-mapped renders are retained beside the filtered PNGs.

This extension adds an explicitly selected `outdoor` lighting rig and explicit `wood`, `concrete` and `grip` material finishes. It does **not** change existing `product` / `classic` defaults or silently infer finish from a part name. The light is a distant finite area emitter with roughly solar angular size; the background/indirect environment and material reflectance are authored RGB approximations. This is **not spectral rendering**, a measured sky, or measured material optics.

`CYBR_YARD_viewer.html` is a separate, self-contained **WebGL raster geometry inspector**, not the offline photographic renderer. It supports orbit/pan/zoom, camera presets, hidden mini-ramp skins, fence visibility, and lifted-skin inspection offsets. The same model vertices and normals are embedded in it; the extra preview ground is identified separately. A depth-buffered Canvas triangle fallback handles browsers where WebGL 2 is unavailable. That fallback was exercised locally at desktop and mobile viewport sizes, including camera changes, hidden skins, and lifted skins. The accelerated WebGL path was not runtime-verified in this environment. Its inspection offsets are not an assembly/disassembly validation.

## What was checked

The fresh local verification passed 25 tests: finite and indexed geometry, valid analytic BReps, expected mesh boundaries/winding, exact transition radii, nominal coping reveal, separated obstacle envelopes, at least 1 m of nominal slab space beyond the access treads, deterministic repeated construction, photographic material export, analytic normals, truth/provenance gates, and unchanged V9 defaults. `tests.xml`, `geometry_validation.json` and each image's render receipt accompany the binary delivery. A source-only checkout does not imply those binary artifacts are committed to GitHub.

## Limits before building anything physical

This is **not a construction-qualified skatepark**. It has no structural/load analysis, foundation design, anchorage calculations, fatigue analysis, drainage design, traction measurement, rider simulation, fall-zone analysis or code approval. Plywood bending feasibility, weather protection, sheet seam staggering, support spacing and fastener/countersink details require a build-specific review. Nominal skin-development and stock tables exclude kerf, nesting, tolerances and fastening qualification. The invented site is not a survey of the user's property.

Project code retains the repository's GPL-2.0 license. No font files are distributed.
