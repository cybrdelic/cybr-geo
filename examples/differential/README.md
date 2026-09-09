# TORSEN-X — internal inspection and kinematic study, v3

This package continues the supplied reference reconstruction. It contains **new renders and new video frames computed from actual 3D meshes**. It does not use an image generator or the original concept image as a texture, backdrop or animation frame.

There are two explicitly different models here:

1. **Unchanged reference reconstruction:** the existing 81 named mesh components. All original vertices, faces and normals are retained for the internal, section, exploded and individual-part views. Its two large coaxial gear masses are a visual layout with no connecting pinion train; this model is not a working differential.
2. **Separate kinematic-core alternative:** 44 new mesh components, including two side gears, three pairs of compound pinions, journals, carrier pins, bushings and support plates. The original exterior remains available around this alternative in a 119-component assembly. This is a prescribed rigid-body kinematic model, not a contact-force, torque-bias or manufacturing simulation.

The new internals are not passed off as recovered hidden geometry from the generated reference. The exterior remains the previous reconstruction; its reference-fidelity issues have not been declared solved.

## Start here

Open `index.html` from the extracted folder. It is an offline gallery with the complete video, large internal/exploded renders, animated-model downloads and a searchable catalogue of all 81 reference components. It requires no network access.

`videos/TORSEN_X_full_inspection.mp4` is a 35-second, 1280 × 720, 24 fps film:

| Time | View / condition |
|---|---|
| 00:00–00:12 | Original assembled model; component separation; exploded orbit; return to original positions |
| 00:12–00:15 | Alternative core in the preserved exterior; both outputs at equal speed |
| 00:15–00:19 | Exposed alternative core; different output speeds |
| 00:19–00:23 | Prescribed left-output-held condition |
| 00:23–00:27 | Prescribed carrier-held condition; opposite output rotation |
| 00:27–00:31 | One connecting pinion pair isolated, including the central spur mesh |
| 00:31–00:35 | **Camera and carrier stationary**; only the linked gears rotate |

The separate clips are `01_exploded_inspection.mp4`, `02_differential_in_motion.mp4`, and `03_fixed_camera_motion.mp4`.

These recordings render the geometry every frame. They are not camera pans over pre-rendered stills, nor a turntable substituted for independent gear motion. On-screen speeds are demonstration speeds rather than measured hardware data.

## New views

- `renders/pt_exploded.png`: path-traced exploded original reconstruction, 2200 × 1400, 48 samples/pixel.
- `renders/pt_internal.png`: path-traced original interior with exterior removed, 1800 × 1200, 48 samples/pixel.
- `renders/pt_section.png`: path-traced, capped longitudinal section of the original mesh geometry, 1800 × 1200, 48 samples/pixel.
- `renders/pt_kinematic_core.png`: path-traced alternative connecting gear train, 1800 × 1200, 64 samples/pixel.
- `renders/reference_exploded_reverse.png`: reverse-angle exploded view.
- `renders/clutch_bearing_detail.png`: individual reference clutch plates, modeled spring and bearing assembly.
- `renders/kinematic_end.png`: end view of the new planet-axis arrangement.
- `renders/working_section.png`: alternative core within a geometric section of the retained original shell.
- `renders/parts_overview.png`: 20 representative original component families.
- `parts/01_*.png` through `parts/81_*.png`: **all 81 individual reference mesh renders**, including every ball, seal, machining line and marking node.
- `parts/atlas_01.png` through `parts/atlas_05.png`: complete original-component contact sheets.
- `parts/models/*.glb`: all 81 original components as separate named 3D files.

Repeated bearing balls and cosmetic marking objects count as named mesh components. This is not a manufacturing bill of materials.

The section views physically clip and cap the triangle geometry. Exploded positions are offsets chosen for inspection; they are not a verified disassembly or insertion path. Some connected sleeves remain grouped with their interfaces for legibility.

## Rendering methods

The four `pt_*.png` main stills use the included C++ triangle path tracer: binned SAH BVH, GGX shading, area-light sampling, multiple importance sampling, up to seven surface interactions and Russian roulette. Classical normal/depth/material/variance-guided filtering is applied after rendering. Corresponding `_raw.png` and `_clean.png` images are included.

The videos, part catalogue and other inspection stills use VTK physically based raster rendering, an analytically constructed studio environment, direct lights and screen-space ambient occlusion through Mesa EGL. **The videos are not path-traced.** The analytic environment supplies reflection lighting; no photograph or generated concept picture is mapped onto the model.

## Alternative core geometry

- Side gears: 48 teeth each.
- Pinion helical stages: 18 teeth each; six pinions in three pairs.
- Central pinion coupling: 18/18 tooth external spur meshes.
- Transverse module: 0.94 mm.
- Transverse pressure angle: 20°.
- Helix magnitude: 25° with opposite hands at mating parallel-axis stages.
- Side pitch radius: 22.56 mm.
- Pinion pitch radius: 8.46 mm.
- Pinion-axis orbit radius: 31.02 mm.
- Nominal maximum outer gear-tip radius: 40.42 mm.

The generated tooth flanks sample the transverse involute and are swept helically. The root transitions are visualization geometry rather than a certified cutter-generated trochoid. Each helical stage and its coupling-spur stage are indexed independently, then assigned to the same rigid pinion motion. Journal/gear meshes represent a compound rigid component and can overlap as a solid union would; they have not been boolean-unioned into manufacturing solids.

## Kinematic relations

For carrier angle `c` and differential coordinate `d`:

```text
left output angle  = c + d
right output angle = c - d
pinion A local spin = -(48/18) d
pinion B local spin = +(48/18) d
```

The planet centres orbit with the carrier. Their local rotations occur about their own offset axes, not the global shaft axis.

The three mesh constraints in the carrier frame are:

```text
48 (w_left  - w_carrier) + 18 (w_A - w_carrier) = 0
18 (w_A - w_carrier)     + 18 (w_B - w_carrier) = 0
48 (w_right - w_carrier) + 18 (w_B - w_carrier) = 0
```

Eliminating the planet rates gives:

```text
w_left + w_right = 2 w_carrier
```

The demonstration prescribes the carrier rate and one remaining independent coordinate, then applies these constraints to every rigid component. It does not derive output speeds from tire traction, friction, external torque or contact impulses.

## Checks performed

`validation/kinematic_checks.json` records:

- 600 independent transformed-polygon cross-section tests, spanning all three pairs, several axial sections and multiple gear phases.
- Maximum sampled intersection area: **0.0 mm²**.
- Minimum sampled mating-profile clearance: approximately **0.0282 mm**, including the modeled tooth-thickness allowance.
- Maximum sampled angular-constraint residual: approximately **4.98 × 10⁻¹⁴ rad·teeth**.
- Per-component finite-coordinate, closed-mesh and bounding-box checks for all 44 new components.

The polygon checks test the modeled gear profiles; they are not a proof of all-phase three-dimensional interference freedom or a full-assembly fit qualification. Positive sampled clearance does not establish loaded tooth contact.

`validation/frame_kinematics.csv` contains the rendered motion clip's carrier/output/planet states and speeds. `validation/operational_video.json` records 456 unique rendered frames and a maximum speed-relation residual of about **8.9 × 10⁻¹⁶ rpm**. `validation/fixed_camera_video.json` records the separate stationary-camera check.

`validation/animated_glb_checks.json` checks the transformation export against the source model equations. Sampled transform errors are below **6 × 10⁻¹⁷ m** in the export calculation. This is a transform-equivalence check, not a mechanical-accuracy claim.

`validation/release_checks.json` records reference preservation, media decoding, file dimensions and the final inventory.

## Geometry files and units

- `geometry/reference_unchanged.glb`: original reconstruction, unchanged.
- `geometry/reference_exploded.glb`: static exploded original.
- `geometry/reference_explosion_animated.glb`: 12-second standard glTF explosion animation.
- `geometry/kinematic_core.glb`: new core only.
- `geometry/kinematic_core_animated.glb`: 19-second standard glTF core animation.
- `geometry/working_variant.glb`: retained exterior with the alternative core.
- `geometry/working_variant_animated.glb`: the same alternative assembly with animation channels.
- `geometry/reference_step/`: six analytic CAD components from the earlier reconstruction, unmodified.
- `geometry/K_Front_support_plate.step`, `geometry/K_Rear_support_plate.step`: analytic CAD for the two new support plates.

The STEP files are **not** a full STEP assembly of all gears and bearings. Most internal components remain editable procedural meshes.

GLB exports use **metres / Y-up**. The NumPy model arrays and renderer use **millimetres / X shaft axis / Z-up**. The original reference STEP bodies use millimetres with their shaft along Z; the two new support STEP bodies use the working model's X shaft axis. No unit conversion should be inferred solely from the file extension.

## Engineering limits that remain

The alternative's motion is kinematically connected, but this is not a qualified torque-biasing differential. The retained clutch, bearing and housing details are visualization geometry, with unresolved fits and interference possibilities outside the sampled gear tests. No claims are made about bearing raceways, rolling-element kinematics, preload, spring stress, spline engagement, fasteners, lubrication, sealing, tolerances, tooth contact stress, heat, fatigue, efficiency or torque-bias ratio.

The new core is intentionally separated from the unchanged reference model. The generated reference itself does not establish hidden geometry, a usable manufacturing drawing or a verified commercial design. “TORSEN-X” is the label from that concept, not a claim of manufacturer endorsement.

## Reproduce

Dependencies used: Python with NumPy, CadQuery, trimesh, VTK, Shapely, Pillow and numba; g++ with C++17/OpenMP; FFmpeg. Environment versions are recorded in `validation/environment.json`.

```bash
python src/model.py
python src/render.py stills
python src/render.py parts
python src/movies.py inspection
python src/movies.py operational
python src/fixed_camera.py
python src/animate_glb.py
python src/catalog.py

g++ -O3 -march=native -fno-math-errno -fopenmp -std=c++17 \
  src/pathtrace.cpp -o src/pathtrace
python src/offline_stills.py
python src/finalize.py
```

`model.py` reads the included exact reference mesh arrays before creating the separate alternative. The unchanged baseline is not approximated by rebuilding a generic differential. `reference_geometry.py` retains the earlier procedural reference-building source for inspection, but regeneration/tessellation need not be bit-identical to the preserved input mesh.

## Mechanical background

- KHK, helical gear overview and parallel-axis mesh terminology: https://khkgears.net/new/helical_gears.html
- KHK, gear-dimension calculations and normal/transverse systems: https://khkgears.net/new/gear_knowledge/gear_technical_reference/calculation_gear_dimensions.html
- Torsen technical material on internal friction and torque-bias behavior: https://torsen.com/category/tech-info/

These references explain the background. They do not validate this reconstructed design.
