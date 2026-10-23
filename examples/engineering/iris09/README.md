# IRIS-09 derived mechanism and source geometry

IRIS is now an authored client of CYBR GEO's shared construction, physical-model,
control, MuJoCo and native replay services. The pinned optical source supplies
the leaf profiles, cant and housing geometry. A separately identified engineering
adaptation adds a single driven annular rotor, nine bored connecting rods, passive
leaf shafts, bearings and retaining features. Only the rotor has a motor; the
leaves respond through the integrated pin constraints and rod forces.

The original source geometry study is retained below. Its prescribed display
motion and world clipping are separate from the new rigid mechanism.

## Run the derived mechanism

With CYBR GEO installed, from the repository root:

```sh
lab mechanism iris09 --out outputs/engineering-iris09-derived --step --media --size 800x600 --fps 8
```

The project uses its declared eight-second cycle. The shared pipeline constructs
native CAD and full inertias, binds the complete project and shared-service source
identities, integrates the causal electrical/motor and constrained mechanical
model, then replays the saved body poses. It writes `engineering.json`, the
editable physical/control sources, `simulation/trace.json.gz`, native build and
source-binding receipts, and actual geometry media. Accepted runs write
`assembly.glb` and the requested analytic STEP; failed runs remain inspectable as
`diagnostic-assembly.glb` and do not gain a passing result.

The [engineering evidence packet](../../../docs/pr-assets/iris-engineering/README.md)
contains the measured run, actual native-geometry PNG/GIF and identity checks.
It was rebuilt with the current shared runtime and linkage helpers. Its original
nine-pose clearance audit remains separate from the exact native-geometry and
recorded-state equivalence receipt; no whole-motion clearance claim is inferred.
The source-derived dimensions and new assumptions are declared in
[data/mechanism.json](data/mechanism.json); the geometry and physical layout are
authored together in [mechanism.py](mechanism.py). Shared `PinnedLink`,
`NativeConstruction`, `PhysicalModel`, `CausalSimulation` and `native_replay`
provide the reusable implementation. This example supplies no separate physics
solver or renderer.

## What was added, and what remains assumed

The original shader removes leaf material with a stationary world cylinder at
each pose. A rigid part cannot do that. The new [leaf construction](leaf_geometry.py)
makes one invariant crop of the source's two-circle canted slab for the declared
0–0.9 radian hinge interval. It also removes the continuous swept envelopes of
neighboring fixed shafts. The source profile and cant remain visible, while the
crop and shaft keepouts are explicitly new geometry. These particular envelope
constructions do not establish clearance for every part of the assembly.

The adaptation adds 20 mm connecting rods, 2 mm pins with 2.075 mm nominal
bores, grooved split retainers, a fixed reaction housing and
an assumed annular direct-drive torque port. Native interface measurements and
integrated closure/travel checks have separate receipts. The solver uses ideal
pin axes and loop constraints; it does not resolve bearing contact, fit or wear.
No independent actuator or prescribed motion is applied to a follower leaf.
The same shared `PinBoreFit` derives rod and fixed-bearing bores from a 0.025 mm
required service radial gap, authored 0.005 mm radial variation budgets for
each of the pin and bore and a further 0.0025 mm design reserve. The resulting
nominal radial gap is 0.0375 mm; the worst gap under those *declared design
targets* is 0.0275 mm, leaving positive reserve beyond service. No process or
supplier has demonstrated these or the axial/radial variation budgets below.
Some axial fits now have authored positive reserves; other keeper/collar fits
remain unallocated and no production tolerance stack is demonstrated. Changing
any fit input invalidates prior CAD, mass/inertia,
interface, simulation and media receipts until regenerated.

The pin, keeper, bearing lip and rotor heights share one `axial_stack`
construction. GEO's reusable `DirectedFaceGap` derives each mating surface
from its required service gap, two authored part-variation budgets and a
positive design reserve. With the supplied 0.005 mm variation per side and
0.005 mm reserve, the lower fixed lip starts **0.215 mm** above the moving
pin/keeper envelope; the rotor starts another **0.215 mm** above the lip and
the upper capture lip has the same constructed gap. The fixed radial bearing
starts **0.115 mm** above the moving output keeper. Native requirements measure
those face gaps and carry the authored budgets into the design screen. The
lip/keeper axial envelopes apply throughout planar motion; the independent
whole-assembly audit still has its stated limited pair coverage.

The rotor's nominal 64 mm outer radius also constructs the fixed sleeve's
64.05 mm inner radius through the same shared face-fit rule: 0.03 mm required
service gap, 0.005 mm variation per side and 0.01 mm design reserve. A native
annular interface measures that radial gap where rotor and sleeve overlap in
height. The fixed post overlap and bearing-wall guards reject fits that would
disconnect their authored support. These budgets are design targets, not
demonstrated process capability, bearing pressure, sleeve strength or a
whole-assembly clearance certificate. Other keeper, collar and optical-leaf
fits remain unresolved for manufacturing qualification.

The **50 mm per scene unit conversion remains an assumption**. It preserves the
source's **0.0325 mm foil thickness**, whose stiffness, buckling, attachment to the
hub, fatigue and manufacturing tolerance stack are unqualified. The rigid-body
model treats each leaf/shaft/crank as a rigid assembly. The split retainers have
nominal CAD capture, but their elastic installation and retention strength are
not established. Joint travel guards do not establish physical stop hardware.
Motor electrical, thermal and torque limits and material densities are explicit
engineering assumptions, not selected supplier data or measurements. Simulation
acceptance does not establish hardware buildability or a complete continuous
clearance proof.

Run the focused construction, linkage and replay tests with:

```sh
python -m pytest tests/test_iris09_mechanism.py tests/test_iris09_leaf_mechanism.py tests/test_linkages.py tests/test_native_replay.py
```

## Original source geometry study

This separate path constructs the actual IRIS-09 source's nine-leaf geometry and
housing features with shared `ShapeEvaluator` and `NativeConstruction` services.
It exports analytic CAD at a defined prescribed reference pose. This path has no
mechanical acceptance gate or physics model.

![IRIS-09 reference CAD](evidence/reference.png)

From the repository root, with CYBR GEO installed:

```sh
python -m examples.engineering.iris09.design --output outputs/iris09-reference --time 5 --scene-unit-mm 50 --render
```

The installed wheel includes the same maintained example. It can also be run
through the installed example locator (normal CLI arguments still apply):

```python
import runpy
from mechanism_lab.engineering_source import project_directory
runpy.run_path(str(project_directory('iris09') / 'design.py'), run_name='__main__')
```

The command writes STEP, STL, GLB, actual-CAD PNG views, a shared leaf DAG, source hashes, and `geometry-study.json`. Native artifacts and their hashes are recorded in `native-build.json`. Rendering uses the existing GEO renderer, without source images, image generation or an alternate shader renderer.

The reference pose is **t = 5 seconds**, hinge angle **0.45 radians**, actuator angle **0.16 radians**. The source formulas repeat after 20 seconds, although its published film covers one ten-second half-cycle. Changing `--time` rebuilds a display snapshot; it does not simulate the mechanism.

The source declares scene units without a physical conversion. The default **50 mm per scene unit is an explicit example assumption**, giving a leaf vertical thickness of 0.0325 mm. The source's `cant` is a dimensionless slope. Material colors and densities are illustrative assumptions recorded in each build; they are not material specifications or measured mass properties.

## Exact source geometry and explicit limits

[source-geometry.json](source-geometry.json) identifies the originating lines, constants, source commit and SHA-256 hashes. The source is [`cybrdelic/cybr-iris09` at `6aac7159`](https://github.com/cybrdelic/cybr-iris09/blob/6aac71594c211cc5934674b2113d9db8c5ec17c6/IRIS-09.glsl). This example directly transcribes:

- Two intersecting circular leaf profiles and the exact canted slab, including the distinction between vertical and normal thickness.
- Nine hinge placements and the source's prescribed leaf/actuator angle formulas.
- Fourteen rounded radial housing profiles, 120 grip flutes, the annular seam, two spanner notches and the curved actuator slot.
- Three bored mounting ears, six recessed slotted screw heads, and the handle, pivot and cap.
- Two closed plano-convex optical volumes, using the source sphere/plane equations.

The thumb knob uses the source's mean radius. Its omitted 28-lobe knurl has radial amplitude 0.0025 scene units, or 0.125 mm at the default scale. Shader grain, engraving, wear, the zero-thickness sensor surface, lighting and refractive appearance are outside this CAD example. Material partitioning preserves the housing's occupied union while avoiding duplicate volume accounting; it does not establish physical assembly interfaces.

## A rigid leaf is different from the displayed clip

The source clips every moving leaf against a stationary world cylinder. That changes the amount of displayed material as the hinge angle changes. This example keeps the distinction in its artifacts:

- `cad/invariant_leaf_template.step` contains the complete, invariant two-circle/slab leaf in its local frame.
- `cad/visual_reference_subassembly.step` and `reference.glb` contain the separately clipped source display at the chosen pose.
- `cad/unclipped_profiles_reference_subassembly.step` and `invariant-profiles.glb` show all nine invariant profiles at their prescribed reference placements, before world clipping.
- `leaf-shape-dag.json` expresses both constructions through the same shared evaluator. `geometry-study.json` records their sampled volumes and explicit body frames.

At the default scale, the invariant leaf volume remains about **230.238 mm³**. The displayed clipped volume changes from about **77.265 mm³** at t=0 to **55.389 mm³** at t=5 and **41.628 mm³** at t=10. These are actual CAD measurements that demonstrate why display clipping cannot be used as rigid-body mass or inertia.

![Unclipped invariant leaf profiles](evidence/invariant-profiles.png)

The authored hinge positions are mathematical placement frames. The source does not supply hinge pins, a solved pin-slot linkage, actuation closure, a validated torque path, contact dynamics or an established tolerance model. The original geometry-study path adds none of those claims; the new derived mechanism above has its own geometry and measured scope. The source's sampled-clearance comment is not transferred into evidence. Neither geometry checks nor the images establish physical buildability.

## Verification and attribution

```sh
python -m pytest tests/test_shape_dag.py tests/test_iris09_geometry.py
```

Tests compare actual CAD point membership with the source's circle/plane equations, frame placement with its hinge basis, volume with analytic annular-profile and spherical-cap formulas, and invariant versus clipped leaf volumes. The supplied evidence images are rendered from this implementation's generated geometry.

The [artifact verification receipt](evidence/verification.json) records the completed default reference build: 26 registered native parts including the separate invariant template, 25 placed reference parts, and 30 solids in the reopened reference STEP. It verifies 96 native artifact hashes, the recipe identity, 25 reference GLB nodes and nine invariant-profile nodes. The three 1280 × 960 images were rendered and visually inspected on Windows with VTK 9.6.2; their actual backend receipts are retained beside the images. The [geometry report](evidence/geometry-study.json) preserves the qualification limits and measured clipping volumes.

The pinned source package supplies no upstream license grant. Its use here follows the owner's request to adapt the source into CYBR GEO; no upstream license grant is invented and no original shader or media files are redistributed. The new adaptation follows CYBR GEO's existing GPL-2.0-only licensing.
