# Four-bar lift: a new shared-core consumer

This example is a newly authored vertical-plane crank-rocker. A single
electrical motor drives a 25 mm crank through one full turn. A 90 mm bored
coupler drives the passive 60 mm rocker. The rocker's integral steel load lobe
is 85 mm from its ground pivot, so the native CAD mass is lifted against
gravity through the crank, pins and coupler. The fixed pivots are 100 mm apart.

It uses the same `EngineeringProject` protocol, `NativeConstruction`,
`PinnedLink`, `PhysicalModel`, `CausalSimulation`, native replay and exporters
as the ARX and IRIS clients. It adds no CAD evaluator, inertia engine,
controller kernel, dynamics solver or renderer. Only its layout, nominal
parts, motor specification and task belong to this example.

## Run and edit

Use the normal [platform installation](../../../docs/PLATFORMS.md), then:

```sh
lab mechanism fourbar_lift --step --out outputs/fourbar_lift
python -m examples.engineering.fourbar_lift.verify --output outputs/fourbar_lift --controls
```

From a source checkout, the following runs the same shared pipeline and
exports, then performs independent readback and both causal controls:

```sh
python -m examples.engineering.fourbar_lift.verify --build --controls --output outputs/fourbar_lift
python -m pytest tests/test_fourbar_lift.py --basetemp outputs/fourbar-test-temp
```

`data/specification.json` is the design input. Its body dimensions, declared
motor parameters, timing and acceptance limits enter the source identity.
The strict Grashof and pin-wall rules reject unreachable or unmated edits
before construction. The full-turn rocker extreme supplies conservative
swept bounds for the extended rocker and load lobe against the input
mechanism; a separate axial bound retains backplate clearance. These reject
oversized load edits before CAD generation. The current bearing/retainer stack fixes shaft and pin
diameters and axial planes; changing those requires a coordinated geometry
edit, not only a different value in the solver.

## Actual mechanical and manufacturing path

The generated 15-part native assembly includes a machined backplate and
bearing supports, two bronze bushings, a motor reaction housing and four
mounting screws, two steel shaft/link/pin pieces, four grooved retainers and
the bored coupler. Moving bodies use density-integrated CAD mass, center of
mass and full inertia. There are no mass overrides. Colored finishes on the
crank and rocker retain the same physical steel density.

The shaft/bushing diametral gap is 0.10 mm. The shared `PinBoreFit` derives
5.15 mm coupler bores around 5 mm pins from a 0.05 mm required service radial
gap, authored 0.01 mm radial variation budgets for each mating part and a
further 0.005 mm design reserve. The resulting nominal radial gap is 0.075 mm;
the worst gap under those *declared design targets* is 0.055 mm, leaving the
positive reserve beyond service. No process or supplier has demonstrated the
variation budget or reserve. The native pin-clearance requirement measures the nominal
gap independently. Coupler shoulders and
keepers leave 0.30 mm axial gaps. Actual grooves clear the split retainers.
The nominal screw shafts fit their matching holes; thread engagement,
preload, bushing retention and an external mounting fixture are not modeled.
The axial and other radial interfaces still need complete measured production
tolerance stacks. Changing any fit input invalidates prior CAD, mass/inertia,
interface, simulation and media receipts until regenerated.

One motor actuator receives voltage/current-derived torque. The rocker and
coupler have passive hinges; the shared material-pin closure transmits their
forces. The circle-intersection equation selects the initial construction
branch and serves as an independent check. It does not prescribe any follower
state or render pose. The world gravity vector is converted into the renderer's
Z-up basis once, together with the recorded body poses.

`fourbar-readback.json` records independently reopened STEP/GLB coverage,
component mass/inertia exchange checks, source/runtime identities and 65
sampled native-solid poses from the integrated trace. Nearby cross-body pairs
use exact BREP intersection and distance; disjoint bounding boxes reject
separated pairs. This finite audit cannot certify continuous-time clearance.
The complete every-step joint and pin-closure measurements remain in the
simulation receipt.

`causal-controls.json` records two separate numerical interventions. Disabling
motor power prevents the requested crank cycle. Removing the coupler closure
disconnects the loaded rocker from the motor and fails the task. The latter
is an intentionally broken numerical model, not a new physical assembly.
Neither intervention qualifies an emergency-stop design.

## Qualification boundary

This is a connected nominal design and an integrated load-bearing model,
with explicit assumptions that remain to be established for manufacture:

- The direct-drive motor constants are assumptions, with no selected vendor
  motor or reconstructed electromagnetic internals. Its housing represents
  the declared mechanical torque/reaction interface.
- Ideal pin constraints carry forces. Bearing contact pressure, elasticity,
  shaft/pin stress, friction, wear and retainer strength are unqualified.
- Nominal gaps are not production tolerances. Supplier fits, process spread,
  surface finish, preload and mounting stiffness require measured inputs.
- Model current and torque limits are not measured motor performance, thermal
  certification, whole-assembly strength or a safe hardware load rating.

Render evidence must use the accepted integrated trace and native parts.
Changing the geometry or physical inputs invalidates that evidence until the
shared pipeline is rebuilt and the relevant checks are rerun.
