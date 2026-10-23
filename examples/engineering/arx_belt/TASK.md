# ARX belt reference task and source baseline

`task.py` ports the reference motion from the source controller at commit
`aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2`. It belongs to this example, not to the
shared engineering core. Pass the pinned source's `parameters.json` mapping:

```python
from examples.engineering.arx_belt.task import reference

target = reference(time_seconds, parameters)
# q: six radians; qd: rad/s; qdd: rad/s²
# gap: commanded jaw half-gap in metres; stage: source stage 0..7
```

The API accepts arbitrary finite times, including negative times and subsequent
cycles. It preserves the original ten-second modulo, inverse-kinematics branch
choices, stage boundaries, switched jaw half-gap, and centred derivative stencil
with a 0.00025-second step. It neither unwraps angles nor changes the reference
to suit a solver. The task outputs controller targets; it does not prescribe
actual body motion or attach the workpiece.

The default project authors its operating policy in `data/default-task.json`.
It removes the source's lateral wrist tilt during transfer while retaining its
TCP path, endpoints, forward tilt, roll formula and inverse-kinematics branch.
This reduces gravity loading on the wrist brake during a transfer stop; it is
an explicit change to requested orientation, not increased brake capacity.
Calling `task.reference` without the optional policy argument still executes
the original source oracle above.

The project applies the shared `mechanism_lab.reference_clock.PhaseClock` to
that authored reference. Source and physical knot times declare a complete
default cycle of 18.05 seconds.
Transfer uses rate 0.5 and loaded descent rate 0.2. Requested velocity scales
with rate and acceleration with rate squared. Every changed-rate join is
screened at two probe widths for continuity and stationarity before simulation;
the source stencil's small numerical residuals are retained with explicit
tolerances in `reference_clock_validation`. This is a bounded numerical screen,
not an analytic trajectory certificate.

The same map converts the source placement landmarks (4.9 and 9.9 seconds) and
fault phases into physical times. Source scenario names retain their original
time labels; receipts state both the requested phase and actual trigger time.
Original-clock checks and matched-phase checks are distinguished. The operating
policy changes requested speed and transfer orientation. Motor, brake, spring
and belt ratings and source fault thresholds remain unchanged. The plain
`task.reference` API above remains the source oracle.

Run the independent Python/JavaScript comparison from the repository root:

```sh
node examples/engineering/arx_belt/verify_task.mjs "$ARX_SOURCE_CHECKOUT" python outputs/arx-belt-source-baseline/reference-parity.json
```

The probe imports the original `controller.mjs` directly from the pinned
checkout. It compares all 24,001 integration-grid times from 0 through 10 seconds
at 2,400 Hz, plus phase/gap boundaries on both sides, negative times and later
cycles. The receipt records source and task hashes and numerical errors. Small
cross-language math-library differences are amplified by the original second
difference; the actual observed error is retained rather than rounded to zero.

The separate source baseline executes the original validator unchanged. Set
`ARX_VALIDATION_OUT` to a distinct directory for each case, so archived results
remain untouched. For example, in PowerShell:

```powershell
$env:ARX_VALIDATION_OUT = '<repo>/outputs/arx-belt-source-baseline/nominal'
node '<source>/ARX-6-P5-Engineering/integration/validate_p5.mjs' nominal
$env:ARX_VALIDATION_OUT = '<repo>/outputs/arx-belt-source-baseline/friction_loss'
node '<source>/ARX-6-P5-Engineering/integration/validate_p5.mjs' friction_loss
$env:ARX_VALIDATION_OUT = '<repo>/outputs/arx-belt-source-baseline/no_counterbalance'
node '<source>/ARX-6-P5-Engineering/integration/validate_p5.mjs' no_counterbalance
```

The third case is an additional diagnostic. Summarize these retained raw reports:

```sh
node examples/engineering/arx_belt/summarize_baseline.mjs "$ARX_SOURCE_CHECKOUT" outputs/arx-belt-source-baseline examples/engineering/arx_belt/evidence/source-baseline.json
```

In the recorded execution, nominal passed the original validator and placed at
B within 0.25048 mm. `friction_loss` also passed its original general checks,
which omit nominal placement criteria. Its saved frames nevertheless show the
puck leaving the pads after friction changes at 2 seconds and missing B by
78.15898 mm. The receipt preserves both facts; it does not relabel the original
validator outcome. Counterbalance removal failed seven original checks.

These are original-source simulation baselines for a later comparison with the
shared runtime. They do not validate that runtime, the distinct compact ARX
design, or hardware. The source's workpiece remains independently simulated;
no attachment or body-state writes were added. Raw trajectories remain under
`outputs/arx-belt-source-baseline/`; compact receipts are in `evidence/`.

## Native geometry at integrated poses

`geometry.build_assembly(native_directory, simulation_result)` is this example's
adapter to the shared `mechanism_lab.physical_replay` engine. It uses all 375
fresh native placements, the five exact handed opposing-jaw parts, a free puck
and four nests from the run's parameters/physical source. All 385 parts retain
analytic BReps by default, so the existing STEP exporter stays available.
The puck follows its independently integrated body; it is never attached to the
tool. Source body-local millimetres and SI Y-up poses are converted once into the
core's millimetre/Z-up convention.

The result must contain `task_parameters` and `physical_source` from the run,
and the physical source must match `model_sha256`. Cached replay checks its
recorded transform digest and each part's body binding. It holds recorded states
between samples: it does not solve a prescribed trajectory or interpolate an
unobserved clearance claim.

After running the shared engineering project, verify/render its trace:

```sh
python -m examples.engineering.arx_belt.verify_geometry \
  --trace outputs/arx_belt/simulation/trace.json.gz \
  --native outputs/arx_belt/native \
  --out outputs/engineering-arx-replay/nominal --render
```

The check compares body transforms and units against actual source frame poses,
validates every native BRep, checks native/display local bounds and restores
cached motion. `replay-check.json` records numerical errors and identities.
`replay-evidence.json` binds generated PNGs, GIF and static GLB to those poses.
The GIF displays a subset of actual recorded states; no source film is reused.

Four deforming coil visuals and the flexible harness are omitted. Input shafts,
drive pulleys, gripper pinion and belt teeth retain native local phase because
those separate material states are not recorded as bodies. The workbench visual
is omitted while its simulation contacts remain active. The puck bore is visible
native geometry while its authored collision primitive is a cylinder. These
display scopes do not establish hardware qualification, exact contact geometry,
or continuous clearance.
