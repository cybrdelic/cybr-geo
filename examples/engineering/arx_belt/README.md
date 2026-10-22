# ARX belt example of shared CYBR GEO engineering

This is the actual source-derived ARX belt geometry and task. Core construction,
mass accounting, contacts, motors, faults, simulation and replay live in
`mechanism_lab`, not in another ARX runtime.

```bash
lab mechanism arx_belt --step --media
lab preview arx_belt --step
python tools/run_arx_causal.py --native outputs/arx_belt/native --case friction_loss --output outputs/arx-friction-loss
python -m examples.engineering.arx_belt.verify_geometry --trace outputs/arx_belt/simulation/trace.json.gz --native outputs/arx_belt/native --out outputs/arx-replay --render
python tools/check_arx_faults.py --native outputs/arx_belt/native --output outputs/arx-faults --workers 3
```

`design.py` uses the shared native builder and checks source nominal mating
interfaces before completion. `contacts.py` supplies authored material-region
layout to shared proxy constructors. `model.py` supplies the physical tree and
drive configuration. `task.py` retains a source-default motion oracle; `project.py`
applies the authored orientation and timing policy in `data/default-task.json`,
binds it through GEO's shared phase clock and
registers the project and its contact-placement outcome. `geometry.py` binds
actual native parts to recorded physical bodies for existing exports/renderers.

Every default generation rebuilds geometry and its mass properties. Explicit
`--reuse-native` requires the source binding issued by a successful fresh
construction: all project Python/data files, shared construction code, CAD
dependencies, specification and native artifact hashes must still match.
Earlier unbound native folders must be rebuilt through `lab mechanism`; they
cannot be relabeled from the current source. Shipped
historical mass records are comparison fixtures only, used by the standalone
counterfactual helper when `--native` is omitted; that output is labeled cached.

The puck is a free body. The default takes 17.4 seconds along the original TCP
path, with lateral transfer tilt removed to reduce wrist-brake gravity load.
Forward tilt, roll formula and endpoints remain source-derived. Transfer runs
at half rate and loaded descent at one fifth rate. The source's
ten-second oracle remains separately available; it is not relabeled as the new
default. Source and physical task times share one map, including placement and
fault triggers. The shared controller receives scaled velocity and acceleration
targets, while motor, brake, spring and belt ratings remain unchanged.

The fault command runs all 28 source cases, retaining original-clock and
phase-matched executions for timed faults. Its output directory must be new.
The report keeps original numeric acceptance, additional transfer outcomes,
measurement coverage and expected adverse outcomes separate. Use
`--case power_loss_3.2 --clock matched-phase --timestep-scale .5` for a bounded
refinement run; this changes the integration step, not physical ratings or
acceptance limits. [Recorded nominal motion](../../../docs/pr-assets/arx-faults/MEDIA.md)
comes from the same fresh default construction.

These are specified simulated tasks, not production hardware. Loss of power
still removes gripper torque and can drop the puck; passing an arm stop check
does not establish payload retention. Contact overcoverage, omitted flexible-part
contact, supplier fits, pretension, complete motion-envelope clearance and
structural/fatigue/thermal qualification remain limits.

See [architecture](../../../docs/CONSTRUCTIVE_MECHANISMS.md),
[task/source comparison](TASK.md) and [notices](../../../NOTICE-SOURCES.md).
