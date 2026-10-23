# ARX belt example of shared CYBR GEO engineering

This is the actual source-derived ARX belt geometry and task. Core construction,
mass accounting, contacts, motors, faults, simulation and replay live in
`mechanism_lab`, not in another ARX runtime.

```bash
lab mechanism arx_belt --step --media
lab preview arx_belt --step
lab replay arx_belt --run outputs/arx_belt --out outputs/arx-film
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
the project to shared `native_replay` for existing exports/renderers. Both handed
jaws are native placements: `place_assembly` integrates and exports the same
located component solids, with no separate replay reconstruction or mirrored
mass fallback. Only the free puck and four model-bound nest solids are scene
additions. `lab replay` saves full-resolution PNG/MP4 with consumer/source and
actual frame-selection receipts.

The shared `RackPinionTravel` constructor places both P5 gripper racks from
one module, 50-tooth span and pinion offset, and rejects a slide range that
would lose its declared tooth-span edge margin. The source permits 10–40 mm
travel; at 40 mm the nominal pinion pitch point remains 4.888 mm from the
outermost usable pitch-line flank. This accounts for the quarter-pitch inset
of the first and last sampled teeth from the rack blank edges. The default
authoring target is one full 0.9425 mm pitch plus one further pitch of
geometric reserve, leaving 3.003 mm beyond that target.
The reserve is a design assumption, not a supplier rating. The compiled
opposed-jaw equality, tendon pitch conversion and controller radius must match
the constructed pinion; the source-bound native receipt records this contract.
Rack pitch-line Y placements also derive from the 3 mm pinion pitch radius.
The P5 example pins module 0.3, 20 pinion teeth, 50 rack teeth and its 2 mm
end datum because the existing bridge, jaw passages and supports were built
around them; changing those values requires a coupled design revision. This
proves nominal tooth-span coverage only, not rolling flank contact, gear
strength, backlash, bushing fit or production capability.

Fresh GEO construction now puts that same source pinion in its own native
`pinion` body and binds its integrated CAD inertia to a `gripper_pinion` hinge.
Both slides are coupled to the hinge with the source 3 mm pitch radius, and
the existing motor applies torque directly to the hinge. The authored home
opening, 10–40 mm jaw travel and initial world geometry are preserved. The
shared simulator limits numerical jaw/pinion displacement error to 0.1 mm;
this is a solver design budget, not measured tooth backlash. The archived P5
XML and cached mass path remain available for historical comparison. The
earlier recorded films and fault numbers do not validate this revised drive;
its actual native rebuild and loaded fault outcomes must be checked afresh.

Every default generation rebuilds geometry and its mass properties. Explicit
`--reuse-native` requires the source binding issued by a successful fresh
construction: all project Python/data files, shared construction code, CAD
dependencies, specification and native artifact hashes must still match.
The diagnostic builder also reads
[qualification-intent.json](data/qualification-intent.json) before placing
parts. It binds all 380 installed part identities and material keys, eight
source-derived spring pin/clevis pairs, and seven loaded drive parts to that
construction. Null process, wall and variation fields mean unresolved
engineering requirements; the resulting intent receipt is **not** a qualified
construction receipt. The 52 source nominal template/fixture checks are kept
separate from exact installed part-pair evidence. This example cannot pass
`lab release` without a complete qualified construction plan and the
serial-bound physical measurements described in the
[release contract](../../../docs/MANUFACTURING-RELEASE.md).
Earlier unbound native folders must be rebuilt through `lab mechanism`; they
cannot be relabeled from the current source. Shipped
historical mass records are comparison fixtures only, used by the standalone
counterfactual helper when `--native` is omitted; that output is labeled cached.

If a complete `data/qualification-plan.json` is supplied, ARX now checks it
against that exact intent **before exporting any CAD**. The one source geometry
recipe then collects its 380 placed solids in memory and hands them directly
to `QualifiedNativeConstruction`. The strict builder checks each declared
material/process and exact initial-positioned STEP interface. It derives
physical mass and inertia from those placed CAD shapes, without importing the
diagnostic source's catalog mass substitutions, spring mass allocations or
cable/fastener reserves. Its initial pose map comes from the pinned MJCF home
state, including the revised pinion at tool-local Y=201.5 mm and both original
jaw world poses. Native `fixed`, `jaw`, `jaw_opposed` and `pinion` map to
physical `fixed_hardware`, `jaw_left`, `jaw_right` and `gripper_pinion`.

**The current ARX intent still rejects a resolved plan.** Its installed
part-to-part interface/load graph is incomplete, and the native and physical
power-loss retention mechanism is not yet constructed. Those are design
requirements, not fields to fill with invented supplier numbers. After they
are resolved in source, a fresh strict build still needs actual material lot,
process capability, part measurements, tested joint capacities and assembled
service/fault results before `lab release` can succeed. The strict CAD-density
mass basis also needs a fresh physical model and runtime verification; the
original vendor-envelope masses cannot become measured values by declaration.

The puck is a free body. The default takes 18.05 seconds along the original TCP
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

A passive jaw-return spring cannot be silently added to claim power-loss
retention. The current source has a 0.05 Nm gripper output torque limit and a
3 mm pinion radius, or at most 8.33 N of ideal opening force per jaw. Its
modeled slide friction is 2.687 N per jaw. At the approximately 19 mm grasp
position, the 81.939 g puck and assumed 0.65 pad friction require at least
0.618 N normal force per jaw for static gravity alone. A return spring with
zero load at the 10 mm closed stop would need at least 367 N/m to overcome
that slide friction and supply the grip force; by 40 mm opening it would
oppose the motor with at least 11 N per jaw before slide friction. A
preloaded low-rate spring or normally engaged brake needs a coupled native
CAD, actuator and contact design, plus sourced component capacity and fault
tests. The nominal pinion journal occupies Z 3-10 mm alongside the front
bushing/post at Z 4-9 mm, and the front wall occupies approximately
Z 14-16 mm. A spring-applied pinion brake cannot be added there as an
invisible friction coefficient or lock; its rotor, reaction mount, release
device, finite torque and clearances need a joint packaging redesign. The
current film and fault report do not contain that design. The new pinion hinge
is a prerequisite for a causal brake, not a brake or a retention claim by
itself. A candidate brake needs a source-bound compatible package, keyed
rotor, stator reaction into the bridge, thrust support, finite engagement
timing and loaded power-loss contact validation before the design is usable.

See [architecture](../../../docs/CONSTRUCTIVE_MECHANISMS.md),
[task/source comparison](TASK.md) and [notices](../../../NOTICE-SOURCES.md).
