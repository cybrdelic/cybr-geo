# Rejected timing-only policy

This packet preserves the unsuccessful intermediate P5 operating policy. It
slowed the source trajectory to 17.4 seconds without changing transfer
orientation. It is diagnostic evidence, not acceptance of the current default.

The complete 49-execution suite finished with 42 of 43 ordinary cases passing
and all six adverse-control rows producing their expected rejection. The
remaining `power_loss_7.2--matched-phase` case failed the unchanged 0.002 rad/s
settling limit: J4 still moved at 0.254242 rad/s after the 0.5-second deadline.
A half-timestep run retained the failure. Further timing reductions at that
same orientation also failed.

The static-load receipt measures 1.478544 Nm of J4 gravity demand at the fault
pose, above its 1.32 Nm dynamic brake plus 0.012 Nm dry friction. These numbers
describe the declared model. They do not qualify a physical brake or payload
retention. The later policy removes lateral transfer tilt instead of increasing
ratings or relaxing acceptance limits.

`full-suite-summary.json.gz` and `refined-suite-summary.json.gz` preserve exact
raw reports. The uncompressed versions replace repeated reference objects with
verified pointers into their matching input files; no numeric check or outcome
is removed. The manifest records both compressed and original byte hashes.

Verify this packet from any directory using Python's standard library:

```sh
python docs/pr-assets/arx-faults/diagnostic-17.4s/inspect_fault_evidence.py
```

The expected readback is **complete but passed: false** for both suites. The
packet also retains immutable baseline reproductions of the three original
ordinary failures at commit `1c216fdaebf14c68c0a0cb5dde794cd4ada7a06a`.
`inspect_fault_evidence.py --restore-baseline PATH` restores that diagnostic
code and its inputs without overwriting existing files and prints its command.

`curate_faults_packet.py` and `readback_final_suite.py` are archived execution
scripts whose exact bytes are retained. They originally lived under
`outputs/fault-diagnosis/`, where their repository-relative paths resolve.
They are not portable entry points from this relocated documentation folder.
Use the inspector above for packet verification and the maintained
`tools/check_arx_faults.py` for a new simulation. The gzip static-load and slower
timing scripts are likewise execution provenance, not substitutes for the
current authored project.

`static-load-snapshot/` adds the executed historical task, phase clock and
physical-model reader snapshots, their explicit runner, and its receipt. This
avoids accidentally using the later policy or core when reproducing the old
static diagnosis. It still
requires the full original diagnostic run and its large traces, which are not
included in this compact packet. The recorded command and original paths are
retained as execution provenance; the standard-library inspector verifies the
packaged bytes without those external run files.
