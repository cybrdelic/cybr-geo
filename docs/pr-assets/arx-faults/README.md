# ARX operating policy and fault evidence

The maintained ARX example uses the shared CAD, electrical drive, contact and
MuJoCo runtime. Its authored operating policy retains the source TCP path,
forward tilt, roll and endpoints, removes lateral transfer tilt, and maps the
source task to 17.4 seconds. Motor, brake, spring and belt ratings and the
original source acceptance limits are unchanged.

The change addresses an observed mechanical demand. The earlier timing-only
policy still asked the wrist brake to resist more gravity torque than its
dynamic braking and dry-friction capacity. Further slowing that same posture
did not remove the deficit. Changing the requested transfer orientation reduces
that demand; it is an explicit task change, not an increased actuator rating.

The [diagnostic archive](diagnostic-17.4s/README.md) preserves the three original
ordinary failures, the subsequent failed timing-only suite, refinement runs,
rejected slower candidates and the associated static-load calculation. Failed
results remain failed. Archive source snapshots identify the code that produced
those diagnostics and are not maintained alternative implementations.

[Nominal motion and its identities](MEDIA.md) show generated native geometry at
recorded solver states. The independent puck is moved by contact. These images
do not establish internal shaft rotation or continuous clearance.

The [final numeric packet](final/README.md) retains **43/43 ordinary passes and
six correctly rejected adverse controls**. All five selected half-timestep
cases pass. The smallest estimated transmitted J2 torque margin is 0.076717 Nm
at the original step and 0.076687 Nm at half step; motor torque reaches its
configured clamp. A contact-sensitive J5 peak changes by 0.916136 Nm in the
weak/delayed 3.2-second case. These results establish the stated outcomes at
two timesteps, not whole-system convergence or hardware reserve. All-step
gripper travel stays within the authored limits in both runs.

To repeat the maintained fault requirements after a fresh build:

```sh
lab mechanism arx_belt --step
python tools/check_arx_faults.py --native outputs/arx_belt/native --output outputs/arx-faults --workers 3
```

Use a new output directory. The command retains all 28 source scenarios and
adds matched-phase executions for timed interventions, yielding 49 distinct
runs. Ordinary criteria and deliberately adverse controls have different
expected outcomes; a correctly rejected adverse control is not a successful
transfer. Receipts preserve original numeric checks, placement outcomes,
all-step extrema, trigger times, source identities and actual trace hashes.

The source scenarios are simulation requirements, not supplier or hardware
qualification. In particular, passing the arm's stop criteria does not establish
payload retention: loss of electrical power can release the gripper and drop
the puck. Complete motion-envelope clearance, production tolerances, supplier
fits, belt pretension and structural, fatigue and thermal qualification remain
outside these results.
