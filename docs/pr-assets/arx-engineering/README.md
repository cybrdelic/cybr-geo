# Fresh native ARX belt engineering evidence

![Native geometry at integrated states](motion.gif)

This is the authored belt example built through GEO's shared native construction,
physical-model, control and replay paths. The images use freshly generated native
geometry at actual integrated MuJoCo body poses. They do not reuse the source
film or its cached display meshes. No image generation was used.

This packet was regenerated after the full source-binding correction. The fresh
public run verifies helper code and authored data as well as the design entry
point. Its model identity begins `627c25b5`; the native source-binding identity
begins `52f6276e`. Exact identities and fresh trace hashes are in the receipts.
The [native build record](source-bound-native-build.json) and
[complete source binding](source-bound-native-binding.json) preserve those
inputs. The [source-closure comparison](source-closure-comparison.json) verifies
that both new nominal and friction-loss frames and metrics exactly match their
earlier measured runs, while their corrected source identities differ.
The earlier packet is retained only in the local, untracked directory
`outputs/engineering-arx-before-sourcebinding/curated`; it is not part of this
published evidence packet.

The nominal ten-second contact-transfer run passed its declared task criteria:
0.258 mm placement error at nest B and 0.310 mm on return to A, with zero imposed
workpiece wrench and no unexpected fault. The puck remains an independent free
body; gripper and support contacts determine its motion. These bounded model
results do not establish continuous clearance, manufacturing suitability or
hardware qualification. The compact ARX and IRIS examples have separate scopes.

The [gripper travel audit](gripper-travel-audit.json) checks both jaw coordinates
against this pinned belt model's 10–40 mm limits. Across 1,201 recorded states
per case at 120 Hz, the nominal minimum was 18.9794 mm and the friction-loss
minimum was 16.9912 mm; both maxima were 38 mm, with no recorded limit violation.
This is a saved-state audit, not a check of every 2,400 Hz integration step or
a continuous travel proof. It does not change the failed friction-loss task
result or transfer conclusions from a newer compact design to this belt model.

The [fresh dynamics summary](fresh-dynamics.json) and
[friction-loss counterfactual](counterfactual-comparison.json) retain the causal
comparison: the adverse contact case fails placement instead of inheriting the
nominal pass. [Native readback](native-readback-verification.json),
[canonical receipt verification](canonical-receipt-verification.json) and
[STEP coverage](step-coverage.json) cover the fresh geometry and all 385 analytic
CAD components. [Aggregate interchange readback](aggregate-step-readback.json)
reopened 392 valid positive-volume STEP solids and compared their bounds with
the 385-component GLB: the maximum difference was 0.00443 mm. A raw receipt-file
byte hash and a canonical JSON semantic hash are
different identities; the records retain that distinction.

[The still](pose.png) is the recorded 2.2-second state. The GIF holds 82 selected
recorded states from the 1,201-frame run. The committed GIF is resized to 800 × 600 with all frame timings preserved; the full-resolution output remains in the run directory. No joint animation or motion
interpolation is substituted. [Media metadata](media.json) records exact sampled
times, camera settings, identities and file hashes. The full run is
`outputs/engineering-arx-sourcebound`, using the verified fresh native build in
`outputs/engineering-arx-sourcebound/native`.

[Native replay checks](replay-check.json) validate all 385 displayed CAD parts,
62,900 vertex comparisons across five recorded poses (zero transform error),
exact cache rebinding, and native CAD/mesh bounds agreement within the declared
tessellation allowance (maximum 0.13933 mm). [The Three.js check](three-replay-check.json)
loads the actual GLB and compares 5,775 transformed vertices against independently
evaluated SI solver poses: maximum error 0.00001444 mm, consistent with float32
mesh storage. Imported/forged flags and stale identities cannot enable a passing
browser receipt. This programmatic check is separate from live UI verification;
the available computer-use inventory had no browser surfaces.
The [local Studio HTTP check](studio-http-check.json) verifies the authored
project default, built assets, compatibility compilation and missing-token
rejection without starting a duplicate physics job. That HTTP check and the
older component/WASM comparisons predate this source-binding refresh; they are
route/compatibility evidence, not newly regenerated native dynamics receipts.

Four deforming spring-coil visuals, flexible cables and the workbench surface
are omitted; their declared forces/contact proxies remain in the physical model.
Input shafts, drive pulleys, pinion and belt teeth retain their native local
phase because separate material/rotor display states are not recorded. The puck
has a visual bore while the declared contact model uses a cylinder. Display
finishes are source appearance, not measured materials. These limitations are
also recorded in the machine-readable receipts.

Reproduce from the repository root after generating the fresh nominal project:

```sh
python -m examples.engineering.arx_belt.verify_geometry \
  --trace outputs/engineering-arx-sourcebound/simulation/trace.json.gz \
  --native outputs/engineering-arx-sourcebound/native \
  --out outputs/engineering-arx-sourcebound-replay --render
```

The shared renderer produces actual PNG/GIF and a static GLB. The browser replay
probe commands are in [Studio's source README](../../../studio/web-auth/README.md).
