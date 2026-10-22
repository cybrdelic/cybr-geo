# Fresh native ARX belt engineering evidence

![Native geometry at integrated states](motion.gif)

This is the authored belt example built through GEO's shared native construction,
physical-model, control and replay paths. The images use freshly generated native
geometry at actual integrated MuJoCo body poses. They do not reuse the source
film or its cached display meshes. No image generation was used.

The nominal ten-second contact-transfer run passed its declared task criteria:
0.258 mm placement error at nest B and 0.310 mm on return to A, with zero imposed
workpiece wrench and no unexpected fault. The puck remains an independent free
body; gripper and support contacts determine its motion. These bounded model
results do not establish continuous clearance, manufacturing suitability or
hardware qualification. The compact ARX and IRIS examples have separate scopes.

The [fresh dynamics summary](fresh-dynamics.json) and
[friction-loss counterfactual](counterfactual-comparison.json) retain the causal
comparison: the adverse contact case fails placement instead of inheriting the
nominal pass. [Native readback](native-readback-verification.json),
[canonical receipt verification](canonical-receipt-verification.json) and
[STEP coverage](step-coverage.json) cover the fresh geometry and all 385 analytic
CAD components. The aggregate STEP export succeeded after filename handling was
corrected. A raw receipt-file byte hash and a canonical JSON semantic hash are
different identities; the records retain that distinction.

[The still](pose.png) is the recorded 2.2-second state. The GIF holds 82 selected
recorded states from the 1,201-frame run. The committed GIF is resized to 800 × 600 with all frame timings preserved; the full-resolution output remains in the run directory. No joint animation or motion
interpolation is substituted. [Media metadata](media.json) records exact sampled
times, camera settings, identities and file hashes. The full run is
`outputs/engineering-arx-default`, using the verified fresh native build in
`outputs/engineering-arx-native-final`.

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
rejection without starting a duplicate physics job.

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
  --trace outputs/engineering-arx-default/simulation/trace.json.gz \
  --native outputs/engineering-arx-native-final \
  --out outputs/engineering-arx-replay/nominal --render
```

The shared renderer produces actual PNG/GIF and a static GLB. The browser replay
probe commands are in [Studio's source README](../../../studio/web-auth/README.md).
