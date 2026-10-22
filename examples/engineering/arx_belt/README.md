# ARX belt example of shared CYBR GEO engineering

This is the actual source-derived ARX belt geometry and task. Core construction,
mass accounting, contacts, motors, faults, simulation and replay live in
`mechanism_lab`, not in another ARX runtime.

```bash
lab mechanism arx_belt --step --media
lab preview arx_belt --step
python tools/run_arx_causal.py --native outputs/arx_belt/native --case friction_loss --output outputs/arx-friction-loss
python -m examples.engineering.arx_belt.verify_geometry --trace outputs/arx_belt/simulation/trace.json.gz --native outputs/arx_belt/native --out outputs/arx-replay --render
```

`design.py` uses the shared native builder and checks source nominal mating
interfaces before completion. `contacts.py` supplies authored material-region
layout to shared proxy constructors. `model.py` supplies the physical tree and
drive configuration. `task.py` supplies requested motion, and `project.py`
registers the project and its contact-placement outcome. `geometry.py` binds
actual native parts to recorded physical bodies for existing exports/renderers.

Every default generation rebuilds geometry and its mass properties. Explicit
`--reuse-native` requires matching spec/recipe/core and all file hashes. Shipped
historical mass records are comparison fixtures only, used by the standalone
counterfactual helper when `--native` is omitted; that output is labeled cached.

The puck is a free body. Nominal success and friction-loss failure concern the
specified ten-second simulated task, not production hardware. Source ordinary
fault failures remain unresolved. Contact overcoverage, omitted flexible-part
contact, supplier fits, pretension, complete motion-envelope clearance and
structural/fatigue/thermal qualification remain limits.

See [architecture](../../../docs/CONSTRUCTIVE_MECHANISMS.md),
[task/source comparison](TASK.md) and [notices](../../../NOTICE-SOURCES.md).
