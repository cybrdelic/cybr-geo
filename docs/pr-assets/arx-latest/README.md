# Actual Refined/P6B source in the shared GEO core

These packets use the published source at compact commit
[`85bd6d2ce5b7c567d08b3ffccf2a9be46005aabf`](https://github.com/cybrdelic/cybr-arx6-compact/tree/85bd6d2ce5b7c567d08b3ffccf2a9be46005aabf).
They are separate from the [P5 belt dynamics](../arx-faults/README.md).
No film or source render substitutes for generated CAD evidence.

| Client | Shared construction | Evidence scope |
| --- | --- | --- |
| [Refined](refined/README.md) | `PartGeometryResolver`, existing CSG DAG, native properties and exporters | All 278 source records, 123 explicit BREP replacements, saved static frame and changed-part property reconciliation |
| [Print first](print-first/README.md) | `RoundedBoxTube`, `SplitLapSplice`, `InvoluteRackPair` and `NativeConstruction` | Four structural and two tooth test articles, nominal mates, density integrals and native/print-local exports |

![Actual P6B test articles from shared native CAD](print-first/print-first-preview.png)

The resolver binds each full source part record to verified BREP bytes in a
body-local millimetre frame. It applies the body transform once, retains CSG
for unselected records and separates physical density from display finish.
The reusable joint and gear modules carry dimensions, coupled interfaces and
explicit conversion equations; example clients retain source-specific layout.
There is no additional solver, inertia engine or renderer for either client.

Reproduce with the normal installed CAD environment:

```sh
python -m examples.engineering.arx_compact.refined --output outputs/arx-refined --render
python -m examples.engineering.arx_compact.print_first --output outputs/arx-print-first --render
python .github/scripts/check_latest_arx_geometry.py --output outputs/arx-latest-ci --timeout 900
```

The final command constructs fresh outputs and independently reopens them in
separate processes. Its explicit `--check-existing` mode only verifies local
saved outputs and records `constructed_fresh: false`; CI uses fresh mode.
[Installed-wheel commands and editable specifications](../../../examples/engineering/arx_compact/README.md)
use the same maintained clients and packaged data without a benchmark checkout.

The Refined static frame does not rerun the inherited contact capture, update
assembly mass overrides or establish mechanics for changed geometry. P6B
articles do not include a complete printed arm, printed reducer internals,
modeled fastener stacks or a load rating. The coarse gear pitch requires
changed surrounding interfaces and actuator scaling. Whole-arm FEA, supplier
fits, thermal/creep/fatigue performance and printing remain unqualified.
Missing P6A controller/thermal source is not reconstructed from its images or
replacement-property records.
