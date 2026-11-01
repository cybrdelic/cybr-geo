# ARX-6 compact source geometry

This example evaluates all **278 authored compact parts** with GEO's shared
`ShapeEvaluator`, including both helical springs. It is a separate design from
the belt-drive example. It creates analytic CAD at the actual source capture's
first pose; it does not run or reconstruct the source's cinematic controls.

From the repository root, in the environment installed with the CAD extra:

```powershell
python -m examples.engineering.arx_compact.design --output outputs/arx-compact-reference --render
```

An installed wheel also includes the example. Resolve and run its maintained
script with `mechanism_lab.engineering_source.project_directory`:

```python
import runpy
from mechanism_lab.engineering_source import project_directory
runpy.run_path(str(project_directory('arx_compact') / 'design.py'), run_name='__main__')
```

The output includes `reference.step`, `reference.glb`, individual body-local
STEP files, the shared content-addressed `shape-dag.json`, and
`geometry-study.json`. With `--render`, the PNGs come from the actual CAD mesh
using GEO's renderer. The report records every source part, validity, solid
count, volume, authored versus actual bounds, source status, and artifact hashes.
Native construction can take several minutes; progress prints every 25 parts.
To rerender the saved, hash-checked GLB without repeating CSG evaluation, pass
`--render-only` with the same `--output` directory.

![Actual source compact CAD reference](evidence/reference.png)

The geometry is explicitly unqualified. Supplier reducers and bearings are
interface envelopes, not internal CAD. No mass, inertia, clearance, operating
load, contact force, dynamics, motion envelope or physical assembly acceptance
is inferred from the CSG. Source `mass_override_kg` values remain in the pinned
input but are not promoted to CAD mass measurements.

The source-native spring geometry has defects which this example preserves:
parts 263 and 269 each contain two disconnected solids. Part 263 extends to
local Z = 92 mm although its authored bounds end at 74 mm. An independent run
of only the pinned native builder's `build()` function confirms these results;
see `evidence/spring-native-source-receipt.json`. These findings are not repaired
or hidden by the shared evaluator. The generated report may reveal other
disconnected records or authored-bound overruns.

`assets/design.json` is copied byte-for-byte from
[`cybrdelic/cybr-arx6-compact`](https://github.com/cybrdelic/cybr-arx6-compact/tree/0003382bd6a2e96f3bed065003efec35001982f7)
at commit `0003382bd6a2e96f3bed065003efec35001982f7`.
Its SHA-256 is
`48b3dcf993f78a9e1f386c8458644bb5829d627c895b88e58e616ae14aa565a0`.
The native operation semantics come directly from `source/build_meshes.py`
(SHA-256 `1b2e4f087634f8ff41ad450195a2a35ef3ef6445d1bc25efbad9c797f7fb2e50`).
No separate compact geometry generator is embedded here: its `op/args/children`
records feed the same core used by other examples and legacy component plans.

`data/initial-pose.json` contains only the exact first frame extracted from the
source `assets/capture.json` (SHA-256
`ba938e6a1e0a829c52435a5ef165e7364bf8cb072a604bb18ff0577edf6f1b33`).
The extracted pose JSON is also pinned byte-for-byte with SHA-256
`d8cbe609a96572e99030c152cce89a726a2e681efc2ff97ce651e0a24bb07e07`.
Body-local shape lengths are millimetres; captured translations are metres and
are multiplied by 1000. The final right-handed coordinate change is
source `(x,y,z)` to GEO `(x,-z,y)`. Missing or nonrigid body frames are rejected.

The design also contains 24 shaft-relative `spin` records for rotors, carriers,
brakes and encoders. This static adapter retains those source records but does
not animate them. The retained first-frame capture supplies ten rigid body
transforms, not measured internal shaft states; neither this geometry study nor
the published film establishes internal drivetrain motion or electrical control.

The **278 parts and 10 captured bodies are pinned benchmark assertions**, not
adapter limits. `build()` verifies those counts and the original byte hashes.
The data-only `build_records(output, design, pose, source=..., body_aliases=...)`
entry point accepts a different nonempty record count through the same shared
DAG. It requires explicit source provenance (`repository` and `commit`), records
canonical hashes of the supplied parsed design and pose, and derives captions
and reported counts from those records. Callers must verify and identify their
actual source files; a provenance label alone does not authenticate them.

Frame aliases are explicit. This benchmark passes
`{"fixed": "fixed_hardware"}`; an alias cannot overwrite a captured frame or
refer to an absent frame. Shape lengths remain millimetres and captured
translations remain metres regardless of record count.

This entry point handles the documented CSG records and static pose schema.
Optional `brep_overrides` and `asset_root` pass explicit native replacements
through the shared `PartGeometryResolver`. Each override binds the complete
part record (including body, material and original shape), body-local mm frame,
confined relative path and exact file hash. It replaces the complete evaluated
local root before the one body-to-world transform. It never imports foreign
generator scripts or guesses replacements from filenames. Synthetic
cardinality/units tests establish the adapter contract, not source compatibility.

## Actual Refined source through the same adapter

```powershell
python -m examples.engineering.arx_compact.refined --output outputs/arx-compact-refined --render
```

`refined.py` uses the actual P5 Refined source at
[`85bd6d2ce5b7c567d08b3ffccf2a9be46005aabf`](https://github.com/cybrdelic/cybr-arx6-compact/tree/85bd6d2ce5b7c567d08b3ffccf2a9be46005aabf/revisions/p5-refined).
Its local `data/refined/source.json` retains the upstream Git blob and SHA-256
identities. The source tessellation report selects **123 BREP replacements**;
the other 155 parts use the existing shared CSG evaluator. The source's actual
fillets, chamfers, socket recesses and envelope-preserving cuts remain native
geometry. This rebuild does not claim byte-preservation of the old rendering
mesh cache or use its triangle meshes as substitute CAD.

The default static placement is saved capture frame 300 at **2.5 seconds**.
`--frame-index` selects another actual saved frame; the client does not run the
old controller, interpolate frames or animate the changed geometry. The full
278-part STEP/GLB and optional PNG come from the selected analytic CAD through
GEO's existing exporters and renderer.

`property-reconciliation.json` independently recomputes original CSG and refined
BREP properties for all 123 changed parts: volume, homogeneous-density mass,
CoM, full inertia about CoM, bounds and solid count. It retains source properties,
residuals, explicit tolerances, rejected refinement attempts and artifact hashes.
Aluminum and steel densities are explicit source assumptions. There is no
silent material fallback. The inherited assembly mass records remain separate
and unchanged; the reported geometric mass delta does not update their mass
overrides, inertias or dynamics.

`current-native-body-properties.json` additionally evaluates every selected
part and aggregates current body-local CoM and full inertia through GEO's
shared native mass integrator. It reports both homogeneous source-density
properties and an explicit source-mass-override view. In the latter, purchased
component masses scale the current envelope's uniform-density tensor; this
does not recover supplier internals or measured inertia. Captured world poses
and display finishes do not affect either property calculation.

The source provides no physical density or mass override for its six encoder
PCBs. Those records remain unresolved, and their affected body aggregates are
marked as partial subtotals that cannot supply complete solver bodies. The
client does not fill missing mass from the archived aggregate or silently use
aluminum density. Current selected native properties, explicit incomplete
material inputs, and unchanged archived dynamics are separate facts. The
report preserves disconnected source solids and records numerical comparisons
only where a complete current mass inventory is available. It does not run a
new compact dynamics model or establish structural/load capacity.

The actual Refined `verify_geometry.py` also supplies a useful correction path:
subtract each changed original component's spatial inertia from its archived
body and add the selected refined component. GEO now executes that operation
through shared `replace_mass_components` and publishes
`native-corrected-archival-body-properties.json`. All nine source body records
are compared with the separately hash-pinned upstream
`refined-mass-properties.json`; all 123 changed parts must map to exactly one
body and must retain their original explicit mass policy. These body properties
are corrected to current native geometry. Unchanged component assumptions,
including inputs unavailable for the all-part reintegration, remain inherited.
Neither property path is presented as a rerun of the archived dynamics.

The original comparison follows the source's single-solid `.clean()` step
before integration; the imported replacements are not modified. Native
property bounds use analytic BREP surfaces, independent of cached display
tessellation. The client captures the explicit resolver, CSG evaluator, mass
integrator and client code identities before construction and rejects changes
to those files during the build. The [evidence packet](../../../docs/pr-assets/arx-latest/refined/README.md)
retains the failed first comparison, its source-specific correction and the
final native readback without changing acceptance tolerances.

Archived property agreement uses the original source integration convention.
The independent STEP check separately compares converged adaptive integrals
of each verified source BREP and its STEP export. Differences between the
archived/default and adaptive methods remain explicit in the evidence; those
methods' values are not silently substituted for one another.

Changed geometry does **not** inherit the old capture's mechanical acceptance.
Reducer/bearing envelopes still lack supplier internal geometry; multi-solid
source coils remain reported. There is no new fit, full-motion clearance, load,
thermal, fatigue, printing or hardware qualification. This P5 Refined client
does not stand in for the missing P6-A implementation or P6-B prototype checks.

An installed wheel supports the same maintained script without a checkout:

```python
import runpy
from mechanism_lab.engineering_source import project_directory
runpy.run_path(str(project_directory('arx_compact') / 'refined.py'), run_name='__main__')
```

## P6B print-first articles through shared construction

```sh
python -m examples.engineering.arx_compact.print_first --output outputs/arx-print-first --render
```

This client directly adapts the original P6B builders at the same `85bd6d2c`
source pin. Shared `RoundedBoxTube` and `SplitLapSplice` construct the actual
source joint ends, reinforced walls, 0.30 mm split, 0.20 mm nominal per-side
sleeve fit and coupled cross-hole geometry. Shared `InvoluteRackPair` supplies
the module 1.2 pinion and rack trials. Its sampled radial roots are not generated
trochoids. The editable `data/print-first-spec.json` controls these dimensions;
`--spec` accepts an explicit copy. Inconsistent combinations fail before
accepted export.

Outputs include six source-local native parts, measured density properties and
STEP readback, plus six plate-local STEP and six STL files under `print-local/`.
The plan records nominal mating checks and geometry assumptions. These are
**six test articles**, not a completed printed arm or a rated transmission.
The coarse trial's four-times larger pitch radius requires four-times ideal
torque for the same rack force; it is not a drop-in gripper/controller change.
The solid-fill PETG density assumption, 246 mm planning cube and plate
orientation are not slicer settings or printing qualification. Bolts and
sleeves are candidates and are not modeled.

An installed wheel can run the same client outside the checkout:

```python
import runpy
import sys
from mechanism_lab.engineering_source import project_directory
client = project_directory("arx_compact") / "print_first.py"
sys.argv = [str(client), "--output", "outputs/arx-print-first", "--render"]
runpy.run_path(str(client), run_name="__main__")
```

The Refined and print-first clients return geometry and reports. They do not
implement a dynamics project or the registry's zero-argument
`build() -> Assembly` recipe contract. Use their commands above;
`lab mechanism` remains the shared causal engineering route.

Source reducer envelopes do not contain actual printed reducer internals.
Torque transfer, bearing retention, printed strength, creep, thermal response,
production fits and load capacity remain unresolved. The source's beam
calculations are payload-only sensitivities, not whole-arm FEA or a rating.
Inherited workpiece and torque commands express demand, not P6B capacity.
Full P6A optical/thermal/contact builders and electrical controller remain
unavailable; they are not inferred from images, film or replacement records.

Reuse follows the user's explicit instruction to adapt their CYBR sources.
No additional upstream project-wide license is asserted. Source attribution
and these limits remain with the example; GEO's existing license is preserved.

Focused checks:

```powershell
python -m pytest tests/test_arx_compact_geometry.py tests/test_arx_refined_geometry.py tests/test_arx_refined_body_properties.py tests/test_part_geometry.py tests/test_shape_dag.py tests/test_manufacturing_joints.py tests/test_manufacturing_gears.py
```
