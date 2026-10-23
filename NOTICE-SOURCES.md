# Source notices

CYBR GEO retains its existing GPL-2.0-only license. Alejandro Figueroa / cybrdelic
explicitly requested reuse of his published IRIS and ARX designs here. These
pins identify actual inspected sources and their separate provenance. No extra
project-wide license grant is inferred from benchmark repositories; their
original notices and asset rights remain applicable. New and adapted CYBR GEO
code follows this repository's license.

## ARX belt

Repository: https://github.com/cybrdelic/cybr-arx6-belt

Pin: `aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2`.

Source: `ARX-6-P5-Engineering/integration/`.

| Original source | Reuse |
| --- | --- |
| `generate_cad_p2.py` (current P5) | Example `design.py`; shared native placement, CAD mass and full inertia |
| `build_p5_collision_proxies.py` | Example contact layout and shared proxy/tabletop construction |
| `build_p5_model.py` | Example physical configuration and shared inertia/proxy lowering |
| `controller.mjs`, `safety.mjs` | Generalized actuation/runtime, numerical parity fixtures and explicit retained assumptions |
| Task reference in `controller.mjs` | Example task and numerical source comparison |
| `validate_p5.mjs` | Pinned scenario definitions and unchanged numeric acceptance thresholds in `data/source-scenarios.json`; shared-runtime runner in `faults.py` and `tools/check_arx_faults.py` |
| `capture_csg.py`, `build_csg_shader.py`, `csg_symmetry.py` | Shared CSG semantics; not authoritative native CAD mass |
| Spec, baseline MJCF/parameters, mass/transmission/component records | Retained configuration and explicitly cached comparison fixtures in `data/` |

Original generator SHA-256:
`ae3b03b11f533c59a3507d18d93f9b0f7df7a804a8e52ced44b1d3f577d55e37`.

Restored geometry cache SHA-256:
`838768d0473b4ba2ed25be1e1b0d5c34c3ac643dce99e140bb415d99f2103208`.

Runtime source hashes and fixture origins are retained beside adaptations and in
generated receipts. Source failures, approximate vendor envelopes and
qualification gaps remain documented.

The default operating policy is a new GEO adaptation: it removes lateral
transfer tilt while retaining the TCP path, forward tilt, roll formula and
endpoints, and reduces transfer and loaded-descent rates. Drive, brake, spring
and belt ratings remain unchanged. The source-default task oracle remains
available for numerical comparison. Original-clock and phase-matched fault executions
retain separate receipts. Added contact-placement outcomes are reported
separately from the source validator's numeric checks; they do not rewrite the
source's historical results.

## IRIS-09

Repository: https://github.com/cybrdelic/cybr-iris09

Pin: `6aac71594c211cc5934674b2113d9db8c5ec17c6`.

`IRIS-09.glsl` supplies the housing, optical volumes, two-circle leaf profile and
canted slab formulas/constants. The original `design.py` geometry study retains
the prescribed reference display and distinguishes its pose-dependent world
clipping from the invariant source profile. Its source hashes, scene-unit
assumption and approximations remain recorded. No original transmission,
actuator model or dynamics validation is inferred from that optical source.

The new `iris09` engineering project adds an explicitly derived mechanism:
one annular motor/rotor, nine pinned rods, passive leaf shafts, bearings and
retaining features. A fixed source-derived leaf crop and neighboring-shaft
keepouts replace per-pose material removal. Shared GEO construction, linkage,
mass/inertia, causal actuation, MuJoCo and native replay services execute this
adaptation. New motor values, materials and the 50 mm/scene-unit conversion are
assumptions. The retained 0.0325 mm foil, attachment/retention strength, bearing
contact, tolerances, fatigue and hardware suitability remain unqualified.
Original geometry-study evidence and new integrated-mechanism evidence retain
separate identities and scopes. No archived artwork or source film is used as
simulation evidence, and no upstream license grant is invented.

## ARX compact

Repository: https://github.com/cybrdelic/cybr-arx6-compact

Pin: `0003382bd6a2e96f3bed065003efec35001982f7`.

`source/build_meshes.py` and `source/geometry.py` define CSG and body-local-mm to
SI-pose rendering interfaces. Its reconstructed cinematic capture and film
lineage remain separate from belt electromechanics. Example data retains source
identity; no original compact scenario-suite result is claimed.

### Refined and P6B print-first source

Additional pin: `85bd6d2ce5b7c567d08b3ffccf2a9be46005aabf`, published in
[compact source PR2](https://github.com/cybrdelic/cybr-arx6-compact/pull/2).
The user explicitly authorized adapting this reviewed source on 2026-10-22.

| Original source | Reuse |
| --- | --- |
| `revisions/p5-refined/source/cad_core.py`, `refine_cad.py` | Inspected CSG/body-local replacement semantics, explicit material densities, native property reconciliation |
| `revisions/p5-refined/source/verify_geometry.py` | Shared named-component spatial inertia replacement and reconciliation with the actual corrected Refined body-property records; source fixture SHA256 `f0dbdd6cbb2d69b5a114f65e88ecd65a89d207ac46a6532c4c0e294f7f253ae1` |
| Refined design, selected BREP files, tessellation/refinement and capture records | Hash-bound data consumed by the shared `PartGeometryResolver`; the static client applies each complete local replacement exactly once |
| `revisions/p6b-print-first/source/build_structural_prototypes.py` | Shared rounded box-tube and lap-splice construction; actual Refined joint-end inputs retained by the client |
| `revisions/p6b-print-first/source/build_printed_gear_trials.py` | Shared sampled involute pinion/rack builder, preserved source dimensions and declared root limitations |
| `audit_print_first.py`, `screen_print_first.py` | Inspected manufacturing decisions and qualification boundaries; payload-only beam screening is not promoted to whole-arm FEA |

Original builder fixtures and their hashes accompany parity tests. Packaged
source manifests retain upstream file identities. Physical materials remain
separate from render finishes. Current selected native properties and the
source-derived corrected body records are computed and compared explicitly;
the inherited captured motion is not relabeled as a dynamics rerun for them.

The six print-first articles are structural and coarse gear/rack prototypes,
not a completed printed arm. Reducer envelopes do not supply printed internals.
Bolts, sleeves, torque transfer, bearing retention, creep, thermal behavior and
transmission strength remain unresolved or unqualified. The P6A workpiece mass
and torque commands describe inherited demand, not P6B capacity. No missing
P6A full controller/thermal source or hardware test is invented.

## Dependencies and evidence

CadQuery/OpenCascade, MuJoCo, VTK and other dependencies retain upstream licenses.
New PNG/GIF evidence is rendered from generated CAD and recorded simulation
states, without image generation or claims to photograph hardware.
