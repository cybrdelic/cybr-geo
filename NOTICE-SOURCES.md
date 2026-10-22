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
| `capture_csg.py`, `build_csg_shader.py`, `csg_symmetry.py` | Shared CSG semantics; not authoritative native CAD mass |
| Spec, baseline MJCF/parameters, mass/transmission/component records | Retained configuration and explicitly cached comparison fixtures in `data/` |

Original generator SHA-256:
`ae3b03b11f533c59a3507d18d93f9b0f7df7a804a8e52ced44b1d3f577d55e37`.

Restored geometry cache SHA-256:
`838768d0473b4ba2ed25be1e1b0d5c34c3ac643dce99e140bb415d99f2103208`.

Runtime source hashes and fixture origins are retained beside adaptations and in
generated receipts. Source failures, approximate vendor envelopes and
qualification gaps remain documented.

## IRIS-09

Repository: https://github.com/cybrdelic/cybr-iris09

Pin: `6aac71594c211cc5934674b2113d9db8c5ec17c6`.

`IRIS-09.glsl` supplies housing and leaf geometric formulas/constants. The example
records the source hash, unit assumption and approximations. Prescribed motion
and pose-dependent world clipping are not a transmission or dynamics validation.
No archived artwork is used as simulation evidence.

## ARX compact

Repository: https://github.com/cybrdelic/cybr-arx6-compact

Pin: `0003382bd6a2e96f3bed065003efec35001982f7`.

`source/build_meshes.py` and `source/geometry.py` define CSG and body-local-mm to
SI-pose rendering interfaces. Its reconstructed cinematic capture and film
lineage remain separate from belt electromechanics. Example data retains source
identity; no original compact scenario-suite result is claimed.

## Dependencies and evidence

CadQuery/OpenCascade, MuJoCo, VTK and other dependencies retain upstream licenses.
New PNG/GIF evidence is rendered from generated CAD and recorded simulation
states, without image generation or claims to photograph hardware.
