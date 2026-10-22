# Constructive browser inspector

This source builds a standalone local CYBR GEO Studio. It needs Node.js 20 or
newer and the repository Python environment. It does not need CYBR LIGHT, an API
key, hosted assets, or a model call for supported component compositions.

```sh
cd studio/web-auth
npm ci
npm test
npm run build
cd ../..
python studio/codex_studio.py
```

Open the printed loopback URL. Studio loads example documents from the local
server, selects its default example and runs the engineering pipeline. The main
editor contains the canonical `cybrgeo-mechanism/2` source: component composition,
interfaces, materials, tolerances, actuator assumptions, loads and contact policy.
Examples have no separate compiler or browser-side catalog. Edit that document,
or open your own JSON source, then choose **Run engineering**. The load controls
edit payload mass and gravity in the same document. The Python environment needs
the repository's native CAD and MuJoCo dependencies for full engineering runs.

**Inspect geometry** compiles without making a physics claim. Manual sliders
remain available when later engineering stages fail. The legacy static-prompt
workflow retains the separately authenticated Codex planner.

Accepted motion evaluates the canonical source's period and cosine waypoint
trajectory, restricted to the fresh server-returned certified command path.
Its gate requires the complete engineering pipeline's motion authorization.
MuJoCo playback instead uses recorded body matrices in a flat Three.js
group, so the ideal joint hierarchy cannot hide tracking or coupling errors.
Failed runs can be replayed as diagnostics; tracking, effort saturation and stage
errors remain visible. A sampled dynamics trace does not establish continuous
clearance. Editing the document invalidates playback access, and imported passed
flags never enable it: the local server must recompute the records.

The browser builds real OpenCascade solids in a worker. **All joint and drive
validation happens before the first CAD operation.** A construction digest binds
the part geometry and joint definitions. Mechanism and model digests additionally
bind the complete source and derived physical assumptions. Editing any bound
field invalidates its record and requires recompilation. Digests are consistency
checks, not signatures, mechanical certification, or proof of trusted authorship.

Export package saves the actual STEP assembly, per-part STEP/STL, selected-pose
GLB, source document, construction recipe, joint graph, viewed controls/body state,
geometry validation and construction record. Fresh server engineering results
are included when present. STEP/STL and optional analytic drawings use the
reference pose; GLB uses the viewed pose in metres/Y-up. Capture PNG saves the
actual interactive CAD viewport. Physics results apply only to the explicit
model, contact and controller assumptions; they do not establish hardware,
fatigue, thermal or manufacturing qualification.

To inspect an emitted native recipe, use **Open a mechanism or CAD recipe**. To run the
same kernel headlessly against a generated artifact:

```sh
node probe-cad.mjs ../../outputs/mechanisms/compact_arm/recipe.json ../../outputs/browser-compact
# Add --drawings for analytic four-view hidden-line drawings.
```

The probe writes `browser-acceptance.json` and actual CAD exports. It reopens the
exported STEP bytes to compare solid count and volume (10 ppm relative numerical
volume tolerance, with the actual difference recorded), and checks every binary
STL's declared record count and finite coordinates. It fails on
invalid B-reps, disconnected/zero-volume solids, invalid or non-closed meshes,
stale construction data, or inconsistent joint relationships. Its geometry
checks are separate from contact/interference sweeps and dynamics.

For independent Python/JavaScript motion agreement, save compiled recipes as
`<example-id>.json` in one directory, including your edited compositions,
and run the CAD probe into `browser-<example-id>` directories under one output
root. Then compare nine samples of the source's motion task against native
positions, velocities, accelerations, peak rates, browser-rig matrices and
vertices from those actual exports:

```sh
python make-motion-fixture.py ../../outputs/browser-inputs ../../outputs/native-motion.json
node probe-motion.mjs ../../outputs/native-motion.json ../../outputs ../../outputs/browser-motion-parity.json
node record-acceptance.mjs ../../outputs/browser-inputs ../../outputs ../../outputs/browser-motion-parity.json evidence/acceptance.json
```

This is a programmatic Three.js rig test. It does not substitute for a live
browser UI inspection. The compact receipt records source and export hashes,
the exact construction recipes, STEP roundtrip measurements and motion errors.
It refuses stale motion or CAD artifacts from a different recipe revision.

To independently compare actual integrated MuJoCo playback with the native
`simulated_assembly` adapter, first run the native dynamics pipeline and the WASM
CAD probe on the same recipe. Then, in the repository's CAD Python environment:

```sh
python make-simulation-fixture.py ../../outputs/browser-ir/compact_arm.json ../../outputs/dynamics/compact_arm/simulation.json ../../outputs/browser-compact_arm/assembly.json ../../outputs/native-simulation.json
node probe-simulation.mjs ../../outputs/native-simulation.json ../../outputs/dynamics/compact_arm/simulation.json ../../outputs/browser-compact_arm/assembly.json ../../outputs/browser-simulation-parity.json
```

The optional `--cache <native-cache-directory>` avoids a native CAD rebuild.
Source/model/geometry identities must still match. The probe compares five
recorded times, every part's world matrix and selected actual mesh vertices in
millimetres; it does not replace live browser inspection or validate the physical
accuracy of the simulation.

`evidence/four-axis-source.json` is an edited composition of the same components,
with four rotary axes, two jaw controls and a twelve-second motion period. It
can be opened in the source editor; it adds no architecture-specific compiler.
The recorded CAD/motion receipt includes this composition. The separate MuJoCo
replay receipt covers the compact-arm example only.

With Studio running, verify its real HTTP source route, built worker/kernel
assets and CSRF guard without starting another physics run:

```sh
node probe-http.mjs http://127.0.0.1:4328 ../../outputs/browser-ir evidence/http-contract.json
```

Use the port printed by your server. The recipe directory must contain the
compiled server examples and `four_axis.json` from the evidence source above.
This protocol check does not execute or visually inspect a browser.

To check the browser's acceptance gate against completed native records without
repeating a solve, use the saved engineering report, simulation and compiled
recipe. The optional final directory tests a belt simulation as a diagnostic
without a complete clearance receipt:

```sh
node probe-engineering.mjs ../../outputs/engineering-current/compact_arm/engineering.json ../../outputs/engineering-current/compact_arm/simulation/simulation.json ../../outputs/browser-ir/compact_arm.json evidence/engineering-gate.json ../../outputs/dynamics/belt_arm-canonical
```

`npm run build` writes ignored files to `studio/public/`. The third-party kernel
remains a separate replaceable `vendor/cad.js` / `vendor/cad.wasm` pair. Build
copies dependency licenses under `public/licenses/`; no third-party image or
mechanism geometry is bundled. Project source retains GPL-2.0-only.
