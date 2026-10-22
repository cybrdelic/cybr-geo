# Native engineering Studio

This source builds a standalone local CYBR GEO Studio. It needs Node.js 20 or
newer and the repository Python environment. It does not need CYBR LIGHT, an API
key, hosted assets, or a model call for authored engineering projects or supported component compositions.

```sh
cd studio/web-auth
npm ci
npm test
npm run build
cd ../..
python studio/codex_studio.py
```

Open the printed loopback URL. Studio selects the server's authored `arx_belt`
engineering project and starts the same native `generate_project` pipeline used
by the CLI. Its asynchronous job builds/validates native CAD and interfaces,
runs the declared electrical, control and contact model, and exposes the actual
GLB plus recorded solver body states. A first run can take several minutes. Edit
`examples/engineering/arx_belt` for project changes; the project-selection JSON
is an identifier, not a replacement design or arbitrary-geometry prompt.

The native viewer flattens body-local mesh nodes and applies each recorded
mm/Z-up matrix exactly once. The puck retains its independent simulated body.
The timeline holds the preceding recorded state without interpolation. Passing
labels require the local server's model/control/reference-bound report and mean
only that the declared task requirements passed. They do not certify continuous
clearance, manufactured hardware or unmodelled physics. Failed jobs expose their
diagnostics; imported passed flags cannot enable a passing receipt.

Native exports contain the reference GLB, viewed-pose GLB, recorded replay and
server report. Analytic STEP files remain in the native server job output. The
selected-pose GLB is standard metres/Y-up. **Capture PNG** captures the actual
interactive viewport when WebGL is available.

The dropdown also labels the older `cybrgeo-mechanism/2` component examples as
**Compatibility**. That route retains editable composition and load fields,
Replicad/OpenCascade geometry, manual joint inspection, prescribed accepted-path
motion and its separate MuJoCo playback. Its ZIP retains reference STEP/STL,
recipe, source and geometry records. Joint and drive validation precedes CAD;
geometry/source/model digests detect inconsistent edits, not trusted authorship
or physical qualification. Editing a document invalidates playback access.
Static custom part prompts retain the separately authenticated Codex planner.

The native replay contract can be checked programmatically against a fresh run:

```sh
python studio/web-auth/make-native-fixture.py \
  --trace outputs/engineering-arx-default/simulation/trace.json.gz \
  --native outputs/engineering-arx-native-final \
  --report outputs/engineering-arx-default/engineering.json \
  --out outputs/engineering-arx-browser
node studio/web-auth/probe-native-replay.mjs \
  outputs/engineering-arx-browser/native-three-fixture.json \
  outputs/engineering-arx-default/assembly.glb \
  outputs/engineering-arx-browser/three-replay-check.json
```

This uses the actual GLTFLoader, native mesh vertices and independent source SI
body-transform expectations; it is not a live browser screenshot test.

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
