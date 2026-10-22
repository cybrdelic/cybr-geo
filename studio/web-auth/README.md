# Constructive browser inspector

This source builds a standalone local CYBR GEO Studio. It needs Node.js 20 or
newer and the repository Python environment. It does not need CYBR LIGHT, an API
key, hosted assets, or a model call for the three constructed mechanisms.

```sh
cd studio/web-auth
npm ci
npm test
npm run build
cd ../..
python studio/codex_studio.py
```

Open the printed loopback URL. The first page automatically constructs the
compact arm. Choose a mechanism and scale/clearance, move the independent joint
sliders, or press Animate. The coupled followers and attached parts inherit their
transforms. Short mechanical briefs use the same Python construction compiler;
custom static briefs retain the separately authenticated Codex planner.

The browser builds real OpenCascade solids in a worker. **All joint and drive
validation happens before the first CAD operation.** A construction digest binds
the part geometry and joint definitions; editing either invalidates the attached
construction record and requires regeneration. The digest is a consistency check,
not a signature, mechanical certification, or proof of trusted authorship.

Export package saves the actual STEP assembly, per-part STEP/STL, selected-pose
GLB, construction recipe, joint graph, selected controls, geometry validation and
construction record. STEP/STL and optional analytic drawings use the reference
pose; GLB uses the selected pose in metres/Y-up. Capture PNG saves the actual
interactive CAD viewport. Nothing in these outputs asserts contact dynamics,
torque capacity, tolerances, fatigue, manufacturability or hardware qualification.

To inspect an emitted native recipe, use **Open an exported recipe**. To run the
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

For independent Python/JavaScript motion agreement, save each compiled recipe as
`compact_arm.json`, `belt_arm.json` and `inspection_head.json` in one directory,
and run the CAD probe into `browser-<architecture>` directories under one output
root. Then compare nine native trajectory samples against browser-rig matrices
and vertices from those actual exports:

```sh
python make-motion-fixture.py ../../outputs/browser-inputs ../../outputs/native-motion.json
node probe-motion.mjs ../../outputs/native-motion.json ../../outputs ../../outputs/browser-motion-parity.json
node record-acceptance.mjs ../../outputs/browser-inputs ../../outputs ../../outputs/browser-motion-parity.json evidence/acceptance.json
```

This is a programmatic Three.js rig test. It does not substitute for a live
browser UI inspection. The compact receipt records source and export hashes,
the exact construction recipes, STEP roundtrip measurements and motion errors.
It refuses stale motion or CAD artifacts from a different recipe revision.

`npm run build` writes ignored files to `studio/public/`. The third-party kernel
remains a separate replaceable `vendor/cad.js` / `vendor/cad.wasm` pair. Build
copies dependency licenses under `public/licenses/`; no third-party image or
mechanism geometry is bundled. Project source retains GPL-2.0-only.
