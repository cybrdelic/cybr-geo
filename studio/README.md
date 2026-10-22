# CYBR GEO Studio: one engineering document

Studio edits the same canonical mechanical source used by `lab mechanism`.
`GET /api/examples` returns example documents from the shared package. The UI
loads a document, edits its components and physical assumptions, then calls the
same compiler and engineering pipeline as the native CLI. There are no separate
browser mechanism generators.

## Run

```bash
cd studio/web-auth
npm ci
npm run build
cd ../..
python studio/codex_studio.py
```

The server binds to `http://127.0.0.1:4320/`. The browser CAD kernel is actual
OpenCascade WebAssembly through Replicad. Mechanical compilation and analysis
need no AI account. They require the main project's Python CAD and MuJoCo install.
The first engineering run can take several minutes because it constructs real
solids and certifies continuous pairwise clearance.

The JSON editor is the primary mechanical input. Payload and gravity controls
update that same document. Run engineering performs construction, dynamics and
clearance analysis. Editing the source invalidates the accepted result.
Inspect-only compilation allows geometry inspection without claiming motion
acceptance. Prescribed command playback and recorded MuJoCo replay are distinct
modes. Failed simulations remain available as labeled diagnostic results.

## Shared endpoints

| Endpoint | Contract |
| --- | --- |
| `GET /api/examples` | Canonical example documents and default ID |
| `GET /api/session` | Per-process request token and login information |
| `POST /api/plan` | Compile `{ "mechanism_source": <document> }` for inspection |
| `POST /api/engineer` | Compile and run the same source through CAD, MuJoCo and continuous clearance |
| `GET /api/mechanism-schema` | Legacy bounded-alias request schema for older clients |

Same-origin POST requests require `X-Studio-Token` from `/api/session`.
`/api/engineer` returns the model, clearance, simulation, stage, errors and overall
`passed` result. A completed failed analysis returns HTTP 200 with `passed: false`;
invalid source returns 422. An imported JSON success flag never activates accepted
motion. Physical assumptions and geometry identities must match the server result.

Legacy requests such as `{ "mechanism": { "architecture": "compact_arm" } }`
are compatibility loaders for external documents. Static custom-part prompts
retain the official Codex CLI path and login gate. Unsupported mechanisms do not
fall through to an arbitrary moving-parts proposal.

## Evidence and limits

```bash
make test-mechanisms
cd studio/web-auth
npm test
npm run build
npm run probe -- ../../outputs/mechanisms/compact_arm/recipe.json ../../outputs/browser-current
```

The WASM probe constructs actual CAD, reimports STEP, checks binary STL and
compares Three.js/native poses. Simulation replay tests compare actual MuJoCo
body matrices, not nominal joint substitutions. These programmatic probes do
not assert that live browser interaction or GPU rendering was inspected.

Read [the source authoring guide](../docs/MECHANISM_SOURCE.md) and
[engineering architecture and evidence](../docs/CONSTRUCTIVE_MECHANISMS.md).
The current component catalog and collision coverage are bounded. A passing
modeled trajectory is not hardware qualification or IRIS/ARX equivalence.
