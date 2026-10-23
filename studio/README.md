# CYBR GEO Studio: native engineering projects

Studio defaults to the authored `arx_belt` project through the same native
`generate_project` pipeline used by the CLI. The shared runtime constructs and
verifies native parts and interfaces, builds the declared physical/control
model, integrates the contact task, and returns geometry plus recorded body
states. The example supplies its design and task; the browser does not introduce
a second geometry generator or solver.

## Run

Use the repository's Python environment with native CAD and MuJoCo installed
as described in [Getting Started](../docs/GETTING_STARTED.md), plus Node.js 20 or newer.

```sh
cd studio/web-auth
npm ci
npm test
npm run build
cd ../..
python studio/codex_studio.py
```

Open the printed loopback URL, normally `http://127.0.0.1:4320/`.
`CYBR_CODEX_PORT` can select another local port. The authored project needs no
AI account, model call or API key. A first run can take several minutes because
it constructs actual native CAD and executes the declared dynamics task.

The server supplies the default project and compatibility examples. The primary
action starts an asynchronous native engineering job and shows its progress.
The project-selection document identifies a registered authored project; edit
the example's source files for design changes. It is not a free-form mechanical
design or an editable replacement for those files.

On completion, Studio loads the actual native GLB and applies the recorded
mm/Z-up body matrices directly. The puck remains an independent contact body.
Replay holds each recorded state until the next sample, with no prescribed joint
animation or interpolation substituted. Task measurements and model assumptions
remain visible. A passing nominal task does not establish continuous clearance,
unmodelled safety behavior, manufacturing suitability or hardware qualification.

The native ZIP contains reference GLB, viewed-pose GLB, replay and server report.
Analytic STEP exports remain in the local native job directory under
`studio/output/codex-jobs/<job-id>/engineering/`. The GLB uses standard metres/Y-up;
the replay contract uses millimetres/Z-up. Capture PNG saves the actual viewport
when WebGL is available.

## Shared endpoints

| Endpoint | Contract |
| --- | --- |
| `GET /api/examples` | Registered projects, default project selection, and explicit compatibility documents |
| `GET /api/session` | Per-process request token and optional static-planner login information |
| `POST /api/project-jobs` | Start `{ "project": "arx_belt" }`; HTTP 202 returns job ID and status URL |
| `GET /api/project-jobs/<id>` | `queued`, `running`, `succeeded` or `failed`, plus completed report and available assets |
| `GET /api/project-jobs/<id>/assets/<name>` | Allowlisted native GLB or recorded replay for that job |
| `POST /api/plan` | A default mechanical brief selects the native project; explicit component source compiles through the compatibility route |
| `POST /api/engineer` | A default mechanical brief starts the native job; explicit component source runs the compatibility engineering pipeline |
| `GET /api/mechanism-schema` | Bounded component-alias request schema for compatibility clients |

Same-origin POST requests require `X-Studio-Token` from `/api/session`. Only
registered local projects are accepted; a request cannot choose an arbitrary
Python path. The native result is `cybrgeo-engineering-result/2`, and recorded
playback is `cybrgeo-native-replay/1`. Model, control and reference identities
must match the trusted server result. Imported success flags cannot enable a
passing browser receipt. Failed jobs show their diagnostics rather than a
successful motion claim.

## Component and static-part compatibility

The dropdown explicitly labels older component documents as **Compatibility**.
That editor retains `cybrgeo-mechanism/2` composition, load and gravity fields,
Replicad/OpenCascade WebAssembly construction, manual joint inspection and its
separate engineering path. Joint validation occurs before browser CAD. Geometry,
source and model identities reject inconsistent edits. These are consistency
checks, not physical qualification or authentication of arbitrary imported data.

Compatibility **Inspect geometry** constructs without a physics claim. Accepted
prescribed motion and recorded MuJoCo replay remain separate controls; their
clearance/authorization gates apply only to that bound component model and path.
Compatibility ZIPs retain reference STEP/STL, recipe and source exports. The
component documentation is [Mechanism Source](../docs/MECHANISM_SOURCE.md).

Static custom-part prompts retain the official Codex CLI login gate. Unsupported
mechanical requests do not fall through to arbitrary moving-part generation.

## Evidence and limits

[Fresh native evidence](../docs/pr-assets/arx-engineering/README.md) includes
actual integrated-pose PNG/GIF, all 385 analytic CAD parts, native STEP readback,
nominal and friction-loss results, Three.js/native vertex parity, and the local
HTTP contract check. The native probe rejects stale/forged receipts, and the
free puck's transforms are checked independently of the arm.

[The browser source README](web-auth/README.md) documents reproducible native
replay probes and the retained compatibility kernel/motion tests. Programmatic
GLTFLoader checks do not assert live browser interaction. Live UI inspection was
unavailable in this execution environment because the supported browser
inventory was empty. The separate examples' geometry, dynamics and qualification
claims remain distinct.
