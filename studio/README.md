# CYBR GEO Studio: construct the mechanism first

The default mechanical path now compiles a bounded assembly specification before
creating any part geometry. The same compiler supplies the Python CLI and this
Studio endpoint. It owns mating dimensions, support clearances, joints, actuation
relationships and construction metadata together.

Three **original nominal studies** are available without an AI account or a model
request: `compact_arm`, `belt_arm` and `inspection_head`. Short requests such as
“build a robot arm”, “belt driven arm”, “inspection head” or “pan-tilt gimbal” select
these architectures. An empty/default request selects `compact_arm`.

They are not reproductions of IRIS-09 or either ARX-6 package. Those source packages
could not be materialized into this executor. The compact and belt studies retain
separate construction records; no validation claim is transferred between designs.

## Run

Install CYBR GEO from the repository's main README, then run:

```bash
cd studio/web-auth
npm ci
npm run build
cd ../..
python studio/codex_studio.py
```

The server binds only to `http://127.0.0.1:4320/`. On Windows,
`studio/start-codex.ps1` uses the repository virtual environment when available.
The build emits ignored `studio/public/` from tracked browser source and npm
dependencies. It does not require CYBR LIGHT or a pre-existing example archive.
The inspector opens a freshly compiled compact arm by default; select a family,
change parameters, move joints, animate, or export actual CAD and viewport images.
No existing live Studio directory is modified by this checkout.

## Structured construction requests

Python callers can use the same planning entry point without a browser:

```python
import sys
sys.path.insert(0, "studio")
from codex_studio import plan_design

plan = plan_design("belt driven arm")
adjusted = plan_design(mechanism={
    "architecture": "compact_arm",
    "parameters": {"scale": 0.8, "joint_limit_deg": 10},
})
```

The returned `parts`, `joints` and `construction` records are generated together.
`generation.inference_used` is `false`. Requests and compiled recipes are saved
under ignored `studio/output/codex-jobs/` for local recovery.

| Parameter | Default | Allowed range |
| --- | ---: | ---: |
| `scale` | 1 | 0.5–2 |
| `radial_clearance_mm` | 0.2 | 0.12–0.4 mm |
| `joint_limit_deg` | 12 | 1–18° |
| `jaw_travel_mm` | 10 | 2–14 mm |

The jaw stroke also depends on scale: an omitted value becomes
`min(14, 10 * scale)` mm, and an explicit value must be at most `14 * scale` mm.

Use structured parameters to change dimensions. Prose requirements such as
“500 mm arm” are rejected instead of being silently discarded. Unknown parameter
names, nonfinite values, booleans and out-of-range values are rejected. Compiler
domain checks remain authoritative. These ranges bound the study; they are not
manufacturing tolerances or qualified operating ratings.

`GET /api/session` returns the per-process request token, available architectures
and `mechanismSchema`. `GET /api/mechanism-schema` returns the same bounded contract.
`POST /api/plan` accepts `{ "prompt": "robot arm" }` or
`{ "mechanism": { "architecture": "belt_arm", "parameters": {} } }`.
Same-origin requests must include `X-Studio-Token` from the session response.
Supported mechanical requests need no Codex login. Their response provider is
`constructive`.

## Static custom parts and existing Studio

Static prompts such as “mounting flange”, “sensor bracket” and “instrument
enclosure” retain the original bounded primitive planner and official Codex CLI
login. That path cannot return moving joints. Unsupported mechanisms fail with a
clear message; they do not fall through to arbitrary geometric proposals.

The copied local server, codec and primitive schema are a source snapshot of the
existing Studio planning boundary. Its legacy browser application is not imported
with private jobs, authentication data, generated examples or renderer binaries.
The new inspector is a separate reproducible consumer of the shared compiler.
When integrating this endpoint with a legacy UI, use `constructiveAvailable` to
enable mechanical generation independently of the `authenticated` field. Do not
apply the old 32/64-part planner budget to compiler-expanded assembly parts.

An exported construction record describes nominal kinematic relationships. It
does not establish contact dynamics, torque capacity, stress, durability,
electrical control, tolerance accumulation or physical qualification. Refer to
the generated evidence for exactly which artifact and motion checks executed.

## Test the actual routing

```bash
python -m unittest discover -s studio/tests -v
```

The tests call the real compiler through `plan_design`, block subprocess execution
for mechanical prompts, exercise HTTP CSRF and login boundaries, check explicit
parameters and unsupported-mechanism rejection, and preserve the static codec/CLI
path using a local inference stub. They do not make paid model calls or claim
mechanical qualification.
