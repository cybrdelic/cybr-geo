# Manufacturing release contract

`lab mechanism` and `lab replay` produce diagnostic engineering artifacts. A
passing MuJoCo task or a clear CAD view does not qualify a manufactured
mechanism. `lab build --step`, `lab mechanism --step`, and native construction
export diagnostic STEP files; their coverage reports say
`manufacturing_validated=false`. Diagnostic build directories and STL folders
also carry a machine-readable `DIAGNOSTIC-NOT-FOR-MANUFACTURE.json` marker.
`lab pack` distributes software and source.

`lab release` is the only CYBR GEO path that writes a qualified product
directory. It fails before creating that directory unless all requirements
below are met. No bundled example currently qualifies: actual purchased
material lots, process capability, fabricated part inspections, load tests,
measured fits, and assembled hardware tests have not been supplied.

## Construction before CAD

Put `data/qualification-plan.json` inside an authored engineering project.
Its `cybrgeo-qualification-plan/1` document names every placed `body__part`,
every material grade and lot, each compatible process/facility/revision, each
native interface and each load case. A part declares a nominal minimum wall,
required dimensional tolerance and maximum volume. The process declares the
smallest supported feature, achievable tolerance and maximum part volume.
Part IDs use one portable `body__part` export stem: duplicate or
case-insensitive-colliding names, extra `__` separators and Windows device
names are rejected before STEP export.
Interfaces specify two exact parts, nominal clearance, two dimensional
allowances, required worst-case clearance, load demand and design capacity.
When the physical model has named MuJoCo `connect` or `weld` equalities, the
plan's `solver_connectors` object must map every such name to a declared
interface whose two native part body prefixes match the equality's explicit
`body1` and `body2` endpoints. Release rejects missing mappings or endpoints,
incomplete every-step solver
reaction records, and modeled peak anchor-force demand above the interface's
declared `demand_N`. Several connectors mapped to one interface are summed
conservatively. These are ideal solver reactions, not measured bearing forces
or validated interface capacities.
Loads specify a part, torque demand/capacity and service temperature. The
assembly declares required endurance cycles, thermal test temperature and a
maximum measured backlash. All values use the units named in the fields.

The project constructor must use `QualifiedNativeConstruction` with that plan
instead of `NativeConstruction`, and place every part directly or through
`place_assembly`. Unbound template STEP, auxiliary CAD, mass-only bodies and
mass overrides are rejected. It rejects unsupported combinations and
out-of-volume shapes *before* its `place` method emits STEP, STL, meshes or
mass records. `finish` requires exact inventory coverage and named native
interface clearances. It reimports each exact placed STEP, compares its solid
with the CAD used at placement, and measures interface gaps from those
exported parts. It writes a hashed construction receipt bound to the
plan and qualification implementation. A plan is a design contract; its
wall minimum is checked against the declared process feature limit, not
measured from the CAD. Actual walls, fits and load capacities require the
signed physical measurements below.
Release independently reimports those STEP files and compares exact part-pair
gaps, material densities, component masses, body aggregates and solver
inertials. A diagnostic constructor cannot write the reserved construction
receipt through its public API.

## External physical evidence

Supply an external trust policy (`cybrgeo-trust-policy/1`) mapping issuer IDs
to Ed25519 public keys and allowed evidence kinds. CYBR GEO has no bundled
trusted issuer. The trust policy must be provisioned outside the project,
engineering run and evidence package. Its authority is an organizational
decision, outside software's ability to prove.

The evidence package contains `evidence.json` with schema
`cybrgeo-signed-evidence-set/1` and its original report documents. Each
`cybrgeo-signed-evidence/1` record has `kind`, `subject`, `basis_sha256`,
`result="observed_pass"`, structured `measurements`, relative `document`,
`document_sha256`, `issuer`, and `signature_base64`. The Ed25519 signature
covers every field except itself as sorted-key compact UTF-8 JSON. The basis
hash covers the complete project source, plan, native build and source binding,
physical model, controller, MuJoCo simulation, integration trace and runtime
identity. Changing any of those inputs invalidates all evidence.
Release also rechecks current causal runtime and actuation source hashes plus
NumPy/MuJoCo package versions before and after staging. The generating host's
OS, machine and Python version remain signed historical run metadata, so a
different supported release host need not match that machine exactly.

An independent signed system service-envelope review must cover every
actuator torque and interface force demand, the thermal and endurance
requirements, allowable backlash and a structured physical load-path graph.
Each active actuator load starts at its declared loaded part, traverses exact
planned part/interface pairs, and ends at a distinct reaction part; all
declared interfaces must appear in the reviewed paths. This verifies coverage
and connectivity of the signed claim, not stress transfer or strength by
software alone. The signed demands cannot be weakened in the construction plan.
The plan must also cover every actuator's peak torque observed in the bound
MuJoCo run. Rotary drive requests that exceed the source belt or gear limit,
and opposed-rack requests clipped by the source output limit, block release
even if delivered peak torque is lower. Simulation alone does not establish
physical interface loads;
that mapping is an explicit responsibility of the trusted engineering review.

Every material needs a grade/lot/density/stress/temperature report; every
process a facility/revision/feature/tolerance/volume report; every placed part
a unique fabricated serial, lot and process facility/method/revision match,
measured wall, dimensional error and exact STEP hash. The assembled-unit
record must identify every inspected part serial in its signed BOM. Every
interface test must identify the same assembled unit and the exact two part
serials while reporting measured clearance and tested load capacity. Every
load test must identify that assembled unit and its inspected loaded part
serial while reporting tested torque and temperature. The assembled unit
also needs witnessed fit, motion, thermal, measured backlash and endurance
results that meet the declared and signed service requirements.
Duplicate, unsigned, stale, untrusted, incomplete or failed records are
rejected. Signed capacities must meet the plan's declared design capacities,
which themselves must exceed the signed service demands.

Run only after a current passing diagnostic engineering run and actual
independent physical evidence:

```text
lab release PROJECT --run RUN_DIRECTORY --evidence EVIDENCE_DIRECTORY \
  --trust-policy EXTERNAL_TRUST_POLICY.json --out NEW_RELEASE_DIRECTORY
```

Success atomically promotes a staged directory containing exact source-bound
placed STEP files and the original signed evidence reports, then writes
`qualified-release.json`. The result is qualified only for the service
envelope and physical serials in the externally signed records. Users must
also meet any applicable regulatory or workplace requirements; this API does
not certify those. The current ARX, IRIS and four-bar diagnostic examples lack
the required physical records and are intentionally blocked from release.
