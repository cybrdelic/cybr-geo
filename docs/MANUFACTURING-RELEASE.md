# Manufacturing release contract

`lab mechanism` and `lab replay` produce diagnostic engineering artifacts. A
passing MuJoCo task or a clear CAD view does not qualify a manufactured
mechanism. `lab build --step`, `lab mechanism --step`, and native construction
export diagnostic STEP files; their coverage reports say
`manufacturing_validated=false`. Diagnostic build directories and STL folders
also carry a machine-readable `DIAGNOSTIC-NOT-FOR-MANUFACTURE.json` marker.
`lab pack` distributes software and source.

The ARX, IRIS and four-bar diagnostic projects now carry
`data/qualification-intent.json`. The shared native builder checks its exact
placed-part inventory, material identities and any known volume ceilings before
each placement export. It records the actual placed IDs, declared part-pair
identities, load entry points and separately categorized unresolved design and
external requirements in `qualification-intent-construction.json`. These are
source-bound construction targets, not process measurements or an exact
placed-pair fit certificate. Existing nominal fixture/template checks remain
diagnostic; they cannot be relabeled as measurements of installed interfaces.
Changing the intent invalidates the native source binding. Its receipt cannot
authorize a qualified release.

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
If a project has a construction intent, the resolved plan must retain every
authored part and named interface, preserve their material and part-pair
identities, and not weaken known wall, gap, variation, demand or source drive
limits. Null intent targets are unresolved, not zero tolerances or inferred
supplier capacities. Strict construction still needs sourced materials,
compatible processes and complete numeric design limits.
Part IDs use one portable `body__part` export stem: duplicate or
case-insensitive-colliding names, extra `__` separators and Windows device
names are rejected before STEP export.
Each interface specifies two exact parts, dimensional allowances, load demand
and design capacity. A clearance interface declares a nominal gap and required
worst-case service gap. An intentional retained contact instead uses
`role: tested_retention_contact`, a maximum nominal closest-surface distance
and maximum service play. Its closest distance plus both participating part
variations must fit the play limit; zero nominal distance is never treated as
a positive-clearance fit. This role requires a serial-bound physical test of
retention under load and measured assembled play. CAD proximity alone proves
neither retention nor contact pressure; interference/press fits need a
separate supported design and physical evidence path.
When the physical model has named MuJoCo `connect` or `weld` equalities, the
plan's `solver_connectors` object must map every such name to a declared
interface whose two native part body prefixes match the equality's `body1` and
`body2` endpoints, or the source XML body owners of `site1` and `site2`.
Release rejects missing or ambiguous mappings or endpoints,
incomplete every-step solver
reaction records, and modeled peak anchor-force demand above the interface's
declared `demand_N`. Several connectors mapped to one interface are summed
conservatively. These are ideal solver reactions, not measured bearing forces
or validated interface capacities.
For each controlled tension spring, `spring_load_paths` maps its indexed source
spring and both ends of every physical spring member to distinct placed
pin/clevis interfaces with `role: tested_retention_contact`; a clearance-only
interface cannot establish capture of a loaded spring eye. Its `joint` must
match the physical ranged hinge;
the control's joint-frame anchor vectors and axial `member_offsets_m` must
resolve to the placed pins under that hinge's child-body and joint transforms.
Unsupported frames and extra child degrees of freedom fail closed. Release
reopens the placed STEP and checks the pin dimensions and nominal bore gap
against that source-bound construction receipt.
`maximum_service_scale` bounds accepted runtime spring scale and
`uncertainty_factor` is an authored factor of at least one. The release path
computes ideal force over the full declared joint range, multiplies it by
both factors, and adds it to any modeled connector demand assigned to the
same interface. The planned `demand_N`, independent signed service demand,
and serial-bound tested capacity must cover that combined force in order.
The factors are design assumptions, not measured spring uncertainty or a
supplier rating; the independent service review must justify the actual
service envelope. Missing spring pin/clevis construction or physical capacity
evidence blocks release.
Loads specify a part, torque demand/capacity and service temperature. The
assembly declares required endurance cycles, thermal test temperature and a
maximum measured backlash. All values use the units named in the fields.

The project constructor must use `QualifiedNativeConstruction` with that plan
instead of `NativeConstruction`, and place every part directly or through
`place_assembly`. Unbound template STEP, auxiliary CAD, mass-only bodies and
mass overrides are rejected. It rejects unsupported combinations and
out-of-volume shapes *before* its `place` method emits STEP, STL, meshes or
mass records. The qualified constructor also requires a complete source-derived
initial rigid pose for every placed body. `finish` requires exact inventory
coverage and reimports each exact placed STEP. It compares each solid with
the CAD used at placement, then measures cross-body interface gaps after
placing copies of those solids in the common initial assembly frame. The
exported STEP and native mass properties remain body-local. The hashed
construction receipt includes the body poses and their digest. It is bound to the
plan and qualification implementation. A plan is a design contract; its
wall minimum is checked against the declared process feature limit, not
measured from the CAD. Actual walls, fits and load capacities require the
signed physical measurements below.
Release independently reimports those STEP files and compares exact part-pair
gaps, material densities, component masses, body aggregates and solver
inertials. It also derives the initial poses from the bound MuJoCo source and
startup keyframe, rejecting a construction pose that does not match the
simulated assembly. Moving CAD bodies require matching solver mass/inertia;
fixed hardware retains its verified native mass without a fictitious dynamic
inertia, and its mapped physical body must have no joint in its world ancestry.
A diagnostic constructor cannot write the reserved construction
receipt through its public API.

## External physical evidence

Supply an external trust policy (`cybrgeo-trust-policy/1`) mapping issuer IDs
to Ed25519 public keys and allowed evidence kinds. CYBR GEO has no bundled
trusted issuer. The trust policy must be provisioned outside the project,
engineering run and evidence package. Choosing its issuers is an accountable
project owner's decision, whether that owner is an individual maker or an
organization. A valid signature establishes origin under that policy; it does
not prove that a physical test occurred or that a review was independent.
Independent review requires a reviewer distinct from the design author and
inspection of the underlying reports, which software cannot establish from a
signature alone.

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
serials while reporting tested load capacity. Clearance interfaces require a
measured minimum gap; retained contacts require an observed under-load
retention pass and measured maximum play within the declared limit. Every
load test must identify that assembled unit and its inspected loaded part
serial while reporting tested torque and temperature. The assembled unit
also needs witnessed fit, motion, thermal, measured backlash and endurance
results that meet the declared and signed service requirements.
Duplicate, unsigned, stale, untrusted, incomplete or failed records are
rejected. Signed capacities must meet the plan's declared design capacities,
which themselves must cover the signed service demands. Any required strength
reserve must be declared in the planned capacity and supported by the test.

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
