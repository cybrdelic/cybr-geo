# Authoring a mechanical source

CYBR GEO lowers one `cybrgeo-mechanism/2` JSON document through a shared component
registry. That source supplies geometry, connected interfaces, joints, drives,
physical assumptions, and the simulation scene. `compact_arm`, `belt_arm`, and
`inspection_head` are historical original studies stored as external documents;
they are not separate generation implementations or reproductions of IRIS or
ARX source material.

```python
from mechanism_lab.construction import compile_mechanism
from mechanism_lab.mechanism import load_example

source = load_example("compact_arm")
source["name"] = "Four-axis inspection fixture"
source["components"] = source["components"][:4] + source["components"][-1:]
source["components"][-1]["parent"] = "J4"  # Same retained output interface.
for unused in ("J5_motor", "J6_motor"):
    del source["actuators"][unused]
plan = compile_mechanism(source)
```

This changes the actual component graph. It does not select another architecture
inside the compiler. The resulting 75-part composition has also been built in
the browser CAD kernel with valid single solids, closed meshes, and STEP/STL
export checks. Those checks concern that geometry; they do not qualify its
hardware or its entire motion range.

Save `source` as JSON for the CLI or Studio mechanism editor. Installed study
documents live in `src/mechanism_lab/mechanisms/`; editable copies are in
`examples/constructed/`. `compile_plan(alias, parameters)` remains a compatibility
adapter that loads a document and applies its old bounded overrides before
calling `compile_mechanism`.

## Source contract

Unknown fields, invalid references, and nonfinite values are rejected. Source
coordinates are reference-pose world coordinates in millimetres, with Z up.
Rigid-body component solids remain invariant during motion.

| Field | Meaning |
| --- | --- |
| `schema`, `name`, `units`, `provenance` | `cybrgeo-mechanism/2`, a design name, `mm`, and an explicit origin statement. |
| `dimensions.hardware_scale` | Scale of reusable hardware dimensions, bounded to 0.5–2. Component `origin_mm` positions remain explicit millimetres. The legacy adapter scales positions as well. |
| `materials` | Named physical assumptions with `density_kg_m3` and provenance. The shaft material also declares `allowable_shear_MPa` for the nominal torsion rejection screen. These are not vendor certificates. |
| `material_roles` | Explicit assignments for frame, shaft, bushing, motor, belt, and pad. Display color never supplies physical material properties. |
| `tolerances` | Named `journal` and `guide` fits. Each has nominal radial clearance, shaft and bore radius tolerances, and required minimum clearance, all in millimetres. |
| `components` | Named instances with a registered `type`, `parent` component ID or null, typed `parameters`, and a `drive` declaration. Input order is normalized by parent dependencies. |
| `actuators` | Independent actuator assumptions. Each is referenced exactly once by a component drive. Revolute effort is `effort_limit_Nmm`; linear effort is `effort_limit_N`. Speed and acceleration use explicit degree/second or millimetre/second fields. |
| `loads` | `gravity_mm_s2`, a payload with component attachment, mass in kilograms and `com_offset_mm` from its output datum, and a duty-cycle assumption. |
| `contacts` | Component proxy policy, friction, ground, and free environment bodies. These settings are part of the same physical source. |
| `motion` | Optional requested motion task: period in seconds and either the bounded-excursion policy or two waypoints keyed by source actuator IDs. Omission normalizes to the same eight-second default. |
| `manufacturing` | Explicit process, simplified-thread, and qualification statements. |

The nominal running fit is constructed from one shaft radius and one clearance.
Its worst-case residual is checked before CAD:

```text
minimum radial clearance = nominal radial clearance
                         - shaft radius tolerance
                         - bore radius tolerance
```

The current component library constructs shared mating bores, bushings, keyed
flanges, retainers, guide rods, and their corresponding parent connections.
Those interface dimensions are derived together; they are not separately
guessed solids that are later assigned a joint. The expanded plan records paired
part IDs and dimensions in `construction.interfaces`, supported paths in
`construction.load_paths`, and geometric axial capture probes.

## Current component domain

The built-in `rotary_cartridge` component creates a supported shaft and housing,
a retained keyed output, and an integral structural connection to its parent.
It accepts `origin_mm`, a positive cardinal `axis`, and a symmetric `limit_deg`
between 1 and 18. Its grounded stand uses a +Z axis at x=y=0, with a height
between 65 and 140 times hardware scale. Adjacent cartridge datums must be at
least 70 times hardware scale apart. Each cartridge currently accepts at most
one output child; a branch needs a component with an actual branch interface.

Its drive is either `direct` or `equal_pulley_belt`, with one named actuator.
The belt component constructs equal pulleys, a continuous loop, removable
torque keys, and axial retention. Its 1:1 follower relation is an ideal no-slip
assumption. Belt tension, traction, elastic stretch, and belt material advection
are not solved by that relation.

The built-in `parallel_gripper` terminates a +X cartridge output. It accepts
`travel_mm` from 2 to the smaller of 14 mm and 14 times hardware scale. Two
independent linear actuators drive guided jaws. There is no fictitious screw
coupling or mechanically enforced jaw synchronization.

The shared source currently permits 1–9 component instances and the expansion
permits at most 256 unique parts, with exactly one parent joint per part. New
registered components may support other frames and branching contracts; that
does not make the existing cartridges support them. Closed-loop linkages,
nonlinear pin-slot closure, arbitrary-axis cartridges, and an iris diaphragm
component are not implemented in this catalog.

This source schema is still bounded by the current construction catalog. Its
six material roles, journal/guide fit records, 0.5–2 hardware scale, and 1–9
instance limit are global requirements, including for newly registered types.
Registration alone does not turn it into a general mechanical IR. Imported
mechanisms need an explicit, inspected adapter to the common lowered physical
model; they must not be relabeled cartridge reconstructions.

## Registering a reusable component

`mechanism_lab.component_catalog.register_component` is an explicit extension
boundary for trusted installed Python code. JSON documents cannot import code
or execute expressions. A component registration supplies:

- A unique type name, revision, provenance, and whether it can be grounded.
- `validator(component, context)`, which enforces its interface and parameter
  domain and returns `(actuator_id, joint_kind)` pairs for independent controls.
- `expander(builder, component, previous_output)`, which constructs actual
  primitives, interfaces, and joints and returns `input_part`, `output_part`,
  `output_datum`, actuator bindings, and optional capture probes.

`context` includes the full source, component map, children, and hardware scale.
The builder's geometric helpers use hardware coordinates and apply its scale;
an expander converting explicit source millimetres divides by that scale first.
`output_datum` is `(origin_in_hardware_coordinates, unit_axis, output_part_id)`.
An extension may supply nominal shaft sections for the torsion screen; other
component structural limits need their own explicit domain checks. Do not infer
shaft strength from a decorative body's bounding box.

The compiler checks expanded part/joint uniqueness, parent connectivity, and
independent actuator coverage. Component revisions and provenance are included
in the full model identity. Duplicate registrations are rejected. A regression
test registers and compiles a new static fixture type without changing central
validation or lowering dispatch.

## Contacts and environment bodies

The source owns this scene contract:

```json
{
  "model": "component_safe_primitives",
  "friction": [0.8, 0.01, 0.001],
  "enable_component_proxies": false,
  "ground": true,
  "ground_position_mm": [0, 0, 0],
  "geoms": [],
  "free_bodies": [{
    "name": "contact_probe",
    "position_mm": [0, 200, 200],
    "velocity_mm_s": [0, 0, 0],
    "angular_velocity_rad_s": [0, 0, 0],
    "mass_kg": 0.1,
    "shape": {
      "kind": "sphere", "size": [20, 20, 20],
      "position": [0, 0, 0], "rotation": [0, 0, 0]
    }
  }]
}
```

Free bodies support box, sphere, and cylinder collision shapes with full-size
dimensions in millimetres. Their dynamics and ground contact are separate from
assembly contact completeness. The compiler currently supplies material-contained
proxies only for cut-free parts. Bored and otherwise cut parts are listed as
omitted; it never fills a bore with a solid collision hull. Component proxies
default to disabled. Explicitly enabling them gives partial contact coverage,
not a complete assembly collision model. Arbitrary authored `geoms` remain
rejected until their material containment can be checked.

## One requested motion task

The default source declares `"motion": {"period_s": 8, "policy":
"bounded_excursion"}`. That policy derives two bounded waypoints from the
constructed control ranges. It starts at each reference value, travels 70% of
the longer available side (alternating directions for symmetric ranges), caps
revolute excursion at 22 degrees, and returns to the reference once per period.

For an explicit task, supply exactly two complete waypoint maps using source
actuator IDs, in their native degrees or millimetres:

```python
points = [{actuator: 0.0 for actuator in source["actuators"]} for _ in range(2)]
points[0]["J2_motor"] = -2.0
points[1]["J2_motor"] = 6.0
source["motion"] = {"period_s": 12, "waypoints": points}
```

The compiler resolves actuator IDs into generated independent joints and saves
`construction.motion`. Its single cosine interpolation travels between the two
waypoints during the first half-period and returns during the second half.
`motion.MotionTask` provides the position and analytic velocity/acceleration
used by prescribed native playback and the simulation controller's request.
Studio evaluates the same source-bound task. Actual simulated playback uses the
recorded integrated states, which may differ from that request.

All waypoints must remain within constructed joint limits. The task's peak
speed and acceleration must remain within the declared actuator assumptions;
otherwise construction rejects it before CAD. A simulation's `--seconds`
controls run duration only: a four-second run of an eight-second task stops at
the second waypoint; it does not compress the full cycle into four seconds.
Changing period or waypoints changes the mechanism/model identity and requires
new motion evidence, while unchanged geometry retains its recipe identity.

## What is checked, and what remains an assumption

Construction checks happen before CAD: supported interfaces and topology,
shared fit residuals, guide stroke, unique parent relationships, actuator
coverage, and finite physical input domains. A load screen uses upper bounds
from positive CSG volumes, the reference layout, gravity, and each actuator's
own-axis acceleration/centripetal envelope. Insufficient declared effort or an
actuator limit above the nominal circular-shaft torsion domain rejects the
source. This screen does not solve coupled dynamics. It does not qualify local
key/thread stress, bending, deflection, bearing pressure, fatigue, thermal duty,
preload, friction capacity, or manufacturing access.

Actual CAD validity, interference checks over a declared motion domain,
MuJoCo simulation, and hardware qualification are separate evidence levels.
Simulation uses CAD mass/inertia and the source's material, actuation, payload,
and contact assumptions. A successful run does not select real motors or prove
that purchased parts fit their envelope or meet their assumed ratings.

Three deterministic identities prevent reusing evidence after edits:

- `recipe_sha256` identifies expanded parts and joints.
- `mechanism_sha256` identifies the normalized complete source.
- `model_sha256` additionally binds derived physical metadata and registered
  component identities.

`validate_constructed_plan(plan)` checks these bindings. They are identity
checks, not signed certificates or proof of mechanical correctness.
