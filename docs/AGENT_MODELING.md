# Model the object, not a collection of approximations

This is authoring guidance, not a new geometry subsystem or a claim that GEO
already provides automatic high-quality modeling. It records what worked in the
microphone, guitar and original boot studies, and what their weaker successors
lost. Those studies are useful construction references, not certified products.

## 1. Establish the shape before choosing the shortcut

Study the references and write a short design brief: scale, silhouette, important
sections, thickness, openings, material boundaries and how components connect.
Separate observed shape from assumptions about hidden geometry. For an image
match, compare the same viewpoints; an attractive unrelated shape is not a match.
Choose a representation suited to the subject: smooth solid lofts and actual
cuts for hollow housings, controlled patches for designed surface continuity,
mesh/field/deformation tools where they fit the intended form. Read the actual
available API; do not invent names or use a legacy capability as evidence about
this rewrite. CadQuery/OpenCascade supplies the optional Solid backend; GEO did
not independently implement that geometry kernel.

## 2. Give related geometry one authority

One outline should drive a body's related boundaries; one host surface should
drive its attachments; one measured bore/frame should drive the mating part.
Declare these dependencies so an edit rebuilds them together. Independent copies
of dimensions or a late group translation are not a relationship model.

Concrete lessons from the authoring studies:

- **Microphone:** its shared `grille_point(theta, z)` drives the grille strands
  and boundary; the housing is a hollow construction. Do not approximate each
  strand and seam with unrelated shapes.
- **Guitar:** the same body outline drives the rim, top and back, with a real
  soundhole and cavity. Binding follows that design, rather than concealing a
  mismatch between separate body approximations.
- **Original boot:** `Solid.loft(..., ruled=False)` makes the curved upper; actual
  cuts create the throat and sloped collar. The tongue is separate. Host queries
  locate eyelet axes, and bores are subtracted through the upper. Rocker, arch
  and tread cuts shape a continuous outsole. A tongue over an uncut upper,
  decorative rings without leather bores, flat panels and floating tread blocks
  are not equivalent constructions, even if they contain more triangles.

Reuse these principles, not asset-specific hardcoding in the core. A successful
example is not permission to copy its remaining limitations into every object.

## 3. Work from large form to actual construction to detail

Inspect front, side, rear, top and three-quarter clay views before small details.
Check characteristic curvature, sections, silhouette and proportions. Preserve
intentional creases; smoothing everything can make a designed surface muddy.
For a car, establish coherent body surfaces before splitting panels. Thickening
independent patches can create internal walls and seams rather than a body.

Then construct real thickness, holes, joints and bounded attachments. Use the
host geometry to derive placement and trimming; a guessed surface labeled
“host-conforming” is still guessed. Add tertiary details only after the main
shape and interfaces work. Repetition or noise is not a substitute for anatomy,
cloth structure, botanical growth or designed upholstery.

## 4. Protection has an explicit scope

Follow [the physical assembly contract](PHYSICAL_ASSEMBLIES.md). Applicable
finished assemblies use `Graph(physical=True)`, registered `part`/`fit` outputs,
and declared dependencies. Cutters and incomplete surfaces remain intermediate
operations. Use real unions for material joins, not overlap waivers.

The current contract assumes valid non-self-intersecting solid inputs; topology
checks alone do not establish that assumption. It checks registered inter-part
interference at the built pose, not arbitrary self-intersection, force dynamics,
structural safety or collision-free motion. Exact touching, sewn interfaces and
some multi-shell cavities are unsupported. Separation also does not prove an
attachment is connected. Ordinary `Graph()` and raw geometry functions do not
inherit these assembly protections. Report which path and checks were used.

Do not silently add clearance to an intended contact, hide a failed component,
or claim a universal guarantee. If a required case is unsupported, demonstrate
that limitation and make a general, tested core improvement when appropriate.

## 5. Diagnose before rewriting

Preserve the best earlier source. Re-run that authoring program on old and new
cores with the same inputs, tessellation and view before blaming the core. Also
compare the old and new authoring programs on the same core. Record which
construction changed, rather than asserting regression from a screenshot alone.
A core defect needs a minimal reproducer; an authoring defect needs better
construction. Neither should be patched with an object-specific core branch.

Inspect actual outputs, not just logs: silhouette, sections, openings, normals,
self-intersection, interfaces and supported edit propagation. Numeric checks and
visual review answer different questions. Do not label a result “AAA” merely
because a test passed or a render completed.

## 6. Deliver the actual build and the actual operations

Keep a small build manifest beside each output: authoring entry point and source
hash, core path/revision or hash, parameters/seed, mesh outputs, camera/render
settings, check results and resulting media. Snapshot inputs before rendering;
regenerate stills, videos and source bundles from the same build. This is a
provenance record, not a request for a new framework.

Preserve authored normals, corner UVs and feature correspondence through exchange.
Check the imported asset in the destination renderer. Recomputed normals or a
bare vertex/face export can discard the very surface quality being demonstrated.
Use the requested renderer for claimed renders; generated reference art is not
proof that GEO constructed it. Materials and lighting must not conceal defects.

For line-by-line construction videos, capture intermediate operation states as
well as final parts. Animate a sweep along its path, a loft through its sections,
a cutter entering and removing volume, and a placement along its actual frame.
Show discarded geometry and changed dependencies, not only surviving geometry.
Imports, units, parameters and helper definitions need meaningful explanations
or 3D guides; they must not be falsely shown as executed modeling operations.
Keep the full source visible/available, including helpers. A film is explanatory
replay unless actual simulation is being performed; do not imply physics from an
assembly animation. Inspect its beginning, middle and end before delivering it.
