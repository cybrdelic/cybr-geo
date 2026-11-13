# One construction workflow

Use the existing types, not a catalog of object-specific classes:

- **Mesh** for explicit surfaces and attributes. Author with `loft`, `sweep`,
  `heightfield`, or fields, then apply explicit edits. Sampling and subdivision
  do not invent the missing real-world shape.
- **Solid** for optional analytic operations (CadQuery/OpenCascade). Keep B-reps
  through cuts/joins and export once, rather than repeatedly converting meshes.
- **Feature / Frame** for an authored reference measured on real geometry.
  `Feature.at(SurfaceAnchor)` handles a point on a curved triangulated surface;
  explicit `Correspondence` supplies its lineage through edits.
- **Graph** for dependencies. For physical assemblies use `Graph(physical=True)`
  ([protected contract](PHYSICAL_ASSEMBLIES.md)); unsupported joins fail, not waive. Use `op` for intermediate calculations, `part` for
  final solids, and `fit` for feature-driven dependent construction. `edit` is the
  transactional change/rebuild entry point. Earlier `op`+`body` remains supported
  but is not an additional workflow new users need to learn.

Existing root imports are preserved for compatibility. Advanced operations are
also available in their focused modules; this pass adds no new root export.

## What this repair pass changes

1. Profiles share one planar-ring validator across mesh extrusion, CAD extrusion,
   CAD loft and shell sweeps. Invalid crossing/touching/nested/backtracking rings
   fail before construction. CAD loft no longer triangulates a mesh cap just to
   validate its input.
2. `cut_through(..., frame=frame)` handles straight through-cuts along any rigid
   orientation and derives depth from the finished host. The no-frame XY/Z path
   is unchanged. Profiles are local to the frame, not world XY coordinates.
3. `sweep(..., closed=True)` closes the actual topology and distributes transported
   frame holonomy around the loop. No duplicated end ring or seam cap is needed.
4. `Feature.at(anchor)` supplies a point/frame on a curved triangle surface.
   `Feature.follow(correspondence)` preserves explicit lineage. A Feature-valued
   graph node can be the `fit(target='contact')` dependency, so host edits can
   rebuild their reference and then dependent placement.
5. Anchor input validation and rotation immutability defects are fixed.

## What this does not fix

A point frame is not a curved contact footprint, insertion corridor, or a model of
fabric. A loop can still self-intersect if the chosen section is too large or its
path folds onto itself. Profile validation does not certify a swept 3D volume.
Arbitrary CAD remeshing still needs authoritative correspondence; named circular
vertex labels may cease to be circular after subdivision and correctly fail.

Pair-wide overlap exceptions still do not express feature-local contact rules.
There is no automatic closed-loop design solver, universal nonintersection,
production retopology/UV pipeline, or proof of mechanically functional assemblies.

The test suite demonstrates explicit bounded behaviors. AAA asset quality still
requires geometric authoring, reference-based proportions and visual qualification;
it cannot be obtained from an API name or number of tests.
