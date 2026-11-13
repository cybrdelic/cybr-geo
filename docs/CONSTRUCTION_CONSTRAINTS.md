# Construction constraints belong to the build

`Graph` now enforces separation **before it commits** the construction's outputs.
This is not an optional inspection script. Once a final solid is registered,
every pair of registered bodies is separate by default on every subsequent build.
This includes builds requesting only one target. Geometry changes, placement
changes and operation replacements therefore cannot silently bypass the policy.

```python
from cybrgeo import Graph

g = Graph()
# ... inputs and actual modeling operations producing Mesh values ...
g.body('housing', assume_valid_solid=True)
g.body('shaft', assume_valid_solid=True, clearance=0.2)
accepted = g.build()

# Later edits rebuild and recheck affected pairs automatically.
g.set('shaft_radius', 6.0)
updated = g.build()  # Raises BuildError if a pair violates its constraint.
```

The snippet illustrates the interface, not a complete model. Run the complete
[`examples/construction_constraints.py`](../examples/construction_constraints.py)
for a reproducible containment/rejection/exception/correction example.

## Natural construction interface

Use `part()` for final physical bodies, and `op()` for intermediate calculations,
profiles, cutters and feature frames. The distinction is explicit because an
intersecting Boolean cutter is correct, but an intersecting final shaft may not be.

```python
g.part('housing', make_housing, 'bore_radius', assume_valid_solid=True)
g.part('shaft', make_shaft, 'bore_radius', assume_valid_solid=True, clearance=0.2)
accepted = g.build(require_checked=True)
updated = g.edit(**{'bore_radius': 8.0})
```

Here `make_housing` and `make_shaft` are the author's actual modeling functions,
not built-in parts or hidden catalogs. Both depend on the same input; more exact
geometry-bound fits use the existing `Feature.resolve`, `Frame.fitted_radius`
and `place` operations as ordinary dependencies. The graph does not invent a
fit or automatically repair a conflicting shape.

`part()` creates and registers an operation in one call. A graph using `part()`
returns those final parts by default; legacy `op()`/`body()` graphs retain their
previous default targets. `edit(**inputs)` is transactional: failed builds restore
the parameter draft as well as retaining the accepted result. `set()` still allows
an intentionally uncommitted draft. External side effects inside impure operation
functions cannot be rolled back.

## Evidence scope is part of the result

Every `Build.report['coverage']` labels the returned outputs:

- `pair_constraints_passed`: returned meshes are registered and at least one
  pair constraint was enforced incident to each returned body; inspect
  `allowed_pairs` for any exceptions. This is not proof of separation for exempt
  relationships.
- `registered_only`: there is only one participating body, so no pair test proves
  anything about its own self-intersections.
- `all_pairs_exempt`: all pairs were explicitly exempted, not proven disjoint.
- `unconstrained_outputs`: at least one returned body has no enforced incident
  pair. An unrelated valid pair cannot certify that output.
- `unchecked_outputs`: a returned mesh was not registered, or a custom container
  cannot be inspected safely. Lists, tuples and dictionaries are inspected
  recursively, so these containers cannot hide an unchecked `Mesh` value. Raw
  arrays are conservatively unknown, including vertex/face array payloads.
  This tracks typed `Mesh` values, not arbitrary serialized geometry hidden in
  numeric lists or strings; serialized payloads must not be presented as checked
  geometry merely because they accompany a checked output.
- `no_geometry_outputs`: output is non-geometric; it is not a verified assembly.

`build(require_checked=True)` rejects all but `pair_constraints_passed`. Even that
status is **only pair interference under the stated solid assumption**, never a
claim of self-intersection freedom, valid manufacturing, physics or safe motion.

## Rules and evidence

- `body(node, assume_valid_solid=True, clearance=...)` registers a final mesh.
  Pair default clearance is the maximum of the two body clearances.
- `separate(a,b,clearance=...)` sets a specific pair requirement, replacing any
  overlap exception for that pair. It can intentionally override body defaults.
- `allow_overlap(a,b,reason='...')` explicitly exempts a pair. It is not a contact
  solver: it permits all overlap of that pair, so use it only when that is truly
  intended. Its reason and actual measured interference remain in the report.
- Profiles, Boolean cutters, fields and other intermediate graph values are not
  automatically assembly bodies. Only explicitly registered bodies participate.
  An unregistered final object is **unchecked**. Standalone modeling calls outside
  this Graph remain ordinary geometry operations, not guarded constructions.
- Failed builds raise `BuildError` with `.cause.conflicts` (for geometric conflicts),
  giving body names and the `Interference` result. Results include nearest-surface
  witnesses/face IDs when the surface query ran and, for containment, an inside
  point and owner. Broad-phase passes explicitly use `gap_kind='aabb_lower_bound'`
  with no witness: that number is a lower bound, not a measured surface gap. The report
  has `committed=False`; no failed result replaces `g.last` or accepted caches.
- `set()` and `replace()` edit the **draft** and retain that draft even after a
  rejected build. The accepted snapshot remains available in `g.last`. Correct the
  draft and rebuild; this is not a full undo/history system.
- Unchanged mesh queries and pair checks are reused. A changed body rechecks its
  pairs; changed policies recheck the affected pair. `force=True` recomputes all.
  No skipped pair is silently treated as evidence of a pass.

## Exact scope, not a universal guarantee

This uses the existing triangle-mesh closed-solid query, with floating tolerances.
It detects surface contact/crossing and full containment; oriented cavity shells
are supported. Contact is conservatively unresolved and rejected by default.
**Tangency is not distinguished from penetrating overlap**, so there is no
misleading blanket “touch allowed” switch. No penetration depth is reported.

The caller explicitly asserts that each body is a valid, non-self-intersecting
closed solid. Open surfaces are rejected, but this feature does not certify
self-intersection freedom or material physics. It does not automatically adjust
placement, solve collision-free layouts, or validate all poses of a mechanism.
The constraint is checked at the current built pose, not continuously between
poses. Exact CAD-kernel collision, sweep-volume/continuous collision detection,
and local same-body foldover prevention remain separate missing work.

Disjoint axis-aligned bounding boxes can certify a conservative separation lower
bound before an exact pair query. This shortcut is used only above the required
clearance plus numerical tolerance. Overlapping/near boxes still use the solid
query; an overlapping box is never itself treated as an intersection.

The cache reduces repeat work but the pair enumeration is still quadratic in the
number of registered bodies. Query construction and changed-pair geometry work
can remain expensive on large models. No real-time or whole-asset local-edit
performance guarantee is made.
