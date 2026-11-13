# Protected physical assembly construction

Use `Graph(physical=True)` when the output is a physical assembly, rather than a
free surface or intermediate calculation. This is a construction contract, not
an optional diagnostic flag passed to a finished model.

```python
g = Graph(physical=True)
g.input('bore_radius', 3.0)
g.input('gap', 0.2)
g.part('housing', make_housing, 'bore_radius', assume_valid_solid=True)
g.fit('insert', make_insert, 'housing',
      target=Feature('bore'), source=Feature('base'),
      dependencies=('gap',), assume_valid_solid=True)
accepted = g.build()
accepted = g.edit(bore_radius=4.0)
```

`make_insert(frame, gap)` derives its dimensions from the measured host feature.
A single authoritative dependency is the construction method; interference is
still a necessary final backstop. The snippets assume authored factories, not
prefab parts or automatic discovery of engineering intent.

## Mandatory within this workflow

- Final objects use `part` or `fit`. All such objects are evaluated and checked,
  even when the requested output names only one of them.
- `op` values can be arbitrary intermediates, including cutters and incomplete
  surfaces. They cannot be exported as finished physical geometry unless they
  are registered final parts. Hidden meshes/unknown geometry containers fail.
- Accepted output containers are recursively immutable. Duplicate references to
  the same Mesh are rejected as ambiguous physical output; independent instances
  must be separately registered parts, not alias copies.
- `build(require_checked=False)` does not disable physical protection.
- Final parts must be individual connected, closed meshes. Concatenating many
  disjoint surfaces into one mesh is rejected, not used to evade pair checks.
- All final part pairs must be separate with their required clearance. There is
  no physical-mode `allow_overlap` exception. Late `body` registration is also
  rejected; final declarations must be explicit from construction.
- Generic `replace` cannot erase a physical `fit` relationship by substituting an
  unbound factory. Rebuild its declaration to change the relationship itself.
- Failed builds do not replace the accepted output. `edit` additionally rolls
  back changed inputs.

## Narrow join/contact semantics

For a material join, construct an actual connected solid using the existing
`Solid.union` or a genuinely connected boundary representation, then declare that
as one final part. Two intersecting rendered meshes plus a 'welded' label are not
an implemented union. Temporary cutters remain intermediate values.

A mechanical fit uses measured features to construct the dependent part and its
positive clearance. Exact tangency is **not supported by the mesh pair checker**:
it cannot reliably distinguish touching from penetration, so unresolved contact
is rejected rather than waived. No fake touching/contact approval is provided.

This intentionally means some sewn softgoods, press-fits and touching mechanisms
cannot yet be accepted in this workflow. A stitched sheet is not automatically a
closed solid. Arbitrary mesh-to-solid Boolean joining, footprint-conforming
attachments and feature-local contact certification remain unsupported.

## Scope, not a universal guarantee

The explicit `assume_valid_solid=True` precondition remains: final meshes are
assumed non-self-intersecting valid solids. Topology checks do not prove geometric
injectivity. Current protection is inter-part interference at the built pose,
not dynamic safety, material strength, full functional fit, or aesthetic quality.
Sealed cavities with disconnected boundary shells are conservatively rejected by
this workflow until they can carry an authoritative solid representation.

A single final connected part is legal and reports `registered_only`, not a
fabricated inter-part pass. No artificial second probe is needed. Ordinary
`Graph()` remains available for nonphysical surface workflows and legacy code;
its output must not be represented as a protected physical assembly. Direct
geometry functions are still geometry functions, not a full assembly solver.

Tests cover attempted output bypass, hidden components, forbidden waivers,
partial builds, invalid placement and edit rollback, real union, and dependent
feature-driven reconstruction. These establish the declared contract, not all
possible physical correctness.

As with the rest of Graph, operation factories must be pure and cannot mutate
inputs or external state. This contract is not a Python security sandbox: code
that deliberately modifies private members or writes files outside the accepted
build is outside it. The protected result is the accepted typed build, not every
arbitrary object a caller can create elsewhere in Python.

Separation alone does not reject a detached but noncolliding component. A declared
`fit` constructs its source frame at the target frame and rebuilds from the host;
it does not establish the whole contact footprint of a strap or stitched panel.
Such an attachment requires additional geometry/physical semantics not yet
implemented, and must not be described as certified by this contract.
