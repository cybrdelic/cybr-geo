# Geometry-bound mates

`Feature` resolves a named **vertex attribute on actual geometry**, rather than
accepting a second manually maintained coordinate for the hole. It measures a
planar circle, its center, normal and radius. Circle orientation needs a normal
sign hint and an explicit phase direction: symmetry cannot supply those.

```python
seat = geo.Feature('bearing-seat').resolve(housing)
shaft_radius = seat.fitted_radius(radial_clearance=0.2)
shaft = make_shaft(shaft_radius)
shoulder = geo.Feature('shoulder').resolve(shaft)
assembled = geo.place(shaft, shoulder, seat, angle=30, slide=-24)
```

The executable example `examples/bound_cartridge.py` puts all of these operations
in the existing `Graph`. Edit only the bore radius or offset. Geometry is rebuilt,
the seat is measured again, and the shaft is sized and placed from that result.
Changing rotation alone does not rebuild either part. A failed feature resolution
raises `BuildError` and preserves the last good committed build.

Existing kinematics accepts the resolved feature without copied coordinates:

```python
rig = geo.Kinematics().fixed('housing')
rig.revolute('shaft', 'housing', seat.origin, seat.axis)
pose = rig.solve({'shaft': 30})
```

Build this rig as another Graph operation depending on `seat` if the geometry is
editable. `mate` directly provides axial rotation, sliding and opposed alignment;
it does not solve a network of competing mates.

## Contract and limits

- Feature labels use scalar vertex attributes. The geometry producer explicitly
  preserves or recreates semantic labels after edits. There is **no nearest-point
  fallback**, and no claim that arbitrary remeshing/CAD changes retain identity.
- Circle validation checks planarity, fit residual and complete angular coverage.
  Missing labels, degenerate/collinear/noncircular selections, mixed rims,
  ambiguous orientation and unusable phase hints fail clearly.
- Tolerance is absolute in the model's units. Radius is measured on polygon rim
  vertices, not a claim that tessellated side walls are analytically cylindrical.
- Fit clearance is radial, not diameter. It is an explicit sizing dependency;
  arbitrary preexisting geometry is not silently resized.
- A mate proves frame alignment, **not** clearance, load capacity, assembly
  manufacturability, watertight fusion or mechanical simulation. Collision and
  structural verification are separate checks.
- Graph operations remain pure and cache whole operations. This adds no new graph
  framework, scene registry or part catalogue.

## Verification

`python -m unittest discover -s tests` passes 97 tests in this branch (91 baseline,
6 added). Added checks cover radius/position propagation, pose-only rebuild reuse,
rollback, lost/invalid features, arbitrary frame rotation, and clearance guards.
The construction trace records all 53 executable statement lines of the example,
441 expression evaluations and 223 completed line visits. It compares the final
scene to an uninstrumented execution. Nongeometric Python context is explicitly
labeled as context, not pretended to have geometry. Library/backend internals are
included as source but are not falsely claimed to be line-traced in the movie.
