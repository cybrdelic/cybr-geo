# Relationships construct geometry; checks are the safety net

The dependency is one way and explicit:

**host → measured feature → derived dimensions → new part → feature alignment**.

```python
g.part('housing', make_housing, 'bore_radius', assume_valid_solid=True)
g.fit('shaft', make_shaft, 'housing',
      target=Feature('bore'), source=Feature('shaft_base'),
      dependencies=('radial_gap',),
      assume_valid_solid=True, clearance=0.1)
```

`make_shaft(frame, radial_gap)` receives the **measured frame of the actual bore**.
Its radius can be `frame.fitted_radius(radial_gap)`, not a second independently
maintained constant. The factory produces geometry with a labeled source feature;
GEO measures that source and rigidly aligns it to the target. Optional `slide`
and `angle` specify a distance along the target axis and angular phase.

Changing the host's size or position rebuilds the measured reference, dependent
geometry and placement. Missing, ambiguous or invalid labels fail rather than
silently choosing a nearby surface. `g.edit(...)` keeps the last accepted state
when construction or its safety checks fail.

This reuses the existing general `Feature`, `Frame`, `place` and graph machinery.
There is no built-in shaft, bearing, tree or other part catalog. Factories remain
normal author-visible modeling code. Labels must be preserved or explicitly
recreated by the producing operation: this is not universal CAD topological naming.

## Through-cuts follow the actual finished host

```python
g.op('finished_host', build_host, 'thickness', 'height')
g.op('cut_host', lambda host: host.cut_through(profile), 'finished_host')
```

`Solid.cut_through(profile, *more_profiles)` derives cutter depth from the finished
host's bounds projected onto the requested cutting axis and performs the profiles in one OpenCascade-backed batch.
An XY polygon describes the cross-section, not an arbitrary guessed cutter length.
Changes to host depth therefore rebuild a cut that remains through the host.
The combined cutter must remove resolved material, otherwise the operation fails.

With no frame, profiles lie in XY and cut along Z. Supply `frame=Frame(...)` to
interpret the same profile in local XY and cut along that frame's local Z.
Current scope is straight polygon-prism cuts along a rigid axis. It does not infer a manufacturing
process, choose a bore automatically, preserve CAD-face identity or support every
possible curved through-cut. It is attributed to CadQuery/OpenCascade, not an
in-house analytic geometry kernel.

## What is solved, and what is checked

**Constructed from relationships:** deriving a dimension from a measured circular
feature, dependency-driven rebuild, rigid feature alignment and host-depth-driven
cuts. The author supplies the factory's dimensional law (such as bore radius minus
gap); GEO does not invent the engineering design.

**Checked afterward as a safety net:** interference/clearance of the final registered
solids. The gate now uses a bounded distance query where only a clearance decision
is needed. No near surface contact still requires containment checks. Such passes
report `gap_kind='threshold_lower_bound'`, not a fictitious nearest distance.

**Not provided:** closed-loop multi-part constraint solving, collision-free path
planning, automatic aesthetically realistic geometry, material strength validation,
continuous-motion certification or universal self-intersection prevention.

`tests/test_relationship_build.py` demonstrates bore-radius/position edits rebuilding
the dependent shaft, lost references failing closed, bad edits rolling back and
through-cuts following changed host depth. These small tests prove the relationship
behavior; they are not substitutes for finished visual examples.
