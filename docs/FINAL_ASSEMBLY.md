# Final physical assembly boundary

`Assembly` is mandatory for a **finalized analytic physical assembly** and its
package export. Modeling remains flexible: create cutters, open surfaces and
uncommitted overlapping candidates first. Do not call those finalized outputs.

```python
from cybrgeo import Assembly, Solid

housing = Solid.cylinder(10, 12).subtract(Solid.cylinder(4, 14).move((0,0,-1)))
shaft = Solid.cylinder(3.8, 12)
a = Assembly({'housing': housing, 'shaft': shaft}, clearance=0.1)
accepted = a.build()
accepted.export('new-output-directory')
# A rejected edit does not change a.last or the accepted inputs:
a.edit(shaft=Solid.cylinder(5, 12))  # AssemblyError: penetration
```

Parts are individually named authoritative `Solid` values. Every pair is checked,
including hidden supports, housings and fasteners. No visibility filter, requested
subset, assumed-valid flag or overlap waiver exists. One part can contain cavity
shells, but disconnected compounds must be declared as individual parts. The core
cannot infer a missing part that was never supplied: the reported membership is
part of the result, not a claim to have inspected an entire project.

`contacts=[('outer_race', 'housing')]` declares intended touching. It requires a
gap no greater than the linear `tolerance` and zero positive Boolean common
volume. It never authorizes penetration; any positive computed overlap rejects.
An undeclared touch, missing declared contact, insufficient clearance, invalid
solid or failed backend query also rejects. Contacts override the *separation*
clearance for that named pair only. They do not specify a feature-local contact
footprint, load, fit class or mechanical joint.

The optional CadQuery/OpenCascade backend performs closed positive-volume solid
validation and self-interference analysis, Boolean intersection and distance.
These are numerical backend decisions, not exact arithmetic guarantees. Reported
tolerance uses the author's consistent model units; OpenCascade also has internal
shape tolerances. Sub-tolerance penetration can be unresolved by a numerical
kernel. Do not advertise universal nonintersection, CCD, engineering certification
or manufacturing suitability. Only one static pose is covered.

`build()` snapshots the successful shapes. `edit(**replacements)` restores prior
inputs and keeps the last accepted build if anything fails. `export()` rechecks
all parts before writing and stages a complete new directory. Existing output
folders are refused, so failed export cannot overwrite a previous good package.
OBJ tessellation is explicitly not a new proof of mesh-level clearance. CAD shape
interoperability is not a Python security boundary: do not mutate exposed shapes.

## Mesh and legacy scope

Mesh-only finalization is rejected as unresolved, rather than trusting
`assume_valid_solid`. General `Graph()` and raw mesh exporters remain useful for
surfaces and intermediate geometry. They do **not** grant this assembly contract.
`Graph(physical=True)` retains its previous mesh separation contract and now also
rejects detected nonadjacent face intersections. Its BVH does not certify adjacent
folds or all self-contact. Do not confuse that legacy bounded check with analytic
assembly finalization. Use Graph to compute dependencies, retain Solid values, and
pass the full final participant mapping to Assembly.

No asset-specific logic, prefab parts, rendering or material authority was added.

## Incremental revalidation

Each build and export fingerprints the **complete serialized B-rep** of every
participant. Accepted validity and pair evidence is reused only when content and
pair policy (including clearance, tolerance and contact intent) match. A changed
part invalidates its own validity and every incident pair; unrelated pairs remain
covered by their previous evidence. Reports list evaluated/reused bodies and pairs.
Bounds, volume and Python object identity alone are never cache keys. Backend
queries use private shape copies because Boolean operations may attach auxiliary
representation data. Failed validation does not commit partial evidence. Changes
to exposed shape contents are detected on the next build/export. The cache is
in-memory, local to one Assembly/runtime, not a portable certification database.

The reproducible `benchmarks/assembly_incremental.py` nested-shell case avoids the
AABB separation shortcut. On this execution environment, 12 solids/66 pairs took
1.60 s cold, 0.013 s unchanged and 1.19 s after changing one ring. That edit reused
55 pair results and evaluated 11. These are measured workload results, not promised
speedups for every asset; first-time complex Boolean validation can still be slow.
