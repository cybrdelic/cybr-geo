# Pinned compact geometry oracle

`build_meshes.py` is an unchanged copy of
[`source/build_meshes.py`](https://github.com/cybrdelic/cybr-arx6-compact/blob/0003382bd6a2e96f3bed065003efec35001982f7/source/build_meshes.py)
from commit `0003382bd6a2e96f3bed065003efec35001982f7`.
Its full-file SHA-256 is
`1b2e4f087634f8ff41ad450195a2a35ef3ef6445d1bc25efbad9c797f7fb2e50`.
The fixture retains the original source bytes and notices without modification.

The regression test verifies this hash, then compiles only the original
`build()` function at lines 10-27 from its syntax tree. It supplies the original
CadQuery, OpenCascade, NumPy, JSON and cache dependencies. It never imports the
fixture as a module or executes its asset reads, tessellation loop or writes.
The independent original builder and GEO's shared DAG evaluate the same pinned
part trees on the same installed CAD kernel. The input design is the already
retained `examples/engineering/arx_compact/data/design.json`, SHA-256
`48b3dcf993f78a9e1f386c8458644bb5829d627c895b88e58e616ae14aa565a0`.

The spring regression compares volume, solid count, component volume, center of
mass and bounds. It preserves both source springs' two disconnected solids and
the first spring's 18 mm extension beyond its authored bounds. Measurements in
the example's archived Windows receipt remain specific to that run; different
OpenCascade versions may integrate swept volume differently. This test does
not loosen its volume tolerance or repair the source geometry, and it does not
establish physical functionality or hardware qualification.

Rights follow [the repository source notices](../../../NOTICE-SOURCES.md).
The source owner Alejandro Figueroa / cybrdelic explicitly authorized reuse in
CYBR GEO. The pinned compact repository states that no project-wide license
grant was present; public availability is not treated as such a grant. No new
license is assigned to this retained original fixture. CYBR GEO's new test code
and existing project license remain GPL-2.0-only, and dependencies retain their
own licenses.
