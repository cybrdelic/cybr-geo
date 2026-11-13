# Analytic lofts through sampled sections

`Solid.loft(sections, ruled=True)` complements the existing triangle-mesh `loft`.
It takes at least two corresponding planar polygon rings, with finite shape
`(section_count, points_per_section, 3)`. Do not repeat the first point at the end.
It returns one capped B-rep solid and can be passed directly to the existing
`union`, `subtract`, `intersect` and `mesh` methods.

The implementation delegates lofting and solid Boolean operations to
**CadQuery/OpenCascade**, the existing optional analytic backend. CYBR GEO does
not claim to implement that kernel.

Ruled side faces are the default. `ruled=False` requests the backend's smooth
interpolation; it does not promise a curvature tolerance, automatic section
matching, or an absence of self-intersections for arbitrary inputs. Polygon
samples remain polygon edges around each section. This operation does not accept
holes, mixed ring cardinalities, nonplanar rings, duplicate adjacent points or
repeated neighboring sections. The B-rep is checked for backend validity and
positive volume. Geometry pair interference remains the construction graph's job.

Unlike extracting a union on a uniform signed-field grid, the analytic route does
not discard thin features because they fall between voxel samples. Exported
triangle meshes are still approximations governed by `mesh` tolerances; narrow
clearances must be qualified against those tolerances.
