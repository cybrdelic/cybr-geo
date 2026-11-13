# Solid interference layer

Run `PYTHONPATH=. python -m unittest discover -s tests -q` (100 tests).
`python example.py` constructs an annular bearing housing, through shaft, and cast-in insert. `python render.py` animates every authored example line, meaningful profile/extrusion operations, retained assembly, witness lines and sampled shaft poses. The housing is cut away ONLY for display. Closed full meshes are used for queries.

`SolidQuery(mesh, assume_valid_solid=True)` deliberately requires an explicit valid, non-self-intersecting, closed-solid precondition. Topological closure alone cannot prove geometric validity. Material/cavity shells must have compatible winding (cavities opposite exterior). Whole-solid reversed winding is accepted. Floating predicates are not exact arithmetic.

`interference(other, clearance=..., allow_reason=...)` returns surface gap separately from solid state. With no surface contacts it classifies a representative of EVERY connected shell by generalized winding. A small wholly contained cube now correctly reports contained_overlap even when surface gap is 1.5. Touching/intersecting surfaces conservatively report contact_or_overlap, not a claim of positive penetration volume. This avoids false safe results for crossings with no interior vertices. Witness point, face pair and optional contained surface point are retained. Explicit intentional intersections waive a violation but never erase evidence.

`thickness(face_ids)` measures normal material chords at selected face centroids, NOT global minimum wall thickness. `sample_motion(fixed, moving_at, times)` checks every supplied pose, NOT continuous collision detection. No penetration depth, guaranteed global minimum thickness, physical structural analysis, electrical analysis or manufacturing certification is claimed.

Cavity/disconnected-component tests, nested cubes, reversed winding, scales 1e-6 to 1e6, gaps, witnesses, touching/crossing/identical cubes, allowed overlap, input preservation and sampled motion are included. Existing MeshBVH surface-distance semantics are unchanged.

Dependencies: numpy, scipy, shapely; render also needs Pillow and ffmpeg. The existing generic renderer is bundled under tools. Core imports require no image generation or proprietary DCC.

Reproduce animation: `python render.py` outputs a 34.8 second 1200x720 MP4. The source panel starts at imports and scrolls with the active line. Imports and assertions explain context without pretending to create model parts. Geometric construction is animated in the viewport. Query witnesses draw in world space, and all actual sampled moving shaft poses are displayed. This is a focused diagnostic assembly, not a photoreal cinematic motor or a full brushless motor.
