# Connected polynomial surface construction

Run `PYTHONPATH=../geo_patch_work python build.py` from this directory.
The video shows every authored line and its modeled consequence. Import lines
have no fake geometric effect; station arrays are drawn as 3D construction
stations. Control nets, growing surface domains, moving continuity control rows,
trim boundaries, normal witnesses and a vertex-normal offset are animated in the
viewport. This is explanatory interpolation of actual operation inputs/results,
not a recording of iterative solver dynamics. The source package includes the
complete CYBR GEO implementation, tests and rasterizer sources.

The intentionally bad join has gap 0.861822, normal angle 0.466042 radians and
normal-curvature mismatch 0.306851. After C2 control-row matching, the gap and
curvature mismatch are zero and sampled normal angle is < 3e-8 radians.
Sampling uses 65 seam points. The algebraic C2 control-row constraint is exact
up to floating point, but the reported geometric checks are sampled.

## Scope and limits
- General polynomial tensor patches, NOT rational NURBS.
- Exact rectangular parameter trimming, NOT arbitrary trim loops or holes.
- Same-orientation u-edge C0/C1/C2 constraints; matching v degree required.
- G1/G2 follow C1/C2 only for regular patches; singular seams fail.
- Stitching welds explicitly corresponding seam IDs, not nearby vertices.
- Offset is a vertex-normal mesh displacement, not a constant-distance solid.
- Offset checks local face inversions and nonadjacent contacts. Sweep checks
  nonadjacent contacts. Adjacent foldovers and between-sample intersections are
  NOT certified absent. `certified: false` is deliberate.
- The offset has open boundary rims and is not a manufactured solid shell.
- Demonstration is a technical curved panel, not a production car-body asset.

99 tests pass including 8 new patch/trim/stitch/offset/sweep cases.
