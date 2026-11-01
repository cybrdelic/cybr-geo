# Shared GEO library review

This clean review snapshot is based on public master
`1c689f750d608388e5a758f61b24b736afbd948a` and contains the current shared library
source from `3702135584161b656e1120235dea589068a3c677`, package version
`0.6.0+rewrite.20261101.2`.

The prerequisite mechanical source and example closure were already published
in draft PR #24 at `691c4546eaf441cd638087dd982c8a06a5fc2ba1`. Those existing
public objects are reused rather than importing private repository history.
All 626 prerequisite changed paths were checked byte-for-byte against that
public tree before publication-specific metadata cleanup. The 28 later changed
paths are generic library code, synthetic regression tests, dependency profiles
and two already-public IRIS consumer corrections. No new BREP/CAD/body dataset,
private platform tree, 547-part ARX packet, Blender scene or private recovered
archive is included. Machine paths were removed from three historical JSON receipts; their numeric
evidence remains unchanged. Eight optional historical operational receipts containing account/machine
metadata are excluded from this overlay (Library-save manifest, failed-run
receipt/log and five compressed local invocation records). No source or numeric
algorithm is removed.

## Changes

- Bounded scene IO and capability-specific dependencies; SCENES can use the
  shared streaming source/hash owner without loading the CAD/render stack.
- Serializable component, pose, motion, mass and export authority, plus explicit
  inspection versus qualified manufacturing-export policy.
- Generic sleeved fasteners, rack/contact/transmission helpers, bounded shaft
  control and source-preserving optical/volume-coordinate lowering.
- Native shape-DAG exchange using existing CadQuery/OpenCascade construction,
  exact native bounds, single-root STEP reuse and analytic profile primitives.
- IRIS initial physical placement and clearance consume one analytic body layout.

## Validation and limits

A fresh exact-byte candidate passed **104 focused tests** across 11 test files,
including real native CAD/STEP readback, synthetic geometry, component authority,
streaming IO and optical contracts. Existing strict CI suites are retained;
hosted success must be checked at this PR's exact head. No Windows, macOS, GPU,
full renderer or manufacturing qualification is inferred from the local checks.
Existing historical media and evidence remain historical, not newly rendered
acceptance. No merge, auto-merge, deployment or paid infrastructure is requested.

The existing GPL-2.0-only license and all source notices are retained. No new
project-wide license is asserted. The private platform remains private.

```sh
python -m pip install '.[cad,test]'
python -m pytest -q tests/test_assembly_model.py tests/test_assembly_physics.py \
  tests/test_inspection_optical_plan.py tests/test_native_exchange.py \
  tests/test_optical_coordinates.py tests/test_rack_contact.py \
  tests/test_scene_io_streaming.py tests/test_shaft_speed_control.py \
  tests/test_shape_dag.py tests/test_sleeved_fasteners.py tests/test_volume_coefficients.py
```
