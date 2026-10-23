# Source-bound clearance analysis

Run the IRIS axial broad phase after rebuilding native CAD from the current
specification and source files. From the repository root, with the package
installed or `src` on `PYTHONPATH`:

```sh
python -m examples.analysis.iris09_axial_broadphase \
  --native path/to/current/iris-native \
  --output path/to/iris-axial-broadphase.json
```

The client verifies the native build and its source binding before calling
`PlanarNativePath.axial_clearance_broadphase(native_directory, verified_receipt,
required_clearance_mm=0, distance_margin_mm=1e-4)`. The shared method imports
every placed STEP in the verified manifest and screens every **cross-body**
part pair. A part's native STEP Z bounds remain unchanged over the complete
source-bounded motion because the planar path permits only XY translation,
rotation about Z, and a fixed body Z offset. If two widened axial slabs are
separated by more than the required clearance plus numerical margin, their
3D solids are separated throughout that ideal rigid path. This uses no sampled
poses or pairwise B-rep distance queries.

The JSON lists each proven `axially_separated_pair` with a conservative gap
lower bound, and every other pair as `unresolved`. It also gives part STEP and
B-rep hashes, source/implementation/native receipt hashes, inventory counts,
and status for the six priority IRIS leaf, rod, pin and housing interfaces.
Only the separated pairs have a clearance result; resolve overlapping slabs
using exact native swept distance analysis or documented intentional-mate
evidence. The broad phase says nothing about part tolerances, bearing fits,
elastic deflection, contact dynamics, manufacturing capability or hardware
qualification. Rebuild and rerun when any source, code, or native STEP changes.
