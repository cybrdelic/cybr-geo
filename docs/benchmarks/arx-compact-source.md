# Compact ARX source inspection

Source inspection and shared-DAG adaptation of [cybr-arx6-compact at 0003382b](https://github.com/cybrdelic/cybr-arx6-compact/tree/0003382bd6a2e96f3bed065003efec35001982f7).
This is the distinct 278-part compact-cartridge design. Its observations and
reconstructed simulation do not inherit the belt design's electrical, thermal,
safety or scenario results. The attributed design data and one initial captured
pose are used by the [compact geometry example](../../examples/engineering/arx_compact/README.md)
under the user's explicit authorization as owner. No cinematic renderer,
controller or source film is imported by that example.

The useful shared geometry boundary is already compatible with GEO's shape DAG:
[`build_meshes.py`](https://github.com/cybrdelic/cybr-arx6-compact/blob/0003382bd6a2e96f3bed065003efec35001982f7/source/build_meshes.py)
evaluates nodes with `op`, `args`, and `children`: box half-extents; cylinder
radius/half-height; polygon points/half-height; helix radius/wire-radius/pitch/turns;
row-major 4x4 transforms; binary union, cut and intersection. It caches by
serialized node and tessellates native OpenCascade geometry at 0.045 mm/0.12 rad.
Those are rendering settings, not manufacturing tolerances. Existing cache files
are skipped, so rerunning that script is not necessarily a fresh geometry check.

[`geometry.py`](https://github.com/cybrdelic/cybr-arx6-compact/blob/0003382bd6a2e96f3bed065003efec35001982f7/source/geometry.py)
assigns body-local millimetre parts to named rigid bodies, mapping `fixed` to
`fixed_hardware`. Its captured body records contain SI position plus a row-major
3x3 rotation. This maps naturally to the shared physical replay adapter. The film
adds linear position interpolation, SO(3) rotation interpolation and saved
explosion offsets; these are presentation operations and should not become
engineering-state or physical disassembly claims. Its experimental CSG ray-query
compiler emits primitive inverse transforms and a postfix Boolean tape, while
the production raster film uses native tessellated meshes.

[`capture.mjs`](https://github.com/cybrdelic/cybr-arx6-compact/blob/0003382bd6a2e96f3bed065003efec35001982f7/source/capture.mjs)
implements a reconstructed computed-torque/PD controller: 55 Nm caps for the
first three axes, 12 Nm for the wrist, a bounded 14 N jaw force command, and linear
screening spring torques. The free puck moves under contacts, not a tool weld.
There is no motor electrical/thermal/safety state model in this controller.
The [capture XML](https://github.com/cybrdelic/cybr-arx6-compact/blob/0003382bd6a2e96f3bed065003efec35001982f7/source/capture-model.xml)
contains opposing jaw slides, a jaw equality and rack-pinion tendon, but lacks
full arm material contact geometry and uses broad reconstructed hinge ranges.
The capture loop records body transforms without refreshing kinematics after
the preceding integration step; the shared runtime refreshes `mj_kinematics`
when capturing, keeping its body poses aligned with the recorded state.

An independent execution of only the inspected native `build` function confirms
that the two torsion-spring instances are valid compounds containing two solids
each, including a detached leg. CB2's native volume is 18777.5475184094 mm³ and
its maximum Z is 92 mm, exceeding its authored 74 mm bound by 18 mm. CB3's native
volume is 5021.3528346917 mm³. Shared-DAG evaluation agrees within 1e-8 mm³.
This establishes source parity and exposes a source discrepancy; it does not
repair the spring or establish a mechanically continuous manufactured part.

The implemented example evaluates all 278 parts and 1,679 nodes with the shared
`ShapeEvaluator`, applying the source's initial rigid-body pose with explicit
SI-to-mm conversion. Native STEP readback contains 280 solids because the two
spring compounds each contain two solids; GLB readback contains 278 named nodes.
Fresh hero and side images and source/native parity evidence are included with
the example. This is geometric source parity, not a new dynamics result or a
qualified physical source for the compact design.
The pinned [README rights statement](https://github.com/cybrdelic/cybr-arx6-compact/blob/0003382bd6a2e96f3bed065003efec35001982f7/README.md#provenance-and-rights)
reports no project-wide license grant. Its MuJoCo notices cover that dependency,
not the design assets or film. The example relies on the owner's explicit reuse
instruction, not an inferred upstream public license grant; attribution and
design-specific scope are retained.
