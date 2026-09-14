# CYBR Compact Torque-Biasing Differential Concept

This package contains actual parametric/CAD-derived geometry and hardware renders. It is not image-generated geometry.

## What is modeled
- 104–108 mm carrier cage with six inspection/lightening windows
- Dual 104 mm output flanges with 8-bolt patterns
- Dual 26-tooth visual splined outputs
- Opposite-hand 24-tooth helical side gears
- Six parallel-axis 13-tooth helical pinions and axles
- Dual multi-plate clutch/preload stacks
- Dual modeled coil preload springs
- Dual ball-bearing assemblies with discrete balls/races
- Carrier end plates, hubs, ring-gear mounting flange, and tie bolts

## Files
- `torque_biasing_differential.step` — complete CAD compound assembly
- `torque_biasing_differential.glb` — colored real-time scene with separate component nodes
- `stl/` — 38 separately exported component meshes
- `build_model.py` — CadQuery parametric model generator
- `render_robust.py` — headless VTK renderer used for the supplied images/video
- `renders/` — assembled, rear, transparent, exploded and internal-core views
- `videos/turntable.mp4` — assembled orbit
- `videos/core_orbit.mp4` — exposed mechanism orbit

## Engineering status
This is a coherent concept model derived from the earlier visual, not a production-qualified differential. The tooth forms are twisted trapezoidal/helical render geometry rather than generated ISO involute gears; splines are visual rather than tolerance-class splines; bearings and clutch preload are spatially modeled but have not been contact/FEA/lubrication validated. Do not manufacture this as a drivetrain component without redesign and analysis.

All dimensions are millimeters.
