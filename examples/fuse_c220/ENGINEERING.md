# FUSE C220 engineering qualification

The recovered printer source now has two separate evidence layers. The functional-path layer proves nominal CAD passages, selected interference/contact relationships, full-travel kinematics, strict G-code replay and current V9 rendering. The engineering layer adds mass, drive-load, first-order structure, screw-buckling, bed-deflection and imposed-flow screens, then exports the same CAD-derived state to CYBR PHYSICS.

This is intentionally not equivalent to a physical machine release.

## Computationally closed in this revision

`engineering_qualify.py` derives screened assembly/head/bed/gantry masses from the active CAD and explicit density assumptions. It calculates ideal X/Y inertial torque at the recipe's 600 mm/s² acceleration, Z lift torque under a disclosed screw-efficiency assumption, an effective-section X-gantry deflection/stress screen, dual-screw Euler buckling margin, 235×235×4 mm bed-plate gravity deflection and the real generated calibration path's imposed volumetric flow. Any hard failure exits non-zero.

`export_cybr_physics_manifest.py` transfers those CAD-derived values and nominal machine dimensions to the separate CYBR PHYSICS repository. The physics runner must remain deterministic and must state its reduction assumptions; it is not allowed to label a reduced beam/coupon model as CAD-resolved continuum FEA.

## Deliberately unresolved release inputs

A green engineering workflow still reports `NOT_RELEASE_READY`. The following require supplier data, measured parts or a physical machine and therefore remain UNKNOWN rather than being assigned invented values:

- selected stepper torque-speed curves, driver current/voltage and drivetrain friction;
- belt construction, preload, working tension, tooth-shear and fatigue ratings;
- rail/block preload, C/C0 life ratings and lubrication;
- selected T8×8 screw/nut tolerances, efficiency, backlash and critical-speed data;
- hotend heater/thermistor specification, heatbreak calibration, melt pressure and maximum stable flow;
- bed heater power, insulation, temperature uniformity and thermal-runaway behavior;
- PSU/controller/connector/fuse/wire/grounding/creepage/firmware release data;
- complete purchased/printed-part tolerance stack, shrinkage and assembly datum plan;
- measured modal response, accelerometer/input-shaping characterization and backlash;
- molten-polymer behavior, layer adhesion, dimensional accuracy and print-quality coupons;
- actual assembly, electrical commissioning, thermal tests and physical prints.

Until those inputs exist, the machine is an increasingly constrained digital engineering prototype, not a claim that building the nominal CAD exactly will produce a safe or high-quality printer.

## Reproduce

```bash
python examples/fuse_c220/verify_mechanism.py
python examples/fuse_c220/verify_edges.py
python examples/fuse_c220/verify_hotend.py
python examples/fuse_c220/engineering_qualify.py --out build/fuse-c220-engineering
python examples/fuse_c220/export_cybr_physics_manifest.py --out build/fuse-c220-engineering
```

The GitHub engineering workflow reruns the recovered regression suite before accepting the engineering report, so the qualification additions cannot silently bypass the hotend/G-code fixes.
