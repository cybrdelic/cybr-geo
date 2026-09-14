# REACH-2 v6 — native CYBR GEO rebuild

This revision replaces the REACH-2 v5 delivery path. V5 used CadQuery + mechanism_lab as the public model API and only reached V9 through a custom wrapper. V6 must be a real CYBR GEO model:

- `build() -> cybrgeo.Assembly`
- all authored solids enter through `cybrgeo.from_shape`
- analytic BReps are stored in `Assembly.cad`
- export uses `Assembly.save()` / `Assembly.export_glb()`
- final path trace uses `cybrgeo.photoreal.render()` (V9 backend)
- no image generation

## Packaging correction

The Ø155 FHA-25 integrated actuator is rejected because it overwhelms the ORBIT wrist visually. V6 returns to a compact size-20 strain-wave gearhead envelope (CSG-20-160-2UH/LW class, ~93 mm OD) and a 40 mm / 100 W inline servo envelope. The joint remains coaxial and narrow; the motor sits behind the reducer instead of beside it.

The target visual language is the existing ORBIT wrist: graphite tapered support, satin circular bearing/reducer interfaces, exposed steel fasteners, small teal service parts, and no monolithic rectangular gearbox body.

Engineering gates remain separate from rendering. Vendor dimensions and final purchased-part STEP overlays remain required before manufacturing release.
