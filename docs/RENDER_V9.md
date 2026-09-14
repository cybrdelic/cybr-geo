# CYBR GEO photographic contract / V9

**V9 is the captured-workshop Mitsuba renderer used for the approved ORBIT V9 image.**
It is not the older CYBR native softbox renderer. The native BVH/GGX/MIS path
tracer remains available explicitly as `photoreal`; V9 is now the ordinary
`lab` and `cybrgeo` rendering default.

## Exact approved V9 reference

The successful ORBIT V9 workflow rendered this stack:

- Mitsuba 3.7.1 path tracing (`llvm_ad_rgb` where available).
- 1100 × 825 pixels, 256 samples per pixel, 14-bounce maximum depth.
- 72 mm thin lens at f/16.
- Poly Haven `small_workshop` 2k HDRI, CC0, rotated 195 degrees and scaled 0.72.
- A finite rendered workbench with the CC0 Poly Haven `blue_metal_plate` 1k
  scanned roughness map. The room is illumination/background from the HDRI; it
  is not a pasted photograph behind the CAD.
- Gaussian reconstruction filter, standard deviation 0.42.
- Linear-HDR beauty, albedo and shading-normal AOVs passed to Intel Open Image
  Denoise 2.5.1 at high quality.
- Exposure 1.04, ACES fitted tone curve, then standard sRGB transfer.
- No generated imagery, compositing, frame interpolation, sharpening or
  screen-space fake mechanical detail.

The corresponding code is now generalized in `mechanism_lab.v9` so the same
renderer can consume any truthful CYBR GEO `Assembly`, rather than being an
ORBIT-only presentation script.

## Public defaults

| Path | Default backend | Default budget |
|---|---|---|
| `lab render` | `v9` | 1100×825, 256 spp, depth 14 |
| `lab video` | `v9` | 1280×720, 128 spp/frame, depth 12 |
| `lab film` | `v9` | 1920×1080, 144 spp/frame, depth 14 |
| `cybrgeo render` | `v9` | 1100×825, 256 spp, depth 14 |
| `cybrgeo video` | `v9` | 1280×720, 128 spp/frame, depth 12 |

Every V9 movie frame is independently rendered from the actual geometry and
OIDN-guided in linear HDR. The implementation does not use optical-flow frame
synthesis. For fast engineering inspection select `pbr`. For the previous
in-house product-studio path select `photoreal` explicitly.

## Materials

V9 translates authored CYBR material values into Mitsuba principled BSDFs and
retains deterministic, subtle per-part variation so assemblies do not read as
one perfectly uniform CG material. The ORBIT material names used in the
reference image carry the exact V9 retuning used by the original successful
render; unrelated recipes retain their authored base material values.

That means the V9 visual character comes from **real captured environment
lighting, PBR transport, realistic roughness response, physical lens framing and
guided path-trace denoising**, rather than from pushing contrast or blurring the
older studio output until it looks photographic.

## Assets and reproducibility

The renderer obtains the two CC0 Poly Haven assets through the provider API and
caches them under `~/.cache/cybr-geo/v9` by default. Downloaded files are checked
against the provider MD5 values. Set `CYBR_GEO_V9_CACHE` to relocate the cache.

Intel OIDN is used from `PATH` when available. On Linux x86_64/WSL the renderer
can obtain the exact 2.5.1 binary release automatically and verifies its pinned
SHA-256 before extraction. Set `CYBR_GEO_OIDN` to use an existing executable.
Other platforms should install OIDN separately.

Mitsuba is a normal CYBR GEO Python dependency in the V9 release and the tested
requirements pin 3.7.1, matching the approved render environment.

## Truth boundary

V9 improves photographic presentation; it does not promote design assumptions
to measurements. Existing truth/provenance gates still run before rendering.
A concept render remains a concept, and a realistic workbench/HDRI does not
establish load capacity, manufacturing tolerance, CFD performance or physical
prototype validation.

## AERIS integration target

`examples/aeris/` is the first large useful-machine integration target for the
shared V9 backend. It contains 215 named components, 202 analytic CAD components,
a true CAD cutaway, impeller, bearings, motor internals, routed conductors,
filter geometry, lattice/gyroid topology, export verification and whiteprints.
Its normal render script now chooses V9; `--native` selects the old native
photographic backend and `--pbr` selects the raster preview explicitly.
