# Shared photographic rendering / version 0.6

CYBR GEO has two photographic backends with deliberately different roles.
**V9 is the public default.** `photoreal` is the earlier native product-studio
renderer retained for compatibility and controlled engineering photography.
They are not aliases and should not be described as the same image-quality tier.

| Entry point | Default | Explicit alternates |
|---|---|---|
| `lab render` | V9: Mitsuba/HDRI/OIDN, 1100×825, 256 spp, depth 14 | `--renderer photoreal`, `--renderer pbr` |
| `lab video` | V9, 1280×720, 128 spp/frame | `--renderer photoreal`, `--renderer pbr` |
| `lab film` | V9, 1920×1080, 144 spp/frame | `--renderer photoreal` |
| `cybrgeo render` | V9, 1100×825, 256 spp | `--backend photoreal`, `--backend pbr` |
| `cybrgeo video` | V9, 1280×720, 128 spp/frame | `--backend photoreal`, `--backend pbr` |

See [the exact V9 contract](RENDER_V9.md) for the approved ORBIT reference,
asset provenance and reproduction details.

## V9

The default V9 backend is the same renderer family as the approved ORBIT V9
artifact:

- Mitsuba 3 path tracing with a physical thin lens.
- Captured CC0 `small_workshop` HDRI illumination/background.
- Finite rendered bench with a scanned CC0 `blue_metal_plate` roughness map.
- Principled PBR material translation with deterministic subtle per-part
  variation; the ORBIT reference material names preserve their exact V9 tuning.
- Linear-HDR beauty, albedo and shading-normal AOVs.
- Intel Open Image Denoise 2.5.1 high-quality guided denoising.
- ACES fitted tone mapping followed by sRGB.
- No generated imagery, composited background plate, sharpening or optical-flow
  frame synthesis.

The two Poly Haven assets are downloaded through their API and cached with
provider MD5 checks. OIDN is used from `PATH` or `CYBR_GEO_OIDN`; Linux x86_64
and WSL can use the built-in pinned/checksummed OIDN 2.5.1 acquisition path.
`CYBR_GEO_V9_CACHE` relocates the V9 cache.

Every encoded V9 movie frame is rendered from the actual current geometry pose
and guided-denoised independently. This is expensive by design; choose the PBR
preview path when the task is interactive inspection rather than final imagery.

## Legacy native `photoreal` backend

The existing native C++ path tracer remains supported and tested. Its behavior
includes analytic CAD surface normals, authored IOR/clearcoat/anisotropy,
explicit part-local microfinish, GGX/MIS transport, model-space product
softboxes, thin-lens DOF, neutral color transfer and guide-aware spatial
filtering. Its fixed-camera film mode can conservatively reuse previous-frame
radiance after geometric reprojection.

This backend remains useful for deterministic product-studio imagery, section
views and environments where Mitsuba/OIDN cannot be installed. It is now
selected explicitly with `--renderer photoreal` / `--backend photoreal` rather
than being labeled V9.

The optional `CYBR_GEO_EMBREE_ROOT` adapter applies only to this legacy native
backend. It changes triangle intersection acceleration, not CAD geometry,
materials or light transport.

## Truth and geometry

Both backends consume actual recipe geometry. Truth/provenance checks remain
upstream of polished rendering; a realistic result cannot turn designed,
estimated or inspection-only geometry into measured reference data. Analytic
sections remain actual clipped/capped geometry rather than image-space masks.

## Evidence

`tests/test_v9_render_profile.py` gates the public V9 defaults and the captured-
workshop contract. `tests/test_shared_photo_pipeline.py` covers the common scene
adapter plus legacy-native behavior. Native BSDF and Embree regression programs
continue to test the compatibility renderer separately.
