# Shared photographic rendering / version 0.5

CYBR GEO's two public assembly APIs now route ordinary still and video commands
to one photographic renderer. This is a source and package change; an existing
installation must install the updated package before its defaults change.

| Entry point | Default | Explicit alternate |
|---|---|---|
| `lab render` | Photographic, 512 spp, 14 bounces | `--renderer pbr` |
| `lab video` | Photographic, 128 spp, 12 bounces | `--renderer pbr` |
| `lab film` | Photographic, 144 spp, 12 bounces | Dedicated final-film controls |
| `cybrgeo render` | Shared photographic, 512 spp | `--backend pbr` |
| `cybrgeo video` | Shared photographic, 128 spp | `--backend pbr` |
| `mechanism_lab.pathtrace.render_pathtrace` | Shared photographic compatibility wrapper | Named legacy function |
| `cybrgeo.offline.render` | Shared photographic compatibility alias | `render_legacy` |

The explicit VTK `Studio`, catalogue and engineering-preview functions remain
available. Making a photographic default does not silently turn a preview
function into an expensive final-film renderer. `lab` section views clip and cap
the actual meshes before tracing. The older `cybrgeo --section` shortcut still
requires an explicit PBR preview or a supplied pre-sectioned mesh scene.

## Appearance and temporal consistency

- Exact dielectric Fresnel from authored IOR and height-correlated Smith
  masking for the anisotropic GGX base and clearcoat lobes.
- Analytic CAD surface normals by default. Real sharp edges remain sharp.
- Authored machining, brushing and other microfinish stays attached to each
  part; grazing-angle and orthographic footprints limit unresolved sparkle.
- Thin-lens depth of field. Standard views default to f/11; scene-specific
  apertures and focus distances remain explicit choices.
- Product softboxes and the ground plane are fixed in model coordinates.
  Camera motion and removed parts no longer drag the studio or floor around.
- A hue-preserving exponential highlight shoulder followed by the standard
  sRGB transfer. `View(tone_mapping='aces')` retains the older operator.
- Still images and film frames share two normal/depth/part/variance-guided
  linear-light filtering passes and verified atomic PNG encoding.
- Fixed-camera film shots may reuse one preceding native radiance estimate
  after geometric reprojection. Current radiance retains at least 80% weight;
  changed visibility, part identity, normals and highlights are rejected.
  Orbiting cameras skip this reuse. Shutter supersampling poses the actual
  geometry at each sample; it does not invent intermediate frames.

Metal Fresnel remains an RGB Schlick approximation, and clearcoat remains an
opaque layered approximation. This is not spectral glass transmission or a
measurement of the real product's optical properties. GLB files carry standard
metal/roughness preview materials; native procedural finishes are not baked
into glTF textures.

## Installation and native compilation

Package dependency ranges include both the earlier repository pins and the versions used for this delivery. `requirements-orbit-v3-tested.txt` records the exact tested environment; the older `requirements-tested.txt` retains the previous repository environment.

The wheel includes `mechanism_lab/native/{photoreal.cpp,pathtrace.cpp,photo_bsdf.h}`.
The renderer compiles on first use with a C++17/OpenMP compiler. A hash of all
three sources and compile flags keys its cache, so source upgrades cannot
accidentally reuse the old executable. `CYBR_GEO_NATIVE_CACHE` can choose the
cache location. Linux/WSL is the supported documented platform.

An optional `CYBR_GEO_EMBREE_ROOT=/path/to/embree` enables an installed Embree 4
triangle accelerator when its `include/embree4` and `lib` directories exist.
Without it, the native BVH remains the default fallback. The optional adapter
changes triangle intersection acceleration, not CAD geometry, materials or
light transport. Embree binaries are not bundled with this source/package.

## Evidence and reproduction

`tests/test_shared_photo_pipeline.py` checks API adaptation, preserved materials,
studio stability, defaults, the color transfer and complete PNG round trips.
`tests/photo_bsdf_check.cpp` checks reciprocity, dielectric reference values,
anisotropy and 72 white-furnace transport cases. `tests/embree_intersection_check.cpp`
compares closest-hit and shadow results against the native BVH using 25,000
deterministic rays. Delivery reports contain the actual execution results.

The microfacet conventions follow [PBRT's rough-surface treatment](https://pbr-book.org/4ed/Reflection_Models/Roughness_Using_Microfacet_Theory)
and [dielectric Fresnel treatment](https://pbr-book.org/4ed/Reflection_Models/Specular_Reflection_and_Transmission).
