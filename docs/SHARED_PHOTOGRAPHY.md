# Rendering with CYBR LIGHT

All photographic entry points use the bundled CYBR LIGHT native CPU engine. Install the documented Python dependencies and g++/OpenMP once; the first render builds a source-hashed executable. There is no network step during rendering. The pinned upstream revision and local kernel changes are recorded in [UPSTREAM.json](../src/cybr_light/UPSTREAM.json).

| Command | Default resolution | Spectral packets per pixel | Depth |
| --- | --- | --- | --- |
| `lab preview` | 640 × 480 | 32 | 8 |
| `lab render` | 1100 × 825 | 96 | 14 |
| `lab video` | 1280 × 720 | 64 per frame | 12 |
| `lab film` | 1920 × 1080 | 128 per frame | 14 |

The default is eight wavelengths per packet. `lab render --bands 12` increases spectral work. Packet count is different from RGB sample count. Increase `--spp` for final images; the preset alone does not establish convergence. The native maximum is eight million pixels and 128 wavelengths per packet.

## Geometry and optics

The adapter converts millimetres to metres, applies each part's actual rigid pose, preserves its vertex normals and material/component IDs, and honors hidden groups, capped sections, perspective/orthographic framing and thin-lens settings. Lights and floor remain fixed in assembly coordinates during a camera orbit. Every film frame is freshly traced; shutter subframes average linear radiance from actual geometry poses.

CLM1 meshlets contain at most **64 full-attribute vertices and 124 indexed triangles**. Deduplication includes normals, UVs and tint so hard edges and texture seams survive. Native triangles retain indices into shared float32 vertex blocks during BVH construction and intersection; intersection math remains double precision. Material and component IDs stay per triangle. Header/count/index/length checks reject malformed input before tracing.

The CPU renderer uses a SAH BVH. Meshlets reduce input and vertex storage; they do not imply a GPU mesh-shader or hardware ray-tracing backend. Both analytic native primitives and indexed meshlet triangles are supported.

## Outputs

Each still writes `.png`, `_unfiltered.png`, `.pfm`, `.exr`, `.cys`, `.clm`, `.json`, `.truth.json` and diagnostic PFM files for normal, albedo, depth, position, component ID and standard error. Scene asset paths are relative so the scene and its meshlet file can move together. Raw films remain untouched. The PNG uses three non-neural camera-footprint/geometry/object/variance guided filtering passes followed by an ACES display transform; render metadata identifies that processing.

The source/executable cache is under the system temporary directory; `CYBR_LIGHT_CACHE` relocates it. Compile/render failures point to retained logs. Preview success receipts are written only after image decoding and GLB readback succeed. FFmpeg is needed for MP4/GIF encoding.

## Material mapping and alternatives

GEO materials use RGB authoring controls, not measured spectral reflectance. The current adapter maps opaque surfaces to plastic or metal, derives conductor controls from authored reflectance, and maps transparent concepts to absorbing glass. Anisotropy is retained; microfinish and coat layers do not yet have an exact spectral layered-BSDF mapping. Review raw/filtered output for thin wires, shallow depth of field and specular features.

`--renderer photoreal` keeps the previous in-house RGB product renderer available. `--renderer pbr` uses VTK/EGL for raster inspection. Neither is an external rendering dependency. Historical gallery image provenance remains in its original records; those images are not relabeled as new CYBR LIGHT output.

## Verification

`tests/test_light_rendering.py` checks real still/video execution, meshlet attribute/ID preservation, malformed input rejection and pose/unit conversion. The bundled native numerical suites test intersection, BVH/brute-force agreement, optics, sampling, media, checkpointing and filters. `make test-core` includes these integration checks alongside the CAD and export contracts.
