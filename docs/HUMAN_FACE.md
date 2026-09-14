# CYBR GEO / original procedural human face

The default portrait is now **original procedural anatomy**. It does not read a
scan, an imported head mesh, a photograph, a skin texture library, a statistical
face model, or learned weights. No generative image model is used. The previous
scan demonstration is retained only under explicitly named reference commands.

![Original procedural portrait](../media/human_face/portrait.jpg)

![Profile of the same generated assembly](../media/human_face/profile.jpg)

![Clay geometry view](../media/human_face/clay.jpg)

## Geometry

`examples/procedural_human_face.py` creates the complete model through the
CYBR GEO `Assembly`, `Part`, `Material` and `View` interfaces. The default
`examples/human_face.py` entry point now delegates to this original recipe.

- Smooth authored skull, jaw, neck and shoulder cross-sections.
- Continuous malar, orbital, nasal, philtral, vermilion and chin surface fields.
- Genuine nasal apertures with recessed vestibule surfaces and an oral recess.
- Eyelid regions in the continuous head mesh, with shared boundary vertices
  and normals and adjustable geometric closure. The default is the closed-eye pose of the first portrait.
- Independent sclera, recessed iris and pupil surfaces, ocular clear envelope,
  lacrimal caruncles, and moist margins. These remain under the closed lids.
- Pinna shells with rolled helix, concha, antihelix, tragus and lobule relief.
- Individually modeled tapered brow hairs, lashes and shaved facial hair.

The final assembly contains **2,448,309 triangles across 23 parts**. Its head
grid has 800 rows and 1,201 meridian samples; additional geometry represents
the anatomical parts and hair. Exact per-part counts are recorded
in the delivered render evidence. The code exposes eye spacing, eye height,
nose projection, mouth width, skull/jaw width, brow weight, eyelid closure,
relief and a deterministic seed. These are artistic controls rather than
measurements fitted to a person.

## Materials and rendering

The 4096-pixel skin maps come from seeded multiscale pigment fields, pore pits,
small pigment clusters, oil/roughness variation and lip microfolds. Iris fibers
and scleral vessel fields are also generated numerically. No source image is
read to generate any of these maps. The shader's pore-height range is 0.032 mm;
this is an authored material scale, not a measured skin calibration.

The studio is entirely modeled: three area lights, a backdrop and an analytic
constant environment. It has no environment photograph. Rendering uses the
approved ORBIT v9 architecture: Mitsuba LLVM CPU path tracing, physical thin
lens, Gaussian reconstruction filter (0.42), linear beauty/albedo/normal guides,
Intel OIDN, and the unchanged original v9 ACES approximation and sRGB transfer.
The normal guide uses **world-space geometric normals**. The installed bump-map
shader exposes its shading-normal attribute in a local frame; mixing that
attribute with world-space normals from other materials was corrected during QA.
The render recipe retains actual geometric relief at dense tessellation.

| View | Resolution | Samples/pixel | Max depth | Lens |
| --- | --- | --- | --- | --- |
| Portrait | 1440 × 1800 | 512 | 14 | 85 mm, f/16 |
| Profile | 1200 × 1500 | 384 | 14 | 85 mm, f/16 |
| Clay | 960 × 1200 | 128 | 14 | 85 mm, f/16 |

Autofocus intersects the visible surface. The renderer rejects missed anatomy,
nonfinite radiance and blank frames. Receipts record sampling, exposure, camera,
render time, geometry, procedural inputs, texture hashes and pixel hashes.
The downloadable package verifies its images against those receipts and includes
raw comparisons, linear float EXRs, and the actual albedo/normal denoiser guides. The GLB contains the real assembly in
metres/Y-up with embedded color maps. Its real-time appearance depends on the
viewer; the dedicated renderer reproduces the pore-height and ocular shaders.

## Reproduce

Check out `feat/procedural-human-face-v9` and install the standard CYBR GEO
requirements plus Mitsuba 3.9.1 and Intel Open Image Denoise. No asset downloader
is needed for this recipe.

```bash
export OIDN_BIN=/absolute/path/to/oidnDenoise

python tools/render_human_face.py --out build/procedural_face_final \
  --view portrait --size 1440x1800 --spp 512 --depth 14 \
  --quality final --texture-size 4096 --exposure 1.4

python tools/render_human_face.py --out build/procedural_face_final \
  --view profile --size 1200x1500 --spp 384 --depth 14 \
  --quality final --reuse-textures --skip-export --exposure 1.4

python tools/render_human_face.py --out build/procedural_face_final \
  --view portrait --clay --size 960x1200 --spp 128 --depth 14 \
  --quality final --reuse-textures --skip-export --exposure 1.4

PYTHONPATH=src python -m pytest -q tests/test_procedural_human_face.py
python tools/package_human_face.py --out build/procedural_face_package
```

Use `--eyelid-closure 0` for open eyes, or a value between zero and one for a
partial blink. For other proportions, pass a `FaceParameters` instance to the
recipe's `build` function.

## Validation and limits

Five targeted tests pass. They rebuild the anatomy with file access disabled and compare
its generated arrays exactly for determinism. They check finite, nondegenerate
geometry and noncollapsed ear UV charts. The eyelid boundaries share exactly matching positions and normals with the
head. Actual Mitsuba ray intersections verify
that closed lids cover the eyes, open lids expose the ocular surface, and nostril
rays reach a recessed interior rather than a painted dark spot.

This is an original artistic head, **not a measured or medically validated
anatomical model**. Its form remains a hand-authored approximation. Skin uses a
surface BSDF approximation rather than multilayer tissue transport. Hidden
surfaces intersect, and the model is a rendering assembly rather than a
watertight printing or finite-element mesh. Matching the v9 pipeline and sample
settings does not by itself establish photographic realism.

The original code follows the repository's GPL-2.0 license. The separate legacy
scan reference keeps its own attribution and is not an input to this model.
