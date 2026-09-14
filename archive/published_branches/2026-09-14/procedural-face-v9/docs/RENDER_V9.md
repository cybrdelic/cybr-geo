# CYBR GEO photographic contract / V9

V9 is the repository-wide default photographic presentation contract. It is not
a second renderer and does not duplicate the transport code. Both public CYBR
GEO APIs route ordinary still/video work through the single native
`mechanism_lab.photoreal` implementation, so renderer fixes apply everywhere.

## Default command budgets

| Path | Renderer | Resolution | Samples | Bounce limit |
|---|---|---:|---:|---:|
| `lab render` | native photographic | 1600×1100 | 512 spp | 14 |
| `lab video` | native photographic | 1280×720 | 128 spp/frame | 12 |
| `lab film` | native photographic | 1920×1080 | 144 spp/frame | 12 |
| `cybrgeo render` | shared native photographic | 1600×1100 | 512 spp | shared still path |
| `cybrgeo video` | shared native photographic | 1280×720 | 128 spp/frame | shared film path |

These are generic production defaults, not a claim that every shot needs the
same budget. The fast VTK PBR renderer remains an explicit opt-in preview.
Compatibility `pathtrace` names resolve to the same photographic implementation
rather than silently selecting an older presentation path.

## Approved V9 reference settings

The V9 reference is the ORBIT photographic proof that established the look now
being made generic. Delivered stills were 1920×1440 at 192 spp, a 12-bounce
limit and **three** normal/depth/part/variance-guided spatial finishing passes.
The hero was rendered at 2304×1728 before Lanczos delivery to 1920×1440. The
reference films were 960×720, 48 spp/frame, depth 10, 24 fps and four seconds,
with conservative geometry-reprojected adjacent-frame radiance reuse followed
by the same three spatial passes. Those shot-specific budgets are recorded in
`mechanism_lab.render_profiles.V9`; the three-pass finishing behavior is now
used by the shared still and film APIs by default.

## What V9 means visually

V9 keeps the high-fidelity stack together as one contract: physical
focal-length/sensor camera rays and thin-lens depth of field; analytic CAD
surface normals and real hard-edge boundaries; authored IOR, clearcoat,
anisotropic GGX and explicit part-local microfinish; visible-normal sampling;
dielectric entry/exit attenuation; a larger four-softbox product studio;
power-weighted area-light sampling with consistent MIS PDFs; randomized
low-discrepancy pixel sampling; band-limited procedural finish to suppress
unresolved sparkle; a subdued indirect environment; hue-preserving neutral
highlight compression followed by sRGB; three normal/depth/material/variance-
guided linear-light finishing passes; and conservative geometry-reprojected
radiance reuse for eligible fixed-camera film frames. Shutter blur remains real
geometry-time supersampling, not optical-flow interpolation.

The studio rig scales with the model and each softbox is raised as needed until
its complete area remains above the matte floor. This avoids the floor/light
intersections and implausible hot strips that can otherwise appear on large
assemblies while preserving the newer shared renderer's BSDF, materials, color
and temporal pipeline.

Truth/provenance gates remain upstream of polished rendering. A V9 render is not
evidence that concept geometry is a measured product, that a mechanism is load
qualified, or that material parameters were measured unless the recipe says so.

`mechanism_lab.render_profiles.V9` names this contract and
`tests/test_v9_render_profile.py` prevents either public command surface from
silently falling back to PBR or drifting away from the documented defaults.

## Useful-machine validation target

`examples/aeris/` is the first large integration target carried forward on top
of the V9 default branch. AERIS is a serviceable benchtop extraction-turbine
concept with 215 named components, analytic CAD for 202 components, real
cutaway/export geometry, prescribed rotor motion, drawings and verification.
It is deliberately more demanding than a decorative part: large scene bounds,
internal reflective geometry, copper routing, a filter cartridge, impeller,
motor internals and topology structures exercise framing, studio scale,
materials and detail retention together.

AERIS remains a geometry/mechanism concept. No airflow, filtration, capture,
structural, electrical, thermal or safety performance is claimed until those
analyses and physical tests exist.
