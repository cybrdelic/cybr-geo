# CYBR ORBIT / Inspection wrist

**Current revision:** [Serviceable revision 3](ORBIT_SERVICE.md) has 148 analytic
components, the shared 0.5 photographic renderer and an ordered reversible
service process. The revision-2 counts, renders, commands and selected checks
below describe the preceding delivery and are retained as historical context.

ORBIT is an original bench inspection and positioning mechanism: a geared
rotary wrist carrying a centered parallel gripper. Two manual inputs operate
the wrist and the opposed lead screw. It is a reusable Mechanism Lab recipe,
not a separate renderer or a reconstruction of a commercial product.

Revision 2 has **126 named components**, **125 analytic CAD components**, and
**2,101,110 rendered triangles**. The held inspection configuration adds a
filleted aluminium coupon and a bronze sleeve, for **128 components**. The sole mesh component is the transported
spline conduit; the STEP coverage report explicitly lists that omission.
The complete GLB contains every component.

![ORBIT assembled](../media/orbit_hero.jpg)

![ORBIT inspection sample detail](../media/orbit_macro.jpg)

![ORBIT mechanism exposed](../media/orbit_internal.jpg)

![ORBIT exploded](../media/orbit_exploded.jpg)

![ORBIT parallel gripper](../media/orbit_gripper.jpg)

![ORBIT gear and bearing detail](../media/orbit_gear_detail.jpg)

## Purpose and nominal geometry

The mechanism is intended to hold small components for inspection, camera
scanning and bench positioning. The mounting shoe has four 6.6 mm clearance
holes on a 48 x 108 mm rectangular pattern. No gripping-force or payload
rating is assigned.

| Parameter | Model value |
|---|---:|
| Assembled envelope at the initial pose | 202.2 x 137 x 116 mm |
| Wrist gears | 72T / 20T, module 1 mm |
| Gear-center separation | 46 mm |
| Input/output ratio | 3.6:1 |
| Prescribed wrist range | -18 to +18 degrees |
| Opposed screw lead | 3 mm per revolution per jaw |
| Travel of each jaw | 6 mm |
| Nominal pad-face aperture | 21.8 to 33.8 mm |
| Animation cycle | 8 seconds |

The drive pinion and its clamping hub form one solid. Radial grub screws
engage actual shaft flats. The wrist shaft has a keyed gear seat, thrust
washers and a retaining ring seated in a modeled groove. The palm is a hollow
loft with a service opening. Each finger and guide carriage forms one solid;
its hexagonal pocket captures a helically cut bronze nut. Both guide rods pass
through bored supports and separate bushings. The elastomer pads occupy
recesses cut into the fingers.

## Capability coverage

| Geometry or assembly capability | Concrete use |
|---|---|
| Analytic primitives | Mounting shoe, shafts, bushings, bearing balls |
| Extrusion and union | Bearing pedestal and two-lobe housing |
| Difference and shell construction | Hollow palm, housing cavity, service windows |
| Fillets and chamfers | Shoe, fastener heads and fluted knobs |
| Bores, counterbores and arrays | Base mounting pattern and cover bolt circle |
| Revolution | Stepped hollow wrist shaft and integral flange |
| Multiple-section loft | Palm and tapered fingers |
| Involute profiles | Actual 72T and 20T gear solids |
| Helical sweeps | Opposite-handed trapezoidal screw ridges |
| Internal helical Boolean tools | Clearanced mating bronze nuts |
| Torus cuts and sphere arrays | Bearing raceways, balls and pocketed cages |
| Transported spline mesh | Conduit through the hollow wrist shaft |
| Text and repeated markings | ORBIT identification and angular ticks |
| Rigid assemblies and explosion offsets | Shared poses for render and GLB export |
| Analytic/mesh exports | STEP coverage, complete GLB and animated GLB |
| Analytic hidden-line drawings | Four-view A3 nominal structural sheet |
| Native photographic rendering | Thin-lens BVH/GGX/MIS path tracing |

This exercises the listed construction and presentation capabilities. It does
not claim that a single model tests every importer, material option or CLI
combination supported by CYBR GEO.

## Revision 2 photographic improvements

Small, analytic edge breaks now catch light on the housing, pedestal, flange,
palm and fingers. Normals come from the OpenCascade parametric surfaces; face
boundaries preserve the real hard edges. Numerically collapsed triangles at
periodic seams and sphere poles are removed without altering visible geometry.
The mounting shoe also has recessed serial lettering with separate infill.

The native photographic tracer now consumes authored IOR, clearcoat and
anisotropy. Brushing/turning axes and origins follow each rigid part. The
anisotropic GGX distribution uses visible-normal sampling, and the diffuse
term accounts for dielectric entry/exit attenuation. The clearcoat is an
opaque Schlick-attenuated approximation, not a spectral multilayer solver.
Equations are documented in [PBRT 4e, Roughness Using Microfacet Theory](https://www.pbr-book.org/4ed/Reflection_Models/Roughness_Using_Microfacet_Theory).

A larger four-softbox product studio supplies controlled reflections. Area
lights use power-weighted sampling with consistent MIS PDFs. Randomized
Hammersley pixel samples improve edge coverage; the procedural finish is
band-limited to reduce unresolved sparkle. The macro lens focuses on the
sample face, with real thin-lens depth of field behind it.

The sample is 21.8 mm wide and touches both closed jaw pads at the nominal
pose. Its sleeve has 0.02 mm radial clearance in the coupon. The new in-use
film holds the jaws closed while the wrist rotates; it does not simulate
contact forces. The original unloaded jaw-opening motion remains in the
animated GLB.

## Reproduce revision 2

From a CYBR GEO checkout with its documented dependencies installed:

```bash
PYTHONPATH=src python tools/orbit_studio.py build --out outputs/orbit_studio --interfaces --drawing
PYTHONPATH=src python tools/orbit_studio.py stills --cache outputs/orbit_studio/inspection-cache --held --out outputs/orbit_studio --work outputs/orbit_studio/still-work --views hero --size 2304x1728 --deliver-size 1920x1440 --spp 192 --passes 3
PYTHONPATH=src python tools/orbit_studio.py stills --cache outputs/orbit_studio/inspection-cache --held --out outputs/orbit_studio --work outputs/orbit_studio/still-work --views macro --size 1920x1440 --spp 192 --passes 3
PYTHONPATH=src python tools/orbit_studio.py stills --cache outputs/orbit_studio/cache --out outputs/orbit_studio --work outputs/orbit_studio/still-work --views internal exploded gripper gear_detail --size 1920x1440 --spp 192 --passes 3
PYTHONPATH=src python tools/orbit_studio.py film --cache outputs/orbit_studio/inspection-cache --held --out outputs/orbit_studio --work outputs/orbit_studio/film-work --clips motion --size 960x720 --spp 48 --depth 10 --passes 3 --temporal --duration 4
PYTHONPATH=src python tools/orbit_studio.py film --cache outputs/orbit_studio/cache --out outputs/orbit_studio --work outputs/orbit_studio/film-work --clips exploded --size 960x720 --spp 48 --depth 10 --passes 3 --temporal --duration 4
```

The six stills use a 12-bounce limit and three normal/depth/part/variance-guided
spatial filtering passes. The hero is rendered at 2304 x 1728 and reduced to
1920 x 1440. The other stills are rendered at their delivered 1920 x 1440 size.
Unfiltered native radiance and primary-hit guides remain in the working
checkpoints for comparison.

Both films contain 96 frames at 24 fps. Every distinct pose is path traced.
Both return sweeps reuse the exact same poses in reverse. With the sample
held, the wrist cycle starts at -18 degrees, reaches +18 degrees halfway, and
returns; every reused part transform is checked against its target pose. Film noise
reduction conservatively reprojects adjacent radiance estimates using actual
per-part transforms and world-space hit points, rejecting changed visibility,
different parts, normal discontinuities and changing highlights. The current
frame retains at least two thirds of the radiance weight. Three spatial
passes follow. This is radiance filtering, not optical-flow frame generation;
no motion blur is claimed. Checkpoint signatures and frame hashes are recorded.

The rendering code is the repository's `mechanism_lab.photoreal` module and
`native/photoreal.cpp`. All scene objects come from the actual recipe geometry.
The legacy six-column material export remains available to the legacy tracer.
GLBs preserve standard metal/roughness preview materials; the rich native
procedural finish is not encoded as a glTF texture.

## Execution evidence

The delivery includes machine-readable reports for:

- finite geometry, valid analytic solids and watertight triangle meshes for
  all 126 unloaded components;
- 39 actual BRep intersection measurements over twelve selected interfaces
  and four prescribed poses, with no measured solid overlap above 0.0001 mm3;
- standard animated GLBs for both unloaded jaw motion and sample-held wrist motion;
- STEP coverage and per-part names/materials/roles;
- each still's resolution, sample count, camera settings, renderer and render
  time, plus its provenance report;
- native film frame hashes, model times, explosion offsets, sampling/filter
  settings, and decoded H.264 dimensions/frame counts.

The selected interface audit covers the gear pair, both screw/nut pairs,
finger/palm clearance, guide rods, pad recesses, two bearing raceway contacts,
shaft/gear clearance and housing/gear clearance. It is not an exhaustive
global collision or swept-volume certificate.

Nineteen focused Python checks cover surface normals, camera/core contracts,
part-following material export and actual geometry reprojection. A separate
native numerical check compares the isotropic lobe with the original and
integrates 72 white-furnace cases; the largest measured reflected energy is
0.8981 for the tested 0.85-albedo materials. The earlier gear-profile and
drawing-origin repairs remain in the source patch.

## Toolkit fixes exposed by this model

The former involute root transitions crossed adjacent tooth sectors for a
72-tooth gear. `cybrgeo.features.involute_profile` now starts those transitions
at the actual involute radius and keeps them inside the tooth sector. Pitch
circle flank construction remains involute; root transitions remain original
visualization geometry, not cutter-envelope certification.

The end-view drawing dimension previously used model Z=0 and an additional
half-height offset. Raised mechanisms could put that dimension inside the
title block. It now anchors to the projected lower geometry edge.

## Limits

This is an original nominal design with prescribed rigid kinematics. No
contact forces, gripping load, gear torque, friction, compliance, tolerances,
fatigue or actuator performance have been qualified. Bearing ball/cage rolling
is not simulated. Threads use the explicitly modeled original trapezoidal
profile, not a certified standard designation. The routed service connector
does not imply a designed electrical circuit. Opaque material rendering does
not establish measured optical properties.

The GPL-2.0 repository license applies to the recipe and toolkit changes.
