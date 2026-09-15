# ROAM — shared spine

This revision replaces the three-mast architecture with one closed aluminum spine. Independent guides on three faces carry the two monitors and the keyboard/mouse support. Original CAD profiles establish the design; purchased bearing, shaft and brake interfaces establish specific hardware boundaries. Catalog internals are not invented.

The deliverable is a nominal CAD and packaging study. It is **not released for fabrication or loaded use**. The geometric repair and the physical qualification have different evidence, recorded below.

## Geometry and construction

- A 140 × 100 × 3 mm spine transfers the three carriage loads through bolted shoes and a saddle into a telescoping aluminum base.
- Each arm has separate bored end fittings, closed tube links, compression sleeves, bushings, thrust washers, shoulder screws and nuts. The link tubes are 60 × 30 × 2 mm for the monitors and 60 × 40 × 2 mm for the tray.
- The root tongue seats in a separate slotted aluminum mounting block. Four M6 carriage screws use 46 mm vertical spacing; two central vertical M5 screws retain the tongue and clear the horizontal screws. A blind relief accommodates the preinstalled guide-block screw head. The former 8 mm screw spacing was rejected because it produced excessive bolt-row tension in the hand-load calculation. The block is a supplier-machined item; it is not a home-printed structural part.
- Two guide blocks, spaced 180 mm apart, carry each face carriage. A manual rail brake retains its height. Gas-assist package envelopes are reserved alongside the guides; their final configuration is open.
- The tray has separate pitch and roll clamps. Pitch clamp screws lie on a 75 mm radius behind the joint. The central pivot establishes the rotation axis; the offset clamps provide an additional friction interface.
- The base has three positive pin positions: 0, 180 and 360 mm extension. The transport preset uses the middle position. The rear caster mounting stem has a retained nut, and an open underside slot in the sliding tube clears it when retracted.
- Guide covers and edge guards are hollow, nonstructural printable parts. Flat structural profiles require accurate supplier cutting/drilling, especially bearing bores. Home assembly does not imply home-machining every precision feature.
- All moving parts use the same CAD frame definitions as the viewer. Nineteen driven frame mates are solved with CadQuery's constraint solver. These are geometric constraints, not physical contact simulation.

The two provisional devices are 610 × 360 × 38 mm and 6 kg each. VESA 75 and 100 mm patterns are present, but the actual device models, attachment depth and connectors are unknown. Screen bodies deliberately remain envelopes.

## Component evidence

| Interface | Evidence | Representation and limit |
| --- | --- | --- |
| HGR20 / HGH20CA guide | [HIWIN linear guideway catalog](https://www.hiwin.com/wp-content/uploads/Linear_Guideway-E.pdf) | Mounting outline, rail hole pitch, block spacing and mounting holes. Raceway/ball geometry omitted. |
| HK2001A + PHK20-1 | [Zimmer HG20 drawing and data](https://www.zimmer-group.com/swk-datasheet/HIW-HGQH-20-HGWHA-HK2001A-EN.pdf) | Drawing-derived envelope: 1,200 N listed axial holding force at 7 N·m; 1 mm compensation plate. Exact guide-block combination, lever sweep, rail lubrication and installation must be confirmed. A catalog force is not the station's load rating. |
| GFM-1214-17 bush | [igus dimensional catalog](https://www.igus.com/contentData/Product_Files/Download/pdf/2016%20iglide%20section.pdf) | Ø12 shaft / Ø14 housing / Ø20 flange / 17 mm overall. Nominal installed ID 12.05 mm. Housing and shaft tolerances still need production drawings. |
| Shoulder screws | [norelem shoulder screw family](https://www.norelemusa.com/medias/07534-Datasheet-27418-Shoulder-screws-similar-to-ISO-7379-enUS.pdf?context=bWFzdGVyfHJvb3R8MTg5MTc5fGFwcGxpY2F0aW9uL3BkZnxhR1l4TDJnM01DODVOVEExTVRjek16Y3lPVFU0THpBM05UTTBYMFJoZEdGemFHVmxkRjh5TnpReE9GOVRhRzkxYkdSbGNsOXpZM0psZDNOZmMybHRhV3hoY2w5MGIxOUpVMDlmTnpNM09TMHRaVzVWVXk1d1pHWXxiOTQwMWZlMTRjMTE4YjljMDYyNmNhN2NiZjgyNTljNWY2NWJkNjFlZDE2NzY5NGJlNTMyOWZlNmZiMTNjNzVl) | Ø12 × 50 mm lap-pivot and Ø12 × 30 mm opposed pitch-pivot boundaries. Simplified threads; final stock item, grade and fit require selection. |
| Button-head screws | [Bossard BN 1593 dimensional catalog](https://bossard.partcommunity.com/3d-cad-models/?info=bossard%2F01%2F01_100%2F01_100_100%2F01_100_100_10%2Fbn_1593_8699%2Fbn_1593.prj) | Conservative maximum cylindrical head and hex-drive envelopes, with matching counterbores. Nominal thread cylinders do not simulate thread contact. |
| Gas lift | [Bansbach type K family](https://www.bansbach.com/en/products/gas-springs/lockable-gas-springs/main-type-k/) | Family reference only. Current body length, force, stroke, rod adapter, lower retention and release mechanism are unselected packaging requirements, not a sourced finished actuator. |
| Casters / base pins | Unselected | Required exterior and stem/pin interfaces only; wheel trail, brake, retention, rating and supply availability are open. |

## Loads and stiffness

Run `engineering.py` to regenerate `verification/engineering.json` from the exported solids. It gives mass, center of gravity, support-edge restoring moments, guide reactions, root bolt-row tension and pitch-clamp preload requirements for every preset. Assumed densities and component mass substitutions are recorded per part. Read `RESULTS.md` for the latest summary.

The static comparison uses a 100 N downward hand load at the worst tray corner plus a 30 N horizontal force at 1,200 mm height. The caster-center rectangle is eroded by an assumed 40 mm trail allowance. Positive residual moment in this scenario is **not** a tested stability rating. Keyboard, mouse, computer, power hardware, impact, slopes, threshold crossing and caster-brake behavior are not included.

Closed-section beam estimates describe an ideal uninterrupted tube. End tongues, bolt slip, lap joints, mast/guide compliance, torsion and local stress concentrations make the whole assembly more flexible. The calculation does not establish typing stiffness.

Pitch friction estimates expose required clamp preload for assumed friction coefficients of 0.10, 0.15 and 0.25. No friction coefficient, contact pressure, bolt preload retention or holding torque has been measured. The smaller monitor pivots and tray roll clamps still require contact/holding calculations and loaded tests. The rail brake's axial catalog force does not establish guide moment capacity or controlled lowering.

Thermal behavior is open because no computer, adapter or power bundle has been selected. The passive frame is not a substitute for an equipment cooling layout.

## Cable geometry

The visible routes are actual swept CAD solids included in STEP, interference queries and browser updates. Each spine U-loop reserves a nominal 1,500 mm path with a 30 mm bend radius. The articulated spans report their changing routing lengths.

These are installation corridors, **not a flex-qualified harness**. The model does not imply that a cable stretches as a joint moves. Continuous slack management, retained clips/hangers, connector fit, abrasion, cable torsion, strain relief and repeated flex remain open. The selected device and cable data must determine the eventual fixed-length harness. No mains enclosure or custom mains wiring is specified.

## What the checked build establishes

`verification/review.json` is the result for the exact hashed inputs. Every unordered component pair is screened, and overlapping bounding regions receive exact OCCT intersection tests. Cable control-hull bounds are a conservative broad phase; surviving candidates still require exact intersections. Boolean failures fail a position.

Full coordinate ranges remain in `requirements.json`, including blocked positions. Samples are at most 50 mm apart vertically and 30 degrees angularly, with all three base pin positions. Other coordinates remain at Working for each single-coordinate sweep; coordinated presets are tested separately. Neither endpoint clearance nor these samples certifies a collision-free continuous path.

The reviewed service sequences are listed in `ASSEMBLY.md`. Deliberate displaced-bearing, missing-flange, missing-thread, omitted-coordinate and stale-export defects must be rejected. STEP roundtrip, valid connected solids, mate residuals and inspection derivatives are also checked. These are nominal geometric results, not manufacturing readiness.

## Remaining release work

1. Select actual monitors, rated casters, base pins, gas-lift configuration, end adapters and release hardware. Resolve gas-body lateral retention and cable anchors with real mounting details.
2. Verify the worst posture/load combination, tipping behavior and caster brakes, then establish load limits. Check the 16 mm end fittings, tube wall bearing, tapped engagements and mast shoe joints.
3. Qualify pitch/roll/yaw holding, preload, friction, wear and polymer creep. Provide controlled lowering and suitable retention during height adjustment.
4. Produce tolerance-controlled hole/thread drawings, fastener grades and lengths, and a purchase-ready BOM. The current CSV is a design list.
5. Build and load-test an unloaded joint/arm fixture before completing a workstation. Measure deflection and holding behavior, then validate the complete cable and equipment layout.
