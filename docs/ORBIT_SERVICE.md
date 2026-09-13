# ORBIT / serviceable revision 3

ORBIT is a manual inspection wrist and parallel gripper. Revision 3 has 148
analytic CAD components and 3,188,510 rendered triangles. The sample-held
photographs add a coupon and its sleeve. Every mechanism component, including
the spline-swept conduit, is included in the analytic STEP export.

The mechanism retains its 72T/20T module-1 gears, 3.6:1 ratio, 46 mm gear-center
distance, opposed 3 mm lead screw and 6 mm travel per jaw. The nominal jaw gap
is 21.8–33.8 mm. It is an original concept with no assigned payload or torque
rating.

![ORBIT revision 3](../media/orbit_v3_hero.jpg)

## Changes required for actual removal paths

| Former obstruction or missing constraint | Revision 3 geometry/procedure |
|---|---|
| One-piece cover trapped behind the integral flange | Two radially removable cover halves |
| Rigid retaining ring could not leave its groove | Two-piece rear clamp collar and accessible screws |
| Gear could drift axially | Front and rear spacers between existing constraints |
| Palm screws inaccessible behind the gripper | Screws moved above/below the rails; full axial clearance before lifting |
| Lead screw lacked an axial stop | Collar, knob shoulder and two thrust washers |
| Hex nuts could slide through their finger pockets | Slotted retaining plates on both ends, removed radially |
| Guide supports contained phantom Boolean plugs | Separate flush landing faces and drilled rod locks |
| Pedestal screws buried under solid ribs | Long coaxial driver-access bores |
| Pinion lock inaccessible inside the shell | Dedicated service-access bore |
| Cover screws/scale ticks interfered during extraction | Correct screw reach and trimmed marking clearances |
| A sideways cable explosion crossed the shaft wall | Withdraw the palm and dressed harness together through the bore |
| Bearing balls crossed their races in the explosion | Complete bearing cartridges remain service units |

## Shop order

Unload the sample, open the jaws fully, and return the wrist to neutral.
Support every released subassembly by hand or a suitable fixture before
removing its last retainer. The modeled bench is at Z = -2 mm.

1. Release the four flange screws and withdraw the complete gripper, feeding
   its dressed harness axially through the hollow shaft.
2. Release the input-shaft locks and collar, then withdraw the input shaft.
3. Remove the six cover screws, separate the cover halves radially, and slide
   the housing over the free flange.
4. Release the split rear collar and rear washer. Withdraw spindle, gear,
   front bearing and spacers as one supported unit.
5. Remove the remaining pinion, rear bearing and bushing. The now-accessible
   pedestal screws release the empty pedestal.
6. On the bench, remove the rear spacer, slide the gear over the key and off
   the shaft, lift the key, and remove the remaining spacer/bearing/washer.
7. Release gripper axial stops and rod locks. Withdraw the guide rods, then
   remove the end supports.
8. Remove the slotted nut plates. Slide the bush-lined fingers off their
   captive nuts. Unthread both handed nuts at their actual 3 mm lead before
   taking out the lead screw.

The declarative procedure contains 331 individual operations, including each
fastener, clearance withdrawal, supported lift, bench transfer and placement.
Assembly follows these same nominal paths in reverse. Bearings, bonded pads,
press-fitted bushings and the dressed harness remain intact service units.
Reversing the animation does not establish a tightening sequence, torque value
or bearing-preload specification.

## What the audit establishes

The static audit uses actual CAD bounds and exact solid intersections. The
service audit tests every moving analytic component against every remaining
component after an AABB broad phase, with at most 1 mm translational parameter
spacing and 15 degrees rotational parameter spacing. Exact helical nut/screw
solids are included. Separate approach envelopes test the specified driver or
short-leg hex key against remaining parts. Removed parts stay collision
obstacles after they are placed on the bench.

The intersection threshold is 0.0001 mm³. Bench placement allows 0.01 mm above
the plane for the triangle/BRep bound difference. Check reports explicitly
record all tested operations, collisions, floor crossings, obstructed tool
approaches and any geometry that could not be tested.

This is sampled nominal rigid-CAD verification, not a continuous swept-volume
certificate. Smooth nominal fastener thread envelopes do not resolve bolt
flanks. Driver approach does not qualify every wrench swing. Hands, gravity,
elastic fits, preload, friction and contact force are not simulated. Production
use still requires tolerances, selected hardware, torque specifications and
physical assembly trials.

## Reproduce

From a checkout with the documented system dependencies. For the exact CAD kernel and Python versions used by this delivery, install `requirements-orbit-v3-tested.txt` (CadQuery 2.7 / OpenCascade 7.8.1 / VTK 9.3.1). Other supported package versions can produce different tessellation counts:

```bash
python tools/orbit_service.py build --out outputs/orbit_service
python tools/orbit_service.py validate --out outputs/orbit_service --jobs 6 --step 1 --angle 15
python tools/orbit_service.py export --out outputs/orbit_service
python tools/orbit_service.py stills --out outputs/orbit_service --size 1920x1440 --spp 512
python tools/orbit_service.py films --out outputs/orbit_service --size 1280x720 --spp 96 --seconds 1 --fps 24
```

The stills use the shared photographic renderer at 512 spp / 14 bounces.
The two six-second, 24 fps films show six selected removal/installation moves;
editorial cuts omit intervening screw removal and bench transfers. Every
distinct pictured pose is native path traced at 96 spp / 12 bounces. The
reverse film reuses precisely the same geometry poses. No optical-flow frame
interpolation or motion blur is claimed for these particular service clips.

`ORBIT_service_animation.glb` contains the complete 128.7-second time-compressed
service animation with every operation boundary and additional helical
keyframes. The time is an inspection presentation, not measured workshop
labor. `ORBIT_service_procedure.json` is the complete readable/editable plan.

Ordinary ORBIT `explode` parameters also follow this ordered process now.
The `exploded` view frames the complete parked service inventory. A held
sample configuration rejects disassembly until the sample is removed.

See [shared renderer changes](SHARED_PHOTOGRAPHY.md) and
[earlier capability coverage](ORBIT_SHOWCASE.md). The GPL-2.0 project license
applies to the original model and toolkit code.
