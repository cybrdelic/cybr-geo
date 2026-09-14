# Motor-to-differential compatibility study

## Conclusion

The ODrive M8325s is a **candidate for a custom adapted drive**, not a verified direct-fit motor for this differential. The differential is a reconstructed concept with no validated torque, speed, bearing, spline, thermal or fatigue rating. Therefore no motor can honestly be certified compatible from the current files alone. The original concept poster's numerical ratings are not engineering evidence.

The new geometry demonstrates a coherent nominal layout: the motor's documented rotating face drives a pulley; a belt drives an annular pulley on the differential carrier; neither output hub is used as the input. This establishes an inspectable layout and prescribed kinematic relationship, not load-carrying capability.

## Interfaces actually modeled

The motor's four M3 face fasteners are located on the documented 31 mm bolt circle. The custom adapter has four nominal 3.4 mm clearance bores and a 3.2 mm locating bore over the 3 mm pin. Its nominal axial contact plane coincides with the motor front face at X = 63 mm after placement. Torque is intended to pass through the bolted face, not the pin. The fixed motor support uses the documented M4/40 mm mounting circle. These are dimensional anchors, not validated clamp-load or thread-engagement calculations. [Manufacturer source](https://shop.odriverobotics.com/products/m8325s).

The retained differential rear carrier flange's nominal hole centers are on a 99.6 mm bolt circle with eight locations at an initial 22.5 degree phase. The custom carrier adapter uses those locations, eight 9 mm nominal clearance bores and a 60 mm through-opening. This opening leaves the modeled output hub clear. The adapter is a new design based on the retained model, not a commercial adapter whose interchangeability has been tested.

The 32-tooth driver and 72-tooth driven pulley use a nominal 5 mm pitch, a 2.25:1 reduction and a 120-tooth/600 mm nominal pitch-length belt loop. Center distance is solved from the exact open-belt tangent/arc geometry rather than guessed visually. Its value is recorded in `validation/compatibility.json`. The tooth outlines and belt teeth are visualization approximations, not qualified HTD/GT or another manufacturer's tooth profile. Belt stiffness, preload, tension adjustment, tooth contact and tooth jumping are not modeled.

Fixture bearings are **unselected bearing envelopes**. Their dimensions are not procurement recommendations or approved fits. The static support blocks/base exist to show the layout. They are not released machining drawings. Guards, emergency-stop behavior, overspeed protection and a completed electrical drive are not included.

## Illustrative torque estimates, not allowable loads

Using the manufacturer's Kt = 0.083 Nm/A and its phase-amplitude current definitions gives the following simple estimates. The manufacturer's performance figures are approximate and cooling dependent. [Motor characteristics and unit definitions](https://docs.odriverobotics.com/v/latest/hardware/odrive-motors.html).

| Source current condition | Kt × current, motor Nm | Ideal ×2.25 carrier Nm, before losses |
|---|---:|---:|
| 40 A, free-air continuous condition | 3.32 | 7.47 |
| 60 A, forced-air continuous condition | 4.98 | 11.205 |
| 80 A, three-second peak condition | 6.64 | 14.94 |

These are arithmetic estimates from source constants, **not approved operating setpoints**, actual simulated torques, differential capacity, output traction, or proof that M3 fasteners/bearings/pulleys can carry them. They do not incorporate controller voltage/current limits, speed-dependent voltage saturation, cooling, copper/iron/mechanical losses, belt efficiency or transient loads.

For a 32-tooth/5 mm pitch driver, the pitch radius is approximately 25.465 mm. Dividing the motor torque by that radius gives the belt tight-side minus slack-side tension difference: approximately 130.4, 195.6 and 260.8 N at those three idealized points. **This difference is not the total radial bearing load**; preload and the sum/directions of belt tensions matter. No allowable radial load has been established for the motor or reconstructed carrier supports.

## What the motion actually proves

The illustrated motor rotates at 18 rpm and the carrier at 8 rpm. Prescribed outputs rotate at 10.4 and 5.6 rpm, satisfying `left + right = 2 * carrier`. Pinion rotations are tied to the preserved 48/18 and 18/18 carrier-frame constraints. Belt tooth blocks advect along the tangent/arc pitch path, and motor rotor/driver transforms share the same axis.

The animation does not derive speeds from applied voltage, current, torque, tire traction, clutch friction, tooth impulses or slip. It is not an electromagnetic simulation and does not demonstrate torque bias. The existing v3 sampled gear-profile checks are retained, but they do not establish full-assembly contact or fit freedom under load.

## Required engineering before hardware

Establish intended output torque, speed, duty cycle, cooling and inertia; recover or redesign the differential's actual interfaces and gears; qualify bearings/journals and radial loads; choose a real pulley/belt system and tensioning scheme; specify motor/adapter fasteners and thread engagement; verify output clearances, fits, backlash, shaft retention and lubrication; complete electrical/controller/encoder/thermal design; analyze stresses, fatigue and balancing; provide rotating-part guards and controlled commissioning procedures.

The released geometry and whiteprints deliberately retain the “concept / not released for fabrication” status until those items are actually resolved.
