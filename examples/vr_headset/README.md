# CYBR VISOR M1

An original **phone-powered, 3DoF VR headset prototype** for the bare Samsung
Galaxy S25 Ultra. This delivery includes a CYBR GEO analytic assembly,
printable parts, an adjustable optical prescription, actual V9 renders and a
working browser stereo scene. It is not a physically built or qualified headset.

## Outputs

The dedicated workflow publishes successful outputs to the task branch under
[media/vr_headset](../../media/vr_headset) and executed evidence under
[docs/validation/vr_headset](../../docs/validation/vr_headset).
Until those files exist, the source is not evidence of a successful run.

- Hero, eye-side, exploded and optical-detail V9 PNGs.
- Four-second IPD motion MP4, with 96 independently traced frames.
- STEP assembly, printable STL parts and fit coupon in the CAD ZIP.
- Named GLB assembly and animated IPD GLB.
- Browser stereo screenshot, pose-change screenshot and execution report.

## Architecture and actual functionality

The phone supplies display, compute, battery and orientation sensors. Two
separate eye origins render a stereoscopic 3D scene. The browser maps each
display pixel to an outgoing eye ray using the specified lens's two spherical
surfaces, Snell refraction and aperture rejection. It does not simply duplicate
one image into two screen halves. Changing IPD changes both eye spacing and
the physical display locations beneath the lenses.

The shell has a continuous opaque tunnel, central binocular divider, removable
phone cradle, ventilation slots, accessible cover screws, independently sliding
lens barrels, clamp screws, lens retainers, rim pads, focus spacers, face
cushion and 25 mm head/crown straps. Lens travel is 58–72 mm. Focus is adjusted
with four matching replacement plate spacers, not an invented motor.

The native CYBR GEO V9 material path now supports explicit Fresnel dielectric
transmission. Existing opaque materials retain their previous behavior. The
rendered lenses are closed refracting CAD surfaces; their IOR is authored as
1.49. V9 here uses RGB light transport, not spectral dispersion.

## Reproduce

Install the repository dependencies and EGL/FFmpeg/font packages as in the
workflow, then:

~~~bash
python -m pytest -q tests/test_vr_headset.py
python examples/vr_headset/build_assets.py
python examples/vr_headset/render_assets.py --views hero eyes exploded
python examples/vr_headset/render_assets.py --views optics --video
python -m pip install playwright
python -m playwright install chromium
python examples/vr_headset/verify_viewer.py
~~~

The normal lab API also accepts the recipe:

~~~bash
lab build examples/vr_headset/recipe.py --step --stl
lab render examples/vr_headset/recipe.py --intent inspection
~~~

The recipe deliberately uses inspection provenance: phone camera bounds, face
cushion shape, flexible straps and fastener envelopes are assumptions. The
part ledger records them instead of upgrading them to measured geometry.

## Use the stereo application

For desktop preview:

~~~bash
python -m http.server 8000 --directory examples/vr_headset/viewer
~~~

Open http://localhost:8000. Drag to look around. Four targets have actual depth
and eye parallax. The calibration grid follows the same optical mapping.

For a USB-connected Android phone with ADB already configured:

~~~bash
adb reverse tcp:8000 tcp:8000
~~~

Open http://localhost:8000 on the phone, switch to landscape and tap Enter VR.
For network hosting, use HTTPS because orientation sensors require a secure
context. Match the application's IPD, focus offset and eye relief to the
physical setup. R or Recenter resets heading. The demo does not implement
position tracking, sensor prediction or a native low-latency compositor.
Chromium software-rendering evidence does not establish phone performance.

The native Google Cardboard SDK is an alternative integration path for head
prediction, viewer profiles and platform rendering. It has not been copied,
integrated, built or device-tested as part of this browser prototype.

## Procurement and assembly

| Item | Quantity | Selection / status |
|---|---:|---|
| Bare Galaxy S25 Ultra | 1 | Nominal 162.8 × 77.6 × 8.2 mm body, 218 g |
| Matching optical lenses | 2 | Designed 34 mm diameter, 45 mm EFL, 8.5 mm center thickness, n=1.49 |
| M3 cover screws | 4 | Nominal 25 mm shank; thread engagement must be checked on prints |
| M3 bridge screws | 4 | Match measured spacer/bridge stack |
| M3 ocular clamp screws/nuts | 4 pairs | Match approximately 10 mm clamp stack |
| 25 mm nylon webbing | As fitted | Head and crown straps with hook-and-loop adjustment |
| Foam, textile, silicone rim and body pads | As fitted | Nominal envelopes; compression and skin fit unmeasured |
| Opaque printed shell and carriers | Exported set | PA12/PETG prototype; material properties not qualified |

**The optical prescription is an original design input, not a confirmed
off-the-shelf part number.** Measure the actual lens diameter, center/edge
thickness, surface prescription and focal length; update spec.py and viewer
parameters before committing to the full print. The fit coupon is mandatory
practical preparation: the assumed print error can consume the nominal radial
clearance. Lens gaskets are compliant and require empirical thickness setting.

Assembly order:

1. Print the fit coupon; measure bore/fastener clearance and correct the recipe.
2. Obtain and measure matching lenses. Print the tunnel, bridge and carriers.
3. Install the center baffle and perimeter bridge spacers; secure the bridge.
4. Seat each lens between compliant rim pads. Insert retainer ring and flanged
   sleeve; clamp the carrier and sleeve through the real IPD slots with nuts.
5. Set both carriers to matching IPD marks and tighten the four clamp screws.
6. Attach the cradle, edge pads, face cushion and straps. The shell/cradle
   interfaces require adhesive or the installed cover-screw stack as modeled.
7. Insert the bare phone display-first along -Y to the bezel stop. The active
   screen remains inside the aperture. Fit rear pads, then the vented cover and
   four accessible screws. The cover compresses foam against the phone back.
8. Calibrate the optical grid and focus while holding the assembly by hand;
   qualify retention and comfort before relying on its straps.

Removal reverses these operations. The tested phone extraction path starts
after all four cover screws and the cover are removed. The exploded view is an
inspection layout, not a sequenced assembly simulation.

## Evidence and limits

Executed gates cover valid analytic bodies, positive lens edge thickness,
sampled IPD rigid clearances, nonintersecting rigid lens geometry, nominal phone
clearance, camera axial allowance, sampled phone extraction, Snell symmetry,
lensmaker focal power, total internal reflection, finite transforms, browser
shader compilation, Python/JavaScript ray agreement, stereo image disparity,
and image changes after a pose command.

They do **not** establish physical fit, strength, fastener torque, print
accuracy, thermal comfort, drop retention, face pressure, optical MTF, usable
eye box, chromatic correction, measured distortion, motion-to-photon latency,
or interoperability with commercial PC VR games. A functional stereo program
and valid CAD are development evidence; hardware qualification remains open.

## Sources

- Samsung publishes the phone's nominal envelope and mass in its
  [Galaxy S25 Ultra specifications](https://www.samsung.com/us/smartphones/galaxy-s25-ultra/).
  The camera bump, active rectangle and corner geometry remain explicit design
  allowances, not manufacturer CAD.
- Google's [Cardboard manufacturer guidance](https://developers.google.com/cardboard/manufacturers)
  describes mobile viewer construction and a 45 mm focal-distance starting point.
  This M1 lens is not claimed to match Google's manufacturer-kit lens.
- Google's [Android Cardboard quickstart](https://developers.google.com/cardboard/develop/c/quickstart)
  documents smartphone stereoscopy, head tracking and per-viewer distortion.
- The rendering stack follows [CYBR GEO's V9 contract](../../docs/RENDER_V9.md):
  Mitsuba 3.7.1, captured CC0 workshop HDRI, scanned bench roughness, guided OIDN,
  ACES and sRGB. Every animation frame comes from geometry.
