# AERIS / useful-machine geometry showcase

AERIS is an original serviceable benchtop extraction-turbine concept built as a
large CYBR GEO integration target. It is intended to exercise real mechanical
geometry, assembly, cutaway, export, drawing, animation, photographic rendering
and engineering-screening workflows together. It is **not** a rated fume
extractor or a respiratory-safety device.

The 215 named components include a pocketed base and connected tilt yoke; an
asymmetric hollow collection hood; removable filter cartridge; one-solid spiral
volute with an open outlet; eleven twisted lofted impeller blades; continuous
shaft and two bearing assemblies; an original finned motor with stator teeth,
magnet sectors and twelve routed copper conductors; terminal enclosure; spline
harness with helical strain relief; and an outlet cartridge containing analytic
BCC support geometry plus an implicit gyroid sheet.

202 components have analytic CAD. The gyroid and twelve copper conductor routes
are intentionally mesh-only and are explicitly omitted from STEP coverage rather
than replaced with fake B-reps. The full GLB retains all named components.

## Engineering revision AERIS-E1

The original concept used an ad-hoc bearing envelope whose modeled bore left a
40 micrometre diametral clearance on the nominal 10 mm rotating shaft. That is
not acceptable evidence for a production rotating-inner-ring fit. AERIS-E1 keeps
the same stable assembly naming and architecture but replaces those envelopes and
carrier seats with nominal 6000-series geometry: 10 mm bore, 26 mm OD, 8 mm
width. This removes the hard nominal-geometry incompatibility without inventing a
production tolerance.

Manufacturer part number, shaft/housing tolerance classes, internal bearing
clearance, preload and L10/contact qualification remain explicitly unassigned.
Consequently AERIS-E1 can pass the deterministic computational screens while
remaining **NOT RELEASE READY**.

`engineering_validate_e1.py` performs deterministic first-order screens for mass,
rotor centrifugal loading, shaft torsion/bending and critical speed, yoke/volute
static loads, fan/duct/filter sensitivity, hood face velocity, nominal bearing
geometry, winding resistance/current-density/thermal sensitivity and blade-pass
frequency. It is not CFD, continuum FEA, electromagnetic FEA, certified filter
validation or physical prototype testing.

`export_cybr_physics_manifest.py` exports CAD-derived E1 mass/CG/bearing/shaft/
hood/filter/outlet parameters for the separate CYBR PHYSICS repository. That
repository then runs deterministic CPU solid-FEM, thermal-FEM and D3Q19-LBM
reduced-order submodels. Those submodels deliberately keep their limitations in
the resulting report rather than promoting a numerical pass to fabrication or
safety qualification.

## Why AERIS uses V9

Small hero parts can hide presentation defects. AERIS cannot: its roughly
366×240×302 mm envelope, internal reflective surfaces, fine conductors, blades,
filter folds and topology details stress studio scale, material response,
antialiasing/detail retention, cutaway handling and large-assembly export at the
same time. Ordinary CYBR GEO rendering uses the repository-wide V9 photographic
contract; the VTK PBR renderer remains an explicit preview option.

## Reproduce the reference geometry

From the repository root in the documented Linux/WSL environment:

```bash
python -m pip install -r requirements-tested.txt
python -m pip install --no-deps -e .

python examples/aeris/build_assets.py --out build/aeris --exports
python examples/aeris/verify.py --out build/aeris --cad
python examples/aeris/render_assets.py --out build/aeris \
  --views hero cutaway exploded rotor_detail topology_detail \
  --size 1440x936 --spp 384 --threads 4 --depth 12
python examples/aeris/draw_assets.py --out build/aeris
python examples/aeris/video_assets.py --out build/aeris --fps 24
```

## Reproduce AERIS-E1 engineering geometry

```bash
python examples/aeris/build_engineered_assets.py --out build/aeris-e1 --exports
python examples/aeris/verify.py --out build/aeris-e1 --cad
python examples/aeris/engineering_validate_e1.py --out build/aeris-e1 --rpm 3600
python examples/aeris/export_cybr_physics_manifest.py --out build/aeris-e1
```

The rendering command above deliberately specifies a 384-spp delivery budget; it
still uses the same shared V9 renderer/material/studio/finishing implementation.
Omit or change explicit quality flags when doing local studies. `--pbr` selects
the separate fast engineering preview path.

## Geometry coverage

The generated feature map ties construction operations to actual named parts:
extrusions and pockets, fillets/chamfers, Boolean union/difference/intersection,
revolution, multisection blade lofts, asymmetric hollow lofts, freeform solid
lofts, 3-D spline sweeps, analytic helix sweep, toroidal seals/cuts, circular and
linear patterns, analytic strut/node lattice geometry, sampled implicit gyroid
geometry, parallel-transport conductor tubes, shelling and physical typography.

`verify.py` checks the assembly contract, feature references, coupled rotor
motion, shaft/bearing envelopes, invalid-parameter guards, GLB units/transforms,
actual animation buffers, STEP coverage, gyroid topology, positive-volume
analytic components, the one-solid volute, blade/disk contacts and a genuine CAD
half-section. The test report is evidence of those nominal geometry checks only.

## Engineering limits

Rotor motion is prescribed for visualization. The first-order E1 screens and
current CYBR PHYSICS adapters do not replace CAD-resolved external/internal CFD,
continuum structural/modal/thermal FEA, electromagnetic motor analysis, bearing
manufacturer life calculations, acoustic qualification, certified filter curves,
GD&T/tolerance release, or physical validation. Those remain required before
AERIS can be treated as a real extractor.
