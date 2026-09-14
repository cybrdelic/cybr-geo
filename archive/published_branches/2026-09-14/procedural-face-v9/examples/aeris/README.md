# AERIS / useful-machine geometry showcase

AERIS is an original serviceable benchtop extraction-turbine concept built as a
large CYBR GEO integration target. It is intended to exercise real mechanical
geometry, assembly, cutaway, export, drawing, animation and photographic
presentation workflows together. It is **not** a rated fume extractor or a
respiratory-safety device.

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

## Why AERIS is in the V9 branch

Small hero parts can hide presentation defects. AERIS cannot: its roughly
366×240×302 mm envelope, internal reflective surfaces, fine conductors, blades,
filter folds and topology details stress studio scale, material response,
antialiasing/detail retention, cutaway handling and large-assembly export at the
same time. Ordinary CYBR GEO rendering uses the repository-wide V9 photographic
contract; the VTK PBR renderer remains an explicit preview option.

## Reproduce

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

The render command above deliberately specifies a 384-spp delivery budget; it
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

Rotor motion is prescribed for visualization. No CFD, fan curve, capture
velocity, pressure drop, filtration efficacy, acoustics, structural stress,
fatigue, balance, loaded bearing/contact behavior, electrical safety, thermal
performance or fabrication release is claimed. Those require dedicated analysis
and physical validation before AERIS could be treated as a real extractor.
