# Planetary Actuator

`planetary_actuator` is an original compact epicyclic reducer/actuator concept built on Mechanism Lab. It is not a reconstruction of a commercial product.

## Nominal gearset

| Parameter | Value |
| --- | ---: |
| Sun | 18 teeth |
| Planets | 3 × 27 teeth |
| Ring | 72 internal teeth |
| Module | 0.8 mm |
| Pressure angle | 20° |
| Face width | 10 mm |
| Circumferential backlash allowance | 0.10 mm at pitch circle |
| Ring | fixed |
| Input | sun |
| Output | carrier |
| Reduction | 5:1 |

The topology is checked before geometry is built:

`Nr = Ns + 2 Np = 18 + 2(27) = 72`.

For the fixed-ring configuration, the Willis relation gives

`omega_c = omega_s / (1 + Nr/Ns) = omega_s / 5`.

The default demonstration runs the sun at 0.45 rev/s, the carrier at 0.09 rev/s, and each planet at -0.15 rev/s absolute while its center orbits with the carrier.

## Geometry contract

`src/mechanism_lab/gears/` is reusable. It contains:

- analytic transverse involute external gear flanks;
- an internal-ring void profile used to cut real internal involute teeth from an annular blank;
- explicit pitch-circle backlash;
- exact simple-planetary center geometry and assembly phase constraints;
- exact rigid sun/carrier/planet kinematics;
- numerical external- and internal-mesh phase residual checks.

Root transitions are concept transitions and are **not** claimed to be hob/shaper cutter envelopes. The solids are therefore appropriate for engineering visualization and kinematic studies, not direct gear manufacturing release.

## Mechanical assembly

The model contains approximately sixty named visual/mechanical components rather than three floating gears. Major interfaces include:

- two-piece 72 mm OD machined housing;
- fixed internal ring/structural insert;
- integral 18T sun and 8 mm input shaft, avoiding an invisible torque joint;
- three 27T planets on carrier-fixed 5 mm pins;
- two bronze close-fit bushings per planet;
- front/rear bronze thrust washers;
- retained planet pins passing through actual carrier-plate clearance bores;
- 12 mm output shaft supported by two 18 mm OD bronze radial bushings;
- 8 mm input shaft supported by two 14 mm OD bronze radial bushings;
- output/input seal envelopes in real housing counterbores;
- carrier-to-output-flange socket screws;
- housing-to-ring screws with housing clearance bores and ring tap-core envelopes;
- motor interface flange, encoder magnet/PCB envelope, and side connector seated in a housing pocket.

Thread helices are not modeled. Tapped interfaces use explicit tap-core envelopes and are labeled as such.

## Authored running/fit allowances

These numbers describe the current concept geometry, not a manufacturing tolerance stack:

- output shaft to bushing radial clearance: 0.08 mm;
- input shaft to bushing radial clearance: 0.08 mm;
- planet pin to bushing radial clearance: 0.06 mm;
- planet gear bore to bushing OD diametral clearance: 0.04 mm;
- carrier OD to internal ring tooth-tip radial clearance: 1.30 mm;
- planet addendum to ring root nominal radial margin: 0.20 mm;
- gear backlash allowance: 0.10 mm circumferential at pitch circle.

## What is verified

Automated tests/validation check:

- tooth-count topology;
- equal-spacing assembly phase condition;
- pitch-circle sun/planet and ring/planet tangency;
- exact 5:1 carrier speed ratio;
- external and internal mesh phase residuals across time;
- rigid proper transforms for every moving component;
- actual support/fastener spans and named interface geometry;
- positive authored clearances;
- truth/provenance status;
- generated CAD validity through Mechanism Lab validation.

## What is not claimed

The current model does **not** qualify:

- loaded tooth contact or contact ratio under deflection;
- Hertz/contact or root bending stress;
- shaft torsion/bending;
- bushing PV limits or wear;
- lubricant selection or churning loss;
- gearbox efficiency, thermal behavior or noise;
- fastener preload/thread strength;
- fatigue or service life;
- GD&T/tolerance stack;
- manufacturability of the root transitions;
- any safe payload/torque rating.

Do not treat the concept as manufacturing-ready without those analyses.

## Commands

Build and validate:

```bash
lab build planetary_actuator --rebuild
lab validate planetary_actuator
```

Fast shot development:

```bash
lab render planetary_actuator --renderer pbr --view hero --size 1600x1000 --intent concept
lab video planetary_actuator --view kinematics --action motion --seconds 4 --size 1280x720
```

Photographic stills:

```bash
lab render planetary_actuator --renderer photoreal --view hero --size 1920x1080 --spp 384 --depth 14 --intent concept
lab render planetary_actuator --renderer photoreal --view gear_macro --size 1920x1080 --spp 384 --depth 14 --intent concept
```

Photoreal proof/final films:

```bash
python tools/render_planetary_film.py --preset proof
python tools/render_planetary_film.py --preset final
```

The final preset is intentionally expensive: every frame is path traced and motion blur is temporal supersampling of real geometry poses, not a screen-space effect.
