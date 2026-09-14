# Nitinol fiber actuator

`nitinol_fiber_actuator` is a parametric straight-wire shape-memory-alloy actuator built on the generic `mechanism_lab` assembly, rendering, animation, CAD export and whiteprint pipeline.

The default model is a **12-fiber, 0.20 mm wire bundle** with **140 mm active wire length** and **3.5% modeled working strain**, producing a nominal geometric stroke of **4.90 mm**. It is deliberately a source-guided engineering concept rather than vendor CAD or a load-qualified product design.

## Default architecture

- 12 straight Nitinol actuator fibers arranged on a 21 mm pitch diameter.
- Four guide rods and a rigid moving carriage.
- Central compression return spring with a 24 N cold preload target and 0.55 N/mm rate target.
- Fixed and moving insulating fiber carriers.
- Individual representative crimp barrels at all fiber ends.
- Three electrically parallel strings, each containing four fibers in series.
- Alternating fixed-end and moving-end copper jumpers let both main supply terminals remain fixed at the rear of the actuator.
- Independent cold and hot hard stops bound the nominal carriage travel.
- Output rod and generic eye interface.

The moving carriage travels toward `-X` during heating. The wire and spring are represented as multiple short rigid pieces whose centers follow the contraction map. That preserves Mechanism Lab's rigid-transform validation while making the contraction visible in videos and animated exports.

## Source-guided wire reference

The default configuration uses Dynalloy's published 0.008 in / 0.20 mm, 70 C Flexinol actuator-wire table as a numerical reference:

- resistance: about 29 ohm/m
- heating pull guide: about 570 g force per wire
- cooling deformation guide: about 228 g force per wire
- approximate current for one-second contraction: 0.66 A per wire
- passive cooling time in static room air: about 3.2 s

Primary references:

- https://dynalloy.com/technical-data-wires/
- https://dynalloy.com/wp-content/uploads/2025/03/TCF1140.pdf
- https://dynalloy.com/crimp-styles/

These values are rough design guides supplied for a commercial actuator-wire family. They do not transfer automatically to arbitrary Nitinol wire, another heat treatment, another diameter tolerance, another crimp process or another thermal environment.

## Derived default electrical/mechanical numbers

For 140 mm of 0.20 mm reference wire:

- one fiber resistance: about 4.06 ohm
- one four-fiber series string: about 16.24 ohm
- nominal string current at the source one-second guide: 0.66 A
- nominal string voltage at that current: about 10.72 V
- three strings in parallel: about 1.98 A total
- nominal electrical input at that operating point: about 21.2 W
- source-guided bundle heating pull: about 67.1 N before return-spring load
- source-guided bundle cooling deformation force: about 26.8 N
- modeled return-spring force: 24.0 N cold to about 26.7 N at full stroke
- estimated hot pull remaining at full modeled stroke: about 40.4 N

The force subtraction is only a static budget. It does not include guide friction, crimp compliance, thermal gradients, wire-to-wire current mismatch, phase hysteresis, spring nonlinearity, acceleration or external linkage losses.

## Why four fibers in series per branch

The actuator has 12 mechanically parallel fibers but does not put all 12 electrically in parallel. A four-wire series path is folded back and forth between the fixed and moving ends:

`fixed + -> fiber 1 -> moving jumper -> fiber 2 -> fixed jumper -> fiber 3 -> moving jumper -> fiber 4 -> fixed -`

Three of those paths run electrically in parallel. The even number of series elements returns both power nodes to the stationary end of the mechanism, avoiding a high-current flexible lead on the carriage.

A physical driver should regulate current rather than assuming resistance is perfectly constant. Add independent over-temperature and timeout protection; the model does not implement electronics or a thermal safety controller.

## Build, render and export

```bash
lab build nitinol_fiber_actuator --step --stl
lab render nitinol_fiber_actuator --view hero
lab render nitinol_fiber_actuator --view fiber_bundle --renderer pathtrace --spp 64
lab render nitinol_fiber_actuator --view electrical
lab video nitinol_fiber_actuator --view hero --action motion --seconds 6
lab video nitinol_fiber_actuator --view exploded --action explode --seconds 8
lab blueprint nitinol_fiber_actuator
lab catalogue nitinol_fiber_actuator
```

The default motion loop uses a one-second smooth heating ramp, a short hot hold, the 3.2 s source-guided passive cooling interval and a cold dwell. It is intentionally a deterministic visualization curve, not a coupled thermal/phase-transformation solver.

## Parameterization

The built-in CLI recipe uses `DEFAULT_CONFIG`. Python callers can create variants directly:

```python
from mechanism_lab.models.nitinol_actuator import ActuatorConfig, build, design_metrics

cfg = ActuatorConfig(
    fiber_count=12,
    fiber_diameter_mm=0.20,
    active_length_mm=180.0,
    design_strain=0.03,
    series_per_string=4,
)
assembly = build(cfg)
print(design_metrics(cfg))
```

`series_per_string` must be even so that the default folded electrical topology returns both supply terminals to the fixed end. The builder also rejects a return-spring target whose full-stroke force exceeds the source-guided bundle cooling-deformation force.

## Physical-build limits

This repository does **not** establish safe continuous current, cycle life, maximum external payload, guide/bushing fits, spring fatigue, crimp pull-out strength, insulation temperature rating, controller design or fail-safe behavior. Those depend on the actual wire, ambient conditions, fabrication and control system.

For a physical prototype, use current limiting, a temperature sensor or equivalent independent thermal protection, a hard actuation timeout, guarded moving parts and a nonflammable test fixture. Crimp attachment is modeled because actuator wire is commonly terminated mechanically; the exact crimp dimensions here are representative rather than copied from a commercial component.
