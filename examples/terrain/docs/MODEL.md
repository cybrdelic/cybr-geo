# Process model

The height field is a 2.5D column model. Each column contains an immobile foundation, ordered bulk layer thicknesses, three loose solid inventories, three suspended solid inventories, surface water, and a shallow soil-water bucket. Solid inventories use solid-equivalent depth; layer porosity and loose-bed porosity are applied explicitly when converting to volume or bulk height.

Initial badlands geology uses gently warped strata clipped by a continuous branching drainage network. The network, large-scale ridge relief, and pre-existing incision represent an authored landscape history. Watershed geology includes channel alluvium and variable soil cover; the soil-profile preset preserves ordered organic, mineral, clay-rich, and sandy horizons. These initial forms are not presented as an outcome of a few minutes of rain.

## Surface flow and transported material

Shared signed face discharges use a local-inertial runoff update with Manning friction. Hydrostatic face depths and a CFL bound handle receding wet fronts. A donor limiter bounds each outgoing flux by available water. Suspended gravel, sand, and fines follow the same upwind face fluxes, including recorded open-boundary exports. Internal updates are equal and opposite.

Bed exchange compares approximate shear stress against exposed-layer resistance. Removed solid material first comes from loose sediment, then from the highest surviving layer, preserving the donor mixture. Three mobility thresholds and three settling speeds distinguish gravel, sand, and fines. Dry cells deposit their suspended inventory. A slope/repose transfer moves actual material downslope rather than adjusting heights without accounting for mass.

Rain and an explicit upstream inlet are recorded sources. Open edges are recorded sinks. Infiltration moves water into a bounded shallow bucket; evaporation removes water from the accounted inventory. There is no groundwater PDE or calibrated turbulence closure.

## Limits

The material rates are illustrative. The bed-exchange acceleration factor is recorded in every simulation report. A conservative implementation demonstrates numerical bookkeeping, not physical calibration. The model cannot represent caves, overhang formation, groundwater circulation, debris mechanics, or geotechnical failure. Mesoscale fragments and shader microrelief are visual detail and are excluded from the sediment ledger.
