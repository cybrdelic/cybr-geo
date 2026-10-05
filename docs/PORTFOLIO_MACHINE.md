# Portfolio machine reference modules

These recipes turn the six generated portfolio reference sheets from 2026-10-04 into real CYBR GEO geometry. The images are treated as **visual design references only**. Nothing is image-projected, scanned, photogrammetry-derived or baked into the mesh.

Every module is built from CadQuery/OpenCascade solids plus generated tube meshes, so the objects can be inspected as parts, exported to GLB/STEP, exploded, animated where applicable, and rendered through the normal CYBR GEO pipeline.

## Recipes

| Recipe | Reference target | Main geometric systems |
| --- | --- | --- |
| portfolio_scenes | Glass desert-world habitat dome | 1120 mm service base, rotation rings, glass hemisphere, water basin, mesas, miniature vegetation, articulated dome arms |
| portfolio_geo | Large exposed electric motor / generator | 1280 x 640 x 680 mm concept envelope, layered end bells, exposed copper stator bars, rotor, shaft, bearing housings, tie rods, mounts and service lines |
| portfolio_light | Precision laser optics bench | emitter barrel, large prism, vertical relay optic, beam steering mirror, output lens, precision rails, visible geometric beam path |
| portfolio_elements | Fire + water + rock containment cube | 820 mm glass chamber, basalt core, orange hot-element tendrils, water streams/droplets, steam, mechanical frame and service wheel |
| portfolio_materials | Material specimen / flooring machine | 980 x 640 x 720 mm concept envelope, stepped wood/stone/ceramic sample plates, linear carriage, inspection wheel and articulated clamp arm |
| portfolio_forest | Sealed mechanical forest biome | 900 mm base, cylindrical glass vessel + dome, layered terrain, trees, waterfall/stream, moss, filtration columns and inspection arm |
| portfolio_machine | Preferred horizontal homepage object | all six modules connected by a continuous structural/service spine |

The generated sheets explicitly showed dimensions for GEO, ELEMENTS and MATERIALS; those values are retained in assembly metadata under concept_dimensions_mm. Other dimensions were selected to preserve the proportions and lineup language of the references.

## Build and inspect

    lab build portfolio_scenes --step
    lab build portfolio_geo --step
    lab build portfolio_light --step
    lab build portfolio_elements --step
    lab build portfolio_materials --step
    lab build portfolio_forest --step

Each module has hero, orthographic/inspection and exploded views where appropriate.

    lab render portfolio_scenes --view hero --intent concept --allow-estimates
    lab render portfolio_geo --view internal --intent concept --allow-estimates
    lab render portfolio_light --view beam_path --intent concept --allow-estimates
    lab render portfolio_elements --view section --intent concept --allow-estimates
    lab render portfolio_materials --view hero --intent concept --allow-estimates
    lab render portfolio_forest --view internal --intent concept --allow-estimates

The complete horizontal machine:

    lab build portfolio_machine --step
    lab render portfolio_machine --view hero --intent concept --allow-estimates
    lab render portfolio_machine --view wide --intent concept --allow-estimates

For quick native checks before a long photographic render:

    lab preview portfolio_geo --view hero --spp 32
    lab preview portfolio_light --view hero --spp 32
    lab preview portfolio_elements --view hero --spp 32

## Reference fidelity

The goal is not to copy incidental 2D noise from the concept images. The goal is to preserve their **object identity and mechanical silhouette** in real geometry:

- SCENES must read first as a contained world, second as a machine.
- GEO must read as a believable electromechanical object with copper visibly doing structural/functional work rather than as a generic sci-fi cylinder.
- LIGHT must expose the full optical chain so the beam explains the geometry.
- ELEMENTS must keep fire/water/rock spatially distinct enough to read at homepage scale while physically overlapping at the interaction boundary.
- MATERIALS must read as a specimen mechanism, not merely boards placed on a table.
- FOREST must make the life-support hardware part of the object rather than hiding it behind the biome.

All geometry is marked designed-concept. Hidden internals, seals, threads, optical prescriptions, fluid dynamics and ecosystem engineering are deliberately not represented as verified production hardware.

## Portfolio integration intent

portfolio_machine follows the user's preferred first concept: a **horizontal line of distinct functional machines** rather than the radial reactor concept. The modules keep independent silhouettes but share a long structural spine and service bus, allowing a web renderer to use the same asset for both the full homepage composition and project-specific camera dives.

The intended portfolio interaction model is:

1. load the full machine;
2. hover a subsystem to activate local motion/light;
3. move the camera to its pre-authored inspection framing;
4. transition from the portfolio-scale CAD object into that project's actual content;
5. return to the same physical machine instead of switching to a separate card/grid interface.
