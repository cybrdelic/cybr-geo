# CYBR TERRAIN

CYBR TERRAIN turns a seeded, layered terrain state into textured CYBR GEO geometry and renders it with native CYBR LIGHT. Surface runoff moves gravel, sand, and fines; erosion exposes surviving sediment interfaces; deposition preserves the transported grain mixture; infiltration fills a shallow soil bucket. Commands check water and grain inventories automatically and write a success receipt only after export and rendering finish.

The initial drainage and sediment history are authored, then the storm is simulated. Exchange rates use a recorded acceleration factor. This is a **procedural, uncalibrated process model**, with illustrative materials and render-only rock fragments and soil aggregates. It is unsuitable for geological, flood, or geotechnical prediction.

These development renders use actual meshes, procedural PBR textures, an authored daylight sky, and spectral transport. No generated imagery or scanned assets are used. They demonstrate the pipeline; visible broad-scale faceting and smooth surface detail still limit photographic realism. Plant and root networks, grain-resolved fracture, and turbulent sediment scattering remain outside the model.

![Runoff channel between layered badlands banks](../media/badlands.png)

![Soil-covered watershed and eroded channel](../media/watershed.png)

![Close view of exposed soil horizons, aggregates, and rock fragments](../media/soil-profile.png)

| Render | Resolution | Spectral packets per pixel | Wavelengths per packet |
| --- | --- | ---: | ---: |
| Badlands | 800 × 600 | 96 | 8 |
| Watershed | 800 × 600 | 48 | 8 |
| Soil profile close view | 800 × 600 | 64 | 8 |

## Run the examples

Use Linux, Python 3.11–3.13, and a C++17 compiler. From this example directory:

```bash
sudo apt-get install g++ libcairo2 libegl1 libgl1 libopengl0 libglib2.0-0
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m pip install --no-deps --no-build-isolation -e .
terrain demo all --quality smoke --grid 33 --duration 2 --threads 2
terrain demo badlands --width 800 --spp 96
terrain demo watershed --width 800 --spp 48 --subdivision 3 --stones 400
terrain demo soil-profile --view macro --width 800 --spp 64 --subdivision 3 --stones 450
```

Open `outputs/<preset>/hero.png` or `macro.png` for the render and `terrain.glb` for the textured model. Each run preserves its state, numerical report, authoritative recipe assets, native linear film, display guides, and hashed receipt. Geometry is exported as metric glTF; CYBR LIGHT receives attribute-preserving indexed meshlets. Changing a view or render budget can reuse the checked simulation.

See the [example README](../README.md), [process model](MODEL.md), and [geometry/material notes](GEOMETRY.md).

## Inspect display finishing

| View | Finished display | Unfiltered display |
| --- | --- | --- |
| Watershed | ![Watershed finished](../media/watershed.png) | ![Watershed unfiltered](../media/watershed_unfiltered.png) |
| Soil horizons | ![Soil finished](../media/soil-profile.png) | ![Soil unfiltered](../media/soil-profile_unfiltered.png) |

Display filtering affects the PNG only. Native scene archives retain linear EXR/PFM films, diagnostic guides, meshes, PBR textures, checked saved states and receipts. The earlier badlands preview retains its finished PNG; its original linear film is not part of this release.

## Soil contact correction

The same saved state, camera, geometry and sampling are rendered before and after sharing continuous albedo, normal and roughness across geological contacts. The contact bake changes material mapping; the state and surface triangle geometry are preserved.

| Earlier independent triangle textures | Shared contact material |
| --- | --- |
| ![Earlier soil material boundaries](../media/soil-profile-before.png) | ![Improved continuous soil contacts](../media/soil-profile.png) |

The badlands and watershed previews precede this contact correction. The soil close-up uses the final contact material. Native archives include each view's own immutable assets and receipts, so the earlier previews retain their original source provenance.
