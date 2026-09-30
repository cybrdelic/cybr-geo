# 6503 Del Playa reference reconstruction

Procedural CYBR GEO reconstruction of the classic east-end Del Playa oceanfront building beside UCSB / Depressions Beach.

The geometry is authored from public exterior references. It does **not** use photogrammetry, scan meshes, scan-derived textures, image billboards, or generated imagery. Dimensions that cannot be recovered from the references are recorded as reference estimates rather than presented as survey measurements.

## References

- Zillow exterior listing photos: https://www.zillow.com/homedetails/6503-Del-Playa-Dr-2-Goleta-CA-93117/2079669030_zpid/
- Depressions Beach / east end of Del Playa context: https://www.californiabeaches.com/beach/depressions-beach/

## Geometry

`recipe.py` builds a named CYBR GEO `Assembly` in millimetres / Z-up. It includes the two-level stucco massing, dark lower bay, wrap-around redwood deck, dense vertical railings, deck supports, glazing, roof vents, patio, bluff-edge fence, visible west neighbor context, procedural bluff/beach/ocean geometry, and authored vegetation.

The corresponding CYBR LIGHT bridge lives in `cybrdelic/cybr-light/tools/render_del_playa_6503.py` and consumes these actual Part vertices directly.
