# Whiteprint drawing tools

`lab blueprint` and `lab whiteprint` are aliases. They create an A3 landscape sheet in SVG, vector PDF, DXF and PNG preview formats. No image generator, OCR, drawing photograph or image-to-CAD converter is involved.

```bash
lab blueprint m8325s
lab blueprint drivetrain --part Drive_01_99p6_BCD_carrier_adapter \
  --out outputs/drivetrain/drawings/carrier
lab blueprint examples/custom_flange.py --scale 1 \
  --title 'CUSTOM FLANGE / INTERFACE STUDY'
```

The analytic path builds an OpenCascade compound from the selected parts after applying their initial rigid poses. It projects visible and hidden BRep edges along specified view axes through `HLRBRep_Algo`. Curved edges are sampled at a declared 0.025 mm model-space deflection. Thus the linework is vector, but the DXF does not claim exact native analytic splines; it has sampled polylines. Edge directions and view labels are explicit rather than claiming a particular first-/third-angle drawing standard.

The DXF uses millimetres in **paper layout space** and separate VISIBLE, HIDDEN, CENTER, DIM and BORDER layers. Multiply projected dimensions by the inverse of the stated scale to recover model-space line lengths. Text dimensions are nominal model lengths, not pixel or paper measurements. SVG and PDF use the same sheet geometry. A PDF viewer's print-to-fit setting can change the physical printed scale; print at actual size to preserve the title-block scale.

Mixed/mesh-only selections use a VTK feature/silhouette projection fallback, explicitly marked **occlusion not suppressed**. Missing analytic solids are not invented. For a clean technical drawing, select actual BRep components with `--part` or build the model with analytic CAD.

## Put dimensions in recipe data

The shared drawing engine accepts a JSON `annotation_spec` through its Python API or `--annotations file.json`. A model can also provide `metadata['drawings']['default']`, or a drawing keyed by one exact selected part name. Supported keys are `parts`, `title`, `auto_dimensions`, and `annotations`.

An annotation uses one of `dimension`, `line`, `circle`, `leader` or `note`. For geometry anchors, select `view` as `top`, `side`, `end` or `iso`. Anchor points are coordinates in that view's 2D model-space basis, in millimetres. Leader elbows and ends are paper-space offsets from the projected anchor. This separates geometric meaning from legible annotation placement.

```json
{
  "auto_dimensions": true,
  "annotations": [
    {"kind":"circle", "view":"end", "radius":33},
    {"kind":"leader", "view":"end", "point":[33,0],
     "elbow_paper_mm":[12,-13], "end_paper_mm":[22,-13],
     "text":"6 x DIA 6.6 / BCD 66"},
    {"kind":"dimension", "view":"side", "points":[[0,-42],[10,-42]],
     "offset_paper_mm":7, "label":"10 NOMINAL"}
  ]
}
```

Automatic dimensions are only projected extents labelled REF. They are not a complete functional dimensioning scheme. Nominal annotations supplied by a model must be traceable to a source or declared design parameter. This release provides motor interface, carrier-adapter and motor-face-adapter sheets. The latter two are original nominal concept designs, not manufacturer drawings.

The system does not assign unspecified tolerances, choose fits, validate thread engagement, infer datums, perform geometric dimensioning/tolerancing, generate qualified cutter geometry, guarantee collision-free assembly, or approve manufacture. There is no automatic section hatching or manufacturing-standard certification in this version. Those are explicit future engineering features, not silently claimed capabilities.

For an offset part, `anchor_origins` can provide a local 2D datum offset for each view. For example, `"anchor_origins":{"end":[-166.95634516260222,0]}` lets annotations use local coordinates on the offset motor adapter while the assembly remains in its real world position. The engine checks that linework and text anchors remain on the sheet; a mistaken datum cannot silently export clipped-off dimensions.
