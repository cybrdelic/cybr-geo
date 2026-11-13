# Optional analytic solid construction

Copy `cybrgeo/solid.py`, `tests/test_solid.py`, and `examples/housing.py` into the
new GEO package. Do NOT copy the baseline modules in this scratch directory over
other workers' changes. No new import in `cybrgeo/__init__.py` is necessary:

```python
from cybrgeo.solid import Solid
body = Solid.box((80, 60, 30)).fillet(4, '|Z')
body = body.shell(-2, opening='>Z')
mesh = body.mesh(tolerance=0.05)
```

Dependency: optional `cadquery>=2.7,<2.8` (tested 2.7.0 and its installed OCP).
SciPy is needed only for tessellation seam welding. All distances use caller's
consistent model units; angles for `rotate` are degrees. Tessellation's angular
tolerance is radians. Importing the module does not load CadQuery until needed.
Do not describe this as proprietary licensed user content: it is procedural
construction using the installed open-source geometry kernel.

## What is implemented

- Analytic volumetric union, subtraction, intersection, with validation of every
  resulting B-rep. Empty intersection is an explicit value; disjoint solids are
  allowed. Input operations return new wrappers.
- Analytic offsets/shelling, optional removed opening faces, inward/outward wall
  thickness; fillets and chamfers selected on current analytic edges.
- Conversion into existing CYBR GEO Mesh and its validation. Closed geometric
  topology must survive conversion; no silent hole filling/repair.
- 17-stage actuator housing, flange fused into barrel, blind cavity, shaft bore,
  bearing recess, four counterbored mount holes, eight rounded axial vents.
  Intermediate solids and actual cutting tools are exported for viewport use.
- 6 focused tests: analytic volume expectations, operand preservation, empty and
  disconnected output, shell wall volume, fillet/chamfer geometry, through-hole
  triangulated volume convergence, rejected parameters/oversized fillet.

## Honest limits

This adds a separate analytic solid representation, not robust booleans for
arbitrary sculpted/imported triangles. Its results can feed the common Mesh,
spatial/attribute/deformation tools. Do not promise a conversion of any arbitrary
Mesh back into a sound CAD solid. This wrapper does not implement that.

The underlying B-rep kernel is floating-point, tolerance-based CAD, not exact
arithmetic or a manufacturing certificate. Coincident/tangent, tiny/sliver or
self-intersecting input features can fail. Native Boolean errors are surfaced;
shell and fillet failures advise reducing offsets or selecting compatible edges.
No automated shrink/retry changes the user's dimensions.

Selectors like `'>Z'` and `'|Z'` are current-shape CadQuery selectors, NOT stable
feature references across arbitrary edits. Persistent feature matching remains
separate work. Materials, UVs and original mesh attributes are not transferred:
mesh export intentionally has no attributes rather than false provenance.

`shape` is exposed for interoperability, and must not be mutated by callers.
Wrappers use new shapes for every supported operation; frozen Python dataclass
is not a guarantee against direct mutations through the CAD interoperability API.

`mesh(tolerance, angular_tolerance, weld_tolerance)` is approximate; weld tolerance
must be far smaller than intentional gaps. Seam welding only reunites nearby
boundary vertices from the same tessellation. Invalid/degenerate results are
rejected by Mesh, not repaired. Report still says `self_intersections_checked=False`:
B-rep `isValid()` is useful but is not a blanket mesh self-intersection certificate.
No structural, thermal, fluid or motor performance claim is implied by housing.

## Reproduce

```
PYTHONPATH=. python -m unittest discover -s tests -v
PYTHONPATH=. python examples/housing.py
```

Housing observed: 1 solid, volume 72412.689806681 mm³, 5668 vertices, 11384 faces,
1 mesh component, zero boundary/nonmanifold/winding/degenerate-face errors.
