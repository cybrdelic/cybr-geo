"""Robust-CAD revision of CYBR REACH-2.

The engineering geometry is unchanged from reach2_engineered_elbow.py. This
revision only makes cosmetic edge radii manufacturing-safe: a requested fillet
is capped below half the thinnest local stock dimension and, if OCCT still
cannot solve that fillet, the part is emitted with a sharp machinable edge.
No structural section is reduced to make CAD validation pass.
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

import cadquery as cq

ROOT = Path(__file__).resolve().parents[1]
BASE_PATH = ROOT / "examples" / "reach2_engineered_elbow.py"


def _load_base():
    spec = importlib.util.spec_from_file_location("reach2_engineered_base", BASE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


_base = _load_base()

# Re-export public constants used by the qualification and renderer layers.
for _name in dir(_base):
    if _name.isupper():
        globals()[_name] = getattr(_base, _name)


def _robust_box(dx, dy, dz, center, fillet=0.0):
    raw = cq.Workplane("XY").box(dx, dy, dz)
    if fillet:
        # Keep at least 10% flat stock across the thinnest dimension.
        radius = min(float(fillet), 0.45 * min(float(dx), float(dy), float(dz)))
        if radius > 1e-6:
            try:
                candidate = raw.edges().fillet(radius)
                shape = candidate.val()
                if shape.isValid():
                    return shape.translate(tuple(center))
            except Exception:
                pass
    return raw.val().translate(tuple(center))


def build():
    # build() resolves box through the base module globals, so this is a real
    # source-level CAD policy change rather than a rendered-mesh modification.
    _base.box = _robust_box
    assembly = _base.build()
    metadata = dict(assembly.metadata)
    metadata.update(
        cad_revision="REACH-2 v2 robust thin-section fillets",
        fillet_policy="radius <= 45% of thinnest stock; unsolved cosmetic fillets fall back to sharp machinable edges",
    )
    return assembly.__class__(
        "CYBR REACH-2 v2 + ORBIT",
        list(assembly.parts),
        list(assembly.materials),
        dict(assembly.views),
        metadata,
        assembly.motion_function,
    )


# Public kinematic helper used by the physical renderer manifest.
def elbow_angle(t):
    return _base.elbow_angle(t)


if __name__ == "__main__":
    a = build()
    print(a.name, len(a.parts), "parts", "ratio", RATIO)
