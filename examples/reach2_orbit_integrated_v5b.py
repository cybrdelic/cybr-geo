"""Tight-pilot release wrapper for REACH-2 v5.

Uses controlled slip-fit bores rather than the intentionally loose first-pass
visual packaging values. Geometry is otherwise identical to v5.
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "examples" / "reach2_orbit_integrated_v5.py"

spec = importlib.util.spec_from_file_location("reach2_v5_base", BASE)
_v5 = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(_v5)

# Custom mating bores: nominal +20 um, +/-10 um machining tolerance.
# Final fit class must be reconciled to the purchased actuator option/drawing.
_v5.PEDESTAL_PILOT_BORE_MM = 128.020
_v5.OUTPUT_PILOT_BORE_MM = 125.020

for _name in dir(_v5):
    if _name.isupper():
        globals()[_name] = getattr(_v5, _name)

PIVOT = _v5.PIVOT
ORBIT_PATTERN = _v5.ORBIT_PATTERN
LOWER_PATTERN = _v5.LOWER_PATTERN
elbow_angle = _v5.elbow_angle
rotation_y = _v5.rotation_y


def build():
    return _v5.build()


if __name__ == "__main__":
    a = build()
    print(a.name, len(a.parts), "parts", ACTUATOR_MODEL)
