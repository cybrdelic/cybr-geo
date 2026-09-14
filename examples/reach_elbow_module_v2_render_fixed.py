"""Render-facing shim for robust REACH-1 v2."""
from __future__ import annotations

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIXED = ROOT / "examples" / "reach_elbow_module_v2_fixed.py"

spec = importlib.util.spec_from_file_location("reach_v2_fixed_impl", FIXED)
impl = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(impl)

v1 = impl.load_base()

build = impl.build
ORBIT_PATTERN = v1.ORBIT_PATTERN
REDUCTION = v1.REDUCTION
PERIOD = v1.PERIOD
elbow_angle = v1.elbow_angle
