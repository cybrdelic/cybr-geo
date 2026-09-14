"""Render-facing shim for REACH-1 v3."""
from __future__ import annotations

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
V3 = ROOT / "examples" / "reach_elbow_module_v3.py"

spec = importlib.util.spec_from_file_location("reach_v3_impl", V3)
impl = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(impl)

v2 = impl.load_v2()
v1 = v2.load_base()

build = impl.build
ORBIT_PATTERN = v1.ORBIT_PATTERN
REDUCTION = v1.REDUCTION
PERIOD = v1.PERIOD
elbow_angle = v1.elbow_angle
