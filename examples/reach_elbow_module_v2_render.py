"""Render-facing shim for REACH-1 v2.

Re-exports the validated v1 kinematic constants expected by the shared Mitsuba
renderer while building the mechanically completed v2 assembly.
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
V2_PATH = ROOT / "examples" / "reach_elbow_module_v2.py"

spec = importlib.util.spec_from_file_location("reach_v2_impl", V2_PATH)
v2 = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(v2)

v1 = v2.load_base()

build = v2.build
ORBIT_PATTERN = v1.ORBIT_PATTERN
REDUCTION = v1.REDUCTION
PERIOD = v1.PERIOD
elbow_angle = v1.elbow_angle
