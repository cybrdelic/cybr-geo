"""Render robust REACH-1 v2 + ORBIT through the v9 Mitsuba pipeline."""
from __future__ import annotations

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "tools" / "reach_v2_mitsuba.py"
FIXED_RECIPE = ROOT / "examples" / "reach_elbow_module_v2_render_fixed.py"

spec = importlib.util.spec_from_file_location("reach_v2_renderer_adapter", BASE)
renderer = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(renderer)
renderer.V2_RECIPE = FIXED_RECIPE

if __name__ == "__main__":
    renderer.main()
