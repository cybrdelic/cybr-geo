"""Render REACH-1 v2 + ORBIT through the verified v9 Mitsuba pipeline.

This is a thin adapter over reach_v1_mitsuba.py. It swaps in the v2 CAD recipe
and rotates the photographic viewpoint farther toward the service side so the
bearing stack, completed gearbox hardware and lower load path are visible.
No image generation is used.
"""
from __future__ import annotations

import importlib.util
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE_RENDER = ROOT / "tools" / "reach_v1_mitsuba.py"
V2_RECIPE = ROOT / "examples" / "reach_elbow_module_v2_render.py"


def load_base_renderer():
    spec = importlib.util.spec_from_file_location("reach_v1_renderer_for_v2", BASE_RENDER)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def arg_value(name: str):
    if name not in sys.argv:
        return None
    i = sys.argv.index(name)
    return sys.argv[i + 1]


def main():
    renderer = load_base_renderer()
    renderer.RECIPE = V2_RECIPE

    # reach_v1_mitsuba currently has fixed hero camera angles. Override only
    # those two literal conversions so the exact same physical renderer runs
    # from a more revealing service-side 3/4 view. Everything else, including
    # material tuning, HDRI, AOVs, OIDN and tone mapping, is unchanged.
    original_radians = renderer.math.radians

    def camera_radians(value):
        if abs(value - 35.0) < 1e-12:
            return original_radians(56.0)
        if abs(value - 18.5) < 1e-12:
            return original_radians(16.0)
        return original_radians(value)

    renderer.math.radians = camera_radians
    renderer.main()

    out_arg = arg_value("--out")
    if not out_arg:
        return
    out = Path(out_arg)

    # Rename the inherited output names and correct the provenance manifest so
    # delivered artifacts explicitly identify the v2 geometry source.
    renames = {
        "REACH_v1_ORBIT_hero.png": "REACH_v2_ORBIT_hero.png",
        "REACH_v1_ORBIT_noisy.png": "REACH_v2_ORBIT_noisy.png",
        "REACH_v1_manifest.json": "REACH_v2_manifest.json",
    }
    for old, new in renames.items():
        src = out / old
        if src.exists():
            src.rename(out / new)

    manifest_path = out / "REACH_v2_manifest.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        manifest["model"] = "CYBR REACH-1 v2 + ORBIT revision 3"
        manifest["geometry_source"] = "examples/reach_elbow_module_v2.py layered on validated REACH-1 + unchanged ORBIT"
        manifest["camera"]["azimuth_degrees"] = 56.0
        manifest["camera"]["elevation_degrees"] = 16.0
        manifest["completion_pass"] = {
            "bearing_retainers": True,
            "thrust_spacers": True,
            "shaft_spacers_and_collars": True,
            "clevis_crossbrace_fasteners": True,
            "saddle_gussets": True,
            "lower_flange_gussets": True,
            "hollow_shaft_service_gland": True,
        }
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")

    final = out / "REACH_v2_ORBIT_hero.png"
    print("DELIVERED_V2", final, flush=True)


if __name__ == "__main__":
    main()
