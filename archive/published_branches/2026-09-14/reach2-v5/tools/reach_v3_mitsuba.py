"""Render CYBR REACH-1 v3 + ORBIT with the verified v9 Mitsuba pipeline.

No image generation. This adapter swaps the mechanically completed v3 CAD into
the existing physical renderer and moves the camera only slightly toward the
service side so both gripper jaws, the elbow bearing stack, and the dual-sided
gearbox cassette remain visible in one hero frame.
"""
from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE_RENDER = ROOT / "tools" / "reach_v1_mitsuba.py"
V3_RECIPE = ROOT / "examples" / "reach_elbow_module_v3_render.py"


def load_base_renderer():
    spec = importlib.util.spec_from_file_location("reach_v1_renderer_for_v3", BASE_RENDER)
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
    renderer.RECIPE = V3_RECIPE

    # Keep the full gripper readable while revealing the service-side cassette.
    # The base renderer converts these two literal values through math.radians;
    # override only those conversions, leaving the physical rendering pipeline
    # and all other trigonometry untouched.
    original_radians = renderer.math.radians

    def camera_radians(value):
        if abs(value - 35.0) < 1e-12:
            return original_radians(39.0)
        if abs(value - 18.5) < 1e-12:
            return original_radians(17.5)
        return original_radians(value)

    renderer.math.radians = camera_radians
    renderer.main()

    out_arg = arg_value("--out")
    if not out_arg:
        return
    out = Path(out_arg)

    renames = {
        "REACH_v1_ORBIT_hero.png": "REACH_v3_ORBIT_hero.png",
        "REACH_v1_ORBIT_noisy.png": "REACH_v3_ORBIT_noisy.png",
        "REACH_v1_manifest.json": "REACH_v3_manifest.json",
    }
    for old, new in renames.items():
        src = out / old
        if src.exists():
            src.rename(out / new)

    manifest_path = out / "REACH_v3_manifest.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        manifest["model"] = "CYBR REACH-1 v3 + ORBIT revision 3"
        manifest["geometry_source"] = (
            "examples/reach_elbow_module_v3.py layered on validated REACH-1 v2 + unchanged ORBIT"
        )
        manifest["camera"]["azimuth_degrees"] = 39.0
        manifest["camera"]["elevation_degrees"] = 17.5
        manifest["structural_completion"] = {
            "dual_sided_gearbox_support": True,
            "outboard_compound_shaft_bearing": True,
            "outboard_input_shaft_bearing": True,
            "four_gearbox_standoffs": True,
            "output_shaft_extension_and_endcap": True,
            "motor_encoder_adapter": True,
            "input_coupling_sleeve": True,
            "elbow_hard_stops": True,
            "lower_center_spine": True,
        }
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")

    print("DELIVERED_V3", out / "REACH_v3_ORBIT_hero.png", flush=True)


if __name__ == "__main__":
    main()
