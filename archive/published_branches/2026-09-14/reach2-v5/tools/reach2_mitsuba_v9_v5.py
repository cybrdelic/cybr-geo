"""v9 renderer for REACH-2 v5 integrated ORBIT-family elbow."""
from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE_RENDERER = ROOT / "tools" / "reach2_mitsuba_v9.py"
RECIPE = ROOT / "examples" / "reach2_orbit_integrated_v5b.py"
ENGINEERING = ROOT / "tools" / "reach2_engineering_v5.py"


def load_base():
    spec = importlib.util.spec_from_file_location("reach2_v5_renderer_base", BASE_RENDERER)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    module.RECIPE = RECIPE
    module.ENGINEERING = ENGINEERING
    return module


def out_from_argv():
    for i, arg in enumerate(sys.argv[:-1]):
        if arg == "--out":
            return Path(sys.argv[i + 1])
    return None


def main():
    base = load_base()
    base.main()
    out = out_from_argv()
    if out:
        path = out / "REACH2_manifest.json"
        if path.exists():
            manifest = json.loads(path.read_text())
            manifest["model"] = "CYBR REACH-2 v5 integrated ORBIT-family + ORBIT"
            manifest["geometry_source"] = "examples/reach2_orbit_integrated_v5b.py + unchanged ORBIT recipe"
            manifest["engineering_source"] = "tools/reach2_engineering_v5.py"
            manifest["engineering_gate"] = "complete moving CAD + integrated actuator + fastener/tolerance/lower-interface gate"
            manifest["generated_imagery"] = False
            manifest["aesthetic_family"] = "CYBR ORBIT"
            path.write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    main()
