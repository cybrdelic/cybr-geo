"""v9 physical renderer entry point for engineering-qualified REACH-2 v2."""
from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE_RENDERER = ROOT / "tools" / "reach2_mitsuba_v9.py"
ROBUST_RECIPE = ROOT / "examples" / "reach2_engineered_elbow_v2.py"
ROBUST_ENGINEERING = ROOT / "tools" / "reach2_engineering_v2.py"


def _load_base():
    spec = importlib.util.spec_from_file_location("reach2_renderer_base", BASE_RENDERER)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    module.RECIPE = ROBUST_RECIPE
    module.ENGINEERING = ROBUST_ENGINEERING
    return module


def _out_from_argv():
    for i, arg in enumerate(sys.argv[:-1]):
        if arg == "--out":
            return Path(sys.argv[i + 1])
    return None


def main():
    base = _load_base()
    base.main()
    out = _out_from_argv()
    if out:
        manifest_path = out / "REACH2_manifest.json"
        if manifest_path.exists():
            manifest = json.loads(manifest_path.read_text())
            manifest["geometry_source"] = "examples/reach2_engineered_elbow_v2.py + unchanged ORBIT recipe"
            manifest["engineering_source"] = "tools/reach2_engineering_v2.py"
            manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    main()
