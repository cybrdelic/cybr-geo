"""Engineering qualification entry point for REACH-2 v2 robust CAD."""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE_ENGINEERING = ROOT / "tools" / "reach2_engineering.py"
ROBUST_RECIPE = ROOT / "examples" / "reach2_engineered_elbow_v2.py"


def _load_base():
    spec = importlib.util.spec_from_file_location("reach2_engineering_base", BASE_ENGINEERING)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    module.RECIPE = ROBUST_RECIPE
    return module


_base = _load_base()


def qualify():
    report = _base.qualify()
    report["cad_revision"] = "examples/reach2_engineered_elbow_v2.py"
    return report


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path)
    ap.add_argument("--require-pass", action="store_true")
    args = ap.parse_args()
    report = qualify()
    text = json.dumps(report, indent=2)
    print(text)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n")
    if args.require_pass and not report["qualified"]:
        failed = [c["name"] for c in report["checks"] if not c["passed"]]
        raise SystemExit("REACH-2 v2 engineering gate failed: " + ", ".join(failed))


if __name__ == "__main__":
    main()
