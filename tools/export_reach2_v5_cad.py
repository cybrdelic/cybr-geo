"""Export REACH-2 v5 + ORBIT as analytic STEP plus GLB and BOM."""
from __future__ import annotations

import argparse
import importlib.util
import shutil
from pathlib import Path

from mechanism_lab.exporters import export_bom, export_glb, export_step

ROOT = Path(__file__).resolve().parents[1]
RECIPE = ROOT / "examples" / "reach2_orbit_integrated_v5b.py"


def load_recipe():
    spec = importlib.util.spec_from_file_location("reach2_v5_export_recipe", RECIPE)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    recipe = load_recipe()
    assembly = recipe.build()

    export_step(assembly, args.out / "REACH2_v5_ORBIT_analytic.step", individual=False)
    export_glb(assembly, args.out / "REACH2_v5_ORBIT.glb")
    export_bom(assembly, args.out / "REACH2_v5_BOM")
    shutil.copy2(RECIPE, args.out / "reach2_orbit_integrated_v5b.py")
    shutil.copy2(ROOT / "examples" / "reach2_orbit_integrated_v5.py",
                 args.out / "reach2_orbit_integrated_v5.py")
    print("CAD_EXPORT_READY", len(assembly.parts), flush=True)


if __name__ == "__main__":
    main()
