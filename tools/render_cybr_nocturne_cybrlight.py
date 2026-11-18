"""Render CYBR NOCTURNE with CYBR LIGHT using exact CYBR GEO meshes.

This script intentionally bridges the two repositories instead of recreating the
outfit in the renderer.  Install CYBR LIGHT normally, set CYBR_LIGHT_ROOT, or
place a cybr-light checkout beside cybr-geo.

Examples:
    python tools/render_cybr_nocturne_cybrlight.py --preset smoke
    python tools/render_cybr_nocturne_cybrlight.py --preset preview --view hero
    python tools/render_cybr_nocturne_cybrlight.py --preset reference --view detail
    CYBR_LIGHT_ROOT=../cybr-light python tools/render_cybr_nocturne_cybrlight.py --export-only
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
from pathlib import Path
import sys
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def _import_cybrlight():
    try:
        import cybrlight  # type: ignore
        return cybrlight
    except ModuleNotFoundError:
        candidate = Path(os.environ.get("CYBR_LIGHT_ROOT", ROOT.parent / "cybr-light")).resolve()
        python_dir = candidate / "python"
        if not (python_dir / "cybrlight" / "__init__.py").is_file():
            raise ModuleNotFoundError(
                "CYBR LIGHT is not importable. Install cybr-light or set "
                "CYBR_LIGHT_ROOT to its repository checkout."
            )
        sys.path.insert(0, str(python_dir))
        import cybrlight  # type: ignore
        return cybrlight


def _load_outfit():
    path = ROOT / "examples" / "cybr_nocturne_outfit.py"
    spec = importlib.util.spec_from_file_location("cybr_nocturne_outfit", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.build()


# CYBR GEO uses mm, +Z up, +Y front. CYBR LIGHT uses metres and its examples
# conventionally use +Y up. Rotate Z-up -> Y-up without mirroring.
_GEO_TO_LIGHT = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=np.float64,
)


def _convert_mesh(vertices: np.ndarray, normals: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    v = np.asarray(vertices, dtype=np.float64) @ _GEO_TO_LIGHT.T
    n = np.asarray(normals, dtype=np.float64) @ _GEO_TO_LIGHT.T
    v *= 0.001
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    return v, n


def _materials(scene) -> list[int]:
    """CYBR LIGHT equivalents for the CYBR GEO authored palette."""
    return [
        scene.material(
            name="mannequin_graphite",
            type="plastic",
            color=(0.030, 0.032, 0.037),
            roughness=0.78,
            ior_a=1.46,
        ),
        scene.material(
            name="nocturne_black_wool",
            type="plastic",
            color=(0.004, 0.005, 0.007),
            roughness=0.78,
            ior_a=1.47,
        ),
        scene.material(
            name="graphite_technical_textile",
            type="plastic",
            color=(0.009, 0.011, 0.015),
            roughness=0.64,
            ior_a=1.47,
        ),
        scene.material(
            name="deep_amethyst_satin",
            type="plastic",
            color=(0.032, 0.005, 0.046),
            roughness=0.36,
            ior_a=1.50,
        ),
        scene.material(
            name="black_calf_leather",
            type="plastic",
            color=(0.006, 0.007, 0.009),
            roughness=0.40,
            ior_a=1.52,
        ),
        scene.material(
            name="brushed_gunmetal",
            type="metal",
            eta=(0.30, 0.38, 0.52),
            k=(3.90, 3.35, 2.70),
            roughness=0.24,
            alpha_u=0.10,
            alpha_v=0.24,
        ),
        scene.material(
            name="deep_amethyst_leather",
            type="plastic",
            color=(0.028, 0.004, 0.043),
            roughness=0.46,
            ior_a=1.52,
        ),
        scene.material(
            name="carbon_rubber",
            type="plastic",
            color=(0.008, 0.010, 0.014),
            roughness=0.90,
            ior_a=1.50,
        ),
        scene.material(
            name="black_seam_binding",
            type="plastic",
            color=(0.015, 0.017, 0.022),
            roughness=0.50,
            ior_a=1.48,
        ),
    ]


def _camera(view: str, Camera):
    if view == "front":
        return Camera(origin=(0.0, 1.10, -4.55), target=(0.0, 0.96, -0.02), fov=25, aperture=0.0, focus=4.55)
    if view == "back":
        return Camera(origin=(0.34, 1.10, 4.48), target=(0.0, 0.96, 0.0), fov=25, aperture=0.0, focus=4.50)
    if view == "detail":
        return Camera(origin=(0.95, 1.50, -2.35), target=(0.0, 1.28, -0.08), fov=27, aperture=0.012, focus=2.55)
    return Camera(origin=(1.10, 1.18, -4.55), target=(0.0, 0.96, -0.02), fov=25, aperture=0.008, focus=4.70)


def build_scene(view: str = "hero", preset: str = "preview"):
    cybrlight = _import_cybrlight()
    Scene = cybrlight.Scene
    Settings = cybrlight.Settings
    Camera = cybrlight.Camera

    outfit = _load_outfit()
    scene = Scene("CYBR NOCTURNE / CYBR GEO x CYBR LIGHT")
    scene.asset_base = str(ROOT)
    budgets = {
        "smoke": (320, 480, 8, 6),
        "iterate": (480, 720, 12, 6),
        "proof": (480, 720, 40, 8),
        "preview": (640, 960, 72, 10),
        "reference": (900, 1350, 256, 14),
    }
    width, height, spp, bands = budgets[preset]
    scene.settings = Settings(
        width=width,
        height=height,
        spp=spp,
        bands=bands,
        max_depth=14,
        rr_depth=5,
        threads=max(1, min(8, os.cpu_count() or 4)),
        seed=90210,
        exposure=0.98,
        mis=True,
        nee=True,
        film_format="openexr",
        integrator="path",
        sampler=1,
        filter="tent",
    )
    scene.camera = _camera(view, Camera)

    material_map = _materials(scene)
    for part in outfit.parts:
        # The procedural body is a construction/dress-form guide.  In the
        # beauty render the clothing replaces torso, limbs and pelvis; keeping
        # those hidden surfaces caused gray mannequin geometry to punch through
        # small garment gaps.  Only the faceless head remains visible.
        if part.group == "mannequin" and part.name != "mannequin_head":
            continue
        vertices, normals = _convert_mesh(part.vertices, part.normals)
        scene.mesh(
            vertices,
            part.faces,
            material_map[part.material],
            normals=normals,
        )

    # Charcoal studio floor.
    floor = scene.material(
        name="charcoal_studio_floor",
        type="plastic",
        color=(0.002, 0.0025, 0.0035),
        roughness=0.58,
        ior_a=1.48,
    )
    scene.quad((-5.0, 0.0, -5.0), (0.0, 0.0, 10.0), (10.0, 0.0, 0.0), floor)

    # Broad emitters create fashion-studio gradients in addition to spectral
    # environment lobes. These are geometry seen by the same path integrator.
    # One overhead geometry softbox gives broad material gradients without
    # entering either front or rear camera frustum. Directional lights provide
    # the key/fill/rim and therefore cannot show up as white cards in-frame.
    overhead = scene.material(
        name="overhead_softbox",
        type="emitter",
        color=(1.0, 0.96, 0.92),
        emission=12.0,
        kelvin=5200,
    )
    scene.rectangle((-1.55, 4.15, -0.70), (3.10, 0.0, 0.0), (0.0, 0.0, 1.40), overhead)

    if view == "back":
        scene.directional_light((0.38, -1.0, 0.62), irradiance=2.5)
        scene.directional_light((-0.58, -0.70, 0.30), irradiance=1.05)
        scene.point_light((-1.55, 2.20, 2.25), intensity=(0.78, 0.86, 1.0), scale=12.0)
        scene.point_light((1.75, 1.62, 1.35), intensity=(1.0, 0.78, 0.62), scale=5.0)
    else:
        scene.directional_light((-0.38, -1.0, -0.62), irradiance=2.6)
        scene.directional_light((0.58, -0.72, -0.28), irradiance=1.05)
        scene.point_light((1.55, 2.20, -2.35), intensity=(1.0, 0.84, 0.70), scale=20.0)
        scene.point_light((-1.65, 1.70, -1.35), intensity=(0.72, 0.82, 1.0), scale=9.0)

    scene.environment.update(color=[0.24, 0.28, 0.40], strength=0.08)
    scene.environment["lobes"] = [
        {"direction": [-0.58, 0.72, -0.34], "exponent": 14, "strength": 0.42, "kelvin": 6500},
        {"direction": [0.48, 0.50, 0.72], "exponent": 22, "strength": 0.22, "kelvin": 8500},
    ]

    scene.notes.extend(
        [
            "Geometry source: examples/cybr_nocturne_outfit.py in CYBR GEO.",
            "All visible outfit surfaces are explicit procedural meshes; no scan, generated mesh, or generated texture input.",
            "The dress-form torso/limbs are construction guides and are omitted from beauty renders; only the procedural faceless head is visible.",
            "CYBR GEO millimetres/Z-up are converted to CYBR LIGHT metres/Y-up without mirroring.",
            "Preview is spectral path tracing with display transform only; no denoising.",
        ]
    )
    return scene, outfit


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--view", choices=("hero", "front", "back", "detail"), default="hero")
    parser.add_argument("--preset", choices=("smoke", "iterate", "proof", "preview", "reference"), default="preview")
    parser.add_argument("--out", type=Path, default=ROOT / "rendered" / "cybr_nocturne")
    parser.add_argument("--export-only", action="store_true")
    parser.add_argument("--executable", type=Path, help="Explicit CYBR LIGHT native renderer executable")
    args = parser.parse_args()

    cybrlight = _import_cybrlight()
    scene, outfit = build_scene(args.view, args.preset)
    args.out.mkdir(parents=True, exist_ok=True)
    prefix = args.out / f"{args.view}_{args.preset}"

    record: dict[str, Any] = {
        "design": outfit.name,
        "view": args.view,
        "preset": args.preset,
        "cybr_geo_validation": outfit.validate(),
        "geometry_source": "examples/cybr_nocturne_outfit.py",
        "render_backend": "CYBR LIGHT native spectral path tracer",
        "image_generated": False,
        "denoised": False,
    }

    if args.export_only:
        bundle = cybrlight.bundle_scene(scene, prefix)
        record["bundle"] = str(bundle)
    else:
        report = cybrlight.render(scene, prefix, executable=args.executable)
        record["render_report"] = report
        record["preview"] = str(prefix.with_suffix(".png"))
        record["film"] = str(prefix.with_suffix(".pfm"))
        if not prefix.with_suffix(".png").is_file():
            raise RuntimeError("CYBR LIGHT completed without a PNG preview")

    prefix.with_suffix(".integration.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps(record, indent=2))


if __name__ == "__main__":
    main()
