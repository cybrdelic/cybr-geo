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
            color=(0.070, 0.075, 0.085),
            roughness=0.76,
            ior_a=1.46,
        ),
        scene.material(
            name="obsidian_shell_textile",
            type="plastic",
            color=(0.013, 0.017, 0.024),
            roughness=0.58,
            ior_a=1.47,
        ),
        scene.material(
            name="graphite_technical_weave",
            type="plastic",
            color=(0.040, 0.048, 0.060),
            roughness=0.72,
            ior_a=1.46,
        ),
        scene.material(
            name="ink_satin_lining",
            type="plastic",
            color=(0.080, 0.018, 0.105),
            roughness=0.24,
            ior_a=1.49,
        ),
        scene.material(
            name="blackened_anisotropic_metal",
            type="metal",
            eta=(0.30, 0.38, 0.52),
            k=(3.90, 3.35, 2.70),
            roughness=0.18,
            alpha_u=0.075,
            alpha_v=0.24,
        ),
        scene.material(
            name="spectral_indigo_trim",
            type="metal",
            eta=(0.16, 0.39, 1.20),
            k=(3.55, 2.75, 1.72),
            roughness=0.20,
            alpha_u=0.09,
            alpha_v=0.16,
        ),
        scene.material(
            name="carbon_rubber",
            type="plastic",
            color=(0.010, 0.013, 0.018),
            roughness=0.90,
            ior_a=1.50,
        ),
    ]


def _camera(view: str, Camera):
    if view == "front":
        return Camera(origin=(0.0, 1.12, -4.15), target=(0.0, 0.96, -0.02), fov=29, aperture=0.0, focus=4.2)
    if view == "back":
        return Camera(origin=(-0.75, 1.18, 4.05), target=(0.0, 0.94, 0.0), fov=31, aperture=0.0, focus=4.1)
    if view == "detail":
        return Camera(origin=(1.18, 1.53, -2.15), target=(0.02, 1.25, -0.08), fov=31, aperture=0.018, focus=2.45)
    return Camera(origin=(2.35, 1.34, -4.25), target=(0.0, 0.98, -0.04), fov=31, aperture=0.012, focus=4.75)


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
        "preview": (640, 960, 64, 10),
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
        exposure=1.05,
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
        color=(0.028, 0.032, 0.040),
        roughness=0.64,
        ior_a=1.48,
    )
    scene.quad((-5.0, 0.0, -5.0), (0.0, 0.0, 10.0), (10.0, 0.0, 0.0), floor)

    # Broad emitters create fashion-studio gradients in addition to spectral
    # environment lobes. These are geometry seen by the same path integrator.
    key = scene.material(name="key_softbox", type="emitter", color=(1.0, 0.96, 0.90), emission=10.0, kelvin=5200)
    fill = scene.material(name="fill_softbox", type="emitter", color=(0.73, 0.84, 1.0), emission=6.0, kelvin=7200)
    rim = scene.material(name="rim_strip", type="emitter", color=(0.58, 0.66, 1.0), emission=13.0, kelvin=9000)
    scene.rectangle((-2.15, 3.15, -1.95), (1.55, 0.0, 0.35), (0.0, 1.65, 0.0), key)
    scene.rectangle((2.45, 2.05, -0.75), (0.95, 0.0, 0.20), (0.0, 1.25, 0.0), fill)
    scene.rectangle((-1.90, 2.25, 1.25), (0.75, 0.0, 0.0), (0.0, 1.55, 0.25), rim)

    scene.environment.update(color=[0.31, 0.38, 0.55], strength=0.075)
    scene.environment["lobes"] = [
        {"direction": [-0.58, 0.71, -0.38], "exponent": 18, "strength": 0.62, "kelvin": 6800},
        {"direction": [0.42, 0.43, 0.76], "exponent": 28, "strength": 0.30, "kelvin": 9600},
    ]

    # A small point source makes the black hardware read without flattening the textile.
    scene.point_light((1.7, 2.55, -2.0), intensity=(1.0, 0.84, 0.70), scale=16.0)

    scene.notes.extend(
        [
            "Geometry source: examples/cybr_nocturne_outfit.py in CYBR GEO.",
            "All outfit surfaces are explicit procedural meshes; no scan or image-generation input.",
            "CYBR GEO millimetres/Z-up are converted to CYBR LIGHT metres/Y-up without mirroring.",
            "Preview is spectral path tracing with display transform only; no denoising.",
        ]
    )
    return scene, outfit


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--view", choices=("hero", "front", "back", "detail"), default="hero")
    parser.add_argument("--preset", choices=("smoke", "preview", "reference"), default="preview")
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
