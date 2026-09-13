"""Render ORBIT with a stricter photographic material/camera study.

This is deliberately geometry-preserving: no geometry, service path, or kinematics
are changed. The study only changes authored PBR parameters, camera settings,
lighting controls, sample count, bounce depth, and final supersampled delivery.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
from dataclasses import replace
from pathlib import Path

from PIL import Image

from mechanism_lab.photoreal import render_photoreal

ROOT = Path(__file__).resolve().parents[1]
RECIPE = ROOT / "examples" / "orbit_inspection_wrist.py"


def load_recipe():
    spec = importlib.util.spec_from_file_location("orbit_v4_recipe", RECIPE)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def photographic_materials(assembly):
    """Use physically calmer reflectance values than the glossy showcase pass."""
    tuned = []
    for material in assembly.materials:
        name = material.name.lower()
        if "graphite anodized" in name:
            material = replace(material, metal=.72, rough=.31, ior=1.58,
                               coat=.48, coat_rough=.18, anisotropy=.10)
        elif "teal anodized" in name:
            material = replace(material, metal=.70, rough=.29, ior=1.58,
                               coat=.52, coat_rough=.17, anisotropy=.10)
        elif "satin machined aluminium" in name:
            material = replace(material, color=(.69, .715, .735), metal=1.0,
                               rough=.235, coat=.05, coat_rough=.20,
                               anisotropy=.72)
        elif "hardened steel" in name:
            material = replace(material, color=(.54, .565, .59), metal=1.0,
                               rough=.19, coat=.03, coat_rough=.16,
                               anisotropy=.48)
        elif "warm bronze" in name:
            material = replace(material, color=(.62, .365, .145), metal=1.0,
                               rough=.245, coat=.04, coat_rough=.18,
                               anisotropy=.38)
        elif "black elastomer" in name:
            material = replace(material, color=(.008, .0095, .012), metal=0.0,
                               rough=.78, ior=1.47, coat=.0, anisotropy=.0)
        elif "ceramic white" in name:
            material = replace(material, color=(.72, .76, .79), metal=.0,
                               rough=.36, ior=1.52, coat=.06,
                               coat_rough=.24, anisotropy=.0)
        elif "connector copper" in name:
            material = replace(material, color=(.63, .275, .105), metal=.98,
                               rough=.185, coat=.02, coat_rough=.15,
                               anisotropy=.42)
        tuned.append(material)
    return replace(assembly, materials=tuned)


def photographic_views(assembly):
    views = dict(assembly.views)
    common = dict(
        studio_style="product",
        environment_strength=.17,
        background_strength=.92,
        light_size=1.42,
        light_intensity=.82,
        exposure=1.22,
        floor_color=(.038, .041, .046),
        floor_roughness=.67,
        background_color=(.0065, .0085, .0115),
        tone_mapping="neutral",
    )
    views["hero"] = replace(
        views["hero"], **common, az=31, el=22.5, scale=94,
        target=(34, 6, 61), focal_length_mm=82, f_stop=9.0,
    )
    views["internal"] = replace(
        views["internal"], **common, az=48, el=26, scale=95,
        target=(30, 8, 63), focal_length_mm=78, f_stop=10.0,
        environment_strength=.20, light_intensity=.90,
    )
    views["macro"] = replace(
        views["macro"], **common, az=10, el=18, scale=34,
        focal_length_mm=105, f_stop=7.1,
        camera_distance_mm=264.0, focus_distance_mm=249.0,
        environment_strength=.19, light_size=1.25,
    )
    return replace(assembly, views=views)


def resize_checked(source: Path, destination: Path, size: tuple[int, int]):
    with Image.open(source) as image:
        image = image.convert("RGB").resize(size, Image.Resampling.LANCZOS)
        image.save(destination, format="PNG", compress_level=6)
    with Image.open(destination) as check:
        check.verify()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--size", default="2560x1920")
    parser.add_argument("--deliver-size", default="1920x1440")
    parser.add_argument("--spp", type=int, default=640)
    parser.add_argument("--depth", type=int, default=16)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--views", nargs="+", default=["hero", "internal", "macro"])
    args = parser.parse_args()

    native_size = tuple(map(int, args.size.split("x")))
    deliver_size = tuple(map(int, args.deliver_size.split("x")))
    args.out.mkdir(parents=True, exist_ok=True)

    recipe = load_recipe()
    unloaded = photographic_views(photographic_materials(recipe.build()))
    held = photographic_views(photographic_materials(recipe.with_inspection_coupon(unloaded)))

    manifest = {
        "geometry_changed": False,
        "native_size": list(native_size),
        "deliver_size": list(deliver_size),
        "spp": args.spp,
        "depth": args.depth,
        "views": {},
        "notes": [
            "No generated imagery, image-to-image synthesis, geometry substitution, or frame interpolation.",
            "ORBIT v3 CAD and kinematics are unchanged; only authored optical/material/render settings differ.",
            "Native renders are supersampled and Lanczos-downsampled for final anti-aliasing.",
        ],
    }

    for view_name in args.views:
        scene = held if view_name in ("hero", "macro") else unloaded
        native = args.out / f"ORBIT_v4_{view_name}_native.png"
        report = render_photoreal(
            scene, native, view_name, native_size, args.spp, args.threads,
            args.depth, intent="concept"
        )
        final = args.out / f"ORBIT_v4_{view_name}.png"
        resize_checked(native, final, deliver_size)
        manifest["views"][view_name] = {
            "native": native.name,
            "final": final.name,
            "render_report": native.with_suffix(".json").name,
            "camera": report,
        }
        print("DELIVERED", final, flush=True)

    (args.out / "ORBIT_v4_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    main()
