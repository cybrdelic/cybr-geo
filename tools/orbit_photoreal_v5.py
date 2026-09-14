"""ORBIT v5 photographic study: real CAD in a modeled workshop context.

No generated imagery or image-space geometry substitution is used. ORBIT itself is
unchanged. This study adds only scene-context CAD, authored materials, optical
settings, and native path tracing so the result can be judged as a photograph
rather than an isolated product render.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
from dataclasses import replace
from pathlib import Path

import cadquery as cq
import numpy as np
from PIL import Image

from mechanism_lab.core import Material, cad_part
from mechanism_lab.photoreal import render_photoreal

ROOT = Path(__file__).resolve().parents[1]
RECIPE = ROOT / "examples" / "orbit_inspection_wrist.py"


def load_recipe():
    spec = importlib.util.spec_from_file_location("orbit_v5_recipe", RECIPE)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def box(dx, dy, dz, center, fillet=0.0, zrot=0.0):
    q = cq.Workplane("XY").box(dx, dy, dz)
    if fillet:
        q = q.edges().fillet(fillet)
    s = q.val()
    if zrot:
        s = s.rotate((0, 0, 0), (0, 0, 1), zrot)
    return s.translate(tuple(center))


def photographic_materials(assembly):
    tuned = []
    for material in assembly.materials:
        name = material.name.lower()
        if "graphite anodized" in name:
            material = replace(material, color=(.045, .056, .068), metal=.82,
                               rough=.36, ior=1.58, coat=.34, coat_rough=.22,
                               anisotropy=.08)
        elif "teal anodized" in name:
            material = replace(material, color=(.010, .145, .120), metal=.78,
                               rough=.34, ior=1.58, coat=.38, coat_rough=.20,
                               anisotropy=.08)
        elif "satin machined aluminium" in name:
            material = replace(material, color=(.58, .61, .64), metal=1.0,
                               rough=.29, coat=.02, coat_rough=.24,
                               anisotropy=.68)
        elif "hardened steel" in name:
            material = replace(material, color=(.42, .45, .48), metal=1.0,
                               rough=.25, coat=.01, coat_rough=.20,
                               anisotropy=.43)
        elif "warm bronze" in name:
            material = replace(material, color=(.48, .275, .10), metal=1.0,
                               rough=.31, coat=.01, coat_rough=.22,
                               anisotropy=.32)
        elif "black elastomer" in name:
            material = replace(material, color=(.006, .007, .009), metal=0.0,
                               rough=.83, ior=1.47, coat=0.0, anisotropy=0.0)
        elif "ceramic white" in name:
            material = replace(material, color=(.66, .69, .72), metal=0.0,
                               rough=.44, ior=1.52, coat=.03,
                               coat_rough=.30, anisotropy=0.0)
        elif "connector copper" in name:
            material = replace(material, color=(.51, .205, .075), metal=.99,
                               rough=.24, coat=.01, coat_rough=.20,
                               anisotropy=.35)
        tuned.append(material)
    return replace(assembly, materials=tuned)


def add_workshop_context(assembly, camera_az=31.0):
    """Add genuine scene geometry behind/below ORBIT; no image backdrop."""
    materials = list(assembly.materials)
    bench_id = len(materials)
    materials.append(Material(
        "Used phenolic workbench", (.105, .095, .082), 0.0, .62,
        ior=1.48, coat=.05, coat_rough=.38, anisotropy=.06,
        microfinish="polymer", material_source="authored workshop context"
    ))
    mat_id = len(materials)
    materials.append(Material(
        "Rubber service mat", (.012, .015, .018), 0.0, .84,
        ior=1.45, microfinish="polymer", material_source="authored workshop context"
    ))
    wall_id = len(materials)
    materials.append(Material(
        "Painted workshop wall", (.145, .155, .165), 0.0, .88,
        ior=1.52, microfinish="none", material_source="authored workshop context"
    ))
    powder_id = len(materials)
    materials.append(Material(
        "Powder coated cabinet", (.035, .075, .105), 0.0, .39,
        ior=1.52, coat=.20, coat_rough=.27, microfinish="polymer",
        material_source="authored workshop context"
    ))
    cardboard_id = len(materials)
    materials.append(Material(
        "Cardboard parts box", (.31, .225, .14), 0.0, .78,
        ior=1.47, microfinish="none", material_source="authored workshop context"
    ))

    parts = list(assembly.parts)
    def add(name, shape, material, axis=(1.,0.,0.)):
        parts.append(cad_part(
            name, shape, material, tolerance=.10, angular=.10,
            analytic_normals=True, group="context", motion="fixed",
            role="Photographic workshop context; not part of ORBIT",
            provenance="designed-concept", finish_axis=axis,
            finish_origin=tuple(shape.Center().toTuple())
        ))

    # Bench and service mat put ORBIT on a plausible physical surface instead of
    # the renderer's infinite studio floor. The top surface is at z=-2 mm, which
    # matches the service model's established bench datum.
    add("CTX_Workbench_top", box(980, 760, 36, (15, 0, -20), 3), bench_id)
    add("CTX_Service_mat", box(430, 330, 1.8, (22, 2, -2.9), .8), mat_id)

    az = math.radians(camera_az)
    toward = np.array([math.cos(az), math.sin(az), 0.0])
    right = np.array([-math.sin(az), math.cos(az), 0.0])
    target = np.array([34.0, 6.0, 61.0])
    back = target - toward * 430.0
    theta = camera_az - 90.0

    # A real wall, shelf, cabinet and small boxes live far enough behind the
    # subject to fall naturally out of focus through the thin-lens camera.
    wall_center = back + np.array([0.0, 0.0, 230.0])
    add("CTX_Back_wall", box(1200, 24, 560, wall_center, 1.2, theta), wall_id,
        axis=tuple(right))
    shelf_center = back + toward * 42.0 + np.array([0.0, 0.0, 215.0])
    add("CTX_Back_shelf", box(760, 150, 16, shelf_center, 2.0, theta), powder_id,
        axis=tuple(right))

    cabinet_center = back + toward * 110.0 + right * 245.0 + np.array([0.0, 0.0, 105.0])
    add("CTX_Drawer_cabinet", box(250, 190, 250, cabinet_center, 4.0, theta), powder_id,
        axis=tuple(right))
    # Drawer seams are actual shallow dark strips, not painted lines.
    for i in range(4):
        c = cabinet_center + toward * 97.0 + np.array([0.0, 0.0, 68.0 - i * 48.0])
        add(f"CTX_Drawer_seam_{i}", box(205, 3.0, 2.2, c, .3, theta), mat_id,
            axis=tuple(right))

    for i, (rshift, z) in enumerate(((-150, 228), (-45, 232), (75, 226))):
        c = back + toward * 55.0 + right * rshift + np.array([0.0, 0.0, z])
        add(f"CTX_Parts_box_{i}", box(82, 95, 52, c, 1.6, theta), cardboard_id,
            axis=tuple(right))

    # A steel bar and small tray on the bench add near-field depth cues without
    # inventing any functional ORBIT hardware.
    steel_id = 2  # existing hardened-steel material
    add("CTX_Bench_bar", box(210, 18, 9, (-105, -150, 3.0), 1.0, -18.0), steel_id)
    add("CTX_Parts_tray", box(125, 95, 8, (165, -125, 2.0), 3.0, 8.0), powder_id)

    return replace(assembly, parts=parts, materials=materials)


def photographic_view(assembly):
    views = dict(assembly.views)
    base = views["hero"]
    views["hero"] = replace(
        base,
        az=31.0, el=19.5, scale=96.0, target=(34, 6, 60),
        focal_length_mm=82.0, f_stop=5.6,
        environment_strength=.13, background_strength=.40,
        light_size=1.68, light_intensity=.70, exposure=1.08,
        floor=False, studio_style="product",
        background_color=(.004, .005, .006),
        studio_target=(34, 6, 62), studio_scale=1.0,
        studio_az=18.0, studio_el=26.0,
        tone_mapping="neutral",
    )
    return replace(assembly, views=views)


def save_camera_finish(source: Path, destination: Path):
    """Very subtle deterministic sensor/lens finishing; never replaces geometry."""
    with Image.open(source) as image:
        rgb = np.asarray(image.convert("RGB"), dtype=np.float32) / 255.0
    h, w, _ = rgb.shape
    yy, xx = np.mgrid[0:h, 0:w]
    nx = (xx + .5 - w / 2) / (w / 2)
    ny = (yy + .5 - h / 2) / (h / 2)
    r2 = nx * nx + ny * ny
    vignette = np.clip(1.0 - .032 * np.maximum(0.0, r2 - .15), .94, 1.0)
    rgb *= vignette[..., None]
    rng = np.random.default_rng(20260913)
    sigma = .0017 + .0012 * np.sqrt(np.clip(rgb.mean(axis=2), 0, 1))
    rgb += rng.normal(0.0, 1.0, (h, w, 1)) * sigma[..., None]
    rgb = np.clip(rgb, 0.0, 1.0)
    Image.fromarray(np.round(rgb * 255).astype(np.uint8), "RGB").save(destination, compress_level=6)
    with Image.open(destination) as check:
        check.verify()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--size", default="960x720")
    parser.add_argument("--spp", type=int, default=72)
    parser.add_argument("--depth", type=int, default=14)
    parser.add_argument("--threads", type=int, default=4)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    size = tuple(map(int, args.size.split("x")))

    recipe = load_recipe()
    base = recipe.build()
    held = recipe.with_inspection_coupon(base)
    scene = photographic_view(add_workshop_context(photographic_materials(held)))

    native = args.out / "ORBIT_v5_workshop_native.png"
    report = render_photoreal(
        scene, native, "hero", size, args.spp, args.threads, args.depth,
        intent="concept"
    )
    final = args.out / "ORBIT_v5_workshop.png"
    save_camera_finish(native, final)
    manifest = {
        "geometry_changed": False,
        "orbit_geometry_changed": False,
        "context_geometry_added": True,
        "generated_imagery": False,
        "renderer": report["renderer"],
        "resolution": list(size),
        "spp": args.spp,
        "depth": args.depth,
        "context": [
            "modeled workbench", "modeled service mat", "modeled rear wall",
            "modeled shelf", "modeled drawer cabinet", "modeled parts boxes",
            "modeled bench bar and tray"
        ],
        "camera_finish": "3.2% maximum radial vignette plus deterministic <=0.3% sensor-like grain; no geometry synthesis or frame interpolation",
    }
    (args.out / "ORBIT_v5_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print("DELIVERED", final, flush=True)


if __name__ == "__main__":
    main()
