"""ORBIT v9: unchanged verified CAD, captured workshop light, scanned bench roughness.

This renderer does not use image generation. ORBIT geometry is loaded from the
Mechanism Lab recipe. Illumination is a CC0 photographic HDRI from Poly Haven.
The finite workbench is rendered geometry with a CC0 scanned roughness map.
Beauty, albedo, and shading-normal AOVs guide Intel Open Image Denoise.
"""
from __future__ import annotations

import argparse
import importlib.metadata
import importlib.util
import json
import math
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
BASE_PATH = ROOT / "tools" / "orbit_mitsuba_photo.py"


def load_base():
    spec = importlib.util.spec_from_file_location("orbit_v8_base", BASE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def poly_record(tree, map_name: str, resolution: str, preferred=("jpg", "png", "exr")):
    branch = tree.get(map_name)
    if not isinstance(branch, dict):
        raise KeyError(f"Poly Haven map {map_name!r} unavailable; keys={list(tree)}")
    level = branch.get(resolution)
    if not isinstance(level, dict):
        raise KeyError(f"Poly Haven map {map_name!r} lacks {resolution}; keys={list(branch)}")
    for fmt in preferred:
        record = level.get(fmt)
        if isinstance(record, dict) and record.get("url"):
            return record
    raise KeyError(f"No usable {map_name} file at {resolution}; keys={list(level)}")


def download_hdri(base, destination: Path, slug="small_workshop", resolution="2k"):
    tree = base.get_json(f"https://api.polyhaven.com/files/{slug}")
    branch = tree.get("hdri", {}).get(resolution, {})
    record = branch.get("hdr") or branch.get("exr")
    if not isinstance(record, dict) or not record.get("url"):
        raise RuntimeError(f"No {resolution} HDR/EXR for {slug}")
    digest = base.download_record(record, destination)
    return {
        "asset": slug,
        "resolution": resolution,
        "url": record["url"],
        "md5": digest,
        "license": "CC0",
        "provider": "Poly Haven",
    }


def download_bench_roughness(base, destination: Path, slug="blue_metal_plate", resolution="1k"):
    tree = base.get_json(f"https://api.polyhaven.com/files/{slug}")
    record = poly_record(tree, "Rough", resolution)
    digest = base.download_record(record, destination)
    return {
        "asset": slug,
        "map": "Rough",
        "resolution": resolution,
        "url": record["url"],
        "md5": digest,
        "license": "CC0",
        "provider": "Poly Haven",
    }


def save_checked(linear, path: Path, exposure: float):
    base = load_base()
    base.save_png(linear, path, exposure)
    with Image.open(path) as im:
        im.verify()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--size", default="1100x825")
    p.add_argument("--spp", type=int, default=256)
    p.add_argument("--max-depth", type=int, default=14)
    p.add_argument("--oidn", default="oidnDenoise")
    p.add_argument("--env-rotation", type=float, default=195.0)
    p.add_argument("--env-scale", type=float, default=.72)
    p.add_argument("--exposure", type=float, default=1.04)
    args = p.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    width, height = map(int, args.size.split("x"))

    import mitsuba as mi
    variants = mi.variants()
    variant = "llvm_ad_rgb" if "llvm_ad_rgb" in variants else "scalar_rgb"
    mi.set_variant(variant)

    base = load_base()
    recipe = base.load_recipe()
    assembly = base.tune_materials(recipe.with_inspection_coupon(recipe.build()))

    # Keep the machine large in frame while leaving enough of the captured room
    # visible that the viewer gets real-world scale and reflection context.
    target = np.array((34., 6., 56.), dtype=float)
    az, el = math.radians(31.), math.radians(22.0)
    focal = 72.0
    fstop = 16.0
    sensor = 36.0
    aspect = width / height
    sensor_h = sensor / aspect
    vfov = 2 * math.atan(sensor_h / (2 * focal))
    framing = 101.0
    distance = framing / max(1e-6, math.tan(vfov / 2))
    direction = np.array([
        math.cos(az) * math.cos(el),
        math.sin(az) * math.cos(el),
        math.sin(el),
    ])
    camera = target + direction * distance
    hfov = math.degrees(2 * math.atan(sensor / (2 * focal)))

    hdri_path = args.out / "small_workshop_2k.hdr"
    env_info = download_hdri(base, hdri_path)
    rough_path = args.out / "bench_rough_1k.jpg"
    bench_info = download_bench_roughness(base, rough_path)

    uv_scale = mi.ScalarTransform3f.scale([2.2, 1.7])
    bench_rough = {
        "type": "bitmap",
        "filename": str(rough_path),
        "raw": True,
        "filter_type": "bilinear",
        "wrap_mode": "repeat",
        "to_uv": uv_scale,
    }
    bench_bsdf = {
        "type": "principled",
        "base_color": {"type": "rgb", "value": [.105, .100, .094]},
        "metallic": 0.0,
        "roughness": bench_rough,
        "specular": .34,
        "clearcoat": .0,
    }

    scene = {
        "type": "scene",
        "integrator": {
            "type": "aov",
            "aovs": "albedo:albedo,normal:sh_normal",
            "beauty": {"type": "path", "max_depth": args.max_depth, "rr_depth": 5},
        },
        "sensor": {
            "type": "thinlens",
            "fov": hfov,
            "fov_axis": "x",
            "to_world": mi.ScalarTransform4f.look_at(
                origin=camera.tolist(), target=target.tolist(), up=[0, 0, 1]
            ),
            "focus_distance": float(distance),
            "aperture_radius": float(focal / (2 * fstop)),
            "sampler": {"type": "independent", "sample_count": args.spp},
            "film": {
                "type": "hdrfilm",
                "width": width,
                "height": height,
                "component_format": "float32",
                "rfilter": {"type": "gaussian", "stddev": .42},
            },
        },
        "environment": {
            "type": "envmap",
            "filename": str(hdri_path),
            "scale": args.env_scale,
            "to_world": mi.ScalarTransform4f.rotate([0, 0, 1], args.env_rotation),
            "mis_compensation": True,
        },
        # A finite physical top surface only. Its boundaries reveal the captured
        # room instead of becoming an infinite synthetic horizon.
        "bench": {
            "type": "rectangle",
            "to_world": mi.ScalarTransform4f.translate([28, 0, -2.04])
                        @ mi.ScalarTransform4f.scale([185, 135, 1]),
            "bsdf": bench_bsdf,
        },
    }

    mesh_dir = args.out / "mesh"
    mesh_dir.mkdir(exist_ok=True)
    triangle_count = 0
    for i, part in enumerate(assembly.parts):
        vertices, normals, faces = base.transformed_mesh(assembly, part)
        triangle_count += len(faces)
        path = mesh_dir / f"{i:03d}_{part.name}.ply"
        base.write_binary_ply(path, vertices, normals, faces)
        scene[f"part_{i:03d}"] = {
            "type": "ply",
            "filename": str(path),
            "face_normals": False,
            "bsdf": base.principled(assembly.materials[part.material], part.name),
        }

    loaded = mi.load_dict(scene)
    reported_names = list(loaded.integrator().aov_names())
    rendered = np.asarray(mi.render(loaded, spp=args.spp), dtype=np.float32)
    names = base.reconcile_aov_names(reported_names, rendered)
    beauty = base.extract_channels(rendered, names, "beauty")
    albedo = np.clip(base.extract_channels(rendered, names, "albedo"), 0, 1)
    normal = np.clip(base.extract_channels(rendered, names, "normal"), -1, 1)
    if not np.isfinite(beauty).all() or float(beauty.min()) < 0:
        raise RuntimeError("Invalid Mitsuba radiance")

    beauty_pfm = args.out / "ORBIT_v9_beauty_linear.pfm"
    albedo_pfm = args.out / "ORBIT_v9_albedo.pfm"
    normal_pfm = args.out / "ORBIT_v9_normal.pfm"
    denoised_pfm = args.out / "ORBIT_v9_denoised_linear.pfm"
    base.write_pfm(beauty_pfm, beauty)
    base.write_pfm(albedo_pfm, albedo)
    base.write_pfm(normal_pfm, normal)
    subprocess.run([
        args.oidn, "-d", "cpu", "--hdr", str(beauty_pfm),
        "--alb", str(albedo_pfm), "--nrm", str(normal_pfm),
        "-q", "high", "-o", str(denoised_pfm),
    ], check=True)
    denoised = base.read_pfm(denoised_pfm)
    if denoised.shape != beauty.shape or not np.isfinite(denoised).all():
        raise RuntimeError("Invalid OIDN output")

    noisy = args.out / "ORBIT_v9_noisy.png"
    final = args.out / "ORBIT_v9_pbr_workshop.png"
    base.save_png(beauty, noisy, args.exposure)
    base.save_png(denoised, final, args.exposure)

    manifest = {
        "model": "CYBR ORBIT revision 3",
        "orbit_geometry_changed": False,
        "generated_imagery": False,
        "renderer": "Mitsuba 3 path + albedo/normal-guided Intel OIDN",
        "mitsuba_version": importlib.metadata.version("mitsuba"),
        "variant": variant,
        "resolution": [width, height],
        "spp": args.spp,
        "max_depth": args.max_depth,
        "triangles": triangle_count,
        "parts": len(assembly.parts),
        "camera": {
            "focal_length_mm": focal,
            "f_stop": fstop,
            "horizontal_fov_degrees": hfov,
            "camera_distance_scene_mm": distance,
        },
        "environment": env_info | {
            "rotation_degrees": args.env_rotation,
            "scale": args.env_scale,
        },
        "bench_surface": bench_info | {
            "geometry": "finite 370 x 270 mm rendered rectangle",
            "base_color": [.105, .100, .094],
        },
        "post": "Guided OIDN on linear HDR radiance, ACES fitted tone curve, sRGB transfer only; no compositing, image synthesis, frame interpolation, or sharpening",
        "credit": "Powered by Poly Haven live API; referenced assets are CC0.",
        "aov_channels": names,
    }
    (args.out / "ORBIT_v9_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print("DELIVERED", final, flush=True)


if __name__ == "__main__":
    main()
