"""Render validated REACH-2 + ORBIT with the ORBIT v9 physical architecture.

No image generation. Rendering is gated by tools/reach2_engineering.py.
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

ROOT = Path(__file__).resolve().parents[1]
RECIPE = ROOT / "examples" / "reach2_engineered_elbow.py"
ENGINEERING = ROOT / "tools" / "reach2_engineering.py"
V9 = ROOT / "tools" / "orbit_mitsuba_photo_v9.py"


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def transformed_mesh_at(assembly, part, t):
    T = np.asarray(assembly.pose(part, t, 0.0), dtype=np.float64)
    v = np.asarray(part.vertices, dtype=np.float64)
    vertices = (np.c_[v, np.ones(len(v))] @ T.T)[:, :3].astype("<f4")
    normals = np.asarray(part.normals, dtype=np.float64) @ T[:3, :3].T
    normals /= np.maximum(np.linalg.norm(normals, axis=1)[:, None], 1e-12)
    return vertices, normals.astype("<f4"), np.asarray(part.faces, dtype="<i4")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--size", default="1200x900")
    p.add_argument("--spp", type=int, default=256)
    p.add_argument("--max-depth", type=int, default=14)
    p.add_argument("--pose-time", type=float, default=1.6)
    p.add_argument("--oidn", default="oidnDenoise")
    p.add_argument("--env-rotation", type=float, default=202.0)
    p.add_argument("--env-scale", type=float, default=.76)
    p.add_argument("--exposure", type=float, default=1.05)
    args = p.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    width, height = map(int, args.size.split("x"))

    engineering = load(ENGINEERING, "reach2_engineering_render_gate")
    report = engineering.qualify()
    if not report["qualified"]:
        failed = [c["name"] for c in report["checks"] if not c["passed"]]
        raise RuntimeError("Refusing to render unqualified REACH-2: " + ", ".join(failed))
    (args.out / "REACH2_engineering_report.json").write_text(json.dumps(report, indent=2) + "\n")

    import mitsuba as mi
    variant = "llvm_ad_rgb" if "llvm_ad_rgb" in mi.variants() else "scalar_rgb"
    mi.set_variant(variant)

    v9 = load(V9, "reach2_v9_pipeline")
    base = v9.load_base()
    recipe = load(RECIPE, "reach2_recipe_render")
    assembly = base.tune_materials(recipe.build())

    # Determine posed world bounds from actual tessellated vertices so framing
    # remains correct if structural dimensions change.
    posed = []
    for part in assembly.parts:
        vv, _, _ = transformed_mesh_at(assembly, part, args.pose_time)
        if len(vv):
            posed.append(vv)
    allv = np.concatenate(posed, axis=0)
    bmin, bmax = allv.min(axis=0), allv.max(axis=0)
    target = (bmin + bmax) * .5
    target[2] += 5.0

    # Favor the output side enough to show both ORBIT jaws while retaining the
    # coaxial reducer and motor package in the same frame.
    az, el = math.radians(29.0), math.radians(16.0)
    focal = 72.0
    fstop = 16.0
    sensor = 36.0
    aspect = width / height
    sensor_h = sensor / aspect
    vfov = 2 * math.atan(sensor_h / (2 * focal))
    extent_xz = max(bmax[0] - bmin[0], bmax[2] - bmin[2])
    framing = max(180.0, float(extent_xz) * .68)
    distance = framing / max(1e-6, math.tan(vfov / 2))
    direction = np.array([math.cos(az) * math.cos(el), math.sin(az) * math.cos(el), math.sin(el)])
    camera = target + direction * distance
    hfov = math.degrees(2 * math.atan(sensor / (2 * focal)))

    hdri_path = args.out / "small_workshop_2k.hdr"
    env_info = v9.download_hdri(base, hdri_path, slug="small_workshop", resolution="2k")
    rough_path = args.out / "bench_rough_1k.jpg"
    bench_info = v9.download_bench_roughness(base, rough_path, slug="blue_metal_plate", resolution="1k")

    bench_z = float(bmin[2]) - 1.0
    bench_bsdf = {
        "type": "principled",
        "base_color": {"type": "rgb", "value": [.100, .096, .090]},
        "metallic": 0.0,
        "roughness": {
            "type": "bitmap", "filename": str(rough_path), "raw": True,
            "filter_type": "bilinear", "wrap_mode": "repeat",
            "to_uv": mi.ScalarTransform3f.scale([2.6, 2.0]),
        },
        "specular": .33,
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
            "fov": hfov, "fov_axis": "x",
            "to_world": mi.ScalarTransform4f.look_at(origin=camera.tolist(), target=target.tolist(), up=[0, 0, 1]),
            "focus_distance": float(distance),
            "aperture_radius": float(focal / (2 * fstop)),
            "sampler": {"type": "independent", "sample_count": args.spp},
            "film": {
                "type": "hdrfilm", "width": width, "height": height,
                "component_format": "float32",
                "rfilter": {"type": "gaussian", "stddev": .42},
            },
        },
        "environment": {
            "type": "envmap", "filename": str(hdri_path), "scale": args.env_scale,
            "to_world": mi.ScalarTransform4f.rotate([0, 0, 1], args.env_rotation),
            "mis_compensation": True,
        },
        "bench": {
            "type": "rectangle",
            "to_world": mi.ScalarTransform4f.translate([target[0], target[1], bench_z])
                        @ mi.ScalarTransform4f.scale([270, 210, 1]),
            "bsdf": bench_bsdf,
        },
    }

    mesh_dir = args.out / "mesh"
    mesh_dir.mkdir(exist_ok=True)
    triangles = 0
    for i, part in enumerate(assembly.parts):
        vertices, normals, faces = transformed_mesh_at(assembly, part, args.pose_time)
        triangles += len(faces)
        path = mesh_dir / f"{i:03d}_{part.name}.ply"
        base.write_binary_ply(path, vertices, normals, faces)
        scene[f"part_{i:03d}"] = {
            "type": "ply", "filename": str(path), "face_normals": False,
            "bsdf": base.principled(assembly.materials[part.material], part.name),
        }

    loaded = mi.load_dict(scene)
    reported = list(loaded.integrator().aov_names())
    rendered = np.asarray(mi.render(loaded, spp=args.spp), dtype=np.float32)
    names = base.reconcile_aov_names(reported, rendered)
    beauty = base.extract_channels(rendered, names, "beauty")
    albedo = np.clip(base.extract_channels(rendered, names, "albedo"), 0, 1)
    normal = np.clip(base.extract_channels(rendered, names, "normal"), -1, 1)
    if not np.isfinite(beauty).all() or float(beauty.min()) < 0:
        raise RuntimeError("Invalid Mitsuba radiance")

    beauty_pfm = args.out / "REACH2_beauty_linear.pfm"
    albedo_pfm = args.out / "REACH2_albedo.pfm"
    normal_pfm = args.out / "REACH2_normal.pfm"
    denoised_pfm = args.out / "REACH2_denoised_linear.pfm"
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

    noisy = args.out / "REACH2_ORBIT_noisy.png"
    final = args.out / "REACH2_ORBIT_hero.png"
    base.save_png(beauty, noisy, args.exposure)
    base.save_png(denoised, final, args.exposure)

    manifest = {
        "model": assembly.name,
        "generated_imagery": False,
        "engineering_qualified": report["qualified"],
        "engineering_report": "REACH2_engineering_report.json",
        "geometry_source": "examples/reach2_engineered_elbow.py + unchanged ORBIT recipe",
        "renderer": "ORBIT v9 architecture: Mitsuba 3 path + albedo/normal-guided Intel OIDN",
        "mitsuba_version": importlib.metadata.version("mitsuba"),
        "variant": variant,
        "resolution": [width, height], "spp": args.spp, "max_depth": args.max_depth,
        "pose_time": args.pose_time,
        "elbow_angle_degrees": math.degrees(recipe.elbow_angle(args.pose_time)),
        "parts": len(assembly.parts), "triangles": triangles,
        "reducer": assembly.metadata["selected_reducer"],
        "reducer_ratio": recipe.RATIO,
        "load_case": report["load_case"],
        "camera": {"focal_length_mm": focal, "f_stop": fstop, "horizontal_fov_degrees": hfov},
        "environment": env_info | {"rotation_degrees": args.env_rotation, "scale": args.env_scale},
        "bench_surface": bench_info,
        "post": "Guided OIDN on linear HDR radiance, ACES fitted tone curve and sRGB transfer only; no image generation, compositing, interpolation or sharpening",
        "aov_channels": names,
    }
    (args.out / "REACH2_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print("DELIVERED", final, flush=True)


if __name__ == "__main__":
    main()
