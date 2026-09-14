"""Render the real ORBIT CAD through a Mitsuba path-tracing backend.

This is a rendering-backend study for CYBR GEO. Geometry and kinematics come
straight from the ORBIT Mechanism Lab recipe. The environment is a CC0 Poly
Haven HDRI, not generated imagery. No image-to-image synthesis is used.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import importlib.util
import json
import math
import struct
import urllib.request
from dataclasses import replace
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
RECIPE = ROOT / "examples" / "orbit_inspection_wrist.py"
POLYHAVEN_FILES = "https://api.polyhaven.com/files/workshop"


def load_recipe():
    spec = importlib.util.spec_from_file_location("orbit_mitsuba_recipe", RECIPE)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def tune_materials(assembly):
    tuned=[]
    for m in assembly.materials:
        name=m.name.lower()
        if "graphite anodized" in name:
            m=replace(m,color=(.038,.047,.058),metal=.72,rough=.39,coat=.22,coat_rough=.26,anisotropy=.05)
        elif "teal anodized" in name:
            m=replace(m,color=(.010,.125,.105),metal=.70,rough=.37,coat=.26,coat_rough=.24,anisotropy=.05)
        elif "satin machined aluminium" in name:
            m=replace(m,color=(.60,.63,.66),metal=1.0,rough=.31,coat=.01,anisotropy=.55)
        elif "hardened steel" in name:
            m=replace(m,color=(.43,.46,.49),metal=1.0,rough=.27,coat=.01,anisotropy=.30)
        elif "warm bronze" in name:
            m=replace(m,color=(.50,.29,.105),metal=1.0,rough=.30,coat=.01,anisotropy=.22)
        elif "black elastomer" in name:
            m=replace(m,color=(.006,.007,.009),metal=0.0,rough=.82,coat=0.0,anisotropy=0.0)
        elif "ceramic white" in name:
            m=replace(m,color=(.68,.71,.74),metal=0.0,rough=.43,coat=.02,anisotropy=0.0)
        elif "connector copper" in name:
            m=replace(m,color=(.55,.22,.078),metal=.98,rough=.25,coat=.01,anisotropy=.25)
        tuned.append(m)
    return replace(assembly,materials=tuned)


def download_hdri(destination: Path, resolution="2k"):
    request=urllib.request.Request(POLYHAVEN_FILES,headers={"User-Agent":"CYBR-GEO/0.6 photoreal-backend-study"})
    with urllib.request.urlopen(request,timeout=30) as response:
        files=json.load(response)
    hdri=files["hdri"][resolution]
    record=hdri.get("hdr") or hdri.get("exr")
    if not record:
        raise RuntimeError(f"No HDR/EXR record for Poly Haven workshop {resolution}")
    url=record["url"]
    urllib.request.urlretrieve(url,destination)
    digest=hashlib.md5(destination.read_bytes()).hexdigest()
    expected=record.get("md5")
    if expected and digest.lower()!=expected.lower():
        raise RuntimeError("Poly Haven HDRI checksum mismatch")
    return {"asset":"workshop","resolution":resolution,"url":url,"md5":digest,"license":"CC0","provider":"Poly Haven"}


def transformed_mesh(assembly,part):
    T=np.asarray(assembly.pose(part,0.0,0.0),dtype=np.float64)
    v=np.asarray(part.vertices,dtype=np.float64)
    h=np.c_[v,np.ones(len(v))]
    vertices=(h@T.T)[:,:3].astype("<f4")
    R=T[:3,:3]
    normals=np.asarray(part.normals,dtype=np.float64)@R.T
    lengths=np.linalg.norm(normals,axis=1)
    normals=(normals/np.maximum(lengths[:,None],1e-12)).astype("<f4")
    faces=np.asarray(part.faces,dtype="<i4")
    return vertices,normals,faces


def write_binary_ply(path: Path, vertices, normals, faces):
    path.parent.mkdir(parents=True,exist_ok=True)
    header=(
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {len(vertices)}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property float nx\nproperty float ny\nproperty float nz\n"
        f"element face {len(faces)}\n"
        "property list uchar int vertex_indices\nend_header\n"
    ).encode("ascii")
    data=np.empty((len(vertices),6),dtype="<f4")
    data[:,:3]=vertices;data[:,3:]=normals
    with path.open("wb") as f:
        f.write(header);f.write(data.tobytes(order="C"))
        # 13 bytes per triangle: count byte plus 3 little-endian int32 indices.
        for tri in faces:
            f.write(struct.pack("<Biii",3,int(tri[0]),int(tri[1]),int(tri[2])))


def part_variation(name: str):
    raw=hashlib.sha256(name.encode()).digest()
    a=int.from_bytes(raw[:4],"little")/2**32
    b=int.from_bytes(raw[4:8],"little")/2**32
    return .94+.12*a,.985+.03*b


def principled(material,part_name):
    rough_mul,color_mul=part_variation(part_name)
    color=[float(np.clip(c*color_mul,0,1)) for c in material.color]
    rough=float(np.clip(material.rough*rough_mul,.045,.96))
    clearcoat=float(np.clip(material.coat,0,1))
    # Mitsuba's clearcoat_gloss is inverse-ish in spirit: 1 is glossier.
    clearcoat_gloss=float(np.clip(1.0-material.coat_rough,0,1))
    return {
        "type":"principled",
        "base_color":{"type":"rgb","value":color},
        "metallic":float(np.clip(material.metal,0,1)),
        "roughness":rough,
        "anisotropic":float(np.clip(abs(material.anisotropy)*.55,0,.8)),
        "clearcoat":clearcoat,
        "clearcoat_gloss":clearcoat_gloss,
        "specular":.48,
    }


def aces_tonemap(linear,exposure=1.0):
    x=np.maximum(0.0,np.asarray(linear,dtype=np.float32)*exposure)
    a,b,c,d,e=2.51,.03,2.43,.59,.14
    y=np.clip((x*(a*x+b))/(x*(c*x+d)+e),0,1)
    # Convert linear display RGB to sRGB.
    y=np.where(y<=.0031308,12.92*y,1.055*np.power(y,1/2.4)-.055)
    return np.clip(y,0,1)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out",type=Path,required=True)
    p.add_argument("--size",default="900x675")
    p.add_argument("--spp",type=int,default=96)
    p.add_argument("--max-depth",type=int,default=14)
    p.add_argument("--hdri-resolution",default="2k",choices=["1k","2k","4k"])
    p.add_argument("--env-rotation",type=float,default=205.0)
    p.add_argument("--env-scale",type=float,default=.80)
    p.add_argument("--exposure",type=float,default=1.08)
    args=p.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    width,height=map(int,args.size.split("x"))

    import mitsuba as mi
    variants=mi.variants()
    variant="llvm_ad_rgb" if "llvm_ad_rgb" in variants else "scalar_rgb"
    mi.set_variant(variant)

    recipe=load_recipe()
    assembly=tune_materials(recipe.with_inspection_coupon(recipe.build()))
    view=assembly.views["hero"]
    target=np.array((34.,6.,61.),dtype=float)
    az,el=math.radians(31.),math.radians(22.)
    # Reuse CYBR GEO's framing semantics, but move to a slightly longer lens.
    focal=85.0;sensor=36.0
    aspect=width/height;sensor_h=sensor/aspect
    vfov=2*math.atan(sensor_h/(2*focal))
    distance=96.0/max(1e-6,math.tan(vfov/2))
    direction=np.array([math.cos(az)*math.cos(el),math.sin(az)*math.cos(el),math.sin(el)])
    camera=target+direction*distance
    hfov=math.degrees(2*math.atan(sensor/(2*focal)))

    hdri_path=args.out/("workshop_"+args.hdri_resolution+".hdr")
    hdri_info=download_hdri(hdri_path,args.hdri_resolution)

    scene={
        "type":"scene",
        "integrator":{"type":"path","max_depth":args.max_depth,"rr_depth":5},
        "sensor":{
            "type":"thinlens",
            "fov":hfov,"fov_axis":"x",
            "to_world":mi.ScalarTransform4f.look_at(origin=camera.tolist(),target=target.tolist(),up=[0,0,1]),
            "focus_distance":float(distance),
            "aperture_radius":float(focal/(2*7.1)),
            "sampler":{"type":"independent","sample_count":args.spp},
            "film":{"type":"hdrfilm","width":width,"height":height,"pixel_format":"rgb",
                    "component_format":"float32","rfilter":{"type":"gaussian","stddev":.45}},
        },
        "environment":{
            "type":"envmap","filename":str(hdri_path),"scale":args.env_scale,
            "to_world":mi.ScalarTransform4f.rotate([0,0,1],args.env_rotation),
        },
        "ground":{
            "type":"rectangle",
            "to_world":mi.ScalarTransform4f.translate([0,0,-2.02]) @ mi.ScalarTransform4f.scale([650,650,1]),
            "bsdf":{"type":"principled","base_color":{"type":"rgb","value":[.075,.070,.062]},
                    "metallic":0.0,"roughness":.58,"specular":.42,"clearcoat":.03,"clearcoat_gloss":.25},
        },
    }

    mesh_dir=args.out/"mesh";mesh_dir.mkdir(exist_ok=True)
    triangle_count=0
    for i,part in enumerate(assembly.parts):
        vertices,normals,faces=transformed_mesh(assembly,part)
        triangle_count+=len(faces)
        path=mesh_dir/f"{i:03d}_{part.name}.ply"
        write_binary_ply(path,vertices,normals,faces)
        scene[f"part_{i:03d}"]={
            "type":"ply","filename":str(path),
            "face_normals":False,
            "bsdf":principled(assembly.materials[part.material],part.name),
        }

    loaded=mi.load_dict(scene)
    image=mi.render(loaded,spp=args.spp)
    linear=np.asarray(image,dtype=np.float32)
    if not np.isfinite(linear).all():
        raise RuntimeError("Non-finite Mitsuba radiance")
    ldr=aces_tonemap(linear,args.exposure)
    final=args.out/"ORBIT_v6_mitsuba_workshop.png"
    Image.fromarray(np.round(ldr*255).astype(np.uint8),"RGB").save(final,compress_level=6)
    with Image.open(final) as check:check.verify()

    manifest={
        "model":"CYBR ORBIT revision 3",
        "geometry_source":"CYBR GEO Mechanism Lab ORBIT analytic/tessellated assembly",
        "orbit_geometry_changed":False,
        "generated_imagery":False,
        "renderer_backend":"Mitsuba 3 path integrator proof backend for CYBR GEO",
        "mitsuba_version":importlib.metadata.version("mitsuba"),
        "mitsuba_variant":variant,
        "resolution":[width,height],"spp":args.spp,"max_depth":args.max_depth,
        "triangles":triangle_count,"parts":len(assembly.parts),
        "camera":{"focal_length_mm":focal,"f_stop":7.1,"camera_distance_scene_mm":distance,"horizontal_fov_degrees":hfov},
        "environment":hdri_info|{"rotation_degrees":args.env_rotation,"scale":args.env_scale},
        "environment_credit":"Powered by Poly Haven (live API); Workshop HDRI is CC0.",
        "post":"ACES fitted tone curve + sRGB transfer only; no denoiser, compositing, image synthesis, or sharpening",
    }
    (args.out/"ORBIT_v6_manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    print("DELIVERED",final,flush=True)


if __name__=="__main__":
    main()
