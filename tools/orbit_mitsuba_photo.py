"""Render verified ORBIT CAD with captured lighting and guided denoising.

No image generation is used. Geometry and kinematics come directly from the
verified ORBIT Mechanism Lab recipe. Lighting is a CC0 Poly Haven workshop HDRI.
The support bench is real render geometry. Intel OIDN receives linear HDR beauty,
albedo, and shading-normal AOVs from Mitsuba so it can preserve real CAD edges
instead of smearing them.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import importlib.util
import json
import math
import struct
import subprocess
import urllib.request
from dataclasses import replace
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
RECIPE = ROOT / "examples" / "orbit_inspection_wrist.py"
POLYHAVEN_FILES = "https://api.polyhaven.com/files/workshop"
USER_AGENT = "CYBR-GEO/0.8 photoreal-backend-study"


def load_recipe():
    spec = importlib.util.spec_from_file_location("orbit_mitsuba_recipe", RECIPE)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def tune_materials(assembly):
    """Move the showcase palette toward physically plausible manufactured finishes."""
    tuned=[]
    for m in assembly.materials:
        name=m.name.lower()
        if "graphite anodized" in name:
            m=replace(m,color=(.028,.034,.041),metal=.12,rough=.34,
                      ior=1.55,coat=.16,coat_rough=.27,anisotropy=.06)
        elif "teal anodized" in name:
            m=replace(m,color=(.008,.115,.095),metal=.10,rough=.33,
                      ior=1.55,coat=.18,coat_rough=.25,anisotropy=.06)
        elif "satin machined aluminium" in name:
            m=replace(m,color=(.70,.72,.735),metal=1.0,rough=.27,
                      coat=.01,coat_rough=.24,anisotropy=.56)
        elif "hardened steel" in name:
            m=replace(m,color=(.49,.505,.52),metal=1.0,rough=.25,
                      coat=.01,coat_rough=.20,anisotropy=.31)
        elif "warm bronze" in name:
            m=replace(m,color=(.58,.32,.11),metal=1.0,rough=.30,
                      coat=.01,coat_rough=.22,anisotropy=.22)
        elif "black elastomer" in name:
            m=replace(m,color=(.0045,.0055,.007),metal=0.0,rough=.86,
                      ior=1.47,coat=0.0,anisotropy=0.0)
        elif "ceramic white" in name:
            m=replace(m,color=(.70,.72,.735),metal=0.0,rough=.42,
                      ior=1.51,coat=.02,coat_rough=.30,anisotropy=0.0)
        elif "connector copper" in name:
            m=replace(m,color=(.62,.245,.085),metal=.99,rough=.24,
                      coat=.01,coat_rough=.20,anisotropy=.26)
        tuned.append(m)
    return replace(assembly,materials=tuned)


def get_json(url):
    request=urllib.request.Request(url,headers={"User-Agent":USER_AGENT})
    with urllib.request.urlopen(request,timeout=45) as response:
        return json.load(response)


def download_record(record,destination: Path):
    urllib.request.urlretrieve(record["url"],destination)
    digest=hashlib.md5(destination.read_bytes()).hexdigest()
    expected=record.get("md5")
    if expected and digest.lower()!=expected.lower():
        raise RuntimeError(f"Checksum mismatch for {destination.name}")
    return digest


def download_hdri(destination: Path, resolution="2k"):
    files=get_json(POLYHAVEN_FILES)
    hdri=files["hdri"][resolution]
    record=hdri.get("hdr") or hdri.get("exr")
    if not record:
        raise RuntimeError(f"No HDR/EXR record for Poly Haven workshop {resolution}")
    digest=download_record(record,destination)
    return {"asset":"workshop","resolution":resolution,"url":record["url"],
            "md5":digest,"license":"CC0","provider":"Poly Haven"}


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


def write_binary_ply(path: Path,vertices,normals,faces):
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
        for tri in faces:
            f.write(struct.pack("<Biii",3,int(tri[0]),int(tri[1]),int(tri[2])))


def part_variation(name:str):
    raw=hashlib.sha256(name.encode()).digest()
    a=int.from_bytes(raw[:4],"little")/2**32
    b=int.from_bytes(raw[4:8],"little")/2**32
    return .93+.14*a,.985+.03*b


def principled(material,part_name):
    rough_mul,color_mul=part_variation(part_name)
    color=[float(np.clip(c*color_mul,0,1)) for c in material.color]
    rough=float(np.clip(material.rough*rough_mul,.055,.96))
    return {
        "type":"principled",
        "base_color":{"type":"rgb","value":color},
        "metallic":float(np.clip(material.metal,0,1)),
        "roughness":rough,
        "anisotropic":float(np.clip(abs(material.anisotropy)*.50,0,.75)),
        "clearcoat":float(np.clip(material.coat,0,1)),
        "clearcoat_gloss":float(np.clip(1.0-material.coat_rough,0,1)),
        "specular":.47,
    }


def write_pfm(path:Path,image):
    arr=np.asarray(image,dtype="<f4")
    if arr.ndim!=3 or arr.shape[2]!=3:
        raise ValueError("PFM requires HxWx3 RGB")
    h,w,_=arr.shape
    with path.open("wb") as f:
        f.write(f"PF\n{w} {h}\n-1.0\n".encode("ascii"))
        f.write(np.flipud(arr).tobytes(order="C"))


def read_pfm(path:Path):
    with path.open("rb") as f:
        if f.readline().strip()!=b"PF":raise ValueError("Expected RGB PFM")
        w,h=map(int,f.readline().split());scale=float(f.readline())
        dtype="<f4" if scale<0 else ">f4"
        arr=np.fromfile(f,dtype=dtype,count=w*h*3).reshape(h,w,3)
    return np.flipud(arr).astype(np.float32,copy=False)


def aces_tonemap(linear,exposure=1.0):
    x=np.maximum(0.0,np.asarray(linear,dtype=np.float32)*exposure)
    a,b,c,d,e=2.51,.03,2.43,.59,.14
    y=np.clip((x*(a*x+b))/(x*(c*x+d)+e),0,1)
    y=np.where(y<=.0031308,12.92*y,1.055*np.power(y,1/2.4)-.055)
    return np.clip(y,0,1)


def save_png(linear,path,exposure):
    ldr=aces_tonemap(linear,exposure)
    Image.fromarray(np.round(ldr*255).astype(np.uint8),"RGB").save(path,compress_level=6)
    with Image.open(path) as check:check.verify()


def extract_channels(rendered,names,prefix):
    indices=[i for i,n in enumerate(names) if n.startswith(prefix+".")]
    if len(indices)<3:
        raise RuntimeError(f"Missing {prefix} RGB/XYZ channels: {names}")
    return rendered[...,indices[:3]].astype(np.float32,copy=False)


def reconcile_aov_names(names,rendered):
    """Mitsuba 3.7 reports beauty.A even when numpy output omits that alpha plane."""
    if rendered.ndim!=3:
        raise RuntimeError(f"Unexpected AOV tensor shape {rendered.shape}")
    if rendered.shape[2]==len(names):
        return names
    if rendered.shape[2]==len(names)-1 and "beauty.A" in names:
        compact=[n for n in names if n!="beauty.A"]
        if len(compact)==rendered.shape[2]:
            return compact
    raise RuntimeError(f"Unexpected AOV tensor shape {rendered.shape} for {names}")


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out",type=Path,required=True)
    p.add_argument("--size",default="1000x750")
    p.add_argument("--spp",type=int,default=192)
    p.add_argument("--max-depth",type=int,default=14)
    p.add_argument("--hdri-resolution",default="2k",choices=["1k","2k","4k"])
    p.add_argument("--env-rotation",type=float,default=205.0)
    p.add_argument("--env-scale",type=float,default=.82)
    p.add_argument("--exposure",type=float,default=1.16)
    p.add_argument("--oidn",default="oidnDenoise")
    args=p.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    width,height=map(int,args.size.split("x"))

    import mitsuba as mi
    variants=mi.variants();variant="llvm_ad_rgb" if "llvm_ad_rgb" in variants else "scalar_rgb"
    mi.set_variant(variant)

    recipe=load_recipe()
    assembly=tune_materials(recipe.with_inspection_coupon(recipe.build()))

    target=np.array((34.,6.,54.),dtype=float)
    az,el=math.radians(31.),math.radians(21.5)
    focal=72.0;fstop=13.0;sensor=36.0
    aspect=width/height;sensor_h=sensor/aspect
    vfov=2*math.atan(sensor_h/(2*focal))
    framing=108.0
    distance=framing/max(1e-6,math.tan(vfov/2))
    direction=np.array([math.cos(az)*math.cos(el),math.sin(az)*math.cos(el),math.sin(el)])
    camera=target+direction*distance
    hfov=math.degrees(2*math.atan(sensor/(2*focal)))

    hdri_path=args.out/("workshop_"+args.hdri_resolution+".hdr")
    hdri_info=download_hdri(hdri_path,args.hdri_resolution)

    bench_bsdf={
        "type":"principled",
        "base_color":{"type":"rgb","value":[.085,.080,.073]},
        "metallic":0.0,"roughness":.91,"specular":.30,"clearcoat":0.0,
    }

    scene={
        "type":"scene",
        "integrator":{
            "type":"aov","aovs":"albedo:albedo,normal:sh_normal",
            "beauty":{"type":"path","max_depth":args.max_depth,"rr_depth":5},
        },
        "sensor":{
            "type":"thinlens","fov":hfov,"fov_axis":"x",
            "to_world":mi.ScalarTransform4f.look_at(origin=camera.tolist(),target=target.tolist(),up=[0,0,1]),
            "focus_distance":float(distance),"aperture_radius":float(focal/(2*fstop)),
            "sampler":{"type":"independent","sample_count":args.spp},
            "film":{"type":"hdrfilm","width":width,"height":height,
                    "component_format":"float32","rfilter":{"type":"box"}},
        },
        "environment":{
            "type":"envmap","filename":str(hdri_path),"scale":args.env_scale,
            "to_world":mi.ScalarTransform4f.rotate([0,0,1],args.env_rotation),
            "mis_compensation":True,
        },
        "bench":{
            "type":"cube",
            "to_world":mi.ScalarTransform4f.translate([28,0,-14.0]) @ mi.ScalarTransform4f.scale([245,180,12]),
            "bsdf":bench_bsdf,
        },
    }

    mesh_dir=args.out/"mesh";mesh_dir.mkdir(exist_ok=True)
    triangle_count=0
    for i,part in enumerate(assembly.parts):
        vertices,normals,faces=transformed_mesh(assembly,part);triangle_count+=len(faces)
        path=mesh_dir/f"{i:03d}_{part.name}.ply";write_binary_ply(path,vertices,normals,faces)
        scene[f"part_{i:03d}"]={
            "type":"ply","filename":str(path),"face_normals":False,
            "bsdf":principled(assembly.materials[part.material],part.name),
        }

    loaded=mi.load_dict(scene)
    names=list(loaded.integrator().aov_names())
    rendered=np.asarray(mi.render(loaded,spp=args.spp),dtype=np.float32)
    print("AOV_CHANNELS",names,flush=True)
    print("AOV_SHAPE",rendered.shape,flush=True)
    names=reconcile_aov_names(names,rendered)
    print("AOV_EFFECTIVE_CHANNELS",names,flush=True)

    beauty=extract_channels(rendered,names,"beauty")
    albedo=np.clip(extract_channels(rendered,names,"albedo"),0,1)
    normal=np.clip(extract_channels(rendered,names,"normal"),-1,1)
    if not np.isfinite(beauty).all() or np.min(beauty)<0:
        raise RuntimeError("Invalid Mitsuba radiance")

    beauty_pfm=args.out/"ORBIT_v8_beauty_linear.pfm"
    albedo_pfm=args.out/"ORBIT_v8_albedo.pfm"
    normal_pfm=args.out/"ORBIT_v8_normal.pfm"
    denoised_pfm=args.out/"ORBIT_v8_denoised_linear.pfm"
    write_pfm(beauty_pfm,beauty);write_pfm(albedo_pfm,albedo);write_pfm(normal_pfm,normal)
    subprocess.run([
        args.oidn,"-d","cpu","--hdr",str(beauty_pfm),
        "--alb",str(albedo_pfm),"--nrm",str(normal_pfm),
        "-q","high","-o",str(denoised_pfm)
    ],check=True)
    denoised=read_pfm(denoised_pfm)
    if denoised.shape!=beauty.shape or not np.isfinite(denoised).all():
        raise RuntimeError("Invalid OIDN output")

    noisy_png=args.out/"ORBIT_v8_noisy.png"
    final=args.out/"ORBIT_v8_guided_workshop.png"
    save_png(beauty,noisy_png,args.exposure);save_png(denoised,final,args.exposure)

    manifest={
        "model":"CYBR ORBIT revision 3",
        "geometry_source":"CYBR GEO Mechanism Lab ORBIT assembly",
        "orbit_geometry_changed":False,"generated_imagery":False,
        "renderer_backend":"Mitsuba 3 path integrator + albedo/normal AOVs + Intel OIDN RT",
        "mitsuba_version":importlib.metadata.version("mitsuba"),"mitsuba_variant":variant,
        "oidn":"Intel Open Image Denoise high-quality RT filter guided by first-hit albedo and shading normals",
        "resolution":[width,height],"spp":args.spp,"max_depth":args.max_depth,
        "triangles":triangle_count,"parts":len(assembly.parts),
        "camera":{"focal_length_mm":focal,"f_stop":fstop,"camera_distance_scene_mm":distance,"horizontal_fov_degrees":hfov},
        "support":{"type":"finite rendered workbench slab","size_mm":[490,360,24],"top_z_mm":-2.0},
        "environment":hdri_info|{"rotation_degrees":args.env_rotation,"scale":args.env_scale},
        "environment_credit":"Powered by Poly Haven (live API); Workshop HDRI is CC0.",
        "post":"Guided OIDN on linear HDR radiance, then ACES fitted tone curve + sRGB transfer; no compositing, image synthesis, sharpening, or geometry substitution",
        "aov_channels":names,
    }
    (args.out/"ORBIT_v8_manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    print("DELIVERED",final,flush=True)


if __name__=="__main__":main()
