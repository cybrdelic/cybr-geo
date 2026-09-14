"""CYBR GEO V9 photographic backend.

This is the actual renderer family used by the approved ORBIT V9 image:
Mitsuba 3 path tracing, a captured CC0 workshop HDRI, a finite rendered bench
with a scanned CC0 roughness map, thin-lens optics, principled PBR materials,
and albedo/normal-guided Intel Open Image Denoise in linear HDR.

No image generation, background compositing, fake mechanical geometry, optical
flow, or sharpening is used. Geometry comes from the Assembly passed in.
"""
from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import hashlib
import json
import math
import os
import platform
import shutil
import struct
import subprocess
import tarfile
import tempfile
import urllib.request

import numpy as np
from PIL import Image

from .render_profiles import V9

USER_AGENT = "CYBR-GEO/V9 photographic renderer"
POLYHAVEN_API = "https://api.polyhaven.com/files/{slug}"
OIDN_VERSION = "2.5.1"
OIDN_LINUX_X86_64 = "https://github.com/RenderKit/oidn/releases/download/v2.5.1/oidn-2.5.1.x86_64.linux.tar.gz"
OIDN_LINUX_X86_64_SHA256 = "743c3e2aff8c220d5d70fe6cb970fb3d36f2702d2693c61d1d148e404cf37cd6"


def cache_root() -> Path:
    configured=os.environ.get("CYBR_GEO_V9_CACHE")
    if configured:return Path(configured).expanduser().resolve()
    xdg=os.environ.get("XDG_CACHE_HOME")
    root=Path(xdg).expanduser() if xdg else Path.home()/".cache"
    return root/"cybr-geo"/"v9"


def _json(url: str):
    request=urllib.request.Request(url,headers={"User-Agent":USER_AGENT})
    with urllib.request.urlopen(request,timeout=60) as response:return json.load(response)


def _md5(path: Path) -> str:
    h=hashlib.md5()
    with path.open("rb") as f:
        for block in iter(lambda:f.read(1024*1024),b""):h.update(block)
    return h.hexdigest()


def _sha256(path: Path) -> str:
    h=hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda:f.read(1024*1024),b""):h.update(block)
    return h.hexdigest()


def _download(record,destination: Path):
    destination.parent.mkdir(parents=True,exist_ok=True)
    expected=(record.get("md5") or "").lower()
    if destination.exists() and (not expected or _md5(destination).lower()==expected):
        return _md5(destination)
    temporary=destination.with_suffix(destination.suffix+".partial")
    temporary.unlink(missing_ok=True)
    request=urllib.request.Request(record["url"],headers={"User-Agent":USER_AGENT})
    with urllib.request.urlopen(request,timeout=120) as source,temporary.open("wb") as target:
        shutil.copyfileobj(source,target,1024*1024)
    digest=_md5(temporary)
    if expected and digest.lower()!=expected:
        temporary.unlink(missing_ok=True);raise RuntimeError(f"Checksum mismatch for {destination.name}")
    temporary.replace(destination);return digest


def _poly_record(tree,map_name,resolution,preferred=("jpg","png","exr","hdr")):
    branch=tree.get(map_name)
    if not isinstance(branch,dict):raise KeyError(f"Poly Haven map {map_name!r} unavailable")
    level=branch.get(resolution)
    if not isinstance(level,dict):raise KeyError(f"Poly Haven map {map_name!r} lacks {resolution}")
    for fmt in preferred:
        record=level.get(fmt)
        if isinstance(record,dict) and record.get("url"):return record
    raise KeyError(f"No usable {map_name} file at {resolution}")


def ensure_assets():
    root=cache_root()/"assets";root.mkdir(parents=True,exist_ok=True)
    # Exact assets from the approved V9 branch.
    hdri_slug=V9.environment_asset;hdri_res=V9.environment_resolution
    hdri_tree=_json(POLYHAVEN_API.format(slug=hdri_slug))
    hdri_level=hdri_tree.get("hdri",{}).get(hdri_res,{})
    hdri_record=hdri_level.get("hdr") or hdri_level.get("exr")
    if not isinstance(hdri_record,dict) or not hdri_record.get("url"):
        raise RuntimeError(f"No {hdri_res} HDR/EXR for Poly Haven {hdri_slug}")
    hdri_ext=".hdr" if "hdr" in hdri_level and hdri_record is hdri_level.get("hdr") else ".exr"
    hdri=root/f"{hdri_slug}_{hdri_res}{hdri_ext}"
    hdri_md5=_download(hdri_record,hdri)

    bench_slug=V9.bench_asset;bench_res=V9.bench_resolution
    bench_tree=_json(POLYHAVEN_API.format(slug=bench_slug))
    bench_record=_poly_record(bench_tree,"Rough",bench_res)
    suffix=Path(urllib.request.urlparse(bench_record["url"]).path).suffix or ".jpg"
    bench=root/f"{bench_slug}_rough_{bench_res}{suffix}"
    bench_md5=_download(bench_record,bench)
    return {
        "hdri":hdri,"roughness":bench,
        "environment":{"asset":hdri_slug,"resolution":hdri_res,"url":hdri_record["url"],"md5":hdri_md5,"license":"CC0","provider":"Poly Haven"},
        "bench":{"asset":bench_slug,"map":"Rough","resolution":bench_res,"url":bench_record["url"],"md5":bench_md5,"license":"CC0","provider":"Poly Haven"},
    }


def ensure_oidn() -> Path:
    configured=os.environ.get("CYBR_GEO_OIDN")
    if configured:
        path=Path(configured).expanduser().resolve()
        if not path.is_file():raise FileNotFoundError(f"CYBR_GEO_OIDN does not exist: {path}")
        return path
    system=shutil.which("oidnDenoise")
    if system:return Path(system)
    machine=platform.machine().lower()
    if platform.system()!="Linux" or machine not in {"x86_64","amd64"}:
        raise RuntimeError("V9 needs Intel Open Image Denoise. Install oidnDenoise or set CYBR_GEO_OIDN; automatic install is supported on Linux x86_64/WSL.")
    root=cache_root()/f"oidn-{OIDN_VERSION}"
    existing=list(root.glob("**/bin/oidnDenoise")) if root.exists() else []
    if existing:return existing[0]
    root.mkdir(parents=True,exist_ok=True)
    archive=cache_root()/f"oidn-{OIDN_VERSION}.tar.gz"
    if not archive.exists() or _sha256(archive)!=OIDN_LINUX_X86_64_SHA256:
        temporary=archive.with_suffix(".partial");temporary.unlink(missing_ok=True)
        request=urllib.request.Request(OIDN_LINUX_X86_64,headers={"User-Agent":USER_AGENT})
        with urllib.request.urlopen(request,timeout=180) as source,temporary.open("wb") as target:
            shutil.copyfileobj(source,target,1024*1024)
        if _sha256(temporary)!=OIDN_LINUX_X86_64_SHA256:
            temporary.unlink(missing_ok=True);raise RuntimeError("OIDN archive checksum mismatch")
        temporary.replace(archive)
    with tarfile.open(archive,"r:gz") as tar:
        members=tar.getmembers()
        for member in members:
            target=(root/member.name).resolve()
            if root.resolve() not in target.parents and target!=root.resolve():raise RuntimeError("Unsafe OIDN archive path")
        tar.extractall(root)
    candidates=list(root.glob("**/bin/oidnDenoise"))
    if not candidates:raise RuntimeError("OIDN archive did not contain oidnDenoise")
    return candidates[0]


def _mitsuba():
    try:import mitsuba as mi
    except ImportError as error:raise RuntimeError("V9 requires Mitsuba 3. Install cybr-geo with its current dependencies or `pip install 'mitsuba>=3.6,<3.8'`.") from error
    variants=mi.variants();variant="llvm_ad_rgb" if "llvm_ad_rgb" in variants else "scalar_rgb"
    try:mi.set_variant(variant)
    except Exception:
        # Mitsuba can already have this variant selected in a long-running process.
        if mi.variant()!=variant:raise
    return mi,variant


def transformed_mesh(assembly,part,time_seconds=0.0,explode=0.0):
    T=np.asarray(assembly.pose(part,time_seconds,explode),dtype=np.float64)
    vertices=np.asarray(part.vertices,dtype=np.float64)
    h=np.c_[vertices,np.ones(len(vertices))]
    vertices=(h@T.T)[:,:3].astype("<f4")
    R=T[:3,:3]
    normals=np.asarray(part.normals,dtype=np.float64)@R.T
    normals=(normals/np.maximum(np.linalg.norm(normals,axis=1)[:,None],1e-12)).astype("<f4")
    return vertices,normals,np.asarray(part.faces,dtype="<i4")


def write_binary_ply(path: Path,vertices,normals,faces):
    path.parent.mkdir(parents=True,exist_ok=True)
    header=("ply\nformat binary_little_endian 1.0\n"
            f"element vertex {len(vertices)}\n"
            "property float x\nproperty float y\nproperty float z\n"
            "property float nx\nproperty float ny\nproperty float nz\n"
            f"element face {len(faces)}\n"
            "property list uchar int vertex_indices\nend_header\n").encode("ascii")
    data=np.empty((len(vertices),6),dtype="<f4");data[:,:3]=vertices;data[:,3:]=normals
    with path.open("wb") as f:
        f.write(header);f.write(data.tobytes(order="C"))
        for tri in faces:f.write(struct.pack("<Biii",3,int(tri[0]),int(tri[1]),int(tri[2])))


def _part_variation(name:str):
    raw=hashlib.sha256(name.encode()).digest()
    a=int.from_bytes(raw[:4],"little")/2**32;b=int.from_bytes(raw[4:8],"little")/2**32
    return .93+.14*a,.985+.03*b


def _reference_material(material):
    """Exact ORBIT V9 appearance overrides; all other recipes keep authored values."""
    name=material.name.lower();m=material
    if "graphite anodized" in name:m=replace(m,color=(.028,.034,.041),metal=.12,rough=.34,ior=1.55,coat=.16,coat_rough=.27,anisotropy=.06)
    elif "teal anodized" in name:m=replace(m,color=(.008,.115,.095),metal=.10,rough=.33,ior=1.55,coat=.18,coat_rough=.25,anisotropy=.06)
    elif "satin machined aluminium" in name:m=replace(m,color=(.70,.72,.735),metal=1.0,rough=.27,coat=.01,coat_rough=.24,anisotropy=.56)
    elif "hardened steel" in name:m=replace(m,color=(.49,.505,.52),metal=1.0,rough=.25,coat=.01,coat_rough=.20,anisotropy=.31)
    elif "warm bronze" in name:m=replace(m,color=(.58,.32,.11),metal=1.0,rough=.30,coat=.01,coat_rough=.22,anisotropy=.22)
    elif "black elastomer" in name:m=replace(m,color=(.0045,.0055,.007),metal=0.0,rough=.86,ior=1.47,coat=0.0,anisotropy=0.0)
    elif "ceramic white" in name:m=replace(m,color=(.70,.72,.735),metal=0.0,rough=.42,ior=1.51,coat=.02,coat_rough=.30,anisotropy=0.0)
    elif "connector copper" in name:m=replace(m,color=(.62,.245,.085),metal=.99,rough=.24,coat=.01,coat_rough=.20,anisotropy=.26)
    return m


def principled(material,part_name):
    material=_reference_material(material)
    rough_mul,color_mul=_part_variation(part_name)
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


def _write_pfm(path:Path,image):
    arr=np.asarray(image,dtype="<f4")
    if arr.ndim!=3 or arr.shape[2]!=3:raise ValueError("PFM requires HxWx3 RGB")
    h,w,_=arr.shape
    with path.open("wb") as f:
        f.write(f"PF\n{w} {h}\n-1.0\n".encode("ascii"));f.write(np.flipud(arr).tobytes(order="C"))


def _read_pfm(path:Path):
    with path.open("rb") as f:
        if f.readline().strip()!=b"PF":raise ValueError("Expected RGB PFM")
        w,h=map(int,f.readline().split());scale=float(f.readline());dtype="<f4" if scale<0 else ">f4"
        arr=np.fromfile(f,dtype=dtype,count=w*h*3).reshape(h,w,3)
    return np.flipud(arr).astype(np.float32,copy=False)


def _aces(linear,exposure=1.0):
    x=np.maximum(0.0,np.asarray(linear,dtype=np.float32)*exposure)
    a,b,c,d,e=2.51,.03,2.43,.59,.14
    y=np.clip((x*(a*x+b))/(x*(c*x+d)+e),0,1)
    y=np.where(y<=.0031308,12.92*y,1.055*np.power(y,1/2.4)-.055)
    return np.clip(y,0,1)


def _save_png(linear,path:Path,exposure):
    ldr=_aces(linear,exposure)
    path.parent.mkdir(parents=True,exist_ok=True)
    Image.fromarray(np.round(ldr*255).astype(np.uint8),"RGB").save(path,compress_level=6)
    with Image.open(path) as check:check.verify()


def _extract(rendered,names,prefix):
    indices=[i for i,n in enumerate(names) if n.startswith(prefix+".")]
    if len(indices)<3:raise RuntimeError(f"Missing {prefix} channels: {names}")
    return rendered[...,indices[:3]].astype(np.float32,copy=False)


def _reconcile(names,rendered):
    if rendered.ndim!=3:raise RuntimeError(f"Unexpected AOV tensor shape {rendered.shape}")
    if rendered.shape[2]==len(names):return names
    if rendered.shape[2]==len(names)-1 and "beauty.A" in names:
        compact=[n for n in names if n!="beauty.A"]
        if len(compact)==rendered.shape[2]:return compact
    raise RuntimeError(f"Unexpected AOV tensor shape {rendered.shape} for {names}")


def _camera(view,size,azimuth=None):
    from .photoreal import _camera_distance
    width,height=size;focal=float(view.focal_length_mm);sensor=float(view.sensor_width_mm)
    distance=_camera_distance(view,size);az=math.radians(view.az if azimuth is None else azimuth);el=math.radians(view.el)
    target=np.asarray(view.target,dtype=float)
    direction=np.array([math.cos(az)*math.cos(el),math.sin(az)*math.cos(el),math.sin(el)])
    origin=target+direction*distance
    hfov=math.degrees(2*math.atan(sensor/(2*focal)))
    return origin,target,distance,hfov


def _scene_dict(mi,assembly,view,size,spp,depth,assets,mesh_dir,time_seconds=0.0,explode=0.0,azimuth=None,f_stop=None,focus_distance=None):
    origin,target,distance,hfov=_camera(view,size,azimuth)
    focal=float(view.focal_length_mm);fstop=float(f_stop or view.f_stop or V9.reference_f_stop)
    focus=float(focus_distance or view.focus_distance_mm or distance)
    bench_rough={"type":"bitmap","filename":str(assets["roughness"]),"raw":True,"filter_type":"bilinear","wrap_mode":"repeat","to_uv":mi.ScalarTransform3f.scale([2.2,1.7])}
    bench_bsdf={"type":"principled","base_color":{"type":"rgb","value":[.105,.100,.094]},"metallic":0.0,"roughness":bench_rough,"specular":.34,"clearcoat":0.0}
    scene={
        "type":"scene",
        "integrator":{"type":"aov","aovs":"albedo:albedo,normal:sh_normal","beauty":{"type":"path","max_depth":depth,"rr_depth":5}},
        "sensor":{"type":"thinlens","fov":hfov,"fov_axis":"x","to_world":mi.ScalarTransform4f.look_at(origin=origin.tolist(),target=target.tolist(),up=[0,0,1]),"focus_distance":focus,"aperture_radius":focal/(2*fstop),"sampler":{"type":"independent","sample_count":spp},"film":{"type":"hdrfilm","width":size[0],"height":size[1],"component_format":"float32","rfilter":{"type":"gaussian","stddev":V9.reconstruction_filter_stddev}}},
        "environment":{"type":"envmap","filename":str(assets["hdri"]),"scale":V9.environment_scale,"to_world":mi.ScalarTransform4f.rotate([0,0,1],V9.environment_rotation_degrees),"mis_compensation":True},
    }
    bounds=assembly.bounds;center=bounds.mean(axis=0);span=bounds[1]-bounds[0]
    floor_z=float(view.floor_z_mm if view.floor_z_mm is not None else bounds[0,2]-view.floor_gap_mm)
    if view.floor:
        scene["bench"]={"type":"rectangle","to_world":mi.ScalarTransform4f.translate([float(center[0]),float(center[1]),floor_z-.04]) @ mi.ScalarTransform4f.scale([max(185.,float(span[0])*.90),max(135.,float(span[1])*.90),1]),"bsdf":bench_bsdf}
    triangle_count=0
    for i,part in enumerate(assembly.parts):
        vertices,normals,faces=transformed_mesh(assembly,part,time_seconds,explode);triangle_count+=len(faces)
        path=mesh_dir/f"{i:04d}_{part.name}.ply";write_binary_ply(path,vertices,normals,faces)
        scene[f"part_{i:04d}"]={"type":"ply","filename":str(path),"face_normals":False,"bsdf":principled(assembly.materials[part.material],part.name)}
    return scene,triangle_count,{"origin":origin.tolist(),"target":target.tolist(),"distance":distance,"focal_length_mm":focal,"f_stop":fstop,"horizontal_fov_degrees":hfov}


def _render_linear(mi,scene,spp):
    loaded=mi.load_dict(scene);names=list(loaded.integrator().aov_names())
    rendered=np.asarray(mi.render(loaded,spp=spp),dtype=np.float32);names=_reconcile(names,rendered)
    beauty=_extract(rendered,names,"beauty");albedo=np.clip(_extract(rendered,names,"albedo"),0,1);normal=np.clip(_extract(rendered,names,"normal"),-1,1)
    if not np.isfinite(beauty).all() or float(beauty.min())<0:raise RuntimeError("Invalid Mitsuba radiance")
    return beauty,albedo,normal,names


def _oidn(beauty,albedo,normal,work:Path,oidn:Path):
    beauty_pfm=work/"beauty.pfm";albedo_pfm=work/"albedo.pfm";normal_pfm=work/"normal.pfm";denoised_pfm=work/"denoised.pfm"
    _write_pfm(beauty_pfm,beauty);_write_pfm(albedo_pfm,albedo);_write_pfm(normal_pfm,normal)
    env=os.environ.copy();lib=oidn.parent.parent/"lib"
    if lib.is_dir():env["LD_LIBRARY_PATH"]=str(lib)+(":"+env["LD_LIBRARY_PATH"] if env.get("LD_LIBRARY_PATH") else "")
    subprocess.run([str(oidn),"-d","cpu","--hdr",str(beauty_pfm),"--alb",str(albedo_pfm),"--nrm",str(normal_pfm),"-q","high","-o",str(denoised_pfm)],check=True,env=env)
    result=_read_pfm(denoised_pfm)
    if result.shape!=beauty.shape or not np.isfinite(result).all():raise RuntimeError("Invalid OIDN output")
    return result


def render_v9(assembly,output,view_name="hero",size=None,spp=None,depth=None,intent="auto",allow_estimates=False,time_seconds=0.0,explode=None,f_stop=None,focus_distance=None,exposure=None):
    """Render one actual V9 still from the supplied assembly geometry."""
    from .truth import assert_renderable,write_truth_report
    from .photoreal import prepare_view_geometry,resolve_studio
    truth=assert_renderable(assembly,intent,allow_estimates)
    size=tuple(size or V9.still_size);spp=int(spp or V9.still_spp);depth=int(depth or V9.still_depth);exposure=float(V9.exposure if exposure is None else exposure)
    view=resolve_studio(assembly,assembly.views[view_name]);subset,view=prepare_view_geometry(assembly,view)
    if explode is None:explode=view.explode
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    assets=ensure_assets();oidn=ensure_oidn();mi,variant=_mitsuba()
    with tempfile.TemporaryDirectory(prefix="cybr_v9_") as td:
        work=Path(td);mesh=work/"mesh";mesh.mkdir()
        scene,triangles,camera=_scene_dict(mi,subset,view,size,spp,depth,assets,mesh,time_seconds,explode,f_stop=f_stop,focus_distance=focus_distance)
        beauty,albedo,normal,names=_render_linear(mi,scene,spp);denoised=_oidn(beauty,albedo,normal,work,oidn)
        _save_png(beauty,output.with_name(output.stem+"_noisy.png"),exposure);_save_png(denoised,output,exposure)
    manifest={"model":assembly.name,"view":view_name,"render_profile":"v9","renderer":"Mitsuba 3 path + albedo/normal-guided Intel OIDN","variant":variant,"resolution":list(size),"spp":spp,"max_depth":depth,"triangles":triangles,"parts":len(subset.parts),"camera":camera,"environment":assets["environment"]|{"rotation_degrees":V9.environment_rotation_degrees,"scale":V9.environment_scale},"bench_surface":assets["bench"]|{"geometry":"finite rendered rectangle","roughness_source":"scanned map"},"post":"Guided OIDN on linear HDR radiance, ACES fitted tone curve, sRGB transfer only; no compositing, image synthesis, frame interpolation, or sharpening","generated_imagery":False,"aov_channels":names,"truth":truth}
    output.with_suffix(".json").write_text(json.dumps(manifest,indent=2)+"\n");write_truth_report(truth,output.with_suffix(".truth.json"));return manifest


def render_v9_video(assembly,output,shots,size=None,fps=24,spp=None,depth=None,intent="auto",allow_estimates=False,exposure=None):
    """Path trace and OIDN-denoise every frame using the same V9 scene model."""
    from .truth import assert_renderable,write_truth_report
    from .photoreal import prepare_view_geometry,resolve_studio
    from .media import probe
    truth=assert_renderable(assembly,intent,allow_estimates);size=tuple(size or V9.video_size);spp=int(spp or V9.video_spp);depth=int(depth or V9.video_depth);exposure=float(V9.exposure if exposure is None else exposure)
    if fps<=0 or any(v<=0 or v%2 for v in size):raise ValueError("V9 video needs positive fps and even H.264 dimensions")
    assets=ensure_assets();oidn=ensure_oidn();mi,variant=_mitsuba();output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    frame_count=0;records=[]
    with tempfile.TemporaryDirectory(prefix="cybr_v9_film_") as td:
        root=Path(td)
        for shot_index,shot in enumerate(shots):
            if shot.view not in assembly.views or shot.duration<=0:raise ValueError("Invalid V9 shot")
            base_view=resolve_studio(assembly,assembly.views[shot.view]);subset,base_view=prepare_view_geometry(assembly,base_view);n=round(shot.duration*fps)
            for frame in range(n):
                u=frame/max(1,n-1);time_seconds=frame_count/fps
                az=base_view.az+shot.orbit_degrees*(u-.5) if shot.action=="orbit" else base_view.az
                explosion=(.5-.5*math.cos(math.tau*u)) if shot.action=="explode" else base_view.explode
                motion_time=time_seconds if shot.action in ("motion","orbit") else 0.0
                frame_dir=root/f"work_{frame_count:06d}";frame_dir.mkdir();mesh=frame_dir/"mesh";mesh.mkdir()
                scene,triangles,camera=_scene_dict(mi,subset,base_view,size,spp,depth,assets,mesh,motion_time,explosion,azimuth=az)
                beauty,albedo,normal,_=_render_linear(mi,scene,spp);denoised=_oidn(beauty,albedo,normal,frame_dir,oidn)
                path=root/f"{frame_count:06d}.png";_save_png(denoised,path,exposure)
                records.append({"frame":frame_count,"shot":shot_index,"triangles":triangles,"camera":camera});frame_count+=1
                shutil.rmtree(frame_dir,ignore_errors=True)
        if not frame_count:raise ValueError("No V9 video frames")
        subprocess.run(["ffmpeg","-y","-v","error","-xerror","-framerate",str(fps),"-i",str(root/"%06d.png"),"-frames:v",str(frame_count),"-an","-c:v","libx264","-preset","slow","-crf","15","-pix_fmt","yuv420p","-movflags","+faststart",str(output)],check=True)
        subprocess.run(["ffmpeg","-v","error","-xerror","-i",str(output),"-f","null","-"],check=True)
    encoded=probe(output);report={"model":assembly.name,"render_profile":"v9","renderer":"Mitsuba 3 path + guided OIDN per encoded frame","variant":variant,"frames":frame_count,"duration":frame_count/fps,"fps":fps,"resolution":list(size),"spp_per_frame":spp,"max_depth":depth,"generated_imagery":False,"frame_interpolation":False,"environment":assets["environment"],"bench_surface":assets["bench"],"probe":encoded,"frames_log":records,"truth":truth}
    output.with_suffix(".json").write_text(json.dumps(report,indent=2)+"\n");write_truth_report(truth,output.with_suffix(".truth.json"));return report
