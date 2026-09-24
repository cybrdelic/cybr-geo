"""Render CYBR GEO's scan-derived morphable human face.

No image generation is used. The canonical Lee Perry-Smith scan and attributed
texture set are deformed as real 3D geometry and path traced through the same
portrait pipeline as the reference scan.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/"src"), str(ROOT/"tools"), str(ROOT/"examples")]

import drjit as dr
import mitsuba as mi
import numpy as np
from PIL import Image

import render_reference_scan_face as base
from mechanism_lab.v9 import ensure_oidn
from morphable_human_face import MorphableFaceParameters, build


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out",type=Path,default=ROOT/"build/morphable_face")
    p.add_argument("--view",choices=["portrait","front","profile","detail"],default="portrait")
    p.add_argument("--size",default="640x800")
    p.add_argument("--spp",type=int,default=96)
    p.add_argument("--batch-spp",type=int,default=32)
    p.add_argument("--depth",type=int,default=14)
    p.add_argument("--threads",type=int,default=4)
    p.add_argument("--exposure",type=float,default=1.0)
    p.add_argument("--clay",action="store_true")
    p.add_argument("--skip-export",action="store_true")
    p.add_argument("--oidn",default=os.environ.get("OIDN_BIN","auto"))

    p.add_argument("--skull-width",type=float,default=0.0)
    p.add_argument("--temple-width",type=float,default=0.0)
    p.add_argument("--cheek-width",type=float,default=0.0)
    p.add_argument("--jaw-width",type=float,default=0.0)
    p.add_argument("--lower-face-length",type=float,default=0.0)
    p.add_argument("--face-length",type=float,default=0.0)
    p.add_argument("--nose-width",type=float,default=0.0)
    p.add_argument("--mouth-width",type=float,default=0.0)
    p.add_argument("--nose-projection-mm",type=float,default=0.0)
    p.add_argument("--brow-projection-mm",type=float,default=0.0)
    p.add_argument("--chin-projection-mm",type=float,default=0.0)
    p.add_argument("--asymmetry",type=float,default=0.0)
    args=p.parse_args()

    if min(args.spp,args.batch_spp,args.depth,args.threads)<1:
        p.error("Sampling, depth and thread counts must be positive")
    width,height=map(int,args.size.split("x"))
    if width<1 or height<1:
        p.error("Positive image dimensions required")

    params=MorphableFaceParameters(
        skull_width=args.skull_width,
        temple_width=args.temple_width,
        cheek_width=args.cheek_width,
        jaw_width=args.jaw_width,
        lower_face_length=args.lower_face_length,
        face_length=args.face_length,
        nose_width=args.nose_width,
        mouth_width=args.mouth_width,
        nose_projection_mm=args.nose_projection_mm,
        brow_projection_mm=args.brow_projection_mm,
        chin_projection_mm=args.chin_projection_mm,
        asymmetry=args.asymmetry,
    )

    mi.set_variant("llvm_ad_rgb")
    dr.set_thread_count(args.threads)
    args.out=args.out.resolve()
    args.out.mkdir(parents=True,exist_ok=True)
    if args.oidn=="auto":
        args.oidn=str(ensure_oidn())

    t=time.time()
    assembly=build(params)
    mesh=base.prepare_assets(args.out,assembly)
    if not args.skip_export:
        base.export_glb(assembly,args.out)

    scene_dict,camera=base.create_scene(
        assembly,args.out,mesh,args.view,width,height,args.spp,args.depth,args.clay
    )
    loaded=mi.load_dict(scene_dict)

    # Focus on the actual morphed skin, never the interior framing target.
    origin=np.array(camera["camera_mm"],dtype=float)
    direction=np.array(camera["target_mm"],dtype=float)-origin
    direction/=np.linalg.norm(direction)
    hit=loaded.ray_intersect(mi.Ray3f(mi.Point3f(origin),mi.Vector3f(direction)))
    if not bool(np.asarray(hit.is_valid()).ravel()[0]):
        raise ValueError("Autofocus ray missed morphed face")
    if hit.shape[0].id()!="anatomy":
        raise ValueError(f"Portrait camera is obstructed by {hit.shape[0].id()}")
    focus=float(np.asarray(hit.t).ravel()[0])
    traversed=mi.traverse(loaded)
    traversed["sensor.focus_distance"]=focus
    traversed.update()
    camera["framing_distance_mm"]=camera["focus_distance_mm"]
    camera["focus_distance_mm"]=focus
    camera["focus_method"]="Central ray intersection with actual morphed skin"
    camera["focus_point_mm"]=(origin+direction*focus).tolist()

    start=time.time()
    rendered=None
    completed=0
    while completed<args.spp:
        n=min(args.batch_spp,args.spp-completed)
        sample=np.asarray(mi.render(loaded,spp=n,seed=20260924+completed*131),dtype=np.float32)
        rendered=sample*n if rendered is None else rendered+sample*n
        completed+=n
        print(f"{args.view}: {completed}/{args.spp} spp / {time.time()-start:.1f}s",flush=True)
    rendered/=args.spp

    names=base.v9.reconcile_aov_names(list(loaded.integrator().aov_names()),rendered)
    beauty=base.v9.extract_channels(rendered,names,"beauty")
    albedo=np.clip(base.v9.extract_channels(rendered,names,"albedo"),0,1)
    normal=np.clip(base.v9.extract_channels(rendered,names,"normal"),-1,1)
    if not np.isfinite(beauty).all() or float(np.ptp(beauty))<1e-5 or float(beauty.mean())<1e-5:
        raise ValueError("Invalid or blank morphable-face render")

    stem="CYBR_Morphable_Face_"+args.view+("_clay" if args.clay else "")
    base.denoise(beauty,albedo,normal,args.out,stem,args.oidn,args.exposure)
    with Image.open(args.out/(stem+".png")) as im:
        im.load()
        pixel_hash=hashlib.sha256(np.asarray(im).tobytes()).hexdigest()

    report={
        "image":stem+".png",
        "resolution":[width,height],
        "spp":args.spp,
        "depth":args.depth,
        "seconds":time.time()-start,
        "total_seconds":time.time()-t,
        "renderer":"CYBR GEO / Mitsuba LLVM CPU path tracing",
        "denoiser":"Intel OIDN / HDR + albedo + shading-normal guides",
        "image_generation":False,
        "camera":camera,
        "morph_parameters":assembly.metadata["morph_parameters"],
        "canonical_topology":assembly.metadata["canonical_topology"],
        "source_license":"CC BY 3.0",
        "identity_generation_limit":assembly.metadata["identity_generation_limit"],
        "triangles":sum(len(part.faces) for part in assembly.parts),
        "pixel_sha256":pixel_hash,
        "sha256":hashlib.sha256((args.out/(stem+".png")).read_bytes()).hexdigest(),
    }
    (args.out/(stem+".json")).write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps({k:report[k] for k in ("image","triangles","seconds","sha256")}),flush=True)


if __name__=="__main__":
    main()
