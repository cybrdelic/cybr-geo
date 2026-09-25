"""Render original procedural anatomy with CYBR GEO's approved v9 image pipeline.

No face assets are read. Geometry and material maps are generated from code.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
for folder in ('src','tools','examples'):sys.path.insert(0,str(ROOT/folder))
import numpy as np
from PIL import Image
import mitsuba as mi
import drjit as dr
import trimesh
import orbit_mitsuba_photo as v9
from procedural_human_face import (build,Anatomy,FaceParameters,SKIN,LID,CAVITY,
                                    SCLERA,IRIS,PUPIL,CORNEA,HAIR)
from procedural_face_materials import generate
from mechanism_lab.v9 import ensure_oidn


def bitmap(path,raw=False):
    return {'type':'bitmap','filename':str(path),'raw':raw,
            'filter_type':'bilinear','wrap_mode':'clamp'}


def save_png_atomic(linear,path,exposure):
    temporary=path.with_name(path.stem+'.writing.png')
    v9.save_png(linear,temporary,exposure)
    with Image.open(temporary) as im:im.load()
    temporary.replace(path)


def materials(out,clay=False):
    neutral={'type':'principled','base_color':{'type':'rgb','value':[.38,.36,.33]},
             'roughness':.5,'specular':.3}
    if clay:return [mi.load_dict(neutral) for _ in range(8)]
    dictionaries=[
        {'type':'bumpmap','texture':bitmap(out/'skin_height.png',True),'scale':.017,
         'bsdf':{'type':'principled','base_color':bitmap(out/'skin_color.png'),
                 'roughness':bitmap(out/'skin_roughness.png',True),'specular':.38,
                 'flatness':.34,'clearcoat':.008,'clearcoat_gloss':.34}},
        {'type':'principled','base_color':{'type':'rgb','value':[.30,.11,.085]},
         'roughness':.24,'specular':.4},
        {'type':'diffuse','reflectance':{'type':'rgb','value':[.025,.005,.003]}},
        {'type':'principled','base_color':bitmap(out/'sclera_color.png'),
         'roughness':.34,'specular':.24},
        {'type':'principled','base_color':bitmap(out/'iris_color.png'),
         'roughness':.48,'specular':.1},
        {'type':'diffuse','reflectance':{'type':'rgb','value':[.001,.001,.001]}},
        {'type':'roughdielectric','int_ior':1.376,'ext_ior':1.,'alpha':.012},
        {'type':'principled','base_color':{'type':'rgb','value':[.023,.011,.006]},
         'roughness':.53,'specular':.22},
    ]
    return [mi.load_dict(d) for d in dictionaries]


def mesh_shape(part,bsdf):
    props=mi.Properties();props['bsdf']=bsdf
    m=mi.Mesh(part.name,len(part.vertices),len(part.faces),props,
              has_vertex_normals=True,has_vertex_texcoords=True)
    params=mi.traverse(m)
    params['vertex_positions']=part.vertices.astype(np.float32).ravel()
    params['vertex_normals']=part.normals.astype(np.float32).ravel()
    # Direct Mitsuba mesh texture coordinates use image-space V. Exported glTF
    # textures use their own writer conversion; no imported UV conventions.
    uv=part.portrait_uv.copy();uv[:,1]=1-uv[:,1]
    params['vertex_texcoords']=uv.astype(np.float32).ravel()
    params['faces']=part.faces.astype(np.uint32).ravel();params.update()
    return m


def light(position,target,size,radiance):
    return {'type':'rectangle','to_world':mi.ScalarTransform4f.look_at(
            origin=position,target=target,up=[0,0,1])@mi.ScalarTransform4f.scale([*size,1]),
            'emitter':{'type':'area','radiance':{'type':'rgb','value':radiance}}}


def scene(assembly,out,args):
    view=assembly.views[args.view];width,height=map(int,args.size.split('x'))
    target=np.array(view.target,float)
    distance=view.scale/(36/(width/height)/(2*view.focal_length_mm))
    az,el=np.radians([view.az,view.el])
    direction=np.array([np.cos(az)*np.cos(el),np.sin(az)*np.cos(el),np.sin(el)])
    origin=target+distance*direction
    fill_scale=3 if args.view=='profile' else 1
    fill_target=np.array([0.,0.,20.]);fill=fill_target+np.array([450.,-310.,90.])*fill_scale
    d={'type':'scene',
       # Bump-map sh_normal attributes in this runtime are tangent-space.
       # OIDN needs a consistent space across skin, hair and backdrop; the
       # densely tessellated geometric normal is world-space for every material.
       'integrator':{'type':'aov','aovs':'albedo:albedo,normal:geo_normal',
                     'beauty':{'type':'path','max_depth':args.depth,'rr_depth':5}},
       'sensor':{'type':'thinlens','fov':math.degrees(2*math.atan(36/(2*view.focal_length_mm))),
                 'fov_axis':'x','to_world':mi.ScalarTransform4f.look_at(origin=origin,target=target,up=[0,0,1]),
                 'focus_distance':distance,'aperture_radius':view.focal_length_mm/(2*view.f_stop),
                 'sampler':{'type':'independent','sample_count':args.spp},
                 'film':{'type':'hdrfilm','width':width,'height':height,'component_format':'float32',
                         'rfilter':{'type':'gaussian','stddev':.42}}},
       'environment':{'type':'constant','radiance':{'type':'rgb','value':[.055,.066,.080]}},
       'key':light([-330,-410,320],[0,-20,25],[180,240],[4.8,4.48,4.15]),
       'fill':light(fill,fill_target,[220*fill_scale,260*fill_scale],[.65,.79,1.0]),
       'rim':light([340,180,270],[0,0,25],[70,180],[3.0,3.75,4.65]),
       'backdrop':{'type':'rectangle','to_world':mi.ScalarTransform4f.look_at(
            origin=[0,330,40],target=[0,0,40],up=[0,0,1])@mi.ScalarTransform4f.scale([1400,1100,1]),
            'bsdf':{'type':'diffuse','reflectance':{'type':'rgb','value':[.038,.049,.058]}}}}
    mats=materials(out,args.clay)
    for i,p in enumerate(assembly.parts):
        if args.clay and p.material in (CORNEA,HAIR):continue
        d[f'anatomy_{i}']=mesh_shape(p,mats[p.material])
    loaded=mi.load_dict(d)
    direction=target-origin;direction/=np.linalg.norm(direction)
    hit=loaded.ray_intersect(mi.Ray3f(mi.Point3f(origin),mi.Vector3f(direction)))
    if not bool(np.asarray(hit.is_valid()).ravel()[0]) or not hit.shape[0].id().startswith('anatomy_'):
        raise ValueError('Central focus ray did not hit the anatomy')
    focus=float(np.asarray(hit.t).ravel()[0]);params=mi.traverse(loaded)
    params['sensor.focus_distance']=focus;params.update()
    camera={'origin_mm':origin.tolist(),'target_mm':target.tolist(),'focal_length_mm':view.focal_length_mm,
            'f_stop':view.f_stop,'focus_distance_mm':focus,'focus_point_mm':(origin+direction*focus).tolist(),
            'filter':'Gaussian .42','environment':'Analytic constant fill; no HDR photograph'}
    return loaded,camera


def export_model(assembly,out):
    skin=Image.open(out/'skin_color.png')
    mats=[]
    for i,m in enumerate(assembly.materials):
        tex=skin if i in (SKIN,LID) else Image.open(out/'iris_color.png') if i==IRIS else (
            Image.open(out/'sclera_color.png') if i==SCLERA else None)
        color=np.array([*m.color,m.opacity])
        mat=trimesh.visual.material.PBRMaterial(name=m.name,baseColorTexture=tex,
              baseColorFactor=[255,255,255,255] if tex else np.uint8(color*255),
              roughnessFactor=m.rough,metallicFactor=0.,doubleSided=True,
              alphaMode='BLEND' if i==CORNEA else 'OPAQUE')
        mats.append(mat)
    result=trimesh.Scene()
    transform=np.array([[.001,0,0,0],[0,0,.001,0],[0,-.001,0,0],[0,0,0,1]])
    for p in assembly.parts:
        mesh=trimesh.Trimesh(p.vertices,p.faces,vertex_normals=p.normals,process=False,
                    visual=trimesh.visual.TextureVisuals(uv=p.portrait_uv,material=mats[p.material]))
        mesh.apply_transform(transform)
        result.add_geometry(mesh,geom_name=p.name,node_name=p.name)
    result.metadata=assembly.metadata
    target=out/'CYBR_Procedural_Human_Face.glb'
    temporary=out/'CYBR_Procedural_Human_Face.writing.glb'
    temporary.write_bytes(result.export(file_type='glb'));temporary.replace(target)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,default=ROOT/'build/procedural_face')
    p.add_argument('--view',choices=['portrait','front','profile','detail'],default='portrait')
    p.add_argument('--size',default='1440x1800');p.add_argument('--spp',type=int,default=512)
    p.add_argument('--depth',type=int,default=14);p.add_argument('--threads',type=int,default=8)
    p.add_argument('--quality',choices=['preview','final'],default='final')
    p.add_argument('--texture-size',type=int,default=4096)
    p.add_argument('--clay',action='store_true');p.add_argument('--no-hair',action='store_true')
    p.add_argument('--skip-export',action='store_true');p.add_argument('--reuse-textures',action='store_true')
    p.add_argument('--oidn',default=os.environ.get('OIDN_BIN','auto'),help='OIDN executable, or auto to use CYBR GEO pinned provisioning')
    p.add_argument('--seed',type=int,default=271828);p.add_argument('--exposure',type=float,default=1.)
    p.add_argument('--eyelid-closure',type=float,default=.06,help='0=open, 1=closed; modifies actual lid geometry')
    p.add_argument('--eye-spacing',type=float,default=62.0)
    p.add_argument('--eye-height',type=float,default=30.0)
    p.add_argument('--eye-width',type=float,default=28.0)
    p.add_argument('--eye-opening',type=float,default=8.4)
    p.add_argument('--eye-tilt',type=float,default=.24)
    p.add_argument('--nose-projection',type=float,default=21.5)
    p.add_argument('--nose-width',type=float,default=1.06)
    p.add_argument('--mouth-width',type=float,default=52.0)
    p.add_argument('--upper-lip-fullness',type=float,default=.92)
    p.add_argument('--lower-lip-fullness',type=float,default=.96)
    p.add_argument('--jaw-width',type=float,default=1.02)
    p.add_argument('--skull-width',type=float,default=.965)
    p.add_argument('--cheek-width',type=float,default=1.045)
    p.add_argument('--chin-width',type=float,default=.90)
    p.add_argument('--brow-weight',type=float,default=1.0)
    p.add_argument('--skin-relief',type=float,default=.0025)
    args=p.parse_args();args.out=args.out.resolve();args.out.mkdir(parents=True,exist_ok=True)
    width,height=map(int,args.size.split('x'))
    if min(width,height,args.spp,args.depth,args.threads,args.texture_size)<1:p.error('Positive settings required')
    mi.set_variant('llvm_ad_rgb');dr.set_thread_count(args.threads)
    code_hashes={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in [
        'examples/procedural_human_face.py','tools/procedural_face_materials.py',
        'tools/render_procedural_face.py','tools/orbit_mitsuba_photo.py']}
    if not 0<=args.eyelid_closure<=1:p.error('Eyelid closure must be between zero and one')
    params=FaceParameters(seed=args.seed,eyelid_closure=args.eyelid_closure,
        eye_spacing=args.eye_spacing,eye_height=args.eye_height,
        eye_width=args.eye_width,eye_opening=args.eye_opening,eye_tilt=args.eye_tilt,
        nose_projection=args.nose_projection,nose_width=args.nose_width,
        mouth_width=args.mouth_width,upper_lip_fullness=args.upper_lip_fullness,
        lower_lip_fullness=args.lower_lip_fullness,
        jaw_width=args.jaw_width,skull_width=args.skull_width,
        cheek_width=args.cheek_width,chin_width=args.chin_width,
        brow_weight=args.brow_weight,skin_relief=args.skin_relief)
    if args.oidn=='auto': args.oidn=str(ensure_oidn())
    print('Building original procedural anatomy',flush=True);t=time.time()
    assembly=build(params,args.quality,hair=not(args.no_hair or args.clay))
    print('Geometry:',sum(len(p.faces) for p in assembly.parts),'triangles in',round(time.time()-t,2),'seconds',flush=True)
    if not args.reuse_textures or not (args.out/'texture_provenance.json').exists():
        print('Generating mathematical material maps',flush=True)
        generate(args.out,Anatomy(params),args.texture_size)
    else:
        texture_report=json.loads((args.out/'texture_provenance.json').read_text())
        if texture_report['seed']!=params.seed:
            raise ValueError('Cached texture seed differs from the anatomy seed')
        for name,digest in texture_report['files'].items():
            if hashlib.sha256((args.out/name).read_bytes()).hexdigest()!=digest:
                raise ValueError(f'Cached procedural texture was changed: {name}')
    if not args.skip_export:export_model(assembly,args.out)
    print('Building v9 path-traced scene',flush=True);loaded,camera=scene(assembly,args.out,args)
    print(f'Rendering {args.view} {width} x {height}, {args.spp} spp',flush=True);start=time.time()
    rendered=np.asarray(mi.render(loaded,spp=args.spp,seed=args.seed),dtype=np.float32)
    render_seconds=time.time()-start
    names=v9.reconcile_aov_names(list(loaded.integrator().aov_names()),rendered)
    beauty=v9.extract_channels(rendered,names,'beauty')
    albedo=np.clip(v9.extract_channels(rendered,names,'albedo'),0,1)
    normal=np.clip(v9.extract_channels(rendered,names,'normal'),-1,1)
    if not np.isfinite(beauty).all() or float(beauty.mean())<1e-5 or float(np.ptp(beauty))<1e-5:
        raise ValueError('Invalid or blank render')
    stem='CYBR_Procedural_Face_'+args.view+('_clay' if args.clay else '')
    for name,data in [('beauty',beauty),('albedo',albedo),('normal',normal)]:
        v9.write_pfm(args.out/f'{stem}_{name}.pfm',data)
    subprocess.run([args.oidn,'-d','cpu','--hdr',str(args.out/f'{stem}_beauty.pfm'),
                    '--alb',str(args.out/f'{stem}_albedo.pfm'),'--nrm',str(args.out/f'{stem}_normal.pfm'),
                    '-q','high','-o',str(args.out/f'{stem}_denoised.pfm')],check=True,stdout=subprocess.DEVNULL)
    filtered=v9.read_pfm(args.out/f'{stem}_denoised.pfm')
    save_png_atomic(beauty,args.out/f'{stem}_raw.png',args.exposure)
    save_png_atomic(filtered,args.out/f'{stem}.png',args.exposure)
    with Image.open(args.out/f'{stem}.png') as im:
        im.load();pixels=hashlib.sha256(np.asarray(im).tobytes()).hexdigest()
    report={'image':stem+'.png','resolution':[width,height],'spp':args.spp,'depth':args.depth,
            'seed':args.seed,'render_seconds':render_seconds,'mitsuba_version':mi.__version__,
            'exposure_multiplier':args.exposure,'bump_height_range_mm':.017,
            'renderer':'CYBR GEO ORBIT v9 / Mitsuba LLVM CPU path tracing',
            'denoiser':'Intel OIDN, beauty + albedo + world-space geometric normal, HDR/high',
            'tone_mapping':'Unchanged v9 ACES approximation and sRGB transfer',
            'camera':camera,'source':assembly.metadata,'geometry':[part.metadata() for part in assembly.parts],
            'texture_provenance':json.loads((args.out/'texture_provenance.json').read_text()),
            'source_code_sha256':code_hashes,
            'model_sha256':hashlib.sha256((args.out/'CYBR_Procedural_Human_Face.glb').read_bytes()).hexdigest()
                if (args.out/'CYBR_Procedural_Human_Face.glb').exists() else None,
            'finite_radiance':True,'png_pixels_sha256':pixels,
            'sha256':hashlib.sha256((args.out/f'{stem}.png').read_bytes()).hexdigest()}
    (args.out/f'{stem}.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ['image','spp','render_seconds','sha256']}),flush=True)


if __name__=='__main__':main()
