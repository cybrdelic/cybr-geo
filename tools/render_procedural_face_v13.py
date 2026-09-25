"""Render CYBR GEO procedural human face v13.

All face geometry and material maps are generated from code. No scanned person,
imported human head, photographic face texture, learned identity basis, or image
generation is used.
"""
from __future__ import annotations
import argparse, hashlib, json, math, os, subprocess, sys, time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
for folder in ('src','tools','examples'):
    sys.path.insert(0,str(ROOT/folder))

import drjit as dr
import mitsuba as mi
import numpy as np
from PIL import Image
import trimesh

import orbit_mitsuba_photo as v9
from mechanism_lab.v9 import ensure_oidn
from procedural_human_face_v13 import (
    build,HumanAnatomy,FaceParameters,SKIN,LID,CAVITY,SCLERA,IRIS,PUPIL,CORNEA,HAIR
)
from procedural_face_v13_materials import generate


def bitmap(path,raw=False):
    return {'type':'bitmap','filename':str(path),'raw':raw,
            'filter_type':'bilinear','wrap_mode':'clamp'}


def save_png_atomic(linear,path,exposure):
    temporary=path.with_name(path.stem+'.writing.png')
    v9.save_png(linear,temporary,exposure)
    with Image.open(temporary) as im:
        im.load()
    temporary.replace(path)


def materials(out,clay=False):
    neutral={'type':'principled','base_color':{'type':'rgb','value':[.40,.38,.35]},
             'roughness':.52,'specular':.28}
    if clay:
        return [mi.load_dict(neutral) for _ in range(8)]

    # Renderer-compatible multilobe skin approximation. The chromophore,
    # roughness, microheight, and spatial scatter guide are all generated
    # numerically by procedural_face_v13_materials.py.
    skin={
        'type':'bumpmap',
        'texture':bitmap(out/'skin_height.png',True),
        'scale':.015,
        'bsdf':{
            'type':'principled',
            'base_color':bitmap(out/'skin_color.png'),
            'roughness':bitmap(out/'skin_roughness.png',True),
            'specular':.37,
            'flatness':bitmap(out/'skin_scatter.png',True),
            'clearcoat':.012,
            'clearcoat_gloss':.40,
        },
    }
    dictionaries=[
        skin,
        {'type':'principled','base_color':{'type':'rgb','value':[.38,.17,.13]},
         'roughness':.30,'specular':.42,'flatness':.20},
        {'type':'principled','base_color':{'type':'rgb','value':[.095,.028,.023]},
         'roughness':.50,'specular':.18,'flatness':.28},
        {'type':'principled','base_color':bitmap(out/'sclera_color.png'),
         'roughness':.30,'specular':.30,'flatness':.12},
        {'type':'principled','base_color':bitmap(out/'iris_color.png'),
         'roughness':.44,'specular':.10},
        {'type':'diffuse','reflectance':{'type':'rgb','value':[.001,.001,.001]}},
        {'type':'roughdielectric','int_ior':1.376,'ext_ior':1.0,'alpha':.010},
        {'type':'principled','base_color':{'type':'rgb','value':[.020,.010,.005]},
         'roughness':.54,'specular':.20},
    ]
    return [mi.load_dict(d) for d in dictionaries]


def mesh_shape(part,bsdf):
    props=mi.Properties();props['bsdf']=bsdf
    mesh=mi.Mesh(part.name,len(part.vertices),len(part.faces),props,
                 has_vertex_normals=True,has_vertex_texcoords=True)
    params=mi.traverse(mesh)
    params['vertex_positions']=part.vertices.astype(np.float32).ravel()
    params['vertex_normals']=part.normals.astype(np.float32).ravel()
    uv=part.portrait_uv.copy();uv[:,1]=1-uv[:,1]
    params['vertex_texcoords']=uv.astype(np.float32).ravel()
    params['faces']=part.faces.astype(np.uint32).ravel()
    params.update()
    return mesh


def light(position,target,size,radiance):
    return {
        'type':'rectangle',
        'to_world':mi.ScalarTransform4f.look_at(origin=position,target=target,up=[0,0,1])
                   @ mi.ScalarTransform4f.scale([size[0],size[1],1]),
        'emitter':{'type':'area','radiance':{'type':'rgb','value':radiance}},
    }


def build_scene(assembly,out,args):
    view=assembly.views[args.view]
    width,height=map(int,args.size.split('x'))
    target=np.array(view.target,float)
    sensor_h=36/(width/height)
    vfov=2*math.atan(sensor_h/(2*view.focal_length_mm))
    distance=view.scale/math.tan(vfov/2)
    az,el=np.radians([view.az,view.el])
    direction=np.array([np.cos(az)*np.cos(el),np.sin(az)*np.cos(el),np.sin(el)])
    origin=target+direction*distance

    fill_scale=3.0 if args.view=='profile' else 1.0
    fill_target=np.array([0.,-8.,15.])
    fill_position=fill_target+np.array([430.,-320.,100.])*fill_scale

    scene={
        'type':'scene',
        'integrator':{'type':'aov','aovs':'albedo:albedo,normal:geo_normal',
                      'beauty':{'type':'path','max_depth':args.depth,'rr_depth':5}},
        'sensor':{
            'type':'thinlens',
            'fov':math.degrees(2*math.atan(36/(2*view.focal_length_mm))),
            'fov_axis':'x',
            'to_world':mi.ScalarTransform4f.look_at(origin=origin,target=target,up=[0,0,1]),
            'focus_distance':distance,
            'aperture_radius':view.focal_length_mm/(2*view.f_stop),
            'sampler':{'type':'independent','sample_count':args.spp},
            'film':{'type':'hdrfilm','width':width,'height':height,'component_format':'float32',
                    'rfilter':{'type':'gaussian','stddev':.42}},
        },
        'environment':{'type':'constant','radiance':{'type':'rgb','value':[.045,.054,.066]}},
        'key':light([-300,-400,300],[0,-20,18],[175,230],[4.5,4.22,3.95]),
        'fill':light(fill_position.tolist(),fill_target.tolist(),
                     [215*fill_scale,250*fill_scale],[.70,.82,1.0]),
        'rim':light([310,170,245],[0,0,22],[78,170],[2.4,2.9,3.5]),
        'backdrop':{
            'type':'rectangle',
            'to_world':mi.ScalarTransform4f.look_at(origin=[0,335,45],target=[0,0,45],up=[0,0,1])
                       @ mi.ScalarTransform4f.scale([1400,1100,1]),
            'bsdf':{'type':'diffuse','reflectance':{'type':'rgb','value':[.032,.040,.050]}},
        },
    }

    mats=materials(out,args.clay)
    for i,part in enumerate(assembly.parts):
        if args.clay and part.material in (CORNEA,HAIR):
            continue
        scene[f'anatomy_{i}']=mesh_shape(part,mats[part.material])

    loaded=mi.load_dict(scene)

    ray_dir=target-origin
    ray_dir/=np.linalg.norm(ray_dir)
    hit=loaded.ray_intersect(mi.Ray3f(mi.Point3f(origin),mi.Vector3f(ray_dir)))
    if not bool(np.asarray(hit.is_valid()).ravel()[0]):
        raise ValueError('Autofocus ray missed v13 anatomy')
    focus=float(np.asarray(hit.t).ravel()[0])
    traversed=mi.traverse(loaded)
    traversed['sensor.focus_distance']=focus
    traversed.update()

    camera={
        'origin_mm':origin.tolist(),'target_mm':target.tolist(),
        'focal_length_mm':view.focal_length_mm,'f_stop':view.f_stop,
        'focus_distance_mm':focus,'focus_point_mm':(origin+ray_dir*focus).tolist(),
        'filter':'Gaussian .42','environment':'Analytic lights only; no HDR/photo environment',
    }
    return loaded,camera


def export_model(assembly,out):
    skin=Image.open(out/'skin_color.png')
    mats=[]
    for i,m in enumerate(assembly.materials):
        tex=skin if i in (SKIN,LID) else Image.open(out/'iris_color.png') if i==IRIS else (
            Image.open(out/'sclera_color.png') if i==SCLERA else None)
        color=np.array([*m.color,m.opacity])
        mats.append(trimesh.visual.material.PBRMaterial(
            name=m.name,
            baseColorTexture=tex,
            baseColorFactor=[255,255,255,255] if tex else np.uint8(np.clip(color,0,1)*255),
            roughnessFactor=m.rough,metallicFactor=0.,doubleSided=True,
            alphaMode='BLEND' if i==CORNEA else 'OPAQUE',
        ))
    scene=trimesh.Scene()
    transform=np.array([[.001,0,0,0],[0,0,.001,0],[0,-.001,0,0],[0,0,0,1]])
    for part in assembly.parts:
        mesh=trimesh.Trimesh(part.vertices,part.faces,vertex_normals=part.normals,
            process=False,visual=trimesh.visual.TextureVisuals(
                uv=part.portrait_uv,material=mats[part.material]))
        mesh.apply_transform(transform)
        scene.add_geometry(mesh,geom_name=part.name,node_name=part.name)
    scene.metadata=assembly.metadata
    tmp=out/'CYBR_V13_Human_Face.writing.glb'
    final=out/'CYBR_V13_Human_Face.glb'
    tmp.write_bytes(scene.export(file_type='glb'))
    tmp.replace(final)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,default=ROOT/'build/procedural_face_v13')
    parser.add_argument('--view',choices=['portrait','front','profile','detail'],default='portrait')
    parser.add_argument('--size',default='720x900')
    parser.add_argument('--spp',type=int,default=128)
    parser.add_argument('--depth',type=int,default=14)
    parser.add_argument('--threads',type=int,default=4)
    parser.add_argument('--quality',choices=['preview','final'],default='preview')
    parser.add_argument('--texture-size',type=int,default=1536)
    parser.add_argument('--clay',action='store_true')
    parser.add_argument('--no-hair',action='store_true')
    parser.add_argument('--skip-export',action='store_true')
    parser.add_argument('--oidn',default=os.environ.get('OIDN_BIN','auto'))
    parser.add_argument('--seed',type=int,default=271828)
    parser.add_argument('--exposure',type=float,default=1.18)

    # Anthropometric parameters.
    parser.add_argument('--face-height',type=float,default=121.)
    parser.add_argument('--bizygomatic-width',type=float,default=139.)
    parser.add_argument('--bigonial-width',type=float,default=108.)
    parser.add_argument('--intercanthal-width',type=float,default=34.)
    parser.add_argument('--eye-width',type=float,default=25.5)
    parser.add_argument('--eye-opening',type=float,default=6.1)
    parser.add_argument('--eye-height',type=float,default=30.)
    parser.add_argument('--eye-tilt',type=float,default=.22)
    parser.add_argument('--nasal-width',type=float,default=35.)
    parser.add_argument('--nasal-projection',type=float,default=21.5)
    parser.add_argument('--mouth-width',type=float,default=52.)
    parser.add_argument('--upper-lip-fullness',type=float,default=1.)
    parser.add_argument('--lower-lip-fullness',type=float,default=1.)
    parser.add_argument('--chin-height',type=float,default=24.)
    parser.add_argument('--gonial-angle',type=float,default=124.)
    parser.add_argument('--forehead-width',type=float,default=131.)
    parser.add_argument('--skull-width',type=float,default=1.)
    parser.add_argument('--brow-weight',type=float,default=1.)
    parser.add_argument('--eyelid-closure',type=float,default=.10)
    parser.add_argument('--asymmetry',type=float,default=.22)
    parser.add_argument('--skin-relief',type=float,default=.0020)

    args=parser.parse_args()
    args.out=args.out.resolve();args.out.mkdir(parents=True,exist_ok=True)
    width,height=map(int,args.size.split('x'))
    if min(width,height,args.spp,args.depth,args.threads,args.texture_size)<1:
        parser.error('Positive render settings required')
    if not 0<=args.eyelid_closure<=1:
        parser.error('eyelid closure must be in [0,1]')

    mi.set_variant('llvm_ad_rgb')
    dr.set_thread_count(args.threads)
    if args.oidn=='auto':
        args.oidn=str(ensure_oidn())

    params=FaceParameters(
        seed=args.seed,face_height=args.face_height,
        bizygomatic_width=args.bizygomatic_width,bigonial_width=args.bigonial_width,
        intercanthal_width=args.intercanthal_width,eye_width=args.eye_width,
        eye_opening=args.eye_opening,eye_height=args.eye_height,eye_tilt=args.eye_tilt,
        nasal_width=args.nasal_width,nasal_projection=args.nasal_projection,
        mouth_width=args.mouth_width,upper_lip_fullness=args.upper_lip_fullness,
        lower_lip_fullness=args.lower_lip_fullness,chin_height=args.chin_height,
        gonial_angle=args.gonial_angle,forehead_width=args.forehead_width,
        skull_width=args.skull_width,brow_weight=args.brow_weight,
        eyelid_closure=args.eyelid_closure,asymmetry=args.asymmetry,
        skin_relief=args.skin_relief,
    )

    source_files=[
        'examples/procedural_human_face_v13.py',
        'tools/procedural_face_v13_materials.py',
        'tools/render_procedural_face_v13.py',
        'tools/orbit_mitsuba_photo.py',
    ]
    code_hashes={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in source_files}

    start=time.time()
    anatomy=HumanAnatomy(params)
    assembly=build(params,args.quality,hair=not(args.no_hair or args.clay))
    geometry_seconds=time.time()-start
    triangles=sum(len(part.faces) for part in assembly.parts)
    print('V13 geometry',triangles,'triangles in',round(geometry_seconds,2),'s',flush=True)

    generate(args.out,anatomy,args.texture_size)
    if not args.skip_export:
        export_model(assembly,args.out)

    loaded,camera=build_scene(assembly,args.out,args)
    print('Rendering',args.view,width,height,args.spp,'spp',flush=True)
    t=time.time()
    rendered=np.asarray(mi.render(loaded,spp=args.spp,seed=args.seed),dtype=np.float32)
    render_seconds=time.time()-t

    names=v9.reconcile_aov_names(list(loaded.integrator().aov_names()),rendered)
    beauty=v9.extract_channels(rendered,names,'beauty')
    albedo=np.clip(v9.extract_channels(rendered,names,'albedo'),0,1)
    normal=np.clip(v9.extract_channels(rendered,names,'normal'),-1,1)
    if not np.isfinite(beauty).all() or float(beauty.mean())<1e-5 or float(np.ptp(beauty))<1e-5:
        raise ValueError('Blank or invalid v13 render')

    stem='CYBR_V13_Face_'+args.view+('_clay' if args.clay else '')
    for name,data in [('beauty',beauty),('albedo',albedo),('normal',normal)]:
        v9.write_pfm(args.out/f'{stem}_{name}.pfm',data)

    subprocess.run([
        args.oidn,'-d','cpu','--hdr',str(args.out/f'{stem}_beauty.pfm'),
        '--alb',str(args.out/f'{stem}_albedo.pfm'),
        '--nrm',str(args.out/f'{stem}_normal.pfm'),
        '-q','high','-o',str(args.out/f'{stem}_denoised.pfm')
    ],check=True,stdout=subprocess.DEVNULL)

    filtered=v9.read_pfm(args.out/f'{stem}_denoised.pfm')
    save_png_atomic(beauty,args.out/f'{stem}_raw.png',args.exposure)
    save_png_atomic(filtered,args.out/f'{stem}.png',args.exposure)

    with Image.open(args.out/f'{stem}.png') as image:
        image.load()
        pixel_hash=hashlib.sha256(np.asarray(image).tobytes()).hexdigest()

    receipt={
        'image':stem+'.png','resolution':[width,height],'spp':args.spp,'depth':args.depth,
        'seed':args.seed,'geometry_seconds':geometry_seconds,'render_seconds':render_seconds,
        'triangles':triangles,'renderer':'CYBR GEO / Mitsuba LLVM path tracing',
        'mitsuba_version':mi.__version__,
        'denoiser':'Intel OIDN HDR + albedo + geometric normal',
        'camera':camera,'source':assembly.metadata,
        'geometry':[part.metadata() for part in assembly.parts],
        'texture_provenance':json.loads((args.out/'texture_provenance.json').read_text()),
        'source_code_sha256':code_hashes,
        'image_generation':False,'scan_used':False,'learned_face_model':False,
        'png_pixels_sha256':pixel_hash,
        'sha256':hashlib.sha256((args.out/f'{stem}.png').read_bytes()).hexdigest(),
    }
    (args.out/f'{stem}.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps({k:receipt[k] for k in ['image','triangles','render_seconds','sha256']}),flush=True)


if __name__=='__main__':
    main()
