"""Render a CYBR GEO human portrait using the approved ORBIT v9 pipeline.

Uses the actual v9 renderer utility functions for linear AOV handling, PFM,
tone mapping, and OIDN. Every view is path traced from the 3D Assembly.
"""
from __future__ import annotations
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'tools'))
import numpy as np
from PIL import Image
import trimesh
import mitsuba as mi
import drjit as dr
import orbit_mitsuba_photo as v9


def recipe():
    spec = importlib.util.spec_from_file_location('human_face_recipe', ROOT/'examples/human_face.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.build()


def write_obj(path, part):
    """OBJ has explicit UV and normal corner indices understood by Mitsuba."""
    with Path(path).open('w') as f:
        f.write('# CYBR GEO portrait: millimetres; Z up.\n')
        for p in part.vertices:
            f.write('v %.9g %.9g %.9g\n' % tuple(p))
        for uv in part.portrait_uv:
            # Mitsuba's OBJ importer flips OBJ V by default. Trimesh has already
            # changed glTF's image-space V to OpenGL V; undo that extra flip here.
            f.write('vt %.9g %.9g\n' % (uv[0], 1. - uv[1]))
        for n in part.normals:
            f.write('vn %.9g %.9g %.9g\n' % tuple(n))
        for face in part.faces + 1:
            f.write('f ' + ' '.join(f'{i}/{i}/{i}' for i in face) + '\n')


def prepare_assets(out, assembly):
    assets = ROOT / 'assets/portrait'
    mesh = out/'human_face.obj'
    write_obj(mesh, assembly.parts[0])
    # The supplied specular map is authored data. Convert its value to a bounded
    # perceptual roughness map without painting or synthesizing image content.
    spec = np.asarray(Image.open(assets/'Map-SPEC.jpg').convert('L'), np.float32)/255.
    rough = np.clip(.54 - .25*spec, .27, .54)
    Image.fromarray(np.uint8(rough*255)).save(out/'skin_roughness.png')
    return mesh


def bitmap(path, raw=False):
    return {'type':'bitmap', 'filename':str(path), 'raw':raw,
            'filter_type':'bilinear', 'wrap_mode':'clamp'}


def skin_bsdf(out, clay=False):
    assets=ROOT/'assets/portrait'
    if clay:
        return {'type':'principled','base_color':{'type':'rgb','value':[.38,.36,.33]},
                'roughness':.5, 'specular':.3}
    return {
        'type':'normalmap',
        'normalmap':bitmap(assets/'Infinite-Level_02_Tangent_SmoothUV.jpg',True),
        'bsdf':{'type':'principled',
                'base_color':bitmap(assets/'Map-COL.jpg'),
                'roughness':bitmap(out/'skin_roughness.png',True),
                'metallic':0., 'specular':.32,
                'flatness':.12, 'clearcoat':.03, 'clearcoat_gloss':.55}
    }


def light(position, target, size, radiance):
    return {'type':'rectangle',
            'to_world':mi.ScalarTransform4f.look_at(origin=position,target=target,up=[0,0,1])
                       @ mi.ScalarTransform4f.scale([size[0],size[1],1]),
            'emitter':{'type':'area','radiance':{'type':'rgb','value':radiance}}}


def create_scene(assembly,out,mesh,view_name,width,height,spp,depth,clay=False):
    view=assembly.views[view_name]
    target=np.array(view.target,float)
    # Match physical v9 filmback/lens construction, with portrait framing.
    aspect=width/height
    sensor_height=36./aspect
    vfov=2*math.atan(sensor_height/(2*view.focal_length_mm))
    distance=view.scale/math.tan(vfov/2)
    az,el=np.radians([view.az,view.el])
    direction=np.array([math.cos(az)*math.cos(el),math.sin(az)*math.cos(el),math.sin(el)])
    camera=target+direction*distance
    # Keep the fill panel behind the side-view camera. Scale it around the
    # lighting target so its angular extent is preserved at that target.
    fill_scale=3. if view_name=='profile' else 1.
    fill_target=np.array([0.,0.,40.])
    fill_position=fill_target+np.array([300.,-220.,70.])*fill_scale
    scene={
        'type':'scene',
        'integrator':{'type':'aov','aovs':'albedo:albedo,normal:sh_normal',
                      'beauty':{'type':'path','max_depth':depth,'rr_depth':5}},
        'sensor':{'type':'thinlens',
                  'fov':math.degrees(2*math.atan(36./(2*view.focal_length_mm))),
                  'fov_axis':'x',
                  'to_world':mi.ScalarTransform4f.look_at(origin=camera.tolist(),target=target.tolist(),up=[0,0,1]),
                  'focus_distance':distance,
                  'aperture_radius':view.focal_length_mm/(2*view.f_stop),
                  'sampler':{'type':'independent','sample_count':spp},
                  'film':{'type':'hdrfilm','width':width,'height':height,
                          'component_format':'float32','rfilter':{'type':'gaussian','stddev':.42}}},
        # Same captured HDR environment used by the approved ORBIT v9 render;
        # reduced to a fill contribution beneath dedicated portrait softboxes.
        'environment':{'type':'envmap','filename':str(ROOT/'assets/portrait/small_workshop_2k.hdr'),
                       'scale':.075,
                       'to_world':mi.ScalarTransform4f.rotate([0,0,1],195.),
                       'mis_compensation':True},
        'anatomy':{'type':'obj','filename':str(mesh),'face_normals':False,'bsdf':skin_bsdf(out,clay)},
        'key':light([-270,-390,270],[0,-15,30],[155,230],[4.8,4.48,4.15]),
        'fill':light(fill_position.tolist(),fill_target.tolist(),
                     [180*fill_scale,220*fill_scale],[.65,.79,1.0]),
        'rim':light([190,150,200],[0,0,25],[85,185],[2.0,2.45,3.0]),
        'backdrop':{'type':'rectangle',
                    'to_world':mi.ScalarTransform4f.look_at(origin=[0,300,80],target=[0,0,80],up=[0,0,1])
                               @ mi.ScalarTransform4f.scale([1200,900,1]),
                    'bsdf':{'type':'diffuse','reflectance':{'type':'rgb','value':[.045,.057,.068]}}},
    }
    return scene,{'camera_mm':camera.tolist(),'target_mm':target.tolist(),
                  'focal_length_mm':view.focal_length_mm,'f_stop':view.f_stop,
                  'focus_distance_mm':distance,'gaussian_filter_stddev':.42,
                  'fill_light_position_mm':fill_position.tolist(),
                  'fill_light_scale':fill_scale}


def denoise(beauty,albedo,normal,out,stem,oidn,exposure):
    files={name:out/f'{stem}_{name}.pfm' for name in ['beauty','albedo','normal','denoised']}
    for name,data in [('beauty',beauty),('albedo',albedo),('normal',normal)]:
        v9.write_pfm(files[name],data)
    command=[oidn,'-d','cpu','--hdr',str(files['beauty']),'--alb',str(files['albedo']),
             '--nrm',str(files['normal']),'-q','high','-o',str(files['denoised'])]
    subprocess.run(command,check=True,stdout=subprocess.DEVNULL)
    filtered=v9.read_pfm(files['denoised'])
    v9.save_png(beauty,out/f'{stem}_raw.png',exposure)
    v9.save_png(filtered,out/f'{stem}.png',exposure)
    return filtered


def export_glb(assembly,out):
    assets=ROOT/'assets/portrait'
    p=assembly.parts[0]
    material=trimesh.visual.material.PBRMaterial(
        name='Scanned skin / Lee Perry-Smith / CC BY 3.0',
        baseColorTexture=Image.open(assets/'Map-COL.jpg'),
        normalTexture=Image.open(assets/'Infinite-Level_02_Tangent_SmoothUV.jpg'),
        metallicFactor=0.,roughnessFactor=.44)
    mesh=trimesh.Trimesh(p.vertices, p.faces, vertex_normals=p.normals,process=False,
                        visual=trimesh.visual.TextureVisuals(uv=p.portrait_uv,material=material))
    # glTF metres / Y-up, unlike CYBR's millimetres / Z-up.
    transform=np.array([[.001,0,0,0],[0,0,.001,0],[0,-.001,0,0],[0,0,0,1]])
    mesh.apply_transform(transform)
    scene=trimesh.Scene(mesh)
    scene.metadata=assembly.metadata
    (out/'CYBR_Human_Face.glb').write_bytes(scene.export(file_type='glb'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,default=ROOT/'build/human_face')
    parser.add_argument('--view',default='portrait',choices=['portrait','front','profile','detail'])
    parser.add_argument('--size',default='1200x1500')
    parser.add_argument('--spp',type=int,default=512)
    parser.add_argument('--depth',type=int,default=14)
    parser.add_argument('--threads',type=int,default=8)
    parser.add_argument('--batch-spp',type=int,default=32,
                        help='Bound the LLVM wavefront size while accumulating independent paths')
    parser.add_argument('--clay',action='store_true')
    parser.add_argument('--exposure',type=float,default=1.)
    parser.add_argument('--oidn',default=os.environ.get('OIDN_BIN','oidnDenoise'))
    args=parser.parse_args()
    if args.spp<1 or args.batch_spp<1 or args.depth<1 or args.threads<1:
        parser.error('Sampling, depth and thread counts must be positive')
    mi.set_variant('llvm_ad_rgb')
    dr.set_thread_count(args.threads)
    args.out=args.out.resolve();args.out.mkdir(parents=True,exist_ok=True)
    width,height=map(int,args.size.split('x'))
    if width<1 or height<1:
        parser.error('Image dimensions must be positive')
    assembly=recipe();mesh=prepare_assets(args.out,assembly)
    export_glb(assembly,args.out)
    scene_dict,camera=create_scene(assembly,args.out,mesh,args.view,width,height,args.spp,args.depth,args.clay)
    loaded=mi.load_dict(scene_dict)
    # Framing targets can be inside an organic volume. Focusing on that target
    # defocuses the entire visible skin. Probe the actual central surface first.
    origin=np.array(camera['camera_mm'],dtype=float)
    direction=np.array(camera['target_mm'],dtype=float)-origin
    direction/=np.linalg.norm(direction)
    hit=loaded.ray_intersect(mi.Ray3f(mi.Point3f(origin),mi.Vector3f(direction)))
    if not bool(np.asarray(hit.is_valid()).ravel()[0]):
        raise ValueError('Portrait autofocus ray missed the visible surface')
    if hit.shape[0].id()!='anatomy':
        raise ValueError(f'Portrait camera is obstructed by {hit.shape[0].id()}')
    focus=float(np.asarray(hit.t).ravel()[0])
    params=mi.traverse(loaded)
    params['sensor.focus_distance']=focus
    params.update()
    camera['framing_distance_mm']=camera['focus_distance_mm']
    camera['focus_distance_mm']=focus
    camera['focus_method']='Central ray intersection with the actual visible surface'
    camera['focus_point_mm']=(origin+direction*focus).tolist()
    print('Rendering',args.view, width,height,args.spp,'spp',flush=True)
    start=time.time()
    rendered=None
    completed=0
    while completed<args.spp:
        n=min(args.batch_spp,args.spp-completed)
        sample=np.asarray(mi.render(loaded,spp=n,seed=20260914+completed*131),dtype=np.float32)
        if rendered is None:
            rendered=sample.copy()*n
        else:
            rendered+=sample*n
        completed+=n
        print(f'{args.view}: {completed}/{args.spp} samples per pixel / {time.time()-start:.1f}s',flush=True)
    rendered/=args.spp
    names=v9.reconcile_aov_names(list(loaded.integrator().aov_names()),rendered)
    beauty=v9.extract_channels(rendered,names,'beauty')
    albedo=np.clip(v9.extract_channels(rendered,names,'albedo'),0,1)
    normal=np.clip(v9.extract_channels(rendered,names,'normal'),-1,1)
    if not np.isfinite(beauty).all() or np.min(beauty)<0:
        raise ValueError('Non-finite or negative render radiance')
    if float(np.ptp(beauty))<1e-5 or float(beauty.mean())<1e-5:
        raise ValueError('Render is blank or unlit; check camera and emitter visibility')
    stem='CYBR_Face_'+args.view+('_clay' if args.clay else '')
    denoise(beauty,albedo,normal,args.out,stem,args.oidn,args.exposure)
    # Decode the full pixel data, not only the PNG header/chunk structure.
    for suffix in ['.png','_raw.png']:
        with Image.open(args.out/(stem+suffix)) as check:
            check.load()
    report={'image':stem+'.png','renderer':'CYBR GEO ORBIT v9 Mitsuba 3 / LLVM CPU',
            'mitsuba_version':mi.__version__,'resolution':[width,height],'spp':args.spp,
            'depth':args.depth,'seed':20260914,'batch_spp':args.batch_spp,
            'batch_seed_rule':'20260914 + previously_completed_spp * 131','seconds':time.time()-start,
            'denoiser':'Intel OIDN / HDR + albedo + shading-normal AOVs / high',
            'tone_mapping':'Original v9 ACES approximation followed by sRGB transfer',
            'camera':camera,'parts':[p.metadata() for p in assembly.parts],
            'source':assembly.metadata,'finite_radiance':True,
            'image_generation':False,'sha256':hashlib.sha256((args.out/(stem+'.png')).read_bytes()).hexdigest()}
    (args.out/(stem+'.json')).write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ['image','spp','seconds','sha256']}),flush=True)


if __name__=='__main__':
    main()
