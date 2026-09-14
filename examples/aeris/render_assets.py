"""Render AERIS through CYBR GEO's actual V9 photographic backend.

V9 is Mitsuba 3 path tracing under the captured CC0 small-workshop HDRI, with a
finite bench using a scanned CC0 roughness map and albedo/normal-guided Intel
OIDN. No image generation, pasted photography, fake geometry, or sharpening.
"""
from pathlib import Path
import sys, os, argparse, json, hashlib
from dataclasses import replace
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(Path(__file__).resolve().parent))
os.environ.setdefault('MECHANISM_LAB_ROOT',str(ROOT))
from recipe import coupled_pose
from mechanism_lab import View
from mechanism_lab.core import load_cache
from mechanism_lab.render_profiles import V9
from mechanism_lab.v9_dispatch import render_v9
from mechanism_lab.photoreal import render_photoreal
from mechanism_lab.render import render_still


def load(out,section=False):
    a=load_cache(out/('cutaway_cache' if section else 'cache'))
    a.motion_function=coupled_pose
    return a


def select_shot(a,name):
    if name=='hero_shell':
        # Exterior photographic shot. These groups are physically internal and
        # occluded by the assembled shell/filter/motor housing from this camera;
        # omitting them changes renderer load only, not the visible exterior CAD.
        # Dedicated internal/detail views below render those components directly.
        internal={'impeller','shaft','bearings','motor_rotor','motor_stator','windings','lattice','gyroid'}
        source_count=len(a.parts)
        visible=[p for p in a.parts if p.group not in internal]
        a=replace(a,name='aeris_hero_shell',parts=visible,views={'hero':a.views['hero']})
        print(f'HERO_SHELL_VISIBLE_PARTS {len(visible)}/{source_count}',flush=True)
        return a,'hero'
    if name=='rotor_detail':
        keep={'impeller','shaft','bearings','motor_rotor','motor_stator','windings','motor_front'}
        a=replace(a,name='aeris_rotor_detail',parts=[p for p in a.parts if p.group in keep])
        a.views={'hero':View(az=246,el=19,scale=82,target=(42,0,0),focal_length_mm=75.,f_stop=16.,floor=False,
                             environment_strength=.34,light_size=1.7,light_intensity=1.12,exposure=1.06)}
        return a,'hero'
    if name=='topology_detail':
        a=replace(a,name='aeris_topology_detail',parts=[p for p in a.parts if p.group in {'lattice','gyroid','outlet'}])
        a.views={'hero':View(az=220,el=19,scale=39,target=(18,80,127),focal_length_mm=85.,f_stop=16.,floor=False,
                             environment_strength=.34,light_size=1.7,light_intensity=1.12,exposure=1.05)}
        return a,'hero'
    return a,name


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,default=ROOT/'build/aeris')
    ap.add_argument('--views',nargs='+',default=['hero_shell','rotor_detail','topology_detail','hero','cutaway','exploded'])
    ap.add_argument('--size',default=f'{V9.still_size[0]}x{V9.still_size[1]}');ap.add_argument('--spp',type=int,default=V9.still_spp)
    ap.add_argument('--threads',type=int,default=4);ap.add_argument('--depth',type=int,default=V9.still_depth)
    ap.add_argument('--preview',action='store_true');ap.add_argument('--pbr',action='store_true');ap.add_argument('--native',action='store_true')
    args=ap.parse_args();size=tuple(map(int,args.size.split('x')))
    folder=args.out/('previews' if args.preview else 'renders');folder.mkdir(parents=True,exist_ok=True)
    for name in args.views:
        a=load(args.out,name=='cutaway');view='hero' if name=='cutaway' else name
        if name in ('hero_shell','rotor_detail','topology_detail'):a,view=select_shot(a,name)
        p=folder/f'aeris_{name}.png';print('RENDER_START',name,size,args.spp,flush=True)
        if args.pbr:
            render_still(a,p,view_name=view,size=size,captions=False,supersample=1 if args.preview else 2,intent='concept')
        elif args.native:
            report=render_photoreal(a,p,view_name=view,size=size,spp=args.spp,threads=args.threads,depth=args.depth,intent='concept',captions=False)
            report['backend']='legacy-native-photoreal'
        else:
            report=render_v9(a,p,view_name=view,size=size,spp=args.spp,depth=args.depth,intent='concept')
            report['backend']='v9-mitsuba-hdri-oidn'
        if not args.pbr:
            report['sha256']=hashlib.sha256(p.read_bytes()).hexdigest()
            p.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
        print('RENDER_DONE',name,str(p),flush=True)

if __name__=='__main__':main()
