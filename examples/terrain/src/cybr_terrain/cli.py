"""One-command terrain simulation, geometry export and checked native rendering."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import importlib.metadata
import json
from pathlib import Path
import shutil
import sys
import tempfile
import time
import numpy as np

from .model import Config, State, make_terrain, PRESETS
from .simulation import Simulator
from .geometry import build_geometry
from .rendering import render_terrain


def positive(value):
    value=int(value)
    if value<1:raise argparse.ArgumentTypeError('Must be positive')
    return value


def parser():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command',required=True)
    sub.add_parser('list',help='List authored geology/soil scenarios')
    sub.add_parser('doctor',help='Check runtime/compiler before generating output')
    for command in ('simulate','demo'):
        r=sub.add_parser(command,help='Simulate terrain'+(' and export/render it' if command=='demo' else ' to a state file'))
        r.add_argument('preset',choices=[*PRESETS,'all']);r.add_argument('--grid',type=positive,default=129)
        r.add_argument('--seed',type=int,default=20260930);r.add_argument('--duration',type=float)
        r.add_argument('--rain',type=float,help='Storm rainfall in mm/hour')
        r.add_argument('--inlet',type=float,help='Explicit upstream inflow in cubic metres/second')
        r.add_argument('--morphological-factor',type=float,help='Acceleration of bed-exchange rates, recorded in output')
        r.add_argument('--output',type=Path,default=Path('outputs'));r.add_argument('--force',action='store_true')
        if command=='demo':add_render_args(r)
    r=sub.add_parser('render',help='Export/render a previously simulated state, without rerunning erosion')
    r.add_argument('state',type=Path);r.add_argument('--output',type=Path);add_render_args(r)
    return p


def add_render_args(p):
    p.add_argument('--quality',choices=['smoke','preview','production'],default='preview')
    p.add_argument('--threads',type=positive,default=4);p.add_argument('--width',type=positive);p.add_argument('--spp',type=positive)
    p.add_argument('--view',choices=['hero','macro','overhead','profile'],default='hero')
    p.add_argument('--subdivision',type=positive,default=2);p.add_argument('--stones',type=int,default=500)
    p.add_argument('--plots',action='store_true',help='Also write scientific state charts (requires matplotlib)')


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1<<20),b''):h.update(block)
    return h.hexdigest()


def source_hash():
    return hashlib.sha256(b''.join(p.read_bytes() for p in sorted(Path(__file__).parent.glob('*.py')))).hexdigest()


def simulation_source_hash():
    """Material, camera or documentation changes do not rerun a checked storm."""
    root=Path(__file__).parent
    return hashlib.sha256(b''.join((root/name).read_bytes() for name in ('model.py','simulation.py'))).hexdigest()


def state_fingerprint(state):
    h=hashlib.sha256(json.dumps({'config':asdict(state.config),'layers':[asdict(l) for l in state.layers],
                                'time':state.time,'steps':state.steps},sort_keys=True).encode())
    for name in ('foundation','thickness','loose','sediment','water','soil_water','qx','qy',
                 'rain_pattern','inlet_pattern','initial_height'):
        array=np.asarray(getattr(state,name),dtype='<f8',order='C')
        h.update(name.encode());h.update(str(array.shape).encode());h.update(array.tobytes())
    return h.hexdigest()


def geometry_recipe(state,args):
    from mechanism_lab import core,exporters
    root=Path(__file__).parent
    sources=('model.py','geometry.py','materials.py','rendering.py')
    backend=hashlib.sha256(b''.join(Path(module.__file__).read_bytes() for module in (core,exporters))).hexdigest()
    versions={}
    for name in ('numpy','scipy','Pillow','trimesh','numba'):
        versions[name]=importlib.metadata.version(name)
    recipe={'state_sha256':state_fingerprint(state),'geometry_source_sha256':
            hashlib.sha256(b''.join((root/name).read_bytes() for name in sources)).hexdigest(),
            'geometry_backend_sha256':backend,'dependency_versions':versions,
            'subdivision':args.subdivision,'stones':args.stones}
    return hashlib.sha256(json.dumps(recipe,sort_keys=True).encode()).hexdigest(),recipe


def publish_copy(source,destination):
    from cybr_light.runtime import publish
    with tempfile.TemporaryDirectory(prefix='cybr-terrain-alias-') as directory:
        staged=Path(directory)/Path(destination).name
        shutil.copyfile(source,staged);publish(staged,destination)


def geometry_assets(state,out,args):
    """Never replace assets referenced by a completed scene or old receipt."""
    from cybr_light import Scene
    from cybr_light.runtime import publish
    from .rendering import daylight
    key,recipe=geometry_recipe(state,args);assets=out/'assets'/key
    with tempfile.TemporaryDirectory(prefix='cybr-terrain-assets-') as directory:
        staging=Path(directory)
        geo=build_geometry(state,staging,args.subdivision,args.stones)
        geo.export_glb(staging/'terrain.glb');write_json(staging/'geometry.json',geo.validate())
        state.save(staging/'state.npz');daylight(Scene(),staging/'textures')
        generated={str(p.relative_to(staging)):sha(p) for p in staging.rglob('*') if p.is_file()}
        marker=assets/'assets.json'
        if marker.exists():
            retained=json.loads(marker.read_text())
            if retained.get('recipe')!=recipe or retained.get('files')!=generated:
                raise RuntimeError('Immutable asset recipe differs from regenerated geometry: '+str(assets))
            for name,digest in generated.items():
                if not (assets/name).is_file() or sha(assets/name)!=digest:
                    raise RuntimeError('Immutable terrain asset is missing or changed: '+str(assets/name))
        else:
            assets.mkdir(parents=True,exist_ok=True)
            for name,digest in generated.items():
                target=assets/name;target.parent.mkdir(parents=True,exist_ok=True)
                publish(staging/name,target)
            write_json(marker,{'recipe':recipe,'files':generated})
        for atlas in geo.atlases.values():
            for name in ('color_pfm','normal_pfm','roughness_pfm'):
                path=getattr(atlas,name,None)
                if path is not None:setattr(atlas,name,assets/Path(path).relative_to(staging))
        geo.asset_directory=assets
    return geo,assets,key,recipe


def write_json(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    from cybr_light.runtime import publish
    import tempfile
    temporary=None
    try:
        with tempfile.NamedTemporaryFile('w',delete=False,prefix='cybr-terrain-',suffix='.json') as stream:
            temporary=Path(stream.name);json.dump(value,stream,indent=2);stream.write('\n')
        publish(temporary,path)
    finally:
        if temporary is not None:temporary.unlink(missing_ok=True)


def doctor():
    issues=[]
    if not (3,11)<=sys.version_info[:2]<(3,14):issues.append('Use Python 3.11–3.13')
    if not shutil.which('g++'):issues.append('Install g++ with C++17/OpenMP support')
    for package in ('numpy','scipy','Pillow','trimesh','numba','cybr-geo'):
        try:importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:issues.append('Missing package: '+package)
    if issues:raise RuntimeError('; '.join(issues))
    from cybr_light import compile_renderer
    executable,digest=compile_renderer();print('CYBR LIGHT ready:',executable,'source',digest)


def simulate(args,preset):
    defaults={k:v for k,v in PRESETS[preset].items() if k!='description'}
    for key,value in (('duration',args.duration),('rain_mm_hour',args.rain),('inlet_m3_second',args.inlet),('morphological_factor',args.morphological_factor)):
        if value is not None:defaults[key]=value
    c=Config(preset=preset,grid=args.grid,seed=args.seed,**defaults)
    out=args.output/preset;out.mkdir(parents=True,exist_ok=True);state_file=out/'state.npz';receipt=out/'simulation.json'
    key={'config':asdict(c),'solver_sha256':simulation_source_hash()}
    if not args.force and receipt.exists() and state_file.exists():
        try:
            old=json.loads(receipt.read_text())
            if old.get('inputs')==key and old.get('state_sha256')==sha(state_file) and old.get('passed'):
                state=State.load(state_file)
                print('Reusing checked simulation:',preset,flush=True);return state,out
        except (ValueError,OSError) as exc:
            print('Regenerating invalid simulation cache:',preset,'|',exc,flush=True)
    receipt.unlink(missing_ok=True);state=make_terrain(c);initial=out/'initial.npz';state.save(initial)
    sim=Simulator(state);started=time.monotonic()
    def progress(r):print(f"{preset}: {r['time_seconds']:.1f}/{c.duration:g} s; {r['steps']} steps; solids error {r['solid_relative_error']:.2e}",flush=True)
    result=sim.run(progress);state.save(state_file)
    result.update(inputs=key,state_sha256=sha(state_file),initial_state_sha256=sha(initial),elapsed_seconds=time.monotonic()-started,history=sim.history)
    write_json(receipt,result);return state,out


def output(state,out,args):
    out.mkdir(parents=True,exist_ok=True);receipt=out/f'{args.view}-receipt.json';receipt.unlink(missing_ok=True)
    geo,assets,key,recipe=geometry_assets(state,out,args);glb=assets/'terrain.glb'
    publish_copy(glb,out/'terrain.glb');publish_copy(assets/'geometry.json',out/'geometry.json')
    report=render_terrain(geo,out/(args.view+'.png'),args.quality,args.threads,args.width,args.spp,args.view)
    if args.plots:
        from .diagnostics import plot_state
        plot_state(state,out/(args.view+'_diagnostics.png'))
    suffixes=('.png','.json','.log','.cys','.clm','.exr','.pfm','.ppm',
              '_normal.pfm','_albedo.pfm','_depth.pfm','_stderr.pfm','_object.pfm','_position.pfm',
              '_unfiltered.png','_diagnostics.png')
    owned=[out/(args.view+suffix) for suffix in suffixes if (out/(args.view+suffix)).is_file()]
    owned.extend(p for p in assets.rglob('*') if p.is_file())
    state_file=out/'state.npz'
    if state_file.exists():
        try:
            if state_fingerprint(State.load(state_file))==recipe['state_sha256']:owned.append(state_file)
        except ValueError:
            # The checked snapshot in this recipe is authoritative. An old
            # mutable state alias from another run does not belong here.
            pass
    files={str(p.relative_to(out)):sha(p) for p in owned}
    write_json(receipt,{'passed':True,'renderer':report['renderer'],'source_sha256':source_hash(),
                       'recipe_sha256':key,'recipe':recipe,'asset_directory':str(assets.relative_to(out)),
                       'authoritative_glb':str(glb.relative_to(out)),'authoritative_state':str((assets/'state.npz').relative_to(out)),
                       'settings':{'view':args.view,'quality':args.quality,'width':report['width'],'height':report['height'],
                                   'packets':report['packets_per_pixel'],'bands':report['wavelengths_per_packet'],
                                   'subdivision':args.subdivision,'stones':args.stones},'files':files})
    print('CYBR TERRAIN:',out/(args.view+'.png'),'| GLB:',glb,flush=True)


def main(argv=None):
    args=parser().parse_args(argv)
    try:
        if args.command in ('demo','render'):
            from .rendering import QUALITY
            w,h,_,_=QUALITY[args.quality]
            if args.width is not None:
                if not 16<=args.width<=4096:raise ValueError('Width must be 16–4096')
                h=round(args.width*h/w);w=args.width
            if w*h>8000000:raise ValueError('Render exceeds 8 million pixels')
            if args.subdivision not in (1,2,3,4) or not 0<=args.stones<=5000:raise ValueError('Invalid geometry detail budget')
            if args.command=='demo' and (args.grid-1)*args.subdivision+1>1025:raise ValueError('Display grid exceeds 1025 vertices per side')
            if args.plots:
                import importlib.util
                if importlib.util.find_spec('matplotlib') is None:raise ValueError('Install plot support: python -m pip install "cybr-terrain[plots]"')
            # Validate the compiler before a potentially long storm. Native
            # compilation is cached by its full source/compiler digest.
            from cybr_light import compile_renderer
            compile_renderer()
        if args.command=='list':
            for name,p in PRESETS.items():print(name+': '+p['description'])
        elif args.command=='doctor':doctor()
        elif args.command in ('simulate','demo'):
            selected=list(PRESETS) if args.preset=='all' else [args.preset];failed=[]
            for preset in selected:
                try:
                    state,out=simulate(args,preset)
                    if args.command=='demo':output(state,out,args)
                except Exception as exc:
                    failed.append(preset);print(preset+': '+str(exc),file=sys.stderr,flush=True)
            if failed:return 1
        else:
            state=State.load(args.state);out=args.output or args.state.parent;output(state,out,args)
    except (ValueError,RuntimeError,OSError) as exc:
        print('CYBR TERRAIN: '+str(exc),file=sys.stderr);return 1
    return 0


if __name__=='__main__':raise SystemExit(main())
