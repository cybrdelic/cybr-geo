"""One command surface for CAD/mesh creation, rendering, film and drawings."""
from __future__ import annotations
import argparse
import importlib.metadata
import json
import os
from pathlib import Path
import shutil
import sys

AA_CHOICES=['none','fxaa','ssaa2','ssaa3','ssaa4']


def resolution(text):
    try:
        width, height = (int(value) for value in text.lower().split('x'))
    except (ValueError, TypeError):
        raise argparse.ArgumentTypeError('Resolution must be WIDTHxHEIGHT') from None
    if width < 64 or height < 64 or max(width,height) > 16384:
        raise argparse.ArgumentTypeError('Dimensions must be between 64 and 16384 pixels')
    return width, height


def parser():
    p = argparse.ArgumentParser(prog='lab', description=__doc__)
    p.add_argument('--root', type=Path, help='Project containing assets/, outputs/, and native/')
    sub = p.add_subparsers(dest='command', required=True)
    sub.add_parser('list', help='List built-in recipes')
    sub.add_parser('doctor', help='Inspect installed dependencies and system tools')
    q = sub.add_parser('build', help='Build/cache a model and export GLB plus its component list')
    q.add_argument('recipe'); q.add_argument('--step', action='store_true')
    q.add_argument('--stl', action='store_true'); q.add_argument('--rebuild', action='store_true')
    q.add_argument('--out', type=Path)
    q = sub.add_parser('render', help='Render a named view from actual geometry')
    q.add_argument('recipe'); q.add_argument('--view', default='hero')
    q.add_argument('--renderer', choices=['pbr','pathtrace'], default='pbr')
    q.add_argument('--size', type=resolution, default=(1600,1100))
    q.add_argument('--aa', choices=AA_CHOICES, default='ssaa2', help='PBR AA mode; path tracing uses stratified stochastic subpixel rays')
    q.add_argument('--spp', type=int, default=64); q.add_argument('--threads', type=int, default=4)
    q.add_argument('--time', type=float, default=0.); q.add_argument('--out', type=Path)
    q = sub.add_parser('video', help='Render every video frame using model poses and a shot list')
    q.add_argument('recipe'); q.add_argument('--shots', type=Path, help='JSON array of Shot objects')
    q.add_argument('--view', default='hero'); q.add_argument('--seconds', type=float, default=8.)
    q.add_argument('--action', choices=['motion','orbit','explode','still'], default='motion')
    q.add_argument('--size', type=resolution, default=(1280,720)); q.add_argument('--fps', type=int, default=24)
    q.add_argument('--aa', choices=AA_CHOICES, default='ssaa2')
    q.add_argument('--out', type=Path)
    q = sub.add_parser('gif', help='Make a palette-optimized looping GIF from a video')
    q.add_argument('video', type=Path); q.add_argument('--out', type=Path, required=True)
    q.add_argument('--start', type=float, default=0.); q.add_argument('--seconds', type=float, default=6.)
    q.add_argument('--width', type=int, default=640); q.add_argument('--fps', type=int, default=10)
    q = sub.add_parser('animate', help='Export a standard node-animated GLB')
    q.add_argument('recipe'); q.add_argument('--seconds', type=float, default=8.)
    q.add_argument('--fps', type=int, default=24); q.add_argument('--mode', choices=['motion','explode'], default='motion')
    q.add_argument('--out', type=Path)
    q = sub.add_parser('catalogue', help='Render/export every named part and an offline HTML index')
    q.add_argument('recipe'); q.add_argument('--size', type=resolution, default=(640,480)); q.add_argument('--aa', choices=AA_CHOICES, default='ssaa2'); q.add_argument('--out', type=Path)
    q = sub.add_parser('blueprint', aliases=['whiteprint'], help='Create white-background SVG/PDF/DXF drawings')
    q.add_argument('recipe'); q.add_argument('--part', action='append', help='Select exact part name; repeatable')
    q.add_argument('--annotations', type=Path, help='Recipe-independent JSON dimension/leader anchors')
    q.add_argument('--scale', type=float); q.add_argument('--title'); q.add_argument('--out', type=Path)
    q = sub.add_parser('validate', help='Check finite/indexed/closed geometry and rigid poses')
    q.add_argument('recipe'); q.add_argument('--expensive', action='store_true'); q.add_argument('--out', type=Path)
    q = sub.add_parser('import', help='Write a portable geometry-import recipe and copy its input')
    q.add_argument('source', type=Path); q.add_argument('--name', required=True)
    q.add_argument('--units', choices=['mm','cm','m','inch'])
    q.add_argument('--up-axis', choices=['Y','Z']); q.add_argument('--out', type=Path)
    q = sub.add_parser('pack', help='Package all source, legacy assets and new outputs with checksums')
    q.add_argument('--out', type=Path, required=True); q.add_argument('--source-only', action='store_true')
    return p


def doctor():
    libraries = ['numpy','cadquery','vtk','trimesh','shapely','scipy','Pillow','numba','ezdxf','CairoSVG']
    dependencies = {}
    for name in libraries:
        try:
            dependencies[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            dependencies[name] = None
    programs = {name: shutil.which(name) for name in ['ffmpeg','ffprobe','cmake','g++','git']}
    from .core import project_root
    root = project_root()
    report = {'python': sys.version, 'project_root': str(root), 'dependencies': dependencies,
              'programs': programs, 'preserved_inputs_present': (root/'assets/differential_v3/geometry/reference_parts.npz').exists(),
              'notes': 'VTK needs an EGL-capable graphics stack (Mesa software rendering is sufficient). '
                       'CairoSVG needs the system Cairo library. Native path tracing additionally requires a C++17 compiler and OpenMP.'}
    print(json.dumps(report, indent=2))
    return 0 if all(dependencies.values()) and programs['ffmpeg'] and programs['ffprobe'] else 2


def run(args):
    if args.root:
        os.environ['MECHANISM_LAB_ROOT'] = str(args.root.expanduser().resolve())
    from .core import project_root, validate
    from .registry import load, BUILTINS
    if args.command == 'list':
        print('\n'.join(BUILTINS)); return 0
    if args.command == 'doctor':
        return doctor()
    root = project_root()
    if args.command == 'pack':
        from .packaging import pack
        print(json.dumps(pack(root,args.out,args.source_only),indent=2)); return 0
    if args.command == 'gif':
        from .media import make_gif
        print(make_gif(args.video,args.out,args.width,args.fps,args.seconds,args.start)); return 0
    if args.command == 'import':
        from .importers import import_geometry, safe_name
        source=args.source.expanduser().resolve()
        # Validate before persisting the recipe and avoid silently guessing STL units.
        assembly=import_geometry(source,args.name,args.units,args.up_axis)
        destination=args.out or root/'examples/imported'/safe_name(args.name)
        destination.mkdir(parents=True,exist_ok=True)
        copied=destination/source.name
        if copied.exists() and copied.resolve()!=source:
            raise FileExistsError(f'Refusing to overwrite imported source: {copied}')
        if copied.resolve()!=source:
            shutil.copy2(source,copied)
        if source.suffix.lower()=='.gltf':
            doc=json.loads(source.read_text())
            for entry in doc.get('buffers',[])+doc.get('images',[]):
                uri=entry.get('uri','')
                if uri and not uri.startswith('data:'):
                    if '://' in uri or Path(uri).is_absolute() or '..' in Path(uri).parts:
                        raise ValueError('Portable glTF imports require local relative buffers/images without parent traversal')
                    target=destination/uri;target.parent.mkdir(parents=True,exist_ok=True)
                    if (source.parent/uri).resolve()!=target.resolve():shutil.copy2(source.parent/uri,target)
        recipe=destination/(safe_name(args.name)+'.json')
        recipe.write_text(json.dumps({'schema':1,'kind':'geometry-import','source':source.name,
                                     'name':assembly.name,'units':assembly.metadata['source_units'],
                                     'up_axis':assembly.metadata['source_up_axis']},indent=2)+'\n')
        print(recipe); return 0
    analytic=args.command in ('blueprint','whiteprint') or (args.command=='build' and args.step)
    assembly=load(args.recipe,rebuild=getattr(args,'rebuild',False),analytic=analytic)
    out=root/'outputs'/assembly.name
    out.mkdir(parents=True,exist_ok=True)
    if args.command=='build':
        from .exporters import export_glb,export_step,export_bom,export_stls
        out=args.out or out;out.mkdir(parents=True,exist_ok=True)
        export_glb(assembly,out/(assembly.name+'.glb'));export_bom(assembly,out)
        if args.step:export_step(assembly,out/(assembly.name+'_analytic.step'))
        if args.stl:export_stls(assembly,out/'stl')
        print(json.dumps({'model':assembly.name,'parts':len(assembly.parts),'output':str(out)},indent=2))
    elif args.command=='render':
        output=args.out or out/'renders'/(args.view+'_'+args.renderer+'.png')
        if args.renderer=='pathtrace':
            if args.time!=0:raise ValueError('Offline pathtrace views currently use time zero; use PBR for a nonzero animation time')
            from .pathtrace import render_pathtrace
            render_pathtrace(assembly,output,args.view,args.size,args.spp,args.threads)
        else:
            from .render import render_still
            render_still(assembly,output,args.view,args.size,args.time,aa=args.aa)
        print(output)
    elif args.command=='video':
        from .media import Shot,render_video
        shots=[Shot(**record) for record in json.loads(args.shots.read_text())] if args.shots else [Shot(args.view,args.seconds,args.action)]
        output=args.out or out/'videos'/(assembly.name+'.mp4')
        report=render_video(assembly,output,shots,args.size,args.fps,args.aa)
        print(json.dumps({k:v for k,v in report.items() if k!='frames_log'},indent=2))
    elif args.command=='animate':
        from .exporters import export_animated_glb
        output=args.out or out/(assembly.name+'_'+args.mode+'.glb')
        print(json.dumps(export_animated_glb(assembly,output,args.seconds,args.fps,args.mode),indent=2))
    elif args.command=='catalogue':
        from .media import catalogue
        output=args.out or out/'parts';items=catalogue(assembly,output,args.size,args.aa)
        print(json.dumps({'parts':len(items),'catalogue':str(output/'index.html')},indent=2))
    elif args.command in ('blueprint','whiteprint'):
        from .whiteprint import whiteprint
        output=args.out or out/'drawings'/(assembly.name+'_whiteprint')
        specification=json.loads(args.annotations.read_text()) if args.annotations else None
        print(json.dumps(whiteprint(assembly,output,args.part,args.title,args.scale,specification),indent=2))
    elif args.command=='validate':
        report=validate(assembly,args.expensive);text=json.dumps(report,indent=2)+'\n'
        if args.out:args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(text)
        print(text)
    return 0


def main(argv=None):
    args=parser().parse_args(argv)
    try:
        return run(args)
    except (ValueError,FileNotFoundError,FileExistsError,RuntimeError,KeyError) as error:
        print(f'lab: {error}',file=sys.stderr)
        return 2

if __name__=='__main__':
    raise SystemExit(main())
