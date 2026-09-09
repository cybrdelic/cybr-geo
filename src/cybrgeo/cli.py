"""Model plugins supply geometry; common commands supply every deliverable."""
from pathlib import Path
import argparse,importlib.util,json,sys
from .core import Assembly

def plugin(path):
    path=Path(path).resolve();spec=importlib.util.spec_from_file_location('cybrgeo_user_model',path)
    if spec is None or spec.loader is None:raise ValueError('Unable to load model plugin')
    module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    if not callable(getattr(module,'build',None)):raise ValueError('Plugin must implement build() -> Assembly')
    return module

def main(argv=None):
    p=argparse.ArgumentParser(prog='cybrgeo');sub=p.add_subparsers(dest='command',required=True)
    q=sub.add_parser('import-step');q.add_argument('step');q.add_argument('--out',required=True);q.add_argument('--axis',default='native',choices=['native','motor-x']);q.add_argument('--tolerance',type=float,default=.08)
    q=sub.add_parser('build');q.add_argument('plugin');q.add_argument('--out',required=True)
    q=sub.add_parser('render');q.add_argument('scene');q.add_argument('--out',required=True);q.add_argument('--explode',type=float,default=0);q.add_argument('--section',choices=['all','shell']);q.add_argument('--size',default='1600x1100');q.add_argument('--backend',default='pbr',choices=['pbr','pathtrace']);q.add_argument('--samples',type=int,default=64)
    q=sub.add_parser('video');q.add_argument('scene');q.add_argument('--out',required=True);q.add_argument('--seconds',type=float,default=6);q.add_argument('--fps',type=int,default=24);q.add_argument('--mode',choices=['orbit','explode'],default='orbit')
    q=sub.add_parser('parts');q.add_argument('scene');q.add_argument('--out',required=True)
    q=sub.add_parser('whiteprint');q.add_argument('scene');q.add_argument('--out',required=True);q.add_argument('--part');q.add_argument('--title',default='CAD WHITEPRINT');q.add_argument('--number',default='CYBR-0001')
    q=sub.add_parser('validate');q.add_argument('scene');q.add_argument('--out')
    args=p.parse_args(argv)
    if args.command=='import-step':
        from .cad import import_step
        a=import_step(args.step,args.tolerance,args.axis);a.save(args.out);a.export_glb(Path(args.out)/'assembly.glb');return
    if args.command=='build':
        a=plugin(args.plugin).build()
        if not isinstance(a,Assembly):raise TypeError('build() did not return an Assembly')
        a.save(args.out);a.export_glb(Path(args.out)/'assembly.glb');return
    a=Assembly.load(args.scene,with_cad=args.command=='whiteprint')
    if args.command=='validate':
        text=json.dumps(a.validate(),indent=2)
        if args.out:Path(args.out).write_text(text)
        print(text);return
    if args.command=='render':
        from .render import Studio,labelled
        size=tuple(map(int,args.size.split('x')))
        if args.backend=='pathtrace':
            from .offline import render
            import numpy as np
            if args.section:raise ValueError('Path-traced section requires a pre-sectioned mesh scene')
            from .core import translation
            poses={p.name:translation(p.explode*args.explode) for p in a.parts}
            render(a,args.out,*size,samples=args.samples,poses=poses,camera=(235,23,float(np.linalg.norm(np.ptp(a.bounds,axis=0))*.65*(1+args.explode)),a.bounds.mean(0).tolist()))
            return
        with Studio(a,size,section=args.section) as s:
            s.pose(explode=args.explode);s.fit(padding=1.35+args.explode)
            Path(args.out).parent.mkdir(parents=True,exist_ok=True)
            labelled(s.render(),a.name).save(args.out)
    elif args.command=='video':
        from .media import video
        import numpy as np
        b=a.bounds;rad=np.linalg.norm(np.ptp(b,axis=0))*.65
        def traj(t,u):
            if args.mode=='orbit':return {'camera':(235+360*u,23,rad,b.mean(0))}
            ex=np.sin(np.pi*u)**2
            return {'explode':ex,'camera':(235,23,rad*(1+ex),b.mean(0))}
        video(a,Path(args.out),traj,args.seconds,args.fps,title=a.name)
    elif args.command=='parts':
        from .media import catalogue
        a.export_parts(Path(args.out)/'models');catalogue(a,args.out)
    elif args.command=='whiteprint':
        import cadquery as cq
        from .whiteprint import write_whiteprint
        shapes=[s for n,s in a.cad.items() if args.part is None or n==args.part]
        if not shapes:raise ValueError('No analytic B-rep parts available; use an imported STEP or CAD plugin')
        write_whiteprint(cq.Compound.makeCompound(shapes),args.out,args.title,args.number)
if __name__=='__main__':main()
