"""Build, audit, export and render ORBIT's genuine ordered service process.

python tools/orbit_service.py build --out outputs/orbit_service
python tools/orbit_service.py validate --out outputs/orbit_service --jobs 6
python tools/orbit_service.py export --out outputs/orbit_service
python tools/orbit_service.py stills --out outputs/orbit_service
"""
from __future__ import annotations
import argparse,hashlib,importlib.util,json,math,subprocess
from concurrent.futures import ProcessPoolExecutor,as_completed
from dataclasses import replace,asdict
from pathlib import Path
import numpy as np
from mechanism_lab.core import load_cache,save_cache,validate
from mechanism_lab.assembly_process import validate_process

ROOT=Path(__file__).resolve().parents[1]


def recipe(name):
    spec=importlib.util.spec_from_file_location(name,ROOT/'examples'/(name+'.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def load_model(out,analytic=False):
    a=load_cache(Path(out)/'cache')
    a.motion_function=recipe('orbit_inspection_wrist').bind_service_motion(a)
    if analytic:
        import cadquery as cq
        for p in a.parts:p.cad=cq.Shape.importBrep(str(Path(out)/'brep'/(p.name+'.brep')))
    return a


def static_audit(a):
    rows=[];tests=0;bounds={}
    for p in a.parts:
        b=p.cad.BoundingBox();bounds[p.name]=np.array([[b.xmin,b.ymin,b.zmin],[b.xmax,b.ymax,b.zmax]])
    for i,p in enumerate(a.parts):
        for q in a.parts[i+1:]:
            pb,qb=bounds[p.name],bounds[q.name]
            if np.any(np.minimum(pb[1],qb[1])-np.maximum(pb[0],qb[0])<=1e-6):continue
            tests+=1;v=sum(abs(s.Volume()) for s in p.cad.intersect(q.cad).Solids())
            if v>1e-4:rows.append(dict(a=p.name,b=q.name,overlap_mm3=v))
    return dict(passed=not rows,cad_tests=tests,collisions=rows,volume_tolerance_mm3=1e-4)


def worker_init(out):
    global _assembly,_process,_out
    _out=Path(out);_assembly=load_model(_out,True)
    _process=recipe('orbit_service_process').procedure(_assembly)


def worker_check(job):
    i,step,angle=job
    result=validate_process(_assembly,_process,step,angle,operation_indices=[i])
    dest=_out/'operation-checks'/f'{i:04d}.json';temp=dest.with_suffix('.tmp')
    temp.write_text(json.dumps(result,indent=2)+'\n');temp.replace(dest)
    return i,result


def run_validation(out,jobs,step,angle):
    a=load_model(out,True);process=recipe('orbit_service_process').procedure(a)
    process.write(out/'ORBIT_service_procedure.json')
    (out/'operation-checks').mkdir(exist_ok=True)
    records=[]
    with ProcessPoolExecutor(max_workers=jobs,initializer=worker_init,initargs=(str(out),)) as pool:
        futures=[pool.submit(worker_check,(i,step,angle)) for i in range(len(process.moves))]
        for future in as_completed(futures):
            i,result=future.result();records.append((i,result))
            print(f'Operation {i+1}/{len(process.moves)}: {"PASS" if result["passed"] else "FAIL"}',flush=True)
    report={k:[] for k in ('operations','collisions','tool_obstructions','floor_collisions','unchecked_mesh_parts')}
    for _,record in sorted(records):
        for k in report:report[k].extend(record[k])
    report.update(passed=all(r['passed'] for _,r in records),linear_step_mm=step,angular_step_degrees=angle,
                  checked_operations=len(records),total_operations=len(process.moves),
                  continuous_motion_certificate=False,forces_simulated=False,
                  preconditions=['Unload the sample; fully open the jaws; set the wrist to its neutral angle.',
                                 'Support the released assembly by hand or with a suitable fixture before extracting its last retainer.'])
    (out/'ORBIT_service_validation.json').write_text(json.dumps(report,indent=2)+'\n')
    if not report['passed']:raise ValueError('Service validation failed; inspect the recorded geometry and tool obstructions')
    return report


def service_keyframes(process):
    # Preserve every operation boundary. Extra helical samples prevent the
    # quaternion shortest-path interpolation from aliasing complete turns.
    times=[0.]
    for start,move in zip(process._times,process.moves):
        intervals=max(4,math.ceil(np.linalg.norm(move.delta)/4),math.ceil(abs(move.turns)*8))
        times.extend(start+move.duration*np.linspace(0,1,intervals+1)[1:])
    return np.asarray(times)


def service_films(a,out,args):
    from mechanism_lab.photoreal import render_photoreal_video,compile_renderer
    from mechanism_lab.media import Shot,probe
    process=recipe('orbit_service_process').procedure(a);work=out/'film-work';work.mkdir(exist_ok=True)
    size=tuple(map(int,args.size.split('x')));parts={p.name:p for p in a.parts}
    choices=[
        ('Withdraw the complete gripper',None,175,31,24,'Withdraw gripper and harness','Feed harness and seat gripper'),
        ('Release split bearing cover 1',None,92,31,24,'Release the split front cover','Seat the split front cover'),
        ('Withdraw spindle, gear',None,110,40,26,'Withdraw the spindle service unit','Insert the spindle service unit'),
        ('Slide the wrist gear',('W01_',),65,32,30,'Slide the gear over the shaft key','Slide the keyed gear onto the shaft'),
        ('Slide the bush-lined finger',('P01_',),88,24,30,'Slide the finger off its captive nut','Seat the finger over its captive nut'),
        ('Unthread left handed',('L02_',),52,30,34,'Unthread the left-hand nut','Thread the left-hand nut onto the screw'),
    ]
    rows=[];clips=[];last_time=(round(args.seconds*args.fps)-1)/args.fps
    source_key=hashlib.sha256(Path(__file__).read_bytes()+
        (ROOT/'examples/orbit_inspection_wrist.py').read_bytes()+
        (ROOT/'examples/orbit_service_process.py').read_bytes()+
        compile_renderer().read_bytes()).hexdigest()
    for shot_index,(prefix,context,scale,az,el,label,reverse_label) in enumerate(choices):
        index=next(i for i,m in enumerate(process.moves) if m.title.startswith(prefix));move=process.moves[index]
        chosen=list(move.parts)
        if context:chosen.extend(n for n in parts if n.startswith(context))
        points=[]
        for u in (0.,1.):
            states=process.at_move(index,u)
            for name in chosen:
                b=parts[name].bounds;T=states[name]
                corners=np.array([[x,y,z] for x in b[:,0] for y in b[:,1] for z in b[:,2]])
                points.extend(corners@T[:3,:3].T+T[:3,3])
        points=np.asarray(points);target=tuple((points.min(0)+points.max(0))/2)
        if shot_index<3:
            # Keep the fixture in the composition during removal.
            target=((110.,0.,62.) if shot_index==0 else (38.,0.,67.))
        view=replace(a.views['hero'],az=az,el=el,scale=scale,target=target,
                     f_stop=22.,studio_target=target,studio_scale=1.2,
                     studio_az=az,studio_el=el,floor_z_mm=-2.,hide=(),explode=0.)
        cached=[None,None]
        def pose(part,t,e,operation=index):
            u=float(np.clip(t/last_time,0,1));u=u*u*(3-2*u)
            if u!=cached[0]:cached[:]=[u,process.at_move(operation,u)]
            return cached[1][part.name]
        scene=replace(a,motion_function=pose,views={'service':view})
        clip=work/f'{shot_index:02d}.mp4';settings=work/f'{shot_index:02d}.settings.json'
        key=dict(source=source_key,operation=move.id,size=list(size),fps=args.fps,
                 seconds=args.seconds,spp=args.spp,view=asdict(view))
        if not(clip.exists() and settings.exists() and json.loads(settings.read_text())==json.loads(json.dumps(key))):
            render_photoreal_video(scene,clip,[Shot('service',args.seconds,'motion')],size,args.fps,args.spp,
                                   args.threads,12,shutter_angle=0.,shutter_samples=1,intent='concept')
            settings.write_text(json.dumps(key,indent=2)+'\n')
        clips.append(clip)
        rows.append(dict(operation_id=move.id,title=move.title,disassembly_caption=label,
                         assembly_caption=reverse_label,source_start_seconds=process._times[index],
                         source_end_seconds=process._times[index]+move.duration,camera=asdict(view)))
    concat=work/'concat.txt';concat.write_text(''.join("file '"+str(p)+"'\n" for p in clips))
    raw=work/'highlights.mp4'
    subprocess.run(['ffmpeg','-y','-v','error','-xerror','-f','concat','-safe','0','-i',str(concat),'-c','copy',str(raw)],check=True)
    for reverse in (False,True):
        direction='assembly' if reverse else 'disassembly';ordered=list(reversed(rows)) if reverse else rows
        filters=['reverse'] if reverse else []
        filters.append('drawbox=x=0:y=ih-78:w=iw:h=78:color=black@0.65:t=fill')
        for i,row in enumerate(ordered):
            title=work/f'{direction}_{i}.txt'
            title.write_text(f'{i+1:02d} / '+row[f'{direction}_caption'])
            filters.append(f"drawtext=textfile='{title}':fontcolor=white:fontsize=25:x=28:y=h-58:enable='between(t,{i*args.seconds},{(i+1)*args.seconds-1/args.fps})'")
        output=out/f'ORBIT_v3_{direction}.mp4'
        subprocess.run(['ffmpeg','-y','-v','error','-xerror','-i',str(raw),'-vf',','.join(filters),
                        '-an','-c:v','libx264','-threads','2','-preset','slow','-crf','15',
                        '-pix_fmt','yuv420p','-movflags','+faststart',str(output)],check=True)
        subprocess.run(['ffmpeg','-v','error','-xerror','-i',str(output),'-f','null','-'],check=True)
        report=dict(file=output.name,renderer='CYBR GEO shared native photographic renderer',
                    direction=direction,resolution=list(size),spp=args.spp,fps=args.fps,
                    selected_operations=ordered,full_procedure_operations=len(process.moves),
                    editorial_note='Selected service moves; cuts omit screw removal and bench transfers. The complete ordered process is supplied as JSON and animated GLB.',
                    reverse_uses_identical_validated_geometry=reverse,motion_blur=False,frame_interpolation=False,probe=probe(output))
        output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=['build','validate','export','stills','films'])
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--jobs',type=int,default=6);parser.add_argument('--step',type=float,default=1.)
    parser.add_argument('--angle',type=float,default=15.);parser.add_argument('--spp',type=int,default=512)
    parser.add_argument('--size',default='1920x1440');parser.add_argument('--threads',type=int,default=8)
    parser.add_argument('--fps',type=int,default=24);parser.add_argument('--seconds',type=float,default=1.5)
    parser.add_argument('--views',nargs='+',default=['hero','macro','internal','rear'])
    args=parser.parse_args();out=args.out.resolve();out.mkdir(parents=True,exist_ok=True)
    if args.command=='build':
        a=recipe('orbit_inspection_wrist').build();save_cache(a,out/'cache');(out/'brep').mkdir(exist_ok=True)
        for p in a.parts:p.cad.exportBrep(str(out/'brep'/(p.name+'.brep')))
        geometry=validate(a,True);(out/'geometry-validation.json').write_text(json.dumps(geometry,indent=2)+'\n')
        if any(not p['watertight'] or not p['consistent_winding'] or p['analytic_valid'] is not True for p in geometry['checks']):
            raise ValueError('Invalid analytic or tessellated geometry')
        report=static_audit(a);(out/'static-collisions.json').write_text(json.dumps(report,indent=2)+'\n')
        if not report['passed']:raise ValueError('Static solid overlaps found')
    elif args.command=='validate':run_validation(out,args.jobs,args.step,args.angle)
    elif args.command=='export':
        from mechanism_lab.exporters import export_step,export_glb,export_bom,export_animated_glb
        a=load_model(out,True);process=recipe('orbit_service_process').procedure(a)
        process.write(out/'ORBIT_service_procedure.json')
        export_step(a,out/'ORBIT_serviceable.step',individual=False)
        export_glb(a,out/'ORBIT_serviceable.glb');export_glb(a,out/'ORBIT_disassembled.glb',explode=1.)
        export_animated_glb(process.assembly(a),out/'ORBIT_service_animation.glb',duration=process.duration,
                            sample_times=service_keyframes(process))
        export_bom(a,out/'components')
    elif args.command=='films':service_films(load_model(out),out,args)
    else:
        from mechanism_lab.photoreal import render_photoreal
        a=load_model(out,True);held=recipe('orbit_inspection_wrist').with_inspection_coupon(a)
        size=tuple(map(int,args.size.split('x')))
        for view in args.views:
            scene=held if view in ('hero','macro') else a
            render_photoreal(scene,out/f'ORBIT_v3_{view}.png',view,size,args.spp,args.threads,14,intent='concept')
            print('Rendered',view,flush=True)


if __name__=='__main__':main()
