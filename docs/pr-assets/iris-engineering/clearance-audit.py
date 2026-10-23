"""Read-only BRep audit of a completed IRIS run's actual recorded poses."""
import argparse, faulthandler, gzip, hashlib, itertools, json, math, time
from pathlib import Path
import cadquery as cq
import numpy as np
from mechanism_lab.physical_model import digest

faulthandler.enable()
parser=argparse.ArgumentParser()
parser.add_argument('--source',type=Path,default=Path('outputs/engineering-iris09-derived'))
parser.add_argument('--native',type=Path,help='Completed native build directory (overrides --source/native)')
parser.add_argument('--trace',type=Path,help='Integrated trace.json.gz (overrides --source/simulation/trace.json.gz)')
parser.add_argument('--out',type=Path,default=Path('outputs/iris-mechanism-audit'))
parser.add_argument('--times',nargs='+',type=float,default=[0,1,2,3,4,5,6,7,8])
args=parser.parse_args(); args.out.mkdir(parents=True,exist_ok=True)
native=args.native or args.source/'native'
receipt_path=native/'native-build.json'
binding_path=native/'native-source-binding.json'
trace_path=args.trace or args.source/'simulation/trace.json.gz'
receipt_bytes=receipt_path.read_bytes(); binding_bytes=binding_path.read_bytes(); trace_bytes=trace_path.read_bytes()
receipt=json.loads(receipt_bytes);binding=json.loads(binding_bytes)
trace=json.loads(gzip.decompress(trace_bytes))
assert digest(trace['physical_source'])==trace['model_sha256']
assert trace['physical_source']['provenance']['native_build_sha256']==digest(receipt)
assert trace['physical_source']['provenance']['native_source_binding_sha256']==digest(binding)
for rel,wanted in receipt['files_sha256'].items():
    assert hashlib.sha256((native/rel).read_bytes()).hexdigest()==wanted,rel
placements=json.loads((native/'placed-native-parts.json').read_text())['records']
parts=[]
for row in placements:
    shape=cq.importers.importStep(str(native/row['step_file'])).val()
    assert shape.isValid() and len(shape.Solids())>0 and shape.Volume()>0,row
    parts.append(dict(**row,shape=shape,volume_mm3=float(shape.Volume()),solids=len(shape.Solids())))
    (args.out/'stage.json').write_text(json.dumps(dict(stage='native-load',loaded_parts=len(parts),total_parts=len(placements))))
print(f'Loaded {len(parts)} placed native parts across {len({p["body"] for p in parts})} bodies',flush=True)
def bounds(shape):
    b=shape.BoundingBox(); return np.array([b.xmin,b.ymin,b.zmin]),np.array([b.xmax,b.ymax,b.zmax])
def place(shape,pose):
    rotation=np.asarray(pose['rotation_row_major']).reshape(3,3)
    assert np.allclose(rotation.T@rotation,np.eye(3),rtol=0,atol=1e-10)
    assert np.allclose(rotation[:,2],[0,0,1],rtol=0,atol=1e-10),'Audit only admits this declared planar mechanism'
    yaw=math.atan2(rotation[1,0],rotation[0,0])
    location=cq.Location(cq.Vector(*(np.asarray(pose['position_m'])*1000)),cq.Vector(0,0,1),math.degrees(yaw))
    return shape.moved(location)
pair_indices=[(i,j) for i,j in itertools.combinations(range(len(parts)),2) if parts[i]['body']!=parts[j]['body']]
frames=[];seen=set()
for t in args.times:
    i=min(range(len(trace['frames'])),key=lambda n:abs(trace['frames'][n]['time_s']-t))
    if i not in seen:frames.append(trace['frames'][i]);seen.add(i)
identity=dict(native_directory=str(native),trace_path=str(trace_path),
    audit_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    trace_sha256=hashlib.sha256(trace_bytes).hexdigest(),
    native_build_sha256=digest(receipt),native_build_file_sha256=hashlib.sha256(receipt_bytes).hexdigest(),
    native_source_binding_sha256=digest(binding),model_sha256=trace['model_sha256'],
    recorded_body_poses_sha256=digest([dict(time_s=f['time_s'],bodies=f['bodies']) for f in trace['frames']]),
    sampled_body_poses_sha256=digest([dict(time_s=f['time_s'],bodies=f['bodies']) for f in frames]))
(args.out/'input-identity.json').write_text(json.dumps(identity,indent=2))
all_reports=[];overlaps=[];touches=[];min_distance=math.inf;nearest=None
started=time.perf_counter()
for frame in frames:
    posed=[place(p['shape'],frame['bodies'][p['body']]) for p in parts]
    boxes=[bounds(s) for s in posed]
    measured=0;skipped=0;booleans=0;frame_overlaps=[];frame_touches=[];close=[]
    closest=math.inf
    for i,j in pair_indices:
        alo,ahi=boxes[i];blo,bhi=boxes[j]
        gap=float(np.linalg.norm(np.maximum(np.maximum(alo-bhi,blo-ahi),0.)))
        # A separated conservative AABB proves nonintersection. Measure all
        # potentially touching/near pairs and enough others to find the true
        # minimum cross-body separation at this sampled pose.
        if gap>max(.1,closest):skipped+=1;continue
        distance=float(posed[i].distance(posed[j])); measured+=1
        closest=min(closest,distance)
        record=dict(time_s=frame['time_s'],parts=[parts[k]['body']+'/'+parts[k]['name'] for k in(i,j)],distance_mm=distance)
        if distance<min_distance:min_distance=distance;nearest=record
        if distance<=.1:close.append(record)
        if distance<1e-7:
            intersection=posed[i].intersect(posed[j]); booleans+=1
            volume=float(intersection.Volume())
            record['intersection_volume_mm3']=volume
            if volume>1e-8:
                overlaps.append(record);frame_overlaps.append(record)
                tag=f'overlap-{len(overlaps):03}'
                cq.exporters.export(intersection,str(args.out/(tag+'.step')))
                with (args.out/'intersections.jsonl').open('a') as stream:
                    stream.write(json.dumps(dict(**record,artifact=tag+'.step'))+'\n')
            else:touches.append(record);frame_touches.append(record)
    item=dict(time_s=frame['time_s'],cross_body_pairs=len(pair_indices),aabb_pruned=skipped,
        native_distances=measured,native_intersections=booleans,minimum_distance_mm=closest,
        overlaps=frame_overlaps,touches=frame_touches,pairs_within_0_1mm=close)
    all_reports.append(item)
    (args.out/'progress.json').write_text(json.dumps(all_reports,indent=2))
    (args.out/'stage.json').write_text(json.dumps(dict(stage='pose-complete',completed_poses=len(all_reports),total_poses=len(frames),time_s=frame['time_s'])))
    print(json.dumps({k:v for k,v in item.items() if k not in('pairs_within_0_1mm',)}),flush=True)
report=dict(schema='cybrgeo-sampled-native-clearance-audit/1',
    scope='Actual recorded integrated poses and completed native STEP solids; samples do not establish a continuous articulated motion envelope.',
    identity=identity,
    method='All differently named physical-body pairs; conservative AABB pruning then exact BRep distance, then BRep intersection for zero-distance candidates. Same-body rigid joins omitted.',
    tolerances=dict(zero_distance_mm=1e-7,numerical_intersection_volume_mm3=1e-8),
    native_parts=len(parts),native_solids=sum(p['solids'] for p in parts),native_artifacts_verified=len(receipt['files_sha256']),
    part_inventory=[{k:v for k,v in p.items() if k!='shape'} for p in parts],
    requested_sample_times_s=args.times,
    pose_count=len(frames),cross_body_pairs_per_pose=len(pair_indices),elapsed_s=time.perf_counter()-started,
    minimum_distance_mm=min_distance,nearest_pair=nearest,unexpected_intersections=overlaps,zero_volume_contacts=touches,
    no_material_intersections=not overlaps,positive_separation=not overlaps and not touches,
    poses=all_reports,qualification='Nominal rigid CAD and sampled poses only; no supplier fits, wear, structural/foil stiffness, fatigue or hardware qualification')
(args.out/'native-clearance.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k not in ('poses','part_inventory')},indent=2),flush=True)

