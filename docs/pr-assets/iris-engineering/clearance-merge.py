"""Merge disjoint native audit samples only after verifying identical inputs."""
import argparse, gzip, hashlib, json
from pathlib import Path
from mechanism_lab.physical_model import digest

parser=argparse.ArgumentParser()
parser.add_argument('--receipts',nargs='+',required=True,type=Path)
parser.add_argument('--out',required=True,type=Path)
parser.add_argument('--expected-times',nargs='+',type=float,default=list(range(9)))
args=parser.parse_args()
reports=[json.loads(path.read_text()) for path in args.receipts]
for report in reports:
    assert report['schema']=='cybrgeo-sampled-native-clearance-audit/1'
identity=dict(reports[0]['identity'])
identity.pop('sampled_body_poses_sha256')
for report in reports[1:]:
    other=dict(report['identity']); other.pop('sampled_body_poses_sha256')
    assert other==identity,'Cannot merge differing native, trace, model or audit-script identities'
trace_path=Path(identity['trace_path'])
trace_bytes=trace_path.read_bytes()
assert hashlib.sha256(trace_bytes).hexdigest()==identity['trace_sha256']
trace=json.loads(gzip.decompress(trace_bytes))
frames=[min(trace['frames'],key=lambda frame:abs(frame['time_s']-t)) for t in args.expected_times]
expected_times=[frame['time_s'] for frame in frames]
assert len(expected_times)==len(set(expected_times))
poses=sorted([pose for report in reports for pose in report['poses']],key=lambda pose:pose['time_s'])
assert [pose['time_s'] for pose in poses]==sorted(expected_times),'Missing or duplicate requested poses'
for report in reports:
    for key in ('part_inventory','native_parts','native_solids','native_artifacts_verified','tolerances','cross_body_pairs_per_pose'):
        assert report[key]==reports[0][key],key
    assert report['pose_count']==len(report['poses'])
identity['sampled_body_poses_sha256']=digest([dict(time_s=f['time_s'],bodies=f['bodies']) for f in sorted(frames,key=lambda f:f['time_s'])])
overlaps=[row for pose in poses for row in pose['overlaps']]
touches=[row for pose in poses for row in pose['touches']]
result=dict(reports[0])
result.update(identity=identity,pose_count=len(poses),requested_sample_times_s=args.expected_times,poses=poses,
    cross_body_pair_evaluations=sum(p['cross_body_pairs'] for p in poses),
    native_distance_evaluations=sum(p['native_distances'] for p in poses),
    aabb_exclusions=sum(p['aabb_pruned'] for p in poses),
    native_intersection_evaluations=sum(p['native_intersections'] for p in poses),
    elapsed_s=sum(r['elapsed_s'] for r in reports),elapsed_scope='Sum of worker times; workers may execute concurrently',
    minimum_distance_mm=min(r['minimum_distance_mm'] for r in reports),
    nearest_pair=min((r['nearest_pair'] for r in reports),key=lambda row:row['distance_mm']),
    unexpected_intersections=overlaps,zero_volume_contacts=touches,
    no_material_intersections=not overlaps and all(r['no_material_intersections'] for r in reports),
    positive_separation=not overlaps and not touches and all(r['positive_separation'] for r in reports),
    merged_from=[dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                     sample_times_s=[p['time_s'] for p in report['poses']]) for path,report in zip(args.receipts,reports)],
    merge_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
args.out.parent.mkdir(parents=True,exist_ok=True)
args.out.write_text(json.dumps(result,indent=2))
print(json.dumps({key:result[key] for key in ('pose_count','cross_body_pair_evaluations','native_distance_evaluations','aabb_exclusions','native_intersection_evaluations','minimum_distance_mm','no_material_intersections','positive_separation','identity')},indent=2))
