"""Compare complete saved states exactly without relabeling either runtime run."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path


def digest(value):
    raw = json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')
    return hashlib.sha256(raw).hexdigest()


def load(path):
    raw = path.read_bytes()
    trace = json.loads(gzip.decompress(raw))
    frames = trace['frames']
    if not frames:
        raise ValueError('Trace must contain saved frames')
    poses = [dict(time_s=row['time_s'], bodies=row['bodies']) for row in frames]
    return trace, dict(path=str(path), trace_sha256=hashlib.sha256(raw).hexdigest(),
        frame_count=len(frames), body_poses_and_times_sha256=digest(poses),
        complete_frames_sha256=digest(frames), runtime_implementation=trace['runtime_implementation'],
        model_sha256=trace['model_sha256'], control_sha256=trace['control_sha256'],
        reference_sha256=trace['reference_sha256'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--previous', type=Path, required=True)
    parser.add_argument('--current', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    previous, old = load(args.previous)
    current, new = load(args.current)
    same_poses = old['body_poses_and_times_sha256'] == new['body_poses_and_times_sha256']
    same_frames = old['complete_frames_sha256'] == new['complete_frames_sha256']
    same_count = old['frame_count'] == new['frame_count']
    result = dict(schema='cybrgeo-recorded-state-equivalence/1', passed=same_poses and same_frames and same_count,
        previous=old, current=new, exact_body_poses_and_times=same_poses,
        exact_complete_saved_frames=same_frames, exact_frame_count=same_count,
        first_differing_frame_index=next((i for i, (a, b) in enumerate(zip(previous['frames'], current['frames']))
                                         if digest(a) != digest(b)), None),
        comparison='Exact canonical JSON values; no rounding, interpolation, rotation normalization or tolerance.',
        scope='Recorded states only. Runtime and source identities remain separate; no new clearance or hardware claim.',
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
