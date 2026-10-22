"""Certify a complete commanded joint-space path against native analytic CAD."""
import argparse
import json
from pathlib import Path
import sys

from mechanism_lab.constructed import build, build_from_plan
from mechanism_lab.motion_envelope import certify_path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('architecture', nargs='?', default='compact_arm')
    parser.add_argument('--recipe', type=Path, help='Saved recipe instead of a default fixture')
    parser.add_argument('--parameters', type=Path)
    parser.add_argument('--waypoints', type=Path, help='JSON list of complete independent-control maps')
    parser.add_argument('--joint-error-bounds', type=Path, help='JSON map of absolute control tracking errors, degrees/mm')
    parser.add_argument('--drive-error-bounds', type=Path, help='JSON map of follower compliance residuals, degrees/mm')
    parser.add_argument('--out', type=Path, default=Path('outputs/mechanisms/motion-envelope.json'))
    parser.add_argument('--max-depth', type=int, default=16)
    args = parser.parse_args(argv)
    args.out.unlink(missing_ok=True)
    read = lambda p: json.loads(p.read_text(encoding='utf-8')) if p else None
    assembly = build_from_plan(read(args.recipe)) if args.recipe else build(args.architecture, read(args.parameters))
    report = certify_path(assembly, read(args.waypoints), max_depth=args.max_depth,
                          joint_error_bounds=read(args.joint_error_bounds),
                          drive_error_bounds=read(args.drive_error_bounds),
                          progress=lambda row: print(json.dumps(row), file=sys.stderr, flush=True))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'passed': report['passed'], 'failures': len(report['failures']),
                      'runtime_seconds': report['runtime_seconds'], 'statistics': report['statistics'],
                      'report': str(args.out)}, indent=2))
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
