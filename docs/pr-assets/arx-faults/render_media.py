"""Render actual ARX poses through the maintained native replay implementation.

From the repository root after the public build and independent readback:
python docs/pr-assets/arx-faults/render_media.py --run outputs/engineering-arx-core-final

The output folder is new render evidence, not a physics run or source film.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import importlib
import json
from pathlib import Path
import shutil

from mechanism_lab.engineering_source import assert_project_source_identity, load_project
from mechanism_lab.physical_model import digest
from mechanism_lab.physical_replay import render_evidence
from mechanism_lab.reference_clock import PhaseClock


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def inspect_media(output, manifest, media):
    """Read encoded files back; pixel statistics do not establish mechanics."""
    from PIL import Image, ImageSequence, ImageStat

    if sha256(Path(__file__)) != manifest['render_script_sha256']:
        raise ValueError('Media generator changed after the manifest was created')
    for name, expected in manifest['files_sha256'].items():
        if sha256(output/name) != expected:
            raise ValueError('Published file identity differs: '+name)
    for name, expected in media['files_sha256'].items():
        if sha256(output/name) != expected:
            raise ValueError('Shared renderer identity differs: '+name)
    durations, geometry_images = [], set()
    with Image.open(output/'integrated-motion.gif') as gif:
        if list(gif.size) != manifest['media']['size_px']:
            raise ValueError('GIF dimensions differ from the requested render')
        for frame in ImageSequence.Iterator(gif):
            durations.append(frame.info['duration'])
            rgb = frame.convert('RGB')
            geometry_images.add(hashlib.sha256(rgb.crop((0, 0, rgb.width, rgb.height-80)).tobytes()).hexdigest())
    if durations != media['gif_frame_durations_ms']:
        raise ValueError('Encoded GIF timing differs from recorded-state timing')
    stills = []
    for row in media['stills']:
        with Image.open(output/row['path']) as still:
            rgb = still.convert('RGB')
            variance = ImageStat.Stat(rgb).var
            if list(rgb.size) != manifest['media']['size_px'] or max(variance) <= 0:
                raise ValueError('Empty or incorrectly sized still: '+row['path'])
            stills.append(dict(file=row['path'], size_px=list(rgb.size), variance=variance))
    write_json(output/'media-inspection.json', dict(passed=True,
        manifest_file_sha256=sha256(output/'media-manifest.json'),
        verified_manifest_files=len(manifest['files_sha256']),
        render_script_hash_matches=True,
        shared_renderer_receipt_hashes_match=True, gif_frames=len(durations),
        gif_duration_ms=sum(durations), distinct_geometry_region_images=len(geometry_images),
        stills=stills, visually_inspected=[],
        inspection='Automated file/hash/dimension/timing readback only. Nonblank or distinct pixels do not establish mechanical correctness; visual inspection is recorded separately.'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--out', type=Path, default=Path('outputs/arx-final-replay'))
    parser.add_argument('--publish', type=Path,
                        help='Optionally copy the resulting media and receipts into an evidence packet')
    args = parser.parse_args()
    run, output = args.run, args.out
    if output.exists() and any(output.iterdir()):
        raise ValueError('Choose a new empty output directory so old media cannot enter the receipt')
    report = json.loads((run/'engineering.json').read_text(encoding='utf-8'))
    readback = json.loads((run/'ci-readback.json').read_text(encoding='utf-8'))
    command = json.loads((run/'ci-command.json').read_text(encoding='utf-8'))
    if not (report.get('passed') and readback.get('passed') and command.get('passed')):
        raise ValueError('Complete public construction, dynamics and independent CAD readback must pass first')
    trace_path = run/'simulation/trace.json.gz'
    with gzip.open(trace_path, 'rt', encoding='utf-8') as stream:
        result = json.load(stream)
    if not result.get('requirements', {}).get('passed'):
        raise ValueError('Expected the accepted nominal task trace')
    if any(result[key] != report[key] for key in ('model_sha256', 'control_sha256')):
        raise ValueError('Nominal trace and engineering report identities differ')
    if result['reference_sha256'] != digest(result['reference_source']):
        raise ValueError('Reference receipt does not match the actual trace')
    source_files = result['reference_source']['source_files']
    assert_project_source_identity('arx_belt', source_files)
    project, current_source = load_project('arx_belt')
    if current_source != source_files:
        raise ValueError('The current project differs from the integrated source')
    clock = PhaseClock.from_source(result['task_parameters']['reference_clock'])
    if result['duration_s'] < clock.physical_period_s-1e-9:
        raise ValueError('The recorded states do not cover the complete authored cycle')
    output.mkdir(parents=True, exist_ok=True)
    assembly = project.replay(run/'native', result)
    verify = importlib.import_module(project.replay.__module__.rsplit('.', 1)[0]+'.verify_geometry').verify
    replay = verify(assembly, result)
    replay['trace_file_sha256'] = sha256(trace_path)
    write_json(output/'replay-check.json', replay)
    source_landmarks = [0., 1.4, 2.2, 3.2, 4.9, 7.2, 9.9, 10.]
    physical_landmarks = [clock.physical_time(value) for value in source_landmarks]
    media = render_evidence(assembly, output, fps=6, size=(800, 600),
                            still_times=physical_landmarks, export_geometry=False)
    assert_project_source_identity('arx_belt', source_files)
    owned = ['replay-check.json', 'replay-evidence.json', 'geometry-truth.json',
             'integrated-motion.gif'] + [row['path'] for row in media['stills']]
    for source, target in (
        (run/'engineering.json', 'nominal-engineering.json'),
        (run/'ci-readback.json', 'native-readback.json'),
        (run/'native/native-build.json', 'native-build.json'),
        (run/'native/native-source-binding.json', 'native-source-binding.json')):
        shutil.copy2(source, output/target)
        owned.append(target)
    manifest = dict(schema='cybrgeo-arx-authored-policy-media/2',
        source_commit='aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2',
        source_revision='Pinned ARX belt P5 reference; not the latest ARX source revision',
        model_sha256=result['model_sha256'], control_sha256=result['control_sha256'],
        reference_sha256=result['reference_sha256'], trace_file_sha256=sha256(trace_path),
        task_parameters_sha256=digest(result['task_parameters']),
        operating_policy=result['task_parameters']['operating_policy'],
        runtime_implementation=result['runtime_implementation'],
        native_build_sha256=report['native_build_sha256'],
        native_source_binding_sha256=report['native_source_binding_sha256'],
        source_files=source_files, reference_clock=clock.source(),
        source_landmark_times_s=source_landmarks, requested_physical_landmark_times_s=physical_landmarks,
        actual_recorded_stills=media['stills'], simulated_seconds=result['duration_s'],
        integration_steps=result['steps'], recorded_frames=len(result['frames']),
        render_script_sha256=sha256(Path(__file__)),
        media=dict(size_px=[800, 600], requested_fps=6,
                   gif_frames=len(media['gif_frame_indices']), gif_duration_ms=media['gif_duration_ms']),
        scope='Fresh native CAD at recorded MuJoCo states, held without interpolation. No source film or image generation. This nominal motion does not establish separate fault acceptance, fail-safe payload retention, latest-source integration or hardware qualification.',
        files_sha256={name: sha256(output/name) for name in owned})
    write_json(output/'media-manifest.json', manifest)
    inspect_media(output, manifest, media)
    if args.publish:
        args.publish.mkdir(parents=True, exist_ok=True)
        for name in owned+['media-manifest.json', 'media-inspection.json']:
            shutil.copy2(output/name, args.publish/name)
    print(json.dumps({key: manifest[key] for key in ('model_sha256', 'reference_sha256',
        'native_source_binding_sha256', 'simulated_seconds', 'integration_steps', 'media')}, indent=2), flush=True)


if __name__ == '__main__':
    main()
