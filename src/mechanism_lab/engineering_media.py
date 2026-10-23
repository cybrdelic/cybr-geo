"""Render accepted engineering runs through their shared native replay.

The renderer consumes saved integrated states. It never solves a trajectory,
interpolates poses, attaches a free body, explodes parts or substitutes images.
The receipt distinguishes the source run from the current rendering code.
"""
from __future__ import annotations

import argparse
from bisect import bisect_right
from copy import deepcopy
from dataclasses import asdict, dataclass
import gzip
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

from .engineering_source import (
    assert_project_source_identity, load_project, verify_native_build,
    verify_native_source_binding,
)
from .physical_model import digest
from .physical_replay import pose as recorded_pose


_RENDER_SOURCES = ('engineering_media.py', 'engineering_source.py', 'native_replay.py',
                   'physical_replay.py', 'render.py', 'media.py', 'cli.py', 'core.py', 'truth.py')


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _save(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def _render_identity():
    root = Path(__file__).parent
    return {name: _sha(root / name) for name in _RENDER_SOURCES}


def _surface_identity(assembly):
    """Bind actual in-memory buffers and materials, independently of labels."""
    result = hashlib.sha256()
    result.update(digest([material.as_dict() for material in assembly.materials]).encode())
    for part in assembly.parts:
        result.update(digest(part.metadata()).encode())
        for array in (part.vertices, part.faces, part.normals):
            values = np.ascontiguousarray(array)
            result.update(str((values.dtype.str, values.shape)).encode())
            result.update(values.tobytes())
    fields = ('recorded_times_s', 'recorded_body_transforms_mm', 'recorded_part_bindings',
              'world_basis_to_core', 'body_mapping', 'replay_state_sha256', 'display_mesh_sha256',
              'model_sha256', 'control_sha256', 'reference_sha256', 'native_build_sha256',
              'native_source_binding_sha256', 'render_bounds_mm')
    result.update(digest({key: assembly.metadata.get(key) for key in fields}).encode())
    return result.hexdigest()


def _motion_identity(motion):
    result = hashlib.sha256(np.asarray(motion.times, dtype='<f8').tobytes())
    for body, rows in sorted(motion.transforms.items()):
        result.update(body.encode() + b'\0')
        result.update(np.asarray(rows, dtype='<f8').tobytes())
    return result.hexdigest()


def _shared_replay(project, directory, trace):
    """Observe the shared builder invocation and its returned render surface.

    This is a local execution receipt, not a claim about arbitrary imports.
    Source-specific camera/notes may change after construction; geometry,
    material buffers and physical replay bindings must remain the shared result.
    """
    from . import native_replay, physical_replay
    codes = {native_replay.build_assembly.__code__: 'native_replay.build_assembly',
             physical_replay.build_assembly.__code__: 'physical_replay.build_assembly'}
    counts = {name: 0 for name in codes.values()}
    built = []
    previous = sys.getprofile()
    def observe(frame, event, arg):
        service = codes.get(frame.f_code)
        if event == 'call' and service:
            counts[service] += 1
        elif event == 'return' and service == 'native_replay.build_assembly' and arg is not None:
            built.append((arg, _surface_identity(arg)))
        if previous is not None:
            previous(frame, event, arg)
    try:
        sys.setprofile(observe)
        assembly = project.replay(directory, trace)
    finally:
        sys.setprofile(previous)
    matches = [surface for value, surface in built if value is assembly]
    if not matches or not all(counts.values()) or matches[-1] != _surface_identity(assembly):
        raise ValueError('Project replay must return unchanged geometry/materials from the shared native replay builder')
    return assembly, counts


def _git_identity():
    """Record checkout context when available; file hashes remain authoritative."""
    root = Path(__file__).resolve().parents[2]
    try:
        commit = subprocess.run(['git', '-C', str(root), 'rev-parse', 'HEAD'],
            capture_output=True, text=True, check=True).stdout.strip()
        changed = subprocess.run(['git', '-C', str(root), 'diff', 'HEAD', '--name-only'],
            capture_output=True, text=True, check=True).stdout.splitlines()
    except (OSError, subprocess.CalledProcessError):
        return {'commit': None, 'tracked_changes': None, 'scope': 'No Git checkout available'}
    return {'commit': commit, 'tracked_changes': changed,
            'scope': 'Checkout context; bound project, native and renderer hashes identify actual inputs'}


def frame_schedule(times, fps=24):
    """Select preceding samples at real-time video ticks, including the endpoint.

    The final state receives a complete video frame. This adds at most two
    frame periods of terminal hold; the source motion is never sped up.
    """
    if isinstance(fps, bool) or not isinstance(fps, int) or not 1 <= fps <= 120:
        raise ValueError('Video fps must be an integer in [1, 120]')
    times = [float(value) for value in times]
    if (len(times) < 2 or not all(math.isfinite(value) and value >= 0 for value in times)
            or any(a >= b for a, b in zip(times, times[1:]))):
        raise ValueError('Recorded times must be finite, nonnegative and strictly increasing')
    duration = times[-1] - times[0]
    count = math.ceil(duration * fps) + 1
    rows = []
    for number in range(count):
        requested = min(times[-1], times[0] + number / fps)
        index = max(0, bisect_right(times, requested) - 1)
        rows.append(dict(video_frame=number, video_time_s=number / fps,
            requested_source_time_s=requested, recorded_frame_index=index,
            recorded_time_s=times[index], hold_age_s=requested - times[index]))
    return rows


def verify_replay(assembly, trace):
    """Compare every recorded body matrix with the trace's SI world poses."""
    metadata = assembly.metadata
    rebound = recorded_pose.bind(assembly)
    frames = trace['frames']
    if metadata['recorded_times_s'] != [frame['time_s'] for frame in frames]:
        raise ValueError('Replay times differ from the integrated trace')
    basis = np.asarray(metadata['world_basis_to_core'], dtype=float)
    transforms = metadata['recorded_body_transforms_mm']
    comparisons = 0
    maximum = 0.
    for body, rows in transforms.items():
        if len(rows) != len(frames):
            raise ValueError('Replay transform count differs from the integrated trace')
        for frame, row in zip(frames, rows):
            state = frame['bodies'][body]
            expected = np.eye(4)
            expected[:3, :3] = basis @ np.asarray(state['rotation_row_major']).reshape(3, 3)
            expected[:3, 3] = basis @ (np.asarray(state['position_m']) * 1000.)
            error = float(np.max(np.abs(expected - np.asarray(row))))
            maximum = max(maximum, error)
            comparisons += 1
    if maximum > 1e-9:
        raise ValueError('Replay body transforms differ from the integrated trace')
    if any(part.cad is None or not part.cad.isValid() or part.cad.Volume() <= 0
           for part in assembly.parts):
        raise ValueError('Engineering media requires valid positive-volume CAD for every displayed part')
    # Compare the active callback too, then replace it with the verified shared
    # holder. A client callback cannot introduce motion after identity checks.
    for part in assembly.parts:
        for frame in (frames[0], frames[len(frames)//2], frames[-1]):
            if not np.array_equal(assembly.pose(part, frame['time_s']),
                                  rebound(part, frame['time_s'])):
                raise ValueError('Client pose callback differs from the recorded replay')
    assembly.motion_function = rebound
    return dict(passed=True, recorded_frames=len(frames), body_matrices_checked=comparisons,
        max_matrix_component_error=maximum, analytical_parts_valid=len(assembly.parts),
        scope='Every recorded body matrix matches the trace; active replay uses the verified shared preceding-state holder. No continuous clearance claim.')


@dataclass
class AcceptedRun:
    source: str
    root: Path
    native: Path
    project: object
    source_files: dict
    report: dict
    trace: dict
    assembly: object
    inputs: dict
    renderer_sources: dict
    replay_check: dict
    assembly_identity: str
    recorded_motion: object
    recorded_motion_identity: str
    observed_services: dict

    def verify_unchanged(self):
        assert_project_source_identity(self.source, self.source_files)
        if self.renderer_sources != _render_identity():
            raise ValueError('Shared rendering code changed after loading the run')
        if _surface_identity(self.assembly) != self.assembly_identity:
            raise ValueError('In-memory replay geometry, materials or bindings changed after loading')
        if (self.assembly.motion_function is not self.recorded_motion
                or _motion_identity(self.recorded_motion) != self.recorded_motion_identity):
            raise ValueError('In-memory recorded motion changed after loading')
        for relative, expected in self.inputs.items():
            if _sha(self.root / relative) != expected:
                raise ValueError(f'Engineering input changed after loading: {relative}')
        native = verify_native_build(self.native, self.project.specification,
                                     source_files=self.source_files)
        if digest(native) != self.report['native_build_sha256']:
            raise ValueError('Native completion receipt changed after loading the run')


def load_run(source, run_directory, *, native_directory=None):
    """Load and re-evaluate one current accepted project, with no regeneration."""
    root = Path(run_directory).resolve()
    native = Path(native_directory).resolve() if native_directory else root / 'native'
    filenames = ('engineering.json', 'design-source.json', 'physical-source.json',
                 'control-source.json', 'simulation/trace.json.gz')
    payloads = {name: (root / name).read_bytes() for name in filenames}
    values = {name: json.loads(gzip.decompress(data) if name.endswith('.gz') else data)
              for name, data in payloads.items()}
    report, trace = values['engineering.json'], values['simulation/trace.json.gz']
    if report.get('schema') != 'cybrgeo-engineering-result/2' or report.get('passed') is not True:
        raise ValueError('Engineering media requires an accepted cybrgeo-engineering-result/2')
    project, sources = load_project(source)
    design = values['design-source.json']
    if (project.name != report.get('name') or sources != report.get('source_files')
            or design.get('source_files') != sources or design.get('specification') != project.specification):
        raise ValueError('Accepted run differs from the current authored project')
    if (trace.get('schema') != 'cybrgeo-causal-simulation/1'
            or trace.get('reference_source', {}).get('source_files') != sources
            or digest(trace.get('reference_source')) != trace.get('reference_sha256')
            or trace.get('task_parameters') != trace.get('reference_source', {}).get('task_parameters')):
        raise ValueError('Integrated trace reference binding is invalid')
    for field, filename in (('model_sha256', 'physical-source.json'),
                            ('control_sha256', 'control-source.json')):
        if digest(values[filename]) != trace.get(field) or trace.get(field) != report.get(field):
            raise ValueError(f'Integrated trace {field} differs from the accepted source')
    if values['physical-source.json'] != trace.get('physical_source'):
        raise ValueError('Integrated physical source differs from its saved source')
    summary = {key: value for key, value in trace.items()
               if key not in ('frames', 'task_parameters', 'physical_source', 'requirements')}
    if report.get('simulation') != summary:
        raise ValueError('Accepted report differs from the integrated trace summary')
    from .causal_runtime import _IMPLEMENTATION_IDENTITY, _runtime_identity
    if trace.get('runtime_implementation') != _IMPLEMENTATION_IDENTITY or _runtime_identity() != _IMPLEMENTATION_IDENTITY:
        raise ValueError('Integrated trace uses another runtime; generate a fresh current run')
    outcomes = project.requirements(trace, trace['task_parameters'])
    if outcomes.get('passed') is not True or outcomes != report.get('requirements') or outcomes != trace.get('requirements'):
        raise ValueError('Current project requirements do not accept the saved trace exactly')
    receipt = verify_native_build(native, project.specification, source_files=sources)
    binding = verify_native_source_binding(native, project.specification, sources, receipt=receipt)
    if digest(receipt) != report.get('native_build_sha256') or digest(binding) != report.get('native_source_binding_sha256'):
        raise ValueError('Native build/source binding differs from the accepted run')
    assembly, observed = _shared_replay(project, native, trace)
    for key in ('model_sha256', 'control_sha256', 'native_build_sha256', 'native_source_binding_sha256'):
        if assembly.metadata.get(key) != report.get(key):
            raise ValueError(f'Native replay has a different {key}')
    check = verify_replay(assembly, trace)
    result = AcceptedRun(str(source), root, native, project, sources, report, trace, assembly,
        {name: hashlib.sha256(data).hexdigest() for name, data in payloads.items()},
        _render_identity(), check, _surface_identity(assembly), assembly.motion_function,
        _motion_identity(assembly.motion_function), observed)
    result.verify_unchanged()
    return result


def _size(value):
    if len(value) != 2 or any(isinstance(n, bool) or not isinstance(n, int) or n < 32 or n % 2 for n in value):
        raise ValueError('Image dimensions must be even integers of at least 32 pixels')
    return tuple(value)


def _backend(studio):
    import vtk
    return dict(name='vtk-pbr', renderer='VTK PBR rasterizer', vtk_version=vtk.vtkVersion.GetVTKVersion(),
        render_window=studio.win.GetClassName(), graphics_capabilities=studio.win.ReportCapabilities(),
        supersample=studio.supersample, generated_imagery=False, spp=None,
        scope='Actual native mesh rasterization with source display materials and studio illumination; not path tracing')


def render_run(run, output, *, view_name='hero', still_size=(1920, 1080), video_size=(1920, 1080),
               fps=24, supersample=2, still_times=None, video=True, azimuth=None, elevation=None,
               scale=None, target=None, floor=None):
    """Render high-resolution stills first, then the complete integrated cycle."""
    from .render import Studio, labelled
    from .truth import assert_renderable
    from .media import probe

    still_size, video_size = _size(still_size), _size(video_size)
    if isinstance(supersample, bool) or not isinstance(supersample, int) or not 1 <= supersample <= 4:
        raise ValueError('Supersample must be an integer in [1,4]')
    run.verify_unchanged()
    assembly = run.assembly
    truth = assert_renderable(assembly, intent='inspection')
    view = deepcopy(assembly.views[view_name])
    if view.hide or view.section is not None or view.explode != 0:
        raise ValueError('Engineering media requires all parts without sections or explosion')
    for key, value in (('az', azimuth), ('el', elevation), ('scale', scale)):
        if value is not None:
            if not math.isfinite(value) or (key == 'scale' and value <= 0):
                raise ValueError('Camera parameters must be finite; scale must be positive')
            setattr(view, key, value)
    if target is not None:
        if len(target) != 3 or not all(math.isfinite(value) for value in target):
            raise ValueError('Camera target requires three finite coordinates')
        view.target = tuple(target)
    if floor is not None:
        view.floor = bool(floor)
    times = assembly.metadata['recorded_times_s']
    schedule = frame_schedule(times, fps)
    targets = [times[0], times[len(times)//2], times[-1]] if still_times is None else list(still_times)
    if not targets or any(not math.isfinite(t) or t < times[0] or t > times[-1] for t in targets):
        raise ValueError('Still times must lie within the recorded trace')
    still_indices = sorted({max(0, bisect_right(times, float(t)) - 1) for t in targets})
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    if any(output.iterdir()):
        raise ValueError('Use an empty output directory for a distinct media receipt')
    started = time.perf_counter()
    progress = output / 'progress.json'
    def checkpoint(stage, **extra):
        _save(progress, dict(stage=stage, elapsed_s=time.perf_counter()-started, **extra))
    checkpoint('stills')
    stills = []
    studio = None
    try:
        studio = Studio(assembly, size=still_size, floor=view.floor,
                        supersample=supersample, exposure=view.exposure)
        studio.set_view(view)
        for index in still_indices:
            studio.pose(times[index])
            raw = studio.render()
            image = labelled(raw, view.title or assembly.name,
                f'Actual integrated state at {times[index]:.4f} s',
                'Native CAD / MuJoCo integrated poses / declared model / hardware unqualified',
                tag='CYBR GEO / SHARED ENGINEERING REPLAY')
            filename = f'pose-{index:05d}.png'
            image.save(output / filename)
            stills.append(dict(path=filename, recorded_frame_index=index, recorded_time_s=times[index],
                resolution=list(still_size), sha256=_sha(output / filename),
                pixel_sha256_before_caption=hashlib.sha256(raw.tobytes()).hexdigest()))
            checkpoint('stills', completed_stills=stills)
            print(json.dumps({'rendered_still': filename, 'sha256': stills[-1]['sha256']}), flush=True)
        still_backend = _backend(studio)
    finally:
        if studio is not None:
            studio.close()
    video_result = None
    if video:
        checkpoint('video', completed_stills=stills, frames=0, total_frames=len(schedule))
        partial = output / 'motion.partial.mp4'
        command = ['ffmpeg', '-y', '-v', 'error', '-threads', '2', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
            '-s', f'{video_size[0]}x{video_size[1]}', '-r', str(fps), '-i', '-', '-an',
            '-c:v', 'libx264', '-threads', '2', '-preset', 'medium', '-crf', '17',
            '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(partial)]
        log = output / 'encoding.log'
        studio = None
        with log.open('wb') as errors:
            process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=errors)
            try:
                studio = Studio(assembly, size=video_size, floor=view.floor,
                                supersample=supersample, exposure=view.exposure)
                studio.set_view(view)
                for row in schedule:
                    studio.pose(row['recorded_time_s'])
                    raw = studio.render()
                    image = labelled(raw, view.title or assembly.name,
                        f'Actual integrated state at {row["recorded_time_s"]:.4f} s',
                        'Native CAD / MuJoCo integrated poses / declared model / hardware unqualified',
                        tag='CYBR GEO / SHARED ENGINEERING REPLAY')
                    process.stdin.write(np.ascontiguousarray(image, dtype=np.uint8).tobytes())
                    row['pixel_sha256_before_caption'] = hashlib.sha256(raw.tobytes()).hexdigest()
                    if row['video_frame'] % fps == 0 or row is schedule[-1]:
                        checkpoint('video', completed_stills=stills, frames=row['video_frame']+1,
                                   total_frames=len(schedule))
                        print(json.dumps({'rendered_frames': row['video_frame']+1,
                                          'total_frames': len(schedule)}), flush=True)
                process.stdin.close()
                if process.wait() != 0:
                    raise RuntimeError('FFmpeg failed; inspect encoding.log')
                video_backend = _backend(studio)
                partial.replace(output / 'motion.mp4')
            except BaseException:
                if process.stdin and not process.stdin.closed:
                    try:
                        process.stdin.close()
                    except (BrokenPipeError, OSError):
                        pass
                if process.poll() is None:
                    process.kill()
                process.wait()
                partial.unlink(missing_ok=True)
                checkpoint('failed', completed_stills=stills, reason='Video rendering or encoding failed')
                raise
            finally:
                if studio is not None:
                    studio.close()
        measured = probe(output / 'motion.mp4')
        streams = measured.get('streams', [])
        if len(streams) != 1 or (int(streams[0]['width']), int(streams[0]['height'])) != video_size or int(streams[0]['nb_read_frames']) != len(schedule):
            raise ValueError('Encoded video dimensions or frame count differ from the rendered sequence')
        numerator, denominator = map(int, streams[0]['r_frame_rate'].split('/'))
        if numerator / denominator != fps:
            raise ValueError('Encoded video frame rate differs from the rendered sequence')
        encoder_version = subprocess.run(['ffmpeg', '-version'], capture_output=True, text=True, check=True).stdout.splitlines()[0]
        video_result = dict(path='motion.mp4', sha256=_sha(output / 'motion.mp4'), resolution=list(video_size),
            fps=fps, frame_count=len(schedule), source_duration_s=times[-1]-times[0],
            video_duration_s=len(schedule)/fps, terminal_hold_s=len(schedule)/fps-(times[-1]-times[0]),
            maximum_sample_hold_age_s=max(row['hold_age_s'] for row in schedule),
            sampling='Preceding recorded state at each real-time video tick; final state included; no interpolation',
            encoding={'codec': 'libx264', 'crf': 17, 'preset': 'medium', 'pixel_format': 'yuv420p',
                      'version': encoder_version, 'command': command[:-1] + ['motion.partial.mp4']},
            probe=measured, renderer=video_backend)
        _save(output / 'frames.json', schedule)
    run.verify_unchanged()
    receipt = dict(schema='cybrgeo-engineering-media/1', name=assembly.name,
        source_checkout=_git_identity(), project_source_files=run.source_files,
        input_files_sha256=run.inputs, renderer_source_files_sha256=run.renderer_sources,
        consumer_path=dict(project=run.project.name, entrypoint='mechanism_lab.engineering_media.render_run',
            replay_callback=run.project.replay.__module__+'.'+run.project.replay.__qualname__,
            observed_builder_calls=run.observed_services,
            active_pose_holder='physical_replay.RecordedMotion', renderer='render.Studio',
            proof='Observed shared builder call/return; identical returned geometry/material buffers; verified and frozen recorded-state holder'),
        model_sha256=run.trace['model_sha256'], control_sha256=run.trace['control_sha256'],
        reference_sha256=run.trace['reference_sha256'], runtime_implementation=run.trace['runtime_implementation'],
        native_build_sha256=run.report['native_build_sha256'],
        native_source_binding_sha256=run.report['native_source_binding_sha256'],
        replay_state_sha256=assembly.metadata['replay_state_sha256'],
        display_mesh_sha256=assembly.metadata['display_mesh_sha256'], replay_check=run.replay_check,
        in_memory_surface_sha256=run.assembly_identity,
        native_source=assembly.metadata.get('native_source'),
        part_count=len(assembly.parts), triangle_count=sum(len(part.faces) for part in assembly.parts),
        camera=asdict(view), stills=stills, still_renderer=still_backend, video=video_result,
        truth=truth, requirements=run.report['requirements'], qualification=run.report['qualification'],
        limitations=assembly.metadata.get('visual_limitations', []), elapsed_s=time.perf_counter()-started,
        scope='Rerendered native geometry at accepted integrated model states. Recorded state holds are explicit. Images do not qualify manufacturing, continuous clearance, contact fidelity or hardware capacity.')
    checkpoint('complete', completed_stills=stills, video=video_result)
    receipt['files_sha256'] = {path.name: _sha(path) for path in sorted(output.iterdir())
                              if path.is_file() and path.name != 'media.json'}
    _save(output / 'media.json', receipt)
    print(json.dumps({'media_receipt': str(output / 'media.json'), 'sha256': _sha(output / 'media.json')}), flush=True)
    return receipt


def add_arguments(parser):
    """One option definition shared by ``lab replay`` and the module command."""
    parser.add_argument('project')
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--native', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--size', type=int, nargs=2, default=(1920, 1080), metavar=('WIDTH', 'HEIGHT'))
    parser.add_argument('--fps', type=int, default=24)
    parser.add_argument('--supersample', type=int, default=2)
    parser.add_argument('--stills-only', action='store_true')
    parser.add_argument('--still-times', type=float, nargs='+')
    parser.add_argument('--view', default='hero')
    parser.add_argument('--azimuth', type=float)
    parser.add_argument('--elevation', type=float)
    parser.add_argument('--scale', type=float)
    parser.add_argument('--target', type=float, nargs=3)
    parser.add_argument('--no-floor', action='store_true')
    return parser


def run_arguments(args):
    run = load_run(args.project, args.run, native_directory=args.native)
    return render_run(run, args.out, view_name=args.view, still_size=tuple(args.size), video_size=tuple(args.size),
        fps=args.fps, supersample=args.supersample, still_times=args.still_times,
        video=not args.stills_only, azimuth=args.azimuth, elevation=args.elevation,
        scale=args.scale, target=args.target, floor=False if args.no_floor else None)


def main(argv=None):
    args = add_arguments(argparse.ArgumentParser(description=__doc__)).parse_args(argv)
    run_arguments(args)


if __name__ == '__main__':
    main()
