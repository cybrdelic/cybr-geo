"""Checkpointed ORBIT films using CYBR GEO's native photographic renderer.

Both five-second clips contain 120 geometry frames. The motion clip plays
the eight-second prescribed mechanism cycle at 1.6x speed. The exploded
clip reuses identical rendered poses for its symmetric return sequence.
Sampling patterns are held constant across frames to stabilize stationary
surfaces. Two geometry-guided a-trous passes reduce Monte Carlo noise; there
is no frame interpolation or image synthesis.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import time

import numpy as np
from PIL import Image

from mechanism_lab.core import load_cache
from mechanism_lab.finish_render import atrous, read_pfm, tonemap
from mechanism_lab.media import probe
from mechanism_lab.photoreal import _invoke, compile_renderer
from mechanism_lab.registry import factory, load
from mechanism_lab.truth import assert_renderable, write_truth_report

ROOT = Path(__file__).resolve().parents[1]
RECIPE = ROOT / 'examples/orbit_inspection_wrist.py'


def render_clip(assembly, exe, output, work, name, size, fps, duration, spp, threads):
    view_name = 'internal' if name == 'motion' else 'exploded'
    view = assembly.views[view_name]
    subset = replace(assembly, parts=[p for p in assembly.parts if p.group not in view.hide])
    truth = assert_renderable(assembly, 'concept', False)
    directory = work / name
    directory.mkdir(parents=True, exist_ok=True)
    total = round(fps * duration)
    settings = dict(view=view_name, resolution=list(size), fps=fps, frames=total,
                    spp=spp, depth=10, seed=2026, denoise_passes=2,
                    recipe_sha256=hashlib.sha256(RECIPE.read_bytes()).hexdigest(),
                    camera=vars(view), cycle_seconds=8.0 if name == 'motion' else None)
    signature = hashlib.sha256(json.dumps(settings, sort_keys=True).encode()).hexdigest()
    checkpoint = directory / 'settings.json'
    if checkpoint.exists() and json.loads(checkpoint.read_text())['signature'] != signature:
        raise ValueError(f'Incompatible frame checkpoint: {directory}')
    checkpoint.write_text(json.dumps(dict(signature=signature, settings=settings), indent=2)+'\n')
    rows = []
    start = time.monotonic()
    for frame in range(total):
        u = frame / total
        model_time = 8.0 * u if name == 'motion' else 0.0
        # A smooth cosine cycle separates and reassembles every authored part.
        symmetric_u = min(frame, total-frame) / total
        explosion = .5 - .5 * math.cos(math.tau*symmetric_u) if name == 'exploded' else view.explode
        png = directory / f'{frame:06d}.png'
        record = directory / f'{frame:06d}.json'
        if png.exists() and record.exists():
            row = json.loads(record.read_text())
            if row['png_sha256'] != hashlib.sha256(png.read_bytes()).hexdigest():
                raise ValueError(f'Frame checkpoint hash mismatch: {png}')
        elif name == 'exploded' and frame > total//2:
            mirror = total-frame
            origin = json.loads((directory/f'{mirror:06d}.json').read_text())
            shutil.copyfile(directory/f'{mirror:06d}.png', png)
            row = dict(frame=frame, presentation_time_seconds=frame/fps,
                       model_time_seconds=model_time, explosion=explosion,
                       render_seconds=0., reused_identical_pose_frame=mirror,
                       pixels_sha256=origin['pixels_sha256'],
                       png_sha256=hashlib.sha256(png.read_bytes()).hexdigest())
            record.write_text(json.dumps(row, indent=2)+'\n')
        else:
            tic = time.monotonic()
            # Distinct intermediate paths avoid stale file reads on hosted
            # filesystems when consecutive native renders replace one path.
            mesh = directory / f'current_{frame:06d}.meshbin'
            ppm = directory / f'current_{frame:06d}.ppm'
            _invoke(exe, subset, view, mesh, ppm, size, spp, threads, 10, 2026,
                    time_seconds=model_time, explode=explosion)
            radiance = read_pfm(str(ppm)+'.pfm')
            with open(str(ppm)+'.guides', 'rb') as stream:
                width, height = np.fromfile(stream, '<u4', 2)
                guide = np.fromfile(stream, '<f4').reshape(height, width, 9)
            variance = guide[:, :, 7].copy()
            for iteration in range(2):
                radiance, variance = atrous(radiance, guide, variance, 2**iteration, iteration,floor_id=-2)
            pixels = tonemap(radiance, exposure=view.exposure)
            Image.fromarray(pixels).save(png)
            row = dict(frame=frame, presentation_time_seconds=frame/fps,
                       model_time_seconds=model_time, explosion=explosion,
                       render_seconds=time.monotonic()-tic,
                       pixels_sha256=hashlib.sha256(pixels.tobytes()).hexdigest(),
                       png_sha256=hashlib.sha256(png.read_bytes()).hexdigest())
            record.write_text(json.dumps(row, indent=2)+'\n')
            for temporary in (mesh, mesh.with_suffix('.materials'), ppm,
                              Path(str(ppm)+'.pfm'), Path(str(ppm)+'.guides'),Path(str(ppm)+'.surfaces')):
                temporary.unlink(missing_ok=True)
        rows.append(row)
        if frame % 4 == 0 or frame == total-1:
            print(f'{name}: {frame+1}/{total}, {time.monotonic()-start:.1f}s elapsed', flush=True)
    output.mkdir(parents=True, exist_ok=True)
    destination = output / f'ORBIT_{name}.mp4'
    partial = destination.with_name(destination.stem+'.partial.mp4')
    subprocess.run(['ffmpeg', '-y', '-v', 'error', '-framerate', str(fps),
                    '-i', str(directory/'%06d.png'), '-frames:v', str(total),
                    '-an', '-c:v', 'libx264', '-threads', '2', '-preset', 'slow',
                    '-crf', '15', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(partial)], check=True)
    partial.replace(destination)
    encoded = probe(destination)
    if int(encoded['streams'][0]['nb_read_frames']) != total:
        raise RuntimeError('Encoded frame count differs from native frame count')
    report = dict(file=destination.name, model=assembly.name,
                  renderer='CYBR GEO native thin-lens BVH/GGX/MIS path tracer',
                  settings=settings, source_cycle_playback_speed=1.6 if name == 'motion' else None,
                  motion_blur=False, interpolation=False,
                  filtering='Two geometry-guided spatial a-trous passes; fixed sampling seed across frames',
                  reused_identical_pose_frames=sum('reused_identical_pose_frame' in r for r in rows),
                  unique_native_frames=len({r['pixels_sha256'] for r in rows}),
                  probe=encoded, frames_log=rows, truth=truth)
    # A film and a still can share a stem; preserve the still's render evidence.
    destination.with_suffix('.video.json').write_text(json.dumps(report, indent=2)+'\n')
    write_truth_report(truth, destination.with_suffix('.video.truth.json'))
    # The frame PNGs remain as resumable checkpoints; large scratch buffers do not.
    for suffix in ('current.meshbin', 'current.materials', 'current.ppm',
                   'current.ppm.pfm', 'current.ppm.guides'):
        (directory/suffix).unlink(missing_ok=True)
    print(f'COMPLETE {destination}: {total} frames', flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=ROOT/'outputs/orbit_showcase')
    parser.add_argument('--work', type=Path, default=ROOT/'outputs/orbit_film_frames')
    parser.add_argument('--cache', type=Path, help='Reuse an explicitly selected, previously validated mesh cache')
    parser.add_argument('--clips', nargs='+', choices=['motion', 'exploded'], default=['motion', 'exploded'])
    parser.add_argument('--size', default='800x600')
    parser.add_argument('--spp', type=int, default=48)
    parser.add_argument('--threads', type=int, default=8)
    args = parser.parse_args()
    if args.cache:
        assembly = load_cache(args.cache)
        assembly.motion_function = factory(str(RECIPE))[1]
    else:
        assembly = load(str(RECIPE))
    exe = compile_renderer()
    for clip in args.clips:
        render_clip(assembly, exe, args.out, args.work, clip,
                    tuple(map(int, args.size.split('x'))), 24, 5., args.spp, args.threads)


if __name__ == '__main__':
    main()
