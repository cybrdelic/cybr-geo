"""FUSE C220 delivery using the repository's current Mitsuba/OIDN V9 API.

The original native-renderer script is retained in the recovery commit history.
Original delivery images are historical legacy-photoreal results, not rerenders
from this adapter. Film frames are serialized because v9_dispatch temporarily
changes module-level Mitsuba dispatch. Every frame has a render receipt.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))
from PIL import Image, ImageDraw, ImageFont
from printer import posed
from toolpath import parse
from mechanism_lab.core import load_cache
from mechanism_lab.render_profiles import V9
from mechanism_lab.v9_dispatch import render_v9

OUT = Path('deliverables')
WORK = Path('work')
FRAMES = WORK / 'film_v9_frames'
FPS = 24
FRAME_COUNT = 96
FRAME_SIZE = (960, 720)


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def fingerprint() -> str:
    """Invalidate film checkpoints when sources, geometry, or toolpath change."""
    paths = list(Path(__file__).parent.glob('*.py'))
    paths += list((ROOT / 'src' / 'mechanism_lab').glob('*.py'))
    paths += sorted((OUT / 'cache').rglob('*'))
    paths.append(OUT / 'FUSE_C220_calibration.gcode')
    records = [(str(p), digest(p)) for p in sorted(set(paths)) if p.is_file()]
    contract = {'source_files': records, 'profile': asdict(V9), 'size': FRAME_SIZE,
                'fps': FPS, 'frame_count': FRAME_COUNT, 'adapter_version': 1}
    return hashlib.sha256(json.dumps(contract, sort_keys=True).encode()).hexdigest()


def setup():
    OUT.mkdir(parents=True, exist_ok=True)
    WORK.mkdir(parents=True, exist_ok=True)
    assembly = load_cache(OUT / 'cache')
    toolpath = parse(OUT / 'FUSE_C220_calibration.gcode')
    toolpath.prepare_beads()
    return assembly, toolpath


def schedule(toolpath):
    start = next(m.t0 for m in toolpath.moves if m.deposits)
    result = []
    for i in range(48):
        u = i / 95 if i < 24 else (23 / 95 + (1 - 23 / 95) * (i - 23) / 24)
        result.append((start + (toolpath.deposition_end - start) * u,
                       '180-layer time-lapse / compressed print time'))
    live_start = max(start, toolpath.deposition_end - 3.0)
    result.extend((live_start + i / FPS,
                   '1x modeled motion / replay of final extrusion') for i in range(48))
    return result


def stills(assembly, toolpath, preview=False):
    t = toolpath.deposition_end * .88
    state, _, _ = toolpath.state(t)
    scene = posed(assembly, state, toolpath.geometry(t))
    shots = ['hero', 'printing'] if preview else ['hero', 'printing', 'drive']
    for view in shots:
        size = (960, 720) if preview else ((1920, 1280) if view == 'printing' else (1920, 1440))
        path = WORK / f'preview_{view}.png' if preview else OUT / f'FUSE_C220_{view}.png'
        start = time.perf_counter()
        render_v9(scene, path, view_name=view, size=size,
                  spp=32 if preview else V9.still_spp, depth=V9.still_depth)
        print('DONE', path, 'seconds', round(time.perf_counter() - start, 3), flush=True)


def label(path, mode, layer, z):
    with Image.open(path) as source:
        im = source.convert('RGB')
    draw = ImageDraw.Draw(im)
    draw.rounded_rectangle((16, 14, im.width - 16, 66), radius=7, fill=(17, 23, 25))
    normal = Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
    bold = normal.with_name('DejaVuSans-Bold.ttf')
    f = ImageFont.truetype(str(bold), 17) if bold.exists() else ImageFont.load_default()
    s = ImageFont.truetype(str(normal), 12) if normal.exists() else ImageFont.load_default()
    draw.text((30, 23), 'CYBR FUSE / C220', font=f, fill=(215, 228, 223))
    draw.text((30, 45), mode, font=s, fill=(102, 208, 182))
    draw.text((im.width - 300, 26), f'Layer {layer + 1:03d} / 180    Z {z:05.2f} mm',
              font=s, fill=(206, 213, 210))
    draw.text((im.width - 300, 44), 'G-code-driven nozzle, bed and beads',
              font=s, fill=(147, 166, 160))
    im.save(path)


def read_checkpoint(index, identity):
    image = FRAMES / f'{index:05d}.png'
    receipt = FRAMES / f'{index:05d}.frame.json'
    if not image.exists() or not receipt.exists():
        return None
    try:
        record = json.loads(receipt.read_text())
        if (record['frame'] == index and record['fingerprint'] == identity
                and record['sha256'] == digest(image)
                and (FRAMES / f'{index:05d}.json').exists()):
            return record
    except (ValueError, KeyError, OSError):
        pass
    return None


def film(assembly, toolpath, only_frames=None):
    FRAMES.mkdir(parents=True, exist_ok=True)
    identity = fingerprint()
    if only_frames is not None and not all(0 <= i < FRAME_COUNT for i in only_frames):
        raise ValueError('Frame indices must lie in [0, 95]')
    for i, (t, mode) in enumerate(schedule(toolpath)):
        if only_frames is not None and i not in only_frames:
            continue
        if read_checkpoint(i, identity) is not None:
            continue
        state, move, fraction = toolpath.state(t)
        deposit = toolpath.geometry(t)
        scene = posed(assembly, state, deposit)
        view = replace(scene.views['printing'], scale=122, target=(0, -9, 130), f_stop=11)
        scene = replace(scene, views={**scene.views, 'printing': view})
        image = FRAMES / f'{i:05d}.png'
        begin = time.perf_counter()
        render_receipt = render_v9(scene, image, view_name='printing', size=FRAME_SIZE,
                                   spp=V9.video_spp, depth=V9.video_depth)
        label(image, mode, toolpath.moves[move].layer, state.z)
        record = {'frame': i, 'fingerprint': identity, 'sha256': digest(image),
                  'gcode_time_s': t, 'mode': mode, 'state': asdict(state),
                  'layer': toolpath.moves[move].layer, 'move_index': move,
                  'move_fraction': fraction, 'render_profile': render_receipt['render_profile'],
                  'deposition_triangles': sum(len(p.faces) for p in deposit),
                  'seconds_to_render': time.perf_counter() - begin,
                  'burned_in_explanatory_labels': True}
        image.with_suffix('.frame.json').write_text(json.dumps(record, indent=2) + '\n')
        print('COMPLETE frame', i, 'of', FRAME_COUNT, flush=True)
    if only_frames is None:
        encode_film(identity)


def encode_film(identity=None):
    identity = fingerprint() if identity is None else identity
    records = [read_checkpoint(i, identity) for i in range(FRAME_COUNT)]
    if any(record is None for record in records):
        raise RuntimeError('Encoding requires all 96 current, checksum-valid V9 frame receipts')
    output = OUT / 'FUSE_C220_printing.mp4'
    subprocess.run(['ffmpeg', '-y', '-v', 'error', '-framerate', str(FPS),
                    '-i', str(FRAMES / '%05d.png'), '-frames:v', str(FRAME_COUNT),
                    '-c:v', 'libx264', '-preset', 'slow', '-crf', '16',
                    '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output)], check=True)
    report = {'renderer': 'Mitsuba 3 path + albedo/normal-guided Intel OIDN',
              'render_profile': 'v9', 'frames': FRAME_COUNT, 'fps': FPS,
              'duration_s': FRAME_COUNT / FPS, 'resolution': FRAME_SIZE,
              'spp': V9.video_spp, 'bounce_limit': V9.video_depth,
              'fingerprint': identity, 'sha256': digest(output),
              'frame_interpolation': False, 'generated_imagery': False,
              'burned_in_explanatory_labels': True,
              'time_lapse_seconds': 2, 'modeled_real_time_replay_seconds': 2,
              'frames_detail': records}
    output.with_suffix('.video.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['preview', 'stills', 'film', 'frames', 'encode'])
    parser.add_argument('indices', nargs='*', type=int)
    args = parser.parse_args()
    if args.mode == 'encode':
        encode_film()
        return
    assembly, toolpath = setup()
    if args.mode in ('preview', 'stills'):
        stills(assembly, toolpath, args.mode == 'preview')
    elif args.mode == 'frames':
        if not args.indices:
            parser.error('frames requires at least one index')
        film(assembly, toolpath, set(args.indices))
    else:
        film(assembly, toolpath)


if __name__ == '__main__':
    main()
