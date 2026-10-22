"""One construction plan feeds CAD, kinematics and reproducible inspection artifacts."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np

from .core import project_root, validate


def inspection_media(assembly, directory, size=(960, 720), seconds=8., fps=12):
    """Software mesh views of the generated CAD at its actual prescribed poses.

    This intentionally uses no external images, native compiler, display server,
    HDRI downloads or image synthesis. It is a technical triangle preview, not a
    photographic render or a dynamics recording. Photographic CLIs stay intact.
    """
    from PIL import Image, ImageDraw, ImageFont
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    width, height = size
    count = max(2, int(round(seconds * fps)))
    times = np.linspace(0, seconds, count, endpoint=False)
    # Keep one camera and framing throughout the complete motion cycle.
    eye = np.array([1.3, -1.7, .85]); eye /= np.linalg.norm(eye)
    right = np.cross([0., 0., 1.], eye); right /= np.linalg.norm(right)
    up = np.cross(eye, right)
    camera = np.array([right, up, eye])
    bounds = []
    for t in np.linspace(0, seconds, 17):
        for part in assembly.parts:
            transform = assembly.pose(part, float(t), 0)
            corners = np.array([[x, y, z] for x in part.bounds[:, 0]
                                for y in part.bounds[:, 1] for z in part.bounds[:, 2]])
            bounds.append((corners @ transform[:3, :3].T + transform[:3, 3]) @ camera.T)
    bounds = np.concatenate(bounds)
    low, high = bounds.min(axis=0), bounds.max(axis=0)
    center = (low + high) / 2
    scale = min((width - 90) / (high[0] - low[0]), (height - 140) / (high[1] - low[1]))
    light = np.array([.2, -.3, .94]); light /= np.linalg.norm(light)
    try:
        font = ImageFont.truetype('DejaVuSans.ttf', 19)
        small = ImageFont.truetype('DejaVuSans.ttf', 13)
    except OSError:
        font = ImageFont.load_default(size=19); small = ImageFont.load_default(size=13)
    frames, digests = [], []
    for t in times:
        polygons, depths, colors = [], [], []
        for part in assembly.parts:
            transform = assembly.pose(part, float(t), 0)
            vertices = part.vertices @ transform[:3, :3].T + transform[:3, 3]
            tri = vertices[part.faces]
            normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
            lengths = np.linalg.norm(normals, axis=1)
            valid = lengths > 1e-12
            normals[valid] /= lengths[valid, None]
            visible = valid & (normals @ eye > 0)
            tri = tri[visible] @ camera.T
            shade = .36 + .64 * np.clip(normals[visible] @ light, 0, 1)
            material = assembly.materials[part.material]
            rgb = np.clip(np.power(np.asarray(material.color), 1/2.2) * 255, 0, 255)
            screen = (tri[..., :2] - center[:2]) * [scale, -scale] + [width/2, height/2+10]
            polygons.extend(screen.tolist()); depths.extend(tri[..., 2].mean(axis=1))
            colors.extend(np.clip(rgb[None, :] * shade[:, None], 0, 255).astype(np.uint8))
        frame = Image.new('RGB', size, '#101921'); drawing = ImageDraw.Draw(frame)
        for i in np.argsort(depths):
            drawing.polygon([tuple(p) for p in polygons[i]], fill=tuple(int(c) for c in colors[i]))
        drawing.text((24, 18), 'CYBR GEO / ' + assembly.name.replace('_', ' ').upper(), fill='#e3eff6', font=font)
        drawing.text((24, 48), 'CONSTRUCTED CAD  /  prescribed rigid-body motion', fill='#65cbbf', font=small)
        drawing.text((24, height-28), 'Nominal concept geometry. Dynamics and hardware qualification have not been performed.', fill='#a0b3bf', font=small)
        # Hash geometry image before a time label: distinctness must come from motion.
        digests.append(hashlib.sha256(frame.tobytes()).hexdigest())
        frames.append(frame)
    frames[0].save(directory / 'reference.png')
    frames[len(frames)//3].save(directory / 'motion.png')
    frames[0].save(directory / 'motion.gif', save_all=True, append_images=frames[1:],
                   duration=round(1000/fps), loop=0, optimize=False)
    return {'renderer': 'software triangle inspection of generated CAD tessellation',
            'frames': len(frames), 'distinct_geometry_frames': len(set(digests)),
            'resolution': list(size), 'seconds': seconds, 'fps': fps}


def generate(architecture='compact_arm', parameters=None, output=None, step=False,
             media=False, size=(960, 720), seconds=8., fps=12):
    from .constructed import build, KinematicGraph
    from .exporters import export_glb, export_animated_glb, export_bom, export_step
    import trimesh
    if not math.isfinite(seconds) or seconds <= 0 or not 1 <= fps <= 120:
        raise ValueError('Use a positive duration and 1-120 fps')
    if round(seconds * fps) < 1:
        raise ValueError('Duration and fps must provide at least one animation interval')
    if seconds * fps > 2400:
        raise ValueError('Inspection bundles are limited to 2400 motion samples')
    assembly = build(architecture, parameters)
    plan = assembly.metadata['construction_recipe']
    graph = KinematicGraph([p.name for p in assembly.parts], plan['joints'])
    output = Path(output or project_root()/'outputs'/'mechanisms'/architecture).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    receipt = output/'bundle.json'; receipt.unlink(missing_ok=True)
    geometry = validate(assembly, expensive=True)
    for name, value in [('recipe.json', plan), ('construction.json', plan['construction']),
                        ('geometry.json', geometry), ('mechanism.json', {
                            'schema': 'cybrgeo-mechanism/1', 'architecture': architecture,
                            'parameters': parameters or {}})]:
        (output/name).write_text(json.dumps(value, indent=2)+'\n', encoding='utf-8')
    export_glb(assembly, output/'assembly.glb'); export_bom(assembly, output)
    animation = export_animated_glb(assembly, output/'motion.glb', seconds, fps, 'motion')
    if step:
        export_step(assembly, output/'assembly.step')
    reopened = trimesh.load_scene(output/'assembly.glb', process=False)
    if len(reopened.geometry) != len(assembly.parts):
        raise RuntimeError('GLB readback changed the number of generated components')
    samples = []
    for t in np.linspace(0, seconds, 17):
        values = graph.demo_values(float(t))
        transforms = graph.transforms(values)
        residual = max(float(np.max(np.abs(T[:3, :3].T @ T[:3, :3] - np.eye(3))))
                       for T in transforms.values())
        samples.append({'time_s': float(t), 'controls': values,
                        'resolved': graph.resolve(values), 'rigid_rotation_error': residual})
    (output/'motion-samples.json').write_text(json.dumps(samples, indent=2)+'\n', encoding='utf-8')
    image_report = inspection_media(assembly, output, size, seconds, fps) if media else None
    if animation['animated_channels'] == 0:
        raise RuntimeError('Constructed mechanism exported no animated channels')
    generated = ['recipe.json', 'construction.json', 'geometry.json', 'mechanism.json',
                 'assembly.glb', 'parts.json', 'parts.csv', 'motion.glb',
                 'motion.animation.json', 'motion-samples.json']
    if step:
        generated += ['assembly.step', 'assembly.coverage.json']
        generated += ['step_parts/'+p.name+'.step' for p in assembly.parts]
    if media:
        generated += ['reference.png', 'motion.png', 'motion.gif']
    report = {'architecture': architecture, 'model': assembly.name, 'parts': len(assembly.parts),
              'construction_recipe_sha256': plan['construction']['recipe_sha256'],
              'independent_controls': list(graph.controls), 'output': str(output),
              'glb_readback_components': len(reopened.geometry), 'animation': animation,
              'media': image_report,
              'scope': 'Constructed nominal CAD, prescribed kinematic motion, artifact readback. '
                       'No dynamics simulation, motor sizing, full swept collision proof or hardware qualification.',
              'files_sha256': {name: hashlib.sha256((output/name).read_bytes()).hexdigest()
                               for name in generated}}
    receipt.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    return report
