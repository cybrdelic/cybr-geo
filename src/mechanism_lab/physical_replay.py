"""Bind native body-local meshes to recorded integrated rigid-body states.

This module does not evaluate a trajectory, run forward kinematics, attach a
payload, or interpret mechanism names. Display geometry and contact geometry
remain separate. Unrecorded times hold the preceding recorded pose.
"""
from __future__ import annotations

from bisect import bisect_right
import hashlib
import itertools
import json
import math
from pathlib import Path

import numpy as np

from .core import Assembly, Material, View, mesh_part


Y_UP_TO_Z_UP = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])


def _rotation(value, label):
    result = np.asarray(value, dtype=float)
    if result.size == 9:
        result = result.reshape(3, 3)
    if (result.shape != (3, 3) or not np.isfinite(result).all()
            or not np.allclose(result.T @ result, np.eye(3), atol=2e-8, rtol=0)
            or not math.isclose(float(np.linalg.det(result)), 1., abs_tol=2e-8)):
        raise ValueError(f'{label} must be a finite proper rotation')
    return result


class RecordedMotion:
    def __init__(self, times, transforms):
        self.times, self.transforms = times, transforms

    def __call__(self, part, time=0., explosion=0.):
        if explosion != 0:
            raise ValueError('Physical replay cannot explode integrated body poses')
        if not math.isfinite(time):
            raise ValueError('Replay time must be finite')
        index = min(len(self.times)-1, max(0, bisect_right(self.times, time)-1))
        return self.transforms[part.motion][index].copy()


class _PoseFactory:
    def bind(self, assembly):
        """Restore cache motion, validating the recorded transform identity.

        The digest detects stale/corrupt cache content; it is not a signature or
        permission to trust imported simulation claims. Registry verifies its
        own cache files separately.
        """
        metadata = assembly.metadata
        times = metadata['recorded_times_s']
        if not times or any(not math.isfinite(t) or t < 0 for t in times) or any(a >= b for a,b in zip(times,times[1:])):
            raise ValueError('Invalid cached recorded times')
        transforms = {name:np.asarray(rows,float) for name,rows in metadata['recorded_body_transforms_mm'].items()}
        for body, rows in transforms.items():
            if rows.shape != (len(times),4,4) or not np.isfinite(rows).all():
                raise ValueError(f'Invalid cached body transforms: {body}')
            if not np.allclose(rows[:,3,:],[0,0,0,1],atol=0,rtol=0):
                raise ValueError('Invalid cached homogeneous transform')
            for matrix in rows:
                _rotation(matrix[:3,:3],body)
        state_hash = hashlib.sha256()
        for index,time in enumerate(times):
            state_hash.update(np.asarray([time],dtype='<f8').tobytes())
            for body in sorted(transforms):
                state_hash.update(body.encode()+b'\0'+transforms[body][index].astype('<f8').tobytes())
        if state_hash.hexdigest() != metadata['replay_state_sha256']:
            raise ValueError('Recorded replay cache identity mismatch')
        bindings = metadata['recorded_part_bindings']
        if set(bindings) != {part.name for part in assembly.parts} or any(
                bindings[part.name] != part.motion or part.motion not in transforms for part in assembly.parts):
            raise ValueError('Recorded replay part bindings changed')
        return RecordedMotion(times,transforms)


pose = _PoseFactory()


def build_assembly(mesh_document, body_mapping, simulation_result, *,
                   name='Integrated physical replay', materials=None,
                   world_rotation=None, metadata=None):
    """Create a core Assembly using actual ``CausalSimulation.run`` frames.

    ``mesh_document`` has units ``mm`` and records under ``meshes`` containing
    name, body, vertices_mm, triangles and optional material/provenance. Mapping
    is explicit for every mesh body. Poses use SI positions and row-major local
    to world rotations. ``world_rotation`` changes the simulation world basis
    into core Z-up; local mesh coordinates are not converted a second time.
    Recorded states are held between timestamps, never interpolated or solved.
    """
    if mesh_document.get('units') != 'mm':
        raise ValueError('Body-local display meshes must explicitly use mm')
    if simulation_result.get('schema') != 'cybrgeo-causal-simulation/1':
        raise ValueError('Replay requires cybrgeo-causal-simulation/1')
    frames = simulation_result.get('frames')
    if not isinstance(frames, list) or not frames:
        raise ValueError('Replay requires actual recorded frames, not a summary')
    records = mesh_document.get('meshes')
    if not isinstance(records, list) or not records:
        raise ValueError('Replay requires nonempty native mesh records')
    basis = _rotation(np.eye(3) if world_rotation is None else world_rotation, 'World basis')
    bodies = set()
    for row in records:
        if row['body'] not in body_mapping:
            raise ValueError(f"Unmapped mesh body {row['body']!r}")
        bodies.add(body_mapping[row['body']])
    times, transforms = [], {body: [] for body in sorted(bodies)}
    state_hash = hashlib.sha256()
    for frame in frames:
        time = float(frame['time_s'])
        if not math.isfinite(time) or time < 0 or (times and time <= times[-1]):
            raise ValueError('Frame times must be finite, nonnegative and strictly increasing')
        times.append(time)
        state_hash.update(np.asarray([time], dtype='<f8').tobytes())
        for body in transforms:
            if body not in frame['bodies']:
                raise ValueError(f'Missing recorded body {body!r} at {time}')
            pose = frame['bodies'][body]
            pos = np.asarray(pose['position_m'], dtype=float)
            rot = _rotation(pose['rotation_row_major'], f'{body} rotation')
            if pos.shape != (3,) or not np.isfinite(pos).all():
                raise ValueError(f'{body} position must be a finite SI vector')
            matrix = np.eye(4)
            matrix[:3, :3] = basis @ rot
            matrix[:3, 3] = basis @ (pos * 1000)
            transforms[body].append(matrix)
            state_hash.update(body.encode() + b'\0' + matrix.astype('<f8').tobytes())
    palette = dict(materials or {})
    palette.setdefault('default', Material('Native mesh', (.64, .68, .73), .45, .36))
    mat_names = sorted({row.get('material', 'default') for row in records})
    missing = set(mat_names) - palette.keys()
    if missing:
        raise ValueError(f'Missing display material declarations: {sorted(missing)}')
    parts, names, corners_by_body = [], set(), {body: [] for body in bodies}
    geometry_hash = hashlib.sha256()
    for row in records:
        part_name = f"{row['body']}__{row['name']}"
        if any(char in part_name for char in '/\\:*?"<>|'):
            raise ValueError(f'Mesh identity is not export-safe: {part_name!r}')
        if part_name in names:
            raise ValueError(f'Duplicate mesh identity {part_name}')
        names.add(part_name)
        vertices = np.asarray(row['vertices_mm'], dtype=float)
        faces_raw = np.asarray(row['triangles'])
        if (vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) < 3
                or not np.isfinite(vertices).all() or faces_raw.ndim != 2
                or faces_raw.shape[1] != 3 or not len(faces_raw)
                or faces_raw.dtype.kind not in 'iu' or faces_raw.min() < 0
                or faces_raw.max() >= len(vertices)):
            raise ValueError(f'Invalid triangle mesh {part_name}')
        faces = faces_raw.astype(np.int64)
        body = body_mapping[row['body']]
        geometry_hash.update(json.dumps([part_name, body, row.get('material', 'default')]).encode())
        geometry_hash.update(vertices.astype('<f8').tobytes())
        geometry_hash.update(faces.astype('<i8').tobytes())
        parts.append(mesh_part(part_name, vertices, faces,
            material=mat_names.index(row.get('material', 'default')), group=body,
            motion=body, provenance=row.get('provenance', 'imported-mesh'),
            role='Native body-local display mesh; collision geometry is separately authored'))
        lo, hi = vertices.min(axis=0), vertices.max(axis=0)
        corners_by_body[body].extend(itertools.product(*zip(lo, hi)))
    # Bound every recorded state using conservative local AABB corners; the
    # camera stays fixed while the physical assembly and free bodies move.
    extrema = []
    for body, corners in corners_by_body.items():
        vertices = np.asarray(corners)
        for matrix in transforms[body]:
            world = vertices @ matrix[:3, :3].T + matrix[:3, 3]
            extrema.extend([world.min(axis=0), world.max(axis=0)])
    bounds = np.array([np.min(extrema, axis=0), np.max(extrema, axis=0)])

    details = dict(metadata or {})
    details.update(truth_intent='inspection',
        motion_source='MuJoCo integrated rigid-body dynamics',
        motion_sampling='preceding recorded state, held; no interpolation or prescribed motion',
        model_sha256=simulation_result.get('model_sha256'),
        control_sha256=simulation_result.get('control_sha256'),
        replay_state_sha256=state_hash.hexdigest(),
        display_mesh_sha256=geometry_hash.hexdigest(),
        render_bounds_mm=bounds.tolist(), recorded_times_s=times,
        recorded_body_transforms_mm={body:[matrix.tolist() for matrix in rows] for body,rows in transforms.items()},
        recorded_part_bindings={part.name:part.motion for part in parts},
        world_basis_to_core=basis.tolist(), body_mapping=dict(body_mapping),
        qualification={'dynamics': 'Recorded integrated body state only; metrics remain in the simulation report',
                       'hardware': 'not qualified', 'continuous_clearance': 'not established by replay'})
    center = bounds.mean(axis=0)
    scale = float(max(bounds[1]-bounds[0])*.62)
    return Assembly(name, parts, [palette[key] for key in mat_names],
        views={'hero': View(az=235, el=24, scale=scale, target=tuple(center),
                          projection='orthographic', title=name,
                          note='Fresh native geometry at recorded integrated body poses')},
        metadata=details, motion_function=RecordedMotion(times,transforms))


def render_evidence(assembly, output, *, fps=8, size=(960, 720),
                    still_times=None, view_name='hero', export_geometry=True):
    """Render PNG/GIF and a static GLB using existing native export/render paths.

    Every image uses an actual recorded state. GIF frame selection is downsampled
    from the trace; no physics or motion is generated here. Animated GLB's linear
    interpolation is intentionally not used for this evidence path.
    """
    from .render import Studio, labelled
    from .truth import assert_renderable, write_truth_report
    from .exporters import export_glb
    if not math.isfinite(fps) or fps <= 0 or fps > 120:
        raise ValueError('Evidence fps must be in (0,120]')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    truth = assert_renderable(assembly, intent='inspection')
    times = assembly.metadata['recorded_times_s']
    requested = np.arange(times[0], times[-1] + 1e-12, 1/fps)
    indices = sorted({max(0, bisect_right(times, float(t))-1) for t in requested} | {len(times)-1})
    targets = still_times if still_times is not None else [times[0], times[len(times)//2], times[-1]]
    still_indices = {min(range(len(times)), key=lambda i: abs(times[i]-target)) for target in targets}
    view = assembly.views[view_name]
    studio = Studio(assembly, size=size, floor=view.floor, supersample=1)
    images, stills = [], []
    try:
        studio.set_view(view)
        for index in sorted(set(indices) | still_indices):
            time = times[index]
            studio.pose(time)
            image = labelled(studio.render(), assembly.name, f'Recorded state at t = {time:.4f} s',
                'Native mesh geometry / integrated poses / hardware not qualified',
                tag='CYBR GEO / PHYSICAL REPLAY')
            if index in still_indices:
                path = output / f'pose-{index:05d}.png'
                image.save(path)
                stills.append(dict(path=path.name, frame_index=index, time_s=time))
            if index in indices:
                images.append(image)
    finally:
        studio.close()
    # GIF stores centiseconds. Quantize cumulative timestamps, not individual
    # intervals: flooring each 125 ms frame to 120 ms speeds an 8 fps replay up.
    ticks = [round((times[index]-times[indices[0]])*100) for index in indices]
    durations = [max(10, (b-a)*10) for a,b in zip(ticks, ticks[1:])]
    durations.append(max(10, round(100/fps)*10))
    images[0].save(output/'integrated-motion.gif', save_all=True, append_images=images[1:],
                   duration=durations, loop=0, disposal=2)
    if export_geometry:
        export_glb(assembly, output/'integrated-pose.glb', time_seconds=times[indices[len(indices)//2]])
    write_truth_report(truth, output/'geometry-truth.json')
    receipt = dict(schema='cybrgeo-physical-replay-evidence/1',
        model_sha256=assembly.metadata['model_sha256'], control_sha256=assembly.metadata['control_sha256'],
        replay_state_sha256=assembly.metadata['replay_state_sha256'],
        display_mesh_sha256=assembly.metadata['display_mesh_sha256'],
        part_count=len(assembly.parts), triangle_count=sum(len(p.faces) for p in assembly.parts),
        stills=stills, gif_frame_indices=indices, gif_recorded_times_s=[times[i] for i in indices],
        gif_frame_durations_ms=durations, gif_duration_ms=sum(durations),
        static_glb_time_s=times[indices[len(indices)//2]] if export_geometry else None,
        camera=dict(azimuth_deg=view.az,elevation_deg=view.el,half_height_mm=view.scale,
                    target_mm=list(view.target),projection=view.projection,image_size_px=list(size)),
        sampling=assembly.metadata['motion_sampling'], limitations=assembly.metadata.get('visual_limitations', []),
        qualification=assembly.metadata['qualification'],
        files_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(output.iterdir())
                      if p.is_file() and p.name != 'replay-evidence.json'})
    (output/'replay-evidence.json').write_text(json.dumps(receipt, indent=2)+'\n', encoding='utf-8')
    return receipt
