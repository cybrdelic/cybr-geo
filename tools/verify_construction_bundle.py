"""Reopen the actual STEP/GLB deliverables and compare their sampled motion.

Run with ``PYTHONPATH=src python tools/verify_construction_bundle.py BUNDLE
--out report.json``. This checks artifact identity, prescribed motion and
integrated GLB replay against recorded MuJoCo states; it does not independently
qualify the physical assumptions, continuous collision proof or hardware.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import struct

import numpy as np

from mechanism_lab.construction import compile_plan, compile_mechanism, recipe_digest, validate_constructed_plan, _canonical
from mechanism_lab.constructed import KinematicGraph
from mechanism_lab.exporters import NATIVE_TO_GLTF


def read_glb(path):
    raw = Path(path).read_bytes()
    if len(raw) < 28:
        raise ValueError('GLB is truncated')
    magic, version, total = struct.unpack_from('<III', raw)
    if (magic, version, total) != (0x46546C67, 2, len(raw)):
        raise ValueError('GLB header does not match its actual bytes')
    length, kind = struct.unpack_from('<II', raw, 12)
    if kind != 0x4E4F534A or 28 + length > len(raw):
        raise ValueError('GLB has no complete JSON chunk')
    document = json.loads(raw[20:20 + length])
    size, kind = struct.unpack_from('<II', raw, 20 + length)
    if kind != 0x004E4942 or 28 + length + size != len(raw):
        raise ValueError('GLB has no complete binary chunk')
    return document, raw[28 + length:]


def accessor(document, binary, index):
    item = document['accessors'][index]
    if 'sparse' in item or item.get('componentType') != 5126:
        raise ValueError('Expected a dense float32 construction accessor')
    view = document['bufferViews'][item['bufferView']]
    width = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}[item['type']]
    stride = view.get('byteStride', 4 * width)
    offset = view.get('byteOffset', 0) + item.get('byteOffset', 0)
    count = item['count']
    if view.get('buffer', 0) != 0 or count < 1 or stride < 4 * width or offset + (count - 1) * stride + 4 * width > len(binary):
        raise ValueError('GLB accessor points outside its actual binary buffer')
    result = np.ndarray((count, width), dtype='<f4', buffer=binary, offset=offset, strides=(stride, 4))
    if not np.isfinite(result).all():
        raise ValueError('GLB accessor contains nonfinite values')
    return result


def mesh_nodes(document):
    result = {}
    for index, node in enumerate(document['nodes']):
        if 'mesh' not in node:
            continue
        name = node.get('name')
        if not isinstance(name, str) or name in result:
            raise ValueError('GLB mesh node names must be unique')
        result[name] = index
    return result


def node_matrix(node):
    from scipy.spatial.transform import Rotation
    if 'matrix' in node:
        return np.asarray(node['matrix'], float).reshape(4, 4).T
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(node.get('rotation', [0, 0, 0, 1])).as_matrix() @ np.diag(node.get('scale', [1, 1, 1]))
    result[:3, 3] = node.get('translation', [0, 0, 0])
    return result


def positions(document, binary, node_index):
    primitives = document['meshes'][document['nodes'][node_index]['mesh']]['primitives']
    if len(primitives) != 1:
        raise ValueError('Expected one primitive per exported construction part')
    return accessor(document, binary, primitives[0]['attributes']['POSITION'])


def verify_motion(static, moving, graph, sample_count=17, expected_transforms=None):
    if not isinstance(sample_count, int) or not 2 <= sample_count <= 129:
        raise ValueError('Motion verification requires 2-129 requested samples')
    doc, binary = moving
    static_doc, static_binary = static
    names = mesh_nodes(doc)
    static_names = mesh_nodes(static_doc)
    if set(names) != set(graph.part_names) or set(static_names) != set(names):
        raise ValueError('Static, animated and source part names differ')
    for name, index in names.items():
        actual = positions(doc, binary, index)
        reference = positions(static_doc, static_binary, static_names[name])
        if actual.shape != reference.shape or not np.array_equal(actual, reference):
            raise ValueError(f'{name}: static and animated GLBs contain different geometry')
    animations = doc.get('animations', [])
    if len(animations) != 1 or not animations[0].get('channels'):
        raise ValueError('Expected one nonempty prescribed animation')
    animation = animations[0]
    channels, times = {}, None
    for channel in animation['channels']:
        sampler = animation['samplers'][channel['sampler']]
        if sampler.get('interpolation', 'LINEAR') != 'LINEAR':
            raise ValueError('Unsupported animation interpolation')
        clock = accessor(doc, binary, sampler['input'])[:, 0]
        values = accessor(doc, binary, sampler['output'])
        target = channel['target']
        key = (target['node'], target['path'])
        if key in channels or target['path'] not in ('rotation', 'translation', 'scale') or 'matrix' in doc['nodes'][target['node']]:
            raise ValueError('Invalid or duplicate animated node target')
        if len(values) != len(clock) or (times is not None and not np.array_equal(times, clock)):
            raise ValueError('Animation samplers do not share a consistent clock')
        channels[key] = values
        times = clock
    if len(times) < 2 or times[0] != 0 or np.any(np.diff(times) <= 0):
        raise ValueError('Animation clock must strictly increase from zero')
    parents = {}
    for index, node in enumerate(doc['nodes']):
        for child in node.get('children', []):
            if child in parents:
                raise ValueError('Animation hierarchy has more than one parent')
            parents[child] = index
    roots = set(doc['scenes'][doc.get('scene', 0)]['nodes'])
    selected = sorted(set(np.linspace(0, len(times) - 1, min(sample_count, len(times)), dtype=int)))
    inverse = np.linalg.inv(NATIVE_TO_GLTF)
    max_rotation = max_translation = max_vertex = 0.
    for sample in selected:
        nodes = [dict(node) for node in doc['nodes']]
        for (index, component), values in channels.items():
            nodes[index][component] = values[sample].tolist()
        matrices = {}

        def world(index, seen=None):
            if index in matrices:
                return matrices[index]
            seen = set() if seen is None else set(seen)
            if index in seen:
                raise ValueError('Animation hierarchy contains a cycle')
            seen.add(index)
            result = node_matrix(nodes[index])
            if index in parents:
                result = world(parents[index], seen) @ result
            elif index not in roots:
                raise ValueError('Part node is not reachable from the active scene')
            matrices[index] = result
            return result

        expected = (expected_transforms(float(times[sample])) if expected_transforms is not None
                    else graph.transforms(graph.demo_values(float(times[sample]))))
        for name, index in names.items():
            exported = world(index)
            native = inverse @ exported
            target = expected[name]
            max_rotation = max(max_rotation, float(np.max(np.abs(native[:3, :3] - target[:3, :3]))))
            max_translation = max(max_translation, float(np.max(np.abs(native[:3, 3] - target[:3, 3]))))
            vertices = positions(doc, binary, index)[[0, len(positions(doc, binary, index)) // 2, -1]]
            actual = vertices @ native[:3, :3].T + native[:3, 3]
            reference = vertices @ target[:3, :3].T + target[:3, 3]
            max_vertex = max(max_vertex, float(np.max(np.abs(actual - reference))))
    if max_rotation > 2e-6 or max_translation > .002 or max_vertex > .002:
        raise ValueError(f'Exported motion differs from construction: rotation={max_rotation}, translation_mm={max_translation}, vertex_mm={max_vertex}')
    return {'parts': len(names), 'channels': len(channels), 'stored_keyframes': len(times),
            'sampled_times_s': [float(times[i]) for i in selected], 'duration_s': float(times[-1]),
            'maximum_rotation_element_error': max_rotation, 'maximum_translation_error_mm': max_translation,
            'maximum_sampled_vertex_error_mm': max_vertex,
            'scope': 'Decoded artifact node transforms at the listed keyframes, including mm/Z-up to m/Y-up conversion. No continuous interpolation or collision proof.'}


def verify_simulated_motion(static, moving, graph, simulation, sample_count=17):
    from types import SimpleNamespace
    from mechanism_lab.dynamics import SimulatedMotion
    if simulation.get('backend') != 'MuJoCo' or not simulation.get('samples'):
        raise ValueError('Integrated replay requires an executed MuJoCo trace')
    motion = SimulatedMotion(simulation)
    parts = {name: SimpleNamespace(name=name, explode=np.zeros(3)) for name in graph.part_names}
    def expected(time):
        return {name: motion(part, time) for name, part in parts.items()}
    report = verify_motion(static, moving, graph, sample_count, expected)
    if not math.isclose(report['duration_s'], simulation['seconds'], abs_tol=1e-5):
        raise ValueError('Integrated GLB and simulation durations differ')
    report['scope'] = 'Actual GLB decoded transforms compared with recorded MuJoCo body states and the declared interpolation; no new physics or hardware qualification.'
    return report


def verify_mujoco_states(model, manifest, simulation, sample_count=17):
    """Recompute recorded body poses from actual reopened MJCF and joint state.

    This checks serialized state/frame consistency, not a second integration of
    the equations or independent validation of the modeling assumptions.
    """
    import mujoco as mj
    from scipy.spatial.transform import Rotation
    data = mj.MjData(model)
    samples = simulation['samples']
    indices = sorted(set(np.linspace(0, len(samples) - 1, min(sample_count, len(samples)), dtype=int)))
    error = 0.
    for index in indices:
        sample = samples[index]
        for name, factor in manifest['joint_native_to_SI'].items():
            jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
            data.qpos[model.jnt_qposadr[jid]] = sample['joint_values'][name] * factor
        for name in manifest.get('free_bodies', []):
            transform = np.array(sample['body_poses_mm'][name])
            jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'free_' + name)
            adr = int(model.jnt_qposadr[jid])
            data.qpos[adr:adr + 3] = transform[:3, 3] / 1000
            data.qpos[adr + 3:adr + 7] = Rotation.from_matrix(transform[:3, :3]).as_quat()[[3, 0, 1, 2]]
        mj.mj_kinematics(model, data)
        for name, matrix in sample['body_poses_mm'].items():
            bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, name)
            if bid < 0:
                raise ValueError(f'Simulation body {name} is absent from reopened MJCF')
            expected = np.eye(4)
            expected[:3, :3] = data.xmat[bid].reshape(3, 3)
            expected[:3, 3] = data.xpos[bid] * 1000
            error = max(error, float(np.max(np.abs(expected - np.asarray(matrix)))))
    if error > 1e-6:
        raise ValueError(f'Recorded MuJoCo body transforms disagree with reopened MJCF joint states: {error}')
    return dict(sampled_states=len(indices), maximum_pose_component_error_mm_or_dimensionless=error,
                scope='Reopened MJCF forward kinematics at recorded joint states; does not rerun dynamics')


def verify_bundle(directory, sample_count=17):
    directory = Path(directory).resolve()
    report = {'bundle': str(directory), 'passed': False, 'errors': [],
              'scope': 'Artifact identity, reopened STEP/GLB/MJCF and sampled prescribed/integrated motion replay. No hardware qualification.'}
    try:
        import cadquery as cq
        import trimesh
        receipt_path = directory / 'bundle.json'
        receipt = json.loads(receipt_path.read_text(encoding='utf-8'))
        report['bundle_receipt_sha256'] = hashlib.sha256(receipt_path.read_bytes()).hexdigest()
        hashes = receipt['files_sha256']
        required = {'recipe.json', 'mechanism.json', 'assembly.glb', 'motion.glb', 'assembly.step'}
        if not required <= hashes.keys():
            raise ValueError('A complete bundle with --step is required; manifest lacks ' + ', '.join(sorted(required - hashes.keys())))
        for name, digest in hashes.items():
            file = (directory / name).resolve()
            if not file.is_relative_to(directory) or not file.is_file():
                raise ValueError(f'Manifest file is missing or outside the bundle: {name}')
            if hashlib.sha256(file.read_bytes()).hexdigest() != digest:
                raise ValueError(f'Manifest hash mismatch: {name}')
        report['manifest'] = {'files_verified': len(hashes), 'all_sha256_match': True}
        plan = json.loads((directory / 'recipe.json').read_text(encoding='utf-8'))
        descriptor = json.loads((directory / 'mechanism.json').read_text(encoding='utf-8'))
        digest = recipe_digest(plan)
        validate_constructed_plan(plan)
        if digest != plan['construction']['recipe_sha256'] or digest != receipt['construction_recipe_sha256']:
            raise ValueError('Recipe, construction and bundle identities disagree')
        canonical = descriptor.get('schema') == 'cybrgeo-mechanism/2'
        current = compile_mechanism(descriptor) if canonical else compile_plan(descriptor['architecture'], descriptor.get('parameters'))
        if recipe_digest(current) != digest:
            raise ValueError('Bundle is stale relative to the current construction compiler and saved parameters')
        if canonical:
            if descriptor != plan['construction']['mechanism']:
                raise ValueError('Saved canonical source differs from the compiled mechanism source')
            for key in ('model_sha256', 'mechanism_sha256'):
                if receipt.get(key) != plan['construction'].get(key):
                    raise ValueError(f'Bundle {key} differs from the full mechanical source')
            if current['construction']['mechanism_sha256'] != plan['construction']['mechanism_sha256']:
                raise ValueError('Bundle is stale relative to canonical mechanism validation')
            if current['construction']['model_sha256'] != plan['construction']['model_sha256']:
                raise ValueError('Bundle is stale relative to the complete compiled mechanical model')
            # The complete model hash above uses the compiler's documented
            # nine-decimal numeric canonicalization. Repeating raw float
            # equality here rejects harmless roundoff in engineering sums.
        report['recipe'] = {'architecture': plan['construction'].get('architecture'), 'sha256': digest,
                            'mechanism_sha256': plan['construction'].get('mechanism_sha256'),
                            'model_sha256': plan['construction'].get('model_sha256'), 'matches_current_compiler': True}
        names = [part['name'] for part in plan['parts']]
        graph = KinematicGraph(names, plan['joints'], plan['construction'].get('motion'))
        if receipt['parts'] != len(names):
            raise ValueError('Bundle component count disagrees with its recipe')
        static = read_glb(directory / 'assembly.glb')
        if set(mesh_nodes(static[0])) != set(names):
            raise ValueError('Static GLB names differ from the construction recipe')
        scene = trimesh.load_scene(directory / 'assembly.glb', process=False)
        if len(scene.geometry) != len(names) or not np.isfinite(scene.bounds).all():
            raise ValueError('Reopened static GLB has wrong geometry count or invalid bounds')
        report['static_glb'] = {'named_mesh_nodes': len(mesh_nodes(static[0])), 'reopened_geometries': len(scene.geometry),
                                'bounds_m_y_up': scene.bounds.tolist()}
        imported = cq.importers.importStep(str(directory / 'assembly.step'))
        solids = imported.solids().vals()
        volumes = [float(solid.Volume()) for solid in solids]
        if len(solids) != len(names) or not all(solid.isValid() for solid in solids) or not all(math.isfinite(v) and v > .001 for v in volumes):
            raise ValueError('Reopened STEP does not contain the expected valid positive-volume solids')
        boxes = [solid.BoundingBox() for solid in solids]
        step_bounds = np.array([[min(getattr(b, key) for b in boxes) for key in ('xmin', 'ymin', 'zmin')],
                                [max(getattr(b, key) for b in boxes) for key in ('xmax', 'ymax', 'zmax')]])
        corners = np.array([[x, y, z] for x in scene.bounds[:, 0] for y in scene.bounds[:, 1] for z in scene.bounds[:, 2]])
        inverse = np.linalg.inv(NATIVE_TO_GLTF)
        corners = corners @ inverse[:3, :3].T + inverse[:3, 3]
        mesh_bounds = np.array([corners.min(axis=0), corners.max(axis=0)])
        bound_error = float(np.max(np.abs(step_bounds - mesh_bounds)))
        if bound_error > .25:
            raise ValueError(f'STEP/GLB reference envelopes disagree by {bound_error} mm')
        report['step'] = {'reopened_solids': len(solids), 'all_analytic_valid': True,
                          'minimum_solid_volume_mm3': min(volumes), 'total_solid_volume_mm3': sum(volumes),
                          'bounds_mm_z_up': step_bounds.tolist(), 'maximum_step_glb_bound_error_mm': bound_error}
        report['motion_glb'] = verify_motion(static, read_glb(directory / 'motion.glb'), graph, sample_count)
        if report['motion_glb']['stored_keyframes'] != receipt['animation']['samples'] or not math.isclose(report['motion_glb']['duration_s'], receipt['animation']['duration'], abs_tol=1e-5):
            raise ValueError('Decoded animation timing differs from the bundle receipt')
        if canonical:
            from mechanism_lab.dynamics import load_model
            required_simulation = {'simulation/model.json', 'simulation/model.xml', 'simulation/simulation.json', 'simulation/integrated-motion.glb'}
            if not required_simulation <= hashes.keys():
                raise ValueError('Canonical engineering bundle lacks its actual simulation artifacts')
            simulation = json.loads((directory / 'simulation/simulation.json').read_text())
            for key in ('recipe_sha256', 'mechanism_sha256', 'model_sha256'):
                if simulation.get(key) != plan['construction'].get(key):
                    raise ValueError(f'Simulation {key} differs from the construction model')
            model, manifest = load_model(directory / 'simulation')
            if simulation.get('model') != manifest:
                raise ValueError('Simulation and reopened MJCF manifests differ')
            if _canonical(manifest['settings']) != _canonical(plan['construction']['dynamics']):
                raise ValueError('MJCF physical settings differ from the canonical source')
            trace_meta = simulation['integration_trace_artifact']
            trace_path = (directory / 'simulation' / trace_meta['path']).resolve()
            if not trace_path.is_relative_to(directory / 'simulation') or 'simulation/' + trace_meta['path'] not in hashes:
                raise ValueError('Integration trace is not a declared simulation artifact')
            compressed = trace_path.read_bytes()
            if hashlib.sha256(compressed).hexdigest() != trace_meta['sha256']:
                raise ValueError('Compressed integration trace identity differs from simulation')
            trace = json.loads(gzip.decompress(compressed))
            if len(trace['times_s']) != simulation['integration_steps'] + 1 or len(trace['joint_values']) != len(trace['times_s']):
                raise ValueError('Integration trace does not contain every declared solver state')
            report['mujoco'] = dict(reopened_bodies=int(model.nbody), generalized_velocities=int(model.nv),
                                    actuators=int(model.nu), backend_version=simulation['backend_version'],
                                    trace_states=len(simulation['samples']),
                                    integration_steps=simulation['integration_steps'],
                                    tracking_passed=simulation['tracking_passed'],
                                    operating_limits_passed=simulation['operating_limits_passed'])
            report['mujoco']['state_readback'] = verify_mujoco_states(model, manifest, simulation, sample_count)
            report['mujoco']['stored_integration_states'] = len(trace['times_s'])
            report['integrated_motion_glb'] = verify_simulated_motion(static,
                read_glb(directory / 'simulation/integrated-motion.glb'), graph, simulation, sample_count)
            timing = receipt['simulated_animation']
            if report['integrated_motion_glb']['stored_keyframes'] != timing['samples'] or not math.isclose(report['integrated_motion_glb']['duration_s'], timing['duration'], abs_tol=1e-5):
                raise ValueError('Decoded integrated animation timing differs from the bundle receipt')
        report['passed'] = True
    except Exception as error:
        report['errors'].append(f'{type(error).__name__}: {error}')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bundle', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--samples', type=int, default=17)
    args = parser.parse_args(argv)
    if not 2 <= args.samples <= 129:
        parser.error('--samples must be between 2 and 129')
    receipt = args.bundle / 'bundle.json'
    if args.out.resolve() == receipt.resolve():
        parser.error('--out must not overwrite the bundle receipt')
    if receipt.exists():
        try:
            manifest = json.loads(receipt.read_text(encoding='utf-8'))
            entries = manifest.get('files_sha256', {}) if isinstance(manifest, dict) else {}
            inputs = [(args.bundle / name).resolve() for name in entries] if isinstance(entries, dict) else []
        except (OSError, ValueError):
            inputs = []  # verify_bundle records the malformed receipt as an error.
        if args.out.resolve() in [receipt.resolve(), *inputs]:
            parser.error('--out must not overwrite a bundle input or its receipt')
    report = verify_bundle(args.bundle, args.samples)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
