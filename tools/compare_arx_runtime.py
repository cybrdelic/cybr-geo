"""Compare actual source-validator traces with the shared native runtime.

This is an example-specific comparison of observations, not a simulator or a
hardware qualification. It does not reinterpret the source validator's pass
flags (its friction-loss scenario deliberately omits the placement checks).
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np

from mechanism_lab.physical_model import digest


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def compare(source, shared, targets):
    original = json.loads(source.read_text(encoding='utf-8'))
    actual = json.loads(gzip.decompress(shared.read_bytes()))
    if (actual.get('reference_identity') != 'explicit source record'
            or digest(actual.get('reference_source')) != actual.get('reference_sha256')
            or digest(actual.get('physical_source')) != actual.get('model_sha256')):
        raise ValueError('Shared comparison trace requires matching model and reference source receipts')
    control = json.loads((shared.parent/'control-source.json').read_text(encoding='utf-8'))
    if digest(control) != actual['control_sha256']:
        raise ValueError('Shared controller source does not match the trace')
    a = np.asarray(original['frames'], float)
    frames = actual['frames']
    times = np.asarray([f['time_s'] for f in frames])
    if len(a) != len(frames) or not np.allclose(a[:, 0], times, atol=1e-11, rtol=0):
        raise ValueError('Comparison requires the same recorded time grid')
    measured = {
        'joint_position_rad': ([f['qpos'][:6] for f in frames], a[:, 1:7]),
        'joint_velocity_rad_s': ([f['qvel'][:6] for f in frames], a[:, 7:13]),
        'jaw_position_m': ([f['qpos'][6:8] for f in frames], a[:, 13:15]),
        'workpiece_position_m': ([f['qpos'][8:11] for f in frames], a[:, 15:18]),
        'workpiece_quaternion_wxyz': ([f['qpos'][11:15] for f in frames], a[:, 18:22]),
        'motor_current_A': ([[m['current_A'] for m in f['motors']] for f in frames], a[:, 22:28]),
        'motor_torque_Nm': ([[m['torque_Nm'] for m in f['motors']] for f in frames], a[:, 28:34]),
        'motor_temperature_C': ([[m['temperature_C'] for m in f['motors']] for f in frames], a[:, 38:44]),
    }
    maxima = {name: float(np.max(np.abs(np.asarray(values)-reference)))
              for name, (values, reference) in measured.items()}
    if original.get('poses'):
        maxima['body_position_m'] = max(float(np.max(np.abs(np.asarray(frame['bodies'][name]['position_m'])-
            pose['position_m']))) for frame, poses in zip(frames, original['poses']) for name, pose in poses.items())
        maxima['body_rotation_matrix_component'] = max(float(np.max(np.abs(np.asarray(frame['bodies'][name]['rotation_row_major'])-
            pose['rotation_row_major']))) for frame, poses in zip(frames, original['poses']) for name, pose in poses.items())
    pad_normal = []
    for frame in frames:
        pad_normal.append([sum(max(0, c['normal_N']) for c in frame['contacts']
                               if set(c['geoms']) == {'puck', pad}) for pad in ('pad_left', 'pad_right')])
    maxima['pad_normal_N'] = float(np.max(np.abs(np.asarray(pad_normal)-a[:, 34:36])))
    placement = {}
    for name, target_time, target in [('B', 3.75, targets['B']), ('A', 8.75, targets['A'])]:
        if target_time > times[-1]:
            continue
        k = int(np.argmin(np.abs(times-target_time)))
        placement[name] = dict(time_s=float(times[k]), shared_error_m=float(np.linalg.norm(np.asarray(frames[k]['qpos'][8:11])-target)),
                              source_error_m=float(np.linalg.norm(a[k, 15:18]-target)))
    return dict(source_path=str(source), source_sha256=sha(source), shared_path=str(shared), shared_sha256=sha(shared),
                shared_model_sha256=actual['model_sha256'], shared_control_sha256=actual['control_sha256'],
                shared_reference_sha256=actual['reference_sha256'], shared_reference_identity=actual['reference_identity'],
                frames=len(frames), duration_s=actual['duration_s'], solver=actual['mujoco_version'],
                maximum_absolute_differences=maxima, placement_at_sample=placement,
                source_original_validator_passed=original['metrics']['all_checks_pass'],
                shared_events=actual['events'], shared_metrics=actual['metrics'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, required=True)
    parser.add_argument('--shared-root', type=Path, required=True)
    parser.add_argument('--parameters', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    targets = json.loads(args.parameters.read_text(encoding='utf-8'))['workpiece_targets_m']
    report = dict(schema='cybrgeo-arx-runtime-comparison/1',
                  scope='Observed source-WASM versus shared-native integration; no hardware qualification', scenarios={})
    for name in ('nominal', 'friction_loss'):
        report['scenarios'][name] = compare(args.source_root/name/'results'/f'{name}.json',
                                           args.shared_root/name/'trace.json.gz', targets)
    traces = [json.loads(gzip.decompress((args.shared_root/name/'trace.json.gz').read_bytes()))['frames']
              for name in ('nominal', 'friction_loss')]
    normal, loss = traces
    original_xml_run = args.shared_root/'original_xml_native'/'trace.json.gz'
    if original_xml_run.exists():
        original_native = json.loads(gzip.decompress(original_xml_run.read_bytes()))['frames']
        if len(original_native) != len(normal):
            raise ValueError('Original XML isolation run uses a different time grid')
        report['original_xml_isolation'] = dict(
            scope='Same shared controller, native solver and task; exact source XML versus shared model lowering',
            original_xml_trace_sha256=sha(original_xml_run),
            maximum_qpos_difference=max(float(np.max(np.abs(np.asarray(a['qpos'])-b['qpos'])))
                for a, b in zip(normal, original_native)),
            maximum_current_difference_A=max(abs(u['current_A']-v['current_A'])
                for a, b in zip(normal, original_native) for u, v in zip(a['motors'], b['motors'])),
            maximum_body_position_difference_m=max(float(np.max(np.abs(np.asarray(a['bodies'][name]['position_m'])-
                b['bodies'][name]['position_m']))) for a, b in zip(normal, original_native) for name in a['bodies']))
    samples = []
    for desired in (1.9, 2., 2.2, 2.5, 3., 3.4):
        k = min(range(len(loss)), key=lambda i: abs(loss[i]['time_s']-desired))
        samples.append(dict(time_s=loss[k]['time_s'], nominal_workpiece_m=normal[k]['qpos'][8:11],
                            friction_loss_workpiece_m=loss[k]['qpos'][8:11]))
    report['shared_contact_effect'] = dict(samples=samples,
        maximum_state_difference_before_change=max(float(np.max(np.abs(np.asarray(a['qpos'])-b['qpos'])))
            for a, b in zip(normal, loss) if a['time_s'] <= 2.),
        maximum_height_drop_before_commanded_release_m=max(a['qpos'][9]-b['qpos'][9]
            for a, b in zip(normal, loss) if 2. < a['time_s'] < 3.4))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
