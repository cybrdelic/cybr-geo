"""Compare two integrated traces of the same model, controller and reference.

Reports the observed effect of a recorded options intervention. This does not
infer hardware qualification or certify unsampled motion between output frames.
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np

from mechanism_lab.physical_model import PhysicalModel, digest


def read(path):
    payload = path.read_bytes()
    result = json.loads(gzip.decompress(payload))
    if result.get('schema') != 'cybrgeo-causal-simulation/1':
        raise ValueError('Expected an integrated causal simulation trace')
    if digest(result.get('physical_source')) != result.get('model_sha256'):
        raise ValueError('Trace physical source identity differs')
    if (result.get('reference_identity') != 'explicit source record'
            or digest(result.get('reference_source')) != result.get('reference_sha256')):
        raise ValueError('Trace reference source identity differs')
    return result, hashlib.sha256(payload).hexdigest()


def compare(baseline_path, intervention_path, body, change_time, before):
    baseline, baseline_sha = read(baseline_path)
    intervention, intervention_sha = read(intervention_path)
    for key in ('model_sha256', 'control_sha256', 'reference_sha256', 'timestep_s', 'mujoco_version'):
        if baseline[key] != intervention[key]:
            raise ValueError(f'Traces cannot isolate one intervention: {key} differs')
    PhysicalModel.from_source(baseline['physical_source']).require_free_body(body)
    a, b = baseline['frames'], intervention['frames']
    times = np.asarray([frame['time_s'] for frame in a])
    if len(a) != len(b) or not np.allclose(times, [frame['time_s'] for frame in b], atol=1e-12, rtol=0):
        raise ValueError('Counterfactual comparison requires identical output time grids')
    if not (times[0] < change_time < before <= times[-1]):
        raise ValueError('Comparison window must follow the intervention within both traces')
    differences = {key: {'baseline': baseline['options'].get(key), 'intervention': intervention['options'].get(key)}
                   for key in baseline['options'].keys() | intervention['options'].keys()
                   if baseline['options'].get(key) != intervention['options'].get(key)}
    if not differences:
        raise ValueError('No recorded runtime option differs')
    pre = times <= change_time
    q_difference = np.asarray([x['qpos'] for x in a])-np.asarray([x['qpos'] for x in b])
    if not np.isfinite(q_difference).all():
        raise ValueError('Trace contains nonfinite generalized coordinates')
    before_difference = float(np.max(np.abs(q_difference[pre])))
    if before_difference > 1e-12:
        raise ValueError(f'States already differ before intervention: {before_difference:g}')
    positions_a = np.asarray([frame['bodies'][body]['position_m'] for frame in a])
    positions_b = np.asarray([frame['bodies'][body]['position_m'] for frame in b])
    selection = (times > change_time) & (times < before)
    if not selection.any() or not np.isfinite(positions_a).all() or not np.isfinite(positions_b).all():
        raise ValueError('Comparison window requires finite recorded body positions')
    delta = positions_b-positions_a
    distance = np.linalg.norm(delta, axis=1)
    k = int(np.flatnonzero(selection)[np.argmax(distance[selection])])
    free = {}
    for name, result in [('baseline', baseline), ('intervention', intervention)]:
        evidence = result['metrics']['free_bodies'][body]
        if evidence['max_imposed_generalized_force'] != 0 or evidence['max_applied_wrench_component'] != 0:
            raise ValueError(f'{name} applies an imposed force to the free body')
        free[name] = evidence
    return dict(schema='cybrgeo-causal-counterfactual/1', body=body,
                baseline=dict(path=str(baseline_path), sha256=baseline_sha, requirements=baseline.get('requirements')),
                intervention=dict(path=str(intervention_path), sha256=intervention_sha, requirements=intervention.get('requirements')),
                model_sha256=baseline['model_sha256'], control_sha256=baseline['control_sha256'],
                reference_sha256=baseline['reference_sha256'], options_difference=differences,
                change_time_s=change_time, comparison_before_s=before,
                maximum_state_difference_before_intervention=before_difference,
                maximum_free_body_position_divergence_m=float(distance[k]), time_of_maximum_s=float(times[k]),
                divergence_vector_m=delta[k].tolist(), free_body_force_monitors=free,
                scope='Same model/control/reference and zero imposed free-body forces; comparison of recorded solver states, not hardware qualification')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--intervention', type=Path, required=True)
    parser.add_argument('--body', required=True)
    parser.add_argument('--change-time', type=float, required=True)
    parser.add_argument('--before', type=float, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    report = compare(args.baseline, args.intervention, args.body, args.change_time, args.before)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
