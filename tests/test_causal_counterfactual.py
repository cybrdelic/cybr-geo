"""Receipt and intervention-isolation checks for saved runtime observations."""
from copy import deepcopy
import gzip
import importlib.util
import json
from pathlib import Path

import pytest

from mechanism_lab.physical_model import PhysicalModel, digest


path = Path(__file__).resolve().parents[1]/'tools'/'compare_causal_counterfactual.py'
spec = importlib.util.spec_from_file_location('counterfactual_comparison', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def trace():
    source = PhysicalModel.from_xml('<mujoco><worldbody><body name="sample"><freejoint/>'
        '<geom type="sphere" size=".01" mass=".1"/></body></worldbody></mujoco>').to_source()
    ref = {'task': 'fixture'}
    return dict(schema='cybrgeo-causal-simulation/1', physical_source=source, model_sha256=digest(source),
                control_sha256='a'*64, reference_source=ref, reference_sha256=digest(ref),
                reference_identity='explicit source record', timestep_s=.001, mujoco_version='3.16.0',
                options={'power': True}, metrics={'free_bodies': {'sample': {
                    'max_imposed_generalized_force': 0., 'max_applied_wrench_component': 0.}}},
                frames=[{'time_s': t, 'qpos': [0, 0, 0, 1, 0, 0, 0],
                         'bodies': {'sample': {'position_m': [0, 0, 0]}}} for t in (0., 1., 2., 3.)])


def pair(tmp_path, mutate=None):
    baseline = trace()
    intervention = deepcopy(baseline)
    intervention['options']['power'] = False
    intervention['frames'][2]['qpos'][0] = .1
    intervention['frames'][2]['bodies']['sample']['position_m'][0] = .1
    if mutate:
        mutate(intervention)
    paths = [tmp_path/'a.json.gz', tmp_path/'b.json.gz']
    for path, value in zip(paths, (baseline, intervention)):
        path.write_bytes(gzip.compress(json.dumps(value).encode()))
    return paths


def test_saved_state_effect_and_file_hashes(tmp_path):
    a, b = pair(tmp_path)
    result = module.compare(a, b, 'sample', 1., 3.)
    assert result['maximum_free_body_position_divergence_m'] == pytest.approx(.1)
    assert result['maximum_state_difference_before_intervention'] == 0
    assert result['time_of_maximum_s'] == 2
    assert result['baseline']['sha256'] != result['intervention']['sha256']


@pytest.mark.parametrize('mutation,reason', [
    (lambda t: t.update(control_sha256='b'*64), 'control_sha256'),
    (lambda t: t['frames'][0]['qpos'].__setitem__(0, .01), 'before intervention'),
    (lambda t: t['metrics']['free_bodies']['sample'].update(max_imposed_generalized_force=1), 'imposed force'),
    (lambda t: t['reference_source'].update(task='changed'), 'reference source'),
    (lambda t: t['frames'][2]['bodies']['sample']['position_m'].__setitem__(0, float('nan')), 'finite recorded'),
])
def test_reject_mismatched_inputs_preexisting_difference_or_imposed_force(tmp_path, mutation, reason):
    a, b = pair(tmp_path, mutation)
    with pytest.raises(ValueError, match=reason):
        module.compare(a, b, 'sample', 1., 3.)
