"""Native rigid-pose envelopes must reject unsampled collisions and stale inputs."""
from copy import deepcopy
import hashlib
import json

import cadquery as cq
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

import mechanism_lab.native_pose_envelope as envelope
from mechanism_lab.native_pose_envelope import (certify_native_parametric_path,
    certify_parametric_path, certify_recorded_path, group_native_parts_by_body,
    load_placed_native_parts,
    verify_implementation_identity, verify_recorded_identity)


def pose(x=0., y=0.):
    result = np.eye(4)
    result[:2, 3] = (x, y)
    return result


def fixture(y=0.):
    shape = cq.Workplane('XY').box(1, 1, 1).val()
    parts = {'fixed': {'body': 'base', 'shape': shape},
             'moving': {'body': 'slide', 'shape': shape}}
    frames = {'base': [pose(), pose()],
              'slide': [pose(-4, y), pose(4, y)]}
    return parts, frames


def test_slerp_path_rejects_collision_between_clear_recorded_frames():
    parts, frames = fixture()
    report = certify_recorded_path(parts, [0, 1], frames,
                                   interpolation='shortest_arc_slerp', max_depth=9)
    assert not report['passed']
    assert report['failures'][0]['parts'] == ['fixed', 'moving']
    assert 'Unresolved or colliding' in report['failures'][0]['reason']
    assert report['statistics']['brep_distance_queries'] > 0
    # The existing MP4 holder is a different, explicit playback contract.
    held = certify_recorded_path(parts, [0, 1], frames, interpolation='hold')
    assert held['passed']
    assert held['segments_completed'] == 2


def test_shortest_arc_rotational_sweep_rejects_hidden_contact():
    cube = cq.Workplane('XY').box(.4, .4, .4)
    parts = {'fixed': {'body': 'base',
                       'shape': cube.translate((1, 3**.5, 0)).val()},
             'rotor': {'body': 'shaft',
                       'shape': cq.Workplane('XY').box(.4, .4, .4).translate((2, 0, 0)).val()}}
    end = pose()
    end[:3, :3] = Rotation.from_euler('z', 120, degrees=True).as_matrix()
    frames = {'base': [pose(), pose()], 'shaft': [pose(), end]}
    report = certify_recorded_path(parts, [0, 1], frames,
                                   interpolation='shortest_arc_slerp', max_depth=9)
    assert not report['passed']
    assert report['failures'][0]['parts'] == ['fixed', 'rotor']


def test_clear_slerp_path_proves_all_intervals_and_binds_source():
    parts, frames = fixture(y=3)
    report = certify_recorded_path(parts, [0, 1], frames,
        interpolation='shortest_arc_slerp', required_clearance_mm=.1,
        source_identity='recording:abc')
    assert report['passed']
    assert report['complete_pair_accounting']
    assert report['clearance_pairs'][0]['minimum_clearance_lower_bound_mm'] >= .1
    assert report['clearance_pairs'][0]['certified_intervals'] >= 1
    assert verify_recorded_identity(report, parts, [0, 1], frames,
                                    source_identity='recording:abc')
    changed = deepcopy(frames)
    changed['slide'][1][0, 3] = 3.9
    with pytest.raises(ValueError, match='changed after certification'):
        verify_recorded_identity(report, parts, [0, 1], changed,
                                 source_identity='recording:abc')
    with pytest.raises(ValueError, match='changed after certification'):
        verify_recorded_identity(report, parts, [0, 1], frames,
                                 source_identity='recording:other')
    changed_parts = dict(parts)
    changed_parts['moving'] = {'body': 'slide',
        'shape': cq.Workplane('XY').box(1.01, 1, 1).val()}
    with pytest.raises(ValueError, match='changed after certification'):
        verify_recorded_identity(report, changed_parts, [0, 1], frames,
                                 source_identity='recording:abc')


def test_pair_exclusions_are_explicit_and_unselected_coverage_is_visible():
    parts, frames = fixture(y=3)
    parts['mating'] = {'body': 'jaw', 'shape': cq.Workplane('XY').box(1, 1, 1).val()}
    frames['jaw'] = [pose(), pose()]
    report = certify_recorded_path(parts, [0, 1], frames, interpolation='hold',
                                   excluded_pairs={('fixed', 'mating'): 'declared journal contact'})
    assert report['passed']
    assert report['cross_body_pairs'] == 3
    assert report['complete_pair_accounting']
    assert report['excluded_pairs'] == [
        {'parts': ['fixed', 'mating'], 'reason': 'declared journal contact'}]
    assert len(report['clearance_pairs']) == 2
    subset = certify_recorded_path(parts, [0, 1], frames, interpolation='hold',
                                   clearance_pairs=[('fixed', 'moving')])
    assert subset['passed']
    assert not subset['complete_pair_accounting']
    assert subset['unexamined_cross_body_pairs'] == 2
    with pytest.raises(ValueError, match='both clearance-certified and excluded'):
        certify_recorded_path(parts, [0, 1], frames, interpolation='hold',
            clearance_pairs=[('fixed', 'mating')],
            excluded_pairs={('fixed', 'mating'): 'contact'})


def test_nonrigid_or_missing_recorded_pose_rejected():
    parts, frames = fixture(y=3)
    frames['slide'][1][0, 0] = -1  # reflection
    with pytest.raises(ValueError, match='proper 4x4 rigid pose'):
        certify_recorded_path(parts, [0, 1], frames)
    frames['slide'][1] = pose(4, 3)
    del frames['base']
    with pytest.raises(ValueError, match='cover every bound body'):
        certify_recorded_path(parts, [0, 1], frames)


def test_unresolved_interval_budget_fails_closed():
    parts, frames = fixture(y=3)
    report = certify_recorded_path(parts, [0, 1], frames,
        interpolation='shortest_arc_slerp', max_depth=0)
    assert not report['passed']
    assert 'Unresolved or colliding' in report['failures'][0]['reason']


def test_parametric_bound_checks_interior_and_needs_source_identity():
    parts, _ = fixture()
    def path(u):
        return {'base': pose(), 'slide': pose(-4+8*u)}
    def speed(name, _lo, _hi):
        return 0. if name == 'fixed' else 8.
    report = certify_parametric_path(parts, path, speed,
                                     source_identity='linear-slide-proof-v1', max_depth=9)
    assert not report['passed']
    assert report['interpolation'] == 'source_bounded_parametric_path'
    def far(u):
        return {'base': pose(), 'slide': pose(-4+8*u, 3)}
    clear = certify_parametric_path(parts, far, speed,
                                    source_identity='offset-linear-slide-proof-v1')
    assert clear['passed']
    assert clear['source'] == 'offset-linear-slide-proof-v1'
    with pytest.raises(ValueError, match='source/bound identity'):
        certify_parametric_path(parts, far, speed, source_identity='')
    with pytest.raises(ValueError, match='nonnegative'):
        certify_parametric_path(parts, far, lambda *_: -1,
                                 source_identity='invalid-speed')


def test_placed_step_loader_binds_actual_files_and_body_local_radius(tmp_path):
    root = tmp_path / 'native'
    (root / 'cad' / 'placed').mkdir(parents=True)
    records = []
    for body, name, y in [('base', 'block', 0), ('slide', 'block', 3)]:
        relative = f'cad/placed/{body}__{name}.step'
        solid = cq.Workplane('XY').box(1, 1, 1).translate((0, y, 0)).val()
        if body == 'base':
            other = cq.Workplane('XY').box(.2, .2, .2).translate((0, 10, 0)).val()
            solid = cq.Compound.makeCompound([solid, other])
        cq.exporters.export(solid, str(root / relative))
        records.append({'body': body, 'name': name, 'step_file': relative})
    (root / 'placed-native-parts.json').write_text(
        json.dumps({'units': 'mm', 'records': records}), encoding='utf-8')
    expected = {relative: hashlib.sha256((root / relative).read_bytes()).hexdigest()
                for relative in ['placed-native-parts.json'] + [r['step_file'] for r in records]}
    parts, imported = load_placed_native_parts(root, expected_file_hashes=expected)
    assert set(parts) == {'base__block', 'slide__block'}
    assert len(parts['base__block']['shape'].Solids()) == 2
    assert imported['externally_verified']
    assert imported['body_local_point_radius_mm']['slide__block'] > 3

    report = certify_native_parametric_path(root,
        lambda u: {'base': pose(), 'slide': pose(-4+8*u)},
        lambda name, *_: 8 if name == 'slide__block' else 0,
        source_identity='analytic-slide-proof-v1', expected_file_hashes=expected)
    assert report['passed']
    assert report['native_step_import']['native_file_set_sha256'] == imported['native_file_set_sha256']
    assert report['body_local_point_radius_mm']['slide__block'] > 3
    assert report['full_native_inventory_selected']
    assert report['full_native_noncontact_pair_coverage']

    bad = deepcopy(records)
    bad[0]['step_file'] = '../outside.step'
    (root / 'placed-native-parts.json').write_text(
        json.dumps({'units': 'mm', 'records': bad}), encoding='utf-8')
    with pytest.raises(ValueError, match='verified receipt'):
        load_placed_native_parts(root, expected_file_hashes=expected)
    with pytest.raises(ValueError, match='escapes the build'):
        load_placed_native_parts(root)


def test_body_compound_catches_contact_in_second_member_and_records_scope():
    cube = cq.Workplane('XY').box(1, 1, 1).val()
    parts = {'first': {'body': 'base', 'shape': cube},
             'second': {'body': 'base', 'shape': cube.translate((10, 0, 0))},
             'opposing': {'body': 'slider', 'shape': cube.translate((10, 0, 0))}}
    grouped, members = group_native_parts_by_body(parts)
    assert set(grouped) == {'base', 'slider'}
    assert len(grouped['base']['shape'].Solids()) == 2
    assert members['cross_body_member_pairs'] == 2
    assert [row['part'] for row in members['bodies']['base']] == ['first', 'second']
    assert len(members['member_manifest_sha256']) == 64
    report = certify_parametric_path(grouped,
        lambda _u: {'base': pose(), 'slider': pose()},
        lambda *_: 0, source_identity='compound-test|' + members['member_manifest_sha256'],
        max_depth=0)
    assert not report['passed']
    assert report['failures'][0]['parts'] == ['base', 'slider']


def test_implementation_change_invalidates_recorded_and_parametric_identity(monkeypatch):
    parts, frames = fixture(y=3)
    recorded = certify_recorded_path(parts, [0, 1], frames, interpolation='hold')
    parametric = certify_parametric_path(parts,
        lambda u: {'base': pose(), 'slide': pose(-4+8*u, 3)},
        lambda name, *_: 0 if name == 'fixed' else 8,
        source_identity='analytic-proof')
    assert recorded['passed'] and parametric['passed']
    assert set(recorded['implementation_sha256']) == {
        'native_pose_envelope.py', 'motion_envelope.py', 'core.py'}
    assert parametric['implementation_sha256'] == recorded['implementation_sha256']
    assert verify_implementation_identity(parametric)
    original = recorded['implementation_sha256']
    changed = dict(original, **{'core.py': 'f'*64})
    monkeypatch.setattr(envelope, '_implementation_identity', lambda: changed)
    with pytest.raises(ValueError, match='implementation changed'):
        verify_recorded_identity(recorded, parts, [0, 1], frames)
    with pytest.raises(ValueError, match='implementation changed'):
        verify_implementation_identity(parametric)
    later_recorded = certify_recorded_path(parts, [0, 1], frames, interpolation='hold')
    later_parametric = certify_parametric_path(parts,
        lambda u: {'base': pose(), 'slide': pose(-4+8*u, 3)},
        lambda name, *_: 0 if name == 'fixed' else 8,
        source_identity='analytic-proof')
    assert later_recorded['input_sha256'] != recorded['input_sha256']
    assert later_parametric['input_sha256'] != parametric['input_sha256']
