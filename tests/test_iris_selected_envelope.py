"""Lightweight source and scope contract for the selected IRIS CAD path."""
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np

from examples.analysis import iris09_selected_envelope as client
from examples.engineering.iris09 import mechanism
from mechanism_lab.engineering_source import load_project
from mechanism_lab.physical_model import digest


_FIXED = ('housing_material_3', 'housing_material_2', 'housing_material_5',
          'housing_material_1', *(f'screw_{i}' for i in range(6)),
          'lens_0', 'lens_1')


def _source_manifest():
    """Name inventory of the derived nine-leaf native construction source."""
    rows = [dict(body='fixed', name=name) for name in _FIXED]
    rows.extend(dict(body='rotor', name=name) for name in
                ('annular_rotor_and_drive_pins', 'knob_mean_radius_approximation',
                 *(f'input_pin_keeper_{i:02}' for i in range(9))))
    for index in range(9):
        body = f'leaf_{index:02}'
        rows.extend(dict(body=body, name=name) for name in
                    ('leaf_shaft_crank', 'output_pin_keeper',
                     'lower_shaft_collar', 'upper_shaft_collar'))
        rows.append(dict(body=f'rod_{index:02}', name='bored_connecting_rod'))
    return rows


def test_selected_pairs_are_exact_named_source_parts_and_account_for_omissions():
    records = _source_manifest()
    inventory = {f"{row['body']}__{row['name']}" for row in records}
    selected = {name for pair in client.PAIRS for name in pair}
    assert len(records) == len(inventory) == 68
    assert len(client.PAIRS) == 6 and len(selected) == 7
    assert selected <= inventory
    assert len({tuple(sorted(pair)) for pair in client.PAIRS}) == 6
    assert all(pair[0].split('__')[0] != pair[1].split('__')[0] for pair in client.PAIRS)
    counts = Counter(row['body'] for row in records)
    assert counts['fixed'] == 12 and counts['rotor'] == 11
    assert all(counts[f'leaf_{index:02}'] == 4 and counts[f'rod_{index:02}'] == 1
               for index in range(9))
    cross = sum(first*second for i, first in enumerate(counts.values())
                for second in list(counts.values())[i+1:])
    assert cross == 2103
    # When a prior generated manifest exists locally, make sure the synthetic
    # fast fixture still mirrors real source output; CI need not render CAD.
    previous = Path('outputs/engineering-iris-consolidated-20261023/native/placed-native-parts.json')
    if previous.is_file():
        actual = json.loads(previous.read_text(encoding='utf-8'))['records']
        assert {f"{row['body']}__{row['name']}" for row in actual} == inventory


def test_client_passes_exact_pose_speed_and_source_identity_to_shared_checker(tmp_path, monkeypatch):
    records = _source_manifest()
    native = tmp_path / 'native'
    native.mkdir()
    (native / 'placed-native-parts.json').write_text(
        json.dumps(dict(units='mm', records=records)), encoding='utf-8')
    project, source_files = load_project('iris09')
    path = project.planar_path(project.specification, source_files)
    initial = mechanism.layout(project.specification)['frames']
    receipt = dict(files_sha256={'placed-native-parts.json': 'receipt'})
    binding = dict(source='verified-native-source-binding')
    monkeypatch.setattr(client, 'load_project', lambda _name: (project, source_files))
    monkeypatch.setattr(client, 'verify_native_build',
                        lambda *_args, **_kwargs: receipt)
    monkeypatch.setattr(client, 'verify_native_source_binding',
                        lambda *_args, **_kwargs: binding)
    selected = {name for pair in client.PAIRS for name in pair}
    imported = dict(body_local_point_radius_mm={name:10. for name in selected},
                    native_file_set_sha256='selected-receipt',
                    complete_native_inventory_selected=False)

    def load(_native, *, expected_file_hashes, selected_names):
        assert expected_file_hashes == receipt['files_sha256']
        assert selected_names == selected
        names = {f"{row['body']}__{row['name']}" for row in records}
        assert selected_names <= names
        return ({name:dict(body=name.split('__')[0], shape=object())
                 for name in selected}, imported)
    monkeypatch.setattr(client, 'load_placed_native_parts', load)

    def check(parts, pose_at, point_speed_bound, *, source_identity,
              clearance_pairs, required_clearance_mm, distance_margin_mm,
              max_depth, max_pair_intervals):
        assert set(parts) == selected and clearance_pairs == client.PAIRS
        assert required_clearance_mm == 0. and distance_margin_mm == 1e-4
        assert path.source_sha256 in source_identity
        assert hashlib.sha256(Path(client.__file__).read_bytes()).hexdigest() in source_identity
        assert digest(receipt) in source_identity
        expected_bodies = {name.split('__')[0] for name in selected}
        poses = pose_at(0.)
        assert set(poses) == expected_bodies
        assert all(np.allclose(poses[name], initial[name], atol=1e-9)
                   for name in expected_bodies)
        assert point_speed_bound('rod_00__bored_connecting_rod', 0., 1.) == path.point_speed_bound(
            'rod_00', 10., 0., 1.)
        assert point_speed_bound('fixed__housing_material_2', 0., 1.) == 0.
        return dict(schema='cybrgeo-native-pose-envelope/1', passed=True,
                    clearance_pairs=[dict(parts=list(pair)) for pair in clearance_pairs],
                    failures=[])
    monkeypatch.setattr(client, 'certify_parametric_path', check)
    report = client.certify(native)
    assert report['passed'] and report['full_native_cross_body_pair_count'] == 2103
    assert report['selected_full_assembly_pair_count'] == 6
    assert report['unexamined_full_assembly_pair_count'] == 2097
    assert report['omitted_native_part_count'] == 61
    assert report['native_source_binding_sha256'] == digest(binding)
    assert report['project_source_files_sha256'] == digest(source_files)
    assert report['native_step_import'] == imported
