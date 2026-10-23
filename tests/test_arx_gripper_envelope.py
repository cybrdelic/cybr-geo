"""ARX client consumes one source slide contract and the common CAD certifier."""
from pathlib import Path
import hashlib
import json

import pytest

from examples.analysis.arx_belt_gripper_envelope import (
    SELECTED_LOGICAL_PARTS, gripper_path, selected_native_parts,
    source_slide_contract)
from mechanism_lab.native_pose_envelope import load_placed_native_parts


SOURCE = Path(__file__).resolve().parents[1] / 'examples' / 'engineering' / 'arx_belt' / 'data' / 'P2-baseline.xml'
INTENT = SOURCE.with_name('qualification-intent.json')


def _source_selection(tmp_path):
    parts = json.loads(INTENT.read_text(encoding='utf-8'))['parts']
    rows = [dict(body=name.split('__', 1)[0], name=name.split('__', 1)[1])
            for name in parts]
    manifest = tmp_path / 'placed-native-parts.json'
    manifest.write_text(json.dumps(dict(units='mm', records=rows)), encoding='utf-8')
    hashes = {manifest.name: hashlib.sha256(manifest.read_bytes()).hexdigest()}
    return selected_native_parts(tmp_path, hashes), manifest, rows


def test_source_bound_opposed_slide_poses_cover_entire_authored_range(tmp_path):
    contract = source_slide_contract(SOURCE)
    selected, _, _ = _source_selection(tmp_path)
    pose_at, bound = gripper_path(contract, selected)
    assert len(SELECTED_LOGICAL_PARTS) == len(selected) == 6
    for parameter, expected in ((0., 10.), (.5, 25.), (1., 40.)):
        poses = pose_at(parameter)
        assert set(poses) == {'tool', 'jaw', 'jaw_opposed'}
        assert poses['jaw'][0, 3] == pytest.approx(expected)
        assert poses['jaw_opposed'][0, 3] == pytest.approx(-expected)
        assert poses['jaw'][1, 3] == poses['jaw_opposed'][1, 3] == pytest.approx(219.2)
    assert bound('jaw__finger_pad_1', 0., 1.) == pytest.approx(30.)
    assert bound('jaw_opposed__left_jaw_carriage', 0., 1.) == pytest.approx(30.)
    assert bound('tool__gripper_motor', 0., 1.) == 0.


def test_changed_gripper_range_or_synchronization_rejects_stale_path(tmp_path):
    text = SOURCE.read_text(encoding='utf-8')
    changed = tmp_path / 'changed.xml'
    changed.write_text(text.replace('range=".010 .040"', 'range=".010 .080"'), encoding='utf-8')
    with pytest.raises(ValueError, match='source changed'):
        source_slide_contract(changed)
    changed.write_text(text.replace('polycoef="0 1 0 0 0"', 'polycoef="0 .5 0 0 0"'), encoding='utf-8')
    with pytest.raises(ValueError, match='synchronized opposed rack'):
        source_slide_contract(changed)


def test_non_source_part_or_parameter_cannot_receive_an_implicit_pose(tmp_path):
    selected, _, _ = _source_selection(tmp_path)
    pose_at, bound = gripper_path(source_slide_contract(SOURCE), selected)
    with pytest.raises(ValueError, match='source travel interval'):
        pose_at(1.01)
    with pytest.raises(ValueError, match='selected native part'):
        bound('unknown__part', 0., 1.)


def test_native_selection_survives_instance_ordinal_drift_but_not_ambiguity(tmp_path):
    selected, manifest, rows = _source_selection(tmp_path)
    mount = [row for row in rows if row['body'] == 'tool'
             and row['name'].startswith('gripper_cable_clamp_mount_')]
    assert len(mount) == 1
    original = mount[0]['name']
    assert f'tool__{original}' in selected
    mount[0]['name'] = 'gripper_cable_clamp_mount_73'
    manifest.write_text(json.dumps(dict(units='mm', records=rows)), encoding='utf-8')
    hashes = {manifest.name: hashlib.sha256(manifest.read_bytes()).hexdigest()}
    changed = selected_native_parts(tmp_path, hashes)
    assert len(changed) == 6
    assert 'tool__gripper_cable_clamp_mount_73' in changed
    assert f'tool__{original}' not in changed
    rows.append(dict(body='tool', name='gripper_cable_clamp_mount_74'))
    manifest.write_text(json.dumps(dict(units='mm', records=rows)), encoding='utf-8')
    hashes[manifest.name] = hashlib.sha256(manifest.read_bytes()).hexdigest()
    with pytest.raises(ValueError, match='missing or ambiguous'):
        selected_native_parts(tmp_path, hashes)
    with pytest.raises(ValueError, match='verified receipt'):
        selected_native_parts(tmp_path, {manifest.name: '0'*64})


def test_selected_native_import_keeps_full_inventory_and_receipt_binding(tmp_path):
    import cadquery as cq

    placed = tmp_path / 'cad' / 'placed'
    placed.mkdir(parents=True)
    rows, hashes = [], {}
    for body in ('jaw', 'jaw_opposed', 'tool'):
        relative = f'cad/placed/{body}__sample.step'
        path = tmp_path / relative
        cq.exporters.export(cq.Workplane('XY').box(1, 1, 1).val(), str(path))
        rows.append(dict(body=body, name='sample', step_file=relative))
        hashes[relative] = hashlib.sha256(path.read_bytes()).hexdigest()
    manifest = tmp_path / 'placed-native-parts.json'
    manifest.write_text(json.dumps(dict(units='mm', records=rows)), encoding='utf-8')
    hashes[manifest.name] = hashlib.sha256(manifest.read_bytes()).hexdigest()
    parts, receipt = load_placed_native_parts(tmp_path, expected_file_hashes=hashes,
        selected_names=('jaw__sample', 'tool__sample'))
    assert set(parts) == {'jaw__sample', 'tool__sample'}
    assert receipt['native_inventory_count'] == 3
    assert receipt['omitted_native_part_names'] == ['jaw_opposed__sample']
    assert not receipt['complete_native_inventory_selected']
    from mechanism_lab.native_pose_envelope import certify_native_parametric_path
    import numpy as np

    def selected_poses(_u):
        jaw, tool = np.eye(4), np.eye(4)
        jaw[0, 3] = 3.
        return {'jaw': jaw, 'tool': tool}

    scoped = certify_native_parametric_path(tmp_path, selected_poses,
        lambda *_: 0., source_identity='selected-only-test',
        expected_file_hashes=hashes,
        selected_names=('jaw__sample', 'tool__sample'))
    assert scoped['passed'] and scoped['complete_pair_accounting']
    assert not scoped['full_native_inventory_selected']
    assert not scoped['full_native_noncontact_pair_coverage']
    (tmp_path / rows[0]['step_file']).write_text('tampered', encoding='utf-8')
    with pytest.raises(ValueError, match='verified receipt'):
        load_placed_native_parts(tmp_path, expected_file_hashes=hashes,
                                 selected_names=('jaw__sample', 'tool__sample'))
