"""Actual Refined source selection and native property reconciliation.

These tests do not run inherited capture controls, whole-arm loads or dynamics.
"""
from copy import deepcopy
import json
from pathlib import Path
import runpy
import sys

import numpy as np
import pytest

cq = pytest.importorskip('cadquery')

from examples.engineering.arx_compact import refined
from examples.engineering.arx_compact.design import geometry_dag
from mechanism_lab.part_geometry import PartGeometryResolver


@pytest.fixture(scope='module')
def bundle():
    return refined.load_bundle()


def test_actual_source_selection_and_saved_pose_are_explicit(bundle):
    manifest, payloads = bundle
    design, pose, source, overrides, changes, masses = refined.source_inputs(manifest, payloads)
    assert source['commit'] == '85bd6d2ce5b7c567d08b3ffccf2a9be46005aabf'
    assert len(design['parts']) == 278
    assert len(overrides) == len(changes) == 123
    assert pose['source_frame_index'] == 300 and pose['time_s'] == 2.5
    assert len(pose['body_names']) == len(pose['body_poses']) == 10
    assert pose['body_poses'] == json.loads(payloads['capture.json'])['bodies'][300]
    assert masses == json.loads(payloads['mass-properties.json'])
    assert {r['part'] for r in overrides} == set(changes)
    # Retained failures/operation records remain source evidence, not hidden.
    assert any(change['rejected_attempts'] for change in changes.values())


@pytest.mark.parametrize('tamper', ['record_hash', 'missing_override', 'extra_brep'])
def test_mismatched_source_reports_are_not_accepted(bundle, tamper):
    manifest, payloads = deepcopy(bundle)
    if tamper == 'record_hash':
        report = json.loads(payloads['cad-refinement.json'])
        report['original_source_sha256'] = '0' * 64
        payloads['cad-refinement.json'] = json.dumps(report).encode()
    elif tamper == 'missing_override':
        report = json.loads(payloads['tessellation.json'])
        report['parts'].pop()
        payloads['tessellation.json'] = json.dumps(report).encode()
    else:
        payloads['cad/999.brep'] = b'unexpected'
    with pytest.raises(ValueError, match='byte identity|different original|disagree|exactly'):
        refined.source_inputs(manifest, payloads)


@pytest.mark.parametrize('key', ['2', '274'])
def test_actual_refined_native_properties_match_source_in_si(bundle, key):
    manifest, payloads = bundle
    design, _, source, overrides, changes, _ = refined.source_inputs(manifest, payloads)
    records = {str(row['id']): row for row in design['parts']}
    resolver = PartGeometryResolver(geometry_dag(design, source=source), records,
        overrides=overrides, asset_root=refined.DATA, source=source)
    properties = resolver.mass_properties(key, density_kg_m3=refined.DENSITIES[records[key]['material']])
    assert all(row['passed'] for row in refined.property_residuals(properties, changes[key]['refined']).values())
    assert properties['solid_count'] == 1
    assert properties['mass_kg'] < changes[key]['original']['mass_kg']
    assert np.asarray(properties['inertia_kg_m2']).shape == (3, 3)


@pytest.mark.parametrize('key', ['100', '132', '168'])
def test_original_source_clean_normalization_preserves_archived_tensor_contract(bundle, key):
    manifest, payloads = bundle
    design, _, source, _, changes, _ = refined.source_inputs(manifest, payloads)
    records = {str(row['id']): row for row in design['parts']}
    resolver = PartGeometryResolver(geometry_dag(design, source=source), records, source=source)
    measured = refined.original_properties(resolver, key, density_kg_m3=2700)
    assert refined.TOLERANCES['inertia_kg_m2'] == 1e-12
    assert all(row['passed'] for row in refined.property_residuals(measured, changes[key]['original']).values())


def test_refined_script_help_also_works_outside_package_context(monkeypatch, capsys):
    monkeypatch.setattr(sys, 'argv', [str(Path(refined.__file__)), '--help'])
    with pytest.raises(SystemExit) as result:
        runpy.run_path(str(Path(refined.__file__)), run_name='__main__')
    assert result.value.code == 0
    assert '--source-directory' in capsys.readouterr().out


@pytest.mark.parametrize('name', ['design', 'refined'])
def test_client_edit_after_import_fails_before_build(tmp_path, monkeypatch, name):
    from examples.engineering.arx_compact import design as compact
    module = compact if name == 'design' else refined
    key = 'examples/engineering/arx_compact/' + name + '.py'
    path = tmp_path / (name + '.py')
    path.write_bytes(module.CONSTRUCTION_CODE_PATHS[key].read_bytes())
    monkeypatch.setitem(module.CONSTRUCTION_CODE_PATHS, key, path)
    before = refined.require_current_construction()
    path.write_bytes(path.read_bytes() + b'\n# simulated concurrent edit\n')
    with pytest.raises(ValueError, match='Construction code changed'):
        refined.build(tmp_path / 'rejected')
    assert not (tmp_path / 'rejected').exists()
    assert before['examples/engineering/arx_compact/' + name + '.py'] != __import__('hashlib').sha256(path.read_bytes()).hexdigest()
