"""ARX source intent may select strict CAD only through a complete plan."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from mechanism_lab.qualification import (QualificationError, QualificationIntentPlan,
    QualificationPlan, _qualified_body_poses)


SOURCE = Path(__file__).resolve().parents[1] / 'examples/engineering/arx_belt'
SPEC = json.loads((SOURCE / 'data/spec.json').read_text(encoding='utf-8'))
RHO = {'aluminum': 2700., 'steel': 7850., 'polymer': 1150., 'rubber': 1100.}


def _client(monkeypatch):
    # Route and pose tests never execute CadQuery or its heavyweight renderer.
    monkeypatch.setitem(sys.modules, 'cadquery', SimpleNamespace())
    spec = importlib.util.spec_from_file_location(
        'examples.engineering.arx_belt._qualified_route_fixture', SOURCE / 'design.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _source_root(tmp_path):
    root = tmp_path / 'arx'
    data = root / 'data'
    data.mkdir(parents=True)
    for name in ('qualification-intent.json', 'P2-baseline-parameters.json'):
        (data / name).write_bytes((SOURCE / 'data' / name).read_bytes())
    return root


def _test_only_plan(intent):
    """Syntactic fixture limits, never supplier or measured process data."""
    parts = {name: dict(material=part['material'], process='test_process',
                        nominal_minimum_wall_mm=.01,
                        required_dimensional_tolerance_mm=.001,
                        maximum_volume_mm3=1e9)
             for name, part in intent.document['parts'].items()}
    interfaces = {name: dict(parts=list(row['parts']), role='tested_retention_contact',
                             first_tolerance_mm=.001, second_tolerance_mm=.001,
                             demand_N=row['force_demand_N'] or 1.,
                             design_capacity_N=2*(row['force_demand_N'] or 1.),
                             nominal_maximum_distance_mm=0., maximum_service_play_mm=1.)
                  for name, row in intent.document['interfaces'].items()}
    loads = {name: dict(part=row['part'], demand_Nm=.01,
                        design_capacity_Nm=.02, service_temperature_C=50.)
             for name, row in intent.document['loads'].items()}
    return QualificationPlan(dict(schema='cybrgeo-qualification-plan/1', parts=parts,
        materials={name: dict(grade='TEST-ONLY', lot='TEST-ONLY', density_kg_m3=density,
                              allowable_stress_MPa=100., temperature_limit_C=100.)
                   for name, density in RHO.items()},
        processes={'test_process': dict(method='TEST-ONLY', facility='TEST-ONLY',
            revision='TEST-ONLY', allowed_materials=list(RHO),
            minimum_feature_mm=.001, capability_tolerance_mm=.001,
            maximum_part_volume_mm3=1e9)}, interfaces=interfaces, loads=loads,
        assembly_requirements=dict(minimum_endurance_cycles=2,
            minimum_thermal_test_C=60., maximum_measured_backlash_mm=1.)))


def _hypothetical_design_resolved_intent():
    """Test the lowerer only; this is not the actual unresolved ARX source."""
    authored = json.loads((SOURCE / 'data/qualification-intent.json').read_text(encoding='utf-8'))
    future = deepcopy(authored)
    future['unresolved_requirements'] = [item for item in authored['unresolved_requirements']
                                         if item['category'] != 'design']
    return QualificationIntentPlan(future)


class _Shape:
    def isValid(self):
        return True

    def Solids(self):
        return [self]

    def Volume(self):
        return 1.


def test_incomplete_plan_rejected_before_any_cad_output(tmp_path, monkeypatch):
    client = _client(monkeypatch)
    root = _source_root(tmp_path)
    (root / 'data/qualification-plan.json').write_text(
        '{"schema":"cybrgeo-qualification-plan/1"}', encoding='utf-8')
    monkeypatch.setattr(client, 'HERE', root)
    monkeypatch.setattr(client, '_PlacedRows', lambda *args, **kwargs: (_ for _ in ()).throw(
        AssertionError('The CAD collector must not run on an incomplete plan')))
    output = tmp_path / 'native'
    with pytest.raises(QualificationError, match='construction contract'):
        client.build(SPEC, output)
    assert not output.exists()


def test_source_home_poses_cover_all_ten_physical_bodies_including_pinion(monkeypatch):
    pytest.importorskip('mujoco')
    client = _client(monkeypatch)
    poses = client._source_home_body_poses_mm()
    intent = QualificationIntentPlan(json.loads(
        (SOURCE / 'data/qualification-intent.json').read_text(encoding='utf-8')))
    assert _qualified_body_poses(poses, intent.document['parts']) == poses
    assert set(poses) == set(client.PHYSICAL_BODY_MAPPING)
    tool = np.asarray(poses['tool'])
    pinion = np.asarray(poses['pinion'])
    np.testing.assert_allclose(pinion[:3, :3], tool[:3, :3], atol=1e-12)
    np.testing.assert_allclose(pinion[:3, 3] - tool[:3, 3],
        tool[:3, :3] @ [0., client.PINION_CENTER_Y_MM, 0.], atol=1e-10)


def test_exact_intent_rows_route_directly_to_strict_cad_without_mass_overrides(
        tmp_path, monkeypatch):
    client = _client(monkeypatch)
    intent = _hypothetical_design_resolved_intent()
    plan = _test_only_plan(intent)
    # This directly checks a hypothetical future resolved-design lowering;
    # public build still reads and rejects today's actual source design gaps.
    shape = _Shape()
    rows = [(name.split('__', 1)[0], name.split('__', 1)[1], shape, part['material'])
            for name, part in intent.document['parts'].items()]
    calls = {'placements': []}

    class StrictSpy:
        def __init__(self, output, densities, **kwargs):
            calls.update(output=output, densities=densities, kwargs=kwargs)

        def place(self, *args, **kwargs):
            calls['placements'].append((args, kwargs))

        def finish(self, *args, **kwargs):
            calls['finish'] = (args, kwargs)
            return 'strict-route-sentinel'

    monkeypatch.setattr(client, 'QualifiedNativeConstruction', StrictSpy)
    result = client._finish_qualified(tmp_path / 'native', RHO, intent, plan, rows,
        {'tool': [dict(name='engineering_mass_reserve', provenance='unmeasured')]},
        [dict(body='tool', name='motor_envelope', source_mass_kg=.012)],
        {'source_commit': client.SOURCE_COMMIT}, {})
    assert result == 'strict-route-sentinel'
    assert len(calls['placements']) == len(intent.document['parts']) == 380
    assert set(f'{args[0]}__{args[1]}' for args, _ in calls['placements']) == set(plan.document['parts'])
    assert all('mass' not in kwargs for _, kwargs in calls['placements'])
    assert calls['kwargs']['physical_body_mapping'] == client.PHYSICAL_BODY_MAPPING
    assert set(calls['kwargs']['initial_body_poses_mm']) == set(client.PHYSICAL_BODY_MAPPING)
    assert 'native_requirements' not in calls['finish'][1]
    basis = calls['finish'][0][1]['arx-strict-mass-basis.json']
    assert basis['excluded_source_mass_overrides'][0]['source_mass_kg'] == .012
    assert basis['excluded_unplaced_mass_allocations'][0]['name'] == 'engineering_mass_reserve'


def test_strict_route_rejects_missing_pinion_inventory_before_export(tmp_path, monkeypatch):
    client = _client(monkeypatch)
    intent = _hypothetical_design_resolved_intent()
    plan = _test_only_plan(intent)
    rows = [(name.split('__', 1)[0], name.split('__', 1)[1], _Shape(), part['material'])
            for name, part in intent.document['parts'].items() if not name.startswith('pinion__')]
    monkeypatch.setattr(client, 'QualifiedNativeConstruction', lambda *args, **kwargs: (_ for _ in ()).throw(
        AssertionError('Strict export must not begin without every source part')))
    with pytest.raises(QualificationError, match='strict placements differ'):
        client._finish_qualified(tmp_path / 'native', RHO, intent, plan, rows, {}, [], {}, {})
    assert not (tmp_path / 'native').exists()


def test_complete_test_only_plan_cannot_erase_unresolved_arx_design(tmp_path, monkeypatch):
    client = _client(monkeypatch)
    root = _source_root(tmp_path)
    intent = QualificationIntentPlan(json.loads(
        (SOURCE / 'data/qualification-intent.json').read_text(encoding='utf-8')))
    (root / 'data/qualification-plan.json').write_text(
        json.dumps(_test_only_plan(intent).document), encoding='utf-8')
    monkeypatch.setattr(client, 'HERE', root)
    monkeypatch.setattr(client, '_PlacedRows', lambda *args, **kwargs: (_ for _ in ()).throw(
        AssertionError('No CAD collection while source design gaps remain')))
    output = tmp_path / 'native'
    with pytest.raises(QualificationError, match='unresolved design requirements'):
        client.build(SPEC, output)
    assert not output.exists()


def test_source_intent_cannot_delete_unverified_power_loss_blocker(tmp_path, monkeypatch):
    client = _client(monkeypatch)
    root = _source_root(tmp_path)
    path = root / 'data/qualification-intent.json'
    document = json.loads(path.read_text(encoding='utf-8'))
    document['unresolved_requirements'] = [row for row in document['unresolved_requirements']
        if row['kind'] != 'power_loss_payload_retention']
    path.write_text(json.dumps(document), encoding='utf-8')
    monkeypatch.setattr(client, 'HERE', root)
    with pytest.raises(QualificationError, match='power-loss payload retention blockers'):
        client.qualification_intent(SPEC)
