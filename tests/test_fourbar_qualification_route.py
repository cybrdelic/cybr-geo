"""A source-provided strict plan changes construction, never its evidence claims."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from mechanism_lab.qualification import QualificationError, QualificationIntentPlan


SOURCE = Path(__file__).resolve().parents[1] / 'examples/engineering/fourbar_lift'


def _client(monkeypatch):
    # These routing tests make no CAD or STEP objects. Isolate the module with
    # a test-only CadQuery import token instead of importing a graphics kernel.
    monkeypatch.setitem(sys.modules, 'cadquery', SimpleNamespace())
    source = importlib.util.spec_from_file_location('fourbar_qualification_route_fixture',
                                                     SOURCE / 'mechanism.py')
    module = importlib.util.module_from_spec(source)
    source.loader.exec_module(module)
    return module


def _test_only_resolved_plan(intent, densities):
    """Syntactic fixture numbers; this is not a supplier or fabricated unit."""
    parts = {}
    for name, part in intent['parts'].items():
        parts[name] = dict(material=part['material'], process='test_process',
            nominal_minimum_wall_mm=max(0.01, part['nominal_minimum_wall_mm'] or 0.),
            required_dimensional_tolerance_mm=min(0.001,
                part['required_dimensional_tolerance_mm'] or 1.),
            maximum_volume_mm3=min(1e9, part['maximum_volume_mm3'] or 1e9))
    interfaces = {}
    for name, row in intent['interfaces'].items():
        common = dict(parts=list(row['parts']), first_tolerance_mm=max(.001, row['first_variation_mm'] or 0.),
                      second_tolerance_mm=max(.001, row['second_variation_mm'] or 0.),
                      demand_N=max(1., row['force_demand_N'] or 0.), design_capacity_N=2.)
        if row['role'] == 'retained_contact':
            interfaces[name] = dict(common, role='tested_retention_contact',
                nominal_maximum_distance_mm=1., maximum_service_play_mm=2.)
        else:
            interfaces[name] = dict(common, nominal_clearance_mm=1.,
                required_clearance_mm=max(0., row['required_service_gap_mm'] or 0.))
    return dict(schema='cybrgeo-qualification-plan/1', parts=parts,
        materials={name: dict(grade='TEST-ONLY', lot='TEST-ONLY', density_kg_m3=density,
                              allowable_stress_MPa=100., temperature_limit_C=100.)
                   for name, density in densities.items()},
        processes={'test_process': dict(method='TEST-ONLY', facility='TEST-ONLY',
            revision='TEST-ONLY', allowed_materials=list(densities),
            minimum_feature_mm=.001, capability_tolerance_mm=.001,
            maximum_part_volume_mm3=1e9)},
        interfaces=interfaces,
        loads={name: dict(part=row['part'], demand_Nm=.1, design_capacity_Nm=.2,
                          service_temperature_C=50.)
               for name, row in intent['loads'].items()},
        assembly_requirements=dict(minimum_endurance_cycles=2,
                                   minimum_thermal_test_C=60.,
                                   maximum_measured_backlash_mm=1.))


class _Shape:
    def isValid(self):
        return True

    def Solids(self):
        return [self]

    def Volume(self):
        return 1.


def _source_root(tmp_path):
    data = tmp_path / 'data'
    data.mkdir()
    (data / 'qualification-intent.json').write_bytes(
        (SOURCE / 'data/qualification-intent.json').read_bytes())
    return tmp_path


def test_source_plan_selects_posed_strict_constructor_without_diagnostic_templates(tmp_path,
                                                                                    monkeypatch):
    client = _client(monkeypatch)
    spec = client.specification()
    original_intent = QualificationIntentPlan(json.loads(
        (SOURCE / 'data/qualification-intent.json').read_text()))
    plan = _test_only_resolved_plan(original_intent.document, client.DENSITIES)
    local = _source_root(tmp_path)
    (local / 'data/qualification-plan.json').write_text(json.dumps(plan))
    monkeypatch.setattr(client, 'HERE', local)
    expected = client.layout(spec)
    shape = _Shape()
    monkeypatch.setattr(client, 'native_rows', lambda _spec: (
        [('fixed', 'backplate_and_bearing_supports', shape, 'aluminum')], expected))

    calls = {}
    class StrictSpy:
        def __init__(self, output, densities, **kwargs):
            calls.update(output=output, densities=densities, kwargs=kwargs,
                         placements=[], addparts=0)

        def addpart(self, *args, **kwargs):
            calls['addparts'] += 1
            raise AssertionError('Strict route must not emit diagnostic templates')

        def place(self, body, name, solid, material):
            calls['placements'].append((body, name, material))

        def finish(self, *args, **kwargs):
            calls['finish'] = kwargs
            return 'test-only strict build sentinel'

    monkeypatch.setattr(client, 'QualifiedNativeConstruction', StrictSpy)
    assert client.build(spec, tmp_path / 'strict-output') == 'test-only strict build sentinel'
    assert calls['addparts'] == 0
    assert calls['placements'] == [('fixed', 'backplate_and_bearing_supports', 'aluminum')]
    assert calls['finish']['native_requirements'] is None
    assert calls['kwargs']['qualification_plan'].document == plan
    assert calls['kwargs']['physical_body_mapping'] == {body: body for body in expected['frames']}
    for body, matrix in expected['frames'].items():
        assert np.array_equal(calls['kwargs']['initial_body_poses_mm'][body], matrix)


def test_incomplete_source_plan_fails_before_any_native_export(tmp_path, monkeypatch):
    client = _client(monkeypatch)
    spec = client.specification()
    local = _source_root(tmp_path)
    (local / 'data/qualification-plan.json').write_text('{"schema":"incomplete"}')
    monkeypatch.setattr(client, 'HERE', local)
    monkeypatch.setattr(client, 'native_rows', lambda _: (_ for _ in ()).throw(
        AssertionError('CAD construction must not begin with an incomplete plan')))
    with pytest.raises(QualificationError, match='construction contract'):
        client.build(spec, tmp_path / 'strict-output')
    assert not (tmp_path / 'strict-output').exists()
