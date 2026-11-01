"""Construction intent is source-bound design work, never hardware evidence."""
from copy import deepcopy
import sys
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest

from mechanism_lab.native_construction import NativeConstruction
from mechanism_lab.qualification import (
    QualificationError, QualificationIntentPlan, QualificationPlan,
    _require_connector_demand_binding,
)


def _intent():
    return dict(
        schema='cybrgeo-qualification-intent/1',
        parts={name: dict(material='alloy', process=None,
                          nominal_minimum_wall_mm=2.,
                          required_dimensional_tolerance_mm=.1,
                          maximum_volume_mm3=500.)
               for name in ('fixed__a', 'moving__b')},
        interfaces={'pin_fit': dict(parts=['fixed__a', 'moving__b'],
                                    role='clearance', required_service_gap_mm=.2,
                                    first_variation_mm=.1, second_variation_mm=.1,
                                    force_demand_N=5.)},
        loads={'output': dict(part='moving__b', source_demand_limit_Nm=2.)},
        unresolved_requirements=[dict(category='external', kind='material_lot',
                                      subject='alloy', reason='No purchased lot or material report')],
    )


def _strict_plan():
    """Test-only resolved numbers; these are not evidence of a real alloy or mill."""
    return dict(
        schema='cybrgeo-qualification-plan/1',
        materials={'alloy': dict(grade='TEST-ALLOY', lot='TEST-LOT',
                                 density_kg_m3=2700., allowable_stress_MPa=100.,
                                 temperature_limit_C=80.)},
        processes={'mill': dict(method='TEST-MILL', facility='TEST-FACILITY',
                                revision='TEST-1', allowed_materials=['alloy'],
                                minimum_feature_mm=1., capability_tolerance_mm=.1,
                                maximum_part_volume_mm3=1000.)},
        parts={name: dict(material='alloy', process='mill',
                          nominal_minimum_wall_mm=2.,
                          required_dimensional_tolerance_mm=.1,
                          maximum_volume_mm3=500.)
               for name in ('fixed__a', 'moving__b')},
        interfaces={'pin_fit': dict(parts=['fixed__a', 'moving__b'],
                                    nominal_clearance_mm=.5,
                                    first_tolerance_mm=.1,
                                    second_tolerance_mm=.1,
                                    required_clearance_mm=.2,
                                    demand_N=5., design_capacity_N=10.)},
        loads={'output': dict(part='moving__b', demand_Nm=1.,
                              design_capacity_Nm=2., service_temperature_C=50.)},
        assembly_requirements=dict(minimum_endurance_cycles=1000,
                                   minimum_thermal_test_C=60.,
                                   maximum_measured_backlash_mm=.3),
    )


class _SyntheticShape:
    def __init__(self, volume):
        self.volume = volume

    def isValid(self):
        return True

    def Solids(self):
        return [self]

    def Volume(self):
        return self.volume


def test_intent_accepts_unknown_external_inputs_without_claiming_capability():
    plan = QualificationIntentPlan(_intent())
    assert plan.document['parts']['fixed__a']['process'] is None
    assert plan.document['unresolved_requirements'][0]['category'] == 'external'
    assert len(plan.sha256) == 64


@pytest.mark.parametrize('change,match', [
    (lambda doc: doc['parts']['fixed__a'].update(maximum_volume_mm3=0),
     'maximum_volume_mm3'),
    (lambda doc: doc['parts']['fixed__a'].update(material=''),
     'Intent material'),
    (lambda doc: doc['interfaces']['pin_fit'].update(parts=['fixed__a', 'lost__part']),
     'two placed part IDs'),
    (lambda doc: doc['loads']['output'].update(part='lost__part'),
     'placed loaded part'),
    (lambda doc: doc['unresolved_requirements'][0].update(category='qualified'),
     'category must be design or external'),
    (lambda doc: doc.update(unresolved_requirements=[]),
     'must name unresolved'),
])
def test_intent_rejects_invalid_inventory_or_fake_resolution(change, match):
    document = _intent()
    change(document)
    with pytest.raises(QualificationError, match=match):
        QualificationIntentPlan(document)


def test_placement_preflight_rejects_unknown_or_oversize_cad_before_export(tmp_path,
                                                                            monkeypatch):
    # Rejection precedes any CadQuery operation, so the lightweight test only
    # supplies its import name and never constructs or exports a CAD shape.
    monkeypatch.setitem(sys.modules, 'cadquery', SimpleNamespace())
    builder = NativeConstruction(tmp_path, {'alloy': 2700.},
                                 qualification_intent=QualificationIntentPlan(_intent()))
    with pytest.raises(QualificationError, match='source-bound intent'):
        builder.place('fixed', 'unexpected', _SyntheticShape(1.), 'alloy')
    with pytest.raises(QualificationError, match='source-bound intent'):
        builder.place('fixed', 'a', _SyntheticShape(1.), 'steel')
    with pytest.raises(QualificationError, match='authored volume limit'):
        builder.place('fixed', 'a', _SyntheticShape(501.), 'alloy')
    assert builder.placed_records == []
    assert list((tmp_path / 'cad' / 'placed').iterdir()) == []


def test_incomplete_intent_inventory_cannot_finish_as_native_build(tmp_path):
    builder = NativeConstruction(tmp_path, {'alloy': 2700.},
                                 qualification_intent=_intent())
    with pytest.raises(ValueError, match='placed inventory differs'):
        builder.finish('test-only')
    assert not (tmp_path / 'native-build.json').exists()
    assert not (tmp_path / 'qualification-intent-construction.json').exists()
    assert not (tmp_path / 'qualification-construction.json').exists()


def test_plain_diagnostic_builder_cannot_write_intent_receipt(tmp_path):
    builder = NativeConstruction(tmp_path, {'alloy': 2700.})
    with pytest.raises(ValueError, match='reserved for the planned native builder'):
        builder.write_json('qualification-intent-construction.json', {'forged': True})
    with pytest.raises(ValueError, match='requires a source-bound construction intent'):
        builder.finish('test-only', {'qualification-intent-construction.json': {'forged': True}})
    assert not (tmp_path / 'qualification-intent-construction.json').exists()
    assert not (tmp_path / 'native-build.json').exists()


@pytest.mark.parametrize('change,match', [
    (lambda plan: plan['parts']['fixed__a'].update(nominal_minimum_wall_mm=1.),
     'weakens intent part targets'),
    (lambda plan: plan['interfaces']['pin_fit'].update(required_clearance_mm=.1),
     'weakens intent service gap'),
    (lambda plan: plan['interfaces']['pin_fit'].update(parts=['moving__b', 'fixed__a']),
     'changes intent interface pair'),
    (lambda plan: plan['loads']['output'].update(part='fixed__a'),
     'changes intent loaded part'),
    (lambda plan: plan['loads']['output'].update(demand_Nm=3., design_capacity_Nm=3.),
     'more than source actuator limit'),
])
def test_later_strict_plan_cannot_weaken_or_relabel_authored_intent(change, match):
    intent, resolved = QualificationIntentPlan(_intent()), _strict_plan()
    assert intent.require_resolved_by(resolved) == resolved
    changed = deepcopy(resolved)
    change(changed)
    with pytest.raises(QualificationError, match=match):
        intent.require_resolved_by(changed)


def _site_connector_fixture():
    plan = _strict_plan()
    plan['solver_connectors'] = {'bearing_joint': 'pin_fit'}
    root = ET.Element('mujoco')
    world = ET.SubElement(root, 'worldbody')
    fixed = ET.SubElement(world, 'body', name='fixed')
    moving = ET.SubElement(fixed, 'body', name='moving')
    ET.SubElement(fixed, 'site', name='fixed_anchor')
    ET.SubElement(moving, 'site', name='moving_anchor')
    connector = ET.SubElement(ET.SubElement(root, 'equality'), 'connect',
                              name='bearing_joint', site1='fixed_anchor',
                              site2='moving_anchor')
    simulation = dict(steps=2, metrics={'constraints': {'bearing_joint': dict(
        type='connect', active_steps=2, missing_row_steps=0,
        incomplete_reaction_row_steps=0,
        reaction_vector_norms={'anchor_force': dict(samples=2,
            peak_absolute=dict(value=4.))})}})
    return QualificationPlan(plan), SimpleNamespace(root=root), simulation, connector, fixed, moving


def test_site_endpoint_connector_uses_xml_body_ancestry_and_exact_planned_pair():
    plan, model, simulation, _, _, _ = _site_connector_fixture()
    modeled = {}
    _require_connector_demand_binding(plan, model, simulation, modeled)
    assert modeled['pin_fit'] == 4.


@pytest.mark.parametrize('change,match', [
    (lambda connector, fixed, moving: connector.set('site2', 'missing_anchor'),
     'ambiguous or unknown site endpoints'),
    (lambda connector, fixed, moving: ET.SubElement(moving, 'site', name='fixed_anchor'),
     'unique source names'),
    (lambda connector, fixed, moving: (
        ET.SubElement(fixed, 'site', name='same_body_anchor'),
        connector.set('site2', 'same_body_anchor')),
     'distinct named body endpoints'),
    (lambda connector, fixed, moving: connector.set('body1', 'fixed'),
     'ambiguous or unknown site endpoints'),
])
def test_site_endpoint_connector_rejects_unowned_or_ambiguous_mapping(change, match):
    plan, model, simulation, connector, fixed, moving = _site_connector_fixture()
    change(connector, fixed, moving)
    with pytest.raises(QualificationError, match=match):
        _require_connector_demand_binding(plan, model, simulation)
