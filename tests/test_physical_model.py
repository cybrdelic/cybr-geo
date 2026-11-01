"""Geometry-to-solver contracts, independent of a particular robot."""
from copy import deepcopy
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from mechanism_lab.physical_model import PhysicalModel, digest, inertia_tensor


XML = '<mujoco><worldbody><body name="link"><joint name="hinge"/></body><body name="payload"><freejoint/></body></worldbody></mujoco>'


def record():
    return dict(mass_kg=2., com_m=[.01, .02, .03],
                inertia_tensor_kg_m2=[[.1, .01, 0], [.01, .12, 0], [0, 0, .15]])


def test_full_cad_tensor_survives_physical_source_and_mjcf(tmp_path):
    model = PhysicalModel.from_xml(XML)
    model.bind_inertias({'native': record()}, {'link': 'native'})
    model.set_joint('hinge', range=[-.5, .8], armature=.01)
    model.add_proxy(dict(name='bore_region', body='link', type='box',
        center_mm=[10, 20, 30], half_size_mm=[2, 3, 4]))
    model.require_free_body('payload')
    restored = PhysicalModel.from_source(model.to_source())
    assert restored.to_source() == model.to_source()
    assert restored.write(tmp_path) == digest(model.to_source())
    xml = ET.parse(tmp_path/'model.xml')
    inertial = xml.find('.//body[@name="link"]/inertial')
    assert np.fromstring(inertial.get('fullinertia'), sep=' ') == pytest.approx([.1, .12, .15, .01, 0, 0])
    proxy = xml.find('.//geom')
    assert np.fromstring(proxy.get('pos'), sep=' ') == pytest.approx([.01, .02, .03])
    assert np.fromstring(proxy.get('size'), sep=' ') == pytest.approx([.002, .003, .004])


@pytest.mark.parametrize('tensor', [np.diag([1, 1, 3]), np.diag([0, 1, 1]),
    [[1, .2, 0], [0, 1, 0], [0, 0, 1]], np.diag([1, 1, float('nan')])])
def test_nonphysical_cad_inertia_rejected(tensor):
    row = record()
    row['inertia_tensor_kg_m2'] = np.asarray(tensor).tolist()
    with pytest.raises(ValueError):
        inertia_tensor(row)


def test_unbound_body_and_fabricated_payload_attachment_rejected():
    model = PhysicalModel.from_xml(XML)
    with pytest.raises(ValueError, match='binding'):
        model.bind_inertias({'native': record()}, {'missing': 'native'})
    ET.SubElement(ET.SubElement(model.root, 'equality'), 'weld', body1='payload', body2='link')
    with pytest.raises(ValueError, match='welded'):
        model.require_free_body('payload')


def test_cad_mass_provenance_cannot_disagree_with_solver_inertia():
    model = PhysicalModel.from_xml(XML)
    model.bind_inertias({'native': record()}, {'link': 'native'})
    source = model.to_source()
    source['mass_records']['link']['mass_kg'] = 3.
    with pytest.raises(ValueError, match='differs from physical inertia'):
        PhysicalModel.from_source(source)


@pytest.mark.parametrize('xml', [
    '<mujoco><include file="elsewhere.xml"/></mujoco>',
    '<mujoco><asset><mesh file="elsewhere.stl"/></asset></mujoco>',
    '<mujoco><worldbody><body name="same"/><body name="same"/></worldbody></mujoco>',
])
def test_source_cannot_silently_gain_external_geometry_or_ambiguous_bodies(xml):
    with pytest.raises(ValueError):
        PhysicalModel.from_xml(xml)


def test_contact_definition_validation_and_removal():
    model = PhysicalModel.from_xml(XML)
    row = dict(name='pad', body='link', type='box', center_mm=[0, 0, 0], half_size_mm=[1, 1, 1])
    model.add_proxy(row)
    with pytest.raises(ValueError, match='Duplicate'):
        model.add_proxy(row)
    assert model.remove_geoms(names=['pad']) == ['pad']
    assert model.contact_records == []
    row['half_size_mm'][0] = -1
    with pytest.raises(ValueError, match='positive'):
        model.add_proxy(row)


CONTACTS = [
    dict(name='pad', body='link', type='box', center_mm=[10, 20, 30],
         half_size_mm=[1, 2, 3], quat_wxyz=[.8, 0, .6, 0]),
    dict(name='ball', body='payload', type='sphere', center_mm=[4, 5, 6], radius_mm=2),
    dict(name='shaft', body='link', type='cylinder', fromto_mm=[[0, 0, 0], [0, 20, 0]], radius_mm=3),
    dict(name='guard', body='world', type='capsule', fromto_mm=[[1, 2, 3], [10, 20, 30]], radius_mm=4),
]


def contact_model():
    model = PhysicalModel.from_xml(XML)
    for row in CONTACTS:
        model.add_proxy(row)
    return model


def test_contact_roundtrip_preserves_all_primitive_sources_and_identity():
    model = contact_model()
    source = model.to_source()
    restored = PhysicalModel.from_source(source)
    assert restored.to_source() == source
    assert restored.xml() == model.xml()
    assert digest(restored.to_source()) == digest(source)


@pytest.mark.parametrize('index,key,value', [
    (0, 'center_mm', [500, 20, 30]),
    (0, 'half_size_mm', [1, 2, 4]),
    (0, 'quat_wxyz', [1, 0, 0, 0]),
    (0, 'rgba', '.1 .2 .3 1'),
    (1, 'radius_mm', 5),
    (2, 'fromto_mm', [[0, 0, 0], [0, 40, 0]]),
    (3, 'type', 'cylinder'),
    (0, 'body', 'payload'),
    (0, 'body', 'world'),
])
def test_edited_contact_record_cannot_disagree_with_physical_geometry(index, key, value):
    source = contact_model().to_source()
    source['contact_records'][index][key] = value
    with pytest.raises(ValueError, match='differs from physical'):
        PhysicalModel.from_source(source)


@pytest.mark.parametrize('attribute,value', [
    ('pos', '0.5 .02 .03'), ('size', '.001 .002 .009'),
    ('quat', '1 0 0 0'), ('euler', '0 0 90'),
    ('size', '.001 .002 nan'), ('size', '.001 .002 .003 trailing'),
])
def test_edited_physical_geometry_cannot_keep_stale_contact_record(attribute, value):
    model = contact_model()
    model.root.find('.//geom[@name="pad"]').set(attribute, value)
    with pytest.raises(ValueError, match='differs from physical'):
        PhysicalModel.from_source(model.to_source())


@pytest.mark.parametrize('change', ['missing_geom', 'duplicate_geom', 'duplicate_record', 'missing_record_name'])
def test_contact_identity_must_resolve_to_one_named_geom(change):
    model = contact_model()
    if change in ('missing_geom', 'duplicate_geom'):
        body = model.bodies['link']
        geom = body.find('geom[@name="pad"]')
        if change == 'missing_geom':
            body.remove(geom)
        else:
            body.append(deepcopy(geom))
    source = model.to_source()
    if change == 'duplicate_record':
        source['contact_records'].append(deepcopy(source['contact_records'][0]))
    elif change == 'missing_record_name':
        source['contact_records'][0]['name'] = 'unbound'
    with pytest.raises(ValueError, match='named physical geom|Duplicate contact record'):
        PhysicalModel.from_source(source)


def test_known_body_attachment_edit_is_rejected_but_historical_alias_is_provenance_only():
    model = PhysicalModel.from_xml(XML)
    row = dict(CONTACTS[0], body='authored_link_alias')
    model.add_proxy(row, body_aliases={'authored_link_alias': 'link'})
    source = model.to_source()
    # The original serialized format did not retain the alias map. Preserve
    # that record unchanged: the actual XML attachment remains authoritative.
    assert PhysicalModel.from_source(source).to_source() == source
    source['contact_records'][0]['body'] = 'link'
    assert PhysicalModel.from_source(source).contact_records[0]['body'] == 'link'
    geom = model.bodies['link'].find('geom')
    model.bodies['link'].remove(geom)
    model.bodies['payload'].append(geom)
    moved = model.to_source()
    moved['contact_records'][0]['body'] = 'link'
    with pytest.raises(ValueError, match='physical body attachment'):
        PhysicalModel.from_source(moved)


def test_numeric_attribute_formatting_is_not_a_geometry_change():
    model = contact_model()
    geom = model.root.find('.//geom[@name="pad"]')
    geom.set('pos', '1e-2 0.020000000000000 3e-2')
    geom.set('rgba', '0.420000 0.460000 0.490000 1.00000')
    assert PhysicalModel.from_source(model.to_source()).xml() == model.xml()
