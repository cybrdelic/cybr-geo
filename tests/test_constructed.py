"""Construction motion survives CAD lowering, caching and actual GLB serialization."""
from copy import deepcopy
import json
import math
import struct

import numpy as np
import pytest

from mechanism_lab.constructed import KinematicGraph, articulation, build_from_plan, pose, primitive


def joint(name, child, parent=None, kind='fixed', origin=(0, 0, 0), axis=(0, 0, 1), limits=(0, 0), drive=None):
    return dict(name=name, child=child, parent=parent, type=kind, origin=list(origin), axis=list(axis),
                min=limits[0], max=limits[1], value=0, drive=drive)


def fixture_plan():
    parts = []
    for name, position in [('base', [0, 0, 0]), ('link', [10, 0, 0]), ('carriage', [15, 0, 0]), ('opposed', [15, 4, 0])]:
        parts.append(dict(name=name, body=dict(kind='box', size=[2, 2, 2], position=position, rotation=[0, 0, 0]),
                          cuts=[], unions=[], fillet=0, color=[.3, .4, .5], metalness=.7, roughness=.4))
    return dict(name='construction_fixture', units='mm', parts=parts, joints=[
        joint('anchor', 'base'),
        joint('yaw', 'link', 'base', 'revolute', (10, 0, 0), limits=(-90, 90)),
        joint('travel', 'carriage', 'link', 'prismatic', axis=(1, 0, 0), limits=(0, 4)),
        joint('opposition', 'opposed', 'link', 'prismatic', axis=(1, 0, 0), limits=(-4, 0),
              drive=dict(joint='travel', ratio=-1, offset=0)),
    ])


def graph(plan=None):
    plan = plan or fixture_plan()
    return KinematicGraph([p['name'] for p in plan['parts']], plan['joints'])


def test_parent_rotation_carries_child_translation_and_pivot():
    rig = graph()
    transforms = rig.transforms({'yaw': 90, 'travel': 3})
    # The child's reference X displacement becomes world Y after parent yaw.
    point = transforms['carriage'] @ [15, 0, 0, 1]
    assert np.allclose(point, [10, 8, 0, 1], atol=1e-10)
    assert np.allclose(transforms['opposed'] @ [15, 4, 0, 1], [6, 2, 0, 1], atol=1e-10)
    assert np.array_equal(transforms['base'], np.eye(4))
    # The joint pivot is invariant under its own rotation.
    assert np.allclose(transforms['link'] @ [10, 0, 0, 1], [10, 0, 0, 1])


def test_drive_chain_uses_derived_values_and_rejects_direct_follower_commands():
    rig = graph()
    assert rig.resolve({'travel': 2.5})['opposition'] == -2.5
    with pytest.raises(ValueError, match='independent'):
        rig.resolve({'opposition': -1})
    with pytest.raises(ValueError, match='outside'):
        rig.resolve({'travel': 5})
    with pytest.raises(ValueError, match='finite'):
        rig.resolve({'yaw': float('nan')})


@pytest.mark.parametrize('fault', ['hierarchy_cycle', 'drive_cycle', 'missing_parent', 'duplicate_child', 'zero_axis', 'drive_range'])
def test_invalid_graph_is_rejected_before_cad(fault, monkeypatch):
    plan = fixture_plan()
    if fault == 'hierarchy_cycle':
        plan['joints'][0]['parent'] = 'carriage'
    elif fault == 'drive_cycle':
        plan['joints'][2]['drive'] = dict(joint='opposition', ratio=-1, offset=0)
    elif fault == 'missing_parent':
        plan['joints'][1]['parent'] = 'missing'
    elif fault == 'duplicate_child':
        plan['joints'][2]['child'] = 'link'
    elif fault == 'zero_axis':
        plan['joints'][1]['axis'] = [0, 0, 0]
    else:
        plan['joints'][3]['min'] = -2
    monkeypatch.setattr('mechanism_lab.constructed.primitive', lambda _: pytest.fail('CAD must not start before graph validation'))
    with pytest.raises(ValueError):
        build_from_plan(plan)


def test_demo_is_bounded_periodic_and_rigid():
    rig = graph()
    assert rig.demo_values(0) == rig.demo_values(8)
    assert set(rig.demo_values(2)) == set(rig.controls)
    for time in np.linspace(0, 8, 33):
        for transform in rig.transforms(rig.demo_values(float(time))).values():
            assert np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3))
            assert abs(np.linalg.det(transform[:3, :3]) - 1) < 1e-10


def test_native_primitive_rotation_and_boolean_bore():
    cylinder = primitive(dict(kind='cylinder', size=[4, 4, 12], position=[3, 5, 7], rotation=[0, 90, 0]))
    box = cylinder.BoundingBox()
    assert [box.xmin, box.xmax] == pytest.approx([-3, 9])
    assert [box.ymin, box.ymax] == pytest.approx([3, 7])
    assert [box.zmin, box.zmax] == pytest.approx([5, 9])
    plan = fixture_plan()
    plan['parts'][0]['body']['size'] = [10, 10, 10]
    plan['parts'][0]['cuts'] = [dict(kind='cylinder', size=[4, 4, 12], position=[0, 0, 0], rotation=[0, 0, 0])]
    assembly = build_from_plan(plan)
    assert assembly.parts[0].cad.Volume() == pytest.approx(1000 - 40 * math.pi)
    assert all(len(part.cad.Solids()) == 1 and part.cad.isValid() for part in assembly.parts)


@pytest.fixture
def built_fixture():
    return build_from_plan(fixture_plan())


def test_cached_graph_rebinds_same_motion_and_explicit_pose_does_not_mutate(built_fixture, tmp_path):
    from mechanism_lab.core import save_cache, load_cache
    save_cache(built_fixture, tmp_path)
    restored = load_cache(tmp_path)
    restored.motion_function = pose.bind(restored)
    for original, cached in zip(built_fixture.parts, restored.parts):
        assert np.allclose(built_fixture.pose(original, 2), restored.pose(cached, 2))
    selected = articulation(restored, {'yaw': 30, 'travel': 1})
    assert 'selected_joint_values' not in restored.metadata
    assert selected.metadata['selected_joint_values']['opposition'] == -1
    assert not np.allclose(selected.pose(selected.parts[1]), restored.pose(restored.parts[1]))
    assert np.allclose(selected.pose(selected.parts[1], 0), selected.pose(selected.parts[1], 3))
    save_cache(selected, tmp_path / 'selected')
    selected_cache = load_cache(tmp_path / 'selected')
    selected_cache.motion_function = pose.bind(selected_cache)
    for original, cached in zip(selected.parts, selected_cache.parts):
        assert np.allclose(selected.pose(original, 3), selected_cache.pose(cached, 3))


def test_animated_glb_serializes_coupled_hierarchy_motion(built_fixture, tmp_path):
    from mechanism_lab.exporters import export_animated_glb
    from scipy.spatial.transform import Rotation
    output = tmp_path / 'constructed.glb'
    export_animated_glb(built_fixture, output, duration=8, fps=4)
    raw = output.read_bytes()
    magic, version, total = struct.unpack_from('<III', raw)
    assert (magic, version, total) == (0x46546C67, 2, len(raw))
    length, kind = struct.unpack_from('<II', raw, 12)
    assert kind == 0x4E4F534A
    doc = json.loads(raw[20:20 + length])
    binary = raw[28 + length:]

    def values(accessor):
        access = doc['accessors'][accessor]
        view = doc['bufferViews'][access['bufferView']]
        width = {'SCALAR': 1, 'VEC3': 3, 'VEC4': 4}[access['type']]
        return np.frombuffer(binary, dtype='<f4', count=access['count'] * width,
                             offset=view.get('byteOffset', 0) + access.get('byteOffset', 0)).reshape(-1, width)

    channels = {}
    animation = doc['animations'][0]
    for channel in animation['channels']:
        sampler = animation['samplers'][channel['sampler']]
        channels[(channel['target']['node'], channel['target']['path'])] = values(sampler['output'])
    assert len(channels) >= 4
    names = {n.get('name'): i for i, n in enumerate(doc['nodes'])}
    for part in built_fixture.parts:
        index = names[part.name]
        node = doc['nodes'][index]
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_quat(channels.get((index, 'rotation'), [node.get('rotation', [0, 0, 0, 1])] * 33)[16]).as_matrix()
        transform[:3, 3] = channels.get((index, 'translation'), [node.get('translation', [0, 0, 0])] * 33)[16]
        assert np.allclose(transform, built_fixture.pose(part, 4), atol=2e-6)


def test_real_construction_round_trip():
    from mechanism_lab.construction import compile_plan
    for architecture in ('compact_arm', 'belt_arm', 'inspection_head'):
        plan = compile_plan(architecture)
        rig = graph(plan)
        assert len(rig.controls) >= 3
        assert len(rig.part_names) == len(plan['parts'])
        assert len(rig.report()['roots']) == 1
        for time in (0, 2, 4, 8):
            resolved = rig.resolve(rig.demo_values(time))
            assert all(math.isfinite(v) for v in resolved.values())


@pytest.mark.parametrize('change', ['geometry', 'joint', 'missing_hash', 'unsupported_schema'])
def test_construction_claims_reject_mutation_before_cad(change, monkeypatch):
    from mechanism_lab.construction import recipe_digest
    plan = fixture_plan()
    plan['construction'] = {'schema': 'cybrgeo-construction/1', 'recipe_sha256': recipe_digest(plan)}
    if change == 'geometry':
        plan['parts'][1]['body']['position'][0] += .01
    elif change == 'joint':
        plan['joints'][1]['origin'][0] += .01
    elif change == 'missing_hash':
        del plan['construction']['recipe_sha256']
    else:
        plan['construction']['schema'] = 'unsupported'
    monkeypatch.setattr('mechanism_lab.constructed.primitive', lambda _: pytest.fail('Mutated construction claims must fail before CAD'))
    with pytest.raises(ValueError, match='Construction no longer|Unsupported construction'):
        build_from_plan(plan)


def test_intact_construction_digest_allows_native_cad():
    from mechanism_lab.construction import recipe_digest
    plan = fixture_plan()
    plan['construction'] = {'schema': 'cybrgeo-construction/1', 'recipe_sha256': recipe_digest(plan)}
    assembly = build_from_plan(plan)
    assert len(assembly.parts) == 4
    assert assembly.metadata['construction']['recipe_sha256'] == recipe_digest(plan)


@pytest.mark.parametrize('change', ['geometry', 'joint', 'schema'])
def test_cached_construction_claims_are_checked_before_motion_binding(built_fixture, tmp_path, change):
    from mechanism_lab.construction import recipe_digest
    from mechanism_lab.core import save_cache, load_cache
    plan = built_fixture.metadata['construction_recipe']
    plan['construction'] = {'schema': 'cybrgeo-construction/1', 'recipe_sha256': recipe_digest(plan)}
    save_cache(built_fixture, tmp_path)
    manifest = tmp_path / 'manifest.json'
    contents = json.loads(manifest.read_text())
    cached_plan = contents['metadata']['construction_recipe']
    if change == 'joint':
        cached_plan['joints'][1]['origin'][0] += 1
    elif change == 'geometry':
        cached_plan['parts'][1]['body']['size'][0] += 1
    else:
        cached_plan['construction']['schema'] = 'unsupported'
    manifest.write_text(json.dumps(contents))
    restored = load_cache(tmp_path)
    with pytest.raises(ValueError, match='Construction no longer|Unsupported construction'):
        pose.bind(restored)
