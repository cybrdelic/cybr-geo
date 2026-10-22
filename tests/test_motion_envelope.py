"""Continuous clearance must catch collisions that endpoint sampling misses."""
from copy import deepcopy

import numpy as np
import pytest

from mechanism_lab.constructed import KinematicGraph, build_from_plan
from mechanism_lab.motion_envelope import (authorized_command, certify_path,
                                           command_in_certified_tube, plan_identity,
                                           joint_state_in_certified_tube,
                                           point_motion_bounds, point_speed_bound)


def primitive(kind='box', size=(2, 2, 2), position=(0, 0, 0)):
    return dict(kind=kind, size=list(size), position=list(position), rotation=[0, 0, 0])


def part(name, body, cuts=()):
    return dict(name=name, body=body, unions=[], cuts=list(cuts), fillet=0,
                color=[.3, .4, .5], roughness=.3, metalness=.5)


def joint(name, child, parent=None, kind='fixed', axis=(1, 0, 0), limits=(0, 0), value=0, origin=(0, 0, 0)):
    return dict(name=name, child=child, parent=parent, type=kind, axis=list(axis),
                origin=list(origin), min=limits[0], max=limits[1], value=value, drive=None)


def sliding_fixture(y=0):
    return dict(name='slide', units='mm', parts=[part('fixed', primitive()),
                part('moving', primitive(position=(-4, y, 0)))],
                joints=[joint('base', 'fixed'), joint('slide', 'moving', kind='prismatic', limits=(0, 8))])


def test_rejects_interior_collision_despite_clear_endpoints():
    assembly = build_from_plan(sliding_fixture())
    report = certify_path(assembly, [{'slide': 0}, {'slide': 8}], max_depth=6)
    assert not report['passed']
    assert report['failures'][0]['parts'] == ['fixed', 'moving']
    assert 'Unresolved clearance' in report['failures'][0]['reason']


def test_certifies_entire_adaptive_path_and_rejects_stale_or_off_path_commands():
    plan = sliding_fixture(y=2.2)
    assembly = build_from_plan(plan)
    report = certify_path(assembly, [{'slide': 1}, {'slide': 7}])
    assert report['passed']
    pair = report['segments'][0]['pairs'][0]
    assert pair['certified_intervals'] > 1
    assert pair['minimum_clearance_lower_bound_mm'] > 0
    assert authorized_command(plan, report, {'slide': 4}) == {'slide': 4}
    with pytest.raises(ValueError, match='outside the certified path'):
        authorized_command(plan, report, {'slide': 0})
    changed = deepcopy(plan)
    changed['parts'][0]['body']['size'][0] = 10
    with pytest.raises(ValueError, match='exact plan'):
        authorized_command(changed, report, {'slide': 4})
    report['passed'] = False
    with pytest.raises(ValueError, match='successful'):
        authorized_command(plan, report, {'slide': 4})


def test_invariant_attachment_still_requires_actual_nonpenetration():
    plan = sliding_fixture()
    plan['joints'][1] = joint('attachment', 'moving', parent='fixed')
    plan['parts'][1]['body']['position'] = [1, 0, 0]
    report = certify_path(build_from_plan(plan), [{}, {}])
    assert not report['passed']
    assert report['failures'][0]['invariant_intersection_volume_mm3'] == pytest.approx(4)


def test_constructive_axisymmetry_proves_rotating_journal_without_name_exemption():
    plan = dict(name='journal', units='mm', parts=[
        part('anything', primitive('cylinder', (6, 6, 4)), [primitive('cylinder', (2.4, 2.4, 6))]),
        part('else', primitive('cylinder', (2, 2, 8)))], joints=[
        joint('mount', 'anything'), joint('spin', 'else', kind='revolute', axis=(0, 0, 1), limits=(-180, 180))])
    report = certify_path(build_from_plan(plan), [{'spin': -180}, {'spin': 180}], max_depth=0)
    assert report['passed']
    row = report['segments'][0]['pairs'][0]
    assert row['invariance']['method'] == 'constructive_coaxial_rotational_invariance'
    assert row['minimum_clearance_lower_bound_mm'] == pytest.approx(.2 - 1e-4)
    plan['joints'][1]['origin'] = [.3, 0, 0]
    bad = certify_path(build_from_plan(plan), [{'spin': -180}, {'spin': 180}], max_depth=2)
    assert not bad['passed']
    assert 'invariance' not in bad['segments'][0]['pairs'][0]


def test_common_ancestor_cancels_but_moving_child_remains_bounded():
    plan = sliding_fixture(y=2.2)
    plan['joints'][0] = joint('root_spin', 'fixed', kind='revolute', axis=(0, 0, 1), limits=(-180, 180))
    plan['joints'][1]['parent'] = 'fixed'
    report = certify_path(build_from_plan(plan), [{'root_spin': -180, 'slide': 0}, {'root_spin': 180, 'slide': 8}])
    assert report['passed']
    assert report['segments'][0]['pairs'][0]['relative_point_speed_bound_mm'] == pytest.approx(8)


def test_rigid_speed_bound_covers_serial_rotation_translation_and_driven_joint():
    joints = [joint('root', 'a', kind='revolute', limits=(-180, 180), axis=(0, 0, 1)),
              joint('translate', 'b', 'a', 'prismatic', limits=(-4, 4)),
              joint('follower', 'c', 'b', 'revolute', axis=(0, 1, 0), limits=(-360, 360), origin=(3, 2, 1))]
    joints[-1]['drive'] = {'joint': 'root', 'ratio': 2, 'offset': 0}
    graph = KinematicGraph(['a', 'b', 'c'], joints)
    start, end = {'root': -30, 'translate': -3}, {'root': 55, 'translate': 4}
    points = np.array([[4, 2, 1], [6, 4, 3]])
    bound = point_speed_bound(points, ('root', 'translate', 'follower'), graph,
                             graph.resolve(start), graph.resolve(end))
    for u in np.linspace(0, .99, 100):
        def transformed(v):
            t = graph.transforms({n: start[n] + v*(end[n]-start[n]) for n in start})['c']
            return points @ t[:3, :3].T + t[:3, 3]
        assert np.max(np.linalg.norm(transformed(u+.001)-transformed(u), axis=1)) <= .001*bound


def test_identity_includes_material_and_mechanical_requirement_changes():
    plan = sliding_fixture()
    changed = deepcopy(plan)
    changed['mechanical_requirements'] = {'payload_kg': 1}
    assert plan_identity(plan) != plan_identity(changed)


def test_joint_error_tube_is_continuous_and_rejects_too_large_tracking_error():
    plan = sliding_fixture(y=2.2)
    plan['parts'].append(part('lateral_guide', primitive(position=(0, 0, 100))))
    plan['joints'].append(joint('lateral', 'lateral_guide', kind='prismatic', axis=(0, 1, 0), limits=(-1, 1)))
    plan['joints'][1]['parent'] = 'lateral_guide'
    assembly = build_from_plan(plan)
    path = [{'slide': 0, 'lateral': 0}, {'slide': 8, 'lateral': 0}]
    small = certify_path(assembly, path, joint_error_bounds={'lateral': .05})
    assert small['passed']
    assert command_in_certified_tube(plan, small, {'slide': 4, 'lateral': -.04})
    assert not command_in_certified_tube(plan, small, {'slide': 4, 'lateral': -.06})
    # A certified error tube never silently enables off-path commands.
    with pytest.raises(ValueError, match='outside the certified path'):
        authorized_command(plan, small, {'slide': 4, 'lateral': -.04})
    large = certify_path(assembly, path, joint_error_bounds={'lateral': .3}, max_depth=5)
    assert not large['passed']
    assert large['failures'][0]['joint_error_displacement_bound_mm'] >= .3
    with pytest.raises(ValueError, match='nonnegative'):
        certify_path(assembly, path, joint_error_bounds={'lateral': -.1})


def test_axisymmetric_claim_must_match_the_actual_brep():
    import cadquery as cq
    plan = dict(name='journal', units='mm', parts=[
        part('ring', primitive('cylinder', (6, 6, 4)), [primitive('cylinder', (2.4, 2.4, 6))]),
        part('shaft', primitive('cylinder', (2, 2, 8)))], joints=[
        joint('mount', 'ring'), joint('spin', 'shaft', kind='revolute', axis=(0, 0, 1), limits=(-180, 180))])
    assembly = build_from_plan(plan)
    assembly.parts[0].cad = assembly.parts[0].cad.fuse(cq.Workplane('XY').box(.3, .3, .3).translate((1.2, 0, 0)).val())
    with pytest.raises(ValueError, match='actual B-rep differs'):
        certify_path(assembly, [{'spin': -180}, {'spin': 180}], max_depth=0)


def test_error_displacement_bound_covers_rotating_descendant_uncertainty():
    joints = [joint('root', 'a', kind='revolute', limits=(-180, 180), axis=(0, 0, 1)),
              joint('child', 'b', 'a', 'revolute', limits=(-180, 180), axis=(0, 1, 0), origin=(3, 0, 0))]
    graph = KinematicGraph(['a', 'b'], joints)
    start, end = {'root': -30, 'child': 10}, {'root': 40, 'child': 70}
    points = np.array([[4, 0, 0], [6, 2, 3]])
    speed, error = point_motion_bounds(points, ('root', 'child'), graph, start, end, {'root': 3, 'child': 5})
    for u in np.linspace(0, 1, 11):
        nominal = {n: start[n] + u*(end[n]-start[n]) for n in start}
        middle = {n: .5*(start[n]+end[n]) for n in start}
        m = graph.transforms(middle)['b']
        p = points @ m[:3, :3].T + m[:3, 3]
        for root_error, child_error in ((-3, -5), (-3, 5), (3, -5), (3, 5)):
            t = graph.transforms({'root': nominal['root']+root_error, 'child': nominal['child']+child_error})['b']
            actual = points @ t[:3, :3].T + t[:3, 3]
            assert np.max(np.linalg.norm(actual-p, axis=1)) <= abs(u-.5)*speed + error


def test_compliant_follower_residual_is_added_to_propagated_source_error():
    plan = dict(name='compliant_drive', units='mm', parts=[part('base', primitive()),
        part('driver', primitive(position=(0, 0, 100))), part('follower', primitive(position=(0, 2.2, 0)))],
        joints=[joint('mount', 'base'), joint('motor', 'driver', kind='prismatic', limits=(0, 1)),
                joint('driven', 'follower', kind='prismatic', axis=(0, 1, 0), limits=(-1, 1))])
    plan['joints'][-1]['drive'] = {'joint': 'motor', 'ratio': .2, 'offset': 0}
    assembly = build_from_plan(plan)
    path = [{'motor': 0}, {'motor': 1}]
    report = certify_path(assembly, path, joint_error_bounds={'motor': .01}, drive_error_bounds={'driven': .05})
    assert report['passed']
    assert report['resolved_joint_error_bounds']['driven'] == pytest.approx(.052)
    assert joint_state_in_certified_tube(plan, report, {'motor': .5, 'driven': .05})
    assert not joint_state_in_certified_tube(plan, report, {'motor': .5, 'driven': 0})
    assert joint_state_in_certified_tube(plan, report, {'motor': -.005, 'driven': -.001})
    with pytest.raises(ValueError, match='outside'):
        authorized_command(plan, report, {'motor': -.005})
    with pytest.raises(ValueError, match='including followers'):
        joint_state_in_certified_tube(plan, report, {'motor': .5})
    unsafe = certify_path(assembly, path, drive_error_bounds={'driven': .3}, max_depth=3)
    assert not unsafe['passed']
