"""One bounded planar CAD path is reusable by distinct engineering clients."""
import hashlib
import json
import math
from pathlib import Path

import cadquery as cq
import numpy as np
import pytest

from mechanism_lab.planar_linkage import PlanarFourBar
from mechanism_lab.planar_native_envelope import PlanarBody, PlanarNativePath


def test_fourbar_client_initial_frames_are_the_shared_closure():
    from examples.engineering.fourbar_lift import mechanism as client
    spec = client.specification()
    initial = client.layout(spec)['frames']
    path = PlanarNativePath(dict(fixed=PlanarBody('ground'),
        crank=PlanarBody('crank', client.linkage(spec)),
        rocker=PlanarBody('rocker', client.linkage(spec)),
        coupler=PlanarBody('coupler', client.linkage(spec), z_mm=spec['coupler_height_mm'])),
        spec['initial_crank_angle_rad'],
        spec['initial_crank_angle_rad']+math.tau,
        source_identity='fourbar-lift-authored-specification')
    pose = path.pose_at(0.)
    for body, expected in initial.items():
        assert np.allclose(pose[body], expected, atol=1e-10), body
    assert path.point_speed_bound('coupler', 100.) > path.point_speed_bound('fixed', 100.)
    assert path.angular_bounds


def test_radial_iris_client_initial_frames_use_the_same_branch():
    from examples.engineering.iris09 import mechanism as client
    spec = client.specification()
    initial = client.layout(spec)['frames']
    solved = client.linkage(spec)
    bodies = dict(fixed=PlanarBody('ground'), rotor=PlanarBody('rocker', solved))
    for index in range(9):
        azimuth = index*math.tau/9
        bodies[f'leaf_{index:02}'] = PlanarBody('crank', solved,
            world_rotation_rad=-azimuth,
            yaw_offset_rad=-spec['leaf_crank_angle_rad'])
        bodies[f'rod_{index:02}'] = PlanarBody('coupler', solved,
            world_rotation_rad=-azimuth, yaw_offset_rad=math.pi,
            anchor='rocker_pin_mm', z_mm=spec['rod_height_mm'])
    path = PlanarNativePath(bodies, 0., -.9,
        source_identity='iris-derived-source-geometry-and-new-rod-layout')
    pose = path.pose_at(0.)
    for body, expected in initial.items():
        assert np.allclose(pose[body], expected, atol=1e-9), body
    assert path.point_speed_bound('rod_00', 21.) > 0.
    assert path.point_speed_bound('fixed', 900.) == 0.


def test_placed_step_binding_and_full_range_clearance(tmp_path):
    native = tmp_path / 'native'
    step = native / 'cad' / 'placed'
    step.mkdir(parents=True)
    shapes = dict(fixed__support=cq.Workplane('XY').box(2, 2, 2).val().translate((90, 90, 0)),
                  rocker__tip=cq.Workplane('XY').box(1, 1, 1).val())
    rows, hashes = [], {}
    for identity, shape in shapes.items():
        body, name = identity.split('__', 1)
        relative = f'cad/placed/{identity}.step'
        destination = native / relative
        cq.exporters.export(shape, str(destination))
        rows.append(dict(body=body, name=name, step_file=relative))
        hashes[relative] = hashlib.sha256(destination.read_bytes()).hexdigest()
    manifest = native / 'placed-native-parts.json'
    manifest.write_text(json.dumps(dict(units='mm', records=rows)), encoding='utf-8')
    hashes[manifest.name] = hashlib.sha256(manifest.read_bytes()).hexdigest()
    linkage = PlanarFourBar((0, 0), (4, 0), 1, 4, 3)
    path = PlanarNativePath(dict(fixed=PlanarBody('ground'),
        rocker=PlanarBody('rocker', linkage)), 0., math.tau,
        source_identity='two-body-unit-construction')
    report = path.certify_native(native, dict(files_sha256=hashes))
    assert report['passed']
    assert report['complete_pair_accounting']
    assert report['unexamined_cross_body_pairs'] == 0
    assert report['native_step_import']['externally_verified']
    assert report['planar_kinematics']['source_sha256'] == path.source_sha256
    assert report['planar_kinematics']['implementation_sha256']['planar_linkage.py'] == hashlib.sha256(
        (Path(__file__).resolve().parents[1] / 'src/mechanism_lab/planar_linkage.py').read_bytes()).hexdigest()
    mass_file = native / 'mass_properties.json'
    mass_file.write_text(json.dumps(dict(fixed=dict(mass_kg=2., com_m=[0., 0., 0.]),
        rocker=dict(mass_kg=1., com_m=[.02, 0., 0.]))), encoding='utf-8')
    hashes['mass_properties.json'] = hashlib.sha256(mass_file.read_bytes()).hexdigest()
    gravity = path.gravity_budget(native, dict(files_sha256=hashes),
                                  [0., -9.81, 0.], [0., 0., 1.])
    assert gravity['total_gravity_torque_bound_Nm'] > 0.
    assert gravity['native_mass_properties_sha256'] == hashes['mass_properties.json']
    assert not gravity['hardware_load_qualified']
    mass_file.write_text(json.dumps(dict(fixed=dict(mass_kg=2., com_m=[0., 0., 0.]))))
    with pytest.raises(ValueError, match='verified receipt'):
        path.gravity_budget(native, dict(files_sha256=hashes),
                            [0., -9.81, 0.], [0., 0., 1.])
    (native / rows[0]['step_file']).write_bytes(b'tampered')
    with pytest.raises(ValueError, match='verified receipt'):
        path.certify_native(native, dict(files_sha256=hashes))


def test_analytic_branch_fails_closed_at_hidden_toggle():
    with pytest.raises(ValueError, match='change-point'):
        PlanarNativePath(dict(fixed=PlanarBody('ground'),
            rocker=PlanarBody('rocker', PlanarFourBar((0, 0), (4, 0), 1, 4, 1))),
            0., math.tau, source_identity='unreachable-across-turn')


def test_source_and_scope_are_explicit():
    linkage = PlanarFourBar((0, 0), (4, 0), 1, 4, 3)
    bindings = dict(fixed=PlanarBody('ground'), rocker=PlanarBody('rocker', linkage))
    a = PlanarNativePath(bindings, 0., 1., source_identity='revision-a')
    b = PlanarNativePath(bindings, 0., 1., source_identity='revision-b')
    assert a.source_sha256 != b.source_sha256
    with pytest.raises(ValueError, match='authored source identity'):
        PlanarNativePath(bindings, 0., 1., source_identity='')
    with pytest.raises(ValueError, match='coordinate yaw'):
        PlanarBody('rocker', linkage, yaw_offset_rad=float('nan'))
