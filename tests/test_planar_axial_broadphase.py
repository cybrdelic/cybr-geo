"""Invariant-Z pruning must never turn unresolved native pairs into clearance."""
import hashlib
import json
import math

import cadquery as cq
import pytest

import mechanism_lab.planar_native_envelope as planar_module
from mechanism_lab.planar_linkage import PlanarFourBar
from mechanism_lab.planar_native_envelope import PlanarBody, PlanarNativePath


def _fixture(tmp_path):
    native = tmp_path / 'native'
    placed = native / 'cad' / 'placed'
    placed.mkdir(parents=True)
    rows, hashes = [], {}
    for name in ('fixed__housing', 'crank__shaft', 'rocker__bridge'):
        body, part = name.split('__')
        relative = f'cad/placed/{name}.step'
        cq.exporters.export(cq.Workplane('XY').box(1., 1., 1.).val(),
                            str(native / relative))
        rows.append(dict(body=body, name=part, step_file=relative))
        hashes[relative] = hashlib.sha256((native / relative).read_bytes()).hexdigest()
    manifest = native / 'placed-native-parts.json'
    manifest.write_text(json.dumps(dict(units='mm', records=rows)), encoding='utf-8')
    hashes[manifest.name] = hashlib.sha256(manifest.read_bytes()).hexdigest()
    linkage = PlanarFourBar((0., 0.), (4., 0.), 1., 4., 3.)
    path = PlanarNativePath(dict(fixed=PlanarBody('ground'),
        crank=PlanarBody('crank', linkage),
        rocker=PlanarBody('rocker', linkage, z_mm=5.)),
        0., 1., source_identity='three-part-unit-source')
    return native, dict(files_sha256=hashes), path


def test_invariant_axial_slabs_prove_only_separated_full_range_pairs(tmp_path):
    native, receipt, path = _fixture(tmp_path)
    report = path.axial_clearance_broadphase(native, receipt,
                                             required_clearance_mm=.25)
    assert report['screened']
    assert report['status'] == 'unresolved_cross_body_pairs_remain'
    assert not report['full_native_noncontact_pair_coverage']
    assert report['full_native_inventory_selected']
    assert report['cross_body_pairs'] == 3
    assert report['axially_separated_pair_count'] == 2
    assert report['unresolved_pair_count'] == 1
    assert report['unresolved_pairs'][0]['parts'] == ['crank__shaft', 'fixed__housing']
    assert all(row['certified_clearance_lower_bound_mm'] > .25
               for row in report['axially_separated_pairs'])
    assert report['part_axial_intervals']['rocker__bridge']['z_interval_mm'][0] > 4.49
    assert report['source_binding']['planar_source_sha256'] == path.source_sha256
    assert report['native_step_import']['externally_verified']


def test_mutated_or_nonplanar_pose_cannot_inherit_axial_proof(tmp_path, monkeypatch):
    native, receipt, path = _fixture(tmp_path)
    path.pose_at = lambda _u: {}  # a caller could otherwise hide Z motion
    with pytest.raises(ValueError, match='unmodified pose and speed'):
        path.axial_clearance_broadphase(native, receipt)
    del path.pose_at
    object.__setattr__(path.bodies['rocker'], 'z_mm', 0.)
    with pytest.raises(ValueError, match='source changed'):
        path.axial_clearance_broadphase(native, receipt)

    class TiltingPath(PlanarNativePath):
        def pose_at(self, u):
            poses = super().pose_at(u)
            poses['rocker'][2, 3] += math.sin(u)
            return poses

    linkage = PlanarFourBar((0., 0.), (4., 0.), 1., 4., 3.)
    tilted = TiltingPath(dict(fixed=PlanarBody('ground'),
        crank=PlanarBody('crank', linkage), rocker=PlanarBody('rocker', linkage,
            z_mm=5.)), 0., 1., source_identity='tilted-path')
    with pytest.raises(ValueError, match='unmodified pose and speed'):
        tilted.axial_clearance_broadphase(native, receipt)

    class RollingPath(PlanarNativePath):
        def pose_at(self, u):
            poses = super().pose_at(u)
            poses['rocker'][2, 1] = math.sin(u)  # out-of-plane rotation
            return poses

    rolled = RollingPath(dict(fixed=PlanarBody('ground'),
        crank=PlanarBody('crank', linkage), rocker=PlanarBody('rocker', linkage,
            z_mm=5.)), 0., 1., source_identity='rolling-path')
    with pytest.raises(ValueError, match='unmodified pose and speed'):
        rolled.axial_clearance_broadphase(native, receipt)

    normal = PlanarNativePath(dict(fixed=PlanarBody('ground'),
        crank=PlanarBody('crank', linkage), rocker=PlanarBody('rocker', linkage,
            z_mm=5.)), 0., 1., source_identity='class-monkeypatch')
    monkeypatch.setattr(PlanarNativePath, 'pose_at', lambda _self, _u: {})
    with pytest.raises(ValueError, match='unmodified pose and speed'):
        normal.axial_clearance_broadphase(native, receipt)


@pytest.mark.parametrize('override', ('pose_at', 'angle_at', 'point_speed_bound'))
def test_all_native_proofs_reject_runtime_path_overrides(tmp_path, override):
    native, receipt, path = _fixture(tmp_path)
    setattr(path, override, lambda *_args, **_kwargs: 0.)
    for prove in (lambda: path.certify_native(native, receipt),
                  lambda: path.axial_clearance_broadphase(native, receipt),
                  lambda: path.gravity_budget(native, receipt, [0., -9.81, 0.],
                                              [0., 0., 1.])):
        with pytest.raises(ValueError, match='unmodified pose and speed'):
            prove()


def test_native_certificate_rejects_source_change_during_screening(tmp_path, monkeypatch):
    native, receipt, path = _fixture(tmp_path)

    def certificate(_parts, _poses, _bound, **_kwargs):
        object.__setattr__(path.bodies['rocker'], 'z_mm', 0.)
        return dict(passed=True)

    monkeypatch.setattr(planar_module, 'certify_parametric_path', certificate)
    with pytest.raises(ValueError, match='source changed'):
        path.certify_native(native, receipt)


def test_tampered_native_step_and_invalid_margin_fail_closed(tmp_path):
    native, receipt, path = _fixture(tmp_path)
    with pytest.raises(ValueError, match='margin positive'):
        path.axial_clearance_broadphase(native, receipt, distance_margin_mm=0.)
    (native / 'cad/placed/rocker__bridge.step').write_bytes(b'altered')
    with pytest.raises(ValueError, match='verified receipt'):
        path.axial_clearance_broadphase(native, receipt)
