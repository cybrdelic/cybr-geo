"""Independent CAD audit must detect geometry faults despite a valid joint tree."""
import importlib.util
from pathlib import Path

import cadquery as cq
import numpy as np

from mechanism_lab.core import Assembly, Material, cad_part


def audit_function():
    path = Path(__file__).resolve().parents[1] / 'tools' / 'audit_constructed.py'
    spec = importlib.util.spec_from_file_location('audit_constructed', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.audit


def fixture():
    shape = cq.Workplane('XY').box(2, 2, 2).val()
    parts = [cad_part('a', shape), cad_part('b', shape.translate((1, 0, 0)))]

    def pose(part, time, _explode):
        transform = np.eye(4)
        transform[0, 3] = time * 3 if part.name == 'b' else 0
        return transform

    return Assembly('collision_fixture', parts, [Material('steel', (.3, .3, .3))], motion_function=pose)


def test_actual_overlap_is_not_hidden_by_attachment_metadata():
    assembly = fixture()
    assembly.metadata = {'construction': {'load_paths': [{'parts': ['a', 'b']}],
                                          'interfaces': [{'id': 'nominal-fit', 'parts': ['a', 'b']}]}}
    report = audit_function()(assembly, times=[0, 1])
    assert not report['passed']
    assert report['samples'][0]['unexpected_interferences'][0]['intersection_volume_mm3'] == 4
    assert not report['samples'][1]['unexpected_interferences']
    assert report['samples'][0]['interfaces'][0]['minimum_surface_distance_mm'] == 0
    assert report['samples'][1]['interfaces'][0]['minimum_surface_distance_mm'] == 2


def test_explicit_interference_allowance_is_bounded_and_visible():
    assembly = fixture()
    assembly.metadata = {'construction': {'intentional_interferences': [
        {'parts': ['a', 'b'], 'reason': 'test-only prescribed bonded overlap', 'maximum_volume_mm3': 4.1}]}}
    report = audit_function()(assembly, times=[0])
    assert report['passed']
    assert report['samples'][0]['declared_interferences'][0]['intersection_volume_mm3'] == 4
    assembly.metadata['construction']['intentional_interferences'][0]['maximum_volume_mm3'] = 3
    assert not audit_function()(assembly, times=[0])['passed']


def test_rigid_pair_reuse_preserves_detected_interference():
    assembly = fixture()

    def together(_part, time, _explode):
        transform = np.eye(4)
        transform[:3, 3] = [time * 4, time * 2, time * 3]
        return transform

    assembly.motion_function = together
    report = audit_function()(assembly, times=[0, 1])
    assert not report['passed']
    assert report['samples'][0]['intersection_kernel_queries'] == 1
    assert report['samples'][1]['unchanged_relative_pose_reuses'] == 1
    assert report['samples'][1]['unexpected_interferences'][0]['intersection_volume_mm3'] == 4


def test_capture_probe_requires_a_geometric_stop():
    assembly = fixture()
    assembly.parts[1] = cad_part('b', cq.Workplane('XY').box(2, 2, 2).translate((2.2, 0, 0)).val())
    assembly.motion_function = None
    probe = {'name': 'forward-stop', 'moving_part': 'a', 'blocker': 'b',
             'direction': [1, 0, 0], 'distance_mm': 1}
    assembly.metadata = {'construction': {'capture_probes': [probe]}}
    report = audit_function()(assembly, times=[0])
    assert report['passed']
    assert report['samples'][0]['capture_probes'][0]['virtual_displacement_interference_mm3'] == 3.2
    probe['direction'] = [-1, 0, 0]
    report = audit_function()(assembly, times=[0])
    assert not report['passed']
    assert not report['samples'][0]['capture_probes'][0]['blocked']
