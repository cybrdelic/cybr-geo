"""Artifact verification must reject stale bytes and wrong motion independently."""
from copy import deepcopy
from dataclasses import replace
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest

from mechanism_lab.construction import recipe_digest
from mechanism_lab.constructed import build_from_plan
from mechanism_lab.exporters import export_glb, export_animated_glb, export_step


@pytest.fixture
def verifier():
    source = Path(__file__).resolve().parents[1] / 'tools' / 'verify_construction_bundle.py'
    spec = importlib.util.spec_from_file_location('verify_construction_bundle', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def bundle(tmp_path, monkeypatch, verifier):
    parts = [dict(name=name, body=dict(kind='box', size=[4, 4, 4], position=position, rotation=[0, 0, 0]),
                  cuts=[], unions=[], fillet=0, color=[.3, .4, .5], metalness=.5, roughness=.3)
             for name, position in [('anchor', [0, 0, 0]), ('link', [12, 0, 0])]]
    joints = [dict(name=name, child=child, parent=parent, type=kind, origin=[4, 0, 0], axis=[0, 0, 1],
                   min=-limit, max=limit, value=0, drive=None)
              for name, child, parent, kind, limit in [('mount', 'anchor', None, 'fixed', 0),
                                                      ('turn', 'link', 'anchor', 'revolute', 45)]]
    plan = dict(name='bundle_fixture', units='mm', parts=parts, joints=joints)
    plan['construction'] = dict(schema='cybrgeo-construction/1', architecture='fixture', recipe_sha256=recipe_digest(plan))
    assembly = build_from_plan(plan)
    (tmp_path / 'recipe.json').write_text(json.dumps(plan))
    (tmp_path / 'mechanism.json').write_text(json.dumps(dict(schema='cybrgeo-mechanism/1', architecture='fixture', parameters={})))
    export_glb(assembly, tmp_path / 'assembly.glb')
    animation = export_animated_glb(assembly, tmp_path / 'motion.glb', duration=8, fps=4)
    export_step(assembly, tmp_path / 'assembly.step', individual=False)
    receipt = dict(parts=len(parts), construction_recipe_sha256=recipe_digest(plan), animation=animation,
                   files_sha256={file.name: hashlib.sha256(file.read_bytes()).hexdigest() for file in tmp_path.iterdir() if file.is_file()})
    (tmp_path / 'bundle.json').write_text(json.dumps(receipt))
    monkeypatch.setattr(verifier, 'compile_plan', lambda *_: deepcopy(plan))
    return tmp_path, assembly, plan


def test_reopened_step_and_decoded_glb_motion_match(bundle, verifier):
    directory, _assembly, _plan = bundle
    report = verifier.verify_bundle(directory, sample_count=5)
    assert report['passed'], report['errors']
    assert report['step']['reopened_solids'] == 2
    assert report['step']['total_solid_volume_mm3'] == pytest.approx(128)
    assert report['static_glb']['named_mesh_nodes'] == 2
    assert report['motion_glb']['sampled_times_s'] == [0, 2, 4, 6, 8]
    assert report['motion_glb']['maximum_sampled_vertex_error_mm'] < 2e-6


def test_file_mutation_is_rejected_before_artifact_readback(bundle, verifier):
    directory, _assembly, _plan = bundle
    with (directory / 'assembly.step').open('ab') as file:
        file.write(b'changed')
    report = verifier.verify_bundle(directory)
    assert not report['passed']
    assert 'Manifest hash mismatch: assembly.step' in report['errors'][0]
    assert 'step' not in report


def test_stale_source_recipe_is_rejected(bundle, verifier, monkeypatch):
    directory, _assembly, plan = bundle
    changed = deepcopy(plan)
    changed['parts'][0]['body']['size'][0] += 1
    monkeypatch.setattr(verifier, 'compile_plan', lambda *_: changed)
    report = verifier.verify_bundle(directory)
    assert not report['passed']
    assert 'stale relative to the current construction compiler' in report['errors'][0]


def test_consistent_hashes_do_not_hide_wrong_animated_transforms(bundle, verifier):
    directory, assembly, _plan = bundle

    def wrong_motion(part, time, explode):
        matrix = assembly.pose(part, time, explode)
        matrix[0, 3] += time * 5  # The grounded anchor must not translate.
        return matrix

    animation = export_animated_glb(replace(assembly, motion_function=wrong_motion), directory / 'motion.glb', duration=8, fps=4)
    receipt = json.loads((directory / 'bundle.json').read_text())
    receipt['animation'] = animation
    for name in ('motion.glb', 'motion.animation.json'):
        receipt['files_sha256'][name] = hashlib.sha256((directory / name).read_bytes()).hexdigest()
    (directory / 'bundle.json').write_text(json.dumps(receipt))
    report = verifier.verify_bundle(directory)
    assert not report['passed']
    assert 'Exported motion differs from construction' in report['errors'][0]


def test_actual_integrated_glb_matches_trace_not_prescribed_motion(bundle, verifier):
    from mechanism_lab.constructed import KinematicGraph
    from mechanism_lab.dynamics import load_model, simulate, simulated_assembly
    directory, assembly, plan = bundle
    result = simulate(assembly, seconds=.3, timestep=.001, controller=False, output=directory / 'physics',
        settings={'gravity_mm_s2': [0, -9810, 0], 'actuators': [{'joint': 'turn', 'effort_limit_Nmm': 25, 'viscous_damping': 0}]})
    animation = export_animated_glb(simulated_assembly(assembly, result), directory / 'integrated.glb', duration=.3, fps=20)
    assert animation['motion_source'] == 'MuJoCo integrated rigid-body dynamics'
    assert 'Interpolated replay' in animation['note']
    graph = KinematicGraph([part.name for part in assembly.parts], plan['joints'])
    static, moving = verifier.read_glb(directory / 'assembly.glb'), verifier.read_glb(directory / 'integrated.glb')
    report = verifier.verify_simulated_motion(static, moving, graph, result, sample_count=5)
    assert report['maximum_sampled_vertex_error_mm'] < .00001
    model, manifest = load_model(directory / 'physics')
    assert verifier.verify_mujoco_states(model, manifest, result, 5)['maximum_pose_component_error_mm_or_dimensionless'] < 1e-10
    with pytest.raises(ValueError, match='differs from construction'):
        verifier.verify_motion(static, moving, graph, sample_count=5)
