"""Engineering media must consume native CAD and accepted integrated states."""
from copy import deepcopy
import gzip
import json
from pathlib import Path

import cadquery as cq
import numpy as np
import pytest

from mechanism_lab import engineering_media as media
from mechanism_lab.causal_runtime import CausalSimulation
from mechanism_lab.core import Material
from mechanism_lab.engineering_source import (
    EngineeringProject, construct_project_native, project_source_identity,
    verify_native_build,
)
from mechanism_lab.native_construction import NativeConstruction
from mechanism_lab.native_replay import build_assembly
from mechanism_lab.physical_model import PhysicalModel, digest


def save(path, value):
    payload = json.dumps(value, allow_nan=False).encode()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(gzip.compress(payload, mtime=0) if path.suffix == '.gz' else payload)


@pytest.fixture
def accepted(tmp_path, monkeypatch):
    source = tmp_path / 'project'
    source.mkdir()
    (source / 'project.py').write_text('# Synthetic media protocol integration fixture.\n')
    sources = project_source_identity(source)
    specification = {'size_mm': [30, 6, 6], 'density_kg_m3': 2700}
    root = tmp_path / 'run'
    root.mkdir()

    def construct(spec, output):
        native = NativeConstruction(output, {'metal': spec['density_kg_m3']},
            provenance={'input_spec_sha256': digest(spec)})
        shape = cq.Workplane('XY').box(*spec['size_mm']).val().translate((15, 0, 0))
        native.addpart('bar', shape, 'metal')
        native.place('bar', 'bar_solid', shape, 'metal')
        native.finish('Synthetic media integration fixture, assumed density')

    def requirements(result, parameters):
        return dict(passed=bool(result['frames'][-1]['qpos'][0] > .05),
                    scope='Synthetic media fixture moved under actual motor force')

    def replay(directory, result):
        return build_assembly(directory, result, specification=specification,
            body_mapping={'bar': 'bar'}, mass_mapping={'bar': 'bar'},
            materials={'metal': Material('Test metal', (.4, .5, .6), .5, .3)})

    project = EngineeringProject('fixture', specification, construct, None, None, replay, requirements)
    receipt, binding = construct_project_native(project, source, sources, root / 'native')
    model = PhysicalModel.from_xml('''<mujoco><option timestep=".001" gravity="0 0 0"/>
      <worldbody><body name="bar"><joint name="swing" axis="0 0 1"/></body></worldbody>
      <actuator><motor name="motor" joint="swing"/></actuator></mujoco>''',
      provenance={'native_build_sha256': digest(receipt), 'native_source_binding_sha256': digest(binding)})
    model.bind_inertias(json.loads((root / 'native/mass_properties.json').read_text()), {'bar': 'bar'})
    motor = dict(gear_ratio=10., gear_efficiency=.9, torque_constant_Nm_A=.02,
        resistance_ohm=2., inductance_H=.01, bus_voltage_V=24., peak_current_A=2.,
        gear_efficiency_transition_motor_rad_s=3., thermal_resistance_K_W=10.,
        thermal_capacity_J_K=20., gear_output_torque_limit_Nm=.2)
    control = dict(drives=[dict(joint='swing', actuator='motor', motor=motor, kp=.03, kd=.002,
        ki=0., integrator_limit=.1, soft_limits=[-3., 3.])],
        inertial_feedforward=False, gravity_and_coriolis_feedforward=False)
    parameters = {'specification': specification}
    simulation = CausalSimulation(model, control, lambda t: dict(q=[.2], qd=[0.], qdd=[0.]),
        reference_source=dict(source_files=sources, task_parameters=parameters))
    trace = simulation.run(.2, output_hz=100)
    trace.update(task_parameters=parameters, physical_source=model.to_source())
    trace['requirements'] = requirements(trace, parameters)
    assert trace['requirements']['passed']
    report = dict(schema='cybrgeo-engineering-result/2', name='fixture', passed=True,
        source_files=sources, model_sha256=trace['model_sha256'], control_sha256=trace['control_sha256'],
        native_build_sha256=digest(receipt), native_source_binding_sha256=digest(binding),
        simulation={key: value for key, value in trace.items()
                    if key not in ('frames', 'task_parameters', 'physical_source', 'requirements')},
        requirements=trace['requirements'], qualification={'hardware': 'not qualified'})
    for filename, value in [('engineering.json', report), ('physical-source.json', model.to_source()),
                            ('control-source.json', control), ('simulation/trace.json.gz', trace),
                            ('design-source.json', dict(source_files=sources, specification=specification))]:
        save(root / filename, value)
    monkeypatch.setattr(media, 'load_project', lambda unused: (project, sources))
    return dict(source=source, root=root, project=project, trace=trace, report=report)


def load(fixture):
    return media.load_run(fixture['source'], fixture['root'])


def test_actual_native_motor_run_loads_with_complete_pose_proof(accepted):
    run = load(accepted)
    assert run.replay_check['passed']
    assert run.replay_check['recorded_frames'] == 21
    assert run.replay_check['body_matrices_checked'] == 21
    assert run.replay_check['max_matrix_component_error'] == 0
    assert run.replay_check['analytical_parts_valid'] == 1
    assert run.observed_services == {'native_replay.build_assembly': 1,
                                     'physical_replay.build_assembly': 1}
    part = run.assembly.parts[0]
    np.testing.assert_array_equal(run.assembly.pose(part, .015), run.assembly.pose(part, .01))
    assert not np.array_equal(run.assembly.pose(part, 0), run.assembly.pose(part, .2))


@pytest.mark.parametrize('duration,fps', [(.2, 24), (8., 24), (17.4, 24), (1.1, 30)])
def test_schedule_preserves_actual_time_and_includes_final_state(duration, fps):
    times = np.linspace(.3, .3 + duration, 1201).tolist()
    rows = media.frame_schedule(times, fps)
    assert rows[0]['recorded_frame_index'] == 0
    assert rows[-1]['recorded_frame_index'] == 1200
    assert rows[-1]['recorded_time_s'] == times[-1]
    assert duration < len(rows) / fps <= duration + 2 / fps + 1e-12
    for row in rows:
        assert row['recorded_time_s'] <= row['requested_source_time_s']
        assert row['recorded_time_s'] in times
        assert row['hold_age_s'] <= duration / 1200 + 1e-12


@pytest.mark.parametrize('times,fps', [([0], 24), ([0, 0], 24), ([1, 0], 24),
    ([0, float('nan')], 24), ([0, 1], 0), ([0, 1], True), ([0, 1], 24.5)])
def test_schedule_rejects_invalid_sampling(times, fps):
    with pytest.raises(ValueError):
        media.frame_schedule(times, fps)


def test_report_cannot_relabel_different_simulation_metrics(accepted):
    accepted['report']['simulation']['metrics']['max_external_body_force_N'] = 99
    save(accepted['root'] / 'engineering.json', accepted['report'])
    with pytest.raises(ValueError, match='trace summary'):
        load(accepted)


def test_source_bound_design_envelope_cannot_be_relabelled_for_replay(accepted):
    from mechanism_lab.engineering_pipeline import _native_design_screens

    root = accepted['root']
    source = accepted['source']
    control = json.loads((root / 'control-source.json').read_text())
    receipt = verify_native_build(root / 'native', accepted['project'].specification,
        source_files=project_source_identity(source))
    screen = _native_design_screens(root / 'native', receipt,
        project_source_identity(source), control, accepted['trace'])
    accepted['report']['design_envelope'] = screen
    accepted['report']['design_envelope_sha256'] = digest(screen)
    save(root / 'engineering.json', accepted['report'])
    save(root / 'design-envelope.json', screen)
    assert load(accepted).report['design_envelope'] == screen
    screen['drives']['motor']['observed_simulated_peak_Nm'] = 0.
    save(root / 'design-envelope.json', screen)
    with pytest.raises(ValueError, match='design budget'):
        load(accepted)


def test_stale_runtime_cannot_be_relabelled_current(accepted):
    trace = accepted['trace']
    trace['runtime_implementation']['source_files']['causal_runtime.py'] = 'stale'
    accepted['report']['simulation']['runtime_implementation'] = deepcopy(trace['runtime_implementation'])
    save(accepted['root'] / 'engineering.json', accepted['report'])
    save(accepted['root'] / 'simulation/trace.json.gz', trace)
    with pytest.raises(ValueError, match='another runtime'):
        load(accepted)


def test_changed_authored_specification_rejects_media(accepted):
    accepted['project'].specification = {'size_mm': [40, 6, 6], 'density_kg_m3': 2700}
    with pytest.raises(ValueError, match='current authored project'):
        load(accepted)


def test_changed_control_source_rejects_media(accepted):
    save(accepted['root'] / 'control-source.json', {'changed': True})
    with pytest.raises(ValueError, match='control_sha256'):
        load(accepted)


def test_failing_current_requirements_reject_media(accepted):
    accepted['project'].requirements = lambda *unused: {'passed': False}
    with pytest.raises(ValueError, match='Current project requirements'):
        load(accepted)


def test_pose_callback_cannot_override_integrated_motion(accepted):
    replay = accepted['project'].replay
    def changed(directory, trace):
        assembly = replay(directory, trace)
        assembly.motion_function = lambda part, time, explosion: np.eye(4)
        return assembly
    accepted['project'].replay = changed
    with pytest.raises(ValueError, match='pose callback'):
        load(accepted)


def test_loaded_run_rejects_mid_render_input_changes(accepted):
    run = load(accepted)
    (accepted['root'] / 'control-source.json').write_text('{}')
    with pytest.raises(ValueError, match='changed after loading'):
        run.verify_unchanged()


def test_loaded_run_rejects_mid_render_source_changes(accepted):
    run = load(accepted)
    (accepted['source'] / 'project.py').write_text('# changed\n')
    with pytest.raises(ValueError, match='source changed after load'):
        run.verify_unchanged()


def test_render_rejects_hidden_or_exploded_parts_before_output(accepted, tmp_path):
    run = load(accepted)
    run.assembly.views['hero'].explode = 1
    with pytest.raises(ValueError, match='all parts'):
        media.render_run(run, tmp_path / 'media', video=False)
    assert not (tmp_path / 'media').exists()


@pytest.mark.parametrize('mutation', ['vertices', 'normals', 'material', 'partlist', 'motion', 'transforms'])
def test_loaded_run_rejects_in_memory_render_mutations(accepted, mutation):
    run = load(accepted)
    assembly = run.assembly
    if mutation == 'vertices':
        assembly.parts[0].vertices[0, 0] += 1
    elif mutation == 'normals':
        assembly.parts[0].normals[0, 0] += .1
    elif mutation == 'material':
        assembly.materials[0] = Material('Changed appearance', (1., 0., 0.))
    elif mutation == 'partlist':
        assembly.parts.clear()
    elif mutation == 'motion':
        assembly.motion_function = lambda *unused: np.eye(4)
    else:
        run.recorded_motion.transforms['bar'][0, 0, 3] = 99
    with pytest.raises(ValueError, match='In-memory'):
        run.verify_unchanged()


def test_client_cannot_mutate_shared_geometry_before_returning(accepted):
    replay = accepted['project'].replay
    def changed(directory, trace):
        assembly = replay(directory, trace)
        assembly.parts[0].vertices[0, 0] += 1
        return assembly
    accepted['project'].replay = changed
    with pytest.raises(ValueError, match='unchanged geometry/materials'):
        load(accepted)


def test_cli_and_module_share_defaults_and_dispatch(monkeypatch):
    import argparse
    from mechanism_lab import cli
    arguments = ['fixture', '--run', 'saved/run', '--out', 'saved/media']
    standalone = vars(media.add_arguments(argparse.ArgumentParser()).parse_args(arguments))
    parsed = cli.parser().parse_args(['replay'] + arguments)
    assert {key: value for key, value in vars(parsed).items() if key not in ('root', 'command')} == standalone
    assert tuple(parsed.size) == (1920, 1080)
    assert parsed.fps == 24 and parsed.supersample == 2
    calls = []
    monkeypatch.setattr(media, 'run_arguments', calls.append)
    assert cli.run(parsed) == 0
    assert calls == [parsed]
