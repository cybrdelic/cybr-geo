"""Engineering native reuse binds the source closure that actually constructed it."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

from mechanism_lab import engineering_source as source
from mechanism_lab.physical_model import PhysicalModel, digest


def authored_project(root):
    (root / 'data').mkdir(parents=True)
    (root / 'helpers').mkdir()
    (root / 'project.json').write_text('{"schema":"cybrgeo-engineering-project/1"}')
    (root / 'data/dimensions.json').write_text('{"offset_mm":4}')
    (root / 'helpers/shape.py').write_text('DIMENSION_MM = 12\n')
    # No design.py: the public callback can live in any captured module.
    (root / 'project.py').write_text(
        'from mechanism_lab.engineering_source import EngineeringProject\n'
        'from .fabrication import construct\n'
        'def create():\n'
        ' return EngineeringProject("fixture", {"scale":1}, construct, None, None, None, None)\n')
    (root / 'fabrication.py').write_text(
        'import hashlib, json\nfrom pathlib import Path\n'
        'from mechanism_lab import native_construction\n'
        'from mechanism_lab.physical_model import digest\n'
        'def construct(specification, directory):\n'
        ' from .helpers.shape import DIMENSION_MM\n'
        ' data=json.loads((Path(__file__).parent/"data/dimensions.json").read_text())\n'
        ' output=Path(directory); output.mkdir(parents=True, exist_ok=True)\n'
        ' artifact=output/"geometry.json"\n'
        ' artifact.write_text(json.dumps({"dimension_mm":DIMENSION_MM+data["offset_mm"]}))\n'
        ' receipt=dict(schema="cybrgeo-native-build/1", parts=1, placements=1,\n'
        '  provenance=dict(input_spec_sha256=digest(specification),\n'
        '   native_construction_sha256=hashlib.sha256(Path(native_construction.__file__).read_bytes()).hexdigest()),\n'
        '  files_sha256={"geometry.json":hashlib.sha256(artifact.read_bytes()).hexdigest()})\n'
        ' (output/"native-build.json").write_text(json.dumps(receipt))\n')
    return root


def freshly_construct(tmp_path):
    project_path = authored_project(tmp_path / 'project')
    project, identity = source.load_project(project_path)
    native = tmp_path / 'native'
    receipt, binding = source.construct_project_native(project, project_path, identity, native)
    return project_path, project, identity, native, receipt, binding


def test_fresh_and_unchanged_reuse_bind_constructor_outside_design_py(tmp_path):
    path, project, identity, native, receipt, binding = freshly_construct(tmp_path)
    assert 'design.py' not in identity
    assert 'fabrication.py' in identity and 'helpers/shape.py' in identity
    assert json.loads((native / 'geometry.json').read_text()) == {'dimension_mm': 16}
    assert binding['source_files'] == identity
    assert binding['native_build_sha256'] == digest(receipt)
    before = (native / 'native-build.json').read_bytes()
    assert source.verify_native_build(native, project.specification, source_files=identity) == receipt
    assert source.verify_native_source_binding(native, project.specification, identity, receipt=receipt) == binding
    assert (native / 'native-build.json').read_bytes() == before


@pytest.mark.parametrize('relative,contents', [
    ('helpers/shape.py', 'DIMENSION_MM = 30\n'),
    ('data/dimensions.json', '{"offset_mm":50}'),
    ('helpers/shape.py', None),
    ('data/dimensions.json', None),
    ('helpers/new_geometry.py', 'VALUE = 1\n'),
    ('project.py', '# controller-only change is conservatively bound\n'),
])
def test_changed_or_missing_project_inputs_reject_native_reuse(tmp_path, relative, contents):
    path, project, identity, native, _, _ = freshly_construct(tmp_path)
    changed = path / relative
    if contents is None:
        changed.unlink()
    else:
        changed.write_text(contents)
    updated = source.project_source_identity(path)
    assert updated != identity
    with pytest.raises(ValueError, match='complete project source/data closure'):
        source.verify_native_build(native, project.specification, source_files=updated)


def test_old_build_cannot_be_retroactively_bound_by_a_noop_constructor(tmp_path):
    path, project, identity, native, _, _ = freshly_construct(tmp_path)
    (native / source.NATIVE_SOURCE_BINDING).unlink()
    with pytest.raises(ValueError, match='binding is missing'):
        source.verify_native_build(native, project.specification, source_files=identity)
    project.construct = lambda *args: None
    with pytest.raises(FileNotFoundError):
        source.construct_project_native(project, path, identity, native)
    assert not (native / source.NATIVE_SOURCE_BINDING).exists()
    assert not (native / 'native-build.json').exists()


def test_failed_construction_clears_old_completion_and_binding(tmp_path):
    path, project, identity, native, _, _ = freshly_construct(tmp_path)
    original = project.construct
    def fail(*args):
        original(*args)
        raise RuntimeError('construction interrupted after partial exports')
    project.construct = fail
    with pytest.raises(RuntimeError, match='interrupted'):
        source.construct_project_native(project, path, identity, native)
    assert not (native / source.NATIVE_SOURCE_BINDING).exists()
    assert not (native / 'native-build.json').exists()


def test_stale_snapshot_before_construction_clears_old_completion_and_binding(tmp_path):
    path, project, identity, native, _, _ = freshly_construct(tmp_path)
    (path / 'helpers/shape.py').write_text('DIMENSION_MM = 99\n')
    with pytest.raises(ValueError, match='source changed after load'):
        source.construct_project_native(project, path, identity, native)
    assert not (native / source.NATIVE_SOURCE_BINDING).exists()
    assert not (native / 'native-build.json').exists()


@pytest.mark.parametrize('changed', ['shared_helper', 'kernel'])
def test_changed_shared_geometry_or_cad_runtime_invalidates_reuse(tmp_path, monkeypatch, changed):
    _, project, identity, native, _, binding = freshly_construct(tmp_path)
    dependencies = binding['construction_dependencies']
    if changed == 'shared_helper':
        dependencies['source_files']['shape_dag.py'] = '0' * 64
    else:
        dependencies['packages']['cadquery-ocp'] = 'other-kernel'
    monkeypatch.setattr(source, 'construction_dependency_identity', lambda: dependencies)
    with pytest.raises(ValueError, match='construction dependencies or CAD runtime'):
        source.verify_native_build(native, project.specification, source_files=identity)


def test_native_receipt_replacement_invalidates_binding(tmp_path):
    _, project, identity, native, receipt, _ = freshly_construct(tmp_path)
    # Semantically equal JSON is still a different original completion file.
    (native / 'native-build.json').write_text(json.dumps(receipt, indent=2))
    with pytest.raises(ValueError, match='different native completion receipt'):
        source.verify_native_build(native, project.specification, source_files=identity)


def test_constructor_source_mutation_does_not_publish_binding(tmp_path):
    path = authored_project(tmp_path / 'project')
    project, identity = source.load_project(path)
    original = project.construct
    def mutate(*args):
        original(*args)
        (path / 'data/dimensions.json').write_text('{"offset_mm":999}')
    project.construct = mutate
    with pytest.raises(ValueError, match='source changed after load'):
        source.construct_project_native(project, path, identity, tmp_path / 'native')
    assert not (tmp_path / 'native' / source.NATIVE_SOURCE_BINDING).exists()
    assert not (tmp_path / 'native' / 'native-build.json').exists()


@pytest.mark.parametrize('mutate_during_simulation', [False, True])
def test_pipeline_fresh_reuse_and_midrun_artifact_acceptance(tmp_path, monkeypatch, mutate_during_simulation):
    from mechanism_lab import causal_runtime, engineering_pipeline, exporters
    path = authored_project(tmp_path / 'project')
    project, identity = source.load_project(path)
    project.compile_physics = lambda *args, **kwargs: (
        PhysicalModel.from_xml('<mujoco><worldbody><body name="fixture"/></worldbody></mujoco>'), {}, {})
    project.reference = lambda *args: {}
    project.requirements = lambda *args: {'passed': True, 'checks': []}
    project.replay = lambda *args: SimpleNamespace(metadata={})
    monkeypatch.setattr(source, 'load_project', lambda _: (project, identity))
    output = tmp_path / 'output'
    class Simulation:
        def __init__(self, *args, **kwargs):
            pass
        def run(self, seconds, *, output):
            output.mkdir(parents=True, exist_ok=True)
            if mutate_during_simulation:
                (output.parent / 'native/geometry.json').write_text('{"dimension_mm":999}')
            return {'frames': [], 'duration_s': seconds}
    monkeypatch.setattr(causal_runtime, 'CausalSimulation', Simulation)
    monkeypatch.setattr(exporters, 'export_glb', lambda assembly, target: target.write_bytes(b'current geometry'))
    if mutate_during_simulation:
        with pytest.raises(ValueError, match='Native artifact differs'):
            engineering_pipeline.generate_project(path, output)
        assert not (output / 'assembly.glb').exists()
        assert not (output / 'engineering.json').exists()
        return
    _, first = engineering_pipeline.generate_project(path, output)
    assert first['passed'] and (output / 'assembly.glb').exists()
    native = output / 'native'
    before = (native / 'native-build.json').read_bytes()
    project.construct = lambda *args: pytest.fail('reuse must not construct again')
    _, second = engineering_pipeline.generate_project(path, tmp_path / 'reused', reuse_native=native)
    assert second['passed'] and second['native_source_binding_sha256'] == first['native_source_binding_sha256']
    assert (native / 'native-build.json').read_bytes() == before


def test_construction_dependency_boundary_includes_shared_shapes_and_kernel():
    identity = source.construction_dependency_identity()
    assert {'native_construction.py', 'shape_dag.py', 'core.py', 'geometry.py', 'physical_model.py'} <= identity['source_files'].keys()
    assert all(len(value) == 64 for value in identity['source_files'].values())
    assert all(identity['packages'][name] for name in ('cadquery', 'cadquery-ocp', 'numpy'))


@pytest.mark.parametrize('change', ['missing_binding', 'changed_helper'])
def test_causal_command_rejects_unbound_or_stale_native_before_model_lowering(tmp_path, monkeypatch, change):
    from tools import run_arx_causal
    path, _, _, native, _, _ = freshly_construct(tmp_path)
    if change == 'missing_binding':
        (native / source.NATIVE_SOURCE_BINDING).unlink()
    else:
        (path / 'helpers/shape.py').write_text('DIMENSION_MM = 30\n')
    monkeypatch.setattr(sys, 'argv', ['run_arx_causal.py', '--project', str(path),
        '--native', str(native), '--output', str(tmp_path / 'causal')])
    # The fixture deliberately has no physics callback. A broken command that
    # skips closure verification would reach lowering and fail differently.
    with pytest.raises(ValueError, match='binding is missing|complete project source/data closure'):
        run_arx_causal.main()
    assert not (tmp_path / 'causal/physical-source.json').exists()
