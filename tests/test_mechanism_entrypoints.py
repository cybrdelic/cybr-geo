"""The constructive pipeline is the default, including after a disk cache hit."""
import json
import pytest

from mechanism_lab.cli import parser
from mechanism_lab.registry import DEFAULT_RECIPE, factory, fingerprint


def test_default_build_and_preview_are_constructive():
    assert DEFAULT_RECIPE == 'compact_arm'
    for command in ('build', 'preview'):
        assert parser().parse_args([command]).recipe == DEFAULT_RECIPE
    assert parser().parse_args(['mechanism']).architecture == DEFAULT_RECIPE
    assert parser().parse_args(['build', 'example_flange']).recipe == 'example_flange'


def test_descriptor_keeps_parameters_and_changes_fingerprint(tmp_path):
    recipe = tmp_path/'arm.json'
    recipe.write_text(json.dumps({'schema':'cybrgeo-mechanism/1',
                                'architecture':'compact_arm','parameters':{'scale':1.0}}))
    first = fingerprint(str(recipe))
    recipe.write_text(json.dumps({'schema':'cybrgeo-mechanism/1',
                                'architecture':'compact_arm','parameters':{'scale':1.2}}))
    assert fingerprint(str(recipe)) != first
    build, motion = factory(str(recipe))
    assert callable(build) and callable(motion.bind)


def test_native_alias_uses_the_canonical_source_model(monkeypatch):
    from types import SimpleNamespace
    from mechanism_lab import constructed
    from mechanism_lab.engineering_pipeline import compile_source
    observed = []
    def build(plan):
        observed.append(plan)
        return SimpleNamespace(name=plan['name'])
    monkeypatch.setattr(constructed, 'build_from_plan', build)
    recipe, _ = factory('compact_arm')
    assert recipe().name == 'compact_arm'
    canonical, _ = compile_source('compact_arm')
    assert observed[0]['construction']['model_sha256'] == canonical['construction']['model_sha256']
    assert observed[0]['construction']['architecture'] == 'component_graph'


def test_constructed_cache_rebuilds_corrupted_mesh_bytes(tmp_path, monkeypatch, flange):
    import mechanism_lab.registry as registry
    from copy import deepcopy
    monkeypatch.setenv('MECHANISM_LAB_ROOT',str(tmp_path))
    calls=[]
    def construct():
        calls.append(True)
        return deepcopy(flange)
    monkeypatch.setattr(registry,'factory',lambda name:(construct,None))
    registry.load('compact_arm')
    cache=registry.cache_directory('compact_arm')
    (cache/'meshes.npz').write_bytes(b'corrupt cache')
    restored=registry.load('compact_arm')
    assert len(calls)==2 and len(restored.parts)==2
    registry.load('compact_arm')
    assert len(calls)==2
    (cache/'construction-cache.json').write_text(json.dumps(['manifest.json','meshes.npz']))
    registry.load('compact_arm')
    assert len(calls)==3


@pytest.mark.parametrize('command', ['build', 'animate'])
def test_native_commands_refuse_failed_engineering(tmp_path, monkeypatch, flange, command):
    from copy import deepcopy
    from mechanism_lab import cli, registry, engineering_pipeline, exporters
    assembly = deepcopy(flange)
    assembly.metadata['construction_recipe'] = {'test': 'shared-source'}
    monkeypatch.setenv('MECHANISM_LAB_ROOT', str(tmp_path))
    loaded = []
    def load(name, **kwargs):
        loaded.append(kwargs)
        return assembly
    monkeypatch.setattr(registry, 'load', load)
    monkeypatch.setattr(engineering_pipeline, 'evaluate_plan', lambda plan, *args, **kwargs:
        (assembly, {'passed': False, 'errors': [{'stage':'simulation', 'message':'Actuator speed exceeded'}]}))
    def unexpected(*args, **kwargs):
        raise AssertionError('An uncertified export was published')
    monkeypatch.setattr(exporters, 'export_glb', unexpected)
    monkeypatch.setattr(exporters, 'export_animated_glb', unexpected)
    args = cli.parser().parse_args([command, 'compact_arm'])
    with pytest.raises(RuntimeError, match='Actuator speed exceeded'):
        cli.run(args)
    assert loaded == [{'rebuild': False, 'analytic': True}]
