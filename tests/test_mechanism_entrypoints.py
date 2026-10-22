"""The constructive pipeline is the default, including after a disk cache hit."""
import json

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
