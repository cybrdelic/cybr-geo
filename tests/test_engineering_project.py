"""Authored project boundaries must invalidate stale accepted artifacts."""
from types import SimpleNamespace

import pytest

from mechanism_lab import engineering_pipeline, engineering_source


def test_failed_native_reuse_invalidates_prior_accepted_outputs(tmp_path, monkeypatch):
    for name in ('engineering.json', 'assembly.glb', 'diagnostic-assembly.glb', 'fixture.step'):
        (tmp_path/name).write_text('old accepted output')
    (tmp_path/'unrelated.step').write_text('keep user artifact')
    project = SimpleNamespace(name='fixture', specification={'dimension_mm': 5})
    monkeypatch.setattr(engineering_source, 'load_project', lambda source: (project, {}))
    def reject(*args, **kwargs):
        raise ValueError('Native build belongs to a different design specification')
    monkeypatch.setattr(engineering_source, 'verify_native_build', reject)
    with pytest.raises(ValueError, match='different design specification'):
        engineering_pipeline.generate_project('fixture', tmp_path, reuse_native=tmp_path/'old-native')
    assert not (tmp_path/'assembly.glb').exists()
    assert not (tmp_path/'diagnostic-assembly.glb').exists()
    assert not (tmp_path/'fixture.step').exists()
    assert not (tmp_path/'engineering.json').exists()
    assert (tmp_path/'unrelated.step').read_text() == 'keep user artifact'


def test_native_receipt_binds_current_construction_code(tmp_path):
    import hashlib
    import json
    from pathlib import Path
    from mechanism_lab.physical_model import digest
    specification = {'dimension_mm': 5}
    receipt = dict(schema='cybrgeo-native-build/1', provenance=dict(
        input_spec_sha256=digest(specification), adapted_recipe_sha256='recipe',
        native_construction_sha256='old implementation'), files_sha256={})
    (tmp_path/'native-build.json').write_text(json.dumps(receipt))
    with pytest.raises(ValueError, match='construction implementation'):
        engineering_source.verify_native_build(tmp_path, specification, recipe_sha256='recipe')
    receipt['provenance']['native_construction_sha256'] = hashlib.sha256(
        Path(engineering_source.__file__).with_name('native_construction.py').read_bytes()).hexdigest()
    (tmp_path/'native-build.json').write_text(json.dumps(receipt))
    assert engineering_source.verify_native_build(tmp_path, specification, recipe_sha256='recipe') == receipt


def test_authored_project_descriptor_is_a_supported_cache_identity():
    from mechanism_lab.registry import factory, fingerprint
    directory = engineering_source.project_directory('arx_belt')
    assert len(fingerprint(str(directory/'project.json'))) == 64
    build, motion = factory(str(directory/'project.json'))
    assert callable(build) and callable(motion.bind)
