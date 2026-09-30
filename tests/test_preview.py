"""Public first-run path and failures that previously surfaced after creation."""
from dataclasses import replace
import json
from pathlib import Path

import pytest

from mechanism_lab.cli import main,parser
from mechanism_lab.core import validate
from mechanism_lab.registry import cache_directory,load


@pytest.mark.parametrize('option',['--spp','--depth','--threads'])
def test_bad_budgets_rejected_before_recipe_execution(option):
    with pytest.raises(SystemExit) as exc:
        parser().parse_args(['preview','missing.py',option,'0'])
    assert exc.value.code==2


def test_cached_build_retains_validation_without_repeating_it(tmp_path,monkeypatch):
    import mechanism_lab.registry as registry
    monkeypatch.setenv('MECHANISM_LAB_ROOT',str(tmp_path))
    calls=[];original=registry.validate
    def checked(assembly):
        calls.append(assembly.name);return original(assembly)
    monkeypatch.setattr(registry,'validate',checked)
    load('example_flange');load('example_flange')
    assert calls==['example_flange']
    report=json.loads((cache_directory('example_flange')/'validation.json').read_text())
    assert report['parts']==2 and all(p['analytic_valid'] for p in report['checks'])


def test_empty_and_nonfinite_camera_rejected_during_build(flange):
    with pytest.raises(ValueError,match='no parts'):validate(replace(flange,parts=[]))
    bad=replace(flange.views['hero'],f_stop=float('nan'))
    with pytest.raises(ValueError,match='camera'):validate(replace(flange,views={'hero':bad}))


def test_actual_preview_and_failed_replacement_do_not_share_success(tmp_path,monkeypatch):
    import mechanism_lab.photoreal as photo
    from PIL import Image
    monkeypatch.setenv('MECHANISM_LAB_ROOT',str(tmp_path))
    output=tmp_path/'result'
    argv=['preview','--step','--size','64x64','--spp','2','--threads','2','--out',str(output)]
    assert main(argv)==0
    report=json.loads((output/'preview.json').read_text())
    assert report['passed'] and report['resolution']==[64,64]
    assert (output/'example_flange.step').read_text().startswith('ISO-10303-21')
    with Image.open(output/'hero.png') as image:assert image.size==(64,64)
    def failure(*args,**kwargs):raise RuntimeError('intentional render failure')
    monkeypatch.setattr(photo,'render_photoreal',failure)
    assert main(argv)==2
    assert not (output/'preview.json').exists()


def test_unknown_view_has_no_published_output(tmp_path,monkeypatch):
    monkeypatch.setenv('MECHANISM_LAB_ROOT',str(tmp_path))
    output=tmp_path/'result'
    assert main(['preview','--view','missing','--out',str(output)])==2
    assert not output.exists()


def test_build_publishes_geometry_report(tmp_path,monkeypatch):
    monkeypatch.setenv('MECHANISM_LAB_ROOT',str(tmp_path))
    output=tmp_path/'result'
    assert main(['build','example_flange','--out',str(output)])==0
    report=json.loads((output/'validation.json').read_text())
    assert report['model']=='example_flange' and report['parts']==2
