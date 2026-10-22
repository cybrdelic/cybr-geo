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
    import mechanism_lab.preview as preview
    import mechanism_lab.render as render
    from PIL import Image
    monkeypatch.setenv('MECHANISM_LAB_ROOT',str(tmp_path))
    monkeypatch.setattr(preview, 'native_unavailable_reason', lambda: 'No C++17/OpenMP compiler in this test')
    output=tmp_path/'result'
    argv=['preview','example_flange','--step','--size','64x64','--spp','2','--threads','2','--out',str(output)]
    assert main(argv)==0
    report=json.loads((output/'preview.json').read_text())
    assert report['passed'] and report['resolution']==[64,64]
    assert report['backend']=='vtk-pbr' and report['requested_renderer']=='auto'
    assert report['fallback_reason']=='No C++17/OpenMP compiler in this test'
    assert report['spp'] is None and report['supersample']==2
    assert report['render_window'].endswith('RenderWindow')
    assert (output/'example_flange.step').read_text().startswith('ISO-10303-21')
    with Image.open(output/'hero.png') as image:
        assert image.size==(64,64)
        assert any(low < high for low,high in image.convert('RGB').getextrema())
    def failure(*args,**kwargs):raise RuntimeError('intentional render failure')
    monkeypatch.setattr(render,'render_still',failure)
    assert main(argv)==2
    assert not (output/'preview.json').exists()


def test_explicit_native_does_not_fall_back_or_leave_old_success(tmp_path,monkeypatch):
    import mechanism_lab.preview as preview
    monkeypatch.setenv('MECHANISM_LAB_ROOT',str(tmp_path))
    monkeypatch.setattr(preview, 'native_unavailable_reason', lambda: 'OpenMP unavailable')
    output=tmp_path/'result';output.mkdir()
    (output/'preview.json').write_text('{"passed": true}')
    assert main(['preview','example_flange','--renderer','native','--out',str(output)])==2
    assert not (output/'preview.json').exists()
    assert not (output/'hero.png').exists()


def test_explicit_pbr_does_not_probe_native_tools(monkeypatch):
    import mechanism_lab.preview as preview
    def unexpected():raise AssertionError('PBR requested a native compiler probe')
    monkeypatch.setattr(preview,'native_unavailable_reason',unexpected)
    assert preview.select_renderer('pbr')==('pbr',None)


def test_compiler_name_without_openmp_is_not_native_support(tmp_path,monkeypatch):
    from types import SimpleNamespace
    import mechanism_lab.preview as preview
    monkeypatch.setattr(preview.shutil,'which',lambda name:'compiler')
    monkeypatch.setattr(preview.subprocess,'run',lambda *args,**kwargs:SimpleNamespace(returncode=1))
    assert 'cannot compile a C++17/OpenMP program' in preview.native_unavailable_reason()


def test_native_render_failure_is_not_replaced_with_raster_output(tmp_path,monkeypatch):
    import mechanism_lab.preview as preview
    import mechanism_lab.photoreal as photo
    monkeypatch.setenv('MECHANISM_LAB_ROOT',str(tmp_path))
    monkeypatch.setattr(preview,'native_unavailable_reason',lambda:None)
    def failure(*args,**kwargs):raise RuntimeError('native rendering failure after capability check')
    monkeypatch.setattr(photo,'render_photoreal',failure)
    output=tmp_path/'result'
    assert main(['preview','example_flange','--out',str(output)])==2
    assert not (output/'preview.json').exists()
    assert not (output/'hero.png').exists()


def test_preview_uses_canonical_engineering_name_for_default_artifacts(tmp_path,monkeypatch):
    import mechanism_lab.preview as preview
    import mechanism_lab.photoreal as photo
    monkeypatch.setenv('MECHANISM_LAB_ROOT',str(tmp_path))
    original=load('example_flange',analytic=True)
    assembly=replace(original,name='ARX belt / integrated native geometry',
                     metadata={**original.metadata,'engineering_result':{'name':'arx_belt'}})
    monkeypatch.setattr(preview,'load',lambda *args,**kwargs:assembly)
    monkeypatch.setattr(preview,'native_unavailable_reason',lambda:None)
    def stop_render(*args,**kwargs):raise RuntimeError('Stop after actual geometry export')
    monkeypatch.setattr(photo,'render_photoreal',stop_render)
    assert main(['preview','example_flange','--step'])==2
    output=tmp_path/'outputs'/'arx_belt'/'preview'
    assert (output/'arx_belt.glb').stat().st_size > 100
    assert (output/'arx_belt.step').read_text().startswith('ISO-10303-21')
    assert not (tmp_path/'outputs'/'ARX belt ').exists()
    assert not (output/'preview.json').exists()


def test_native_compiler_cache_separates_platforms_and_uses_windows_executable_suffix(tmp_path,monkeypatch):
    import mechanism_lab.photoreal as photo
    monkeypatch.setenv('CYBR_GEO_NATIVE_CACHE',str(tmp_path/'native-cache'))
    monkeypatch.delenv('CYBR_GEO_EMBREE_ROOT',raising=False)
    calls=[]
    def compile_output(command,**kwargs):
        destination=Path(command[command.index('-o')+1])
        destination.write_bytes(b'compiler output fixture')
        calls.append(destination)
    monkeypatch.setattr(photo.subprocess,'run',compile_output)
    monkeypatch.setattr(photo.platform,'system',lambda:'Windows')
    monkeypatch.setattr(photo.platform,'machine',lambda:'AMD64')
    windows=photo.compile_renderer()
    assert windows.suffix=='.exe' and calls[-1].suffix=='.exe'
    assert photo.compile_renderer()==windows and len(calls)==1
    monkeypatch.setattr(photo.platform,'system',lambda:'Linux')
    linux=photo.compile_renderer()
    assert linux.suffix=='' and linux.parent != windows.parent
    assert len(calls)==2 and linux.read_bytes()==windows.read_bytes()


def test_unknown_view_has_no_published_output(tmp_path,monkeypatch):
    monkeypatch.setenv('MECHANISM_LAB_ROOT',str(tmp_path))
    output=tmp_path/'result'
    assert main(['preview','example_flange','--view','missing','--out',str(output)])==2
    assert not output.exists()


def test_build_publishes_geometry_report(tmp_path,monkeypatch):
    monkeypatch.setenv('MECHANISM_LAB_ROOT',str(tmp_path))
    output=tmp_path/'result'
    assert main(['build','example_flange','--out',str(output)])==0
    report=json.loads((output/'validation.json').read_text())
    assert report['model']=='example_flange' and report['parts']==2
