import pytest


def test_antialias_modes_and_descriptions():
    from mechanism_lab.render import AA_FACTORS,normalize_aa,antialias_description
    assert AA_FACTORS=={'none':1,'fxaa':1,'ssaa2':2,'ssaa3':3,'ssaa4':4}
    assert normalize_aa('SSAA2')=='ssaa2'
    assert antialias_description('none')=='antialiasing disabled'
    assert antialias_description('fxaa')=='FXAA'
    assert '2x SSAA' in antialias_description('ssaa2')
    assert 'Lanczos' in antialias_description('ssaa4')
    with pytest.raises(ValueError,match='Unknown antialiasing mode'):
        normalize_aa('taa')


def test_cli_exposes_antialiasing_for_geometry_outputs():
    from mechanism_lab.cli import parser
    p=parser()
    render=p.parse_args(['render','example_flange','--aa','ssaa3'])
    video=p.parse_args(['video','example_flange','--aa','ssaa2'])
    catalogue=p.parse_args(['catalogue','example_flange','--aa','fxaa'])
    assert render.aa=='ssaa3'
    assert video.aa=='ssaa2'
    assert catalogue.aa=='fxaa'


def test_pathtrace_metadata_states_integrated_subpixel_aa():
    from pathlib import Path
    text=(Path(__file__).parents[1]/'src/mechanism_lab/pathtrace.py').read_text()
    assert 'stochastic subpixel camera jitter integrated across every path-traced sample' in text
