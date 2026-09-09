from pathlib import Path
import json
import xml.etree.ElementTree as ET
import numpy as np
import pytest


def test_whiteprint_outputs_have_real_vectors_and_mm_layers(flange,tmp_path):
    from mechanism_lab.whiteprint import whiteprint
    import ezdxf
    report=whiteprint(flange,tmp_path/'flange')
    assert report['method']=='OCP analytic hidden-line removal'
    svg=ET.parse(tmp_path/'flange.svg').getroot()
    assert svg.attrib['width']=='420mm' and svg.attrib['height']=='297mm'
    assert len(svg.findall('{http://www.w3.org/2000/svg}polyline'))>20
    assert not svg.findall('.//{http://www.w3.org/2000/svg}image')
    doc=ezdxf.readfile(tmp_path/'flange.dxf');assert doc.units==4
    assert all(name in doc.layers for name in ['VISIBLE','HIDDEN','CENTER','DIM','BORDER'])
    assert (tmp_path/'flange.pdf').read_bytes().startswith(b'%PDF')


def test_mesh_whiteprint_fallback_label_is_honest(flange,tmp_path):
    from dataclasses import replace
    from mechanism_lab.whiteprint import whiteprint
    a=replace(flange,parts=[replace(p,cad=None) for p in flange.parts])
    report=whiteprint(a,tmp_path/'mesh_flange')
    assert 'without hidden removal' in report['method']
    assert 'OCCLUSION NOT SUPPRESSED' in (tmp_path/'mesh_flange.svg').read_text()


@pytest.mark.render
def test_new_geometry_video_and_gif(flange,tmp_path):
    from mechanism_lab.media import render_video,Shot,make_gif
    from PIL import Image
    video=tmp_path/'example.mp4'
    result=render_video(flange,video,[Shot('hero',.5,'orbit',30)],size=(320,240),fps=12)
    assert result['frames']==6 and result['unique_frames']==6
    gif=make_gif(video,tmp_path/'example.gif',width=240,fps=10,seconds=.5)
    with Image.open(gif) as im:assert im.n_frames>=4


def test_release_readme_relative_links(root):
    import re
    text=(root/'README.md').read_text()
    for target in re.findall(r'!?\[[^\]]*\]\(([^)]+)\)',text):
        if '://' in target or target.startswith('#'):continue
        assert (root/target.split('#')[0]).exists(),f'Missing README asset: {target}'


def test_no_missing_individual_motor_parts(root,motor):
    catalogue=root/'outputs/m8325s/parts/index.json'
    if not catalogue.exists():pytest.skip('Full-release catalogue not included')
    items=json.loads(catalogue.read_text())
    assert len(items)==len(motor.parts)==181
    assert {q['name'] for q in items}=={p.name for p in motor.parts}
    for item in items:
        assert (catalogue.parent/item['image']).exists()
        assert (catalogue.parent/item['geometry']).exists()


def test_no_font_or_oversized_git_assets(root):
    from mechanism_lab.packaging import release_files
    for file in release_files(root):
        assert file.suffix.lower() not in {'.ttf','.otf','.woff','.woff2'}
        assert file.stat().st_size<100*1024*1024,f'Asset exceeds normal GitHub blob limit: {file}'


def test_drawing_rejects_off_sheet_annotations(flange,tmp_path):
    from mechanism_lab.whiteprint import whiteprint
    annotation={'annotations':[{'kind':'leader','view':'end','point':[1000,0],
                  'elbow_paper_mm':[10,10],'end_paper_mm':[20,10],'text':'Wrong datum'}]}
    with pytest.raises(ValueError,match='leaves the sheet'):
        whiteprint(flange,tmp_path/'invalid',annotation_spec=annotation)


def test_drawing_centered_anchor_origin_for_translated_part(flange,tmp_path):
    from dataclasses import replace
    from mechanism_lab.whiteprint import whiteprint
    moved=replace(flange,parts=[p.moved([0,-160,0]) for p in flange.parts])
    specification={'anchor_origins':{'end':[-160,0]},'annotations':[
        {'kind':'circle','view':'end','radius':27},
        {'kind':'leader','view':'end','point':[27,0],'elbow_paper_mm':[12,-12],
         'end_paper_mm':[20,-12],'text':'Correct translated origin'}]}
    report=whiteprint(moved,tmp_path/'translated',annotation_spec=specification)
    assert report['annotations']['anchor_origins']['end']==[-160,0]
