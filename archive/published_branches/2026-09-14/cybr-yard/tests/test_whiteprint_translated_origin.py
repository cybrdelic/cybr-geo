"""Drawing dimensions must not depend on the assembly's absolute origin."""
from dataclasses import replace
import numpy as np
from mechanism_lab.models.example import build
from mechanism_lab.whiteprint import Sheet,whiteprint


def test_translated_model_keeps_dimensions_above_the_title_block(tmp_path,monkeypatch):
    a=build()
    a=replace(a,parts=[p.moved((0,0,200)) for p in a.parts])
    captured=[]
    def inspect_sheet(sheet,path):
        captured.extend(points for layer,points in sheet.dxf_lines if layer=='DIM')
    monkeypatch.setattr(Sheet,'save',inspect_sheet)
    whiteprint(a,tmp_path/'translated')
    assert captured
    points=np.concatenate(captured)
    assert points[:,1].min()>27
    assert points[:,1].max()<256
