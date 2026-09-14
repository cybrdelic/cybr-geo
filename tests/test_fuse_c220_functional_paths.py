"""Regression cases for real blocked CAD and lost or misread G-code motion."""
from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest

EXAMPLE = Path(__file__).resolve().parents[1] / "examples" / "fuse_c220"
sys.path.insert(0, str(EXAMPLE))
import printer
import toolpath
import verify_hotend


@pytest.fixture(scope="module")
def assembly():
    return printer.build()


def program(tmp_path, text):
    path = tmp_path / "test.gcode"
    path.write_text(text)
    return toolpath.parse(path)


def test_hotend_brep_paths_and_intersections(assembly):
    report = verify_hotend.verify(assembly)
    assert report["all_passed"], report
    assert len(report["checks"]) == 5


def test_feed_check_detects_solid_obstruction(assembly):
    block = next(part for part in assembly.parts if part.name == "heater_block")
    blocked = block.cad.fuse(printer.cylinder(1., 3., (0, 0, printer.BED + 8.)))
    changed = replace(assembly, parts=[
        replace(part, cad=blocked) if part.name == block.name else part
        for part in assembly.parts
    ])
    report = verify_hotend.verify(changed)
    assert not report["all_passed"]
    assert not report["checks"][0]["passed"]


def test_extrusion_only_retraction_keeps_time_and_shaft_motion(tmp_path):
    tp = program(tmp_path, "M109 S210\nM83\nG1 E-2 F120\nG1 E2\n")
    assert len(tp.moves) == 2 and not tp.deposits
    assert tp.total > 2.
    state, _, fraction = tp.state(tp.moves[0].duration / 2)
    assert state.e == pytest.approx(-1)
    assert fraction == pytest.approx(.5)
    assert state.x == -110 and state.y == -110
    assert tp.geometry(tp.total) == []


def test_g92_does_not_rewind_physical_feed(tmp_path):
    tp = program(tmp_path, "M109 S210\nM82\nG1 E5 F600\nG92 E0\nG1 E1\n")
    assert [(m.e0, m.e1) for m in tp.moves] == [(0., 5.), (5., 6.)]
    assert tp.state(tp.total)[0].e == pytest.approx(6.)


def test_compact_signed_and_fractional_coordinates(tmp_path):
    tp = program(tmp_path, "G21\nG90\nG0X+1.Y.5Z.2F600\nG91\nG0X.5Y-.25\n")
    np.testing.assert_allclose(tp.moves[-1].end, [1.5, .25, .2])


def test_klipper_relative_extrusion_modes(tmp_path):
    tp = program(tmp_path, "M109 S210\nG90\nM82\nG1 E2 F600\nG91\nG1 E1\nG90\nM83\nG1 E1\n")
    assert [m.e1 for m in tp.moves] == [2., 3., 4.]


@pytest.mark.parametrize("bad", [
    "G0 Xnan", "G0 Xinf", "G0 X1.2.3", "G0 X1 X2", "G0 X221",
    "G0 X-1", "G0 F0", "G0 F-1", "G2 X1 I2", "G20", "G92 X50",
    "M104 S210\nG1 E1", "M109 S210\nM104 S0\nG1 E1",
    "G0 X1\nG28", "M84\nG0 X1", "G0 X1 ;valid first\nM999",
    "M109 S210\nG1 X1 Z.2 E1", "G0 X1Q7", "M109",
])
def test_invalid_or_unmodeled_programs_fail_closed(tmp_path, bad):
    with pytest.raises(ValueError):
        program(tmp_path, bad + "\n")


def test_changed_temperature_requires_another_wait(tmp_path):
    with pytest.raises(ValueError, match="M109"):
        program(tmp_path, "M109 S210\nM104 S220\nG1 E1\n")


def test_boundary_does_not_duplicate_completed_bead(tmp_path):
    tp = program(tmp_path, "M109 S210\nG0 X1 Y1 Z.2 F600\nG1 X2 E.03\n")
    parts = tp.geometry(tp.total)
    assert [part.name for part in parts] == ["printed_extrusion_beads"]
    halfway = tp.geometry(tp.moves[-1].t0 + tp.moves[-1].duration / 2)
    assert [part.name for part in halfway] == ["current_extrusion_bead"]


def test_travel_only_and_empty_programs(tmp_path):
    tp = program(tmp_path, "G0 X1 F600\n")
    assert tp.deposition_end == 0 and tp.geometry(tp.total) == []
    with pytest.raises(ValueError, match="no motion"):
        program(tmp_path, "G21\n")


@pytest.mark.parametrize("value", [float("nan"), float("inf")])
def test_nonfinite_time_is_rejected(tmp_path, value):
    tp = program(tmp_path, "G0 X1 F600\n")
    with pytest.raises(ValueError, match="finite"):
        tp.state(value)


@pytest.mark.parametrize("layers", [0, -1, 1076, 2.5, True])
def test_bad_layer_count_is_rejected(tmp_path, layers):
    with pytest.raises(ValueError):
        toolpath.generate(tmp_path / "bad.gcode", layers)


def test_nondefault_layer_count_and_park_height(tmp_path):
    tp = toolpath.generate(tmp_path / "short.gcode", layers=5)
    assert toolpath.verify(tp)["layers"] == 5
    assert tp.moves[-1].end[2] > max(m.end[2] for m in tp.deposits)
