"""Fresh printer numerics and current-renderer wiring, not physical qualification."""
from __future__ import annotations
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

EXAMPLE = Path(__file__).resolve().parents[1] / 'examples' / 'fuse_c220'
sys.path.insert(0, str(EXAMPLE))
import printer
import toolpath
import verify_mechanism
import verify_edges

spec = importlib.util.spec_from_file_location('fuse_recovery_renderer', EXAMPLE / 'render_delivery.py')
render = importlib.util.module_from_spec(spec)
spec.loader.exec_module(render)


@pytest.fixture(scope='module')
def assembly():
    return printer.build()


def test_fresh_nominal_geometry_and_kinematics(assembly):
    report = verify_mechanism.verify(assembly)
    assert report['all_passed'], report
    assert len(report['checks']) == 13
    assert all(item['passed'] for item in report['checks'])


@pytest.mark.parametrize('coordinates', [(111, 0, 0), (0, -111, 1), (0, 0, 221), (float('nan'), 0, 0)])
def test_adversarial_coordinates_are_rejected(coordinates):
    with pytest.raises(ValueError):
        printer.State(*coordinates)


def test_fresh_toolpath_volume_and_coordinate_checks(tmp_path):
    result = toolpath.verify(toolpath.generate(tmp_path / 'calibration.gcode'))
    assert result['all_passed'], result
    assert len(result['checks']) == 6
    assert result['extrusion_moves'] > 50000


def test_independent_cached_edge_checks(assembly, tmp_path, monkeypatch):
    from mechanism_lab.core import save_cache
    monkeypatch.chdir(tmp_path)
    Path('deliverables').mkdir()
    save_cache(assembly, Path('deliverables/cache'))
    verify_edges.main()
    result = json.loads(Path('deliverables/FUSE_C220_edge_validation.json').read_text())
    assert result['all_passed'] and len(result['checks']) == 9


def test_stills_call_current_v9_dispatch(monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(render, 'OUT', tmp_path)
    monkeypatch.setattr(render, 'WORK', tmp_path)
    monkeypatch.setattr(render, 'posed', lambda a, state, geometry: a)
    monkeypatch.setattr(render, 'render_v9', lambda a, p, **kwargs: calls.append(kwargs))
    tp = SimpleNamespace(deposition_end=100, state=lambda t: (None, 0, 0), geometry=lambda t: [])
    render.stills(object(), tp, preview=False)
    assert len(calls) == 3
    assert all(c['depth'] == render.V9.still_depth and c['spp'] == render.V9.still_spp for c in calls)
    assert [c['view_name'] for c in calls] == ['hero', 'printing', 'drive']
    assert not hasattr(render, '_invoke')
    assert not hasattr(render, 'render_photoreal')


def test_preview_is_an_explicit_lower_sample_v9_request(monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(render, 'WORK', tmp_path)
    monkeypatch.setattr(render, 'posed', lambda a, state, geometry: a)
    monkeypatch.setattr(render, 'render_v9', lambda a, p, **kwargs: calls.append(kwargs))
    tp = SimpleNamespace(deposition_end=100, state=lambda t: (None, 0, 0), geometry=lambda t: [])
    render.stills(object(), tp, preview=True)
    assert len(calls) == 2
    assert all(c['spp'] == 32 and c['size'] == (960, 720) for c in calls)


def test_schedule_labels_timelapse_and_final_replay():
    tp = SimpleNamespace(deposition_end=200, moves=[SimpleNamespace(deposits=True, t0=3)])
    shots = render.schedule(tp)
    assert len(shots) == 96
    assert all('time-lapse' in label for _, label in shots[:48])
    assert all('replay' in label for _, label in shots[48:])
    assert shots[49][0] - shots[48][0] == pytest.approx(1 / 24)
    assert shots[0][0] == 3 and shots[47][0] == pytest.approx(200)


def test_checkpoints_reject_changed_bytes_or_fingerprint(tmp_path, monkeypatch):
    monkeypatch.setattr(render, 'FRAMES', tmp_path)
    image = tmp_path / '00000.png'
    image.write_bytes(b'checkpoint test bytes, not a rendered image')
    record = {'frame': 0, 'fingerprint': 'current', 'sha256': render.digest(image)}
    (tmp_path / '00000.frame.json').write_text(json.dumps(record))
    (tmp_path / '00000.json').write_text('{}')
    assert render.read_checkpoint(0, 'current') == record
    assert render.read_checkpoint(0, 'old') is None
    image.write_bytes(b'tampered')
    assert render.read_checkpoint(0, 'current') is None


def test_encoding_refuses_missing_frames(tmp_path, monkeypatch):
    monkeypatch.setattr(render, 'FRAMES', tmp_path)
    with pytest.raises(RuntimeError, match='all 96'):
        render.encode_film('test-identity')
