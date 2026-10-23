"""IRIS keeps diagnostic builds separate from source-provided qualified CAD."""
import json
from pathlib import Path

import pytest

from mechanism_lab.qualification import (
    QualificationError, QualificationIntentPlan, _qualified_body_poses,
)
from examples.engineering.iris09 import mechanism


def test_iris_source_frames_cover_the_exact_intent_body_inventory():
    spec = mechanism.specification()
    intent = QualificationIntentPlan(json.loads(
        (mechanism.HERE / 'data' / 'qualification-intent.json').read_text(encoding='utf-8')))
    frames = {body: frame.tolist() for body, frame in mechanism.layout(spec)['frames'].items()}
    assert _qualified_body_poses(frames, intent.document['parts']) == frames
    assert set(frames) == {'fixed', 'rotor', *(f'leaf_{i:02}' for i in range(9)),
                           *(f'rod_{i:02}' for i in range(9))}


def test_iris_rejects_an_incomplete_resolved_plan_before_cad_export(tmp_path, monkeypatch):
    spec = mechanism.specification()
    root = tmp_path / 'iris'
    (root / 'data').mkdir(parents=True)
    intent = mechanism.HERE / 'data' / 'qualification-intent.json'
    (root / 'data' / 'qualification-intent.json').write_bytes(intent.read_bytes())
    (root / 'data' / 'qualification-plan.json').write_text(
        json.dumps({'schema': 'cybrgeo-qualification-plan/1'}), encoding='utf-8')
    monkeypatch.setattr(mechanism, 'HERE', root)
    output = tmp_path / 'native'
    with pytest.raises(QualificationError, match='Qualification plan fields'):
        mechanism.build(spec, output)
    assert not output.exists()
