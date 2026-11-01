"""Retain the actual P5 JavaScript task oracle when authoring a new policy."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

import numpy as np
import pytest

from examples.engineering.arx_belt.model import compile_model
from examples.engineering.arx_belt.task import HALF_CYCLE_BOUNDARIES_S, reference


FIXTURES = Path(__file__).parent / 'fixtures' / 'arx_actuation'


def test_source_default_task_matches_original_javascript():
    assert hashlib.sha256((FIXTURES / 'source-reference.mjs').read_bytes()).hexdigest() == (
        '609ecfefebd8fd77d24a095008f3b2b7b7ee1a7c4c78fe949b64f16209e97886')
    node = shutil.which('node')
    if node is None:
        pytest.skip('Node is required to execute the published ARX task oracle')
    _, _, parameters = compile_model()
    assert 'task_policy' not in parameters
    # The full source cycle plus both sides of every task/period boundary,
    # negative time and repeated cycles cover the original branch conventions.
    boundaries = [base + offset for base in (-10., 0., 5., 10., 15., 20.)
                  for offset in HALF_CYCLE_BOUNDARIES_S]
    times = sorted(set(np.linspace(0., 10., 24001).tolist() +
                       [value + epsilon for value in boundaries
                        for epsilon in (-.00025, -1e-8, 0., 1e-8, .00025)]))
    result = subprocess.run([node, str(FIXTURES / 'reference-oracle.mjs')],
        input=json.dumps(dict(times=times, parameters=parameters)), text=True,
        capture_output=True, check=True, timeout=30)
    expected = json.loads(result.stdout)
    actual = [reference(time, parameters) for time in times]
    assert len(actual) == len(expected)
    for key, tolerance in (('q', 1e-12), ('qd', 1e-8), ('qdd', 1e-6)):
        np.testing.assert_allclose([row[key] for row in actual],
                                   [row[key] for row in expected], rtol=0, atol=tolerance)
    assert [row['stage'] for row in actual] == [row['stage'] for row in expected]
    assert [row['gap'] for row in actual] == [row['gap'] for row in expected]
