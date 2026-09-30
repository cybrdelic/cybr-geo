"""Regression checks for material details that used to disappear in finishing."""
from pathlib import Path
import subprocess

import numpy as np

from cybr_light.finishing import atrous


def test_native_nested_surface_guides_and_textured_roughness(tmp_path):
    native = Path(__file__).resolve().parents[1] / 'src/cybr_light/native'
    exe = tmp_path / 'surface-guides'
    subprocess.run(['g++', '-std=c++17', '-O2', '-I' + str(native / 'include'),
                    str(native / 'tests/surface_guides.cpp'), '-ldl', '-o', str(exe)], check=True)
    result = subprocess.run([str(exe)], check=True, capture_output=True, text=True)
    assert 'TOTAL 19 passed, 0 failed' in result.stdout


def guides(size=32):
    guide = np.zeros((size, size, 9), np.float32)
    guide[:, :, 2] = 1
    guide[:, :, 3:6] = .4
    guide[:, :, 6] = 2
    guide[:, :, 7] = .03
    return guide


def test_albedo_guide_preserves_fine_material_pattern_under_high_path_noise():
    guide = guides()
    # This alternating physical reflectance used to be blurred even with
    # perfect guides: all adjacent pixels have the same depth and object ID.
    guide[:, ::2, 3:6] = .08
    guide[:, 1::2, 3:6] = .75
    radiance = guide[:, :, 3:6].copy()
    variance = guide[:, :, 7].copy()
    for i in range(3):
        radiance, variance = atrous(radiance, guide, variance, 2 ** i, i, -2)
    assert np.max(np.abs(radiance - guide[:, :, 3:6])) < 1e-5


def test_material_edge_weight_still_reduces_noise_within_a_flat_material():
    guide = guides(48)
    rng = np.random.default_rng(618)
    radiance = np.repeat((.4 + rng.normal(0, .08, (48, 48)))[:, :, None], 3, axis=2).astype('f4')
    variance = np.full((48, 48), .08 ** 2, np.float32)
    filtered, _ = atrous(radiance, guide, variance, 1, 0, -2)
    assert np.var(filtered) < np.var(radiance) * .3
    assert abs(float(filtered.mean()) - float(radiance.mean())) < .01
