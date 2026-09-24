"""Regression checks for scan-derived morphable face geometry."""
from pathlib import Path
import sys
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/"examples"),str(ROOT/"src")]
from morphable_human_face import MorphableFaceParameters,build


@pytest.fixture(scope="module")
def canonical():
    if not (ROOT/"assets/portrait/LeePerrySmith.glb").exists():
        pytest.skip("Run tools/fetch_portrait_assets.py first")
    return build()


@pytest.fixture(scope="module")
def variant():
    if not (ROOT/"assets/portrait/LeePerrySmith.glb").exists():
        pytest.skip("Run tools/fetch_portrait_assets.py first")
    return build(MorphableFaceParameters(
        skull_width=.07,cheek_width=.08,jaw_width=.10,
        lower_face_length=.04,nose_width=.06,nose_projection_mm=3.0,
        mouth_width=.04,chin_projection_mm=2.0,asymmetry=.25))


def test_morph_preserves_topology_uv_and_finite_normals(canonical,variant):
    a,b=canonical.parts[0],variant.parts[0]
    assert np.array_equal(a.faces,b.faces)
    assert np.array_equal(a.portrait_uv,b.portrait_uv)
    assert a.vertices.shape==b.vertices.shape
    assert np.isfinite(b.vertices).all()
    assert np.isfinite(b.normals).all()
    assert np.allclose(np.linalg.norm(b.normals,axis=1),1.,atol=3e-5)
    assert np.max(np.linalg.norm(b.vertices-a.vertices,axis=1))<25.0


def test_identity_parameters_change_real_geometry(canonical,variant):
    a,b=canonical.parts[0],variant.parts[0]
    delta=np.linalg.norm(b.vertices-a.vertices,axis=1)
    assert np.percentile(delta,50)>.05
    assert np.percentile(delta,95)>1.0
    assert np.max(delta)>2.0
    assert variant.metadata["image_generation"] is False
    assert variant.metadata["model_type"]=="scan-derived morphable human head"


def test_regional_morph_is_not_uniform_scaling(canonical,variant):
    a,b=canonical.parts[0],variant.parts[0]
    # Pairwise local displacement ratios vary strongly across face/neck; this
    # guards against accidentally replacing semantic fields with one global scale.
    d=b.vertices-a.vertices
    mag=np.linalg.norm(d,axis=1)
    assert np.std(mag)>.25
    front=a.vertices[:,1] < -45
    assert mag[front].mean()>mag[~front].mean()
