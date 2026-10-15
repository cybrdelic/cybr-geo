import numpy as np
import pytest

from mechanism_lab.models.portfolio_final import BUILDERS, build
from mechanism_lab.models.portfolio_final_machines import build_geo, REFERENCE_ID


MIN_PARTS = {
    "portfolio_scenes": 150,
    "portfolio_geo": 250,
    "portfolio_light": 90,
    "portfolio_elements": 80,
    "portfolio_materials": 115,
    "portfolio_forest": 130,
}


@pytest.mark.parametrize("name", sorted(BUILDERS))
def test_portfolio_reference_module_builds_as_real_geometry(name):
    assembly = build(name)
    assert assembly.name == name
    assert assembly.metadata["truth_intent"] == "concept"
    assert assembly.metadata["reference_id"] == REFERENCE_ID
    assert len(assembly.parts) >= MIN_PARTS[name]
    assert len({part.name for part in assembly.parts}) == len(assembly.parts)
    assert "hero" in assembly.views
    assert "exploded" in assembly.views
    assert np.isfinite(assembly.bounds).all()
    assert np.all(assembly.bounds[1] > assembly.bounds[0])
    assert all(part.provenance == "designed-concept" for part in assembly.parts)
    assert all(len(part.vertices) > 0 and len(part.faces) > 0 for part in assembly.parts)


def test_portfolio_geo_is_a_real_exposed_electromechanical_assembly():
    assembly = build_geo()
    windings = [part for part in assembly.parts if part.group == "windings"]
    rotor = [part for part in assembly.parts if part.motion == "rotor"]
    bearings = [part for part in assembly.parts if part.group in ("bearings", "bearing_balls")]
    assert len(windings) >= 100
    assert len(rotor) >= 34
    assert len(bearings) >= 24
    before = assembly.pose(rotor[0], 0.0, 0.0)
    after = assembly.pose(rotor[0], 1.0, 0.0)
    assert not np.allclose(before, after)


@pytest.mark.parametrize("name,group,min_triangles", [
    ("portfolio_scenes", "environment", 180_000),
    ("portfolio_forest", "environment", 180_000),
    ("portfolio_elements", "process_core", 120_000),
])
def test_portfolio_worlds_use_dense_authored_surface_geometry(name, group, min_triangles):
    assembly = build(name)
    triangles = sum(len(p.faces) for p in assembly.parts if p.group == group)
    assert triangles >= min_triangles


def test_portfolio_reference_dimensions_are_retained():
    expected = {
        "portfolio_geo": {"length": 1280.0, "width": 640.0, "height": 680.0},
        "portfolio_elements": {"width": 820.0, "depth": 820.0, "height": 820.0},
        "portfolio_materials": {"length": 980.0, "depth": 640.0, "height": 720.0},
    }
    for name, dimensions in expected.items():
        assert build(name).metadata["concept_dimensions_mm"] == dimensions
