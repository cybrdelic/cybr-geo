import numpy as np
import pytest

from mechanism_lab.models.portfolio_machine import (
    BUILDERS,
    REFERENCE_ID,
    build,
    build_geo,
)


@pytest.mark.parametrize("name", sorted(BUILDERS))
def test_portfolio_reference_module_builds_as_real_geometry(name):
    assembly = build(name)
    assert assembly.name == name
    assert assembly.metadata["truth_intent"] == "concept"
    assert assembly.metadata["reference_id"] == REFERENCE_ID
    assert len(assembly.parts) >= 20
    assert len({part.name for part in assembly.parts}) == len(assembly.parts)
    assert "hero" in assembly.views
    assert "exploded" in assembly.views
    assert np.isfinite(assembly.bounds).all()
    assert np.all(assembly.bounds[1] > assembly.bounds[0])
    assert all(part.provenance == "designed-concept" for part in assembly.parts)
    assert all(len(part.vertices) > 0 and len(part.faces) > 0 for part in assembly.parts)


def test_portfolio_geo_has_exposed_windings_and_rotor_motion():
    assembly = build_geo()
    windings = [part for part in assembly.parts if part.group == "windings"]
    rotor = [part for part in assembly.parts if part.motion == "rotor"]
    assert len(windings) == 24
    assert len(rotor) >= 4
    before = assembly.pose(rotor[0], 0.0, 0.0)
    after = assembly.pose(rotor[0], 1.0, 0.0)
    assert not np.allclose(before, after)


def test_portfolio_reference_dimensions_are_retained():
    expected = {
        "portfolio_geo": {"length": 1280.0, "width": 640.0, "height": 680.0},
        "portfolio_elements": {"width": 820.0, "depth": 820.0, "height": 820.0},
        "portfolio_materials": {"length": 980.0, "depth": 640.0, "height": 720.0},
    }
    for name, dimensions in expected.items():
        assert build(name).metadata["concept_dimensions_mm"] == dimensions
