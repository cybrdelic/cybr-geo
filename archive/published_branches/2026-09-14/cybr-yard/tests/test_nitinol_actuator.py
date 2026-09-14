import numpy as np
import pytest

from mechanism_lab.core import validate
from mechanism_lab.models import nitinol_actuator as sma


@pytest.fixture(scope="module")
def actuator():
    assembly = sma.build()
    validate(assembly)
    return assembly


def test_source_guided_default_metrics_are_self_consistent():
    m = sma.design_metrics()
    assert m["stroke_mm"] == pytest.approx(4.9)
    assert m["series_string_count"] == 3
    assert m["series_per_string"] == 4
    assert m["wire_resistance_ohm"] == pytest.approx(4.06)
    assert m["string_resistance_ohm"] == pytest.approx(16.24)
    assert m["nominal_string_voltage_V"] == pytest.approx(10.7184)
    assert m["nominal_total_current_A"] == pytest.approx(1.98)
    assert m["nominal_electrical_power_W"] == pytest.approx(21.222432)
    assert m["bundle_heating_pull_force_N"] > 67.0
    assert m["spring_force_hot_end_N"] <= m["bundle_cooling_deformation_force_N"]
    assert m["estimated_net_hot_pull_at_full_stroke_N"] > 40.0


def test_bundle_geometry_and_electrical_topology(actuator):
    c = sma.DEFAULT_CONFIG
    assert actuator.name == "nitinol_fiber_actuator"
    assert sum(p.name.startswith("SMA14_Fiber_") for p in actuator.parts) == c.fiber_count * c.fiber_segments
    assert sum(p.name.startswith("SMA15_Fixed_crimp_") for p in actuator.parts) == c.fiber_count
    assert sum(p.name.startswith("SMA16_Moving_crimp_") for p in actuator.parts) == c.fiber_count
    assert sum(p.name.startswith("SMA17_String_") for p in actuator.parts) == 9
    assert actuator.metadata["electrical_topology"]["electrical_parallel_strings"] == 3
    assert actuator.metadata["electrical_topology"]["series_fibers_per_string"] == 4
    assert len(actuator.metadata["sources"]) == 3


def test_hot_pose_moves_carriage_and_distributes_fiber_contraction(actuator):
    parts = {p.name: p for p in actuator.parts}
    stroke = sma.design_metrics()["stroke_mm"]
    hot_t = 1.2

    assert np.allclose(actuator.pose(parts["SMA01_Rear_frame_plate"], hot_t), np.eye(4))
    assert actuator.pose(parts["SMA06_Output_carriage"], hot_t)[0, 3] == pytest.approx(-stroke)

    first = parts["SMA14_Fiber_01_segment_01"]
    last = parts[f"SMA14_Fiber_01_segment_{sma.DEFAULT_CONFIG.fiber_segments:02}"]
    dx_first = actuator.pose(first, hot_t)[0, 3]
    dx_last = actuator.pose(last, hot_t)[0, 3]
    assert 0 > dx_first > dx_last > -stroke

    for t in np.linspace(0, 6, 9):
        for p in actuator.parts:
            T = actuator.pose(p, float(t), 0.25)
            assert np.isfinite(T).all()
            assert np.allclose(T[3], [0, 0, 0, 1])
            assert np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-10)
            assert np.linalg.det(T[:3, :3]) == pytest.approx(1.0)


def test_hard_stops_bound_the_nominal_carriage_travel(actuator):
    parts = {p.name: p for p in actuator.parts}
    stroke = sma.design_metrics()["stroke_mm"]
    bushing = parts["SMA07_Carriage_bushing_01"]
    cold_stop = parts["SMA22_Cold_stop_01"]
    hot_stop = parts["SMA23_Hot_stop_01"]

    cold_gap = cold_stop.bounds[0, 0] - bushing.bounds[1, 0]
    hot_bushing_left = bushing.bounds[0, 0] - stroke
    hot_gap = hot_bushing_left - hot_stop.bounds[1, 0]
    assert cold_gap == pytest.approx(0.5, abs=1e-7)
    assert hot_gap == pytest.approx(0.1, abs=1e-7)
