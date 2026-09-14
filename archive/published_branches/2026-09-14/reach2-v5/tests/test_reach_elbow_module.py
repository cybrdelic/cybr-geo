from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
RECIPE = ROOT / "examples" / "reach_elbow_module.py"


def load_recipe():
    spec = importlib.util.spec_from_file_location("reach_test_recipe", RECIPE)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def built():
    recipe = load_recipe()
    return recipe, recipe.build()


def part(assembly, name):
    return next(p for p in assembly.parts if p.name == name)


def overlap_mm3(a, b):
    return sum(abs(s.Volume()) for s in a.intersect(b).Solids())


def test_reach_build_contract_and_exact_orbit_interface(built):
    r, a = built
    assert a.name == "CYBR REACH-1 + ORBIT"
    assert len(a.parts) >= 170
    assert len({p.name for p in a.parts}) == len(a.parts)
    assert math.isclose(r.REDUCTION, 16.0)
    assert a.metadata["reduction_ratio"] == 16.0
    assert a.metadata["orbit_interface_pattern_mm"] == [list(p) for p in r.ORBIT_PATTERN]
    assert not any(p.name.startswith("O_B02_") for p in a.parts)

    d2 = np.linalg.norm((r.INTERMEDIATE - r.PIVOT)[[0, 2]])
    d1 = np.linalg.norm((r.INPUT - r.INTERMEDIATE)[[0, 2]])
    assert math.isclose(d2, (r.STAGE2_OUTPUT_TEETH + r.STAGE2_PINION_TEETH) * r.GEAR_MODULE / 2)
    assert math.isclose(d1, (r.STAGE1_GEAR_TEETH + r.STAGE1_PINION_TEETH) * r.GEAR_MODULE / 2)

    for p in a.parts:
        assert p.cad is not None
        assert p.cad.isValid()
        assert np.isfinite(p.vertices).all()
        assert np.isfinite(p.normals).all()
        assert len(p.faces) > 0


def test_interface_faces_contact_without_solid_overlap(built):
    r, a = built
    shoe = part(a, "O_B01_Mounting_shoe")
    saddle = part(a, "R01_ORBIT_interface_saddle")
    assert abs(shoe.bounds[0, 2]) < 1e-6
    assert abs(saddle.bounds[1, 2]) < 1e-6
    assert overlap_mm3(shoe.cad, saddle.cad) < 1e-5

    for i, _ in enumerate(r.ORBIT_PATTERN):
        screw = part(a, f"R06_{i}_ORBIT_M6_socket_screw")
        assert overlap_mm3(shoe.cad, screw.cad) < 1e-4
        insert = part(a, f"R02_{i}_Steel_thread_insert")
        assert overlap_mm3(insert.cad, screw.cad) < 1e-4


def test_reduction_pairs_do_not_have_gross_static_overlap(built):
    _, a = built
    for left, right in [
        ("R12_56T_Output_gear", "R13_14T_Intermediate_pinion"),
        ("R14_48T_Intermediate_gear", "R15_12T_Input_pinion"),
    ]:
        vol = overlap_mm3(part(a, left).cad, part(a, right).cad)
        assert vol < 1e-3, (left, right, vol)


def test_orbit_and_reach_output_move_as_one_rigid_attachment(built):
    r, a = built
    t = r.PERIOD / 4.0
    shoe = part(a, "O_B01_Mounting_shoe")
    saddle = part(a, "R01_ORBIT_interface_saddle")
    screw = part(a, "R06_0_ORBIT_M6_socket_screw")
    Ts = [a.pose(p, t, 0.0) for p in (shoe, saddle, screw)]
    assert np.allclose(Ts[0], Ts[1], atol=1e-10)
    assert np.allclose(Ts[1], Ts[2], atol=1e-10)

    theta = r.elbow_angle(t)
    assert math.isclose(theta, math.radians(r.MAX_ELBOW_DEG), rel_tol=1e-12)
    assert np.allclose(Ts[1], r.rotation_y(theta, r.PIVOT), atol=1e-10)
