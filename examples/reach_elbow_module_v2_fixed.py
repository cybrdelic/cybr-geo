"""Robust build wrapper for REACH-1 v2.

The initial v2 saddle gusset used a boolean intersection that OpenCascade reduced
to a non-triangulatable compound. This wrapper replaces only those two gussets
with direct trapezoidal extrusions; all other v2 geometry and kinematics are
unchanged.
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

import cadquery as cq

ROOT = Path(__file__).resolve().parents[1]
V2_PATH = ROOT / "examples" / "reach_elbow_module_v2.py"

spec = importlib.util.spec_from_file_location("reach_v2_raw", V2_PATH)
v2 = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(v2)

_original_cad_part = v2.cad_part


def _cad_part(name, shape, *args, **kwargs):
    if name.startswith("R31_"):
        y = -47.0 if "_L_" in name else 65.0
        # Direct manifold trapezoid beneath the saddle. No boolean intersection,
        # no zero-area slivers, and the same intended reinforcement envelope.
        shape = (
            cq.Workplane("XZ", origin=(0, y, 0))
            .polyline([(-43.0, -22.0), (0.0, -22.0), (-8.0, -6.0), (-36.0, -6.0)])
            .close()
            .extrude(4.0)
            .val()
        )
    return _original_cad_part(name, shape, *args, **kwargs)


v2.cad_part = _cad_part

build = v2.build
load_base = v2.load_base
