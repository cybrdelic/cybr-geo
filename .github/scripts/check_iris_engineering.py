"""Exercise the public IRIS engineering CLI and independently reopen its output.

This checks the newly derived single-motor linkage, not the source shader's
prescribed animation. Native material checks and ideal-pin dynamics have
different scopes; neither establishes hardware qualification.
"""
from __future__ import annotations

import argparse
import faulthandler
import gzip
import json
import math
from pathlib import Path
import subprocess
import sys
import time

from check_default_engineering import sha256, write_json


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def verify(output, *, require_local_native=False):
    import cadquery as cq
    import mujoco as mj
    import numpy as np
    import trimesh
    from mechanism_lab.engineering_source import NATIVE_SOURCE_BINDING, load_project, verify_native_build
    from mechanism_lab.physical_model import PhysicalModel, digest
    from mechanism_lab.causal_runtime import _IMPLEMENTATION_IDENTITY

    report = read_json(output / "engineering.json")
    assert report["schema"] == "cybrgeo-engineering-result/2" and report["name"] == "iris09"
    assert report["passed"] and report["requirements"]["passed"], "IRIS engineering task failed"
    assert all(row["passed"] for row in report["requirements"]["checks"])
    project, identity = load_project("iris09")
    spec = project.specification
    count = spec["source_geometry"]["leaf"]["count"]
    assert count == 9, "This benchmark is the nine-leaf IRIS-derived client"
    # A local final simulation may explicitly reuse the earlier fresh native
    # build. Follow its recorded directory without copying or relabeling CAD.
    # The CI construction path additionally requires its own output/native.
    native = Path(report["native"]["directory"]).expanduser().resolve()
    if require_local_native:
        assert native == (output / "native").resolve(), "Fresh CLI build did not use its own native directory"
    receipt = verify_native_build(native, spec, source_files=identity)
    assert (receipt["parts"], receipt["placements"]) == (68, 68), "Incomplete native IRIS client"
    binding = read_json(native / NATIVE_SOURCE_BINDING)
    assert report["source_files"] == identity
    assert report["native_build_sha256"] == digest(receipt)
    assert report["native_source_binding_sha256"] == digest(binding)
    assert report["native"]["verified_artifacts"] == len(receipt["files_sha256"])
    for key in ("parts", "placements"):
        assert report["native"][key] == receipt[key]

    physical_source = read_json(output / "physical-source.json")
    control = read_json(output / "control-source.json")
    physical = PhysicalModel.from_source(physical_source)
    assert report["model_sha256"] == digest(physical_source)
    assert report["control_sha256"] == digest(control)
    provenance = physical_source["provenance"]
    assert provenance["native_build_sha256"] == digest(receipt)
    assert provenance["native_source_binding_sha256"] == digest(binding)
    assert provenance["design_source_files"] == identity
    assert physical.xml() == (output / "model.xml").read_text(encoding="utf-8"), "Saved MJCF differs from bound source"

    with gzip.open(output / "simulation" / "trace.json.gz", "rt", encoding="utf-8") as stream:
        trace = json.load(stream)
    simulation = report["simulation"]
    assert simulation == {key: value for key, value in trace.items()
                          if key not in ("frames", "task_parameters", "physical_source", "requirements")}
    assert trace["physical_source"] == physical_source
    assert trace["reference_source"]["source_files"] == identity
    assert trace["reference_source"]["task_parameters"] == trace["task_parameters"]
    assert digest(trace["reference_source"]) == trace["reference_sha256"]
    assert trace["task_parameters"]["specification"] == spec
    assert simulation["model_sha256"] == report["model_sha256"]
    assert simulation["control_sha256"] == report["control_sha256"]
    assert simulation["schema"] == "cybrgeo-causal-simulation/1"
    assert simulation.get("runtime_implementation") == _IMPLEMENTATION_IDENTITY, "Simulation runtime identity is absent or differs"
    assert math.isclose(simulation["timestep_s"], spec["timestep_s"], abs_tol=1e-12)
    expected_steps = round(project.duration_s / simulation["timestep_s"])
    assert expected_steps == 40000 and project.duration_s == 8.
    assert simulation["steps"] == expected_steps and simulation["start_time_s"] == 0
    assert math.isclose(simulation["duration_s"], project.duration_s, abs_tol=1e-9)
    assert trace["frames"][0]["time_s"] == 0 and trace["frames"][-1]["time_s"] == project.duration_s
    checks = project.requirements(trace, trace["task_parameters"])
    assert checks == report["requirements"] == trace["requirements"] and checks["passed"]

    # Compile the persisted MJCF independently. Passive leaves may receive
    # native loop forces, but cannot hide additional position actuators.
    model = mj.MjModel.from_xml_path(str(output / "model.xml"))
    assert model.nu == 1 and len(control["drives"]) == 1 and not control.get("rack")
    assert model.njnt == model.nv == 1 + 2*count and model.neq == count
    assert model.ngeom == model.npair == 0, "Update the explicit contact scope if this client adds collision geometry"
    assert all(model.jnt_type == mj.mjtJoint.mjJNT_HINGE)
    assert all(model.eq_type == mj.mjtEq.mjEQ_CONNECT)
    ring = model.joint("ring_drive").id
    assert model.actuator("annular_motor").id == 0 and model.actuator_trnid[0, 0] == ring
    assert model.actuator_trntype[0] == mj.mjtTrn.mjTRN_JOINT
    assert np.array_equal(model.actuator_gear[0], [1, 0, 0, 0, 0, 0])
    assert control["inertial_feedforward"] is False
    metrics = simulation["metrics"]
    assert metrics["monitoring_interval"] == dict(start_time_s=0., end_time_s=8., integration_steps=expected_steps)
    assert len(metrics["joints"]) == model.njnt and len(metrics["constraints"]) == count
    leaf_extrema, closures = {}, {}
    for index in range(count):
        name = f"leaf_{index:02}_hinge"
        row = metrics["joints"][name]
        assert model.joint(name).id != ring and row["direct_motor_actuator"] is None
        assert row["samples"] == expected_steps + 1 and row["position_unit"] == "rad"
        assert row["speed_unit"] == "rad/s" and row["maximum_position"]["value"] > .89
        leaf_extrema[name] = {key: row[key] for key in ("minimum_position", "maximum_position", "peak_absolute_speed")}
        name = f"rod_{index:02}_closure"
        row = metrics["constraints"][name]
        assert row["type"] == "connect" and row["sampled_steps"] == row["active_steps"] == expected_steps
        assert row["inactive_steps"] == row["missing_row_steps"] == 0
        gap = row["vector_norms"]["anchor_delta"]
        assert gap["unit"] == "m" and gap["samples"] == expected_steps
        assert gap["peak_absolute"]["value"] * 1000 <= spec["closure_limit_mm"]
        closures[name] = gap["peak_absolute"]
    transmission = metrics["actuators"]["annular_motor"]["transmitted_torque_estimate"]
    assert transmission["samples"] == expected_steps and transmission["unit"] == "Nm"

    interfaces = read_json(native / "native-interface-requirements.json")
    assert interfaces["passed"] and len(interfaces["checks"]) == 102
    assert all(row["pass"] for row in interfaces["checks"])
    axial_envelope = next(row for row in interfaces["checks"]
                          if row["name"] == "moving_pin_envelope_to_lower_rotor_lip")
    assert axial_envelope["intersection_volume_mm3"] == 0
    assert axial_envelope["required_minimum_distance_mm"] == spec["axial_stack"]["moving_pin_to_fixed_lip_clearance_mm"]
    assert axial_envelope["required_minimum_distance_mm"] >= .2
    # Use the authored BRep comparison tolerance, not a stricter incidental
    # floating-point rounding requirement on another platform's CAD kernel.
    assert axial_envelope["minimum_distance_mm"] >= axial_envelope["required_minimum_distance_mm"] - 1e-8
    keeper_envelope = next(row for row in interfaces["checks"]
                           if row["name"] == "output_keeper_envelope_to_radial_bearing")
    assert keeper_envelope["intersection_volume_mm3"] == 0
    assert keeper_envelope["required_minimum_distance_mm"] == spec["axial_stack"]["keeper_to_radial_bearing_clearance_mm"]
    assert keeper_envelope["required_minimum_distance_mm"] >= .1
    assert keeper_envelope["minimum_distance_mm"] >= keeper_envelope["required_minimum_distance_mm"] - 1e-8
    layout = read_json(native / "mechanism-layout.json")
    assert len(layout["links"]) == count
    placements = read_json(native / "placed-native-parts.json")["records"]
    native_names = {row["body"] + "__" + row["name"] for row in placements}
    expected_bodies = {"fixed", "rotor"} | {f"leaf_{i:02}" for i in range(count)} | {f"rod_{i:02}" for i in range(count)}
    assert {row["body"] for row in placements} == expected_bodies
    assert len(placements) == len(native_names) == receipt["placements"]
    masses = read_json(native / "mass_properties.json")
    assert set(masses) == expected_bodies
    assert digest({name: masses[name] for name in expected_bodies - {"fixed"}}) == digest(physical.mass_records)

    step = output / "iris09.step"
    coverage = read_json(step.with_suffix(".coverage.json"))
    names = coverage["analytic_components"]
    assert len(names) == len(set(names)) == receipt["placements"]
    assert set(names) == native_names and not coverage["mesh_only_components_not_in_STEP"]
    print("Reopening IRIS aggregate STEP with OpenCascade", flush=True)
    solids = cq.importers.importStep(str(step)).solids().vals()
    assert len(solids) >= len(names), "Aggregate STEP omitted native solids"
    volumes = [solid.Volume() for solid in solids]
    assert all(solid.isValid() for solid in solids), "Invalid reopened STEP solid"
    assert all(math.isfinite(volume) and volume > 0 for volume in volumes), "Nonpositive STEP solid"
    glb = output / "assembly.glb"
    scene = trimesh.load(glb, force="scene")
    assert set(scene.graph.nodes_geometry) == native_names, "GLB and STEP placement coverage differ"
    evidence = dict(passed=True, project="iris09", native_parts=receipt["parts"],
        native_placements=receipt["placements"], verified_artifacts=len(receipt["files_sha256"]),
        native_source_binding_sha256=digest(binding), native_build_sha256=digest(receipt),
        construction_dependencies=binding["construction_dependencies"], project_source_files=len(identity),
        model_sha256=report["model_sha256"], control_sha256=report["control_sha256"],
        runtime_implementation=simulation["runtime_implementation"],
        simulated_seconds=simulation["duration_s"], simulated_steps=simulation["steps"],
        model=dict(actuators=model.nu, scalar_joints=model.njnt, connect_closures=model.neq,
                   collision_geometries=model.ngeom, explicit_contact_pairs=model.npair),
        task_requirements=checks, leaf_extrema=leaf_extrema, closure_peak_events_m=closures,
        constraint_sampling=metrics["constraint_sampling"], motor=metrics["actuators"]["annular_motor"],
        nominal_interface_checks=len(interfaces["checks"]), pin_lip_axial_envelope=axial_envelope,
        keeper_radial_bearing_envelope=keeper_envelope,
        leaf_geometry=layout["leaf_geometry"],
        contact_scope=provenance["contacts"], assumptions=spec["assumptions"],
        step=dict(file=step.name, sha256=sha256(step), bytes=step.stat().st_size,
                  valid_solids=len(solids), minimum_volume_mm3=min(volumes), sum_solid_volumes_mm3=math.fsum(volumes)),
        glb=dict(file=glb.name, sha256=sha256(glb), bytes=glb.stat().st_size, geometry_nodes=len(scene.graph.nodes_geometry)),
        qualification=dict(construction="Native CAD, recorded nominal mating clearances and source-bound CAD mass tensors",
            dynamics="One electrical motor and nine passive leaf/rod loops integrated by MuJoCo with ideal pin constraints",
            contact="No leaf, bearing or housing contact-force model is exercised by this client",
            clearance="Selected nominal CAD fits, authored leaf keepouts and pin/lower-lip and keeper/radial-bearing envelopes; no complete continuous assembly motion-envelope proof",
            hardware="Not qualified: assumed motor/materials/foil thickness; tolerances, retention strength, fatigue and wear remain open"))
    write_json(output / "ci-readback.json", evidence)
    # Full evidence is retained as an artifact. Keep console output readable.
    print(json.dumps({key: evidence[key] for key in ("passed", "project", "native_parts", "native_placements",
        "verified_artifacts", "simulated_seconds", "simulated_steps", "model", "nominal_interface_checks",
        "step", "glb", "qualification")}, indent=2), flush=True)


def run(output, timeout):
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("Timeout must be finite and positive")
    output.mkdir(parents=True, exist_ok=True)
    assert not (output / "engineering.json").exists(), "Expected a fresh IRIS output directory"
    started = time.perf_counter()
    deadline = started + timeout
    diagnostic_entry = "import faulthandler,runpy; faulthandler.enable(); runpy.run_module('mechanism_lab',run_name='__main__')"
    public = [sys.executable, "-m", "mechanism_lab", "mechanism", "iris09", "--step", "--out", str(output)]
    stages = [("iris-command", [sys.executable, "-u", "-c", diagnostic_entry, *public[3:]]),
              ("independent-readback", [sys.executable, "-u", str(Path(__file__).resolve()), "--verify",
                                       "--require-local-native", "--output", str(output)])]
    status = dict(public_command=public, timeout_s=timeout, passed=False, stages=[])
    try:
        for name, command in stages:
            stage = dict(name=name, passed=False)
            status["stages"].append(stage)
            start = time.perf_counter()
            log = output / (name + ".log")
            print("Starting " + name, flush=True)
            try:
                with log.open("w", encoding="utf-8") as stream:
                    result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT,
                                            timeout=max(.001, deadline-time.perf_counter()))
                stage["returncode"] = result.returncode
                result.check_returncode()
                stage["passed"] = True
            except subprocess.TimeoutExpired:
                stage["timed_out"] = True
                raise
            finally:
                stage["elapsed_s"] = time.perf_counter()-start
                if log.exists():
                    print(log.read_text(encoding="utf-8", errors="replace")[-12000:], flush=True)
                print(json.dumps(stage), flush=True)
        status["passed"] = True
    finally:
        status["elapsed_s"] = time.perf_counter()-started
        status["completed_markers"] = {name: (output/name).is_file() for name in
            ("native/native-build.json", "native/native-source-binding.json", "engineering.json",
             "simulation/simulation.json", "iris09.step", "assembly.glb", "ci-readback.json")}
        status["native_files_written"] = sum(path.is_file() for path in (output/"native").rglob("*"))
        write_json(output / "ci-command.json", status)
        print(json.dumps(status, indent=2), flush=True)


if __name__ == "__main__":
    faulthandler.enable()  # Fatal stacks only; no unsafe periodic watchdog frame walks.
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("outputs/portable-iris09"))
    parser.add_argument("--timeout", type=float, default=1800)
    parser.add_argument("--verify", action="store_true")
    parser.add_argument("--require-local-native", action="store_true",
                        help="Require the fresh CI build's native artifacts under output/native")
    args = parser.parse_args()
    if args.verify:
        verify(args.output, require_local_native=args.require_local_native)
    else:
        run(args.output, args.timeout)
