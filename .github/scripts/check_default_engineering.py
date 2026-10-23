"""Run the public default and independently read its artifacts on each CI OS.

The CLI and the fresh readback process share one deadline. This is software
construction/dynamics evidence, not manufacturing or hardware qualification.
"""
from __future__ import annotations

import argparse
import faulthandler
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def sha256(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def verify(output):
    """Reopen persisted evidence and STEP, without reconstructing the assembly."""
    import cadquery as cq
    import trimesh
    from mechanism_lab.engineering_source import (
        NATIVE_SOURCE_BINDING, load_project, verify_native_build,
    )
    from mechanism_lab.physical_model import digest

    report = json.loads((output / "engineering.json").read_text(encoding="utf-8"))
    assert report["schema"] == "cybrgeo-engineering-result/2"
    assert report["name"] == "arx_belt", "The omitted model did not select the authored default"
    assert report["passed"] and report["requirements"]["passed"], "Default engineering acceptance failed"
    assert all(check["passed"] for check in report["requirements"]["checks"])
    project, identity = load_project("arx_belt")
    native = output / "native"
    receipt = verify_native_build(native, project.specification, source_files=identity)
    binding = json.loads((native / NATIVE_SOURCE_BINDING).read_text(encoding="utf-8"))
    assert report["source_files"] == identity
    assert report["native_build_sha256"] == digest(receipt)
    assert report["native_source_binding_sha256"] == digest(binding)
    assert (receipt["parts"], receipt["placements"]) == (118, 380), "Incomplete native default"
    for key in ("parts", "placements"):
        assert report["native"][key] == receipt[key]
    assert report["native"]["verified_artifacts"] == len(receipt["files_sha256"])
    import importlib.util
    client_file = Path(__file__).resolve().parents[2] / 'examples' / 'analysis' / 'arx_belt_gripper_envelope.py'
    client_spec = importlib.util.spec_from_file_location('arx_belt_gripper_envelope', client_file)
    client_module = importlib.util.module_from_spec(client_spec)
    client_spec.loader.exec_module(client_module)
    gripper = client_module.certify(native)
    assert gripper['passed'] and gripper['selected_part_count'] == 6
    assert gripper['noncontact_cross_body_pair_count'] == 12
    assert gripper['unexamined_native_part_count'] == receipt['placements'] - 6
    write_json(output / 'arx-gripper-envelope.json', gripper)
    interfaces = json.loads((native / "native-interface-requirements.json").read_text())
    assert interfaces["passed"] and len(interfaces["checks"]) == 52
    assert all(check["pass"] for check in interfaces["checks"])
    simulation = report["simulation"]
    assert simulation["schema"] == "cybrgeo-causal-simulation/1"
    assert simulation["duration_s"] >= project.duration_s - 1e-9
    assert simulation["steps"] >= round(project.duration_s / simulation["timestep_s"])
    assert simulation["model_sha256"] == report["model_sha256"]
    physical_source = json.loads((output / "physical-source.json").read_text())
    assert report["model_sha256"] == digest(physical_source)
    provenance = physical_source["provenance"]
    assert provenance["native_build_sha256"] == digest(receipt)
    assert provenance["native_source_binding_sha256"] == digest(binding)
    assert provenance["design_source_files"] == identity
    assert simulation["control_sha256"] == report["control_sha256"]
    control = json.loads((output / "control-source.json").read_text())
    assert report["control_sha256"] == digest(control)
    design_screen = json.loads((output / 'design-envelope.json').read_text())
    assert report['design_envelope'] == design_screen
    assert report['design_envelope_sha256'] == digest(design_screen)
    assert not design_screen['manufacturing_qualified']
    for name, motor in design_screen['drives'].items():
        observed = simulation['metrics']['actuators'][name]
        assert motor['complete_demand_telemetry'], name
        assert motor['demand_limit_durations_s'] == {
            limit: observed[f'{limit}_demand_limit_s'] for limit in ('belt', 'gear')}
        assert motor['demanded_torque_was_limited'] == any(
            observed[f'{limit}_demand_limit_s'] > 0 for limit in ('belt', 'gear'))
        assert not motor['demanded_torque_was_limited'], name
        assert motor['model_screen_passed'] is True, name
    constraints = simulation['metrics']['constraints']
    modeled_connectors = {name for name, row in constraints.items()
                          if row['type'] in ('connect', 'weld')}
    connector_screen = design_screen['connector_demands']
    assert set(connector_screen['connectors']) == modeled_connectors
    assert all(row['complete_solver_coverage'] for row in connector_screen['connectors'].values())
    rack = simulation['metrics']['rack_motor']
    assert rack['samples'] == simulation['steps']
    assert rack['source_output_torque_limit_Nm'] == control['rack']['output_torque_limit_Nm']
    assert rack['output_demand_limit_s'] == 0.
    assert rack['peak_requested_output_torque_Nm'] < rack['source_output_torque_limit_Nm']

    step = output / "arx_belt.step"
    coverage = json.loads(step.with_suffix(".coverage.json").read_text())
    names = coverage["analytic_components"]
    # 380 native placements include both actual handed jaws; only the free
    # puck and four run-bound nest components are additional scene geometry.
    assert len(names) == len(set(names)) == 385, "Incomplete aggregate CAD coverage"
    assert not coverage["mesh_only_components_not_in_STEP"], "Default STEP omitted scene components"
    placements = json.loads((native / "placed-native-parts.json").read_text())["records"]
    native_names = {row["body"] + "__" + row["name"] for row in placements}
    assert len(placements) == len(native_names) == receipt["placements"]
    assert native_names <= set(names), "Aggregate coverage omitted native placements"
    from mechanism_lab.native_construction import shape_mass_properties, aggregate_mass_properties
    import numpy as np
    handed = [row for row in placements if row['body'] == 'jaw_opposed']
    assert {row['name'] for row in handed} == {'left_jaw_carriage', 'finger_pad', 'lower_rack',
                                              'guide_bushing_-10', 'guide_bushing_10'}
    masses = json.loads((native/'mass_properties.json').read_text())
    assert physical_source['mass_records']['jaw_right'] == masses['jaw_opposed']
    rebuilt = []
    for row in handed:
        solid = cq.importers.importStep(str(native/row['step_file'])).val()
        mass, center, tensor = shape_mass_properties(solid, receipt['material_densities_kg_m3'][row['material']])
        rebuilt.append(dict(name=row['name'], mass_kg=mass, com_m=center.tolist(),
                            inertia_at_com_kg_m2=tensor.tolist(),
                            provenance=dict(method='Independent placed STEP volume integration',
                                step_file=row['step_file'], step_sha256=sha256(native/row['step_file']),
                                density_kg_m3=receipt['material_densities_kg_m3'][row['material']])))
    reconstructed = aggregate_mass_properties(rebuilt)
    expected = masses['jaw_opposed']
    for key in ('mass_kg', 'com_m', 'inertia_tensor_kg_m2'):
        assert np.allclose(reconstructed[key], expected[key], rtol=2e-7, atol=1e-12), 'Handed jaw STEP/mass mismatch: '+key
    print("Reopening aggregate STEP with OpenCascade", flush=True)
    solids = cq.importers.importStep(str(step)).solids().vals()
    assert len(solids) >= len(names), "Aggregate STEP omitted analytic components"
    volumes = [solid.Volume() for solid in solids]
    assert all(solid.isValid() for solid in solids), "Invalid reopened STEP solid"
    assert all(math.isfinite(volume) and volume > 0 for volume in volumes), "Nonpositive STEP solid"
    glb = output / "assembly.glb"
    scene = trimesh.load(glb, force="scene")
    assert set(scene.graph.nodes_geometry) == set(names), "GLB and analytic coverage differ"
    evidence = dict(
        passed=True, default_name=report["name"], native_parts=receipt["parts"],
        native_placements=receipt["placements"], verified_artifacts=len(receipt["files_sha256"]),
        nominal_interface_checks=len(interfaces["checks"]),
        scoped_gripper_envelope=dict(selected_parts=gripper['selected_part_count'],
            noncontact_pairs=gripper['noncontact_cross_body_pair_count'],
            unexamined_native_parts=gripper['unexamined_native_part_count'],
            receipt_sha256=sha256(output / 'arx-gripper-envelope.json')),
        handed_jaw=dict(native_components=len(handed), physical_body='jaw_right',
            native_mass_body='jaw_opposed', independently_reintegrated_from_placed_STEP=True,
            physical_densities_kg_m3=receipt['material_densities_kg_m3']),
        aggregate_analytic_components=len(names),
        project_source_files=len(identity), construction_dependencies=binding["construction_dependencies"],
        native_source_binding_sha256=digest(binding), native_build_sha256=digest(receipt),
        model_sha256=report["model_sha256"], control_sha256=report["control_sha256"],
        simulated_seconds=simulation["duration_s"], simulated_steps=simulation["steps"],
        model_drive_screens={name: dict(status=row['status'],
            demanded_torque_was_limited=row['demanded_torque_was_limited'])
            for name, row in design_screen['drives'].items()},
        rack_source_model=dict(output_demand_limit_s=rack['output_demand_limit_s'],
            peak_requested_output_torque_Nm=rack['peak_requested_output_torque_Nm'],
            source_output_torque_limit_Nm=rack['source_output_torque_limit_Nm']),
        step=dict(file=step.name, sha256=sha256(step), bytes=step.stat().st_size,
                  valid_solids=len(solids), minimum_volume_mm3=min(volumes),
                  sum_solid_volumes_mm3=math.fsum(volumes)),
        glb=dict(file=glb.name, sha256=sha256(glb), bytes=glb.stat().st_size,
                 geometry_nodes=len(scene.graph.nodes_geometry)),
        scope="Fresh native construction, source binding, nominal contact dynamics and independent CAD readback; no hardware qualification",
    )
    write_json(output / "ci-readback.json", evidence)
    print(json.dumps(evidence, indent=2), flush=True)


def run(output, timeout):
    output.mkdir(parents=True, exist_ok=True)
    # Refuse stale output: this job must actually construct the default.
    assert not (output / "engineering.json").exists(), "Expected a fresh default output directory"
    started = time.perf_counter()
    deadline = started + timeout
    diagnostic_entry = (
        "import faulthandler,runpy; faulthandler.enable(); "
        "runpy.run_module('mechanism_lab',run_name='__main__')"
    )
    public_command = [sys.executable, "-m", "mechanism_lab", "mechanism", "--step", "--out", str(output)]
    stages = [
        ("default-command", [sys.executable, "-u", "-c", diagnostic_entry, *public_command[3:]]),
        ("independent-readback", [sys.executable, "-u", str(Path(__file__).resolve()), "--verify", "--output", str(output)]),
    ]
    status = dict(public_command=public_command, timeout_s=timeout, passed=False, stages=[])
    try:
        for name, command in stages:
            stage = dict(name=name, elapsed_s=0, passed=False)
            status["stages"].append(stage)
            stage_started = time.perf_counter()
            log = output / (name + ".log")
            print("Starting " + name, flush=True)
            try:
                with log.open("w", encoding="utf-8") as stream:
                    result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT,
                                            timeout=max(0.001, deadline - time.perf_counter()))
                stage["returncode"] = result.returncode
                result.check_returncode()
                stage["passed"] = True
            finally:
                stage["elapsed_s"] = time.perf_counter() - stage_started
                # Preserve full logs as artifacts; keep the Actions console concise.
                if log.exists():
                    print(log.read_text(encoding="utf-8", errors="replace")[-12000:], flush=True)
                print(json.dumps(stage), flush=True)
        status["passed"] = True
    finally:
        status["elapsed_s"] = time.perf_counter() - started
        status["completed_markers"] = {
            name: (output / name).is_file() for name in
            ("native/native-build.json", "native/native-source-binding.json", "engineering.json",
             "simulation/simulation.json", "arx_belt.step", "assembly.glb", "ci-readback.json")
        }
        status["native_files_written"] = sum(path.is_file() for path in (output / "native").rglob("*"))
        write_json(output / "ci-command.json", status)
        print(json.dumps(status, indent=2), flush=True)


if __name__ == "__main__":
    # Capture actual SIGSEGV/access-violation stacks. Avoid unsynchronized
    # periodic watchdog walks: https://github.com/python/cpython/issues/140815.
    faulthandler.enable()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("outputs/portable-default"))
    parser.add_argument("--timeout", type=float, default=1800)
    parser.add_argument("--verify", action="store_true")
    args = parser.parse_args()
    if args.verify:
        verify(args.output)
    else:
        run(args.output, args.timeout)
