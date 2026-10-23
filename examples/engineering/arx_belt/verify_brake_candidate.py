"""Build and exercise the opt-in ARX gripper brake diagnostic candidate.

This program intentionally requires fresh native CAD. Its numerical fault
result and sampled BREP gaps are development evidence, never brake supplier,
fabrication, or hardware qualification. The shipped P5 source spec is copied
in memory and is never edited.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import gzip
import hashlib
import json
import math
from pathlib import Path
import traceback
import xml.etree.ElementTree as ET

import cadquery as cq
import numpy as np
from scipy.spatial.transform import Rotation

from mechanism_lab.causal_runtime import CausalSimulation
from mechanism_lab.engineering_source import (assert_project_source_identity,
    construct_project_native, project_source_identity)
from mechanism_lab.part_geometry import place_local_shape
from mechanism_lab.physical_model import digest

from .gripper_retention import (CANDIDATE_REVISION, candidate_recipe,
                                fault_contact_eligibility, DATA)
from .project import create


def _frame(row):
    pose = np.eye(4)
    pose[:3, :3] = np.asarray(row['rotation_row_major'], dtype=float).reshape(3, 3)
    pose[:3, 3] = np.asarray(row['position_m'], dtype=float) * 1000.
    return pose


def _source_cylinder(geom):
    if geom is None or geom.get('type') != 'cylinder':
        raise ValueError('Pinned workpiece and nests must remain cylindrical')
    radius, half_length = [float(value)*1000 for value in geom.get('size').split()]
    center = np.asarray([float(value)*1000 for value in geom.get('pos', '0 0 0').split()])
    w, x, y, z = [float(value) for value in geom.get('quat', '1 0 0 0').split()]
    axis = Rotation.from_quat([x, y, z, w]).apply([0., 0., 1.])
    solid = cq.Solid.makeCylinder(radius, 2*half_length,
        cq.Vector(*(center-axis*half_length)), cq.Vector(*axis))
    return solid, radius, half_length


def _sampled_puck_clearance(native, frames, candidate, onset_s):
    """Check a bounded set of *actual recorded poses*, retaining its limits."""
    # Source MJCF provides the workpiece and both real nest envelopes. A
    # sampled BREP pass cannot establish continuous clearance or tolerances.
    root = ET.fromstring((DATA/'P2-baseline.xml').read_bytes())
    puck, radius, half_length = _source_cylinder(root.find(".//body[@name='puck']/geom[@name='puck']"))
    if (not math.isclose(radius, candidate['proposal']['puck_outer_radius_mm'])
            or not math.isclose(half_length, candidate['proposal']['puck_half_length_mm'])):
        raise ValueError('Native clearance workpiece differs from pinned source dimensions')
    nests = {name: _source_cylinder(root.find(f".//geom[@name='{name}']"))[0]
             for name in ('nest_A_base', 'nest_A_seat', 'nest_B_base', 'nest_B_seat')}
    shapes = {
        'stator': native.placed_shapes[('tool', 'gripper_brake_stator_envelope')],
        'bridge': native.placed_shapes[('tool', 'gripper_bridge_10')],
    }
    jaws = {'jaw_left': native.placed_shapes[('jaw', 'right_jaw_carriage_0')],
            'jaw_right': native.placed_shapes[('jaw_opposed', 'left_jaw_carriage')]}
    count = min(17, len(frames))
    selected = {round(i * (len(frames)-1)/(count-1)) for i in range(count)}
    for index in range(1, len(frames)):
        if frames[index]['stage'] != frames[index-1]['stage']:
            selected.update((index-1, index))
    before_fault = [index for index, row in enumerate(frames) if row['time_s'] < onset_s]
    if not before_fault:
        raise ValueError('Posed clearance needs a pre-fault pickup and carried pose')
    selected.add(before_fault[-1])
    indices = sorted(selected)
    minimum = candidate['proposal']['outboard_gap_design']
    required = (minimum['required_service_gap_mm'] +
                minimum['first_declared_variation_mm'] +
                minimum['second_declared_variation_mm'] +
                minimum['design_reserve_mm'])
    observations = []
    lip_observations = []
    for index in indices:
        saved = frames[index]
        tool_pose = _frame(saved['bodies']['tool'])
        puck_pose = _frame(saved['bodies']['puck'])
        world_puck = place_local_shape(puck, puck_pose)
        for name, native_shape in shapes.items():
            shape = native_shape.val() if hasattr(native_shape, 'val') else native_shape
            world_shape = place_local_shape(shape, tool_pose)
            volume = float(world_shape.intersect(world_puck).Volume())
            gap = float(world_shape.distance(world_puck))
            if not math.isfinite(volume) or not math.isfinite(gap):
                raise ValueError('Candidate posed native clearance produced nonfinite geometry')
            observations.append(dict(part=name, frame_index=index,
                time_s=saved['time_s'], intersection_mm3=volume,
                distance_mm=gap))
        # Check the *full native carriage*, including its integral lip,
        # against the workpiece and fixed nest solids through pickup and the
        # loaded pre-fault hold. Post-fault carriage/puck touch is intentional
        # capture, so only pre-fault intrusion is a clearance failure.
        if saved['time_s'] < onset_s:
            for body, native_shape in jaws.items():
                shape = native_shape.val() if hasattr(native_shape, 'val') else native_shape
                world_shape = place_local_shape(shape, _frame(saved['bodies'][body]))
                for partner, other in [('puck', world_puck), *nests.items()]:
                    volume = float(world_shape.intersect(other).Volume())
                    gap = float(world_shape.distance(other))
                    if not math.isfinite(volume) or not math.isfinite(gap):
                        raise ValueError('Candidate jaw/nest native clearance produced nonfinite geometry')
                    lip_observations.append(dict(part=body, partner=partner,
                        frame_index=index, time_s=saved['time_s'],
                        intersection_mm3=volume, distance_mm=gap))
    stator = [row for row in observations if row['part'] == 'stator']
    bridge = [row for row in observations if row['part'] == 'bridge']
    maximum_jaw_intrusion = max(row['intersection_mm3'] for row in lip_observations)
    return dict(sampled_frames=len(indices), sample_scope='17 uniform poses plus recorded task-stage boundaries and last pre-fault pose; not a continuous envelope',
        source_puck_radius_mm=radius, source_puck_half_length_mm=half_length,
        required_stator_design_gap_mm=required,
        minimum_stator_gap_mm=min(row['distance_mm'] for row in stator),
        minimum_bridge_gap_mm=min(row['distance_mm'] for row in bridge),
        maximum_stator_intersection_mm3=max(row['intersection_mm3'] for row in stator),
        maximum_bridge_intersection_mm3=max(row['intersection_mm3'] for row in bridge),
        maximum_pre_fault_carriage_puck_nest_intersection_mm3=maximum_jaw_intrusion,
        passed=(min(row['distance_mm'] for row in stator) >= required and
                max(row['intersection_mm3'] for row in observations) <= 1e-5 and
                maximum_jaw_intrusion <= 1e-5),
        observations=observations, pre_fault_carriage_puck_nest_observations=lip_observations)


def _fault_metrics(result, simulation, onset_s):
    frames = result['frames']
    before = [row for row in frames if row['time_s'] < onset_s]
    after = [row for row in frames if row['time_s'] >= onset_s]
    if not before or not after:
        raise ValueError('Fault trace does not cover both sides of the event')
    pre = before[-1]
    pre_y = pre['bodies']['puck']['position_m'][1]
    drop = max(0., max(pre_y-row['bodies']['puck']['position_m'][1] for row in after))
    contact_eligibility = fault_contact_eligibility(before, onset_s)
    pinion = simulation.model.joint('gripper_pinion').id
    left, right = (simulation.model.joint(name).id for name in ('jaw_left', 'jaw_right'))
    addresses = {name: int(simulation.model.jnt_qposadr[index]) for name, index in
                 (('pinion', pinion), ('left', left), ('right', right))}
    brake_rows = [row['rack_brake'] for row in after if row['rack_brake'] is not None]
    max_brake = max((row['capacity_Nm'] for row in brake_rows), default=0.)
    max_native = max((abs(row['native_constraint_torque_Nm']) for row in brake_rows), default=0.)
    lip_names = {'ARX_brake_candidate_jaw_left_capture_ledge',
                 'ARX_brake_candidate_jaw_right_capture_ledge'}
    lip_contacts = [(row['time_s'], contact) for row in after for contact in row['contacts']
        if 'puck' in contact['geoms'] and lip_names.intersection(contact['geoms'])
        and contact['normal_N'] > 0]
    observed_lips = sorted(set().union(*(lip_names.intersection(contact['geoms'])
        for _, contact in lip_contacts)))
    return dict(pre_fault_pad_puck_contact=contact_eligibility['both_pads_contact_at_last_sample'],
        pre_fault_unsupported_workpiece=contact_eligibility['no_external_workpiece_support'],
        pre_fault_contact_eligibility=contact_eligibility,
        pre_fault_puck_height_m=pre_y,
        max_post_fault_puck_drop_m=drop,
        final_jaw_opening_m=[after[-1]['qpos'][addresses[name]] for name in ('left', 'right')],
        final_pinion_angle_rad=after[-1]['qpos'][addresses['pinion']],
        brake_capacity_peak_Nm=max_brake,
        native_brake_torque_peak_Nm=max_native,
        brake_energy_J=(result['metrics']['rack_brake'] or {}).get('dissipated_energy_J'),
        post_fault_lip_contact_geoms=observed_lips,
        post_fault_lip_first_contact_s=lip_contacts[0][0] if lip_contacts else None,
        post_fault_lip_last_contact_s=lip_contacts[-1][0] if lip_contacts else None,
        post_fault_lip_contact_at_final_window=bool(lip_contacts and
            lip_contacts[-1][0] >= after[-1]['time_s']-.05),
        post_fault_lip_peak_normal_N=max((contact['normal_N'] for _, contact in lip_contacts), default=0.),
        final_rack_motor_current_A=after[-1]['rack_motor']['current_A'],
        actual_fault_events=result['events'],
        timestep_s=result['timestep_s'],
        simulation_sha256=digest({key: value for key, value in result.items() if key != 'frames'}))


def verify(output):
    destination = Path(output).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    project = create()
    source_specification = deepcopy(project.specification)
    spec = deepcopy(source_specification)
    spec['gripper_retention_revision'] = CANDIDATE_REVISION
    candidate = candidate_recipe(spec)
    native_dir = destination / 'native'
    project.specification = spec
    source = Path(__file__).parent
    source_files = project_source_identity(source)
    native_receipt, binding, native = construct_project_native(
        project, source, source_files, native_dir, return_native=True)
    physical, control, parameters = project.compile_physics(spec, native_directory=native_dir)
    physical.provenance['native_build_sha256'] = digest(native_receipt)
    physical.provenance['native_source_binding_sha256'] = digest(binding)
    physical.provenance['design_source_files'] = source_files
    reference = lambda t: project.reference(t, parameters)
    reference_source = dict(schema='cybrgeo-arx-brake-candidate-reference/1',
        task_parameters=parameters,
        source_files=source_files,
        native_build_sha256=digest(native_receipt))
    onset_s, duration_s = 2.2, 3.4
    options = dict(motorOffTime=onset_s)
    held = CausalSimulation(physical, control, reference,
        options=options, reference_source=reference_source)
    held_result = held.run(duration_s, output_hz=120)
    free_control = deepcopy(control)
    del free_control['rack']['pinion_brake']
    free = CausalSimulation(physical, free_control, reference,
        options=options, reference_source=reference_source)
    free_result = free.run(duration_s, output_hz=120)
    for result in (held_result, free_result):
        result['task_parameters'] = deepcopy(parameters)
        result['physical_source'] = physical.to_source()
        if digest(result['physical_source']) != result['model_sha256']:
            raise ValueError('Recorded candidate physical source differs from simulated model')
        if result['reference_source']['task_parameters'] != result['task_parameters']:
            raise ValueError('Recorded candidate task parameters differ from simulated reference')
    assert_project_source_identity(source, source_files)
    clearance = _sampled_puck_clearance(native, held_result['frames'], candidate, onset_s)
    held_metrics = _fault_metrics(held_result, held, onset_s)
    free_metrics = _fault_metrics(free_result, free, onset_s)
    fault = dict(held=held_metrics, no_pinion_brake_control=free_metrics,
        comparison_scope='Same fresh native CAD mass, contact proxies and source task; only pinion brake disabled in negative control')
    brake_limit = float(spec['brakes']['small']['holding_torque_Nm'])
    # One millimetre is the source's normal-stop held-payload drift limit,
    # reused as a diagnostic service target, not a released safety rating.
    fault['passed'] = bool(
        held_metrics['pre_fault_pad_puck_contact'] and
        free_metrics['pre_fault_pad_puck_contact'] and
        held_metrics['pre_fault_unsupported_workpiece'] and
        free_metrics['pre_fault_unsupported_workpiece'] and
        bool(held_metrics['post_fault_lip_contact_geoms']) and
        held_metrics['post_fault_lip_contact_at_final_window'] and
        held_metrics['max_post_fault_puck_drop_m'] <= .001 and
        held_metrics['brake_capacity_peak_Nm'] > 0 and
        held_metrics['brake_capacity_peak_Nm'] <= brake_limit+1e-9 and
        held_metrics['native_brake_torque_peak_Nm'] <= brake_limit+1e-9 and
        held_metrics['final_rack_motor_current_A'] == 0 and
        free_metrics['final_rack_motor_current_A'] == 0 and
        len(held_metrics['actual_fault_events']) == 1 and
        held_metrics['actual_fault_events'][0]['reason'] == 'power-loss')
    fault['brake_contribution_established'] = bool(
        held_metrics['max_post_fault_puck_drop_m'] <= .001 and
        free_metrics['max_post_fault_puck_drop_m'] > .001)
    fault['causal_readout'] = ('No-brake control loses the loaded puck; finite pinion brake contributes to retention'
        if fault['brake_contribution_established'] else
        'Brake necessity is unestablished: the integral jaw capture may retain the puck without brake torque')
    for name, result in (('brake_engaged', held_result), ('pinion_brake_disabled', free_result)):
        payload = json.dumps(result, separators=(',', ':'), allow_nan=False).encode('utf-8')
        (destination / f'{name}-trace.json.gz').write_bytes(gzip.compress(payload, mtime=0))
    report = dict(schema='cybrgeo-arx-brake-candidate-diagnostic/1',
        source_commit=candidate['proposal']['source_commit'],
        candidate_revision=CANDIDATE_REVISION,
        source_specification_sha256=digest(source_specification),
        candidate_specification_sha256=digest(spec),
        native_build_sha256=digest(native_receipt),
        native_source_binding_sha256=digest(binding),
        model_sha256=digest(physical.to_source()),
        control_sha256=digest(control),
        brake_candidate_receipt_sha256=hashlib.sha256(
            (native_dir/'gripper-retention-candidate.json').read_bytes()).hexdigest(),
        placed_candidate_part_mass_kg=json.loads(
            (native_dir/'gripper-retention-candidate.json').read_text(encoding='utf-8'))['candidate_part_mass_kg'],
        posed_clearance=clearance, motor_off_loaded_puck=fault,
        traces_sha256={name: hashlib.sha256((destination/name).read_bytes()).hexdigest()
            for name in ('brake_engaged-trace.json.gz', 'pinion_brake_disabled-trace.json.gz')},
        passed_software_diagnostic=bool(clearance['passed'] and fault['passed']),
        manufacturing_qualified=False,
        unresolved_external_evidence=candidate['proposal']['missing_external_evidence'],
        scope='Numerical and sampled-CAD candidate development test only; no continuous clearance, verified supplier fit, brake capacity, fabrication or hardware retention')
    (destination/'candidate-verification.json').write_text(
        json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True,
        help='New or reusable diagnostic output folder; source spec is not edited')
    args = parser.parse_args()
    try:
        report = verify(args.output)
    except Exception as error:
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output/'candidate-verification-error.json').write_text(json.dumps(dict(
            schema='cybrgeo-arx-brake-candidate-error/1', error_type=type(error).__name__,
            error=str(error), traceback=traceback.format_exc(),
            manufacturing_qualified=False), indent=2)+'\n', encoding='utf-8')
        raise
    print(json.dumps(dict(passed_software_diagnostic=report['passed_software_diagnostic'],
        report=str(args.output/'candidate-verification.json'),
        native_build_sha256=report['native_build_sha256']), indent=2))
    if not report['passed_software_diagnostic']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
