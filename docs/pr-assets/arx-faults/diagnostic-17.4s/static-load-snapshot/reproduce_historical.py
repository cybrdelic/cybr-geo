"""Reproduce a sampled static-load diagnosis of the preserved P5 failure.

Run from the repository root in the documented simulation environment:
  python outputs/fault-diagnosis/p5-j4-static-load/reproduce_historical.py

No time integration, source mutation, acceptance-threshold change or physical
brake uprating is performed. The saved native model supplies every mass tensor.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import gzip
import hashlib
from importlib import metadata
import json
from pathlib import Path
import platform
import sys
import types


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_json(path):
    raw = path.read_bytes()
    return json.loads(gzip.decompress(raw) if path.suffix == '.gz' else raw)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repository', type=Path, default=Path.cwd())
    parser.add_argument('--suite', type=Path, default=Path('outputs/engineering-arx-faults-final'))
    parser.add_argument('--output', type=Path,
                        default=Path('outputs/fault-diagnosis/p5-j4-static-load/receipt-historical.json'))
    args = parser.parse_args()
    repo = args.repository.resolve()
    suite = (repo / args.suite).resolve()
    output = (repo / args.output).resolve()
    sys.path[:0] = [str(repo / 'src'), str(repo)]
    import numpy as np
    import mujoco as mj

    inputs_path = suite / 'inputs.json'
    inputs = read_json(inputs_path)
    task_path = Path(__file__).with_name('source_task.py')
    if sha(task_path) != inputs['source_files']['task.py']:
        raise ValueError('Bundled historical source task differs from the preserved suite')
    def historical_module(name):
        path = Path(__file__).with_name('source_' + name + '.py')
        if sha(path) != inputs['engine_files_sha256'][name + '.py']:
            raise ValueError(f'{name} snapshot differs from the preserved implementation')
        module = types.ModuleType('preserved_diagnosis_' + name)
        module.__file__ = str(path)
        sys.modules[module.__name__] = module
        exec(compile(path.read_text(encoding='utf-8'), str(path), 'exec'), module.__dict__)
        return module
    historical_physical = historical_module('physical_model')
    PhysicalModel, digest = historical_physical.PhysicalModel, historical_physical.digest
    PhaseClock = historical_module('reference_clock').PhaseClock
    task_text = task_path.read_text(encoding='utf-8')
    source_task = types.ModuleType('preserved_arx_source_task')
    exec(compile(task_text, str(task_path), 'exec'), source_task.__dict__)
    needle = 'direction_sign * .34 * hump'
    if task_text.count(needle) != 1:
        raise ValueError('The exact source direction expression is not unique')
    variant_text = task_text.replace(needle, 'direction_sign * .34 * hump * 0.0')
    zero_lateral = types.ModuleType('diagnostic_zero_lateral_task')
    exec(compile(variant_text, '<diagnostic-zero-lateral-in-memory>', 'exec'), zero_lateral.__dict__)

    physical = PhysicalModel.from_source(inputs['physical_source'])
    model_id = digest(physical.to_source())
    control_id = digest(inputs['control'])
    model = mj.MjModel.from_xml_string(physical.xml())
    data = mj.MjData(model)
    parameters = inputs['task_parameters']
    clock = PhaseClock.from_source(parameters['reference_clock'])
    joint_ids = [model.joint(f'j{i+1}').id for i in range(6)]
    qadr, vadr = model.jnt_qposadr[joint_ids], model.jnt_dofadr[joint_ids]
    soft = np.asarray([row['soft_limits'] for row in inputs['control']['drives']])
    hard = model.jnt_range[joint_ids].copy()
    if not model.nkey:
        raise ValueError('Expected the bound source home keyframe')
    mj.mj_resetDataKeyframe(model, data, model.key('home').id)
    home = data.qpos.copy()
    brake = inputs['control']['drives'][3]['brake']
    friction = float(model.dof_frictionloss[vadr[3]])
    dynamic_capacity = brake['screening_dynamic_torque_Nm'] + friction

    trace_names = ('nominal--same-clock-independent', 'power_loss_7.2--matched-phase',
                   'weak_spring_double_delay_7.2--matched-phase')
    traces, identities = {}, {}
    for name in trace_names:
        path = suite / name / 'trace.json.gz'
        trace = read_json(path)
        if trace['model_sha256'] != model_id or trace['control_sha256'] != control_id:
            raise ValueError(f'{name} does not use the bound model/control')
        traces[name] = trace
        identities[name] = dict(path=path.relative_to(repo).as_posix(), sha256=sha(path),
            model_sha256=trace['model_sha256'], control_sha256=trace['control_sha256'],
            reference_sha256=trace['reference_sha256'], steps=trace['steps'],
            timestep_s=trace['timestep_s'], runtime_implementation=trace['runtime_implementation'])
    if len({trace['reference_sha256'] for trace in traces.values()}) != 1:
        raise ValueError('Compared traces do not share one reference')

    def gravity(qpos):
        data.qpos[:] = qpos
        data.qvel[:] = 0
        mj.mj_forward(model, data)
        return data.qfrc_bias[vadr].copy()

    def desired_qpos(q):
        state = home.copy()
        state[qadr] = q
        return state

    fault_comparison = {}
    for name in trace_names[1:]:
        trace = traces[name]
        observation = trace['source_fault_observation']
        fault_time = observation['time_s']
        frames = [f for f in trace['frames'] if f['time_s'] >= fault_time]
        timeline = []
        for offset in (0., .025, .05, .1, .2, .3, .4, .5, .6):
            frame = min(frames, key=lambda f: abs(f['time_s'] - fault_time - offset))
            load = float(gravity(frame['qpos'])[3])
            timeline.append(dict(requested_offset_s=offset, time_s=frame['time_s'],
                offset_s=frame['time_s']-fault_time, q_rad=[frame['qpos'][i] for i in qadr],
                qd_rad_s=[frame['qvel'][i] for i in vadr], J4_gravity_Nm=load,
                J4_dynamic_static_load_margin_Nm=dynamic_capacity-abs(load),
                recorded_brake_capacity_Nm=frame['brake_capacity_Nm'][3],
                recorded_brake_torque_Nm=frame['brake_torque_Nm'][3]))
        peak = max(frames, key=lambda f: abs(f['qvel'][vadr[3]]))
        settled = next((f for i, f in enumerate(frames)
                        if all(abs(g['qvel'][vadr[3]]) < .002 for g in frames[i:])), None)
        pad_times = [f['time_s']-fault_time for f in frames
                     if any(set(c['geoms']) & {'pad_left', 'pad_right'} for c in f['contacts'])]
        checks_path = suite / name / 'checks.json'
        checks = read_json(checks_path)
        fault_comparison[name] = dict(actual_fault_time_s=fault_time,
            mapped_source_phase_s=clock.source_time(fault_time),
            saved_sample_peak_J4_speed_rad_s=peak['qvel'][vadr[3]],
            saved_sample_peak_offset_s=peak['time_s']-fault_time,
            saved_sample_settling_offset_s=None if settled is None else settled['time_s']-fault_time,
            saved_sample_final_travel_deg=float((observation['q_before'][3]-frames[-1]['qpos'][qadr[3]])*180/np.pi),
            J4_recorded_brake_energy_J=frames[-1]['brake_energy_J'][3],
            last_saved_pad_contact_offset_s=max(pad_times) if pad_times else None,
            source_acceptance_passed=checks['passed'], checks_sha256=sha(checks_path),
            source_settling_check=next(c for c in checks['checks'] if c['name']=='all joints settle below 0.002 rad/s'),
            gravity_timeline=timeline)

    ordinary = traces[trace_names[1]]
    fault_time = ordinary['source_fault_observation']['time_s']
    fault_frame = min(ordinary['frames'], key=lambda f: abs(f['time_s']-fault_time))
    nominal_frame = min(traces[trace_names[0]]['frames'], key=lambda f: abs(f['time_s']-fault_time))
    source_q7 = np.asarray(source_task.reference(7.2, parameters)['q'])
    transfer_times = np.r_[np.linspace(1.65, 2.75, 111), np.linspace(6.65, 7.75, 111)]
    cycle_times = np.linspace(0, 10, 1001)

    def roll_q(t, coefficient):
        q = np.asarray(source_task.reference(float(t), parameters)['q'])
        progress = source_task._phase(t % 5, 1.65, 2.75)
        hump = 16*progress**2*(1-progress)**2
        q[5] += (1 if t % 10 < 5 else -1)*(coefficient-.5)*hump
        return q

    def screen(callback):
        sampled = []
        for t in transfer_times:
            q = callback(float(t))
            loads = gravity(desired_qpos(q))
            sampled.append(dict(source_time_s=float(t), q_rad=q.tolist(), gravity_Nm=loads.tolist()))
        qs = np.asarray([callback(float(t)) for t in cycle_times])
        worst = max(sampled, key=lambda row: abs(row['gravity_Nm'][3]))
        return dict(transfer_sample_count=len(sampled), worst_J4_sample=worst,
            worst_abs_J4_gravity_Nm=abs(worst['gravity_Nm'][3]),
            worst_sample_J4_margin_Nm=dynamic_capacity-abs(worst['gravity_Nm'][3]),
            sampled_min_soft_margin_per_joint_rad=np.min(np.minimum(qs-soft[:,0], soft[:,1]-qs), axis=0).tolist(),
            sampled_min_hard_margin_per_joint_rad=np.min(np.minimum(qs-hard[:,0], hard[:,1]-qs), axis=0).tolist(),
            sampled_q_min_rad=qs.min(axis=0).tolist(), sampled_q_max_rad=qs.max(axis=0).tolist())

    rolls = []
    for coefficient in (-.3, -.2, -.1, 0., .1, .2, .3, .4, .5, .6):
        row = screen(lambda t: roll_q(t, coefficient))
        actual = np.asarray(fault_frame['qpos']).copy()
        candidate = roll_q(7.2, coefficient)
        actual[qadr[5]] += candidate[5]-source_q7[5]
        row.update(coefficient=coefficient,
            desired_phase_7_2_J4_gravity_Nm=float(gravity(desired_qpos(candidate))[3]),
            actual_fault_pose_only_roll_changed_J4_gravity_Nm=float(gravity(actual)[3]))
        rolls.append(row)
    full_roll = []
    for q6 in np.linspace(soft[5,0], soft[5,1], 191):
        actual = np.asarray(fault_frame['qpos']).copy()
        actual[qadr[5]] = q6
        full_roll.append(dict(q6_rad=float(q6), J4_gravity_Nm=float(gravity(actual)[3])))
    lateral = screen(lambda t: np.asarray(zero_lateral.reference(t, parameters)['q']))
    lateral.update(name='zero lateral tilt, forward tilt .15 and source roll .5 retained',
        source_replacement=dict(original=needle, replacement='direction_sign * .34 * hump * 0.0', occurrences=1),
        in_memory_task_utf8_sha256=hashlib.sha256(variant_text.encode('utf-8')).hexdigest(),
        desired_phase_7_2_J4_gravity_Nm=float(gravity(desired_qpos(zero_lateral.reference(7.2, parameters)['q']))[3]))

    receipt = dict(schema='cybrgeo-sampled-static-fault-diagnosis/1',
        reproduction=dict(script=Path(__file__).resolve().relative_to(repo).as_posix(),
            script_sha256=sha(Path(__file__)), command='python outputs/fault-diagnosis/p5-j4-static-load/reproduce_historical.py',
            python=platform.python_version(), system=platform.system(),
            packages={name:metadata.version(name) for name in ('numpy','mujoco')}),
        source=dict(inputs_path=inputs_path.relative_to(repo).as_posix(), inputs_sha256=sha(inputs_path),
            model_sha256=model_id, control_sha256=control_id,
            source_task_path=task_path.resolve().relative_to(repo).as_posix(), source_task_sha256=sha(task_path),
            snapshot_helper_sha256={name:sha(Path(__file__).with_name('source_'+name)) for name in ('physical_model.py','reference_clock.py')},
            pinned_ARX_commit=source_task.SOURCE_COMMIT,
            native_build_sha256=inputs['native_build_sha256'],
            native_source_binding_sha256=inputs['native_source_binding_sha256'], trace_files=identities),
        method=dict(scope='Sampled static gravity and saved-trace readback, not a new dynamic run, continuous bound or hardware qualification',
            evaluated_gravity='MuJoCo qfrc_bias with qvel zero at declared qpos; this excludes contact-transmitted free-puck loads, motor/brake torques and dynamic inertial demand',
            actual_state_source='Exact saved ordinary/weak fault qpos; only qvel zeroed for gravity; roll sweep changes only q6',
            desired_state_source='Bound home keyframe for non-arm coordinates; six arm q overwritten by unchanged source task or declared in-memory variation; qvel zero',
            transfer_times_s=transfer_times.tolist(), full_cycle_range_samples=dict(start_s=0., end_s=10., count=1001),
            no_unsampled_bound_claim=True, no_source_files_modified=True,
            brake_dynamic_rating_status=brake['dynamic_torque_status']),
        J4_capacity=dict(dynamic_brake_Nm=brake['screening_dynamic_torque_Nm'], dry_friction_Nm=friction,
            total_dynamic_brake_plus_friction_Nm=dynamic_capacity, holding_brake_Nm=brake['holding_torque_Nm'],
            speed_switch_rad_s=.01, delay_s=brake['delay_s'], ramp_s=brake['ramp_s']),
        nominal_fault_prefix_at_saved_fault_frame=dict(nominal_time_s=nominal_frame['time_s'], fault_time_s=fault_frame['time_s'],
            max_abs_qpos_difference=float(np.max(abs(np.asarray(nominal_frame['qpos'])-fault_frame['qpos']))),
            max_abs_qvel_difference=float(np.max(abs(np.asarray(nominal_frame['qvel'])-fault_frame['qvel'])))),
        fault_comparison=fault_comparison, roll_only_coefficient_screen=rolls,
        allowed_roll_samples_at_actual_fault_pose=full_roll,
        minimum_sampled_roll_load=min(full_roll,key=lambda r:abs(r['J4_gravity_Nm'])),
        zero_lateral_candidate=lateral)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(receipt, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps(dict(receipt=str(output), sha256=sha(output), model_sha256=model_id,
        best_roll=receipt['minimum_sampled_roll_load'],
        zero_lateral_worst_J4_Nm=lateral['worst_abs_J4_gravity_Nm'],
        zero_lateral_worst_source_s=lateral['worst_J4_sample']['source_time_s'],
        zero_lateral_sampled_min_margin_Nm=lateral['worst_sample_J4_margin_Nm'])))


if __name__ == '__main__':
    main()
