"""One named-coordinate electromechanical runtime for authored physical models.

Adapted from ARX belt controller.mjs at aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2.
The original equations live in actuation.py; this host binds them by name to
MuJoCo. Geometry, body count, task policy and joint ordering are not hardcoded.
Reference targets produce forces only. Free-body positions are never written.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict
import gzip
import json
import math
from pathlib import Path

import numpy as np

from .actuation import GearedDCMotor, CurrentBudget, TensionSpring, DelayedBrake, SafetyState
from .physical_model import digest


def _finite(value, name, minimum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    if minimum is not None and value < minimum:
        raise ValueError(f'{name} must be at least {minimum}')
    return float(value)


class CausalSimulation:
    def __init__(self, physical, control, reference, *, options=None, reference_source=None):
        import mujoco as mj
        self.mj, self.physical = mj, physical
        self.physical_source = deepcopy(physical.to_source())
        self.reference_source = deepcopy(reference_source)
        # Fail before integration if a source receipt cannot be serialized.
        self.model_sha256 = digest(self.physical_source)
        self.reference_sha256 = digest(self.reference_source) if self.reference_source is not None else None
        self.control, self.reference = deepcopy(control), reference
        self.model = mj.MjModel.from_xml_string(physical.xml())
        self.data = mj.MjData(self.model)
        self.options = dict(power=True, heartbeat=True, enforceLimits=True, brakes=True,
                            torqueScale=1., gripScale=1., springScale=1., brakeScale=1.,
                            brakeDelayScale=1., frictionScale=1.)
        self.options.update(deepcopy(options or {}))
        scale_keys = {'torqueScale', 'gripScale', 'springScale', 'brakeScale',
                      'brakeDelayScale', 'frictionScale', 'frictionAfterScale'}
        scale_keys.update(row['scale_option'] for row in self.control.get('springs', []) if 'scale_option' in row)
        time_keys = {'frictionChangeTime', 'motorOffTime', 'heartbeatUntil', 'normalStopTime',
                     'estopTime', 'invalidFeedbackTime'}
        allowed = scale_keys | time_keys | {'power', 'heartbeat', 'enforceLimits', 'brakes'}
        # Silently ignoring a perturbation would mislabel a negative control.
        if set(self.options) - allowed:
            raise ValueError(f'Unknown simulation options: {sorted(set(self.options) - allowed)}')
        for key in scale_keys & self.options.keys():
            _finite(self.options[key], key, 0)
        if self.options['brakeDelayScale'] <= 0:
            raise ValueError('brakeDelayScale must be positive')
        for key in time_keys & self.options.keys():
            if self.options[key] is not None:
                _finite(self.options[key], key, 0)
        for key in ('power', 'heartbeat', 'enforceLimits', 'brakes'):
            if not isinstance(self.options[key], bool):
                raise ValueError(f'{key} must be boolean')
        self.initial_options = deepcopy(self.options)
        self.friction_changed = False
        self.drives = self.control['drives']
        if not self.drives:
            raise ValueError('At least one named motor drive is required')
        self.joint_ids = [self.model.joint(row['joint']).id for row in self.drives]
        self.qadr = self.model.jnt_qposadr[self.joint_ids]
        self.vadr = self.model.jnt_dofadr[self.joint_ids]
        self.actuator_ids = [self.model.actuator(row['actuator']).id for row in self.drives]
        if len(set(self.joint_ids)) != len(self.joint_ids):
            raise ValueError('Independent drive joints must be unique')
        if len(set(self.actuator_ids)) != len(self.actuator_ids):
            raise ValueError('Independent motor actuators must be unique')
        if any(self.model.jnt_type[i] != mj.mjtJoint.mjJNT_HINGE for i in self.joint_ids):
            raise ValueError('Rotary motor feedback requires hinge coordinates; linear output requires an explicit transmission')
        for row, joint, actuator in zip(self.drives, self.joint_ids, self.actuator_ids):
            self._check_direct_actuator(actuator, mj.mjtTrn.mjTRN_JOINT, joint, 1.)
            for key in ('kp', 'kd', 'ki', 'integrator_limit'):
                _finite(row[key], key, 0)
        self.motors = [GearedDCMotor(row['motor']) for row in self.drives]
        self.brakes = [DelayedBrake(row['brake']) if row.get('brake') else None for row in self.drives]
        self.safety = SafetyState([row['soft_limits'] for row in self.drives], **self.control.get('safety', {}))
        if any(item['joint'] not in {row['joint'] for row in self.drives} for item in self.control.get('springs', [])):
            raise ValueError('A spring must name a controlled hinge coordinate')
        self.springs = [(self.drives.index(next(row for row in self.drives if row['joint'] == item['joint'])),
                         TensionSpring(item['parameters'], item.get('axis', [0, 0, -1])), item)
                        for item in self.control.get('springs', [])]
        self.base_friction = self.model.dof_frictionloss[self.vadr].copy()
        self.base_pair_friction = self.model.pair_friction.copy()
        self.model.pair_friction[:] = self.base_pair_friction * self.options['frictionScale']
        self.integral = np.zeros(len(self.drives))
        self.mass = np.zeros((self.model.nv, self.model.nv))
        self.brake_capacity = np.zeros(len(self.drives))
        self.brake_energy = np.zeros(len(self.drives))
        self.brake_torque = np.zeros(len(self.drives))
        self.rack = self.control.get('rack')
        if self.rack:
            ids = [self.model.joint(name).id for name in self.rack['joints']]
            if (len(ids) != 2 or len(set(ids)) != 2 or set(ids) & set(self.joint_ids)
                    or any(self.model.jnt_type[i] != mj.mjtJoint.mjJNT_SLIDE for i in ids)):
                raise ValueError('An opposed rack requires two distinct slide joints')
            self.rack_qadr, self.rack_vadr = self.model.jnt_qposadr[ids], self.model.jnt_dofadr[ids]
            self.rack_actuator = self.model.actuator(self.rack['actuator']).id
            if self.rack_actuator in self.actuator_ids:
                raise ValueError('Rack and rotary motors must use distinct actuators')
            for key in ('pinion_pitch_radius_m', 'max_jaw_velocity_m_s', 'position_to_velocity_gain_per_s',
                        'velocity_force_gain_N_s_m', 'output_torque_limit_Nm',
                        'output_torque_per_current_Nm_A', 'no_load_current_A'):
                _finite(self.rack[key], key, 0)
            if self.rack['pinion_pitch_radius_m'] <= 0:
                raise ValueError('Pinion radius must be positive')
            self._check_rack_transmission(ids)
            self.rack_motor = GearedDCMotor(self.rack['motor'])
            self.rack_budget = CurrentBudget(self.rack['current_budget'])
        key = self.control.get('initial_keyframe')
        if key is not None:
            mj.mj_resetDataKeyframe(self.model, self.data, self.model.key(key).id)
        mj.mj_forward(self.model, self.data)
        estimated = self._spring_torques(self.data.qpos[self.qadr], estimated=True)
        for i, motor in enumerate(self.motors):
            cfg = motor.config
            motor.state.current_A = float(np.clip((self.data.qfrc_bias[self.vadr[i]] - estimated[i]) /
                (cfg['gear_ratio'] * cfg['torque_constant_Nm_A']),
                -cfg['peak_current_A'] * self.options['torqueScale'], cfg['peak_current_A'] * self.options['torqueScale']))
        self.steps = 0
        self.stage = 0
        self.desired_q = self.data.qpos[self.qadr].copy()
        self.body_names = [mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_BODY, i) or f'body_{i}' for i in range(self.model.nbody)]
        self.geom_names = [mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_GEOM, i) or f'geom_{i}' for i in range(self.model.ngeom)]
        self.monitor = [dict(peak_current_A=0., peak_output_torque_Nm=0., peak_motor_speed_rad_s=0.,
                             peak_voltage_V=0., over_continuous_current_s=0., over_continuous_gear_torque_s=0.)
                        for _ in self.drives]
        self.free_bodies = {}
        for joint in range(self.model.njnt):
            if self.model.jnt_type[joint] == mj.mjtJoint.mjJNT_FREE:
                body = int(self.model.jnt_bodyid[joint])
                self.free_bodies[self.body_names[body]] = dict(body_id=body, dof=int(self.model.jnt_dofadr[joint]),
                    max_imposed_generalized_force=0., max_applied_wrench_component=0.)

    def _check_direct_actuator(self, actuator, kind, target, gear):
        m, mj = self.model, self.mj
        if (m.actuator_trntype[actuator] != kind or m.actuator_trnid[actuator, 0] != target
                or not np.allclose(m.actuator_gear[actuator], [gear, 0, 0, 0, 0, 0], atol=1e-12, rtol=0)
                or m.actuator_dyntype[actuator] != mj.mjtDyn.mjDYN_NONE
                or m.actuator_gaintype[actuator] != mj.mjtGain.mjGAIN_FIXED
                or m.actuator_gainprm[actuator, 0] != 1
                or m.actuator_biastype[actuator] != mj.mjtBias.mjBIAS_NONE):
            raise ValueError('Electrical output requires a unit-gain direct force actuator with the declared transmission')
        if m.actuator_ctrllimited[actuator] or m.actuator_forcelimited[actuator]:
            raise ValueError('Native actuator clipping would duplicate the declared electrical limits')

    def _check_rack_transmission(self, joint_ids):
        m, mj, actuator = self.model, self.mj, self.rack_actuator
        tendon = int(m.actuator_trnid[actuator, 0])
        self._check_direct_actuator(actuator, mj.mjtTrn.mjTRN_TENDON, tendon, 1.)
        start, count = int(m.tendon_adr[tendon]), int(m.tendon_num[tendon])
        expected = 1 / (2 * self.rack['pinion_pitch_radius_m'])
        if (count != 2 or set(m.wrap_objid[start:start+count]) != set(joint_ids)
                or any(m.wrap_type[start:start+count] != mj.mjtWrap.mjWRAP_JOINT)
                or not np.allclose(m.wrap_prm[start:start+count], expected, rtol=1e-10, atol=1e-12)):
            raise ValueError('Opposed rack tendon must map pinion torque to equal tau/(2*r) jaw forces')

    def _target(self, value):
        for key in ('q', 'qd', 'qdd'):
            vector = np.asarray(value[key], dtype=float)
            if vector.shape != (len(self.drives),) or not np.isfinite(vector).all():
                raise ValueError(f'Reference {key} must contain one finite value per drive')
        if self.rack:
            _finite(value['gap'], 'reference gap')
        return value

    def _spring_torques(self, q, *, estimated=False):
        result = np.zeros(len(self.drives))
        for index, spring, config in self.springs:
            scale = 1. if estimated else self.options.get(config.get('scale_option', 'springScale'), self.options['springScale'])
            result[index] += spring.state(float(q[index]), scale)['torque_Nm']
        return result

    def step(self):
        mj, m, d, o = self.mj, self.model, self.data, self.options
        dt, t = m.opt.timestep, self.steps * m.opt.timestep
        if o.get('frictionChangeTime') is not None and not self.friction_changed and t >= o['frictionChangeTime']:
            o['frictionScale'] = o.get('frictionAfterScale', .01)
            m.pair_friction[:] = self.base_pair_friction * o['frictionScale']
            self.friction_changed = True
        if o.get('motorOffTime') is not None and t >= o['motorOffTime']:
            o['power'] = False
        normal = self._target(self.reference(t))
        q, qd = d.qpos[self.qadr].copy(), d.qvel[self.vadr].copy()
        if o['heartbeat'] and (o.get('heartbeatUntil') is None or t <= o['heartbeatUntil']):
            self.safety.command(t)
        if o.get('normalStopTime') is not None and t >= o['normalStopTime']:
            self.safety.normal_stop(t, q.tolist(), qd.tolist(), gap=normal.get('gap'))
        self.safety.update(t, q.tolist(), qd.tolist(),
            estop=o.get('estopTime') is not None and t >= o['estopTime'], power=o['power'],
            feedback_valid=not (o.get('invalidFeedbackTime') is not None and t >= o['invalidFeedbackTime']),
            limits=o['enforceLimits'])
        powered = o['power'] and self.safety.mode != 'EMERGENCY'
        target = self.safety.reference(t, normal)
        self.stage, self.desired_q = target.get('stage'), np.asarray(target['q'])
        applied = self._spring_torques(q)
        estimated = self._spring_torques(q, estimated=True)
        d.qfrc_applied[:] = 0
        d.qfrc_applied[self.vadr] = applied
        for i, brake in enumerate(self.brakes):
            self.brake_capacity[i] = 0 if brake is None else brake.capacity(t, self.safety.fault_time_s, float(qd[i]),
                delay_scale=o['brakeDelayScale'], strength=o['brakeScale'], enabled=o['brakes'])
            m.dof_frictionloss[self.vadr[i]] = self.base_friction[i] + self.brake_capacity[i]
        if self.control.get('inertial_feedforward', True):
            mj.mj_fullM(m, d, self.mass)
        for i, row in enumerate(self.drives):
            error = float(target['q'][i] - q[i])
            limit = row['integrator_limit']
            self.integral[i] = np.clip(self.integral[i] + error * dt, -limit, limit)
            tau = row['kp'] * error + row['kd'] * (target['qd'][i] - qd[i]) + row['ki'] * self.integral[i]
            if self.control.get('gravity_and_coriolis_feedforward', True):
                tau += d.qfrc_bias[self.vadr[i]] - estimated[i]
            if self.control.get('inertial_feedforward', True):
                tau += sum(self.mass[self.vadr[i], self.vadr[j]] * target['qdd'][j] for j in range(len(self.drives)))
            state = self.motors[i].step(float(tau), float(qd[i]), dt, power=bool(powered), torque_scale=o['torqueScale'])
            d.ctrl[self.actuator_ids[i]] = state['torque_Nm']
        if self.rack:
            r = self.rack
            gap = float(np.mean(d.qpos[self.rack_qadr]))
            speed = float(np.mean(d.qvel[self.rack_vadr]))
            requested_speed = np.clip(r['position_to_velocity_gain_per_s'] * (target['gap'] - gap),
                                      -r['max_jaw_velocity_m_s'], r['max_jaw_velocity_m_s'])
            force = r['velocity_force_gain_N_s_m'] * (requested_speed - speed)
            radius, limit = r['pinion_pitch_radius_m'], r['output_torque_limit_Nm'] * o['gripScale']
            output = float(np.clip(2 * radius * force, -limit, limit))
            gross = output + r['output_torque_per_current_Nm_A'] * r['no_load_current_A'] * np.sign(output)
            ceiling = self.rack_budget.step(self.rack_motor.state.current_A, dt)
            state = self.rack_motor.step(float(gross), speed / radius, dt, power=bool(powered),
                torque_scale=o['gripScale'], current_ceiling_A=ceiling, clamp_actual_current=True)
            d.ctrl[self.rack_actuator] = state['torque_Nm']
        mj.mj_step(m, d)
        self.steps += 1
        self.brake_torque[:] = 0
        for k in range(d.nefc):
            if d.efc_type[k] == mj.mjtConstraint.mjCNSTR_FRICTION_DOF:
                found = np.flatnonzero(self.vadr == d.efc_id[k])
                if len(found):
                    i = int(found[0])
                    if self.brake_capacity[i] > 0:
                        fraction = self.brake_capacity[i] / (self.brake_capacity[i] + self.base_friction[i])
                        self.brake_torque[i] = d.efc_force[k] * fraction
                        self.brake_energy[i] += max(0., -self.brake_torque[i] * (qd[i] + d.qvel[self.vadr[i]]) * .5 * dt)
        if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():
            raise RuntimeError('Nonfinite simulation state')
        for i, (motor, monitor) in enumerate(zip(self.motors, self.monitor)):
            s, cfg = motor.state, motor.config
            for key, value in [('peak_current_A', abs(s.current_A)), ('peak_output_torque_Nm', abs(s.torque_Nm)),
                               ('peak_motor_speed_rad_s', abs(d.qvel[self.vadr[i]]*cfg['gear_ratio'])),
                               ('peak_voltage_V', abs(s.voltage_V))]:
                monitor[key] = max(monitor[key], float(value))
            if abs(s.current_A) > cfg.get('continuous_current_A', math.inf):
                monitor['over_continuous_current_s'] += dt
            if abs(s.torque_Nm) > cfg.get('gear_continuous_torque_Nm', math.inf):
                monitor['over_continuous_gear_torque_s'] += dt
        for record in self.free_bodies.values():
            dof = record['dof']
            record['max_imposed_generalized_force'] = max(record['max_imposed_generalized_force'],
                float(np.max(np.abs(d.qfrc_applied[dof:dof+6]))), float(np.max(np.abs(d.qfrc_actuator[dof:dof+6]))))
            record['max_applied_wrench_component'] = max(record['max_applied_wrench_component'],
                float(np.max(np.abs(d.xfrc_applied[record['body_id']]))))

    def frame(self):
        d = self.data
        # The source validator refreshes only kinematics at capture. This makes
        # body poses agree with post-integration qpos without recomputing bias,
        # constraints or accelerations used by the source's next control step.
        self.mj.mj_kinematics(self.model, d)
        contacts, force = [], np.zeros(6)
        for index in range(d.ncon):
            c = d.contact[index]
            self.mj.mj_contactForce(self.model, d, index, force)
            contacts.append(dict(geoms=[self.geom_names[c.geom1], self.geom_names[c.geom2]],
                distance_m=float(c.dist), normal_N=float(force[0]), tangent_N=float(np.linalg.norm(force[1:3]))))
        return dict(time_s=self.steps * self.model.opt.timestep, qpos=d.qpos.tolist(), qvel=d.qvel.tolist(),
            desired_q=self.desired_q.tolist(), stage=self.stage, mode=self.safety.mode, reason=self.safety.reason,
            bodies={name: dict(position_m=d.xpos[i].tolist(), rotation_row_major=d.xmat[i].tolist())
                    for i, name in enumerate(self.body_names)},
            motors=[asdict(m.state) for m in self.motors], contacts=contacts,
            brake_capacity_Nm=self.brake_capacity.tolist(), brake_torque_Nm=self.brake_torque.tolist(),
            brake_energy_J=self.brake_energy.tolist(),
            rack_motor=asdict(self.rack_motor.state) if self.rack else None,
            external_applied_force_N=d.xfrc_applied.tolist(),
            external_wrench_units=['N', 'N', 'N', 'Nm', 'Nm', 'Nm'])

    def run(self, seconds, *, output_hz=120, output=None):
        _finite(seconds, 'seconds', 0)
        _finite(output_hz, 'output_hz', 0)
        if seconds <= 0 or output_hz <= 0:
            raise ValueError('Positive finite duration and output rate required')
        dt = self.model.opt.timestep
        stride, total = max(1, round(1 / (output_hz * dt))), round(seconds / dt)
        if total < 1:
            raise ValueError('Duration must span at least one integration step')
        start_step = self.steps
        frames = [self.frame()]
        max_tracking = 0.
        max_qfrc = 0.
        max_xfrc = 0.
        for _ in range(total):
            self.step()
            max_tracking = max(max_tracking, float(np.max(np.abs(self.data.qpos[self.qadr] - self.desired_q))))
            max_qfrc = max(max_qfrc, float(np.max(np.abs(self.data.qfrc_applied))))
            max_xfrc = max(max_xfrc, float(np.max(np.abs(self.data.xfrc_applied))))
            if (self.steps - start_step) % stride == 0 or self.steps == start_step + total:
                frames.append(self.frame())
        actuator_metrics = {}
        for row, motor, monitor in zip(self.drives, self.motors, self.monitor):
            state = motor.state
            actuator_metrics[row['actuator']] = dict(deepcopy(monitor),
                rms_current_A=math.sqrt(state.current_squared_integral_A2_s/(self.steps*dt)),
                final_temperature_C=state.temperature_C, electrical_energy_J=state.electrical_energy_J,
                copper_heat_J=state.copper_heat_J, mechanical_energy_J=state.mechanical_energy_J,
                gear_heat_J=state.gear_heat_J, belt_demand_limit_s=state.belt_demand_limit_s,
                gear_demand_limit_s=state.gear_demand_limit_s, voltage_saturation_s=state.voltage_saturation_s)
        result = dict(schema='cybrgeo-causal-simulation/1', model_sha256=self.model_sha256,
            control_sha256=digest(self.control), options=deepcopy(self.initial_options),
            reference_source=deepcopy(self.reference_source), reference_sha256=self.reference_sha256,
            reference_identity='explicit source record' if self.reference_source is not None else 'unbound Python callback',
            final_options=deepcopy(self.options), mujoco_version=self.mj.__version__,
            timestep_s=dt, steps=total, start_time_s=start_step*dt, duration_s=total * dt,
            output_interval_s=stride*dt, frames=frames,
            events=deepcopy(self.safety.events), metrics=dict(max_tracking_error_rad=max_tracking,
            max_applied_generalized_force=max_qfrc, max_external_body_force_N=max_xfrc,
            actuators=actuator_metrics, free_bodies=deepcopy(self.free_bodies),
            monitor_scope='Actuator and free-body extrema cover every integration step since initialization'),
            qualification=dict(dynamics='actual integrated electromechanical/contact model', hardware='not qualified'))
        if output:
            path = Path(output)
            path.mkdir(parents=True, exist_ok=True)
            (path / 'trace.json.gz').write_bytes(gzip.compress(
                json.dumps(result, separators=(',', ':'), allow_nan=False).encode(), mtime=0))
            summary = {key: value for key, value in result.items() if key != 'frames'}
            summary['frames'] = len(frames)
            (path / 'simulation.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
        return result
