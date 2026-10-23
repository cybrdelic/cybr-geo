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
import hashlib
from importlib import metadata
import json
import math
from pathlib import Path
import platform

import numpy as np

from .actuation import GearedDCMotor, CurrentBudget, TensionSpring, DelayedBrake, SafetyState
from .physical_model import digest


def _runtime_identity():
    versions = {}
    for package in ('numpy', 'mujoco'):
        try:
            versions[package] = metadata.version(package)
        except metadata.PackageNotFoundError:
            versions[package] = None
    return dict(schema='cybrgeo-causal-runtime/1',
        source_files={name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                      for name in ('causal_runtime.py', 'actuation.py')},
        packages=versions, runtime=dict(python=platform.python_version(), system=platform.system(), machine=platform.machine()),
        capture='Source files and package versions observed at causal_runtime module import; not the complete Python import closure')


_IMPLEMENTATION_IDENTITY = _runtime_identity()


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
        self.implementation_identity = deepcopy(_IMPLEMENTATION_IDENTITY)
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
        self.base_geom_friction = self.model.geom_friction.copy()
        self.base_pair_friction = self.model.pair_friction.copy()
        self._apply_contact_friction_scale(self.options['frictionScale'])
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
        configured = set(self.actuator_ids) | ({self.rack_actuator} if self.rack else set())
        undeclared = set(range(self.model.nu)) - configured
        if undeclared:
            names = [mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_ACTUATOR, i) or f'actuator_{i}'
                     for i in sorted(undeclared)]
            raise ValueError(f'All model actuators must have declared electrical drives; undeclared: {names}')
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
        self.geom_names = self._object_names(mj.mjtObj.mjOBJ_GEOM, self.model.ngeom, 'geom')
        self.monitor = [dict(peak_current_A=0., peak_output_torque_Nm=0., peak_motor_speed_rad_s=0.,
                             peak_voltage_V=0., over_continuous_current_s=0., over_continuous_gear_torque_s=0.)
                        for _ in self.drives]
        self.free_bodies = {}
        for joint in range(self.model.njnt):
            if self.model.jnt_type[joint] == mj.mjtJoint.mjJNT_FREE:
                body = int(self.model.jnt_bodyid[joint])
                self.free_bodies[self.body_names[body]] = dict(body_id=body, dof=int(self.model.jnt_dofadr[joint]),
                    max_imposed_generalized_force=0., max_applied_wrench_component=0.)
        self.scalar_joints = {}
        self.joint_monitor = {}
        joint_names = self._object_names(mj.mjtObj.mjOBJ_JOINT, self.model.njnt, 'joint')
        for joint in range(self.model.njnt):
            # Tuple membership asks the enum to compare first. Some MuJoCo
            # platform bindings reject NumPy integer scalars in that direction,
            # even when scalar == enum succeeds. Compare Python ints explicitly.
            kind = int(self.model.jnt_type[joint])
            if kind not in (int(mj.mjtJoint.mjJNT_HINGE), int(mj.mjtJoint.mjJNT_SLIDE)):
                continue
            name = joint_names[joint]
            hinge = kind == int(mj.mjtJoint.mjJNT_HINGE)
            self.scalar_joints[name] = (int(self.model.jnt_qposadr[joint]), int(self.model.jnt_dofadr[joint]))
            self.joint_monitor[name] = dict(joint_id=joint, type='hinge' if hinge else 'slide',
                position_unit='rad' if hinge else 'm', speed_unit='rad/s' if hinge else 'm/s',
                direct_motor_actuator=(self.drives[self.joint_ids.index(joint)]['actuator']
                                       if joint in self.joint_ids else None),
                samples=0, minimum_position=None,
                maximum_position=None, peak_absolute_speed=None)
        self._observe_joints(0.)
        self.constraint_monitor = {}
        self.constraint_names = {}
        equality_names = self._object_names(mj.mjtObj.mjOBJ_EQUALITY, self.model.neq, 'equality')
        for equality in range(self.model.neq):
            name = equality_names[equality]
            self.constraint_names[equality] = name
            kind = mj.mjtEq(int(self.model.eq_type[equality])).name.removeprefix('mjEQ_').lower()
            self.constraint_monitor[name] = dict(equality_id=equality, type=kind, sampled_steps=0,
                active_steps=0, inactive_steps=0, missing_row_steps=0, components=[], vector_norms={})
        self.transmission_monitor = [dict(joint=row['joint'], unit='Nm', minimum=None,
            maximum=None, peak_absolute=None, samples=0,
            method='motor output minus reflected dof armature times velocity increment / timestep; '
                   'ideal output torque estimate, not a measured gear or belt stress') for row in self.drives]
        self.contact_monitor = dict(samples=0, initial_state_sampled=True,
            sampling='Initial mj_forward plus each mj_step solver contact evaluation; pre-integration for '
                     'Euler/implicit, last internal stage for RK4. No terminal post-state refresh.')
        self.contact_pairs = {}
        self._observe_contacts(0., 0., initial=True)
        self.rack_monitor = (dict(samples=0, peak_current_A=0., peak_output_torque_Nm=0.,
                                  peak_absolute_current=None) if self.rack else None)

    def _object_names(self, kind, count, prefix):
        names = [self.mj.mj_id2name(self.model, kind, i) for i in range(count)]
        used = set(names) - {None}
        for i, name in enumerate(names):
            if name is None:
                name = f'{prefix}_{i}'
                while name in used:
                    name = '_' + name
                names[i] = name
                used.add(name)
        return names

    @staticmethod
    def _extrema(record, minimum, maximum, peak, event):
        """Keep signed extrema and their first occurrence without retaining a trace."""
        value = event['value']
        if minimum and (record[minimum] is None or value < record[minimum]['value']):
            record[minimum] = event.copy()
        if maximum and (record[maximum] is None or value > record[maximum]['value']):
            record[maximum] = event.copy()
        if peak and (record[peak] is None or abs(value) > abs(record[peak]['value'])):
            record[peak] = event.copy()

    def _observe_joints(self, time_s):
        for name, (qadr, vadr) in self.scalar_joints.items():
            record = self.joint_monitor[name]
            self._extrema(record, 'minimum_position', 'maximum_position', None,
                          dict(value=float(self.data.qpos[qadr]), time_s=time_s))
            self._extrema(record, None, None, 'peak_absolute_speed',
                          dict(value=float(self.data.qvel[vadr]), time_s=time_s))
            record['samples'] += 1

    def _constraint_component(self, equality, component):
        """MuJoCo residual coordinates are not uniformly distances or angles.

        See MuJoCo 3.16 engine_core_constraint.c: connect/weld/joint/tendon/flex.
        Weld's angular residual is torquescale * sin(angle/2) * axis, not angle.
        A fixed tendon may combine angular and linear coordinates via coefficients.
        """
        m, mj = self.model, self.mj
        kind = self.constraint_monitor[self.constraint_names[equality]]['type']
        info = dict(index=component, label=f'component_{component}', unit=None, peak_absolute=None, samples=0)
        if kind == 'connect' or (kind == 'weld' and component < 3):
            info.update(label=f'anchor_delta_{"xyz"[component]}', unit='m')
        elif kind == 'weld':
            info.update(label=f'torquescale_scaled_quaternion_{"xyz"[component-3]}', unit='m',
                        torquescale_m=float(m.eq_data[equality, 10]),
                        interpretation='scaled quaternion vector, not an angle or positional gap')
        elif kind == 'joint':
            joint = int(m.eq_obj1id[equality])
            info.update(label='joint_polynomial_residual',
                        unit='rad' if m.jnt_type[joint] == mj.mjtJoint.mjJNT_HINGE else 'm')
        elif kind == 'tendon':
            tendon = int(m.eq_obj1id[equality])
            start, count = int(m.tendon_adr[tendon]), int(m.tendon_num[tendon])
            fixed = any(m.wrap_type[start:start+count] == mj.mjtWrap.mjWRAP_JOINT)
            info.update(label='tendon_polynomial_residual', unit='tendon_coordinate' if fixed else 'm')
        elif kind == 'flex':
            info.update(label=f'edge_length_residual_{component}', unit='m')
        else:
            info.update(label=f'native_scaled_{kind}_residual_{component}')
        return info

    def _observe_constraints(self, start_time_s, end_time_s):
        # mj_step leaves solver residuals at its dynamics evaluation. For Euler
        # and implicit integrators this precedes state integration. RK4 retains
        # an internal stage: report the interval, without inventing a timestamp.
        rk4 = self.model.opt.integrator == self.mj.mjtIntegrator.mjINT_RK4
        counts, values = {}, {}
        for k in range(self.data.nefc):
            if self.data.efc_type[k] != self.mj.mjtConstraint.mjCNSTR_EQUALITY:
                continue
            equality = int(self.data.efc_id[k])
            record = self.constraint_monitor[self.constraint_names[equality]]
            component = counts.get(equality, 0)
            counts[equality] = component + 1
            if len(record['components']) <= component:
                record['components'].append(self._constraint_component(equality, component))
            row = record['components'][component]
            value = float(self.data.efc_pos[k])
            if not math.isfinite(value):
                raise RuntimeError('Nonfinite equality constraint residual')
            values.setdefault(equality, []).append(value)
            event = dict(value=value,
                         time_s=None if rk4 else start_time_s,
                         interval_start_s=start_time_s, interval_end_s=end_time_s)
            self._extrema(row, None, None, 'peak_absolute', event)
            row['samples'] += 1
        for equality, name in self.constraint_names.items():
            record = self.constraint_monitor[name]
            record['active_steps' if self.data.eq_active[equality] else 'inactive_steps'] += 1
            record['sampled_steps' if equality in counts else 'missing_row_steps'] += 1
            if record['type'] in ('connect', 'weld') and equality in values:
                for start, label in [(0, 'anchor_delta')] + ([(3, 'torquescale_scaled_quaternion')]
                                                            if record['type'] == 'weld' else []):
                    group = record['vector_norms'].setdefault(label, dict(unit='m', peak_absolute=None, samples=0))
                    event = dict(value=float(np.linalg.norm(values[equality][start:start+3])),
                                 time_s=None if rk4 else start_time_s,
                                 interval_start_s=start_time_s, interval_end_s=end_time_s)
                    self._extrema(group, None, None, 'peak_absolute', event)
                    group['samples'] += 1

    def _observe_transmissions(self, previous_speed, start_time_s, end_time_s):
        dt = self.model.opt.timestep
        for i, record in enumerate(self.transmission_monitor):
            acceleration = float((self.data.qvel[self.vadr[i]] - previous_speed[i]) / dt)
            armature = float(self.model.dof_armature[self.vadr[i]])
            torque = float(self.motors[i].state.torque_Nm)
            event = dict(value=torque - armature * acceleration, time_s=end_time_s,
                interval_start_s=start_time_s, motor_output_torque_Nm=torque,
                armature_inertia_kg_m2=armature, acceleration_rad_s2=acceleration)
            self._extrema(record, 'minimum', 'maximum', 'peak_absolute', event)
            record['samples'] += 1

    def _observe_contacts(self, start_time_s, end_time_s, *, initial=False):
        rk4 = self.model.opt.integrator == self.mj.mjtIntegrator.mjINT_RK4
        seen = set()
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            endpoints = tuple(sorted((int(geom), int(flex)) for geom, flex in zip(contact.geom, contact.flex)))
            if endpoints not in self.contact_pairs:
                names = [self.geom_names[geom] if geom >= 0 else
                         (self.mj.mj_id2name(self.model, self.mj.mjtObj.mjOBJ_FLEX, flex) or f'flex_{flex}')
                         for geom, flex in endpoints]
                self.contact_pairs[endpoints] = dict(geom_ids=[row[0] for row in endpoints],
                    flex_ids=[row[1] for row in endpoints], geoms=names, contact_samples=0,
                    initial_contacts=0, steps_with_contacts=0, maximum_penetration_m=0., peak_event=None)
            row = self.contact_pairs[endpoints]
            distance = float(contact.dist)
            if not math.isfinite(distance):
                raise RuntimeError('Nonfinite native contact distance')
            event = dict(value=max(0., -distance), signed_distance_m=distance,
                time_s=None if rk4 and not initial else start_time_s,
                interval_start_s=start_time_s, interval_end_s=end_time_s,
                sampling='initial-forward' if initial else 'integration-solver')
            self._extrema(row, None, None, 'peak_event', event)
            row['maximum_penetration_m'] = row['peak_event']['value']
            row['contact_samples'] += 1
            row['initial_contacts'] += int(initial)
            seen.add(endpoints)
        if not initial:
            self.contact_monitor['samples'] += 1
            for key in seen:
                self.contact_pairs[key]['steps_with_contacts'] += 1

    def _observe_rack(self, time_s):
        if self.rack_monitor is not None:
            row, state = self.rack_monitor, self.rack_motor.state
            row['samples'] += 1
            row['peak_current_A'] = max(row['peak_current_A'], abs(state.current_A))
            row['peak_output_torque_Nm'] = max(row['peak_output_torque_Nm'], abs(state.torque_Nm))
            self._extrema(row, None, None, 'peak_absolute_current', dict(value=state.current_A, time_s=time_s))

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

    def _apply_contact_friction_scale(self, scale):
        # MuJoCo uses geom coefficients for implicit contacts and pair
        # coefficients for explicit pairs. Always scale the original arrays;
        # a scheduled change replaces the scale rather than compounding it.
        self.model.geom_friction[:] = self.base_geom_friction * scale
        self.model.pair_friction[:] = self.base_pair_friction * scale

    def step(self):
        mj, m, d, o = self.mj, self.model, self.data, self.options
        dt, t = m.opt.timestep, self.steps * m.opt.timestep
        if o.get('frictionChangeTime') is not None and not self.friction_changed and t >= o['frictionChangeTime']:
            o['frictionScale'] = o.get('frictionAfterScale', .01)
            self._apply_contact_friction_scale(o['frictionScale'])
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
        self._observe_joints(self.steps * dt)
        self._observe_constraints(t, self.steps * dt)
        self._observe_transmissions(qd, t, self.steps * dt)
        self._observe_contacts(t, self.steps * dt)
        self._observe_rack(self.steps * dt)
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
        for row, motor, monitor, transmission in zip(self.drives, self.motors, self.monitor, self.transmission_monitor):
            state = motor.state
            actuator_metrics[row['actuator']] = dict(deepcopy(monitor),
                rms_current_A=math.sqrt(state.current_squared_integral_A2_s/(self.steps*dt)),
                final_temperature_C=state.temperature_C, electrical_energy_J=state.electrical_energy_J,
                copper_heat_J=state.copper_heat_J, mechanical_energy_J=state.mechanical_energy_J,
                gear_heat_J=state.gear_heat_J, belt_demand_limit_s=state.belt_demand_limit_s,
                gear_demand_limit_s=state.gear_demand_limit_s, voltage_saturation_s=state.voltage_saturation_s,
                transmitted_torque_estimate=deepcopy(transmission))
        result = dict(schema='cybrgeo-causal-simulation/1', model_sha256=self.model_sha256,
            runtime_implementation=deepcopy(self.implementation_identity),
            control_sha256=digest(self.control), options=deepcopy(self.initial_options),
            reference_source=deepcopy(self.reference_source), reference_sha256=self.reference_sha256,
            reference_identity='explicit source record' if self.reference_source is not None else 'unbound Python callback',
            final_options=deepcopy(self.options), mujoco_version=self.mj.__version__,
            timestep_s=dt, steps=total, start_time_s=start_step*dt, duration_s=total * dt,
            output_interval_s=stride*dt, frames=frames,
            events=deepcopy(self.safety.events), metrics=dict(max_tracking_error_rad=max_tracking,
            max_applied_generalized_force=max_qfrc, max_external_body_force_N=max_xfrc,
            actuators=actuator_metrics, free_bodies=deepcopy(self.free_bodies),
            joints=deepcopy(self.joint_monitor), constraints=deepcopy(self.constraint_monitor),
            contacts=dict(deepcopy(self.contact_monitor), pairs=deepcopy(list(self.contact_pairs.values()))),
            rack_motor=deepcopy(self.rack_monitor),
            monitoring_interval=dict(start_time_s=0., end_time_s=self.steps*dt, integration_steps=self.steps),
            constraint_sampling='Native solver residuals from every mj_step; pre-integration for Euler/implicit, '
                'last internal stage for RK4. Missing rows are not zero residuals. No terminal post-state refresh.',
            monitor_scope='Rotary/rack actuator, transmission, joint, constraint, contact and free-body extrema '
                'cover every integration step since initialization; joints and contacts also include the initial state'),
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
