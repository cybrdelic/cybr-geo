"""Reusable electromechanical kernels ported from the user's ARX P5 source.

Source: aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2,
ARX-6-P5-Engineering/integration/{controller,safety}.mjs. Original SI field
names and equations are retained. This module has no geometry, joint-count,
trajectory, MuJoCo-model or named-body assumptions. It is not safety rated.

The original arm model limits requested current/torque, then integrates the
winding RL equation. Actual current/torque can exceed those request limits
during transients. Only the gripper model adds an actual-current clamp. These
distinct behaviors are preserved rather than silently improving the source.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass
import math


SOURCE_PROVENANCE = {
    'repository_commit': 'aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2',
    'controller.mjs_sha256': '6c4bf36dfc8cbc034a81eef2930d5287b1cd09650e55b2f86dae04a44e8aa9de',
    'safety.mjs_sha256': '10e967f2efd33ccb3ad547d18dfea57548aed2ebb7b0f4eebcc426f68eff06c5',
    'scope': 'Source-derived equation port; no vendor or hardware qualification',
}


def _number(value, name, minimum=None, strictly_positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    value = float(value)
    if (minimum is not None and value < minimum) or (strictly_positive and value <= 0):
        raise ValueError(f'{name} is outside its physical domain')
    return value


def _vector(value, name, length=3, finite=True):
    try:
        result = list(value)
    except TypeError as exc:
        raise ValueError(f'{name} requires {length} coordinates') from exc
    if isinstance(value, (str, bytes, dict)) or len(result) != length:
        raise ValueError(f'{name} requires {length} coordinates')
    return [_number(v, name) for v in result] if finite else result


def _clip(value, low, high):
    return max(low, min(high, value))


def _sign(value):
    return 0 if value == 0 else math.copysign(1, value)


@dataclass
class MotorState:
    current_A: float = 0.
    temperature_C: float = 25.
    voltage_V: float = 0.
    torque_Nm: float = 0.
    requested_current_A: float = 0.
    desired_torque_Nm: float = 0.
    limited_demand_Nm: float = 0.
    electrical_energy_J: float = 0.
    copper_heat_J: float = 0.
    mechanical_energy_J: float = 0.
    gear_heat_J: float = 0.
    current_squared_integral_A2_s: float = 0.
    belt_demand_limit_s: float = 0.
    gear_demand_limit_s: float = 0.
    voltage_saturation_s: float = 0.


class GearedDCMotor:
    """One source-equivalent winding and output transmission, entirely in SI.

    ``config`` retains the source's scalar motor field names; extract any row
    from the source motor arrays before constructing an instance. Optional
    ``back_emf_V_s_rad`` defaults to Kt, as in the arm source. For its gripper,
    pass the explicit Ke and use ``clamp_actual_current=True`` with the current
    ceiling from CurrentBudget. Rotor/reflected inertias belong in the physics
    model and must not be counted again here.
    """
    def __init__(self, config, state=None):
        self.config = deepcopy(config)
        required = ('gear_ratio', 'gear_efficiency', 'torque_constant_Nm_A',
                    'resistance_ohm', 'inductance_H', 'bus_voltage_V',
                    'peak_current_A', 'gear_efficiency_transition_motor_rad_s',
                    'thermal_resistance_K_W', 'thermal_capacity_J_K')
        for key in required:
            _number(self.config[key], key, strictly_positive=True)
        if self.config['gear_efficiency'] > 1:
            raise ValueError('gear_efficiency must be within (0,1]')
        duty = _number(self.config.setdefault('max_pwm_duty', 1.), 'max_pwm_duty', 0)
        if duty > 1:
            raise ValueError('max_pwm_duty must be within [0,1]')
        for key in ('gear_output_torque_limit_Nm', 'belt_allowable_output_torque_Nm'):
            if key in self.config:
                _number(self.config[key], key, minimum=0)
        _number(self.config.setdefault('back_emf_V_s_rad', self.config['torque_constant_Nm_A']),
                'back_emf_V_s_rad', minimum=0)
        ambient = _number(self.config.setdefault('ambient_temperature_C', 25.), 'ambient_temperature_C')
        self.state = deepcopy(state) if state is not None else MotorState(temperature_C=ambient)
        for key, value in asdict(self.state).items():
            _number(value, key)

    def step(self, demand_Nm, speed_rad_s, dt, *, power=True, torque_scale=1.,
             current_ceiling_A=None, clamp_actual_current=False):
        dt = _number(dt, 'dt', strictly_positive=True)
        demand = _number(demand_Nm, 'demand_Nm')
        speed = _number(speed_rad_s, 'speed_rad_s')
        multiplier = _number(torque_scale, 'torque_scale', minimum=0)
        if not isinstance(power, bool) or not isinstance(clamp_actual_current, bool):
            raise ValueError('power and clamp_actual_current must be boolean')
        c, s = self.config, self.state
        N, eta, kt = c['gear_ratio'], c['gear_efficiency'], c['torque_constant_Nm_A']
        R, L, V = c['resistance_ohm'], c['inductance_H'], c['bus_voltage_V'] * c['max_pwm_duty']
        limit = (c['peak_current_A'] if current_ceiling_A is None else
                 _number(current_ceiling_A, 'current_ceiling_A', minimum=0)) * multiplier
        rotor_speed = N * speed
        transition = math.tanh(rotor_speed / c['gear_efficiency_transition_motor_rad_s'])
        gear_limit = c['gear_output_torque_limit_Nm'] * multiplier if 'gear_output_torque_limit_Nm' in c else math.inf
        belt_limit = c.get('belt_allowable_output_torque_Nm', math.inf)
        s.desired_torque_Nm = demand
        s.belt_demand_limit_s += dt if abs(demand) > belt_limit else 0
        s.gear_demand_limit_s += dt if abs(demand) > gear_limit else 0
        s.limited_demand_Nm = _clip(demand, -min(gear_limit, belt_limit), min(gear_limit, belt_limit))
        efficiency = math.exp(math.log(eta) * transition * _sign(s.limited_demand_Nm))
        s.requested_current_A = _clip(s.limited_demand_Nm / (N * efficiency * kt), -limit, limit)
        emf = c['back_emf_V_s_rad'] * rotor_speed
        s.voltage_V = _clip(R * s.requested_current_A + emf, -V, V)
        if not power:
            # Source power-loss assumption: current is reset, not a flyback or
            # regenerative drive model. Keep this limitation visible to callers.
            s.voltage_V = s.current_A = 0.
        else:
            equilibrium = (s.voltage_V - emf) / R
            s.current_A = equilibrium + (s.current_A - equilibrium) * math.exp(-R * dt / L)
        if clamp_actual_current:
            s.current_A = _clip(s.current_A, -limit, limit)
        rotor_torque = kt * s.current_A
        actual_efficiency = math.exp(math.log(eta) * transition * _sign(rotor_torque))
        s.torque_Nm = N * rotor_torque * actual_efficiency
        copper = s.current_A * s.current_A * R
        loss = rotor_torque * rotor_speed - s.torque_Nm * speed
        s.temperature_C += (copper - (s.temperature_C - c['ambient_temperature_C']) /
                            c['thermal_resistance_K_W']) * dt / c['thermal_capacity_J_K']
        s.electrical_energy_J += s.voltage_V * s.current_A * dt
        s.copper_heat_J += copper * dt
        s.mechanical_energy_J += s.torque_Nm * speed * dt
        s.gear_heat_J += loss * dt
        s.current_squared_integral_A2_s += s.current_A * s.current_A * dt
        s.voltage_saturation_s += dt if abs(s.voltage_V) >= .999 * V else 0
        return asdict(s)


class CurrentBudget:
    """Source gripper current-budget policy, generalized without joint names."""
    def __init__(self, config, *, recovery_fraction=.8, tolerance_A=1e-5):
        self.continuous = _number(config['continuous_current_A'], 'continuous_current_A', strictly_positive=True)
        self.peak = _number(config['peak_current_A'], 'peak_current_A', minimum=self.continuous)
        self.budget = _number(config['peak_budget_s'], 'peak_budget_s', minimum=0)
        self.credit_s = self.budget
        self.recovery_fraction = _number(recovery_fraction, 'recovery_fraction', minimum=0)
        if self.recovery_fraction > 1:
            raise ValueError('recovery_fraction must be within [0,1]')
        self.tolerance_A = _number(tolerance_A, 'tolerance_A', minimum=0)

    def step(self, previous_current_A, dt):
        current = abs(_number(previous_current_A, 'previous_current_A'))
        dt = _number(dt, 'dt', strictly_positive=True)
        if current > self.continuous + self.tolerance_A:
            self.credit_s = max(0., self.credit_s - dt)
        elif current < self.recovery_fraction * self.continuous:
            self.credit_s = min(self.budget, self.credit_s + dt)
        return self.peak if self.credit_s > 0 else self.continuous


class TensionSpring:
    """Hookean tension-only spring(s) between moving anchors, about any axis.

    Anchors and axis share the joint frame, whose origin is its pivot. The
    default -Z axis reproduces the ARX shoulder/elbow source exactly. A full
    parent-child spatial spring can call this kernel after resolving anchors
    into that frame; no external support or position lock is introduced.
    """
    def __init__(self, config, axis=(0, 0, -1)):
        self.config = deepcopy(config)
        self.parent = _vector(config['parent_anchor_m'], 'parent_anchor_m')
        self.child = _vector(config['child_anchor_m'], 'child_anchor_m')
        self.axis = _vector(axis, 'axis')
        length = math.sqrt(sum(v * v for v in self.axis))
        if length < 1e-12:
            raise ValueError('Spring joint axis cannot be zero')
        self.axis = [v / length for v in self.axis]
        for key in ('spring_count', 'free_eye_length_m', 'initial_tension_N', 'stiffness_N_m'):
            _number(config[key], key, minimum=0)

    def state(self, q, strength=1.):
        q = _number(q, 'joint angle')
        strength = _number(strength, 'spring strength', minimum=0)
        a, b, c = self.axis, self.child, self.config
        dot = sum(x * y for x, y in zip(a, b))
        cross = [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
        moving = [math.cos(q)*b[i] + math.sin(q)*cross[i] + (1-math.cos(q))*a[i]*dot for i in range(3)]
        delta = [x-y for x, y in zip(self.parent, moving)]
        length = math.sqrt(sum(v*v for v in delta))
        if length < 1e-12:
            raise ValueError('Coincident spring anchors have no defined force direction')
        extension = max(0., length-c['free_eye_length_m'])
        force = strength*(c['initial_tension_N']+c['stiffness_N_m']*extension) if length >= c['free_eye_length_m'] else 0.
        f = [force*v/length for v in delta]
        moment = [moving[1]*f[2]-moving[2]*f[1], moving[2]*f[0]-moving[0]*f[2], moving[0]*f[1]-moving[1]*f[0]]
        torque = c['spring_count']*sum(x*y for x, y in zip(a, moment))
        energy = c['spring_count']*strength*(c['initial_tension_N']*extension+.5*c['stiffness_N_m']*extension*extension)
        return dict(length_m=length, force_each_N=force, torque_Nm=torque, potential_energy_J=energy)


class DelayedBrake:
    """Delayed finite friction capacity; caller applies it as a native constraint."""
    def __init__(self, config, speed_threshold_rad_s=.01):
        self.config = deepcopy(config)
        for key in ('delay_s', 'holding_torque_Nm', 'screening_dynamic_torque_Nm'):
            _number(config[key], key, minimum=0)
        _number(config['ramp_s'], 'ramp_s', strictly_positive=True)
        self.speed_threshold = _number(speed_threshold_rad_s, 'speed threshold', minimum=0)

    def fraction(self, time_s, fault_time_s, delay_scale=1.):
        time_s = _number(time_s, 'time_s')
        scale = _number(delay_scale, 'delay_scale', strictly_positive=True)
        if fault_time_s is None:
            return 0.
        fault = _number(fault_time_s, 'fault_time_s')
        return _clip((time_s-fault-self.config['delay_s']*scale)/(self.config['ramp_s']*scale), 0., 1.)

    def capacity(self, time_s, fault_time_s, speed_rad_s, *, delay_scale=1., strength=1., enabled=True):
        speed = abs(_number(speed_rad_s, 'speed_rad_s'))
        strength = _number(strength, 'brake strength', minimum=0)
        if not isinstance(enabled, bool):
            raise ValueError('Brake enabled must be boolean')
        torque = self.config['screening_dynamic_torque_Nm' if speed > self.speed_threshold else 'holding_torque_Nm']
        return self.fraction(time_s, fault_time_s, delay_scale)*torque*strength if enabled else 0.

    @staticmethod
    def constraint_work(constraint_force_Nm, capacity_Nm, base_friction_Nm,
                        velocity_before_rad_s, velocity_after_rad_s, dt):
        """Attribute native friction work as in the source; not a new brake force.

        The physics solver supplies the actual joint-friction constraint force.
        Proportional allocation between dry friction and brake capacity is the
        source's stated attribution approximation, not a separate simulation.
        """
        capacity = _number(capacity_Nm, 'brake capacity', minimum=0)
        base = _number(base_friction_Nm, 'base friction', minimum=0)
        force = _number(constraint_force_Nm, 'constraint force')
        before = _number(velocity_before_rad_s, 'prior velocity')
        after = _number(velocity_after_rad_s, 'current velocity')
        dt = _number(dt, 'dt', strictly_positive=True)
        torque = force*capacity/(capacity+base) if capacity else 0.
        return dict(torque_Nm=torque, energy_increment_J=max(0., -torque*(before+after)*.5*dt))


class SafetyState:
    """Source fault latch and controlled stop for any number of coordinates.

    The historical class name describes simulation behavior only: this is not a
    safety-rated controller. Velocity-feedback finiteness is additionally
    checked because an imported runtime must not propagate nonfinite feedback.
    """
    def __init__(self, soft_limits, *, stop_duration_s=.35, watchdog_s=.1, limit_margin=.002):
        self.soft_limits = [_vector(pair, 'soft limit', 2) for pair in soft_limits]
        if not self.soft_limits or any(a > b for a, b in self.soft_limits):
            raise ValueError('Finite ordered soft limits are required')
        self.stop_duration_s = _number(stop_duration_s, 'stop duration', strictly_positive=True)
        self.watchdog_s = _number(watchdog_s, 'watchdog duration', strictly_positive=True)
        self.limit_margin = _number(limit_margin, 'limit margin', minimum=0)
        self.mode, self.reason = 'RUN', ''
        self.fault_time_s = self.stop_time_s = None
        self.last_command_time_s = 0.
        self.stop_snapshot = None
        self.events = []

    def trip(self, reason, time_s):
        time_s = _number(time_s, 'time_s')
        if self.mode == 'EMERGENCY':
            return
        self.mode, self.reason, self.fault_time_s = 'EMERGENCY', str(reason), time_s
        self.events.append(dict(t=time_s, mode=self.mode, reason=self.reason))

    def command(self, time_s, valid=True):
        if valid:
            self.last_command_time_s = _number(time_s, 'time_s')

    def normal_stop(self, time_s, q, qd, **held_commands):
        time_s = _number(time_s, 'time_s')
        q = _vector(q, 'q', len(self.soft_limits))
        qd = _vector(qd, 'qd', len(self.soft_limits))
        if self.mode != 'RUN':
            return
        duration = self.stop_duration_s
        end = [x+.5*duration*v for x, v in zip(q, qd)]
        if any(x < limits[0] or x > limits[1] for x, limits in zip(end, self.soft_limits)):
            self.trip('controlled-stop-envelope', time_s)
            return
        self.mode, self.stop_time_s = 'CONTROLLED_STOP', time_s
        self.stop_snapshot = dict(q=q, qd=qd, held=deepcopy(held_commands))
        self.events.append(dict(t=time_s, mode=self.mode, reason='operator normal stop'))

    def update(self, time_s, q, qd, *, estop=False, power=True, feedback_valid=True, limits=True):
        time_s = _number(time_s, 'time_s')
        q = _vector(q, 'q', len(self.soft_limits), finite=False)
        qd = _vector(qd, 'qd', len(self.soft_limits), finite=False)
        finite = all(isinstance(x, (float, int)) and not isinstance(x, bool) and math.isfinite(x) for x in [*q, *qd])
        if estop:
            self.trip('emergency-stop', time_s)
        elif not power:
            self.trip('power-loss', time_s)
        elif not feedback_valid or not finite:
            self.trip('feedback-invalid', time_s)
        elif time_s-self.last_command_time_s > self.watchdog_s+1e-9:
            self.trip('command-watchdog', time_s)
        elif limits and any(x < bounds[0]-self.limit_margin or x > bounds[1]+self.limit_margin for x, bounds in zip(q, self.soft_limits)):
            self.trip('software-travel-limit', time_s)
        if self.mode == 'CONTROLLED_STOP' and time_s-self.stop_time_s >= self.stop_duration_s:
            self.mode = 'HOLD'
            self.events.append(dict(t=time_s, mode=self.mode, reason='controlled deceleration complete'))

    def reference(self, time_s, normal):
        time_s = _number(time_s, 'time_s')
        if self.stop_snapshot is None or self.mode not in ('CONTROLLED_STOP', 'HOLD'):
            return deepcopy(normal)
        s, duration = self.stop_snapshot, self.stop_duration_s
        u = _clip((time_s-self.stop_time_s)/duration, 0., 1.)
        f = u-2.5*u**4+3*u**5-u**6
        v = 1-10*u**3+15*u**4-6*u**5
        a = (-30*u*u+60*u**3-30*u**4)/duration
        return dict(q=[x+speed*duration*f for x, speed in zip(s['q'], s['qd'])],
                    qd=[speed*v for speed in s['qd']], qdd=[speed*a for speed in s['qd']],
                    **deepcopy(s['held']), stage=8)
