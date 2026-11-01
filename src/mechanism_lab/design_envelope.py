"""Source-bound nominal fit, drive and load-path design screens.

These calculations use authored CAD and model inputs. A design allowance is
not a measured process capability or a hardware rating.
"""
from __future__ import annotations

import hashlib
import json
import math
import re


def _digest(document):
    payload = json.dumps(document, sort_keys=True, separators=(',', ':'),
                         allow_nan=False).encode('utf-8')
    return hashlib.sha256(payload).hexdigest()


def _number(value, label, *, positive=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value < 0 or (positive and value == 0)):
        raise ValueError(f'{label} must be finite and {"positive" if positive else "nonnegative"}')
    return float(value)


def _source_sha(value):
    if value is not None and (not isinstance(value, str)
                              or re.fullmatch(r'[0-9a-f]{64}', value) is None):
        raise ValueError('Source SHA-256 must be a lowercase 64-character hex digest')
    return value


def radial_clearance_mm(diametral_clearance_mm):
    """Convert an authored diameter difference to a nominal radial gap."""
    return _number(diametral_clearance_mm, 'Diametral clearance') / 2.


def _vector3(value, label):
    if (not isinstance(value, (list, tuple)) or len(value) != 3
            or any(isinstance(item, bool) or not isinstance(item, (int, float))
                   or not math.isfinite(item) for item in value)):
        raise ValueError(f'{label} must be a finite three-component vector')
    return [float(item) for item in value]


def planar_gravity_torque_bound(body_masses_kg,
                                com_point_speed_bounds_mm_per_input_rad,
                                gravity_m_s2, motion_plane_normal, *, source_sha256=None):
    """Bound quasi-static input torque from planar CAD-body gravity potential.

    Each speed must conservatively bound that body's COM motion over the
    intended input and configuration range, in mm per input radian. Projecting
    gravity onto the motion plane then gives |dU/dq| no greater than the sum
    of mass * in-plane gravity * COM speed. This is not a dynamics or strength
    analysis; inertia, friction and support/contact loads are excluded.
    """
    _source_sha(source_sha256)
    masses, speeds = body_masses_kg, com_point_speed_bounds_mm_per_input_rad
    if (not isinstance(masses, dict) or not isinstance(speeds, dict)
            or not masses or set(masses) != set(speeds)
            or any(not isinstance(name, str) or not name for name in masses)):
        raise ValueError('Exact nonempty named CAD mass and COM speed inventories are required')
    gravity = _vector3(gravity_m_s2, 'Gravity in m/s^2')
    normal = _vector3(motion_plane_normal, 'Motion plane normal')
    normal_length = math.hypot(*normal)
    if not math.isfinite(normal_length) or normal_length == 0:
        raise ValueError('Motion plane normal must have finite nonzero length')
    unit_normal = [component / normal_length for component in normal]
    gravity_normal = sum(a*b for a, b in zip(gravity, unit_normal))
    gravity_in_plane = [a-gravity_normal*b for a, b in zip(gravity, unit_normal)]
    in_plane_m_s2 = math.hypot(*gravity_in_plane)
    if not math.isfinite(in_plane_m_s2):
        raise ValueError('Projected gravity must be finite')
    contributions = {}
    validated_masses, validated_speeds = {}, {}
    for name in sorted(masses):
        mass = _number(masses[name], f'{name} CAD mass', positive=True)
        speed = _number(speeds[name], f'{name} COM speed bound')
        contribution = mass * in_plane_m_s2 * (speed / 1000.)
        if not math.isfinite(contribution):
            raise ValueError(f'{name} gravity torque contribution must be finite')
        validated_masses[name], validated_speeds[name] = mass, speed
        contributions[name] = contribution
    total = math.fsum(contributions.values())
    if not math.isfinite(total):
        raise ValueError('Total gravity torque bound must be finite')
    inputs = dict(body_masses_kg=validated_masses,
                  com_point_speed_bounds_mm_per_input_rad=validated_speeds,
                  gravity_m_s2=gravity, motion_plane_normal=unit_normal)
    return dict(schema='cybrgeo-planar-gravity-torque-bound/1',
                source_sha256=source_sha256, source_inputs_sha256=_digest(inputs),
                body_names=sorted(masses), body_contributions_Nm=contributions,
                gravity_in_motion_plane_m_s2=in_plane_m_s2,
                total_gravity_torque_bound_Nm=total,
                status='source_geometry_bound', hardware_load_qualified=False,
                scope='Quasi-static planar CAD-mass gravity bound only; excludes inertia, friction, impact, compliance and dynamic support/contact forces')


def fit_budget_report(native_interface_report, *, variation_allowances_mm=None,
                      source_sha256=None):
    """Screen native B-rep gaps against authored per-interface variation.

    Native measured checks can carry their source-authored first_mm/second_mm
    design targets. The optional caller mapping supports older diagnostic
    clients but cannot override a target embedded in a native receipt. Missing
    entries remain unresolved. Numerical CAD allowances never become
    manufacturing variation or a positive service-fit result.
    """
    _source_sha(source_sha256)
    report = native_interface_report
    if (not isinstance(report, dict)
            or report.get('schema') != 'cybrgeo-native-interfaces/1'
            or not isinstance(report.get('checks'), list)):
        raise ValueError('Expected a native interface measurement report')
    allowances = {} if variation_allowances_mm is None else variation_allowances_mm
    if not isinstance(allowances, dict):
        raise ValueError('Variation allowances must be a mapping')
    rows, nonclearance, names = [], [], set()
    for check in report['checks']:
        if not isinstance(check, dict) or not isinstance(check.get('name'), str) or not check['name']:
            raise ValueError('Every native interface check needs a name')
        name = check['name']
        if name in names:
            raise ValueError(f'Duplicate native interface check: {name}')
        names.add(name)
        if 'required_minimum_distance_mm' not in check:
            nonclearance.append(name)
            continue
        measured = _number(check.get('minimum_distance_mm'), f'{name} measured gap')
        required = _number(check['required_minimum_distance_mm'], f'{name} required gap')
        kernel = _number(check.get('distance_kernel_allowance_mm', 0.),
                         f'{name} numerical distance allowance')
        overlap = _number(check.get('intersection_volume_mm3'),
                          f'{name} B-rep intersection volume')
        allowed_overlap = _number(check.get('volume_tolerance_mm3'),
                                  f'{name} numerical volume allowance')
        nominal_passed = (check.get('pass') is True and overlap <= allowed_overlap
                          and measured + kernel >= required)
        def checked_variation(value):
            if not isinstance(value, dict) or set(value) != {'first_mm', 'second_mm'}:
                raise ValueError(f'{name} needs exactly first_mm and second_mm variations')
            return {key: _number(value[key], f'{name} {key} design variation')
                    for key in ('first_mm', 'second_mm')}

        embedded = (checked_variation(check['declared_design_variations_mm'])
                    if 'declared_design_variations_mm' in check else None)
        supplied = checked_variation(allowances[name]) if name in allowances else None
        if embedded is not None and supplied is not None and embedded != supplied:
            raise ValueError(f'{name} explicit variation conflicts with its source-bound native design target')
        variation = embedded if embedded is not None else supplied
        variation_source = ('native_authored_design' if embedded is not None else
                            'explicit_caller' if supplied is not None else None)
        first = second = worst = stack_passed = None
        if variation is not None:
            first = variation['first_mm']
            second = variation['second_mm']
            worst = measured - first - second
            # A declared dimensional stack must fit the authored service gap
            # without borrowing numerical B-rep error as physical clearance.
            stack_passed = bool(nominal_passed and worst >= required)
        rows.append(dict(name=name, measured_nominal_gap_mm=measured,
                         required_service_gap_mm=required,
                         available_total_variation_mm=measured-required,
                         numerical_distance_allowance_mm=kernel,
                         nominal_cad_passed=nominal_passed,
                         first_declared_variation_mm=first,
                         second_declared_variation_mm=second,
                         variation_source=variation_source,
                         worst_case_declared_gap_mm=worst,
                         declared_stack_screen_passed=stack_passed,
                         status=('unresolved' if variation is None
                                 else 'declared_stack_passed' if stack_passed
                                 else 'declared_stack_failed'),
                         unresolved_reason=('part_variation_unavailable'
                                            if variation is None else None)))
    if set(allowances) - {row['name'] for row in rows}:
        raise ValueError('Variation allowances name an unknown or non-clearance interface')
    native_passed = (report.get('passed') is True
                     and bool(report['checks'])
                     and all(check.get('pass') is True for check in report['checks']))
    complete = bool(rows) and all(row['declared_stack_screen_passed'] is not None
                                  for row in rows)
    screen_passed = (native_passed and complete
                     and all(row['declared_stack_screen_passed'] is True for row in rows))
    return dict(schema='cybrgeo-design-fit-budget/1',
                source_sha256=source_sha256,
                native_interface_report_sha256=_digest(report),
                fits=rows, nonclearance_checks=nonclearance,
                native_interface_passed=native_passed,
                complete_declared_stack=complete,
                declared_stack_screen_passed=screen_passed,
                status=('unresolved' if not complete else
                        'declared_stack_passed' if screen_passed else
                        'declared_stack_failed'),
                manufacturing_fit_qualified=False,
                scope='Source CAD nominal gaps and authored design variation targets only; numerical CAD allowance is not part variation or measured process capability')


def motor_torque_screen(name, source_motor, observed_peak_output_torque_Nm=None,
                        *, demand_limit_durations_s=None, source_sha256=None):
    """Screen delivered torque *and* requested-torque clipping in the model."""
    _source_sha(source_sha256)
    if not isinstance(name, str) or not name or not isinstance(source_motor, dict):
        raise ValueError('Motor name and source definition are required')
    limits = {}
    for key in ('gear_output_torque_limit_Nm', 'belt_allowable_output_torque_Nm'):
        if key in source_motor:
            limits[key] = _number(source_motor[key], key, positive=True)
    if not limits:
        raise ValueError('Source motor needs a finite positive output torque limit')
    governing = min(limits.values())
    observed = (None if observed_peak_output_torque_Nm is None
                else _number(observed_peak_output_torque_Nm, 'Observed simulated peak torque'))
    durations = {} if demand_limit_durations_s is None else demand_limit_durations_s
    if not isinstance(durations, dict) or set(durations) - {'belt', 'gear'}:
        raise ValueError('Demand-limit durations must name only belt and gear')
    durations = {key: _number(value, f'{key} source-model demand limit duration')
                 for key, value in durations.items()}
    complete_demand_telemetry = set(durations) == {'belt', 'gear'}
    demand_saturated = any(value > 0 for value in durations.values())
    passed = (None if observed is None or not complete_demand_telemetry else
              observed <= governing and not demand_saturated)
    return dict(schema='cybrgeo-model-torque-screen/1', name=name,
                source_sha256=source_sha256, source_motor_sha256=_digest(source_motor),
                source_model_limits_Nm=limits, governing_model_limit_Nm=governing,
                observed_simulated_peak_Nm=observed,
                demand_limit_durations_s=durations,
                complete_demand_telemetry=complete_demand_telemetry,
                demanded_torque_was_limited=demand_saturated,
                model_limit_margin_Nm=None if observed is None else governing-observed,
                model_limit_utilization=None if observed is None else observed/governing,
                model_screen_passed=passed,
                status=('unresolved' if observed is None or not complete_demand_telemetry else
                        'source_model_demand_limited' if demand_saturated else
                        'source_model_passed' if passed else
                        'source_model_failed'),
                hardware_capacity_qualified=False,
                scope='Source-declared drive model versus delivered output torque and requested-torque clipping; not a supplier or tested hardware rating')


def connector_force_screens(constraint_metrics, integration_steps):
    """Retain every-step modeled connect/weld anchor demand, if available.

    An equality solver multiplier is a modeled ideal connector reaction. It is
    not a bearing pressure, pin shear, stress concentration or hardware rating.
    A missing/incomplete solver row never becomes a zero-force measurement.
    A diagnostic run with no integration steps may still be recorded, but
    cannot report complete reaction coverage even if a malformed record
    advertises zero active steps and zero samples.
    """
    if (not isinstance(constraint_metrics, dict)
            or isinstance(integration_steps, bool) or not isinstance(integration_steps, int)
            or integration_steps < 0):
        raise ValueError('Connector screens require named constraints and a nonnegative integration-step count')
    rows = {}
    for name, record in constraint_metrics.items():
        if not isinstance(name, str) or not name or not isinstance(record, dict):
            raise ValueError('Constraint force record must be named')
        if record.get('type') not in ('connect', 'weld'):
            continue
        reaction = record.get('reaction_vector_norms', {}).get('anchor_force', {})
        peak = reaction.get('peak_absolute', {}) if isinstance(reaction, dict) else {}
        observed = peak.get('value') if isinstance(peak, dict) else None
        if observed is not None:
            observed = _number(observed, f'{name} modeled connector force')
        complete = (integration_steps > 0
                    and record.get('active_steps') == integration_steps
                    and record.get('missing_row_steps') == 0
                    and record.get('incomplete_reaction_row_steps') == 0
                    and reaction.get('samples') == integration_steps
                    and observed is not None)
        rows[name] = dict(name=name, kind=record['type'],
            peak_modeled_anchor_force_N=observed if complete else None,
            observed_peak_if_incomplete_N=None if complete else observed,
            integration_steps=integration_steps,
            measured_reaction_steps=reaction.get('samples'),
            missing_row_steps=record.get('missing_row_steps'),
            incomplete_reaction_row_steps=record.get('incomplete_reaction_row_steps'),
            complete_solver_coverage=complete,
            hardware_capacity_qualified=False,
            status='modeled_demand_recorded' if complete else 'unresolved')
    return dict(schema='cybrgeo-connector-force-screens/1',
                connectors=rows,
                complete_solver_coverage=bool(rows) and all(
                    row['complete_solver_coverage'] for row in rows.values()),
                hardware_capacity_qualified=False,
                scope='Source-model MuJoCo equality anchor multipliers over integration steps; no physical bearing/pin capacity or measured load')


def validate_load_paths(parts, interfaces, loads, paths):
    """Validate named actuator-to-reaction graph connectivity.

    Interfaces map names to two part IDs; loads map names to loaded part IDs.
    This is topology coverage, never actual force transfer or strength.
    """
    if (not isinstance(parts, (list, tuple, set)) or not parts
            or any(not isinstance(part, str) or not part for part in parts)
            or len(set(parts)) != len(parts)
            or not isinstance(interfaces, dict) or not interfaces
            or not isinstance(loads, dict) or not loads or not isinstance(paths, dict)
            or set(paths) != set(loads)):
        raise ValueError('Load paths require exact nonempty part, interface and load inventories')
    part_set = set(parts)
    for name, pair in interfaces.items():
        if (not isinstance(name, str) or not name
                or not isinstance(pair, (list, tuple)) or len(pair) != 2
                or pair[0] == pair[1] or not set(pair) <= part_set):
            raise ValueError(f'Invalid load-path interface {name}')
    covered = set()
    for name, loaded in loads.items():
        path = paths[name]
        if (not isinstance(name, str) or not name or loaded not in part_set
                or not isinstance(path, dict)
                or set(path) != {'parts', 'interfaces'}):
            raise ValueError(f'Invalid load path {name}')
        members, edges = path['parts'], path['interfaces']
        if (not isinstance(members, list) or not isinstance(edges, list)
                or len(members) < 2 or len(edges) != len(members)-1
                or members[0] != loaded or len(set(members)) != len(members)
                or len(set(edges)) != len(edges) or not set(members) <= part_set
                or not set(edges) <= set(interfaces)):
            raise ValueError(f'Load path {name} needs a distinct connected reaction part')
        for first, second, edge in zip(members, members[1:], edges):
            if set(interfaces[edge]) != {first, second}:
                raise ValueError(f'Load path {name} disconnects at {edge}')
        covered.update(edges)
    if covered != set(interfaces):
        raise ValueError('Load paths omit a declared interface')
    return dict(schema='cybrgeo-load-path-topology/1', load_names=sorted(loads),
                covered_interface_names=sorted(covered), topology_passed=True,
                hardware_load_path_qualified=False,
                scope='Named graph connectivity only; no force transfer, stress or physical capacity proof')
