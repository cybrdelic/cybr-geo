"""ARX belt spring-anchor placements from the same source as drive forces.

The reusable extremum and native pin/clevis verifier live in mechanism_lab.
This adapter only maps the pinned P5 joint frames and named native parts.
"""
from __future__ import annotations

import math

from mechanism_lab.actuation import TensionSpring
from mechanism_lab.physical_model import digest
from mechanism_lab.spring_anchor import tension_anchor_demand


EYE_PIN_DIAMETER_MM = 4.
EYE_PIN_LENGTH_MM = 16.
# Nominal CAD relief beyond each pin end prevents an end-face contact that
# existed in the inherited 16 mm deep cuts. This is not a process capability.
EYE_PIN_END_RELIEF_MM = 1.
CLEVIS_BORE_DIAMETER_MM = 4.2


def arx_spring_anchor_plan(spec, transmissions):
    by_joint = {row['joint']: row for row in transmissions}
    if len(by_joint) != len(transmissions) or not {'J2', 'J3'} <= set(by_joint):
        raise ValueError('ARX spring anchors require distinct J2 and J3 native transmissions')
    groups = []
    definitions = (
        ('shoulder', 'counterbalance', 'J2', 'turret', 'upper',
         'shoulder_spring_fixed_clevis', 'shoulder_spring_moving_clevis'),
        ('elbow', 'elbow_counterbalance', 'J3', 'upper', 'fore',
         'elbow_spring_fixed_clevis', 'elbow_spring_moving_clevis'),
    )
    for label, key, joint, parent_body, child_body, fixed_part, moving_part in definitions:
        transmission = by_joint[joint]
        if (transmission['parent'], transmission['child']) != (parent_body, child_body):
            raise ValueError(f'{joint} spring anchor body path disagrees with the native transmission')
        # The drive sits at +/-45 mm in Z; the baseline hinge and source
        # spring anchors use the zero-Z pivot plane at the same X/Y position.
        # child_origin_mm instead describes the shifted drive shaft in the
        # downstream body, not this parent-body spring pivot.
        pivot = [*transmission['origin_mm'][:2], 0.]
        if (not isinstance(pivot, (list, tuple)) or len(pivot) != 3
                or any(isinstance(v, bool) or not isinstance(v, (int, float))
                       or not math.isfinite(v) for v in pivot)):
            raise ValueError(f'{joint} requires a finite child-frame joint pivot')
        config = spec[key]
        if config.get('joint_index') != int(joint[1:])-1 or config.get('spring_count') != 2:
            raise ValueError(f'{joint} source requires two separately pinned springs')
        parent, child = config['parent_anchor_m'], config['child_anchor_m']
        if (len(parent) != 3 or len(child) != 3
                or parent[2] != child[2] or not math.isfinite(config['second_spring_z_m'])):
            raise ValueError(f'{joint} source anchors require a shared first eye plane and a second eye plane')
        nominal = TensionSpring(config).state(config['reference_q_rad'])
        for name, observed, expected in (
                ('length', nominal['length_m'], config['reference_length_m']),
                ('force', nominal['force_each_N'], config['reference_force_each_N']),
                ('torque', abs(nominal['torque_Nm']), config['reference_total_torque_Nm'])):
            if not math.isclose(observed, expected, rel_tol=1e-9, abs_tol=1e-9):
                raise ValueError(f'{joint} spring {name} reference disagrees with its force model')
        demand = tension_anchor_demand(config, spec['joint_hard_limits_rad'][config['joint_index']],
                                        pin_diameter_mm=EYE_PIN_DIAMETER_MM)
        anchors = []
        for index, z_m in enumerate((parent[2], config['second_spring_z_m'])):
            for role, body, coordinates, clevis in (
                    ('fixed', parent_body, parent, fixed_part),
                    ('moving', child_body, child, moving_part)):
                local_mm = [1000*coordinates[0], 1000*coordinates[1], 1000*z_m]
                if role == 'fixed':
                    local_mm = [origin + value for origin, value in zip(pivot, local_mm)]
                anchors.append(dict(name=f'{label}_{role}_{index}', role=role,
                    body=body, body_local_anchor_mm=local_mm,
                    pin_part='spring_eye_pin', clevis_part=clevis,
                    required_capacity_N=demand['required_capacity_per_separate_pin_N']))
        groups.append(dict(name=label, source_config=key, joint=joint,
            parent_body=parent_body, child_body=child_body,
            parent_joint_origin_mm=list(pivot), nominal_reference=nominal,
            demand=demand, anchors=anchors))
    plan = dict(schema='cybrgeo-arx-spring-anchor-plan/1',
        nominal_pin_diameter_mm=EYE_PIN_DIAMETER_MM,
        nominal_pin_length_mm=EYE_PIN_LENGTH_MM,
        nominal_bore_end_relief_mm=EYE_PIN_END_RELIEF_MM,
        nominal_clevis_bore_diameter_mm=CLEVIS_BORE_DIAMETER_MM,
        groups=groups,
        scope='Native pin positions and ideal full-hard-stop spring force demand share source joint frames; no measured spring/anchor capacity')
    plan['plan_sha256'] = digest(plan)
    return plan


def verify_arx_anchor_receipt(spec, transmissions, receipt, placements, parts):
    """Read back named native pins and demand without elevating it to capacity.

    The normal project release path separately verifies file hashes and the
    native source binding. This check makes simulation compilation reject a
    stale or altered spring plan and pin/clevis identities as well.
    """
    expected_plan = arx_spring_anchor_plan(spec, transmissions)
    expected_rows = [row for group in expected_plan['groups'] for row in group['anchors']]
    if (not isinstance(receipt, dict)
            or receipt.get('schema') != 'cybrgeo-native-spring-anchor-load-path/1'
            or receipt.get('plan') != expected_plan
            or receipt.get('tested_pin_clevis_capacity_available') is not False
            or not isinstance(placements, dict)
            or placements.get('units') != 'mm'
            or not isinstance(placements.get('records'), list)
            or not isinstance(parts, dict)):
        raise ValueError('Native spring anchor source or construction receipt is invalid')
    actual_rows = receipt.get('native_anchor_bindings')
    if not isinstance(actual_rows, list) or len(actual_rows) != len(expected_rows) == 8:
        raise ValueError('Native spring anchor receipt requires all eight distinct pins')
    placed = {}
    for record in placements['records']:
        if not isinstance(record, dict):
            raise ValueError('Invalid native spring anchor placement record')
        key = (record.get('body'), record.get('name'))
        if not all(isinstance(value, str) for value in key):
            raise ValueError('Invalid native spring anchor placement identity')
        if key in placed:
            raise ValueError('Duplicate native spring anchor placement identity')
        placed[key] = record
    seen = set()
    nominal_gap = (CLEVIS_BORE_DIAMETER_MM-EYE_PIN_DIAMETER_MM)/2
    for row, expected in zip(actual_rows, expected_rows):
        if not isinstance(row, dict):
            raise ValueError('Invalid native spring anchor binding')
        name, body = row.get('pin_native_part'), expected['body']
        key = (body, name)
        valid_name = (isinstance(name, str) and name.startswith('spring_eye_pin_')
                      and name[len('spring_eye_pin_'):].isdigit())
        native = placed.get(key)
        clevis_part = expected['clevis_part']
        clevis_name = row.get('clevis_placed_part')
        clevis_key = (body, clevis_name)
        valid_clevis_name = (isinstance(clevis_name, str)
                             and clevis_name.startswith(clevis_part+'_')
                             and clevis_name[len(clevis_part)+1:].isdigit())
        placed_clevis = placed.get(clevis_key)
        measured_gap = row.get('measured_nominal_radial_gap_mm')
        overlap = row.get('intersection_volume_mm3')
        if (not valid_name or key in seen or native is None
                or native.get('material') != 'steel'
                or native.get('step_file') != f'cad/placed/{body}__{name}.step'
                or not valid_clevis_name or placed_clevis is None
                or placed_clevis.get('material') != 'aluminum'
                or placed_clevis.get('step_file') != f'cad/placed/{body}__{clevis_name}.step'
                or row.get('anchor') != expected['name']
                or row.get('body') != body
                or row.get('clevis_native_part') != clevis_part
                or row.get('required_capacity_N') != expected['required_capacity_N']
                or row.get('body_local_anchor_mm') != expected['body_local_anchor_mm']
                or row.get('pin_diameter_mm') != EYE_PIN_DIAMETER_MM
                or row.get('pin_length_mm') != EYE_PIN_LENGTH_MM
                or row.get('clevis_bore_diameter_mm') != CLEVIS_BORE_DIAMETER_MM
                or not isinstance(measured_gap, (float, int))
                or not math.isfinite(measured_gap)
                or not math.isclose(measured_gap, nominal_gap, rel_tol=0, abs_tol=1e-5)
                or not isinstance(overlap, (float, int))
                or not math.isfinite(overlap) or overlap < 0 or overlap > 1e-5
                or parts.get('spring_eye_pin', {}).get('material') != 'steel'
                or parts.get(clevis_part, {}).get('material') != 'aluminum'):
            raise ValueError('Native spring anchor pin/clevis geometry or load path disagrees with source')
        seen.add(key)
    return expected_plan


def verify_arx_spring_joint_frames(model_root, plan):
    """Bind spring pivot and rotation direction to the simulated child hinges."""
    parents = {child: parent for parent in model_root.iter() for child in parent}
    for group in plan['groups']:
        body = model_root.find(f".//body[@name='{group['child_body']}']")
        joint = None if body is None else body.find(f"./joint[@name='j{group['joint'][1:]}']")
        try:
            origin = [float(value) for value in body.get('pos').split()]
            axis = [float(value) for value in joint.get('axis').split()]
        except (AttributeError, ValueError) as error:
            raise ValueError('ARX spring model requires native child hinges with explicit frames') from error
        expected = [value/1000 for value in group['parent_joint_origin_mm']]
        if (parents[body].get('name') != group['parent_body']
                or len(origin) != 3 or len(axis) != 3
                or not all(math.isfinite(value) for value in origin+axis)
                or any(not math.isclose(value, target, rel_tol=0, abs_tol=1e-9)
                       for value, target in zip(origin, expected))
                or any(not math.isclose(value, target, rel_tol=0, abs_tol=1e-12)
                       for value, target in zip(axis, (0, 0, -1)))):
            raise ValueError('ARX spring model hinge frame disagrees with native spring anchors')
