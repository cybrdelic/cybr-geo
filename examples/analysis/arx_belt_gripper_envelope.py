"""Source-bound native CAD clearance for ARX's opposed gripper slide range.

This client deliberately covers six named noncontact STEP components, not the
entire arm, pinion/rack tooth contact, guide bearings, or the puck. The same
general native B-rep interval checker serves other GEO mechanisms.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from mechanism_lab.engineering_source import (load_project, project_directory,
    verify_native_build, verify_native_source_binding)
from mechanism_lab import native_pose_envelope as pose_service
from mechanism_lab.native_pose_envelope import certify_native_parametric_path
from mechanism_lab.physical_model import digest


SELECTED_LOGICAL_PARTS = (
    ('jaw', 'finger_pad'),
    ('jaw', 'right_jaw_carriage'),
    ('jaw_opposed', 'finger_pad'),
    ('jaw_opposed', 'left_jaw_carriage'),
    ('tool', 'gripper_motor'),
    ('tool', 'gripper_cable_clamp_mount'),
)


def selected_native_parts(native_directory, expected_file_hashes):
    """Resolve six source part roles from a verified native placement manifest.

    Assembly enumeration may change a numeric instance suffix when an upstream
    component moves to another body. A missing or ambiguous role fails closed.
    """
    native = Path(native_directory).resolve()
    manifest = native / 'placed-native-parts.json'
    if (not manifest.is_file() or manifest.is_symlink()
            or not isinstance(expected_file_hashes, dict)
            or expected_file_hashes.get(manifest.name) !=
               hashlib.sha256(manifest.read_bytes()).hexdigest()):
        raise ValueError('Native placed-part manifest differs from verified receipt')
    inventory = json.loads(manifest.read_text(encoding='utf-8'))
    if inventory.get('units') != 'mm' or not isinstance(inventory.get('records'), list):
        raise ValueError('Native placed-part manifest has invalid inventory')
    selected = []
    for body, base in SELECTED_LOGICAL_PARTS:
        matches = []
        for row in inventory['records']:
            if not isinstance(row, dict):
                raise ValueError('Native placed-part manifest has invalid record')
            name = row.get('name')
            if (row.get('body') == body and isinstance(name, str)
                    and (name == base or
                         (name.startswith(base + '_') and name[len(base)+1:].isdecimal()))):
                matches.append(f'{body}__{name}')
        if len(matches) != 1:
            raise ValueError(f'Native gripper role {body}__{base} is missing or ambiguous')
        selected.extend(matches)
    return tuple(selected)


def _triple(raw, label):
    try:
        values = tuple(float(value) for value in raw.split())
    except (AttributeError, ValueError) as error:
        raise ValueError(f'{label} must have three finite source components') from error
    if len(values) != 3 or any(not math.isfinite(value) for value in values):
        raise ValueError(f'{label} must have three finite source components')
    return values


def source_slide_contract(xml_path):
    """Read the actual nested jaw joint ranges and common rack constraint."""
    path = Path(xml_path)
    payload = path.read_bytes()
    root = ET.fromstring(payload)
    source = {}
    for body, axis in (('jaw_left', (1., 0., 0.)),
                       ('jaw_right', (-1., 0., 0.))):
        element = root.find(f".//body[@name='tool']/body[@name='{body}']")
        if element is None:
            raise ValueError(f'ARX source omits tool-child {body}')
        joint = element.find(f"joint[@name='{body}']")
        if joint is None or joint.get('type') != 'slide':
            raise ValueError(f'ARX source omits slide joint {body}')
        position = _triple(element.get('pos'), f'{body} body position')
        direction = _triple(joint.get('axis'), f'{body} slide axis')
        try:
            limits = tuple(float(value) for value in joint.get('range', '').split())
        except ValueError as error:
            raise ValueError(f'{body} slide range is invalid') from error
        if (len(limits) != 2 or any(not math.isfinite(value) for value in limits)
                or not 0 < limits[0] < limits[1] or direction != axis
                or position != (0., .2192, 0.) or limits != (.010, .040)):
            raise ValueError('ARX gripper source changed; revise the CAD path and mating contract together')
        source[body] = dict(base_position_m=position, axis=direction,
                            range_m=limits)
    equality = root.find(".//equality/joint[@name='opposed_rack_synchronization']")
    if (equality is None or equality.get('joint1') != 'jaw_left'
            or equality.get('joint2') != 'jaw_right'
            or equality.get('polycoef') != '0 1 0 0 0'):
        raise ValueError('ARX source no longer declares the synchronized opposed rack')
    return dict(schema='cybrgeo-arx-gripper-slide-source/1',
        xml_sha256=hashlib.sha256(payload).hexdigest(),
        jaws=source, equality='opposed_rack_synchronization',
        scope='Nominal synchronized 10–40 mm slide coordinates in source tool frame; no tooth, bushing or hardware load evidence')


def gripper_path(contract, selected_names):
    """Pose and rigorous point-speed bound for every point of six rigid parts."""
    left, right = contract['jaws']['jaw_left'], contract['jaws']['jaw_right']
    if (left['range_m'] != right['range_m']
            or left['base_position_m'] != right['base_position_m']):
        raise ValueError('Opposed slide source lost its common base and travel')
    start, end = left['range_m']
    base_mm = np.asarray(left['base_position_m'], dtype=float)*1000.

    def pose_at(u):
        if isinstance(u, bool) or not isinstance(u, (float, int)) or not 0 <= u <= 1:
            raise ValueError('Gripper parameter must lie in the source travel interval')
        q_mm = (start + float(u)*(end-start))*1000.
        poses = {'tool': np.eye(4)}
        for body, sign in (('jaw', 1.), ('jaw_opposed', -1.)):
            matrix = np.eye(4)
            matrix[:3, 3] = base_mm + [sign*q_mm, 0., 0.]
            poses[body] = matrix
        return poses

    def point_speed_bound(part_name, _lo, _hi):
        if part_name not in selected_names:
            raise ValueError('Point-speed bound has no such selected native part')
        return 0. if part_name.startswith('tool__') else (end-start)*1000.

    return pose_at, point_speed_bound


def certify(native_directory):
    """Bind the selected STEP, source XML and full native receipt to one audit."""
    native = Path(native_directory).resolve()
    project, source_files = load_project('arx_belt')
    source = project_directory('arx_belt') / 'data' / 'P2-baseline.xml'
    contract = source_slide_contract(source)
    if source_files.get('data/P2-baseline.xml') != contract['xml_sha256']:
        raise ValueError('Parsed gripper joint source differs from the bound project source')
    receipt = verify_native_build(native, project.specification, source_files=source_files)
    binding = verify_native_source_binding(native, project.specification,
                                          source_files, receipt=receipt)
    selected_names = selected_native_parts(native, receipt['files_sha256'])
    pose_at, speed = gripper_path(contract, selected_names)
    client_sha256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    checker_sha256 = hashlib.sha256(Path(pose_service.__file__).read_bytes()).hexdigest()
    report = certify_native_parametric_path(native, pose_at, speed,
        source_identity=(f'ARX source gripper joint {contract["xml_sha256"]}; '
            f'native {digest(receipt)}; client {client_sha256}; checker {checker_sha256}'),
        expected_file_hashes=receipt['files_sha256'],
        selected_names=selected_names,
        required_clearance_mm=0., distance_margin_mm=1e-4,
        max_depth=14, max_pair_intervals=16383)
    report['arx_gripper_contract'] = contract
    report['native_build_sha256'] = digest(receipt)
    report['native_source_binding_sha256'] = digest(binding)
    report['project_source_files_sha256'] = digest(source_files)
    report['client_source_sha256'] = client_sha256
    report['checker_source_sha256'] = checker_sha256
    report['selected_part_count'] = len(selected_names)
    report['unexamined_native_part_count'] = (
        report['native_step_import']['native_inventory_count']-len(selected_names))
    report['noncontact_cross_body_pair_count'] = len(report['clearance_pairs'])
    if report['noncontact_cross_body_pair_count'] != 12:
        raise ValueError('Selected ARX gripper bodies no longer define twelve cross-body pairs')
    report['known_intentional_contacts_outside_scope'] = [
        'puck with each finger pad and each nest seat',
        'gripper pinion with upper and lower racks',
        'guide rods with sliding jaw bushings',
    ]
    report['scope'] = ('All nominal source-authorized synchronized jaw positions from 10 to 40 mm '
        'for six selected native STEP components and their twelve noncontact cross-body pairs. '
        'No full-arm, free-puck, MuJoCo between-step, deformation, tolerance or hardware qualification claim.')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native', required=True, type=Path,
                        help='Fresh verified ARX native build folder')
    parser.add_argument('--output', required=True, type=Path,
                        help='Diagnostic JSON receipt path')
    args = parser.parse_args()
    result = certify(args.native)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps(dict(passed=result['passed'],
                          selected_parts=result['selected_part_count'],
                          noncontact_pairs=result['noncontact_cross_body_pair_count'],
                          unexamined_native_parts=result['unexamined_native_part_count'],
                          output=str(args.output)), indent=2))
    if not result['passed']:
        raise SystemExit(1)
