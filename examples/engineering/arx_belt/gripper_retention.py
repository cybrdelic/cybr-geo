"""Source-bound construction input for an optional ARX gripper brake candidate.

The default P5-derived design has no gripper retention brake. The optional
candidate uses a vendor envelope and authored interface geometry; neither
that CAD nor a reduced-order brake simulation qualifies configured hardware.

The source brake envelope comes from cybrdelic/cybr-arx6-belt commit
aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2. This GPL-2.0-only
adaptation retains its configured-hub and supplier-qualification limits.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from mechanism_lab.fit_construction import CoaxialShaftHubSeat, DirectedFaceGap

from .rack_contract import JAW_ORIGIN_Y_MM, PINION_CENTER_Y_MM


DATA = Path(__file__).with_name('data')
SOURCE_COMMIT = 'aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2'
BRAKE_SIGNATURE_FIELDS = {
    'OD_mm', 'ID_mm', 'length_mm', 'mass_kg', 'holding_torque_Nm',
    'mount_pcd_mm', 'mount_count', 'coil_voltage_V', 'coil_power_W'}
CANDIDATE_REVISION = 'outboard_pm_brake_capture_diagnostic_v2'
CANDIDATE_PARTS = {
    'tool__gripper_brake_stator_envelope': 'steel',
    'tool__gripper_brake_mount_screw_0': 'steel',
    'tool__gripper_brake_mount_screw_1': 'steel',
    'tool__gripper_brake_mount_screw_2': 'steel',
    'tool__gripper_brake_lower_thrust_envelope': 'steel',
    'tool__gripper_brake_upper_thrust_envelope': 'steel',
    'pinion__gripper_brake_hub_surrogate': 'steel',
    'pinion__gripper_brake_armature_surrogate': 'steel',
    'pinion__gripper_brake_key_surrogate': 'steel',
    'pinion__gripper_brake_rotor_screw_0': 'steel',
    'pinion__gripper_brake_rotor_screw_1': 'steel',
    'pinion__gripper_brake_rotor_screw_2': 'steel',
    'pinion__gripper_brake_rotor_screw_3': 'steel',
}


def _load(name):
    path = DATA / name
    return json.loads(path.read_text(encoding='utf-8'))


def _digest(document):
    payload = json.dumps(document, sort_keys=True, separators=(',', ':'),
                         allow_nan=False).encode('utf-8')
    return hashlib.sha256(payload).hexdigest()


def _positive(value, label):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value <= 0):
        raise ValueError(f'{label} must be finite and positive')
    return float(value)


def _same(actual, expected, label):
    number = _positive(actual, label)
    if not math.isclose(number, _positive(expected, f'authored {label}'),
                        rel_tol=1e-12, abs_tol=1e-12):
        raise ValueError(f'Pinned ARX {label} changed; revise the brake and gripper construction together')
    return number


def _design_source(specification, baseline, document):
    if (not isinstance(specification, dict) or not isinstance(baseline, dict)
            or not isinstance(document, dict)
            or document.get('schema') != 'cybrgeo-arx-gripper-retention-proposal/1'
            or document.get('source_commit') != SOURCE_COMMIT
            or document.get('source_brake_key') != 'small'):
        raise ValueError('The gripper brake needs its pinned source and authored proposal')
    if (not isinstance(document.get('source_brake_signature'), dict)
            or set(document['source_brake_signature']) != BRAKE_SIGNATURE_FIELDS):
        raise ValueError('Exact pinned small-brake source signature is required')
    try:
        vendor = specification['brakes']['small']
        baseline_vendor = baseline['integration']['brakes']['small']
        gripper = baseline['gripper']
        puck = baseline['puck']
        source_shaft = specification['interface_completion']['pinion_shaft_nominal_diameter_mm']
        baseline_shaft = baseline['integration']['interface_completion']['pinion_shaft_nominal_diameter_mm']
    except (KeyError, TypeError) as error:
        raise ValueError('Pinned ARX gripper and small-brake source fields are required') from error
    for name, expected in document['source_brake_signature'].items():
        _same(vendor.get(name), expected, f'small brake {name}')
        _same(baseline_vendor.get(name), expected, f'baseline small brake {name}')
    source_shaft = _same(source_shaft, document['source_gripper_upper_shaft_diameter_mm'],
                         'gripper upper shaft diameter')
    _same(baseline_shaft, source_shaft, 'baseline gripper upper shaft diameter')
    puck_radius = _positive(puck.get('outer_radius_m'), 'puck outer radius') * 1000.
    puck_half_length = _positive(puck.get('height_m'), 'puck full length') * 500.
    tool_tcp = _positive(baseline.get('tool_length_m'), 'tool TCP distance') * 1000.
    return vendor, gripper, puck, source_shaft, puck_radius, puck_half_length, tool_tcp


def proposal(*, specification=None, baseline=None, document=None):
    """Derive a nominal outboard target and report the actual P5 mismatch.

    The input allocations are design targets only. A qualified native CAD
    receipt, configured hub drawing, installed brake tests and measured
    fabrication process are outside this function's authority.
    """
    spec = _load('spec.json') if specification is None else specification
    base = _load('P2-baseline-parameters.json') if baseline is None else baseline
    authored = _load('gripper-retention-proposal.json') if document is None else document
    vendor, gripper, puck, source_shaft, puck_radius, puck_half_length, tool_tcp = _design_source(spec, base, authored)
    gap_document = authored['puck_to_stator_axial_design']
    if (not isinstance(gap_document, dict) or
            set(gap_document) != {'required_service_gap_mm', 'first_declared_variation_mm',
                                  'second_declared_variation_mm', 'design_reserve_mm'}):
        raise ValueError('Outboard puck/stator gap needs a complete authored design budget')
    gap = DirectedFaceGap.from_document(gap_document['required_service_gap_mm'],
        {key: gap_document[key] for key in ('first_declared_variation_mm',
                                            'second_declared_variation_mm', 'design_reserve_mm')})
    seat = authored['candidate_nominal_seat']
    if not isinstance(seat, dict) or set(seat) != {
            'minimum_nominal_radial_gap_mm', 'maximum_nominal_radial_gap_mm'}:
        raise ValueError('Candidate shaft/hub nominal mating interval is required')
    hub_bore = _positive(authored['authored_small_brake_hub_bore_diameter_mm'],
                         'authored small brake hub bore')
    target_shaft = _positive(authored['candidate_shaft_target_diameter_mm'],
                             'candidate shaft diameter')
    proposed_seat = CoaxialShaftHubSeat(target_shaft, hub_bore, **seat)
    source_gap = (hub_bore-source_shaft)/2.
    required_face = gap.second_face_after(puck_radius)
    surveyed_face = _positive(authored['surveyed_outboard_inner_face_z_mm'],
                              'surveyed outboard inner face')
    motor_torque = _positive(gripper['output_torque_limit_Nm'], 'gripper motor output limit')
    pitch_radius = _positive(gripper['pinion_pitch_radius_m'], 'gripper pitch radius')
    maximum_speed = _positive(gripper['max_jaw_velocity_m_s'], 'maximum jaw velocity')
    model_root = ET.fromstring((DATA/'P2-baseline.xml').read_bytes())
    tool_inertial = model_root.find(".//body[@name='tool']/inertial")
    if tool_inertial is None:
        raise ValueError('Pinned ARX model omits tool mass')
    archived_tool_mass = _positive(float(tool_inertial.get('mass')), 'archived tool mass')
    brake_mass = float(vendor['mass_kg'])
    brake_length = float(vendor['length_mm'])
    nominal_brake_center_z = required_face + brake_length/2.
    delay = _positive(vendor['delay_s'], 'brake model delay')
    ramp = _positive(vendor['ramp_s'], 'brake model ramp')
    source_paths = ('spec.json', 'P2-baseline-parameters.json', 'P2-baseline.xml',
                    'gripper-retention-proposal.json')
    for key in ('required_native_redesign', 'missing_external_evidence'):
        values = authored.get(key)
        if (not isinstance(values, list) or not values
                or any(not isinstance(value, str) or not value.strip() for value in values)):
            raise ValueError(f'{key} must name the unresolved interfaces')
    return dict(schema='cybrgeo-arx-gripper-brake-construction-input/1',
        source_commit=SOURCE_COMMIT,
        source_file_sha256={name: hashlib.sha256((DATA/name).read_bytes()).hexdigest()
                            for name in source_paths},
        resolved_input_sha256=dict(specification=_digest(spec), baseline=_digest(base),
                                   proposal=_digest(authored)),
        vendor_envelope=dict(part=vendor['proposed_part'], outer_diameter_mm=vendor['OD_mm'],
                             inner_diameter_mm=vendor['ID_mm'], axial_length_mm=brake_length,
                             mass_kg=brake_mass, mounting_pitch_circle_mm=vendor['mount_pcd_mm'],
                             mounting_hole_count=vendor['mount_count'],
                             holding_torque_static_Nm=vendor['holding_torque_Nm'],
                             coil_voltage_V=vendor['coil_voltage_V'],
                             coil_power_W=vendor['coil_power_W']),
        source_pinion_axis_tool_mm=[0., PINION_CENTER_Y_MM, 0.],
        source_upper_shaft_diameter_mm=source_shaft,
        authored_hub_bore_diameter_mm=hub_bore,
        source_direct_hub_radial_gap_mm=source_gap,
        source_direct_hub_seat_valid=bool(seat['minimum_nominal_radial_gap_mm'] <= source_gap
                                          <= seat['maximum_nominal_radial_gap_mm']),
        proposed_upper_shaft_diameter_mm=target_shaft,
        proposed_nominal_shaft_hub_seat=proposed_seat.report(),
        puck_outer_radius_mm=puck_radius,
        puck_half_length_mm=puck_half_length,
        tool_tcp_y_mm=tool_tcp,
        surveyed_outboard_inner_face_z_mm=surveyed_face,
        surveyed_outboard_nominal_gap_mm=surveyed_face-puck_radius,
        minimum_authored_outboard_inner_face_z_mm=required_face,
        outboard_gap_design=gap.report(),
        nominal_stator_outer_face_z_mm=required_face+brake_length,
        source_gripper_motor_limit_Nm=motor_torque,
        brake_static_to_motor_torque_ratio=vendor['holding_torque_Nm']/motor_torque,
        motor_force_limit_per_jaw_N=motor_torque/(2*pitch_radius),
        full_brake_rating_equivalent_force_per_jaw_N=vendor['holding_torque_Nm']/(2*pitch_radius),
        archived_tool_mass_kg=archived_tool_mass,
        added_brake_mass_fraction_of_archived_tool=brake_mass/archived_tool_mass,
        added_gravity_torque_bound_about_J5_Nm=brake_mass*9.80665*PINION_CENTER_Y_MM/1000.,
        added_gravity_torque_bound_about_J6_Nm=brake_mass*9.80665*nominal_brake_center_z/1000.,
        maximum_model_jaw_opening_during_delay_and_ramp_mm=maximum_speed*(delay+ramp)*1000.,
        nominal_mating_constructed=False, brake_installed=False,
        motor_off_payload_retained=False, manufacturing_qualified=False,
        required_native_redesign=authored['required_native_redesign'],
        missing_external_evidence=authored['missing_external_evidence'],
        scope='Source-bound diagnostic construction target only; no installed brake, supplier fit, fault retention or manufacturing qualification')


def require_nominal_brake_placement(native_upper_shaft_diameter_mm,
                                    brake_inner_face_z_mm, *, construction_input=None):
    """Fail before CAD placement if a claimed brake still uses the P5 shaft/gap.

    Call with dimensions derived from the *new* native pinion and proposed
    brake placement; this method cannot authenticate those solids itself.
    Passing its nominal interval is never a retention or hardware approval.
    """
    design = proposal() if construction_input is None else construction_input
    seat_design = design['proposed_nominal_shaft_hub_seat']
    seat = CoaxialShaftHubSeat(native_upper_shaft_diameter_mm,
        design['authored_hub_bore_diameter_mm'],
        seat_design['minimum_nominal_radial_gap_mm'],
        seat_design['maximum_nominal_radial_gap_mm'])
    gap_design = design['outboard_gap_design']
    gap = DirectedFaceGap(gap_design['required_service_gap_mm'],
        gap_design['first_declared_variation_mm'],
        gap_design['second_declared_variation_mm'],
        gap_design['design_reserve_mm'])
    gap.require_second_face_at_or_beyond(design['puck_outer_radius_mm'],
                                         brake_inner_face_z_mm)
    return dict(schema='cybrgeo-arx-nominal-brake-placement/1',
                shaft_hub_seat=seat.report(),
                outboard_inner_face_z_mm=float(brake_inner_face_z_mm),
                outboard_outer_face_z_mm=float(brake_inner_face_z_mm)
                                         + design['vendor_envelope']['axial_length_mm'],
                nominal_puck_to_stator_gap_mm=float(brake_inner_face_z_mm)
                                              - design['puck_outer_radius_mm'],
                authored_service_gap_mm=gap.required_service_gap_mm,
                declared_design_variations_mm=gap.declared_design_variations_mm(),
                process_capability_verified=False, native_solids_verified=False,
                motor_off_payload_retained=False, manufacturing_qualified=False,
                scope='Nominal shaft/hub and outboard face placement only; native solids, torque path, power-loss retention and hardware evidence remain required')


def candidate_recipe(specification):
    """Resolve the explicit diagnostic revision against pinned source geometry."""
    revision = specification.get('gripper_retention_revision')
    if revision is None:
        return None
    if revision != CANDIDATE_REVISION:
        raise ValueError('Unknown ARX gripper retention revision')
    design = proposal(specification=specification)
    authored = _load('gripper-retention-proposal.json')
    if authored.get('candidate_revision') != revision:
        raise ValueError('Candidate revision differs from its single native recipe')
    geometry = authored['candidate_native_geometry_mm']
    required = {
        'front_bearing_outer_diameter', 'front_bearing_inner_diameter',
        'front_bearing_z', 'front_wall_bore_diameter', 'front_wall_z',
        'stator_bracket_z', 'stator_inner_face_z',
        'stator_bracket_outer_radius', 'stator_bracket_inner_radius',
        'stator_standoff_centers_xy', 'stator_standoff_radius',
        'hub_outer_radius', 'armature_outer_radius', 'armature_thickness',
        'shaft_end_above_armature', 'key_z', 'capture_lip',
    }
    if set(geometry) != required:
        raise ValueError('Candidate CAD revision needs its complete authored geometry')
    bearing_z, wall_z, bracket_z, key_z = (
        geometry[name] for name in
        ('front_bearing_z', 'front_wall_z', 'stator_bracket_z', 'key_z'))
    for name, pair in (('front bearing', bearing_z), ('front wall', wall_z),
                       ('stator bracket', bracket_z), ('shaft key', key_z)):
        if (not isinstance(pair, list) or len(pair) != 2 or not all(
                isinstance(value, (int, float)) and math.isfinite(value)
                for value in pair) or pair[0] >= pair[1]):
            raise ValueError(f'Candidate {name} requires two ordered finite faces')
    if (geometry['front_bearing_inner_diameter'] <=
            design['proposed_upper_shaft_diameter_mm'] or
            geometry['front_bearing_outer_diameter'] <= geometry['front_bearing_inner_diameter'] or
            geometry['front_wall_bore_diameter'] <= design['proposed_upper_shaft_diameter_mm'] or
            not math.isclose(bracket_z[1], geometry['stator_inner_face_z'], abs_tol=1e-12) or
            bracket_z[0] < design['minimum_authored_outboard_inner_face_z_mm'] or
            geometry['stator_bracket_inner_radius'] <= geometry['hub_outer_radius'] or
            geometry['hub_outer_radius'] >= design['vendor_envelope']['inner_diameter_mm']/2 or
            geometry['armature_outer_radius'] >= design['vendor_envelope']['outer_diameter_mm']/2 or
            len(geometry['stator_standoff_centers_xy']) != 2):
        raise ValueError('Candidate brake bearing, bracket, shaft and vendor envelope do not mate')
    for xy in geometry['stator_standoff_centers_xy']:
        if (not isinstance(xy, list) or len(xy) != 2 or
                not all(isinstance(v, (int, float)) and math.isfinite(v) for v in xy) or
                xy[1] + PINION_CENTER_Y_MM >= 210):
            raise ValueError('Candidate stator standoffs cannot occupy the puck approach region')
    require_nominal_brake_placement(design['proposed_upper_shaft_diameter_mm'],
                                    bracket_z[0], construction_input=design)
    placement = require_nominal_brake_placement(
        design['proposed_upper_shaft_diameter_mm'],
        geometry['stator_inner_face_z'], construction_input=design)
    capture = _capture_lip_recipe(geometry['capture_lip'], design)
    stator_outer_z = placement['outboard_outer_face_z_mm']
    armature_outer_z = stator_outer_z + geometry['armature_thickness']
    if not key_z[0] > geometry['stator_inner_face_z'] or not key_z[1] < stator_outer_z:
        raise ValueError('Candidate key must remain within the brake hub seat')
    return dict(revision=revision, geometry=geometry, capture_lip=capture, proposal=design,
                placement=placement, stator_outer_z_mm=stator_outer_z,
                armature_outer_z_mm=armature_outer_z,
                shaft_end_z_mm=armature_outer_z + geometry['shaft_end_above_armature'])


def _capture_lip_recipe(authored, design):
    """One candidate solid recipe for both handed native jaws and contact boxes.

    The lip is integral with each source aluminum carriage. It reaches under
    the puck's load-facing axial face during the authored pickup pose; the
    actual motion and contact must still be checked on fresh CAD/MuJoCo.
    """
    required = {'nominal_puck_face_gap_mm', 'inner_reach_below_pad_face_mm',
        'ledge_outer_x_mm', 'ledge_depth_y_mm', 'ledge_half_width_z_mm',
        'stem_inner_x_mm', 'stem_outer_x_mm', 'stem_start_y_mm',
        'stem_end_below_ledge_top_mm'}
    if not isinstance(authored, dict) or set(authored) != required:
        raise ValueError('Capture lip needs one complete source-bound geometry recipe')
    g = {key: _positive(value, f'capture lip {key}') for key, value in authored.items()}
    source = ET.fromstring((DATA/'P2-baseline.xml').read_bytes())
    finger = source.find(".//geom[@name='structure_jaw_finger_jaw_left']")
    seat = source.find(".//geom[@name='nest_A_seat']")
    tcp = source.find(".//site[@name='tcp']")
    if finger is None or seat is None or tcp is None:
        raise ValueError('Pinned finger, puck nest and TCP geometry are required')
    finger_pos = [float(x)*1000 for x in finger.get('pos').split()]
    finger_half = [float(x)*1000 for x in finger.get('size').split()]
    source_finger = dict(x_min=finger_pos[0]-finger_half[0],
                         x_max=finger_pos[0]+finger_half[0],
                         y_end=JAW_ORIGIN_Y_MM+finger_pos[1]+finger_half[1])
    _same(float(tcp.get('pos').split()[1]), design['tool_tcp_y_mm']/1000.,
          'model TCP y')
    nest_radius = float(seat.get('size').split()[0])*1000.
    top = design['tool_tcp_y_mm'] + design['puck_half_length_mm'] + g['nominal_puck_face_gap_mm']
    stem_end = top + g['stem_end_below_ledge_top_mm']
    inner_x = -g['inner_reach_below_pad_face_mm']
    outer_x = g['ledge_outer_x_mm']
    if (not 0 < g['nominal_puck_face_gap_mm'] < 1.
            or not source_finger['x_min'] <= g['stem_inner_x_mm'] < outer_x
            or not g['stem_outer_x_mm'] <= source_finger['x_max']
            or not source_finger['y_end'] > g['stem_start_y_mm']
            or not stem_end > top
            or not g['stem_end_below_ledge_top_mm'] < g['ledge_depth_y_mm']
            or not design['puck_outer_radius_mm']-g['inner_reach_below_pad_face_mm'] > nest_radius
            or not inner_x < 0 < outer_x):
        raise ValueError('Capture lip must join the source finger, fit the puck rim and clear the nest seat')
    def box(x0, x1, y0, y1):
        return dict(center_tool_mm=[(x0+x1)/2, (y0+y1)/2, 0.],
                    half_size_mm=[(x1-x0)/2, (y1-y0)/2, g['ledge_half_width_z_mm']])
    boxes = dict(stem=box(g['stem_inner_x_mm'], g['stem_outer_x_mm'],
                          g['stem_start_y_mm'], stem_end),
                 ledge=box(inner_x, outer_x, top, top+g['ledge_depth_y_mm']))
    return dict(boxes=boxes, nominal_puck_face_y_mm=design['tool_tcp_y_mm']+design['puck_half_length_mm'],
        ledge_top_y_mm=top, nominal_axial_gap_mm=g['nominal_puck_face_gap_mm'],
        source_finger=source_finger, source_nest_seat_radius_mm=nest_radius,
        scope='Integral aluminum jaw lips, native CAD and matching collision boxes; no strength, continuous clearance or hardware retention qualification')


def fault_contact_eligibility(frames, onset_s, *, window_s=.05):
    """Require a two-pad suspended puck before a diagnostic motor-off fault.

    The sampled interval is deliberately strict: any puck contact except the
    two named pad geoms could externally support the part. Contact samples do
    not prove continuous retention or a tested brake.
    """
    if (isinstance(onset_s, bool) or not isinstance(onset_s, (int, float))
            or not math.isfinite(onset_s) or onset_s <= 0
            or isinstance(window_s, bool) or not isinstance(window_s, (int, float))
            or not math.isfinite(window_s) or window_s <= 0):
        raise ValueError('A positive finite fault time and observation window are required')
    recent = [row for row in frames if onset_s-window_s <= row['time_s'] < onset_s]
    if not recent or onset_s-recent[-1]['time_s'] > .02:
        raise ValueError('No sampled puck contacts sufficiently near motor-off onset')
    def partners(row):
        found = set()
        for contact in row['contacts']:
            geoms = contact['geoms']
            if len(geoms) != 2:
                raise ValueError('A contact must name two geoms')
            if 'puck' in geoms:
                found.add(geoms[1] if geoms[0] == 'puck' else geoms[0])
        return found
    expected = {'pad_left', 'pad_right'}
    last_partners = partners(recent[-1])
    other = sorted(set().union(*(partners(row)-expected for row in recent)))
    return dict(last_sample_time_s=recent[-1]['time_s'],
                observation_window_s=window_s, sampled_frames=len(recent),
                both_pads_contact_at_last_sample=expected <= last_partners,
                nonpad_puck_contact_geoms=other,
                no_external_workpiece_support=not other,
                passed=expected <= last_partners and not other,
                scope='Sampled puck contacts before motor-off; both pads required and every non-pad puck contact disqualifies suspended retention')
