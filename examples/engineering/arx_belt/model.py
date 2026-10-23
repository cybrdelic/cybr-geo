"""ARX belt as a client of GEO's shared native/physics/control architecture.

Direct adaptation of build_p5_model.py and controller.mjs configuration at
aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2. Geometry is authored in design.py;
generic mass, contact, rigid-body and motor machinery lives in mechanism_lab.
The original three ordinary failures remain preserved historical diagnostics;
the maintained task policy has separate source-bound fault evidence.
"""
from copy import deepcopy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from mechanism_lab.physical_model import PhysicalModel, digest
from mechanism_lab.contact_geometry import tabletop_tiles
from mechanism_lab.contact_geometry import ProxyBuilder

from .contacts import build as build_contacts
from .rack_contract import arx_rack_report, rack_travel_from_source
from .spring_anchor_contract import (arx_member_offsets, verify_arx_anchor_receipt,
                                     verify_arx_spring_joint_frames)
from .rack_contract import PINION_CENTER_Y_MM, JAW_ORIGIN_Y_MM
from .gripper_retention import CANDIDATE_PARTS, CANDIDATE_REVISION, candidate_recipe

DATA = Path(__file__).with_name('data')
SOURCE_COMMIT = 'aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2'


def attach_rotating_pinion(physical, travel):
    """Derive a physical pinion axis and both rack meshes from the P5 pitch.

    The archived P5 model is preserved as an input. This coupled modification
    is used only with a newly constructed pinion body and its own CAD inertia.
    It replaces the ideal tendon/one-jaw equality with a hinged pinion and two
    source-radius mesh constraints. Tooth flank contact remains an unresolved
    qualification requirement, rather than being inferred from an equality.
    """
    root = physical.root
    tool = root.find(".//body[@name='tool']")
    equality = root.find('equality')
    tendon = root.find(".//tendon/fixed[@name='rack_pinion_angle']")
    motor = root.find(".//actuator/motor[@name='gripper_motor']")
    home = root.find(".//keyframe/key[@name='home']")
    if (tool is None or equality is None or tendon is None or motor is None
            or home is None or root.find(".//joint[@name='gripper_pinion']") is not None):
        raise ValueError('Rotating pinion requires the exact pinned P5 rack topology')
    coordinates = [float(item) for item in home.get('qpos', '').split()]
    if len(coordinates) != 15 or not all(math.isfinite(item) for item in coordinates):
        raise ValueError('Pinned P5 home keyframe must have six arm, two jaw and seven free-body coordinates')
    home_gap = coordinates[6]
    if (not math.isclose(coordinates[7], home_gap, abs_tol=1e-12)
            or not travel.slide_min_mm/1000. < home_gap < travel.slide_max_mm/1000.):
        raise ValueError('Both source jaws must have the same interior home opening')
    pitch = travel.pitch_radius_m
    source_gripper = load('P2-baseline-parameters.json')['gripper']
    rail_friction = float(source_gripper['rail_friction_force_N_each'])
    motor_no_load_torque = (float(source_gripper['no_load_current_A']) *
                            float(source_gripper['output_torque_per_current_Nm_A']))
    if (not all(math.isfinite(value) and value >= 0 for value in
                (rail_friction, motor_no_load_torque)) or pitch <= 0):
        raise ValueError('Source gripper rail and motor no-load losses must be finite')
    reflected_slide_armatures = []
    reflected_slide_friction = []
    for name, sign in (('jaw_left', 1), ('jaw_right', -1)):
        body = tool.find(f"body[@name='{name}']")
        joint = None if body is None else body.find(f"joint[@name='{name}']")
        if body is None or joint is None or joint.get('ref') is not None:
            raise ValueError('Rotating pinion requires unshifted source jaw frames')
        origin = [float(item) for item in body.get('pos', '').split()]
        if (len(origin) != 3 or any(not math.isfinite(item) for item in origin)
                or any(not math.isclose(a, b, abs_tol=1e-12)
                       for a, b in zip(origin, (0., JAW_ORIGIN_Y_MM/1000., 0.)))):
            raise ValueError('Source jaw frame differs from native rack placement')
        # The P5 tendon motor has moved to the pinion. Retain the source
        # effective drive inertia without leaving it on the two jaw slides:
        # v_jaw = pitch * omega makes I_pinion = sum(m_slide) * pitch**2.
        # Native CAD jaw and pinion inertias remain separate.
        armature = float(joint.get('armature', '0'))
        if not math.isfinite(armature) or armature < 0:
            raise ValueError('Source jaw reflected armature must be finite and nonnegative')
        reflected_slide_armatures.append(armature)
        joint.set('armature', '0')
        # The old ideal tendon also split the motor's no-load Coulomb loss
        # across the jaws. Keep real rail friction on each slide, and react
        # the motor loss through its new physical pinion hinge instead.
        old_friction = float(joint.get('frictionloss', '0'))
        expected_friction = rail_friction + motor_no_load_torque/(2*pitch)
        if (not math.isfinite(old_friction) or
                not math.isclose(old_friction, expected_friction,
                                 rel_tol=1e-9, abs_tol=1e-12)):
            raise ValueError('Source slide friction does not match rail plus split motor no-load loss')
        reflected_slide_friction.append(old_friction)
        joint.set('frictionloss', f'{rail_friction:.15g}')
        # ``ref`` changes qpos0 but not the body's authored initial placement.
        # Shift the body by its source home opening to preserve world geometry
        # at q=.038 m while making the new pinion angle zero at home.
        body.set('pos', f'{sign*home_gap:.15g} {origin[1]:.15g} 0')
        joint.set('ref', f'{home_gap:.15g}')
    shaft = ET.SubElement(tool, 'body', name='gripper_pinion',
                          pos=f'0 {PINION_CENTER_Y_MM/1000.:.15g} 0')
    reflected_pinion_armature = sum(reflected_slide_armatures) * pitch * pitch
    reflected_pinion_friction = (sum(reflected_slide_friction) -
                                  2*rail_friction) * pitch
    motor_reflected_inertia = (float(source_gripper['motor_rotor_inertia_kg_m2']) *
                               float(source_gripper['gear_ratio'])**2)
    if (not math.isfinite(motor_reflected_inertia) or motor_reflected_inertia <= 0
            or not math.isclose(reflected_pinion_armature, motor_reflected_inertia,
                                rel_tol=1e-9, abs_tol=1e-15)
            or not math.isclose(reflected_pinion_friction, motor_no_load_torque,
                                rel_tol=1e-9, abs_tol=1e-12)):
        raise ValueError('Source jaw drivetrain inertia/loss does not match the gripper motor reflected to the pinion')
    ET.SubElement(shaft, 'joint', name='gripper_pinion', type='hinge', axis='0 0 1',
                  range=f'{(travel.slide_min_mm/1000.-home_gap)/pitch:.15g} '
                        f'{(travel.slide_max_mm/1000.-home_gap)/pitch:.15g}',
                  limited='true', damping='0',
                  frictionloss=f'{reflected_pinion_friction:.15g}',
                  armature=f'{reflected_pinion_armature:.15g}')
    original = equality.find("joint[@name='opposed_rack_synchronization']")
    if original is None or motor.get('tendon') != 'rack_pinion_angle':
        raise ValueError('Source jaw equality or motor transmission changed')
    equality.remove(original)
    # MuJoCo's refsafe floor raises a positive solref time constant below
    # 2*dt. Choose the integration step from this constructed mesh rather
    # than authoring a separate fault-suite-only numerical override. The
    # 20% margin keeps the requested stiffness effective at the source's
    # original step and leaves the 0.1 mm dynamic coupling budget unchanged.
    mesh_time_constant_s = .0005
    option = root.find('option')
    if option is None:
        raise ValueError('Rotating pinion requires a source integration option')
    source_step_s = float(option.get('timestep', 'nan'))
    if not math.isfinite(source_step_s) or source_step_s <= 0:
        raise ValueError('Source integration step must be finite and positive')
    integration_step_s = min(source_step_s, mesh_time_constant_s/(2*1.2))
    option.set('timestep', f'{integration_step_s:.15g}')
    # Solver equality softness must stay within the explicit 0.1 mm dynamic
    # coupling budget below. These numerical parameters are not gear flank
    # material properties, manufacturing backlash or tested capacity.
    for name in ('jaw_left', 'jaw_right'):
        ET.SubElement(equality, 'joint', name=f'{name}_pinion_mesh',
                      joint1=name, joint2='gripper_pinion',
                      polycoef=f'0 {pitch:.15g} 0 0 0',
                      solref=f'{mesh_time_constant_s:.15g} 1',
                      solimp='.9999 .99999 .000001')
    root.find('tendon').remove(tendon)
    del motor.attrib['tendon']
    motor.set('joint', 'gripper_pinion')
    coordinates.insert(8, 0.)  # hinge follows the two source jaw slides
    home.set('qpos', ' '.join(f'{item:.15g}' for item in coordinates))
    physical.provenance['rotating_pinion'] = dict(
        kind='derived CAD-bound hinge', source_commit=SOURCE_COMMIT,
        pitch_radius_m=pitch, home_jaw_opening_m=home_gap,
        cad_body='pinion', physical_body='gripper_pinion',
        source_slide_reflected_armatures_kg=reflected_slide_armatures,
        transferred_pinion_armature_kg_m2=reflected_pinion_armature,
        source_motor_reflected_inertia_kg_m2=motor_reflected_inertia,
        source_slide_friction_N=reflected_slide_friction,
        retained_rail_friction_N_each=rail_friction,
        transferred_pinion_friction_Nm=reflected_pinion_friction,
        source_motor_no_load_torque_Nm=motor_no_load_torque,
        source_integration_step_s=source_step_s,
        derived_integration_step_s=integration_step_s,
        mesh_time_constant_s=mesh_time_constant_s,
        transmission_transfer='Preserve source motor rotor kinetic energy with I=sum(m)*pitch_radius**2 = motor_rotor_inertia*gear_ratio**2 and dissipative power with torque=(sum(F)-2*rail_friction)*pitch_radius = no_load_current*torque_per_current; native CAD jaw and pinion inertias remain separate',
        scope='Ideal pitch coupling; rolling tooth contact, backlash, brake and shaft fit are unresolved')
    return dict(home_jaw_opening_m=home_gap, pitch_radius_m=pitch,
                source_slide_reflected_armatures_kg=reflected_slide_armatures,
                transferred_pinion_armature_kg_m2=reflected_pinion_armature,
                source_motor_reflected_inertia_kg_m2=motor_reflected_inertia,
                source_slide_friction_N=reflected_slide_friction,
                retained_rail_friction_N_each=rail_friction,
                transferred_pinion_friction_Nm=reflected_pinion_friction,
                source_motor_no_load_torque_Nm=motor_no_load_torque)


def load(name, directory=DATA):
    return json.loads((Path(directory) / name).read_text(encoding='utf-8'))


def compile_model(spec=None, *, native_directory=None):
    """Compile actual authored bodies and one source spec; never invent geometry.

    A supplied native_directory uses freshly regenerated mass and transmission
    records. The shipped reference records allow an explicitly cached run.
    """
    spec = deepcopy(spec or load('spec.json'))
    candidate = candidate_recipe(spec)
    if candidate is not None and native_directory is None:
        raise ValueError('ARX brake candidate requires fresh native CAD and mass records')
    records = Path(native_directory) if native_directory else DATA
    props = load('mass_properties.json', records)
    transmission = load('folded-transmissions.json', records)
    proxy_config = build_contacts(spec, transmission)
    if candidate is not None:
        receipt = load('gripper-retention-candidate.json', records)
        expected = set(CANDIDATE_PARTS)
        if (receipt.get('schema') != 'cybrgeo-arx-native-gripper-brake-candidate/1'
                or receipt.get('revision') != CANDIDATE_REVISION
                or receipt.get('canonical_specification_sha256') != digest(spec)
                or set(receipt.get('placed_candidate_parts', [])) != expected
                or receipt.get('native_placement') != candidate['placement']
                or receipt.get('authored_geometry_mm') != candidate['geometry']
                or receipt.get('capture_lip_recipe') != candidate['capture_lip']):
            raise ValueError('ARX brake candidate physical model lacks matching native construction')
        masses = receipt.get('candidate_part_mass_kg', {})
        components = {f'{body}__{row["name"]}': row['mass_kg']
            for body in ('tool', 'pinion') for row in props[body]['components']
            if f'{body}__{row["name"]}' in expected}
        if (set(components) != expected or set(masses) != expected
                or any(not math.isclose(components[name], masses[name], rel_tol=1e-12)
                       for name in expected)
                or not math.isclose(masses['tool__gripper_brake_stator_envelope'],
                                    spec['brakes']['small']['mass_kg'], rel_tol=1e-12)):
            raise ValueError('ARX brake candidate mass is not bound into both native rigid bodies')
        builder = ProxyBuilder(prefix='ARX_brake_candidate_')
        g = candidate['geometry']
        b = spec['brakes']['small']
        builder.annulus('stator','tool',b['ID_mm']/2,b['OD_mm']/2,
            g['stator_inner_face_z'],candidate['stator_outer_z_mm'],
            center_xy_mm=(0,PINION_CENTER_Y_MM),
            native_part='gripper_brake_stator_envelope')
        builder.annulus('integral_mount','tool',g['stator_bracket_inner_radius'],
            g['stator_bracket_outer_radius'],*g['stator_bracket_z'],
            center_xy_mm=(0,PINION_CENTER_Y_MM),native_part='gripper_bridge')
        for index,(x,y) in enumerate(g['stator_standoff_centers_xy']):
            builder.cylinder(f'standoff_{index}','tool',
                [x,y+PINION_CENTER_Y_MM,16],
                [x,y+PINION_CENTER_Y_MM,g['stator_bracket_z'][0]],
                g['stator_standoff_radius'],native_part='gripper_bridge')
        for sign, body, native in ((1, 'jaw_left', 'right_jaw_carriage_0'),
                                    (-1, 'jaw_right', 'left_jaw_carriage')):
            for name, box in candidate['capture_lip']['boxes'].items():
                x, y, z = box['center_tool_mm']
                builder.box(f'{body}_capture_{name}', body,
                    [sign*x, y-JAW_ORIGIN_Y_MM, z], box['half_size_mm'],
                    native_part=native,
                    approximation='Exact box used by the candidate integral jaw carriage CAD')
        proxy_config['records'].extend(builder.records)
        proxy_config['scope'] += ' Opt-in outboard brake stator, bracket and standoffs are conservative native-dimension proxies; two integral jaw capture lips use the same boxes as native CAD. Vendor internals, lip strength and full posed clearance remain unresolved.'
    physical = PhysicalModel.from_xml((DATA / 'P2-baseline.xml').read_text(), provenance=dict(
        repository='https://github.com/cybrdelic/cybr-arx6-belt', commit=SOURCE_COMMIT,
        design_spec_sha256=digest(spec), native_records='freshly generated' if native_directory else 'shipped source records',
        hardware_qualification='not qualified'))
    # Validate the unchanged P5 source slides/tendon first; a fresh native
    # build then replaces that ideal tendon with two source-pitch jaw meshes.
    travel = rack_travel_from_source(spec, model_root=physical.root)
    if native_directory:
        recorded = load('gripper-rack-travel.json', records)
        if recorded != arx_rack_report(travel):
            raise ValueError('Native rack construction disagrees with the compiled physical slide and tendon')
        physical.provenance['gripper_rack_travel'] = recorded
        anchors = load('spring-anchor-load-path.json', records)
        anchor_plan = verify_arx_anchor_receipt(
            spec, transmission, anchors, load('placed-native-parts.json', records),
            load('cad_parts.json', records))
        verify_arx_spring_joint_frames(physical.root, anchor_plan)
        physical.provenance['spring_anchor_load_path_plan_sha256'] = anchor_plan['plan_sha256']
    # Archived/cached P5 records retain the original tendon model. A fresh
    # native build has the separately integrated pinion mass and uses the
    # revised physical transmission instead of silently reusing P5 inertias.
    if native_directory:
        if 'pinion' not in props:
            raise ValueError('A rotating gripper pinion requires its own fresh native CAD mass')
        pinion = attach_rotating_pinion(physical, travel)
        physical.root.set('model', 'ARX6_P5_derived_outboard_brake_capture_candidate' if candidate else
                          'ARX6_P5_derived_rotating_pinion')
    else:
        pinion = None
        physical.root.set('model', 'ARX6_P5_compact_belt_drives')
    physical.bind_inertias(props, {name: name for name in ('turret', 'upper', 'fore', 'roll', 'bend', 'tool')})
    if pinion is not None:
        physical.bind_inertias(props, {'gripper_pinion': 'pinion'})
    for name in ('jaw_left', 'jaw_right'):
        if name in props:
            physical.bind_inertias(props, {name: name})
    if native_directory and 'jaw' in props:
        # Both handed CAD assemblies are integrated independently. Different
        # rack seats/bores must not disappear into a reflection approximation.
        physical.bind_inertias(props, {'jaw_left': 'jaw'})
        physical.bind_inertias(props, {'jaw_right': 'jaw_opposed'})
    for index, limit in enumerate(spec['joint_hard_limits_rad']):
        physical.set_joint(f'j{index+1}', range=limit)
    physical.remove_geoms(names=proxy_config.get('replace_names', []), prefixes=proxy_config['replace_prefixes'])
    for row in proxy_config['records']:
        physical.add_proxy(row, body_aliases={'fixed': 'fixed_hardware'})
    physical.remove_geoms(names=['floor'])
    tabletop = proxy_config['tabletop']
    for row in tabletop_tiles(tabletop, first_name='floor', name_prefix='P5_table_material_'):
        physical.add_proxy(row)
    contact = physical.root.find('contact')
    for item in list(contact.findall('exclude')):
        if {item.get('body1'), item.get('body2')} == {'fixed_hardware', 'turret'}:
            contact.remove(item)
    for row in proxy_config.get('body_exclusions', []):
        if not row.get('reason'):
            raise ValueError('A contact exclusion requires its geometric rationale')
        import xml.etree.ElementTree as ET
        ET.SubElement(contact, 'exclude', body1=row['body1'], body2=row['body2'])
    p = load('P2-baseline-parameters.json')
    if (not math.isclose(p['gripper']['pinion_pitch_radius_m'], travel.pitch_radius_m,
                         rel_tol=1e-12, abs_tol=1e-12)
            or not math.isclose(p['gripper']['module_m'], travel.module_mm/1000.,
                                rel_tol=1e-12, abs_tol=1e-12)
            or p['gripper']['pinion_teeth'] != travel.pinion_teeth):
        raise ValueError('Configured gripper drive differs from the constructed rack pitch')
    # The baseline includes historical inertias for its browser controller.
    # Carry only the masses actually bound into this physical tree, so exported
    # task metadata cannot advertise stale CAD values beside the new model.
    p['bodies'] = deepcopy(physical.mass_records)
    p['integration'] = spec
    p['model_revision'] = (CANDIDATE_REVISION if candidate is not None else
        'P5-derived-rotating-pinion' if pinion is not None else 'P5')
    if candidate is not None:
        physical.provenance['gripper_retention_candidate'] = dict(
            revision=CANDIDATE_REVISION,
            native_receipt_sha256=digest(receipt),
            capture_lip_recipe_sha256=digest(candidate['capture_lip']),
            full_posed_clearance_verified=False,
            loaded_power_loss_retention_verified=False,
            configured_vendor_brake_verified=False,
            scope='CAD-bound diagnostic brake and integral jaw capture; no lip strength, supplier or hardware qualification')
    p['controller']['ki_Nm_rad_s'][1:3] = [80., 60.]
    efficiency = spec['folded_transmission'].get('efficiency_screening', .97)
    if isinstance(efficiency, (float, int)):
        efficiency = [efficiency] * 6
    p['motors']['gear_efficiency'] = [a*b for a, b in zip(p['motors']['gear_efficiency'], efficiency)]
    extra = proxy_config['joint_extra_armature_kg_m2']
    for index, additional in enumerate(extra):
        joint = physical.root.find(f".//joint[@name='j{index+1}']")
        physical.set_joint(f'j{index+1}', armature=f"{float(joint.get('armature', '0'))+additional:.15g}")
    p['motors']['extra_transmission_inertia_kg_m2'] = extra
    p['motors']['belt_screen_capacity_Nm'] = [19.1984]*3 + [3.7422]*3
    p['motors']['belt_service_factor'] = 1.5
    p['motors']['belt_allowable_output_torque_Nm'] = [value/1.5 for value in p['motors']['belt_screen_capacity_Nm']]
    physical.require_free_body('puck')
    physical.provenance['contacts'] = dict(policy='conservative material-region overcoverage',
        scope=proxy_config.get('scope'), omitted='flexible belt, spring and cable contact; detailed small holes')
    return physical, control_source(p), p


def control_source(p):
    """Map example identities to shared motors; no physical equations here."""
    spec, cfg, motors = p['integration'], p['controller'], p['motors']
    drives = []
    for index in range(6):
        motor = {key: (value[index] if isinstance(value, list) else value) for key, value in motors.items()}
        drives.append(dict(joint=f'j{index+1}', actuator=f'motor{index+1}', motor=motor,
            kp=cfg['kp_Nm_rad'][index], kd=cfg['kd_Nm_s_rad'][index], ki=cfg['ki_Nm_rad_s'][index],
            integrator_limit=cfg['integrator_limit_rad_s'], soft_limits=spec['joint_soft_limits_rad'][index],
            brake=spec['brakes']['large' if index < 3 else 'small']))
    gripper = deepcopy(p['gripper'])
    rack = dict(joints=['jaw_left', 'jaw_right'], actuator='gripper_motor', motor=gripper,
                current_budget=spec['gripper_driver'])
    if p['model_revision'] in ('P5-derived-rotating-pinion', CANDIDATE_REVISION):
        rack.update(pinion_joint='gripper_pinion', pinion_body='gripper_pinion',
                    maximum_coupling_error_m=.0001,
                    coupling_budget_scope='Authored 0.1 mm solver/kinematic discrepancy ceiling; '
                        'not measured tooth backlash or production fit')
    if p['model_revision'] == CANDIDATE_REVISION:
        # A finite, delayed PM holding torque reacts through the physical
        # pinion hinge into its tool parent. This is a reduced-order actuator
        # model, not rotor/stator contact or measured brake engagement.
        rack['pinion_brake'] = deepcopy(spec['brakes']['small'])
    for name in ('pinion_pitch_radius_m', 'position_to_velocity_gain_per_s', 'max_jaw_velocity_m_s',
                 'velocity_force_gain_N_s_m', 'output_torque_limit_Nm', 'output_torque_per_current_Nm_A', 'no_load_current_A'):
        rack[name] = gripper[name]
    return dict(schema='cybrgeo-control/1', drives=drives, rack=rack,
        springs=[dict(joint='j2', parameters=spec['counterbalance'], axis=[0, 0, -1],
                      member_offsets_m=arx_member_offsets(spec['counterbalance'])),
                 dict(joint='j3', parameters=spec['elbow_counterbalance'], axis=[0, 0, -1],
                      member_offsets_m=arx_member_offsets(spec['elbow_counterbalance']),
                      scale_option='elbowSpringScale')],
        gravity_and_coriolis_feedforward=cfg['gravity_and_coriolis_feedforward'],
        inertial_feedforward=cfg['inertial_feedforward'], initial_keyframe='home',
        provenance=dict(commit=SOURCE_COMMIT, source='controller.mjs and safety.mjs; example task stays in task.py'))
