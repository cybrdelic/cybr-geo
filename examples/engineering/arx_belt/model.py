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

from mechanism_lab.physical_model import PhysicalModel, digest
from mechanism_lab.contact_geometry import tabletop_tiles

from .contacts import build as build_contacts
from .rack_contract import arx_rack_report, rack_travel_from_source
from .spring_anchor_contract import verify_arx_anchor_receipt, verify_arx_spring_joint_frames

DATA = Path(__file__).with_name('data')
SOURCE_COMMIT = 'aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2'


def load(name, directory=DATA):
    return json.loads((Path(directory) / name).read_text(encoding='utf-8'))


def compile_model(spec=None, *, native_directory=None):
    """Compile actual authored bodies and one source spec; never invent geometry.

    A supplied native_directory uses freshly regenerated mass and transmission
    records. The shipped reference records allow an explicitly cached run.
    """
    spec = deepcopy(spec or load('spec.json'))
    records = Path(native_directory) if native_directory else DATA
    props = load('mass_properties.json', records)
    transmission = load('folded-transmissions.json', records)
    proxy_config = build_contacts(spec, transmission)
    physical = PhysicalModel.from_xml((DATA / 'P2-baseline.xml').read_text(), provenance=dict(
        repository='https://github.com/cybrdelic/cybr-arx6-belt', commit=SOURCE_COMMIT,
        design_spec_sha256=digest(spec), native_records='freshly generated' if native_directory else 'shipped source records',
        hardware_qualification='not qualified'))
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
    physical.root.set('model', 'ARX6_P5_compact_belt_drives')
    physical.bind_inertias(props, {name: name for name in ('turret', 'upper', 'fore', 'roll', 'bend', 'tool')})
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
    p['model_revision'] = 'P5'
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
    for name in ('pinion_pitch_radius_m', 'position_to_velocity_gain_per_s', 'max_jaw_velocity_m_s',
                 'velocity_force_gain_N_s_m', 'output_torque_limit_Nm', 'output_torque_per_current_Nm_A', 'no_load_current_A'):
        rack[name] = gripper[name]
    return dict(schema='cybrgeo-control/1', drives=drives, rack=rack,
        springs=[dict(joint='j2', parameters=spec['counterbalance'], axis=[0, 0, -1]),
                 dict(joint='j3', parameters=spec['elbow_counterbalance'], axis=[0, 0, -1], scale_option='elbowSpringScale')],
        gravity_and_coriolis_feedforward=cfg['gravity_and_coriolis_feedforward'],
        inertial_feedforward=cfg['inertial_feedforward'], initial_keyframe='home',
        provenance=dict(commit=SOURCE_COMMIT, source='controller.mjs and safety.mjs; example task stays in task.py'))
