"""New IRIS-derived rigid linkage, authored through shared GEO services.

The optical source supplied no linkage or actuator model. This module supplies
an explicitly new mechanism: one annular rotor, nine real bored straps, nine
passive leaf shafts and a fixed reaction housing. No follower state is assigned
by the controller. Original source-study geometry remains in design.py.
"""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import cadquery as cq
import numpy as np

from mechanism_lab.linkages import PinnedLink
from mechanism_lab.native_construction import NativeConstruction, NativeInterfaceRequirements, read_specification
from mechanism_lab.physical_model import PhysicalModel, digest
from . import design
from .leaf_geometry import build_leaf

HERE = Path(__file__).resolve().parent
DENSITIES = dict(aluminum=2700., steel=7850., brass=8500., glass=2500.)
MATERIALS = {1: 'aluminum', 2: 'aluminum', 3: 'brass', 4: 'steel', 5: 'aluminum', 6: 'glass'}


def specification():
    value = json.loads((HERE / 'data' / 'mechanism.json').read_text(encoding='utf-8'))
    value['source_geometry'] = deepcopy(design.SOURCE)
    return value


def validate(spec):
    spec = deepcopy(spec)
    if spec.get('schema') != 'cybrgeo-iris-derived/1' or spec.get('source_geometry') != design.SOURCE:
        raise ValueError('IRIS mechanism requires the exact bound source geometry mapping')
    if spec['scene_unit_mm'] != 50. or spec['hinge_range_rad'] != [0., .9]:
        raise ValueError('This derived housing layout is authored at 50 mm per scene unit over 0..0.9 rad')
    positive = ('rotor_pin_radius_mm', 'leaf_crank_length_mm', 'rod_length_mm', 'rod_width_mm',
                'rod_thickness_mm', 'pin_diameter_mm', 'diametral_clearance_mm', 'minimum_wall_mm',
                'rod_height_mm', 'period_s', 'timestep_s', 'closure_limit_mm', 'leaf_tracking_limit_rad',
                'ring_tracking_limit_rad')
    for key in positive:
        if isinstance(spec[key], bool) or not math.isfinite(spec[key]) or spec[key] <= 0:
            raise ValueError(f'{key} must be finite and positive')
    # The existing source housing, shaft grooves and radial supports are an
    # authored layout. Changing these coupled dimensions needs a new design;
    # accepting arbitrary positive values would leave disconnected material.
    authored = dict(rotor_pin_radius_mm=60., leaf_crank_length_mm=20., leaf_crank_angle_rad=.45,
        rod_length_mm=20., rod_width_mm=4., pin_diameter_mm=2., diametral_clearance_mm=.05,
        minimum_wall_mm=.7)
    for key, value in authored.items():
        if spec[key] != value:
            raise ValueError(f'{key} is fixed at {value} in this authored radial housing/shaft layout; redesign its mating geometry first')
    axial_stack(spec)
    return spec


def ring_angle(q, spec):
    """Circle-intersection construction relation, not a follower controller."""
    h = spec['source_geometry']['leaf']['hinge_radius'] * spec['scene_unit_mm']
    b, beta, radius, length = (spec[key] for key in ('leaf_crank_length_mm', 'leaf_crank_angle_rad',
                                                  'rotor_pin_radius_mm', 'rod_length_mm'))
    x, y = h + b * math.cos(beta - q), b * math.sin(beta - q)
    distance = math.hypot(x, y)
    cosine = (distance * distance + radius * radius - length * length) / (2 * distance * radius)
    if not -1 < cosine < 1:
        raise ValueError('Specified linkage is unreachable or at a toggle')
    return math.atan2(y, x) + math.acos(cosine)


def axial_stack(spec):
    """One mating stack for rods, captured pins and the fixed rotor lips."""
    cfg = spec['axial_stack']
    for name, value in cfg.items():
        if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'axial_stack.{name} must be finite and positive')
    bottom = spec['rod_height_mm'] - spec['rod_thickness_mm']/2
    top = spec['rod_height_mm'] + spec['rod_thickness_mm']/2
    gap, keeper = cfg['rod_keeper_clearance_mm'], cfg['keeper_thickness_mm']
    input_keeper = (bottom-gap-keeper, bottom-gap)
    output_keeper = (top+gap, top+gap+keeper)
    input_pin_bottom = input_keeper[0]-cfg['pin_tip_extension_mm']
    output_pin_top = output_keeper[1]+cfg['pin_tip_extension_mm']
    moving_upper_envelope = max(top, output_keeper[1], output_pin_top)
    lower_lip_bottom = moving_upper_envelope+cfg['moving_pin_to_fixed_lip_clearance_mm']
    lower_lip_top = lower_lip_bottom+cfg['capture_lip_thickness_mm']
    rotor_bottom = lower_lip_top+cfg['rotor_capture_axial_clearance_mm']
    rotor_top = cfg['rotor_top_mm']
    if rotor_bottom+cfg['pin_root_overlap_mm'] >= rotor_top:
        raise ValueError('Axial mating stack leaves no annular rotor thickness')
    if rotor_top != 38. or input_pin_bottom < 30.2 or rotor_bottom > 35.4:
        raise ValueError('Axial mating stack lies outside the authored crank and handle support envelope')
    if min(gap, cfg['pin_tip_extension_mm']) <= cfg['keeper_groove_clearance_mm']:
        raise ValueError('Axial keeper groove must retain positive shoulder and pin-tip material')
    radial_bearing = (32.5, 35.5)
    # The authored radial bearing also surrounds the outward-sweeping keeper.
    # Checking only the inner capture lip would miss this second mating face.
    keeper_clearance = cfg['keeper_to_radial_bearing_clearance_mm']
    if output_keeper[1]+keeper_clearance > radial_bearing[0]+1e-10:
        raise ValueError('Axial keeper stack violates the authored radial-bearing clearance; redesign its mating support first')
    if min(lower_lip_top, radial_bearing[1]) <= max(lower_lip_bottom, radial_bearing[0]):
        raise ValueError('Lower capture lip must overlap the authored radial bearing in Z')
    return dict(rod_bottom_mm=bottom, rod_top_mm=top,
        input_keeper_mm=input_keeper, output_keeper_mm=output_keeper,
        input_pin_bottom_mm=input_pin_bottom, output_pin_top_mm=output_pin_top,
        moving_upper_envelope_mm=moving_upper_envelope, radial_bearing_mm=radial_bearing,
        lower_lip_mm=(lower_lip_bottom, lower_lip_top),
        rotor_mm=(rotor_bottom, rotor_top),
        pin_root_top_mm=rotor_bottom+cfg['pin_root_overlap_mm'],
        upper_lip_mm=(rotor_top+cfg['rotor_capture_axial_clearance_mm'],
                      rotor_top+cfg['rotor_capture_axial_clearance_mm']+cfg['capture_lip_thickness_mm']))


def layout(spec):
    spec = validate(spec)
    count = spec['source_geometry']['leaf']['count']
    start, end = spec['hinge_range_rad']
    phi0, phi1 = ring_angle(start, spec), ring_angle(end, spec)
    samples = [ring_angle(q, spec) for q in np.linspace(start, end, 901)]
    if not all(a > b for a, b in zip(samples, samples[1:])):
        raise ValueError('Chosen circle-intersection branch is not monotone across the task')
    rotor_frame = design._frame(phi0)
    frames, links = {'fixed': np.eye(4), 'rotor': rotor_frame}, []
    radius, height = spec['rotor_pin_radius_mm'], spec['rod_height_mm']
    b, beta = spec['leaf_crank_length_mm'], spec['leaf_crank_angle_rad']
    for index in range(count):
        name, azimuth = f'leaf_{index:02}', index * math.tau / count
        frames[name] = design.leaf_frame(index, start, spec['scene_unit_mm'])
        link = PinnedLink.from_frames(f'rod_{index:02}', 'rotor', name, rotor_frame, frames[name],
            [radius * math.cos(azimuth), -radius * math.sin(azimuth), height],
            [b * math.cos(beta), b * math.sin(beta), height],
            width_mm=spec['rod_width_mm'], thickness_mm=spec['rod_thickness_mm'],
            pin_diameter_mm=spec['pin_diameter_mm'], diametral_clearance_mm=spec['diametral_clearance_mm'],
            minimum_wall_mm=spec['minimum_wall_mm'])
        if not math.isclose(link.length_mm, spec['rod_length_mm'], abs_tol=1e-8):
            raise ValueError('Physical pin-frame length differs from the requested strap')
        local = design._frame(link.input_relative_yaw_rad, link.input_anchor_mm)
        frames[link.name] = rotor_frame @ local
        links.append(link)
    return dict(phi0=phi0, phi1=phi1, frames=frames, links=links, axial_stack=axial_stack(spec),
                ring_delta_rad=phi1-phi0, count=count)


def _cylinder(radius, bottom, top, xy=(0, 0)):
    return cq.Workplane('XY').circle(radius).extrude(top-bottom).val().translate((*xy, bottom))


def _annulus(inner, outer, bottom, top):
    return cq.Workplane('XY').circle(outer).circle(inner).extrude(top-bottom).val().translate((0, 0, bottom))


def _strap(length, width, bottom, top, angle=0.):
    shape = cq.Workplane('XY').center(length/2, 0).slot2D(length+width, width).extrude(top-bottom).val()
    return shape.translate((0, 0, bottom)).rotate((0, 0, 0), (0, 0, 1), math.degrees(angle))


def _split_keeper(inner, outer, bottom, top, xy=(0, 0)):
    ring = _annulus(inner, outer, bottom, top)
    mouth = cq.Workplane('XY').box(2*outer, .4, top-bottom+1).val().translate((-outer, 0, (top+bottom)/2))
    return ring.cut(mouth).translate((*xy, 0))


def moving_geometry(spec):
    """Body-local native solids and exact pin frames used by both backends."""
    info = layout(spec)
    blade, crop = build_leaf(spec['scene_unit_mm'], tuple(spec['hinge_range_rad']))
    # Neighbor shaft keepouts are provided by the shared geometric authoring
    # helper in leaf_geometry; its receipt describes the conservative envelope.
    from .leaf_geometry import cut_fixed_shaft_keepouts
    blade, keepouts = cut_fixed_shaft_keepouts(blade, scene_unit_mm=spec['scene_unit_mm'],
        hinge_range_rad=tuple(spec['hinge_range_rad']), shaft_radius_mm=1.8, clearance_mm=.1)
    b, beta = spec['leaf_crank_length_mm'], spec['leaf_crank_angle_rad']
    xy = (b*math.cos(beta), b*math.sin(beta))
    pin_radius = spec['pin_diameter_mm']/2
    stack = info['axial_stack']
    input_keeper, output_keeper = stack['input_keeper_mm'], stack['output_keeper_mm']
    groove_gap = spec['axial_stack']['keeper_groove_clearance_mm']
    crank = _strap(b, 4., 28.5, 30., beta)
    shaft = _cylinder(1., 13., 29.5)
    hub = _cylinder(1.8, 12.95, 13.1)
    output_pin = _cylinder(pin_radius, 29.9, stack['output_pin_top_mm'], xy)
    output_shoulder = _cylinder(1.8, 29.9, input_keeper[1], xy)
    leaf_body = blade.fuse(hub, shaft, crank, output_pin, output_shoulder).clean()
    for bottom, top, center in ((21.58, 21.82, (0, 0)), (23.68, 23.92, (0, 0)),
                               (output_keeper[0]-groove_gap, output_keeper[1]+groove_gap, xy)):
        leaf_body = leaf_body.cut(_annulus(.8, 1.01, bottom, top).translate((*center, 0)))
    rotor = _annulus(58., 64., *stack['rotor_mm'])
    for link in info['links']:
        rotor = rotor.fuse(_cylinder(pin_radius, stack['input_pin_bottom_mm'], stack['pin_root_top_mm'], link.input_anchor_mm[:2]),
                           _cylinder(1.8, output_keeper[0], stack['pin_root_top_mm'], link.input_anchor_mm[:2]))
        rotor = rotor.cut(_annulus(.8, 1.01, input_keeper[0]-groove_gap, input_keeper[1]+groove_gap).translate((*link.input_anchor_mm[:2], 0)))
    # Reduced source-handle travel is centred in the existing curved slot.
    phase = -.16 - (info['phi0'] + info['phi1'])/2
    upper = cq.Workplane('XY').box(12., 3., 1.5).val().translate((66., 0, 40.))
    stem = cq.Workplane('XY').box(2., 3., 5.).val().translate((61., 0, 38.))
    drop = cq.Workplane('XY').box(2., 2., 12.).val().translate((71.95, 0, 34.))
    rotor = rotor.fuse(design._rotate(upper.fuse(stem, drop), phase)).clean()
    handle_other = []
    for name, shape, material in design.handle_geometry(spec['scene_unit_mm']):
        shape = design._rotate(shape, phase)
        if MATERIALS[material] == 'steel':
            rotor = rotor.fuse(shape)
        else:
            handle_other.append((name, shape, MATERIALS[material]))
    rotor = rotor.clean()
    rows = [('rotor', 'annular_rotor_and_drive_pins', rotor, 'steel')]
    for name, shape, material in handle_other:
        rows.append(('rotor', name, shape.cut(rotor).clean(), material))
    for index, link in enumerate(info['links']):
        # Split retainers sit in actual shaft grooves with 0.02 mm nominal
        # radial/axial fit. Spring installation and retention strength are unqualified.
        keeper = _split_keeper(.82, 1.6, *input_keeper, link.input_anchor_mm[:2])
        rows.append(('rotor', f'input_pin_keeper_{index:02}', keeper, 'steel'))
        rows.append((f'leaf_{index:02}', 'leaf_shaft_crank', leaf_body, 'steel'))
        rows.append((f'leaf_{index:02}', 'output_pin_keeper',
                     _split_keeper(.82, 1.6, *output_keeper, xy), 'steel'))
        rows.append((f'leaf_{index:02}', 'lower_shaft_collar', _split_keeper(.82, 1.8, 21.6, 21.8), 'steel'))
        rows.append((f'leaf_{index:02}', 'upper_shaft_collar', _split_keeper(.82, 1.8, 23.7, 23.9), 'steel'))
        rows.append((link.name, 'bored_connecting_rod', link.native_shape(), 'steel'))
    return rows, info, dict(crop=crop, shaft_keepouts=keepouts)


def build(spec, output):
    spec, identity = read_specification(spec)
    spec = validate(spec)
    rows, info, leaf_report = moving_geometry(spec)
    source = dict(spec['source_geometry']['source'])
    source.update(identity, adapted_recipe_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  scope='New rigid pinned-link mechanism derived from source optical geometry; no hardware qualification',
                  assumptions=spec['assumptions'])
    native = NativeConstruction(output, DENSITIES, provenance=source)
    # The new bearing flange is one nominal piece with the source retainer;
    # fixed source material is partitioned before adding it to mass accounting.
    bearing = _annulus(40., 52., 22., 23.5)
    h = spec['source_geometry']['leaf']['hinge_radius'] * spec['scene_unit_mm']
    bore_radius = (spec['pin_diameter_mm'] + spec['diametral_clearance_mm'])/2
    for index in range(info['count']):
        a = index*math.tau/info['count']
        bearing = bearing.cut(_cylinder(bore_radius, 21.5, 24., (h*math.cos(a), -h*math.sin(a))))
    stack = info['axial_stack']
    radial_bearing = _annulus(64.05, 66.8, *stack['radial_bearing_mm'])
    lower_lip = _annulus(63., 66.8, *stack['lower_lip_mm'])
    radial_bearing = radial_bearing.fuse(lower_lip, _annulus(63., 66.8, *stack['upper_lip_mm']))
    motor_stator = _annulus(64.4, 69., 35.5, 38.5)
    fixed_drive = radial_bearing.fuse(motor_stator)
    for index in range(3):
        a = (.2+index*math.tau/3)
        fixed_drive = fixed_drive.fuse(_cylinder(1.5, 26.8, 33.5, (66*math.cos(a), 66*math.sin(a))))
    housing = design.housing_geometry(spec['scene_unit_mm'])
    attachments = {}
    for name, shape, material in housing:
        addition = bearing if name == 'housing_material_3' else fixed_drive if name == 'housing_material_2' else None
        if addition is not None:
            partition = []
            for other_name, other, _ in housing:
                if other_name == name or not design._overlaps_bounds(addition, other):
                    continue
                overlap = float(addition.intersect(other).Volume())
                if overlap > 1e-8:
                    addition = addition.cut(other)
                    partition.append(dict(other_part=other_name, removed_overlap_mm3=overlap))
            joint_volume = float(shape.intersect(addition).Volume())
            old_count = len(shape.Solids())
            # Keep the valid Boolean topology: OCC's optional same-domain
            # face unifier corrupts this source face's fine recess edges.
            shape = shape.fuse(addition)
            if joint_volume <= 1e-5 or not shape.isValid() or len(shape.Solids()) > old_count:
                raise ValueError('New fixed support must join its actual source housing material: '+name)
            attachments[name] = dict(positive_join_volume_mm3=joint_volume,
                                     source_solid_count=old_count, final_solid_count=len(shape.Solids()),
                                     other_material_partition=partition)
        rows.append(('fixed', name, shape, MATERIALS[material]))
    for name, shape, material in design.lens_geometry(spec['scene_unit_mm']):
        rows.append(('fixed', name, shape, MATERIALS[material]))
    for body, name, shape, material in rows:
        unique = body + '__' + name
        native.addpart(unique, shape, material, 'New derived mechanism' if body != 'fixed' else 'Source housing with disclosed new bearing and reaction features')
        native.place(body, name, shape, material)
    requirements = NativeInterfaceRequirements(dict(scope='Nominal mating CAD and a solved initial closure; ideal bearings in dynamics'))
    # This full radial disk bounds the upper axial extent relevant to the
    # lip, at every planar angle. Lower pin/keeper material lies below it.
    moving_envelope = _cylinder(h+spec['leaf_crank_length_mm']+spec['rod_width_mm'],
                               stack['rod_bottom_mm'], stack['moving_upper_envelope_mm'])
    row = requirements.noninterference('moving_pin_envelope_to_lower_rotor_lip', moving_envelope, lower_lip)
    row['required_minimum_distance_mm'] = spec['axial_stack']['moving_pin_to_fixed_lip_clearance_mm']
    row['pass'] = row['pass'] and row['minimum_distance_mm'] >= row['required_minimum_distance_mm']-1e-8
    row['scope'] = 'Continuous planar axial envelope; not a whole-assembly continuous clearance proof'
    keeper_envelope = _cylinder(h+spec['leaf_crank_length_mm']+1.6, *stack['output_keeper_mm'])
    row = requirements.noninterference('output_keeper_envelope_to_radial_bearing', keeper_envelope, radial_bearing)
    row['required_minimum_distance_mm'] = spec['axial_stack']['keeper_to_radial_bearing_clearance_mm']
    row['pass'] = row['pass'] and row['minimum_distance_mm'] >= row['required_minimum_distance_mm']-1e-8
    row['scope'] = 'Continuous planar keeper envelope against radial bearing only; support posts require separate clearance assessment'
    for index, link in enumerate(info['links']):
        pin = _cylinder(spec['pin_diameter_mm']/2, -.6, .6)
        requirements.noninterference(f'rod_{index:02}_input_pin_clearance', link.native_shape(), pin)
        requirements.noninterference(f'rod_{index:02}_output_pin_clearance', link.native_shape(), pin.translate((link.length_mm, 0, 0)))
        lower = _split_keeper(.82, 1.6, *(z-spec['rod_height_mm'] for z in stack['input_keeper_mm']))
        upper = _split_keeper(.82, 1.6, *(z-spec['rod_height_mm'] for z in stack['output_keeper_mm']), (link.length_mm, 0))
        for label, keeper in (('input', lower), ('output', upper)):
            row = requirements.noninterference(f'rod_{index:02}_{label}_axial_keeper_clearance', link.native_shape(), keeper)
            row['required_minimum_distance_mm'] = spec['axial_stack']['rod_keeper_clearance_mm']
            row['pass'] = row['pass'] and row['minimum_distance_mm'] >= row['required_minimum_distance_mm']-1e-8
        pivot = _cylinder(1., 21.8, 23.7, (h*math.cos(index*math.tau/info['count']), -h*math.sin(index*math.tau/info['count'])))
        requirements.noninterference(f'leaf_{index:02}_bearing_clearance', bearing, pivot)
        xy = (h*math.cos(index*math.tau/info['count']), -h*math.sin(index*math.tau/info['count']))
        for label, bottom, top in (('lower', 21.6, 21.8), ('upper', 23.7, 23.9)):
            collar = _split_keeper(.82, 1.8, bottom, top, xy)
            row = requirements.noninterference(f'leaf_{index:02}_{label}_axial_collar_clearance', bearing, collar)
            row['required_minimum_distance_mm'] = .19
            row['pass'] = row['pass'] and row['minimum_distance_mm'] >= .19
    rotor_shape = next(shape for body, name, shape, _ in rows if name == 'annular_rotor_and_drive_pins')
    requirements.noninterference('rotor_radial_and_axial_bearing', design._rotate(rotor_shape, info['phi0']), fixed_drive)
    for body, name, shape, _ in rows:
        if 'keeper' in name or 'collar' in name:
            mating = rotor_shape if body == 'rotor' else next(s for b,n,s,_ in rows if b==body and n=='leaf_shaft_crank')
            requirements.noninterference(body+'_'+name+'_groove_fit', mating, shape)
    return native.finish('CAD density and exact full inertia; source foil and new motor assumptions, no hardware qualification',
        {'mechanism-layout.json': dict(frames={k:v.tolist() for k,v in info['frames'].items()},
            links=[link.source() for link in info['links']], phi0=info['phi0'], phi1=info['phi1'],
            leaf_geometry=leaf_report, fixed_support_attachments=attachments, axial_stack=stack,
            assumptions=spec['assumptions'])}, native_requirements=requirements)


def physical_from_records(spec, masses):
    """Compile the same named pin frames; only the ring receives an actuator."""
    info = layout(spec)
    root = ET.Element('mujoco', model='IRIS09_derived_single_drive')
    ET.SubElement(root, 'compiler', angle='radian', autolimits='true')
    ET.SubElement(root, 'option', timestep=str(spec['timestep_s']), gravity='0 0 -9.81', integrator='implicitfast',
                  solver='Newton', iterations='100', tolerance='1e-12')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'body', name='fixed')
    rotor = ET.SubElement(world, 'body', name='rotor', quat=f'{math.cos(info["phi0"]/2)} 0 0 {math.sin(info["phi0"]/2)}')
    ET.SubElement(rotor, 'joint', name='ring_drive', type='hinge', axis='0 0 1', damping='.0002',
                  range=f'{info["ring_delta_rad"]-.02} .02', frictionloss='.00005')
    for index in range(info['count']):
        name = f'leaf_{index:02}'
        frame = info['frames'][name]
        angle = -index*math.tau/info['count']
        body = ET.SubElement(world, 'body', name=name, pos=' '.join(map(str, frame[:3,3]*.001)),
                             quat=f'{math.cos(angle/2)} 0 0 {math.sin(angle/2)}')
        ET.SubElement(body, 'joint', name=name+'_hinge', type='hinge', axis='0 0 -1',
                      range='-.02 .92', damping='.00001', frictionloss='.000001')
    actuator = ET.SubElement(root, 'actuator')
    ET.SubElement(actuator, 'motor', name='annular_motor', joint='ring_drive')
    physical = PhysicalModel(root, provenance=dict(design='New IRIS-derived linkage, not source dynamics',
        specification_sha256=digest(spec), assumptions=spec['assumptions'],
        contacts=dict(policy='Ideal pin bearings and loop closures; no leaf/contact forces hidden in the model',
                      scope='Native material clearance is evaluated separately; bearings are not wear/friction contact models')))
    for link in info['links']:
        link.attach(physical, damping_Nm_s_rad=.000001, solref=(.001, 1.), solimp=(.999, .9999, .000001))
    physical.bind_inertias(masses, {name:name for name in info['frames'] if name != 'fixed'})
    cfg = spec['controller']
    control = dict(schema='cybrgeo-control/1', drives=[dict(joint='ring_drive', actuator='annular_motor',
        motor=deepcopy(spec['motor']), **cfg, soft_limits=[info['ring_delta_rad']-.025, .025])],
        inertial_feedforward=False, gravity_and_coriolis_feedforward=False,
        provenance=dict(scope='Assumed annular motor; generic source-derived electrical kernel, not inherited IRIS motor data'))
    parameters = dict(specification=deepcopy(spec), ring_delta_rad=info['ring_delta_rad'],
                      period_s=spec['period_s'], body_ids=list(info['frames']))
    return physical, control, parameters


def compile_model(spec=None, *, native_directory=None):
    spec = validate(spec or specification())
    if native_directory is None:
        raise ValueError('IRIS dynamics requires freshly constructed native mass records')
    masses = json.loads((Path(native_directory)/'mass_properties.json').read_text(encoding='utf-8'))
    return physical_from_records(spec, masses)


def reference(t, parameters):
    period, amplitude = parameters['period_s'], parameters['ring_delta_rad']
    frequency = math.tau/period
    return dict(q=[amplitude*.5*(1-math.cos(frequency*t))],
                qd=[amplitude*.5*frequency*math.sin(frequency*t)],
                qdd=[amplitude*.5*frequency*frequency*math.cos(frequency*t)], stage='single ring drive')


def requirements(result, parameters):
    spec, info = parameters['specification'], layout(parameters['specification'])
    closure_max, leaf_spread, peak_leaf_excursion, endpoint_error = 0., 0., 0., None
    first_leaf_angle = None
    for frame in result['frames']:
        angles = []
        for index, link in enumerate(info['links']):
            body = frame['bodies'][f'leaf_{index:02}']
            rotation = np.asarray(body['rotation_row_major']).reshape(3,3)
            yaw = math.atan2(rotation[1,0], rotation[0,0])
            angle = math.atan2(math.sin(-yaw-index*math.tau/info['count']), math.cos(-yaw-index*math.tau/info['count']))
            angles.append(angle)
            rod = frame['bodies'][link.name]
            a = np.asarray(rod['position_m']) + np.asarray(rod['rotation_row_major']).reshape(3,3) @ np.array([link.length_mm*.001,0,0])
            b = np.asarray(body['position_m']) + rotation @ (np.asarray(link.output_anchor_mm)*.001)
            closure_max = max(closure_max, float(np.linalg.norm(a-b))*1000)
        if first_leaf_angle is None:
            first_leaf_angle = angles[0]
        leaf_spread = max(leaf_spread, max(angles)-min(angles))
        peak_leaf_excursion = max(peak_leaf_excursion, max(angles))
        # At half-cycle all physical leaves must reach the requested source span.
        if abs(frame['time_s']-spec['period_s']/2) <= result['output_interval_s']*.6:
            endpoint_error = max(abs(angle-.9) for angle in angles)
    constraints = result['metrics'].get('constraints', {})
    joints = result['metrics'].get('joints', {})
    observed_steps = result['metrics'].get('monitoring_interval', {}).get('integration_steps')
    full_closure, step_closure = True, 0.
    for link in info['links']:
        measured = constraints.get(link.name+'_closure', {})
        metric = measured.get('vector_norms', {}).get('anchor_delta', {})
        full_closure = full_closure and (measured.get('active_steps') == observed_steps == result['steps']
            and measured.get('missing_row_steps') == 0 and metric.get('samples') == result['steps'])
        if metric.get('peak_absolute'):
            step_closure = max(step_closure, abs(metric['peak_absolute']['value'])*1000)
        else:
            full_closure = False
    full_joint_coverage, joint_limits = True, True
    for index in range(info['count']):
        measured = joints.get(f'leaf_{index:02}_hinge', {})
        full_joint_coverage = full_joint_coverage and (measured.get('samples') == result['steps']+1
            and measured.get('direct_motor_actuator') is None)
        if 'minimum_position' not in measured or 'maximum_position' not in measured:
            joint_limits = False
        else:
            joint_limits = joint_limits and (measured['minimum_position']['value'] >= -1e-10
                                             and measured['maximum_position']['value'] <= .9+1e-10)
    checks = [dict(name='complete ring cycle', passed=result['duration_s'] >= spec['period_s']-1e-9),
              dict(name='loop closure at recorded states', passed=closure_max <= spec['closure_limit_mm'],
                   maximum_mm=closure_max, limit_mm=spec['closure_limit_mm']),
              dict(name='all passive leaves reach aperture endpoint', passed=endpoint_error is not None and endpoint_error <= spec['leaf_tracking_limit_rad'] and peak_leaf_excursion > .85,
                   endpoint_error_rad=endpoint_error, peak_leaf_excursion_rad=peak_leaf_excursion),
              dict(name='every-step loop closure', passed=full_closure and step_closure <= spec['closure_limit_mm'],
                   maximum_mm=step_closure, limit_mm=spec['closure_limit_mm'], full_coverage=full_closure,
                   covered_integration_steps=observed_steps),
              dict(name='every-step passive leaf range', passed=full_joint_coverage and joint_limits,
                   full_coverage=full_joint_coverage, constructed_range_rad=[0., .9], numerical_allowance_rad=1e-10),
              dict(name='passive leaf synchrony', passed=leaf_spread <= spec['leaf_tracking_limit_rad'], maximum_difference_rad=leaf_spread),
              dict(name='single ring tracking', passed=result['metrics']['max_tracking_error_rad'] <= spec['ring_tracking_limit_rad'],
                   maximum_error_rad=result['metrics']['max_tracking_error_rad']),
              dict(name='no applied body wrenches', passed=result['metrics']['max_external_body_force_N']==0),
              dict(name='no directly applied follower forces', passed=result['metrics']['max_applied_generalized_force']==0),
              dict(name='no unexpected controller fault', passed=not result['events'])]
    return dict(passed=all(row['passed'] for row in checks), checks=checks,
                scope='Actual integrated ideal-pin linkage task; all integration-step native solver closure and joint extrema plus recorded post-state closure. This is not a continuous-time clearance proof or hardware qualification.')
