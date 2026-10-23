"""Example-specific four-bar geometry and task using the shared GEO services.

Circle intersection chooses the construction branch once. During integration,
the crank motor produces torque and the shared PinnedLink closes the passive
rocker loop. There is no follower trajectory or render-only motion solver.
"""
from copy import deepcopy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import cadquery as cq
import numpy as np

from mechanism_lab.fit_construction import PinBoreFit
from mechanism_lab.linkages import PinnedLink
from mechanism_lab.planar_linkage import PlanarFourBar
from mechanism_lab.native_construction import NativeConstruction, NativeInterfaceRequirements, read_specification
from mechanism_lab.qualification import QualificationPlan, QualifiedNativeConstruction
from mechanism_lab.physical_model import PhysicalModel, digest
from mechanism_lab.qualification import QualificationIntentPlan

HERE = Path(__file__).resolve().parent
DENSITIES = dict(aluminum=2700., bronze=8800., steel=7850., steel_blue=7850., steel_orange=7850.)


def specification():
    return validate(json.loads((HERE/'data'/'specification.json').read_text(encoding='utf-8')))


def qualification_intent(spec):
    """Bind unresolved fabrication requirements to this exact four-bar design.

    This diagnostic construction intent cannot issue a qualified release. The
    source motor limit and pin fit are demand/variation targets, not measured
    motor capacity or process capability.
    """
    document = json.loads((HERE/'data'/'qualification-intent.json').read_text(encoding='utf-8'))
    fit = PinBoreFit.from_document(spec['pin_diameter_mm'], spec['pin_bore_fit'])
    if not math.isclose(document['loads']['crank_motor']['source_demand_limit_Nm'],
                        spec['motor']['gear_output_torque_limit_Nm'], rel_tol=0, abs_tol=1e-12):
        raise ValueError('Four-bar qualification intent motor demand differs from the physical source')
    for name in ('input_pin_coupler_bore', 'output_pin_coupler_bore'):
        row = document['interfaces'][name]
        if any(not math.isclose(row[key], expected, rel_tol=0, abs_tol=1e-12)
               for key, expected in (
                   ('required_service_gap_mm', fit.required_service_radial_gap_mm),
                   ('first_variation_mm', fit.pin_declared_radial_variation_mm),
                   ('second_variation_mm', fit.bore_declared_radial_variation_mm))):
            raise ValueError('Four-bar qualification intent pin fit differs from construction source')
    if document['parts']['coupler__bored_coupler']['nominal_minimum_wall_mm'] != spec['minimum_wall_mm']:
        raise ValueError('Four-bar qualification intent coupler wall differs from construction source')
    return QualificationIntentPlan(document)


def validate(spec):
    spec = deepcopy(spec)
    if spec.get('schema') != 'cybrgeo-fourbar-lift/1':
        raise ValueError('Expected cybrgeo-fourbar-lift/1')
    fit = PinBoreFit.from_document(spec['pin_diameter_mm'], spec['pin_bore_fit'])
    if ('diametral_clearance_mm' in spec and not math.isclose(
            spec['diametral_clearance_mm'], fit.nominal_diametral_clearance_mm,
            rel_tol=0, abs_tol=1e-12)):
        raise ValueError('Derived pin bore clearance disagrees with its fit design source')
    spec['diametral_clearance_mm'] = fit.nominal_diametral_clearance_mm
    for name, value in spec.items():
        if name.endswith(('_mm', '_s', '_rad')):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(name+' must be finite and positive')
    a, b, c, d = (spec[key] for key in ('crank_length_mm', 'coupler_length_mm', 'rocker_length_mm', 'ground_length_mm'))
    lengths = sorted((a, b, c, d))
    if lengths[0] != a or lengths[0]+lengths[-1] >= lengths[1]+lengths[2]:
        raise ValueError('The authored branch requires a strict Grashof crank-rocker without a change-point')
    # Certify that the one physical closure branch exists through the entire
    # commanded revolution before constructing any solids or actuator model.
    linkage(spec).angular_rate_bounds(0., math.tau)
    # These supports/retainers are a coupled nominal stack, not generic vendor
    # fittings. A different interface must change its geometry first.
    fixed = dict(shaft_diameter_mm=6., pin_diameter_mm=5.,
        link_thickness_mm=4., coupler_height_mm=8.5)
    for key, value in fixed.items():
        if spec[key] != value:
            raise ValueError(key+' must match the authored bearing and keeper stack')
    if spec['link_width_mm'] < spec['pin_diameter_mm']+spec['diametral_clearance_mm']+2*spec['minimum_wall_mm']:
        raise ValueError('Link bores violate minimum nominal wall')
    if d-c <= a+spec['link_width_mm']:
        raise ValueError('Crank and rocker planar radial envelopes must be separated')
    if spec['load_position_mm'] <= c+spec['load_radius_mm']+spec['link_width_mm']/2:
        raise ValueError('Integral load lobe must clear the output pin and coupler end')
    if spec['load_thickness_mm'] > 5.:
        raise ValueError('Integral load lobe requires at least 0.5 mm axial backplate clearance')
    # Over a complete crank turn, the output extreme occurs when crank and
    # coupler align. The conservative disk around the input crank includes its
    # shaft, bearing support and the motor-mount heads. Reject a load lobe or
    # extended rocker that can enter it before constructing any solids.
    cosine = ((b-a)**2-d*d-c*c)/(2*d*c)
    if not -1 < cosine < 1:
        raise ValueError('Four-bar output sweep must avoid a change-point')
    p = spec['load_position_mm']
    radial_minimum = math.sqrt(d*d+p*p+2*d*p*cosine)
    if radial_minimum <= spec['load_radius_mm']+a+spec['link_width_mm']/2:
        raise ValueError('Integral load lobe swept envelope intersects the input mechanism envelope')
    closest = min(p,max(0.,-d*cosine))
    rocker_minimum = math.sqrt(d*d+closest*closest+2*d*closest*cosine)
    if rocker_minimum <= a+spec['link_width_mm']:
        raise ValueError('Extended rocker swept envelope intersects the input crank envelope')
    return spec


def frame(angle=0., position=(0., 0., 0.)):
    result = np.eye(4)
    result[:3, :3] = [[math.cos(angle), -math.sin(angle), 0.], [math.sin(angle), math.cos(angle), 0.], [0., 0., 1.]]
    result[:3, 3] = position
    return result


def linkage(spec):
    """One reusable planar closure source for CAD, physics setup and bounds."""
    return PlanarFourBar((0., 0.), (spec['ground_length_mm'], 0.),
        spec['crank_length_mm'], spec['coupler_length_mm'],
        spec['rocker_length_mm'], branch=1)


def rocker_angle(theta, spec):
    """Upper circle-intersection branch for initialization and verification."""
    return linkage(spec).solve(theta)['rocker_angle_rad']


def layout(spec):
    spec = validate(spec)
    theta = spec['initial_crank_angle_rad']
    phi = rocker_angle(theta, spec)
    crank, rocker = frame(theta), frame(phi, (spec['ground_length_mm'], 0., 0.))
    link = PinnedLink.from_frames('coupler', 'crank', 'rocker', crank, rocker,
        (spec['crank_length_mm'], 0., spec['coupler_height_mm']),
        (spec['rocker_length_mm'], 0., spec['coupler_height_mm']),
        width_mm=spec['link_width_mm'], thickness_mm=spec['link_thickness_mm'],
        pin_diameter_mm=spec['pin_diameter_mm'], diametral_clearance_mm=spec['diametral_clearance_mm'],
        minimum_wall_mm=spec['minimum_wall_mm'])
    if not math.isclose(link.length_mm, spec['coupler_length_mm'], abs_tol=1e-8):
        raise ValueError('Constructed pin frames differ from coupler length')
    frames = dict(fixed=np.eye(4), crank=crank, rocker=rocker,
                  coupler=crank@frame(link.input_relative_yaw_rad, link.input_anchor_mm))
    return dict(theta=theta, phi=phi, link=link, frames=frames)


def cylinder(radius, bottom, top, xy=(0., 0.)):
    return cq.Workplane('XY').circle(radius).extrude(top-bottom).val().translate((*xy, bottom))


def annulus(inner, outer, bottom, top, xy=(0., 0.)):
    return cq.Workplane('XY').circle(outer).circle(inner).extrude(top-bottom).val().translate((*xy, bottom))


def keeper(inner, outer, bottom, top, xy=(0., 0.)):
    shape = annulus(inner, outer, bottom, top)
    mouth = cq.Workplane('XY').box(2*outer, .7, top-bottom+1.).val().translate((-outer, 0., (top+bottom)/2))
    return shape.cut(mouth).translate((*xy, 0.))


def strap(length, width, bottom, top):
    return cq.Workplane('XY').center(length/2, 0).slot2D(length+width, width).extrude(top-bottom).val().translate((0., 0., bottom))


def moving_link(length, spec, *, loaded=False):
    tip = spec['load_position_mm'] if loaded else length
    shape = strap(tip, spec['link_width_mm'], .5, 4.5)
    # Single machined steel link, integral pivot shaft and coupler pin. The
    # grooves and retainers are actual solids, not textures or unconstrained caps.
    shape = shape.fuse(cylinder(3., -16.5, 4.5), cylinder(2.5, 4., 12.5, (length, 0.)),
                       cylinder(4., 4., 6.2, (length, 0.)))
    shape = shape.cut(annulus(2.7, 3.1, -15.8, -14.6),
                      annulus(2.2, 2.6, 10.7, 11.9, (length, 0.)))
    if loaded:
        shape = shape.fuse(cylinder(5., -2.5, 1., (tip, 0.)),
                           cylinder(spec['load_radius_mm'], -2.5-spec['load_thickness_mm'], -2.5, (tip, 0.)))
    return shape.clean()


def native_rows(spec):
    info = layout(spec)
    distance = spec['ground_length_mm']
    # The integral vertical backplate carries both bearing supports. Four
    # through-holes remain open for mounting to an external fixture.
    plate = cq.Workplane('XY').box(distance+48., 58., 6.).val().translate((distance/2, 0., -11.))
    for x in (-14., distance+14.):
        for y in (-20., 20.):
            plate = plate.cut(cylinder(2.25, -15., -7., (x, y)))
    for x in (0., distance):
        plate = plate.cut(cylinder(3.2, -15., -7., (x, 0.)))
        plate = plate.fuse(annulus(6.05, 10., -8., 0., (x, 0.)))
    motor_bolts = [(13*math.cos(a), 13*math.sin(a)) for a in
                   (math.pi/4, 3*math.pi/4, 5*math.pi/4, 7*math.pi/4)]
    for xy in motor_bolts:
        plate = plate.cut(cylinder(1.65, -15., -7., xy))
    rows = [('fixed', 'backplate_and_bearing_supports', plate.clean(), 'aluminum')]
    for label, x in (('crank', 0.), ('rocker', distance)):
        rows.append(('fixed', label+'_bushing', annulus(3.05, 6., -8., 0., (x, 0.)), 'bronze'))
    # The declared direct-drive torque port has a physical reaction envelope
    # fastened to the backplate. It is not an invented electromagnetic model.
    motor = annulus(6.5, 17., -36., -14.)
    for index, xy in enumerate(motor_bolts):
        motor = motor.cut(cylinder(1.65, -37., -13., xy))
        screw = cylinder(1.5, -22., -7.9, xy).fuse(cylinder(2.7, -8., -5., xy))
        socket = cq.Workplane('XY').polygon(6, 2.5).extrude(2.).val().translate((*xy, -6.5))
        rows.append(('fixed', f'motor_mount_screw_{index}', screw.cut(socket), 'steel'))
    rows.append(('fixed', 'declared_motor_reaction_housing', motor, 'aluminum'))
    for name, length, material, loaded in (('crank', spec['crank_length_mm'], 'steel_blue', False),
                                         ('rocker', spec['rocker_length_mm'], 'steel_orange', True)):
        rows.append((name, name+'_shaft_link_and_pin', moving_link(length, spec, loaded=loaded), material))
        rows.append((name, 'rear_shaft_keeper', keeper(2.8, 5., -15.7, -14.7), 'steel'))
        rows.append((name, 'coupler_pin_keeper', keeper(2.3, 4., 10.8, 11.8, (length, 0.)), 'steel'))
    rows.append(('coupler', 'bored_coupler', info['link'].native_shape(), 'steel'))
    return rows, info


def build(spec, output):
    spec, identity = read_specification(spec)
    spec = validate(spec)
    intent = qualification_intent(spec)
    # A sourced complete plan opts this same mechanism into strict construction.
    # Incomplete lots, process capability, interfaces or loads fail before any
    # native STEP/template is exported. Without it, the intent-backed build
    # remains explicitly diagnostic and never emits a qualified receipt.
    plan_path = HERE / 'data' / 'qualification-plan.json'
    strict_plan = QualificationPlan(json.loads(plan_path.read_text(encoding='utf-8'))) if plan_path.is_file() else None
    if strict_plan is not None:
        intent.require_resolved_by(strict_plan)
    rows, info = native_rows(spec)
    provenance = dict(identity,
        scope='New gravity-loaded four-bar using shared native CAD, full inertia and pinned-link construction',
        assumptions=spec['assumptions'])
    if strict_plan is None:
        native = NativeConstruction(output, DENSITIES, provenance=provenance,
                                    qualification_intent=intent)
    else:
        native = QualifiedNativeConstruction(output, DENSITIES, provenance=provenance,
            qualification_plan=strict_plan,
            initial_body_poses_mm=info['frames'],
            physical_body_mapping={body: body for body in info['frames']})
    for body, name, shape, material in rows:
        # The same authored parts and layout feed either construction stage.
        # Diagnostic templates remain useful but cannot be a qualified part.
        intent.check_placement(body, name, material, shape, DENSITIES)
        if strict_plan is None:
            native.addpart(body+'__'+name, shape, material,
                'New nominal machined geometry; no measured process qualification')
        native.place(body, name, shape, material)
    fit = PinBoreFit.from_document(spec['pin_diameter_mm'], spec['pin_bore_fit'])
    requirements = None
    if strict_plan is None:
        # These historical checks use local fixtures, not exact placed STEP
        # pairs. QualifiedNativeConstruction creates its own requirements from
        # the imported, startup-posed part pairs named in the strict plan.
        pin_bore_variations = dict(first_mm=fit.bore_declared_radial_variation_mm,
                                   second_mm=fit.pin_declared_radial_variation_mm)
        requirements = NativeInterfaceRequirements(dict(scope='Nominal bored shaft/pin/keeper interfaces and separated axial layers'))
        for body, length in (('crank', spec['crank_length_mm']), ('rocker', spec['rocker_length_mm'])):
            shape = next(shape for b,n,shape,m in rows if b==body and n.endswith('_shaft_link_and_pin'))
            requirements.noninterference(body+'_shaft_bushing', shape, annulus(3.05, 6., -8., 0.))
            requirements.noninterference(body+'_rear_keeper_groove', shape, keeper(2.8, 5., -15.7, -14.7))
            requirements.noninterference(body+'_pin_keeper_groove', shape, keeper(2.3, 4., 10.8, 11.8, (length, 0.)))
        coupler = info['link'].native_shape()
        for label, x in (('input', 0.), ('output', info['link'].length_mm)):
            requirements.clearance(label+'_pin_bore', coupler, cylinder(2.5, -2., 2., (x, 0.)),
                                   minimum_distance_mm=fit.required_service_radial_gap_mm,
                                   kernel_allowance_mm=1e-8,
                                   declared_design_variations_mm=pin_bore_variations)
            for position, lo, hi in (('lower_shoulder', -4.5, -2.3), ('upper_keeper', 2.3, 3.3)):
                requirements.noninterference(label+'_'+position, coupler, cylinder(4., lo, hi, (x, 0.)))
    return native.finish('Generated nominal CAD density and full inertia, no mass overrides or hardware qualification',
        {'mechanism-layout.json': dict(frames={name:value.tolist() for name,value in info['frames'].items()},
            links=[info['link'].source()], gravity_m_s2=[0., -9.81, 0.],
            pin_bore_fit_design=fit.report(),
            axial_stack_mm=dict(fixed_plate=[-14., -8.], bushings=[-8., 0.],
                rotating_links=[.5,4.5], coupler=[6.5,10.5], keeper=[10.8,11.8]),
            assumptions=spec['assumptions'])}, native_requirements=requirements)


def physical_from_records(spec, masses):
    info = layout(spec)
    root = ET.Element('mujoco', model='fourbar_lift')
    ET.SubElement(root, 'compiler', angle='radian', autolimits='true')
    ET.SubElement(root, 'option', timestep=str(spec['timestep_s']), gravity='0 -9.81 0',
                  integrator='implicitfast', solver='Newton', iterations='100', tolerance='1e-12')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'body', name='fixed')
    for name, angle in (('crank', info['theta']), ('rocker', info['phi'])):
        position = info['frames'][name][:3,3]*.001
        body = ET.SubElement(world, 'body', name=name, pos=' '.join(map(str, position)),
            quat=f'{math.cos(angle/2)} 0 0 {math.sin(angle/2)}')
        ET.SubElement(body, 'joint', name=name+'_hinge', type='hinge', axis='0 0 1',
            limited='false', damping='.0001', frictionloss='0')
    ET.SubElement(ET.SubElement(root, 'actuator'), 'motor', name='crank_motor', joint='crank_hinge')
    physical = PhysicalModel(root, provenance=dict(specification_sha256=digest(spec),
        design='New four-bar lift client', assumptions=spec['assumptions'],
        force_path='Electrical crank torque -> crank pin -> passive coupler constraint -> rocker pin -> actual steel rocker/load CAD inertia -> gravity; fixed bearing reactions',
        contacts=dict(policy='Ideal revolute bearings; no hidden local collision forces or arbitrary follower actuation')))
    info['link'].attach(physical, damping_Nm_s_rad=.00001, solref=(.001, 1.), solimp=(.999, .9999, .000001))
    physical.bind_inertias(masses, {name:name for name in info['frames'] if name!='fixed'})
    control = dict(schema='cybrgeo-control/1', drives=[dict(joint='crank_hinge', actuator='crank_motor',
        motor=deepcopy(spec['motor']), **spec['controller'], soft_limits=[-.5, math.tau+.5])],
        inertial_feedforward=False, gravity_and_coriolis_feedforward=False,
        provenance=dict(scope='Declared direct-drive motor values; shared electrical equations, no selected supplier motor'))
    return physical, control, dict(specification=deepcopy(spec), body_ids=list(info['frames']), period_s=spec['period_s'])


def compile_model(spec=None, *, native_directory=None):
    spec = validate(spec or specification())
    if native_directory is None:
        raise ValueError('Four-bar simulation requires newly constructed native mass records')
    masses = json.loads((Path(native_directory)/'mass_properties.json').read_text(encoding='utf-8'))
    return physical_from_records(spec, masses)


def reference(t, parameters):
    """One smooth full-turn target for the motor only."""
    period = parameters['period_s']
    u = min(1., max(0., t/period))
    position = 10*u**3-15*u**4+6*u**5
    speed = (30*u*u-60*u**3+30*u**4)/period
    acceleration = (60*u-180*u*u+120*u**3)/(period*period)
    return dict(q=[math.tau*position], qd=[math.tau*speed], qdd=[math.tau*acceleration], stage='one crank revolution lifting the integral steel load')


def requirements(result, parameters):
    spec = parameters['specification']
    joints = result['metrics']['joints']
    rocker = joints.get('rocker_hinge', {})
    closure = result['metrics']['constraints'].get('coupler_closure', {})
    metric = closure.get('vector_norms', {}).get('anchor_delta', {})
    complete = (closure.get('active_steps') == result['steps'] and closure.get('missing_row_steps') == 0
                and metric.get('samples') == result['steps'])
    peak = metric.get('peak_absolute', {}).get('value')
    closure_mm = None if peak is None else abs(peak)*1000
    measured = rocker.get('samples') == result['steps']+1 and rocker.get('direct_motor_actuator') is None
    excursion = (rocker.get('maximum_position', {}).get('value', 0.)-
                 rocker.get('minimum_position', {}).get('value', 0.))
    motor = result['metrics']['actuators']['crank_motor']
    final = result['frames'][-1]['qpos'][0]
    checks = [dict(name='complete crank revolution', passed=abs(final-math.tau)<spec['tracking_limit_rad'], final_rad=final),
        dict(name='complete declared task duration', passed=result['duration_s']>=spec['period_s']-1e-9),
        dict(name='passive loaded rocker excursion', passed=measured and excursion>spec['minimum_rocker_excursion_rad'],
             excursion_rad=excursion, minimum_rad=spec['minimum_rocker_excursion_rad'], every_step_coverage=measured),
        dict(name='every-step material pin closure', passed=complete and closure_mm is not None and closure_mm<spec['closure_limit_mm'],
             maximum_mm=closure_mm, limit_mm=spec['closure_limit_mm'], every_step_coverage=complete),
        dict(name='motor tracking', passed=result['metrics']['max_tracking_error_rad']<spec['tracking_limit_rad'],
             maximum_rad=result['metrics']['max_tracking_error_rad'], limit_rad=spec['tracking_limit_rad']),
        dict(name='declared current limit', passed=motor['peak_current_A']<=spec['motor']['peak_current_A']+1e-12,
             peak_A=motor['peak_current_A'], limit_A=spec['motor']['peak_current_A']),
        dict(name='no applied body wrenches', passed=result['metrics']['max_external_body_force_N']==0),
        dict(name='no directly applied follower forces', passed=result['metrics']['max_applied_generalized_force']==0),
        dict(name='no unexpected controller faults', passed=not result['events'])]
    return dict(passed=all(row['passed'] for row in checks), checks=checks,
        scope='Integrated DC-driven crank and passive gravity-loaded linkage; nominal CAD and ideal constraints, not process capability or hardware load qualification')
