"""Construct mechanical geometry and its motion from one dimensional plan.

This module deliberately needs only the Python standard library.  Its output is
the same millimetre/Z-up primitive contract consumed by native CAD and Studio.
The motor and linear actuator bodies are declared purchased-component envelopes;
the assemblies are nominal kinematic designs, not qualified hardware.
"""
from __future__ import annotations

import hashlib
import json
import math


ARCHITECTURES = ('compact_arm', 'belt_arm', 'inspection_head')
DEFAULTS = dict(scale=1.0, radial_clearance_mm=0.2, joint_limit_deg=12.0,
                jaw_travel_mm=10.0)
BOUNDS = dict(scale=(.5, 2.), radial_clearance_mm=(.12, .4),
              joint_limit_deg=(1., 18.), jaw_travel_mm=(2., 14.))
COLORS = dict(frame=[.07, .12, .16], shaft=[.55, .59, .64],
              bushing=[.62, .36, .12], motor=[.035, .045, .055],
              belt=[.022, .025, .028], pad=[.14, .18, .2])


def _add(a, b):
    return [a[i] + b[i] for i in range(3)]


def _mul(a, k):
    return [v * k for v in a]


def _rotation(direction):
    """XYZ Euler degrees mapping local Z onto direction (Rz Ry Rx)."""
    x, y, z = direction
    length = math.sqrt(x*x + y*y + z*z)
    if length <= 1e-9:
        raise ValueError('A construction segment cannot have zero length')
    return [0., math.degrees(math.acos(max(-1., min(1., z/length)))),
            math.degrees(math.atan2(y, x))]


def _canonical(value):
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return value
    if isinstance(value, (int, float)):
        return format(0. if abs(value) < .5e-9 else value, '.9f')
    if isinstance(value, dict):
        return {key: _canonical(value[key]) for key in sorted(value)}
    return [_canonical(item) for item in value]


def recipe_digest(plan):
    """Portable identity, not a geometry validation or certification signature."""
    raw = json.dumps(_canonical({key: plan[key] for key in ('parts', 'joints')}),
                     ensure_ascii=False, separators=(',', ':'))
    return hashlib.sha256(raw.encode()).hexdigest()


class _Builder:
    def __init__(self, architecture, parameters):
        self.parameters = parameters
        self.scale = parameters['scale']
        self.clearance = parameters['radial_clearance_mm'] / self.scale
        self.parts = []
        self.joints = []
        self.interfaces = []
        self.actuators = []
        self.paths = []
        self.roles = {}
        self.architecture = architecture

    def box(self, size, center):
        return dict(kind='box', size=_mul(size, self.scale),
                    position=_mul(center, self.scale), rotation=[0., 0., 0.])

    def cylinder(self, radius, start, end):
        direction = [end[i]-start[i] for i in range(3)]
        length = math.sqrt(sum(v*v for v in direction))
        return dict(kind='cylinder', size=_mul([2*radius, 2*radius, length], self.scale),
                    position=_mul([(start[i]+end[i])/2 for i in range(3)], self.scale),
                    rotation=_rotation(direction))

    def axial(self, origin, axis, radius, lo, hi):
        return self.cylinder(radius, _add(origin, _mul(axis, lo)),
                             _add(origin, _mul(axis, hi)))

    def part(self, name, body, role, material='frame', unions=(), cuts=()):
        part = dict(name=name, body=body, unions=list(unions), cuts=list(cuts),
                    fillet=0., color=list(COLORS[material]), metalness=.0 if material in ('belt', 'pad') else .7,
                    roughness=.5 if material in ('belt', 'pad') else .3)
        self.parts.append(part)
        self.roles[name] = role
        return part

    def joint(self, name, child, parent, kind='fixed', origin=(0, 0, 0),
              axis=(0, 0, 1), limits=(0, 0), drive=None):
        unit = self.scale if kind == 'prismatic' else 1.
        result = dict(name=name, child=child, parent=parent, type=kind,
                      origin=_mul(origin, self.scale), axis=list(axis),
                      min=limits[0]*unit, max=limits[1]*unit, value=0., drive=drive)
        self.joints.append(result)
        return result

    def fixed(self, child, parent):
        self.joint('fix_' + child, child, parent)

    def interface(self, name, a, b, kind, **measurements):
        self.interfaces.append(dict(name=name, parts=[a, b], kind=kind, **measurements))

    def ring(self, name, origin, axis, outer, inner, lo, hi, role, material='shaft'):
        return self.part(name, self.axial(origin, axis, outer, lo, hi), role, material,
                         cuts=[self.axial(origin, axis, inner, lo-1, hi+1)])


def _radials(axis):
    # Cardinal construction axes. v is deliberately world-up for horizontal joints.
    return ([1., 0., 0.], [0., 1., 0.]) if axis[2] else ([0., 0., 1.], [0., 1., 0.] if axis[0] else [1., 0., 0.])


def _carrier(b, index, origin, axis, previous=None):
    """A load-carrying link: prior keyed flange, bridge and next housing in one solid."""
    name = f'J{index}_carrier'
    outer = b.axial(origin, axis, 18, -18, 18)
    cuts = [b.axial(origin, axis, 14, -19, 19)]
    unions = []
    if previous is None:
        # A wide root stand keeps the motor below the bearing housing accessible.
        unions.append(b.box([100, 100, 8], [0, 0, 4]))
        for x in (-30, 30):
            for y in (-30, 30):
                unions.append(b.box([12, 12, origin[2]-8], [x, y, (origin[2]+8)/2]))
                unions.append(b.cylinder(6, [x, y, origin[2]],
                                         [x*.4, y*.4, origin[2]]))
        for x in (-40, 40):
            for y in (-40, 40):
                cuts.append(b.cylinder(3.2, [x, y, -1], [x, y, 9]))
        parent = None
    else:
        prev_origin, prev_axis, parent = previous
        u, _ = _radials(prev_axis)
        flange = b.axial(prev_origin, prev_axis, 22, 24, 30)
        unions.append(flange)
        cuts.append(b.axial(prev_origin, prev_axis, 6+b.clearance, 23, 31))
        # Shared key and keyway are both authored in the flange's local frame.
        key_center = _add(_add(prev_origin, _mul(prev_axis, 27)), _mul(u, 7.))
        size = [0., 0., 0.]
        for k in range(3):
            size[k] = 6.4 if prev_axis[k] else (3.4+2*b.clearance if u[k] else 3.+2*b.clearance)
        cuts.append(b.box(size, key_center))
        # Leave the flange axially before bending toward the next bearing.
        # A large diagonal beam at the flange plane would intrude behind its
        # back face into the thrust collar/bushing even with a valid joint graph.
        flange_lug = _add(_add(prev_origin, _mul(prev_axis, 27)), _mul(u, 15.5))
        start = _add(_add(prev_origin, _mul(prev_axis, 47)), _mul(u, 15.5))
        unions.append(b.cylinder(5, flange_lug, start))
        target = _add(origin, [0, 0, 17] if not axis[2] else [17, 0, 0])
        path = [start]
        if axis[0]:
            # Approach X-axis cartridges outside their rear motor envelope.
            path.append(_add(origin, [-25, 35, 30]))
        path.append(target)
        for a, z in zip(path, path[1:]):
            unions.append(b.cylinder(8, a, z))
        for point in path[1:-1]:
            unions.append(dict(kind='sphere', size=[16*b.scale]*3,
                               position=_mul(point, b.scale), rotation=[0., 0., 0.]))
        b.interface(f'J{index-1}_keyed_flange', parent, name, 'keyed_shaft_flange',
                    radial_clearance_mm=b.parameters['radial_clearance_mm'],
                    axial_seat_mm=24*b.scale, keyway_clearance_mm=2*b.parameters['radial_clearance_mm'])
    part = b.part(name, outer, 'Integral housing and structural link; bore and flange datums shared',
                  unions=unions, cuts=cuts)
    b.fixed(name, parent)
    b.paths.append(dict(child=name, support=parent, path='integral bridge and keyed flange' if parent else 'root stand and mounting holes'))
    return part


def _cartridge(b, index, carrier, origin, axis, belt=False):
    label = f'J{index}'
    u, v = _radials(axis)
    c = b.clearance
    # Two axially separated flanged plain bearings. Housing bores, journal sizes,
    # washers and flange positions are all derived here, not planned independently.
    for side in (-1, 1):
        lo, hi = (-18, -10) if side < 0 else (10, 18)
        flo, fhi = (-20, -18) if side < 0 else (18, 20)
        name = label + ('_rear_bushing' if side < 0 else '_front_bushing')
        part = b.ring(name, origin, axis, 13.95, 6+c, lo, hi,
                      'Flanged journal bushing with shared shaft running clearance', 'bushing')
        part['unions'].append(b.axial(origin, axis, 16, flo, fhi))
        part['cuts'][0] = b.axial(origin, axis, 6+c, min(lo, flo)-1, max(hi, fhi)+1)
        b.fixed(name, carrier['name'])
        b.interface(name+'_journal', label+'_shaft', name, 'journal',
                    shaft_radius_mm=6*b.scale, bore_radius_mm=(6+c)*b.scale,
                    radial_clearance_mm=c*b.scale, bearing_span_mm=26*b.scale)
        b.interface(name+'_seat', name, carrier['name'], 'cylindrical_seat',
                    radial_clearance_mm=.05*b.scale)
    shaft = b.part(label+'_shaft', b.axial(origin, axis, 6, -38, 30),
                   'Front-inserted output shaft with integral front thrust shoulder and flange key', 'shaft',
                   unions=[b.axial(origin, axis, 10, 20.2, 24)])
    key_center = _add(_add(origin, _mul(axis, 27)), _mul(u, 7.))
    key_size = [5.6 if axis[k] else (3. if u[k] else 3.) for k in range(3)]
    shaft['unions'].append(b.box(key_size, key_center))
    limit = b.parameters['joint_limit_deg']
    output = b.joint(label+'_output', shaft['name'], carrier['name'], 'revolute', origin, axis,
                     (-limit, limit))
    _split_retainer(b, label, origin, axis, u, v, shaft['name'])
    _output_cap(b, label, origin, axis, shaft)
    b.interface(label+'_thrust', shaft['name'], label+'_front_bushing', 'axial_retention',
                axial_clearance_mm=.2*b.scale,
                note='Integral front shoulder and removable split rear collar constrain nominal axial travel; clamp force, friction and wear unqualified')
    b.paths.append(dict(child=shaft['name'], support=carrier['name'], path='two separated bushings and opposed thrust collars'))
    if belt:
        _belt_drive(b, label, carrier, origin, axis, u, v, output, shaft)
    else:
        motor = b.ring(label+'_motor_case', origin, axis, 18, 6+c, -48, -26,
                       'Purchased coaxial motor envelope; internal electromagnetic components unspecified', 'motor')
        b.fixed(motor['name'], carrier['name'])
        for k, (a, z) in enumerate(((17, 0), (-17, 0), (0, 17), (0, -17))):
            p = _add(origin, _add(_mul(u, a), _mul(v, z)))
            post = b.part(label+f'_motor_standoff_{k}', b.axial(p, axis, .9, -26, -18),
                          'Motor mounting standoff connecting envelope to housing', 'shaft')
            b.fixed(post['name'], carrier['name'])
        b.actuators.append(dict(joint=output['name'], type='coaxial_rotary_motor_envelope',
                                case=motor['name'], output=shaft['name'],
                                command_units='degrees', sizing='unspecified', dynamics='not simulated'))
    return shaft['name']


def _split_retainer(b, label, origin, axis, u, v, shaft):
    """Removable collar permits front shaft insertion through both bushings."""
    center = _add(origin, _mul(axis, -21.6))
    for sign, side in ((1, 'a'), (-1, 'b')):
        name = label+'_retainer_'+side
        ring = b.ring(name, origin, axis, 10, 6+.02, -23, -20.2,
                      'Removable split clamp collar; install after shaft insertion', 'shaft')
        # Remove the opposite half including a .2mm clamp split.
        cut_center = _add(center, _mul(u, -sign*5.95))
        sizes = [12.1 if u[k] else (5. if axis[k] else 24.) for k in range(3)]
        ring['cuts'].append(b.box(sizes, cut_center))
        for offset in (-8., 8.):
            p = _add(center, _mul(v, offset))
            ring['cuts'].append(b.cylinder(.85, _add(p, _mul(u, -11)), _add(p, _mul(u, 11))))
            ring['cuts'].append(b.cylinder(1.3, _add(p, _mul(u, sign*4.8)), _add(p, _mul(u, sign*11))))
        b.fixed(name, shaft)
    for n, offset in enumerate((-8., 8.)):
        p = _add(center, _mul(v, offset))
        bolt = b.part(label+f'_retainer_bolt_{n}',
                      b.cylinder(.7, _add(p, _mul(u, -6.2)), _add(p, _mul(u, 4.8))),
                      'Split collar clamp fastener; ordinary threads simplified', 'shaft',
                      unions=[b.cylinder(1.2, _add(p, _mul(u, 4.8)), _add(p, _mul(u, 6.2)))])
        b.fixed(bolt['name'], shaft)
        nut = b.part(label+f'_retainer_nut_{n}',
                     b.cylinder(1.2, _add(p, _mul(u, -6.2)), _add(p, _mul(u, -4.8))),
                     'Split collar clamp nut envelope; ordinary threads simplified', 'shaft',
                     cuts=[b.cylinder(.72, _add(p, _mul(u, -6.4)), _add(p, _mul(u, -4.6)))])
        b.fixed(nut['name'], shaft)
    b.interface(label+'_rear_retention', label+'_retainer_a', label+'_rear_bushing',
                'removable_axial_retainer', axial_clearance_mm=.2*b.scale,
                clamp_bore_radial_clearance_mm=.02*b.scale, split_gap_mm=.2*b.scale,
                assembly_sequence='Insert shaft from output side; install both split collar halves and clamp fasteners radially')


def _output_cap(b, label, origin, axis, shaft):
    """Seat on the shaft tip while capturing its keyed output flange axially."""
    shaft['cuts'].append(b.axial(origin, axis, 2.2, 16, 31))
    cap = b.part(label+'_output_cap', b.axial(origin, axis, 10, 30.2, 33.2),
                 'Removable axial cap captures keyed output flange; inner boss seats on shaft end', 'shaft',
                 unions=[b.axial(origin, axis, 5.8, 30, 30.2)],
                 cuts=[b.axial(origin, axis, 2.2, 29, 34)])
    b.fixed(cap['name'], shaft['name'])
    bolt = b.part(label+'_output_cap_screw', b.axial(origin, axis, 2, 17, 33.2),
                  'Output retention screw in shared blind shaft bore; ordinary threads simplified', 'shaft',
                  unions=[b.axial(origin, axis, 4, 33.2, 36.2)])
    b.fixed(bolt['name'], shaft['name'])
    b.interface(label+'_output_capture', shaft['name'], cap['name'], 'axial_flange_capture',
                flange_interval_mm=[24*b.scale, 30*b.scale],
                retaining_face_mm=30.2*b.scale, axial_clearance_mm=.2*b.scale,
                screw_engagement_mm=13*b.scale,
                assembly_sequence='Slide keyed output flange against shoulder; install boss-seated cap and axial screw')


def _removable_key(b, label, origin, axis, u, shaft, pulley, shaft_radius, bottom, top, width):
    """An inserted rectangular key, so rear shaft keys cannot block bearing insertion."""
    center = _add(_add(origin, _mul(axis, -27)), _mul(u, (bottom+top)/2))
    sizes = [5.6 if axis[k] else (top-bottom if u[k] else width) for k in range(3)]
    key = b.part(label+'_key', b.box(sizes, center),
                 'Removable transmission key installed after shaft insertion', 'shaft')
    b.fixed(key['name'], shaft['name'])
    slot_center = _add(_add(origin, _mul(axis, -27)), _mul(u, (bottom-.1+shaft_radius+.4)/2))
    slot_sizes = [6. if axis[k] else (shaft_radius+.4-bottom+.1 if u[k] else width+.2) for k in range(3)]
    shaft['cuts'].append(b.box(slot_sizes, slot_center))
    hub_center = _add(_add(origin, _mul(axis, -27)), _mul(u, (shaft_radius-.2+top+.2)/2))
    hub_sizes = [10. if axis[k] else (top+.2-shaft_radius+.2 if u[k] else width+.4) for k in range(3)]
    pulley['cuts'].append(b.box(hub_sizes, hub_center))
    b.interface(label+'_keyed_hub', shaft['name'], pulley['name'], 'removable_keyed_pulley',
                key=key['name'], key_slot_side_clearance_mm=.1*b.scale,
                assembly_sequence='Insert supported shaft first, then removable key, pulley and axial retainers')


def _belt_drive(b, label, carrier, origin, axis, u, v, output, output_shaft):
    """Equal smooth pulleys and a continuous closed capsule belt, nominal no-slip."""
    distance, pulley_r, belt_inner, belt_outer = 58., 12., 12.1, 14.3
    center = _add(origin, _mul(u, distance))
    # Integral outboard motor mount; all supports lie outside the belt sweep.
    carrier['unions'].append(b.cylinder(6, _add(origin, _mul(v, 17)), _add(center, _mul(v, 22))))
    carrier['unions'].append(b.cylinder(6, _add(center, _mul(v, -22)), _add(center, _mul(v, 22))))
    motor = b.ring(label+'_belt_motor_case', center, axis, 18, 4+b.clearance, -54, -36,
                   'Purchased offset motor envelope; internal electromagnetic components unspecified', 'motor')
    motor['unions'].append(b.axial(center, axis, 26, -38, -36))
    motor['cuts'][0] = b.axial(center, axis, 4+b.clearance, -55, -35)
    for sign in (-1, 1):
        post = _add(center, _mul(v, sign*22))
        motor['unions'].append(b.axial(post, axis, 2.5, -36, -6))
    b.fixed(motor['name'], carrier['name'])
    shaft = b.part(label+'_motor_shaft', b.axial(center, axis, 4, -45, -22),
                   'Offset motor output shaft with pulley locating shoulder', 'shaft',
                   unions=[b.axial(center, axis, 7, -33, -31.2)])
    source = b.joint(label+'_motor', shaft['name'], carrier['name'], 'revolute', center, axis,
                     (output['min'], output['max']))
    output['drive'] = dict(joint=source['name'], ratio=1., offset=0.)
    pulleys = []
    for name, point, parent, bore in ((label+'_driven_pulley', origin, output['child'], 6+b.clearance),
                                      (label+'_motor_pulley', center, shaft['name'], 4+b.clearance)):
        part = b.ring(name, point, axis, pulley_r, bore, -30, -24,
                      'Keyed and axially retained smooth pulley; traction and fastener sizing unqualified', 'shaft')
        part['unions'] += [b.axial(point, axis, 14.5, -31, -30), b.axial(point, axis, 14.5, -24, -23)]
        part['cuts'][0] = b.axial(point, axis, bore, -32, -22)
        b.fixed(name, parent)
        pulleys.append(part)
    _removable_key(b, label+'_driven', origin, axis, u, output_shaft, pulleys[0], 6, 5.0, 8.0, 2.4)
    _removable_key(b, label+'_driver', center, axis, u, shaft, pulleys[1], 4, 3.1, 5.4, 2.)
    # Output pulley is captured between its rear cup and the removable journal
    # retainer. The cup seats on the shaft end, with .2mm pulley end clearance.
    output_shaft['cuts'].append(b.axial(origin, axis, 2.2, -39, -25))
    cup = b.part(label+'_pulley_rear_cup', b.axial(origin, axis, 10, -40, -31.2),
                 'Removable cup captures driven pulley against rear split shaft collar', 'shaft',
                 cuts=[b.axial(origin, axis, 6.2, -38, -30), b.axial(origin, axis, 2.2, -41, -30)])
    b.fixed(cup['name'], output_shaft['name'])
    screw = b.part(label+'_pulley_rear_screw', b.axial(origin, axis, 2, -40, -26),
                   'Rear pulley retention screw; ordinary threads simplified', 'shaft',
                   unions=[b.axial(origin, axis, 4, -43, -40)])
    b.fixed(screw['name'], output_shaft['name'])
    # Driver pulley has its own shoulder and front cap; it does not rely on
    # friction at an unmodeled hub clamp to transmit motion.
    shaft['cuts'].append(b.axial(center, axis, 1.5, -32, -21))
    cap = b.part(label+'_motor_pulley_cap', b.axial(center, axis, 7, -22.8, -20),
                 'Removable driver pulley cap with shaft-end seating face', 'shaft',
                 cuts=[b.axial(center, axis, 4.2, -23, -22), b.axial(center, axis, 1.5, -24, -19)])
    b.fixed(cap['name'], shaft['name'])
    screw = b.part(label+'_motor_pulley_screw', b.axial(center, axis, 1.3, -31, -20),
                   'Driver pulley retention screw; ordinary threads simplified', 'shaft',
                   unions=[b.axial(center, axis, 2.6, -20, -18)])
    b.fixed(screw['name'], shaft['name'])
    b.interface(label+'_driven_pulley_capture', pulleys[0]['name'], cup['name'],
                'axial_pulley_capture', axial_clearance_mm=.2*b.scale)
    b.interface(label+'_driver_pulley_capture', pulleys[1]['name'], cap['name'],
                'axial_pulley_capture', axial_clearance_mm=.2*b.scale)
    # Capsule outer minus capsule inner. The connecting box axes are cardinal.
    middle = _add(_add(origin, _mul(u, distance/2)), _mul(axis, -27))
    sizes = [distance if u[k] else (3. if axis[k] else 2*belt_outer) for k in range(3)]
    inner_sizes = [distance if u[k] else (5. if axis[k] else 2*belt_inner) for k in range(3)]
    belt = b.part(label+'_belt', b.box(sizes, middle), 'Continuous smooth belt; fixed envelope under nominal no-slip motion', 'belt',
                  unions=[b.axial(p, axis, belt_outer, -28.5, -25.5) for p in (origin, center)],
                  cuts=[b.box(inner_sizes, middle)] + [b.axial(p, axis, belt_inner, -29.5, -24.5) for p in (origin, center)])
    b.fixed(belt['name'], carrier['name'])
    b.interface(label+'_belt_path', label+'_motor_pulley', label+'_driven_pulley', 'equal_pulley_belt',
                center_distance_mm=distance*b.scale, pulley_radius_mm=pulley_r*b.scale,
                belt_inner_radius_mm=belt_inner*b.scale, radial_clearance_mm=(belt_inner-pulley_r)*b.scale,
                nominal_path_length_mm=(2*distance+2*math.pi*(belt_inner+belt_outer)/2)*b.scale,
                no_slip_assumption=True, tension_qualified=False)
    b.actuators.append(dict(joint=source['name'], type='equal_pulley_belt_drive',
                            follower=output['name'], ratio=1., case=motor['name'],
                            output=output['child'], command_units='degrees', sizing='unspecified',
                            note='Equal smooth pulleys imply equal rotation only under assumed no-slip; belt material motion and tension are not solved'))


def _gripper(b, origin, axis, shaft):
    # The last cartridge is X-axis; all gripper interfaces use that same datum.
    x, y, z = origin
    c = b.clearance
    palm_center = [x+67, y, z]
    palm = b.part('tool_palm', b.box([12, 104, 56], palm_center),
                  'Integral keyed tool flange, palm, guide supports and linear actuator cases',
                  unions=[b.axial(origin, axis, 22, 24, 30),
                          b.cylinder(5, [x+27, y, z+15.5], [x+47, y, z+15.5]),
                          b.cylinder(8, [x+47, y, z+15.5], [x+61, y, z+15.5])],
                  cuts=[b.axial(origin, axis, 6+c, 23, 31),
                        b.box([6.4, 3+2*c, 3.4+2*c], [x+27, y, z+7])])
    b.fixed(palm['name'], shaft)
    b.interface('tool_keyed_flange', shaft, palm['name'], 'keyed_shaft_flange', radial_clearance_mm=c*b.scale)
    rod_x = x+85
    for sign in (-1, 1):
        for up in (-1, 1):
            point = [rod_x, y+sign*43, z+up*15]
            palm['unions'].append(b.box([25, 8, 12], [x+79.5, point[1], point[2]]))
            palm['cuts'].append(b.cylinder(3+c, [point[0], point[1]-5, point[2]], [point[0], point[1]+5, point[2]]))
    for up in (-1, 1):
        name = 'tool_guide_' + ('upper' if up > 0 else 'lower')
        rod = b.part(name, b.cylinder(3, [rod_x, y-47, z+up*15], [rod_x, y+47, z+up*15]),
                     'Continuous dual-rail gripper guide with paired support bores', 'shaft')
        b.fixed(rod['name'], palm['name'])
    for sign, side in ((-1, 'left'), (1, 'right')):
        jaw_y = y+sign*17
        case_center = [x+103, y+sign*47, z]
        lo, hi = sorted((y+sign*36, y+sign*58))
        # Bodies are integrated in the fixed palm; actuator rods are integral in
        # each moving jaw. The through-rod envelope remains engaged at both stops.
        palm['unions'].append(b.cylinder(6, [case_center[0], lo, z], [case_center[0], hi, z]))
        palm['unions'].append(b.box([42, 8, 10], [x+82, y+sign*47, z]))
        palm['cuts'].append(b.cylinder(3+c, [case_center[0], lo-2, z], [case_center[0], hi+2, z]))
        rod_lo, rod_hi = sorted((y+sign*50, jaw_y))
        jaw = b.part('jaw_'+side, b.box([18, 12, 48], [rod_x, jaw_y, z]),
                     'Integral guided carriage, gripping finger and actuator through rod',
                     unions=[b.box([32, 8, 22], [x+105, jaw_y+(-sign*2), z]),
                             b.cylinder(3, [case_center[0], rod_lo, z], [case_center[0], rod_hi, z])],
                     cuts=[b.cylinder(3+c, [rod_x, jaw_y-7, z+up*15], [rod_x, jaw_y+7, z+up*15]) for up in (-1, 1)])
        travel = b.parameters['jaw_travel_mm']/b.scale
        joint = b.joint('jaw_'+side+'_travel', jaw['name'], palm['name'], 'prismatic',
                        [rod_x, jaw_y, z], [0, sign, 0], (0, travel))
        pad = b.part('jaw_'+side+'_pad', b.box([18, 1.2, 18], [x+112, y+sign*10.4, z]),
                     'Replaceable finger contact pad', 'pad')
        b.fixed(pad['name'], jaw['name'])
        b.interface('jaw_'+side+'_guidance', jaw['name'], 'tool_guide_upper', 'dual_linear_guide',
                    shaft_radius_mm=3*b.scale, bore_radius_mm=(3+c)*b.scale,
                    radial_clearance_mm=c*b.scale, rail_length_mm=94*b.scale,
                    carriage_length_mm=12*b.scale, travel_mm=travel*b.scale,
                    minimum_guide_engagement_mm=12*b.scale)
        b.interface('jaw_'+side+'_actuator', palm['name'], jaw['name'], 'through_rod_linear_actuator_envelope',
                    radial_clearance_mm=c*b.scale, minimum_rod_engagement_mm=14*b.scale)
        b.actuators.append(dict(joint=joint['name'], type='through_rod_linear_actuator_envelope',
                                case=palm['name'], output=jaw['name'], command_units='mm',
                                sizing='unspecified', synchronization='independent commands; no mechanical coupling claimed'))
        b.paths.append(dict(child=jaw['name'], support=palm['name'], path='two guide rods and actuator through rod'))


def compile_plan(architecture='compact_arm', parameters=None):
    """Return a complete constructive recipe; reject impossible input before CAD.

    The bounded parameters describe a family, not arbitrary disconnected parts.
    Each independent joint command starts at the constructed reference pose.
    """
    if architecture not in ARCHITECTURES:
        raise ValueError(f'Unknown architecture {architecture!r}; choose {ARCHITECTURES}')
    if parameters is not None and not isinstance(parameters, dict):
        raise ValueError('Construction parameters must be an object')
    unknown = set(parameters or {}) - set(DEFAULTS)
    if unknown:
        raise ValueError(f'Unknown construction parameters: {sorted(unknown)}')
    values = {**DEFAULTS, **(parameters or {})}
    if 'jaw_travel_mm' not in (parameters or {}) and isinstance(values['scale'], (int, float)):
        values['jaw_travel_mm'] = min(14., 10*values['scale'])
    for key, value in values.items():
        lo, hi = BOUNDS[key]
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or not lo <= value <= hi:
            raise ValueError(f'{key} must be finite and within [{lo}, {hi}]')
        values[key] = float(value)
    # Preserve full guide engagement: jaw outer face must remain before support.
    if values['jaw_travel_mm'] > 14*values['scale']:
        raise ValueError('jaw_travel_mm exceeds the scaled guide/support clearance')
    b = _Builder(architecture, values)
    if architecture == 'inspection_head':
        locations = [([0, 0, 80], [0, 0, 1]), ([90, 0, 155], [0, 1, 0]),
                     ([195, 0, 155], [1, 0, 0])]
    else:
        locations = [([0, 0, 80], [0, 0, 1]), ([90, 0, 155], [0, 1, 0]),
                     ([235, 0, 155], [0, 1, 0]), ([375, 0, 155], [1, 0, 0]),
                     ([455, 0, 155], [0, 1, 0]), ([540, 0, 155], [1, 0, 0])]
    previous = None
    for index, (origin, axis) in enumerate(locations, 1):
        carrier = _carrier(b, index, origin, axis, previous)
        shaft = _cartridge(b, index, carrier, origin, axis,
                           belt=architecture == 'belt_arm' and index in (2, 3))
        previous = (origin, axis, shaft)
    _gripper(b, *previous)
    plan = dict(name='CYBR ' + architecture.replace('_', ' ').title(), units='mm',
                summary='Constructed supported serial mechanism with derived mating interfaces and guided jaws.',
                assumptions=[
                    'Original parametric design; not a reconstruction or validation of IRIS-09 or either ARX-6 source.',
                    'Nominal dimensions and prescribed joint motion. Clearance values are design inputs, not tolerance qualification.',
                    'Motor and through-rod linear actuator internals are purchased-component envelopes with unspecified power, force and controls.',
                    'Smooth belt no-slip motion is an assumption; tension, traction, dynamics, wear and hardware loads are unqualified.'
                    if architecture == 'belt_arm' else 'No dynamics, collision avoidance controller, force, stress, wear or hardware qualification is implied.',
                ], parts=b.parts, joints=b.joints)
    probes = []
    for index, (_, axis) in enumerate(locations, 1):
        moving = f'J{index+1}_carrier' if index < len(locations) else 'tool_palm'
        for side, blocker, direction in (('forward', f'J{index}_output_cap', axis),
                                          ('backward', f'J{index}_shaft', _mul(axis, -1))):
            probes.append(dict(name=f'J{index}_flange_{side}_capture', moving_part=moving,
                               blocker=blocker, direction=direction, distance_mm=b.scale))
        if architecture == 'belt_arm' and index in (2, 3):
            for end, moving_part, blocker, direction in (
                ('driven_front', f'J{index}_driven_pulley', f'J{index}_retainer_a', axis),
                ('driven_rear', f'J{index}_driven_pulley', f'J{index}_pulley_rear_cup', _mul(axis, -1)),
                ('driver_front', f'J{index}_motor_pulley', f'J{index}_motor_pulley_cap', axis),
                ('driver_rear', f'J{index}_motor_pulley', f'J{index}_motor_shaft', _mul(axis, -1)),
            ):
                probes.append(dict(name=f'J{index}_{end}_capture', moving_part=moving_part,
                                   blocker=blocker, direction=direction, distance_mm=b.scale))
    plan['construction'] = dict(schema='cybrgeo-construction/1', architecture=architecture,
                                parameters=values, interfaces=b.interfaces, actuators=b.actuators,
                                load_paths=b.paths, part_roles=b.roles, capture_probes=probes,
                                qualification=dict(kinematics='prescribed graph from construction',
                                                   dynamics='not simulated', hardware='not qualified',
                                                   interference='requires executed sampled CAD audit'),
                                recipe_sha256=recipe_digest(plan))
    return plan
