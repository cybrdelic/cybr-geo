"""Bind P5 rack construction to the actual opposed MJCF slide and tendon."""
from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from mechanism_lab.rack_travel import RackPinionTravel


SOURCE_MODEL = Path(__file__).with_name('data') / 'P2-baseline.xml'
PINION_CENTER_Y_MM = 201.5
JAW_ORIGIN_Y_MM = 219.2
# The P5 bridge, motor socket, jaw passages and supports were authored for
# exactly this transmission. Changing these values needs a coupled native
# redesign rather than merely widening the generic pitch-span certificate.
P5_SUPPORTED_GEOMETRY = dict(module_mm=.3, pinion_teeth=20,
    rack_teeth=50, toothed_end_local_x_mm=2.)


def _numbers(value, count, label):
    try:
        numbers = tuple(float(token) for token in value.split())
    except (AttributeError, ValueError, OverflowError) as error:
        raise ValueError(f'{label} needs {count} finite components') from error
    if len(numbers) != count or any(not math.isfinite(number) for number in numbers):
        raise ValueError(f'{label} needs {count} finite components')
    return numbers


def rack_travel_from_source(specification, *, model_root=None):
    """Fail before CAD when an authored rack cannot cover both full joint ranges.

    ``model_root`` accepts the compiled physical model for the second check;
    construction parses the exact pinned baseline MJCF that compilation uses.
    These checks establish nominal kinematic consistency, not tooth engagement
    force or production backlash.
    """
    document = specification['gripper_rack_travel']
    if not isinstance(document, dict):
        raise ValueError('P5 rack geometry must be an authored document')
    for name, expected in P5_SUPPORTED_GEOMETRY.items():
        if document.get(name) != expected or isinstance(document.get(name), bool):
            raise ValueError(f'Unsupported P5 rack {name} edit requires bridge, jaw and support redesign')
    root = model_root if model_root is not None else ET.fromstring(SOURCE_MODEL.read_bytes())
    ranges = []
    for body_name, expected_axis in (('jaw_left', (1., 0., 0.)),
                                     ('jaw_right', (-1., 0., 0.))):
        body = root.find(f".//body[@name='tool']/body[@name='{body_name}']")
        if body is None:
            raise ValueError(f'Opposed rack omits tool child {body_name}')
        joint = body.find(f"joint[@name='{body_name}']")
        if joint is None or joint.get('type') != 'slide':
            raise ValueError(f'Opposed rack omits slide joint {body_name}')
        origin = _numbers(body.get('pos'), 3, f'{body_name} origin')
        if (any(not math.isclose(actual, expected, rel_tol=0., abs_tol=1e-12)
                for actual, expected in zip(origin, (0., JAW_ORIGIN_Y_MM/1000., 0.)))
                or _numbers(joint.get('axis'), 3, f'{body_name} axis') != expected_axis):
            raise ValueError('Opposed rack body frame or slide axis differs from constructed geometry')
        ranges.append(_numbers(joint.get('range'), 2, f'{body_name} slide range'))
    if ranges[0] != ranges[1]:
        raise ValueError('Opposed rack jaws must share a complete slide interval')
    equality = root.find(".//equality/joint[@name='opposed_rack_synchronization']")
    if (equality is None or equality.get('joint1') != 'jaw_left'
            or equality.get('joint2') != 'jaw_right'
            or _numbers(equality.get('polycoef'), 5, 'rack synchronization') != (0., 1., 0., 0., 0.)):
        raise ValueError('Opposed rack needs its one-to-one jaw equality')
    travel = RackPinionTravel.from_document(document, ranges[0])
    tendon = root.find(".//tendon/fixed[@name='rack_pinion_angle']")
    members = [] if tendon is None else tendon.findall('joint')
    expected_coefficient = 1. / (2. * travel.pitch_radius_m)
    if (len(members) != 2
            or {row.get('joint') for row in members} != {'jaw_left', 'jaw_right'}
            or any(not math.isclose(_numbers(row.get('coef'), 1, 'rack tendon coefficient')[0],
                                     expected_coefficient, rel_tol=1e-10, abs_tol=1e-7)
                   for row in members)):
        raise ValueError('Rack tendon pitch conversion differs from the constructed pinion radius')
    return travel


def arx_rack_report(travel):
    """Record the source frame and both derived native pitch-line placements."""
    report = travel.report()
    report['arx_p5_mating_frame'] = travel.opposed_pitch_line_y(
        PINION_CENTER_Y_MM, JAW_ORIGIN_Y_MM)
    report['arx_p5_geometry_pinned'] = True
    return report
