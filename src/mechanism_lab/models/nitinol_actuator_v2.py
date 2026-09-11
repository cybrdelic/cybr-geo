"""Higher-fidelity static Nitinol actuator concept for truth-gated rendering.

This is deliberately an ORIGINAL CONCEPT, not recovered manufacturer hardware.
It replaces several low-detail visual proxies from the first actuator study with
continuous source geometry: a true helical spring path, continuous 0.20 mm NiTi
wires, explicit crimp envelopes, routed copper jumpers, softened CAD edges and
visible fastener geometry. No hidden factory internals are claimed.

The v2 recipe is currently a static engineering-visualization assembly. It does
not fake a thermal contraction animation; coupled SMA motion belongs in a later
solver rather than a prescribed visual deformation.
"""
from __future__ import annotations

import math
import numpy as np
import cadquery as cq

from ..core import Assembly, Material, View, cad_part, mesh_part
from ..geometry import tube_mesh


MATERIALS = [
    Material(
        'Black anodized aluminium', (0.018, 0.022, 0.030), .72, .28,
        ior=1.48, coat=.10, coat_rough=.20, anisotropy=.10,
        microfinish='anodized', material_source='original concept finish',
    ),
    Material(
        'Machined aluminium', (0.52, 0.56, 0.62), .92, .20,
        ior=1.45, coat=.05, coat_rough=.18, anisotropy=.12,
        microfinish='machined', material_source='original concept finish',
    ),
    Material(
        'Stainless steel', (0.48, 0.52, 0.56), .96, .17,
        ior=1.50, coat=.025, coat_rough=.16, anisotropy=.18,
        microfinish='turned', material_source='original concept finish',
    ),
    Material(
        'Nitinol actuator wire', (0.39, 0.43, 0.47), .94, .22,
        ior=1.50, coat=.015, coat_rough=.20,
        microfinish='drawn-wire', material_source='0.20 mm source-guided wire concept',
    ),
    Material(
        'Copper electrical jumper', (0.66, 0.24, 0.075), .96, .19,
        ior=1.50, coat=.02, coat_rough=.16,
        microfinish='copper-wire', material_source='original concept conductor routing',
    ),
    Material(
        'PEEK / ceramic-like insulator', (0.55, 0.42, 0.19), .02, .38,
        ior=1.62, coat=.06, coat_rough=.24,
        microfinish='polymer', material_source='original concept material assignment',
    ),
    Material(
        'Polymer linear bushing', (0.025, 0.030, 0.038), .0, .48,
        ior=1.50, coat=.04, coat_rough=.28,
        microfinish='polymer', material_source='original concept bushing',
    ),
]


def _soften(shape, radius=.4):
    try:
        return shape.edges().fillet(radius)
    except Exception:
        return shape


def _cylinder(x0, x1, y, z, radius):
    return cq.Workplane('YZ', origin=(x0, y, z)).circle(radius).extrude(x1 - x0).val()


def _annulus(x0, x1, y, z, outer, inner):
    return cq.Workplane('YZ', origin=(x0, y, z)).circle(outer).circle(inner).extrude(x1 - x0).val()


def _plate(x0, x1, radius, holes=()):
    shape = cq.Workplane('YZ', origin=(x0, 0, 0)).circle(radius).extrude(x1 - x0).val()
    for y, z, r in holes:
        cutter = cq.Workplane('YZ', origin=(x0 - .5, y, z)).circle(r).extrude((x1 - x0) + 1).val()
        shape = shape.cut(cutter)
    return _soften(shape, .55)


def _fastener(x0, length, y, z, head_radius=2.6, shaft_radius=1.45):
    shaft = _cylinder(x0, x0 + length, y, z, shaft_radius)
    head = _cylinder(x0 - 2.6, x0, y, z, head_radius)
    shape = shaft.fuse(head)
    socket = cq.Workplane('YZ', origin=(x0 - 2.8, y, z)).polygon(6, 2.2).extrude(1.25).val()
    return _soften(shape.cut(socket), .16)


def _mesh_tube(name, points, radius, material, *, group, role, provenance='designed-concept', sides=14):
    vertices, faces = tube_mesh(points, radius, sides)
    return mesh_part(name, vertices, faces, material, group=group, role=role, provenance=provenance)


def _circle_points(radius, count, phase=0.0):
    return [(radius * math.cos(phase + math.tau * i / count), radius * math.sin(phase + math.tau * i / count)) for i in range(count)]


def build() -> Assembly:
    parts = []

    def add_cad(name, shape, material, group, role, provenance='designed-concept', **kw):
        parts.append(cad_part(name, shape, material, tolerance=.035, angular=.055, group=group, role=role, provenance=provenance, **kw))

    guides = _circle_points(18.0, 4, math.pi / 4)
    fibers = _circle_points(10.5, 12, math.pi / 12)
    guide_holes = [(y, z, 1.72) for y, z in guides]
    fiber_holes = [(y, z, .34) for y, z in fibers]

    add_cad('V2_01_Rear_frame', _plate(0, 6, 24, guide_holes), 0, 'structure', 'Fixed rear reaction plate')
    add_cad('V2_02_Front_frame', _plate(182, 188, 24, guide_holes), 0, 'structure', 'Front guide/output plate')

    for i, (y, z) in enumerate(guides):
        add_cad(f'V2_03_Guide_{i+1:02}', _soften(_cylinder(6, 182, y, z, 1.5), .08), 2, 'guides', 'Precision guide rod')
        add_cad(f'V2_04_Bushing_{i+1:02}', _annulus(155.5, 162.5, y, z, 2.85, 1.58), 6, 'carriage', 'Designed low-friction carriage bushing')

    add_cad('V2_05_Fixed_anchor', _plate(8, 12, 14.2, fiber_holes), 5, 'anchors', 'Insulating fixed fiber/crimp carrier')
    add_cad('V2_06_Moving_anchor', _plate(152, 156, 14.2, fiber_holes), 5, 'carriage', 'Insulating moving fiber/crimp carrier')
    add_cad('V2_07_Carriage', _plate(156, 162, 21.2, [(y, z, 1.82) for y, z in guides]), 1, 'carriage', 'Rigid output carriage')
    add_cad('V2_08_Output_rod', _soften(_cylinder(162, 204, 0, 0, 3.0), .16), 2, 'output', 'Linear output rod')

    eye = cq.Workplane('YZ', origin=(202, 0, 0)).circle(6).extrude(8).val()
    eye_hole = cq.Solid.makeCylinder(2.5, 14, cq.Vector(205, -7, 0), cq.Vector(0, 1, 0))
    add_cad('V2_09_Output_eye', _soften(eye.cut(eye_hole), .55), 1, 'output', 'Designed clevis-style output eye')

    add_cad('V2_10_Spring_fixed_seat', _annulus(20, 23, 0, 0, 7.5, 2.2), 1, 'spring', 'Return-spring fixed seat')
    add_cad('V2_11_Spring_moving_seat', _annulus(148, 152, 0, 0, 7.5, 2.2), 1, 'spring', 'Return-spring moving seat')

    helix = []
    turns = 15.5
    for i in range(560):
        u = i / 559
        x = 24.0 + u * (148.0 - 24.0)
        a = math.tau * turns * u
        helix.append((x, 6.0 * math.cos(a), 6.0 * math.sin(a)))
    parts.append(_mesh_tube('V2_12_Return_spring', helix, .46, 2, group='spring', role='Continuous original-design helical return spring geometry', sides=14))

    for i, (y, z) in enumerate(fibers):
        parts.append(_mesh_tube(f'V2_13_Nitinol_{i+1:02}', [(12, y, z), (152, y, z)], .10, 3, group='fibers', role='0.20 mm nominal NiTi active wire', provenance='source-guided-concept', sides=12))
        add_cad(f'V2_14_Fixed_crimp_{i+1:02}', _soften(_cylinder(9.4, 12.4, y, z, .62), .08), 2, 'electrical', 'Original-design crimp barrel envelope')
        add_cad(f'V2_15_Moving_crimp_{i+1:02}', _soften(_cylinder(151.6, 154.6, y, z, .62), .08), 2, 'electrical', 'Original-design moving crimp barrel envelope')

    for s in range(3):
        ids = [s * 4 + j for j in range(4)]
        for j in range(3):
            ia, ib = ids[j], ids[j + 1]
            ya, za = fibers[ia]; yb, zb = fibers[ib]
            moving = (j % 2 == 0)
            x_end = 152.8 if moving else 11.0
            x_mid = 155.2 if moving else 8.1
            parts.append(_mesh_tube(f'V2_16_String_{s+1}_jumper_{j+1}', [(x_end, ya, za), (x_mid, (ya+yb)/2, (za+zb)/2), (x_end, yb, zb)], .38, 4, group='electrical', role='Original-design copper series jumper', sides=14))

    bolt_points = _circle_points(20.0, 4, math.pi / 4)
    for i, (y, z) in enumerate(bolt_points):
        add_cad(f'V2_17_Rear_fastener_{i+1:02}', _fastener(-2.5, 9, y, z), 2, 'fasteners', 'Original-design M3-class socket-fastener envelope')
        add_cad(f'V2_18_Front_fastener_{i+1:02}', _fastener(179.5, 11, y, z), 2, 'fasteners', 'Original-design M3-class socket-fastener envelope')

    views = {
        'hero': View(
            az=218, el=18, scale=72, target=(101, 0, 0),
            focal_length_mm=62, sensor_width_mm=36, camera_distance_mm=360,
            f_stop=5.6, environment_strength=.20, background_strength=.80,
            light_size=1.55, floor_gap_mm=1.5, floor_roughness=.88, exposure=.96,
            title='NITINOL FIBER ACTUATOR / V2 CONCEPT',
            note='Continuous 12-wire bundle / real helical spring path / explicit electrical routing',
        ),
        'carriage_macro': View(
            az=155, el=14, scale=48, target=(152, 0, 0),
            focal_length_mm=72, sensor_width_mm=36, camera_distance_mm=255,
            f_stop=4.0, environment_strength=.18, background_strength=.75,
            light_size=1.7, floor_gap_mm=1.5, floor_roughness=.90, exposure=.98,
            title='MOVING CARRIAGE / FIBER TERMINATION',
            note='Crimps, bushings, guide rods and output interface remain visible at close range',
        ),
        'fiber_bundle': View(
            az=226, el=15, scale=62, target=(87, 0, 0),
            focal_length_mm=68, sensor_width_mm=36, camera_distance_mm=315,
            f_stop=6.3, environment_strength=.20, background_strength=.78,
            light_size=1.55, floor_gap_mm=1.5, floor_roughness=.88, exposure=.98,
            hide=('structure',),
            title='ACTIVE BUNDLE / INTERNAL INSPECTION',
            note='12 continuous 0.20 mm design wires; no segmented contraction proxy',
        ),
        'engineering': View(
            az=45, el=20, scale=118, target=(101, 0, 0), projection='orthographic', floor=False,
            f_stop=16., environment_strength=.28,
            title='V2 ACTUATOR / ORTHOGRAPHIC INSPECTION',
            note='Engineering view remains orthographic by explicit choice',
        ),
    }

    metadata = {
        'truth_intent': 'concept',
        'fidelity': 'Original higher-detail actuator concept. No recovered manufacturer frame/crimp/spring/bushing internals are claimed.',
        'motion_model': 'Static v2 presentation geometry. No fake prescribed SMA contraction animation.',
        'source_guidance': {
            'wire_diameter_mm': .20,
            'active_wire_length_mm': 140.0,
            'wire_source_note': 'same Dynalloy source-guided diameter used by the v1 study',
        },
        'assumptions': [
            'Frame, spring, bushings, fasteners, crimps, output interface and routing are original design geometry.',
            'The render validates authored geometry/provenance, not stress, fatigue, fit, heat transfer or load capacity.',
        ],
    }
    return Assembly('nitinol_fiber_actuator_v2', parts, MATERIALS, views, metadata=metadata)
