"""Compact 5:1 planetary gearbox actuator module.

This is an ORIGINAL ENGINEERING CONCEPT built to exercise Mechanism Lab gear,
assembly, kinematic and photographic-rendering infrastructure. It is not copied
from a commercial reducer and is not load/contact/fatigue/manufacturing qualified.

Topology: 18T sun / 27T planets / 72T fixed internal ring, module 0.8 mm,
20 degree pressure angle, three equally spaced planets, carrier output.
"""
from __future__ import annotations

from dataclasses import asdict
import math
import numpy as np
import cadquery as cq

from ..core import Assembly, Material, View, cad_part
from ..gears import (
    PlanetarySpec,
    external_spur_gear,
    internal_ring_gear,
    planetary_angles,
    planet_centers,
    planetary_validation_report,
)
from ..gears.planetary import planet_initial_phases, orbit_and_spin, rotation_x4


SPEC = PlanetarySpec(
    sun_teeth=18,
    planet_teeth=27,
    ring_teeth=72,
    planets=3,
    module=0.8,
    pressure_angle_deg=20.0,
    backlash_mm=0.10,
    face_width_mm=10.0,
    input_speed_rps=0.45,
)

MATERIALS = [
    Material('Black hard-anodized housing', (0.018, 0.022, 0.028), .72, .29,
             coat=.06, coat_rough=.23, anisotropy=.05, microfinish='anodized', material_source='original concept finish'),
    Material('Machined 7075-like carrier', (0.48, 0.52, 0.58), .92, .21,
             coat=.04, coat_rough=.18, anisotropy=.12, microfinish='machined', material_source='original concept finish'),
    Material('Nitrided gear steel', (0.17, 0.19, 0.21), .96, .18,
             coat=.02, coat_rough=.16, anisotropy=.10, microfinish='machined', material_source='original concept finish'),
    Material('Ground/turned shaft steel', (0.46, 0.50, 0.54), .97, .15,
             coat=.02, coat_rough=.14, anisotropy=.20, microfinish='turned', material_source='original concept finish'),
    Material('Oil-impregnated bronze bushing', (0.42, 0.24, 0.075), .83, .29,
             coat=.02, coat_rough=.24, microfinish='machined', material_source='original concept bearing choice'),
    Material('Matte elastomer seal', (0.012, 0.014, 0.017), .0, .62,
             ior=1.48, microfinish='polymer', material_source='original concept seal'),
    Material('FR4 encoder PCB', (0.025, 0.115, 0.055), .0, .46,
             ior=1.55, coat=.08, coat_rough=.20, microfinish='polymer', material_source='original concept electronics envelope'),
    Material('Copper sensor/contact metal', (0.63, 0.23, 0.065), .96, .19,
             coat=.02, coat_rough=.16, microfinish='copper-wire', material_source='original concept conductor'),
    Material('Encoder magnet', (0.075, 0.085, 0.095), .72, .34,
             microfinish='bead-blasted', material_source='original concept magnet envelope'),
    Material('Stainless fastener steel', (0.34, 0.37, 0.40), .95, .20,
             anisotropy=.12, microfinish='turned', material_source='original concept fastener'),
    Material('Connector polymer', (0.025, 0.030, 0.038), .0, .48,
             ior=1.50, microfinish='polymer', material_source='original concept connector shell'),
]


def _cylinder(x0, x1, y, z, radius):
    return cq.Workplane('YZ', origin=(x0, y, z)).circle(radius).extrude(x1 - x0).val()


def _annulus(x0, x1, y, z, outer, inner):
    return cq.Workplane('YZ', origin=(x0, y, z)).circle(outer).circle(inner).extrude(x1 - x0).val()


def _soften(shape, radius=.35):
    try:
        return cq.Workplane(obj=shape).edges().fillet(radius).val()
    except Exception:
        return shape


def _axial_holes(shape, points, radius, x0, x1):
    holes = cq.Workplane('YZ').pushPoints(points).circle(radius).extrude(x1 - x0).translate((x0, 0, 0))
    return cq.Workplane(obj=shape).cut(holes).val()


def _disk_with_holes(x0, x1, radius, holes=(), center_bore=0.0):
    w = cq.Workplane('YZ', origin=(x0, 0, 0)).circle(radius)
    if center_bore:
        w = w.circle(center_bore)
    shape = w.extrude(x1 - x0).val()
    if holes:
        shape = _axial_holes(shape, holes, 1.58, x0 - .5, x1 + .5)
    return _soften(shape, .32)


def _socket_bolt(x0, x1, y, z, *, head_at='start', shaft_r=1.48, head_r=2.75):
    if x1 <= x0:
        raise ValueError('bolt x1 must exceed x0')
    shaft = _cylinder(x0, x1, y, z, shaft_r)
    if head_at == 'start':
        head = _cylinder(x0 - 2.7, x0, y, z, head_r)
        socket = cq.Workplane('YZ', origin=(x0 - 2.85, y, z)).polygon(6, 2.15).extrude(1.2).val()
    elif head_at == 'end':
        head = _cylinder(x1, x1 + 2.7, y, z, head_r)
        socket = cq.Workplane('YZ', origin=(x1 + 1.65, y, z)).polygon(6, 2.15).extrude(1.2).val()
    else:
        raise ValueError('head_at must be start or end')
    return _soften(shaft.fuse(head).cut(socket), .10)


def _radial_pin(x, y0, y1, z, radius):
    return cq.Solid.makeCylinder(radius, y1 - y0, cq.Vector(x, y0, z), cq.Vector(0, 1, 0))


def _housing_front(bolt_points):
    shape = _cylinder(0, 22, 0, 0, 36.0)
    # Through shaft clearance plus two separated plain-bearing seats and a real
    # shoulder between them. The rear opens into the rotating-carrier cavity.
    shape = shape.cut(_cylinder(-1, 23, 0, 0, 6.20))
    shape = shape.cut(_cylinder(1.5, 9.5, 0, 0, 9.05))
    shape = shape.cut(_cylinder(11.0, 19.0, 0, 0, 9.05))
    shape = shape.cut(_cylinder(19.0, 23.0, 0, 0, 30.30))
    shape = _axial_holes(shape, bolt_points, 1.62, -.5, 22.5)
    return _soften(shape, .65)


def _housing_rear(bolt_points):
    shape = _cylinder(36, 60, 0, 0, 36.0)
    shape = shape.cut(_cylinder(35, 61, 0, 0, 4.25))
    shape = shape.cut(_cylinder(35.5, 43.0, 0, 0, 30.30))
    shape = shape.cut(_cylinder(44.0, 50.0, 0, 0, 7.05))
    shape = shape.cut(_cylinder(51.0, 57.0, 0, 0, 7.05))
    shape = _axial_holes(shape, bolt_points, 1.62, 35.5, 60.5)
    # Small side connector pocket through the +Y wall; a separate connector shell
    # occupies this opening rather than floating on the housing surface.
    pocket = cq.Workplane('XY').box(10, 8, 8, centered=(True, True, True)).translate((48, 34.5, 0)).val()
    shape = shape.cut(pocket)
    return _soften(shape, .65)


def _ring_gear(bolt_points):
    gear = internal_ring_gear(
        SPEC.ring_teeth, SPEC.module, SPEC.face_width_mm,
        outer_radius=35.0, origin=(24, 0, 0), pressure_angle_deg=SPEC.pressure_angle_deg,
        backlash=SPEC.backlash_mm,
    )
    front_flange = _annulus(22, 24, 0, 0, 35.0, 30.20)
    rear_flange = _annulus(34, 36, 0, 0, 35.0, 30.20)
    shape = gear.fuse(front_flange).fuse(rear_flange)
    shape = _axial_holes(shape, bolt_points, 1.62, 21.5, 24.5)
    shape = _axial_holes(shape, bolt_points, 1.62, 33.5, 36.5)
    return shape


def _output_shaft(output_holes):
    shaft = _cylinder(-12, 19, 0, 0, 6.0)
    flange = _cylinder(-18, -12, 0, 0, 15.0)
    carrier_flange = _cylinder(16.5, 19.0, 0, 0, 12.0)
    shape = shaft.fuse(flange).fuse(carrier_flange)
    # Downstream interface: six through holes in the exposed output flange.
    downstream = [(11.0 * math.cos(math.tau * i / 6), 11.0 * math.sin(math.tau * i / 6)) for i in range(6)]
    shape = _axial_holes(shape, downstream, 1.65, -18.5, -11.5)
    # Carrier mounting holes are actual holes through the integral shaft flange.
    shape = _axial_holes(shape, output_holes, 1.58, 16.0, 19.5)
    return _soften(shape, .30)


def _motor_adapter():
    shape = _annulus(60, 64, 0, 0, 30.0, 7.2)
    holes = [(24.0 * math.cos(math.pi/4 + math.tau*i/4), 24.0 * math.sin(math.pi/4 + math.tau*i/4)) for i in range(4)]
    shape = _axial_holes(shape, holes, 2.05, 59.5, 64.5)
    return _soften(shape, .35)


def pose(part, time_seconds: float, explode: float, spec: PlanetarySpec = SPEC) -> np.ndarray:
    q = planetary_angles(spec, time_seconds)
    motion = str(part.motion)
    if motion == 'sun':
        T = rotation_x4(q['sun'])
    elif motion == 'carrier':
        T = rotation_x4(q['carrier'])
    elif motion.startswith('planet_'):
        index = int(motion.split('_', 1)[1])
        base_center = planet_centers(spec, 0.0)[index]
        phase0 = planet_initial_phases(spec)[index]
        relative_spin = (q['planets'][index] - phase0) - q['carrier']
        T = orbit_and_spin(base_center, q['carrier'], relative_spin)
    else:
        T = np.eye(4)
    T = np.asarray(T, float).copy()
    T[:3, 3] += np.asarray(part.explode, float) * float(explode)
    return T


def build() -> Assembly:
    SPEC.validate()
    report = planetary_validation_report(SPEC)
    parts = []

    def add(name, shape, material, group, role, *, motion='fixed', center=(0,0,0), explode=(0,0,0), provenance='designed-concept', tolerance=.028, angular=.045):
        parts.append(cad_part(
            name, shape, material, tolerance=tolerance, angular=angular,
            group=group, role=role, motion=motion, center=np.asarray(center,float),
            explode=np.asarray(explode,float), provenance=provenance,
        ))

    bolt_points = [(32.4 * math.cos(math.pi/6 + math.tau*i/6), 32.4 * math.sin(math.pi/6 + math.tau*i/6)) for i in range(6)]
    output_holes = [(9.2 * math.cos(math.pi/4 + math.tau*i/4), 9.2 * math.sin(math.pi/4 + math.tau*i/4)) for i in range(4)]
    centers = planet_centers(SPEC, 0.0)
    phases = planet_initial_phases(SPEC)

    add('PGA01_Front_housing', _housing_front(bolt_points), 0, 'front_cover', 'Front machined housing with separated output-bearing seats and carrier cavity', explode=(-55,0,0))
    add('PGA02_Rear_housing', _housing_rear(bolt_points), 0, 'housing', 'Rear machined housing with input-bearing seats, encoder cavity and connector pocket', explode=(55,0,0))
    add('PGA03_Fixed_internal_ring', _ring_gear(bolt_points), 2, 'ring', '72T fixed internal involute ring and structural insert', explode=(0,0,0), tolerance=.020, angular=.035)

    # Fixed housing screws genuinely cross housing material and terminate in the
    # ring insert flange rather than floating beside it.
    for i,(y,z) in enumerate(bolt_points):
        add(f'PGA04_Front_ring_screw_{i+1:02}', _socket_bolt(0,24,y,z,head_at='start'), 9, 'fasteners', 'Front housing-to-ring M3-class socket screw', explode=(-65,0,0))
        add(f'PGA05_Rear_ring_screw_{i+1:02}', _socket_bolt(34,60,y,z,head_at='end'), 9, 'fasteners', 'Rear housing-to-ring M3-class socket screw', explode=(65,0,0))

    # Integrated sun + input shaft: no invisible key/spline is implied.
    sun = external_spur_gear(
        SPEC.sun_teeth, SPEC.module, SPEC.face_width_mm, origin=(24,0,0), phase=0.0,
        pressure_angle_deg=SPEC.pressure_angle_deg, backlash=SPEC.backlash_mm,
    )
    input_shaft = _cylinder(24,72,0,0,4.0)
    add('PGA06_Input_sun_shaft', sun.fuse(input_shaft), 2, 'sun', 'Integral 18T sun/input shaft; avoids an unmodeled torque joint', motion='sun', explode=(72,0,0), tolerance=.020, angular=.035)

    # Output shaft with real downstream and carrier mounting interfaces.
    add('PGA07_Output_shaft', _output_shaft(output_holes), 3, 'output', '12 mm dual-supported output shaft and interface flange', motion='carrier', explode=(-72,0,0))

    # Dual plain bearings for both coaxial shafts. These are explicit concept
    # bushings, not decorative ball-bearing rings with missing rolling elements.
    add('PGA08_Output_bushing_front', _annulus(1.5,9.5,0,0,9.0,6.08), 4, 'bearings', 'Front 12 mm shaft bronze radial bushing', explode=(-38,0,0))
    add('PGA09_Output_bushing_rear', _annulus(11,19,0,0,9.0,6.08), 4, 'bearings', 'Rear 12 mm shaft bronze radial bushing', explode=(-28,0,0))
    add('PGA10_Input_bushing_front', _annulus(44,50,0,0,7.0,4.08), 4, 'bearings', 'Front 8 mm input-shaft bronze radial bushing', explode=(30,0,0))
    add('PGA11_Input_bushing_rear', _annulus(51,57,0,0,7.0,4.08), 4, 'bearings', 'Rear 8 mm input-shaft bronze radial bushing', explode=(40,0,0))
    add('PGA12_Output_seal', _annulus(-.5,1.5,0,0,9.0,6.03), 5, 'seals', 'Output radial lip-seal envelope', explode=(-45,0,0))
    add('PGA13_Input_seal', _annulus(58,60,0,0,7.0,4.03), 5, 'seals', 'Input radial lip-seal envelope', explode=(48,0,0))

    pin_holes = [(y,z) for y,z in centers]
    spacer_points = [(25.0 * math.cos(math.pi/3 + math.tau*i/3),25.0 * math.sin(math.pi/3 + math.tau*i/3)) for i in range(3)]
    all_front_holes = pin_holes + output_holes
    front_plate = _disk_with_holes(19,22,26.7,all_front_holes,center_bore=6.15)
    rear_plate = _disk_with_holes(36,39,26.7,pin_holes,center_bore=4.65)
    add('PGA14_Carrier_front', front_plate, 1, 'carrier', 'Front planet carrier plate', motion='carrier', explode=(-14,0,0))
    add('PGA15_Carrier_rear', rear_plate, 1, 'carrier', 'Rear planet carrier plate with input-shaft clearance', motion='carrier', explode=(14,0,0))

    # Output flange fasteners tie the carrier plate to an actual shaft flange.
    for i,(y,z) in enumerate(output_holes):
        add(f'PGA16_Output_carrier_screw_{i+1:02}', _socket_bolt(17,22,y,z,head_at='end',shaft_r=1.42,head_r=2.55), 9, 'carrier_fasteners', 'Carrier-to-output-flange socket screw', motion='carrier', explode=(-18,0,0))

    # Planet pins and alternate carrier spacers form a mechanically connected cage.
    for i,(y,z) in enumerate(centers):
        add(f'PGA17_Planet_pin_{i+1:02}', _cylinder(18.5,39.5,y,z,2.50), 3, 'planet_pins', '5 mm carrier-fixed planet pin', motion='carrier', center=(29,y,z), explode=(0,10*math.cos(math.tau*i/3),10*math.sin(math.tau*i/3)))
        add(f'PGA18_Pin_clip_front_{i+1:02}', _annulus(18.25,18.65,y,z,3.45,2.55), 9, 'planet_pins', 'Front planet-pin retaining ring', motion='carrier', center=(18.45,y,z), explode=(-8,0,0))
        add(f'PGA19_Pin_clip_rear_{i+1:02}', _annulus(39.35,39.75,y,z,3.45,2.55), 9, 'planet_pins', 'Rear planet-pin retaining ring', motion='carrier', center=(39.55,y,z), explode=(8,0,0))
    for i,(y,z) in enumerate(spacer_points):
        add(f'PGA20_Carrier_spacer_{i+1:02}', _cylinder(21.5,36.5,y,z,1.35), 3, 'carrier', 'Carrier plate spacer/dowel', motion='carrier', center=(29,y,z), explode=(0,7*math.cos(math.pi/3+math.tau*i/3),7*math.sin(math.pi/3+math.tau*i/3)))

    # Three exact-phase involute planets, each with two real plain bushings rather
    # than a visually implied but mechanically absent bearing.
    for i,((y,z),phase) in enumerate(zip(centers,phases)):
        gear = external_spur_gear(
            SPEC.planet_teeth, SPEC.module, SPEC.face_width_mm,
            bore=10.15, origin=(24,y,z), phase=phase,
            pressure_angle_deg=SPEC.pressure_angle_deg, backlash=SPEC.backlash_mm,
        )
        radial_ex = (0,14*math.cos(math.tau*i/3),14*math.sin(math.tau*i/3))
        add(f'PGA21_Planet_{i+1:02}', gear, 2, 'planets', f'27T involute planet {i+1}', motion=f'planet_{i}', center=(29,y,z), explode=radial_ex, tolerance=.020, angular=.035)
        add(f'PGA22_Planet_bushing_front_{i+1:02}', _annulus(24,29,y,z,5.0,2.56), 4, 'planet_bushings', 'Front press-fit plain bushing inside planet', motion=f'planet_{i}', center=(26.5,y,z), explode=radial_ex)
        add(f'PGA23_Planet_bushing_rear_{i+1:02}', _annulus(29,34,y,z,5.0,2.56), 4, 'planet_bushings', 'Rear press-fit plain bushing inside planet', motion=f'planet_{i}', center=(31.5,y,z), explode=radial_ex)
        add(f'PGA24_Thrust_washer_front_{i+1:02}', _annulus(23.55,23.90,y,z,6.1,2.57), 4, 'thrust_washers', 'Front carrier-fixed bronze thrust washer', motion='carrier', center=(23.72,y,z), explode=(-5,0,0))
        add(f'PGA25_Thrust_washer_rear_{i+1:02}', _annulus(34.10,34.45,y,z,6.1,2.57), 4, 'thrust_washers', 'Rear carrier-fixed bronze thrust washer', motion='carrier', center=(34.28,y,z), explode=(5,0,0))

    add('PGA26_Motor_adapter', _motor_adapter(), 1, 'motor_interface', 'Rear motor pilot/interface flange; motor itself intentionally not invented', explode=(72,0,0))

    # Coaxial encoder concept: magnet rotates with input shaft; PCB is fixed and
    # physically separated. This is an envelope/interface study, not vendor CAD.
    add('PGA27_Encoder_magnet', _annulus(40.0,41.5,0,0,5.6,4.02), 8, 'encoder', 'Input-shaft encoder magnet ring envelope', motion='sun', explode=(22,0,0))
    add('PGA28_Encoder_PCB', _annulus(42.0,42.8,0,0,15.0,7.3), 6, 'encoder', 'Fixed annular encoder PCB envelope', explode=(20,0,0))
    add('PGA29_Encoder_copper_ring', _annulus(42.8,43.05,0,0,12.0,8.0), 7, 'encoder', 'Visible copper sensing/contact region on concept PCB', explode=(20,0,0))

    connector = cq.Workplane('XY').box(8,8,6,centered=(True,True,True)).translate((48,37.0,0)).val()
    add('PGA30_Connector_shell', _soften(connector,.45), 10, 'connector', 'Side connector shell seated into rear housing pocket', explode=(0,18,0))
    for j,x in enumerate((46.5,49.5)):
        add(f'PGA31_Connector_pin_{j+1:02}', _radial_pin(x,33.0,39.0,0,0.65), 7, 'connector', 'Copper connector contact pin', explode=(0,20,0))

    views = {
        'exterior': View(
            az=225, el=18, scale=50, target=(23,0,0), focal_length_mm=78, sensor_width_mm=36,
            camera_distance_mm=285, f_stop=7.1, focus_distance_mm=285,
            environment_strength=.18, light_size=1.55, light_intensity=.96, floor_gap_mm=2.0, floor_roughness=.86,
            title='PLANETARY ACTUATOR / EXTERIOR', note='72 mm housing / coaxial input and carrier output / original concept',
        ),
        'hero': View(
            az=225, el=19, scale=48, target=(28,0,0), focal_length_mm=82, sensor_width_mm=36,
            camera_distance_mm=265, f_stop=6.3, focus_distance_mm=268,
            environment_strength=.16, light_size=1.65, light_intensity=.98, floor_gap_mm=2.0, floor_roughness=.88,
            hide=('front_cover',),
            title='PLANETARY ACTUATOR / 5:1 INTERNAL HERO', note='18T sun / 3 x 27T planets / 72T fixed ring / exact carrier kinematics',
        ),
        'gear_macro': View(
            az=205, el=10, scale=24, target=(29,15,1), focal_length_mm=90, sensor_width_mm=36,
            camera_distance_mm=175, f_stop=4.5, focus_distance_mm=177,
            environment_strength=.13, light_size=1.75, light_intensity=1.02, floor=False,
            hide=('front_cover','housing','motor_interface','connector'),
            title='INVOLUTE MESH / MACRO', note='Explicit 0.10 mm pitch-circle backlash; root transitions are not cutter-certified',
        ),
        'kinematics': View(
            az=220, el=13, scale=40, target=(29,0,0), focal_length_mm=76, sensor_width_mm=36,
            camera_distance_mm=230, f_stop=6.3, focus_distance_mm=230,
            environment_strength=.14, light_size=1.65, light_intensity=.98, floor=False,
            hide=('front_cover','housing','motor_interface','connector','encoder'),
            title='WORKING PLANETARY SET', note='Sun input; fixed ring; carrier output exactly 1/5 input speed',
        ),
        'encoder_macro': View(
            az=115, el=14, scale=28, target=(43,0,0), focal_length_mm=90, sensor_width_mm=36,
            camera_distance_mm=185, f_stop=4.0, focus_distance_mm=185,
            environment_strength=.15, light_size=1.6, light_intensity=.96, floor=False,
            hide=('front_cover',),
            title='INPUT SUPPORT / ENCODER INTERFACE', note='Two input bushings / magnet ring / fixed PCB envelope',
        ),
        'exploded': View(
            az=225, el=20, scale=95, target=(25,0,0), explode=1.0, focal_length_mm=72, sensor_width_mm=36,
            camera_distance_mm=430, f_stop=9.0, focus_distance_mm=430, floor=False,
            environment_strength=.18, light_size=1.5, light_intensity=.96,
            title='PLANETARY ACTUATOR / EXPLODED INSPECTION', note='Explode offsets are inspection-only, not an assembly procedure',
        ),
        'engineering_front': View(
            az=180, el=0, scale=42, target=(29,0,0), projection='orthographic', floor=False,
            title='PLANETARY ACTUATOR / FRONT ORTHOGRAPHIC', note='Concept geometry / units mm',
        ),
        'section': View(
            az=222, el=9, scale=52, target=(27,-2,0), section=(0,1,0), floor=False,
            focal_length_mm=72, sensor_width_mm=36, camera_distance_mm=285, f_stop=8.0, focus_distance_mm=285,
            title='PLANETARY ACTUATOR / LONGITUDINAL SECTION', note='Dual shaft bushings, ring insert, carrier and encoder support',
        ),
    }

    metadata = {
        'truth_intent': 'concept',
        'design': asdict(SPEC),
        'planetary_validation': report,
        'fidelity': 'Original compact planetary actuator concept; no commercial reducer internals are claimed.',
        'kinematic_model': 'Exact rigid simple-planetary Willis relation with fixed ring. No loaded tooth contact, compliance or friction model.',
        'interfaces': {
            'housing_outer_diameter_mm': 72.0,
            'nominal_body_length_mm': 64.0,
            'output_shaft_diameter_mm': 12.0,
            'input_shaft_diameter_mm': 8.0,
            'output_support': 'two 18 mm OD bronze plain radial bushings',
            'input_support': 'two 14 mm OD bronze plain radial bushings',
            'planet_support': 'two 10 mm OD bronze bushings per 5 mm carrier-fixed pin',
        },
        'assumptions': [
            'Gear flanks are analytic 20 degree involutes; root transitions are concept geometry and not generated cutter envelopes.',
            '0.10 mm backlash is a design allowance at the pitch circle; no loaded deflection or thermal growth is solved.',
            'Housing, shafts, bushings, seals, connector, encoder and fasteners are original design geometry, not vendor CAD.',
            'No stress, bearing PV, tooth contact, lubrication, efficiency, fatigue, noise, tolerance-stack or manufacturing qualification is claimed.',
        ],
        'safety': [
            'Do not fabricate/load this concept without independent gear, shaft, bearing, fastener and housing calculations.',
            'Guard rotating gears and shafts during any physical prototype testing.',
        ],
    }
    return Assembly('planetary_actuator', parts, MATERIALS, views, metadata=metadata, motion_function=pose)
