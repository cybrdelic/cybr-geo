"""ATLAS: a self-centering desktop inspection and small-part assembly fixture.

All visible surfaces are built through CYBR GEO's geometry contract.  Three
Archimedean slots drive three independently guided jaws.  Motion is prescribed
from that exact cam constraint; it is not a contact-force simulation.

Coordinates: millimetres, optical/workpiece axis +X, Z up.  This is an original
design study, with nominal interfaces, not a toleranced manufacturing release.
"""
from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass

import cadquery as cq
import numpy as np
from cybrgeo.features import involute_spur_gear

from ..advanced_geometry import (
    bcc_lattice, drafted_cylinder, gyroid_sheet_mesh, helical_sweep,
    lofted_elliptic_shell, lofted_solid, spline_sweep_tube, toroidal_groove,
)
from ..core import Assembly, Material, View, cad_part, mesh_part, rotation_x
from ..geometry import ring, sector, tube_mesh


@dataclass(frozen=True)
class FixtureSpec:
    jaw_count: int = 3
    closed_aperture_radius_mm: float = 14.0
    radial_travel_mm: float = 10.0
    cam_angle_rad: float = math.pi / 4
    follower_radius_mm: float = 3.0
    slot_half_width_mm: float = 3.18
    follower_closed_radius_mm: float = 36.0
    cycle_seconds: float = 8.0
    shell_wall_mm: float = 2.5


SPEC = FixtureSpec()
JAW_ANGLES = tuple(math.radians(90 + i * 120) for i in range(3))
BLACK, BLUE, SILVER, STEEL, BRASS, COPPER, POLYMER, CERAMIC, TITANIUM, INK = range(10)
MATERIALS = [
    Material('Graphite / hard anodized aluminium', (.045, .051, .060), .56, .30, microfinish='anodized'),
    Material('Deep petrol / anodized aluminium', (.018, .115, .145), .68, .28, microfinish='anodized'),
    Material('Bead blasted aluminium', (.49, .54, .59), .88, .27, microfinish='bead-blasted'),
    Material('Ground stainless steel', (.57, .60, .64), .96, .21, microfinish='turned'),
    Material('Phosphor bronze / cam plate', (.53, .29, .086), .92, .25, microfinish='machined'),
    Material('Copper / service conductor', (.68, .235, .066), .97, .23, microfinish='copper-wire'),
    Material('Charcoal elastomer', (.022, .026, .032), .0, .52, microfinish='polymer'),
    Material('Ivory PEEK / replaceable soft jaws', (.56, .53, .43), .0, .32, microfinish='polymer'),
    Material('Sintered titanium / porous insert', (.30, .33, .37), .73, .42, microfinish='bead-blasted'),
    Material('Pale laser identification marks', (.62, .72, .73), .15, .44),
]


def cam_angle(t):
    """Closed at t=0; open at half-cycle; continuous zero end velocities."""
    return SPEC.cam_angle_rad * (.5 - .5 * math.cos(math.tau * t / SPEC.cycle_seconds))


def jaw_travel(t):
    return SPEC.radial_travel_mm * cam_angle(t) / SPEC.cam_angle_rad


def pose(part, t, explode):
    T = np.eye(4)
    if part.motion == 'cam':
        T[:3, :3] = rotation_x(-cam_angle(t))
    elif part.motion == 'pinion':
        R=rotation_x(cam_angle(t)*44/16)
        pivot=np.array((0,75,0))
        T[:3,:3]=R
        T[:3,3]=pivot-R@pivot
    elif part.motion.startswith('jaw:'):
        a = JAW_ANGLES[int(part.motion.split(':')[1])]
        T[:3, 3] = (0, jaw_travel(t) * math.cos(a), jaw_travel(t) * math.sin(a))
    T[:3, 3] += np.asarray(part.explode) * explode
    return T


def _rotate(shape, a):
    return shape.rotate((0, 0, 0), (1, 0, 0), math.degrees(a))


def _box(x, y, z, center, radius=0):
    s = cq.Workplane('XY').box(x, y, z)
    if radius:
        s = s.edges().fillet(radius)
    return s.val().translate(center)


def _axial_cylinder(x, length, radius, y=0, z=0):
    return cq.Solid.makeCylinder(radius, length, cq.Vector(x, y, z), cq.Vector(1, 0, 0))


def _socket_screw(x, length, y, z, shaft=1.45, head=2.65):
    body = _axial_cylinder(x, length, shaft, y, z)
    cap = cq.Workplane('YZ', origin=(x + length, y, z)).circle(head).extrude(2.4).edges('>X').chamfer(.28).val()
    socket = cq.Workplane('YZ', origin=(x + length + 1.0, y, z)).polygon(6, 2.3).extrude(2).val()
    return body.fuse(cap).cut(socket)


def _revolve_profile(points):
    return cq.Workplane('XY').polyline(points).close().revolve(360, (0, 0), (1, 0)).val()


def spiral_centerline(a, phase=0):
    """The exact geometric follower locus, before the cam rotates."""
    r = SPEC.follower_closed_radius_mm + SPEC.radial_travel_mm * a / SPEC.cam_angle_rad
    return (r * math.cos(a + phase), r * math.sin(a + phase))


def _spiral_slot(phase):
    """Closed spline ribbon with circular end caps, extruded axially.

    The two flanks offset the analytic spiral along its planar normal.  They are
    interpolating BREP splines, sampled at 101 points; the numerical fidelity is
    measured separately against the exact cam law.
    """
    aa = np.linspace(-.13, SPEC.cam_angle_rad + .13, 101)
    k = SPEC.radial_travel_mm / SPEC.cam_angle_rad
    rr = SPEC.follower_closed_radius_mm + k * aa
    angles = aa + phase
    c = np.column_stack((rr * np.cos(angles), rr * np.sin(angles)))
    tangent = np.column_stack((k * np.cos(angles) - rr * np.sin(angles),
                               k * np.sin(angles) + rr * np.cos(angles)))
    tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    left = c + SPEC.slot_half_width_mm * normal
    right = c - SPEC.slot_half_width_mm * normal
    wp = cq.Workplane('YZ', origin=(98, 0, 0)).moveTo(*left[0])
    wp = wp.spline([tuple(p) for p in left[1:]], includeCurrent=True)
    end = c[-1] + SPEC.slot_half_width_mm * tangent[-1]
    wp = wp.threePointArc(tuple(end), tuple(right[-1]))
    wp = wp.spline([tuple(p) for p in right[-2::-1]], includeCurrent=True)
    end = c[0] - SPEC.slot_half_width_mm * tangent[0]
    return wp.threePointArc(tuple(end), tuple(left[0])).close().extrude(11).val()


def build():
    started = time.time()
    parts = []

    def add(name, shape, material, group, role, tags=(), motion='fixed', explode=(0, 0, 0), tolerance=.04, angular=.11):
        p = cad_part(name, shape, material, tolerance=tolerance, angular=angular,
                     group=group, role=role, tags=tuple('technique:' + t for t in tags),
                     motion=motion, explode=np.array(explode, float), provenance='designed-concept')
        parts.append(p)
        print(f'[{len(parts):03}] {name}: {len(p.faces):,} triangles / {time.time()-started:.1f}s', flush=True)
        return p

    # Workbench interface: real counterbored slots, machined edge radii, rubber feet.
    base = _box(158, 142, 8, (55, 0, -82), 3)
    for x in (-7, 117):
        for y in (-54, 54):
            hole = cq.Workplane('XY', origin=(x, y, -89)).slot2D(18, 6.5).extrude(14).val()
            cb = cq.Workplane('XY', origin=(x, y, -80.8)).slot2D(22, 10.5).extrude(4).val()
            base = base.cut(hole).cut(cb)
    add('AT_001_Fixture_base', base, BLACK, 'base', 'Bench plate with four elongated counterbored mounting slots', ('extrusion', 'fillet', 'slot', 'boolean-csg'), explode=(0, 0, -22))
    for i, (x, y) in enumerate(((-5,-53),(-5,53),(115,-53),(115,53))):
        foot = cq.Workplane('XY', origin=(x, y, -88.2)).circle(10).extrude(2.6).val()
        add(f'AT_002_Elastomer_foot_{i+1}', foot, POLYMER, 'feet', 'Bench isolation foot', ('pattern',), explode=(0, 0, -25))

    # An additive-style BCC pedestal seated between the base and curved saddle.
    lattice = bcc_lattice(origin=(30,-24,-77), cells=(4,4,1), pitch=(12,12,13), strut_radius=1.30, node_radius=1.50)
    add('AT_010_BCC_pedestal', lattice, TITANIUM, 'lattice', 'Analytic BREP BCC support compound; intersecting struts/nodes, not boolean-unified', ('bcc-lattice', 'analytic-compound'), explode=(0,0,-10), tolerance=.09, angular=.24)
    saddle = sector(66,59.4,25,83,math.radians(228),math.radians(312))
    add('AT_011_Conformal_saddle', saddle, SILVER, 'support', 'Circular contact saddle for the stationary housing', ('arc-profile', 'extrusion'), explode=(0,0,-10))
    for i, y in enumerate((-39,39)):
        # A tapered four-sided ruled BREP web anchors the curved saddle edges.
        sections = cq.Workplane('XY', origin=(54,y,-78)).rect(61,6)
        sections = sections.workplane(offset=30).rect(46,4)
        rib = sections.loft(ruled=True).val()
        add(f'AT_012_Tapered_support_web_{i+1}', rib, SILVER, 'support', 'Ruled tapered load-transfer web from base to saddle', ('ruled-loft', 'gusset'), explode=(0,0,-10))

    # Stationary body: a hollow BREP spline loft, with actual swept access windows.
    shell = lofted_elliptic_shell([(8,59.6,59.6,0,0),(26,63,61,0,.8),(52,64,61.5,0,1.1),
                                  (74,62.5,60.5,0,.4),(88,59.6,59.6,0,0)], SPEC.shell_wall_mm)
    for phase in (math.radians(37), math.radians(180), math.radians(285)):
        cutter = _rotate(_box(41,24,22,(49,60,0),5),phase)
        shell = shell.cut(cutter)
    for x in (18,77):
        for j in range(12):
            a = j*math.tau/12
            cutter = cq.Solid.makeCylinder(1.6,12,cq.Vector(x,68*math.cos(a),68*math.sin(a)),cq.Vector(0,-math.cos(a),-math.sin(a)))
            shell = shell.cut(cutter)
    shell=shell.cut(_box(12,44,46,(27,68,0),2.0))
    add('AT_020_Freeform_housing', shell, BLUE, 'housing', 'Hollow five-section elliptical BREP housing with three real inspection windows', ('freeform-loft','hollow-shell','boolean-csg','radial-pattern'), explode=(-18,0,38), tolerance=.05)

    rear = ring(59.5,20,3,10)
    rear = rear.cut(_axial_cylinder(1,12,6.15,-22,24))
    for a in np.arange(6)*math.tau/6:
        rear = rear.cut(_axial_cylinder(1,12,1.65,54*math.cos(a),54*math.sin(a)))
    add('AT_021_Rear_bulkhead',rear,BLACK,'housing','Rear mounting bulkhead, through access bore and six fastener bores',('annular-profile','bolt-circle','boolean-csg'),explode=(-35,0,0))
    for i,a in enumerate(np.arange(6)*math.tau/6):
        add(f'AT_022_Rear_screw_{i+1}',_socket_screw(4,9,54*math.cos(a),54*math.sin(a)),STEEL,'fasteners','Socket screw on matching rear bulkhead bore',('socket','chamfer','pattern'),explode=(-30,0,0))

    # Two actual bearing sleeve seats support the rotating input/cam sleeve.
    for i,x in enumerate((12,76)):
        add(f'AT_030_Bronze_radial_bushing_{i+1}',ring(57.0,52.2,x,x+8),BRASS,'bearings','Nominal 0.2 mm radial running clearance on input sleeve',('bearing-seat','annulus'),explode=(-13 if i==0 else 12,0,0))
        add(f'AT_031_Bearing_retaining_ring_{i+1}',ring(58.0,51.6,x-1.8,x),STEEL,'bearings','Axial bushing retaining ring',('revolve','annulus'),explode=(-18 if i==0 else 15,0,0))
    drive = ring(52,46,10,100)
    # An actual reduced-diameter gear seat replaces overlapping nominal solids.
    drive=drive.cut(ring(54,48.95,23.8,30.2))
    gear_keyway=_box(6.4,4.0,2.4,(27,49,0))
    drive=drive.cut(gear_keyway)
    # Three broad windows preserve ribs tying the handwheel to the front cam.
    for a in (math.radians(0),math.radians(120),math.radians(240)):
        drive = drive.cut(_rotate(_box(40,20,33,(48,51,0),4),a))
    add('AT_032_Windowed_drive_sleeve',drive,SILVER,'drive','One connected torque sleeve between handwheel and cam plate',('annulus','boolean-csg','window'),motion='cam',explode=(0,0,0))

    # The original cybrgeo involute tool supplies a fine-adjustment gear input.
    # Pitch diameters 110 and 40 mm, 75 mm centers, exact 44:16 rotation ratio.
    gear=involute_spur_gear(44,2.5,6,bore=98,origin=(24,0,0),backlash=.12).val()
    gear=gear.cut(gear_keyway)
    add('AT_033_Involute_input_gear',gear,BRASS,'gears','44-tooth involute-profile input ring; flanks sampled by the original CYBR GEO gear helper',('involute-gear','legacy-api','parametric-tooth-profile'),motion='cam',explode=(-15,0,0))
    add('AT_033a_Drive_gear_key',_box(5.8,3.1,2.3,(27,49,0),.15),STEEL,'gears','Separate fitted key engaging real keyways in drive sleeve and gear ring',('keyed-interface','fillet'),motion='cam',explode=(-15,0,8))
    pinion=involute_spur_gear(16,2.5,6,bore=6,origin=(24,75,0),phase=math.pi/16,backlash=.12).val()
    add('AT_034_Involute_adjustment_pinion',pinion,STEEL,'gears','16-tooth mating fine-adjustment pinion at 75 mm shaft spacing',('involute-gear','gear-ratio'),motion='pinion',explode=(-15,12,0))
    add('AT_035_Pinion_shaft',_axial_cylinder(-1,45,3,75,0),STEEL,'gears','Continuous pinion shaft passing through two bearing brackets',('turned-shaft',),motion='pinion',explode=(-15,12,0))
    knob=ring(12,0,-6,2).translate((0,75,0))
    for j in range(24):
        a=math.tau*j/24
        knob=knob.cut(_axial_cylinder(-8,12,.85,75+12*math.cos(a),12*math.sin(a)))
    add('AT_036_Fine_adjustment_knob',knob,BLACK,'gears','Fluted input knob, mechanically locked to pinion shaft',('circular-pattern','boolean-csg'),motion='pinion',explode=(-22,12,0))
    for i,x in enumerate((14,38)):
        bracket=_box(7,28,21,(x,67.8,0),2)
        bracket=bracket.cut(_axial_cylinder(x-5,10,4.6,75,0))
        add(f'AT_037_Pinion_bearing_bracket_{i+1}',bracket,BLACK,'gear_support','Housing-mounted pinion bearing bracket, away from rotating teeth',('fillet','bearing-bore'),explode=(0,15,0))
        sleeve=ring(4.55,3.1,x-4,x+4).translate((0,75,0))
        add(f'AT_038_Pinion_bushing_{i+1}',sleeve,BRASS,'gear_support','Nominal running clearance sleeve for pinion shaft',('annular-fit',),explode=(0,15,0))
    guard=_box(12,50,51,(27,77,0),5).cut(_box(15,45,46,(27,77,0),4))
    # Opening toward the driven ring provides an actual tooth-mesh passage.
    guard=guard.cut(_box(18,16,33,(27,52,0)))
    add('AT_039_Pinion_guard',guard,BLUE,'gear_guard','Open-face peripheral pinion guard with ring-mesh clearance',('fillet','hollow-shell','boolean-csg'),explode=(0,25,0))

    # Revolved handwheel with individually cut, geometric grip flutes.
    wheel = _revolve_profile([(85,52),(100,52),(100,61),(98.8,66),(96.5,68),(88,68),(85,65)])
    for j in range(64):
        a = math.tau*j/64
        wheel = wheel.cut(_axial_cylinder(83,19,1.50,68.05*math.cos(a),68.05*math.sin(a)))
    add('AT_040_Revolved_handwheel',wheel,BLACK,'handwheel','Manual cam actuator; 64 real scalloped grip flutes',('revolve','stepped-profile','circular-pattern','boolean-csg'),motion='cam',explode=(10,0,0))
    add('AT_041_Handwheel_accent_band',ring(67.85,67.1,91.2,93.6),BLUE,'handwheel','Anodized inset accent band',('annulus',),motion='cam',explode=(10,0,0))
    groove = toroidal_groove(58.1,.85,center=(101,0,0))
    cam = ring(60,19,100,107).cut(groove)
    for a in JAW_ANGLES:
        cam = cam.cut(_spiral_slot(a))
    add('AT_050_Three_spiral_cam',cam,BRASS,'cam','Three normal-offset Archimedean cam slots; all jaws obey the same exact radial law',('spiral-spline','offset-curve','boolean-csg','toroidal-gland'),motion='cam',explode=(27,0,0),tolerance=.035)
    add('AT_051_Annular_seal',toroidal_groove(58.1,.76,center=(101,0,0)),POLYMER,'cam','Elastomer ring seated in toroidal cam gland',('analytic-torus','seal-fit'),motion='cam',explode=(27,0,0),angular=.15)

    # Stationary guide face: three genuine stepped prismatic slideways.
    front = cq.Workplane(obj=ring(76,19,109,119)).edges().chamfer(.5).val()
    for a in JAW_ANGLES:
        lower = _rotate(_box(6.0,52,12.5,(113,41,0)),a)
        upper = _rotate(_box(7,52,10.4,(118.5,41,0)),a)
        pin_slot = _rotate(_box(12,31,6.6,(110,42,0)),a)
        front = front.cut(lower).cut(upper).cut(pin_slot)
    for a in JAW_ANGLES:
        window=sector(63,28,107,122,a+math.radians(22),a+math.radians(98))
        window=cq.Workplane(obj=window).edges('|X').fillet(2.6).val()
        front=front.cut(window)
    for a in np.arange(6)*math.tau/6:
        a += math.radians(30)
        front = front.cut(_axial_cylinder(107,15,1.65,71.5*math.cos(a),71.5*math.sin(a)))
    for a in np.arange(6)*math.tau/6:
        front = front.cut(_axial_cylinder(107,15,4.4,68.5*math.cos(a),68.5*math.sin(a)))
    add('AT_060_Precision_guide_face',front,SILVER,'face','Stationary faceplate with three stepped anti-lift slideways and through follower slots',('chamfer','boolean-csg','prismatic-guide','radial-pattern'),explode=(49,0,0))
    for i,a in enumerate(np.arange(6)*math.tau/6+math.radians(30)):
        add(f'AT_061_Face_screw_{i+1}',_socket_screw(104,15,71.5*math.cos(a),71.5*math.sin(a)),STEEL,'face_fasteners','Six screws seated in front guide face and penetrating the support posts',('socket','pattern'),explode=(58,0,0))
    # Axial standoffs connect front face to rear housing, clear of cam slots.
    for i,a in enumerate(np.arange(6)*math.tau/6+math.radians(30)):
        standoff = _axial_cylinder(83,26,3.1,71.5*math.cos(a),71.5*math.sin(a))
        standoff = standoff.cut(_axial_cylinder(82,29,1.4,71.5*math.cos(a),71.5*math.sin(a)))
        add(f'AT_062_Guide_standoff_{i+1}',standoff,STEEL,'standoffs','Thread-pilot standoff on the guide fastener circle',('axial-bore',),explode=(7,0,0))
    cage=ring(76,57.2,81.8,84.6)
    for a in np.arange(6)*math.tau/6+math.radians(30):
        cage=cage.cut(_axial_cylinder(80,7,1.4,71.5*math.cos(a),71.5*math.sin(a)))
    add('AT_063_Stationary_cage_flange',cage,BLACK,'standoffs','Peripheral stationary flange supports guide posts outside the rotating handwheel',('annulus','bolt-circle'),explode=(-5,0,0))

    # Moving parts are rotated into three radial coordinate frames once; their
    # animation is a pure radial translation.  Pads remain at 120 degrees.
    for i,a in enumerate(JAW_ANGLES):
        motion=f'jaw:{i}'
        exp=(61,18*math.cos(a),18*math.sin(a))
        rail = _box(4.8,25,11.8,(113,35,0),.38).fuse(_box(8,25,9.8,(118,35,0),.4))
        rail = rail.cut(_axial_cylinder(109,20,1.9,36,0))
        add(f'AT_070_Jaw_slide_{i+1}',_rotate(rail,a),STEEL,'jaws',f'Jaw {i+1} stepped slider in the matching guide pocket',('anti-lift-guide','fillet','bore'),motion=motion,explode=exp)
        # An asymmetric loft creates each outward-sweeping finger.
        finger = lofted_solid([(121,12,5,32,0),(128,11,5.8,29,0),(139,8.4,6,25.5,0),(146,7.5,5.7,24.5,0)])
        finger = finger.cut(_box(31,11,20,(138,12.0,0)))
        for yy in (26,39):
            finger=finger.cut(_axial_cylinder(118,35,1.55,yy,0))
        add(f'AT_071_Freeform_jaw_{i+1}',_rotate(finger,a),BLACK,'jaws','Spline-lofted replaceable jaw carrier',('asymmetric-loft','boolean-csg','pattern'),motion=motion,explode=exp)
        pad=_box(14,4.5,10.7,(140,16.25,0),.5)
        # Shallow concentric real ribs improve visual readability of the pads.
        for xx in (135,138,141,144):
            pad=pad.cut(_box(.45,1.0,13,(xx,14.1,0)))
        add(f'AT_072_PEEK_contact_pad_{i+1}',_rotate(pad,a),CERAMIC,'jaws','Replaceable soft pad; closed tangential contact plane at radius 14 mm',('fillet','linear-pattern','soft-contact'),motion=motion,explode=exp)
        pin = _axial_cylinder(99.6,9,3,36,0).fuse(_axial_cylinder(108.6,16,1.8,36,0))
        add(f'AT_073_Cam_follower_{i+1}',_rotate(pin,a),STEEL,'followers','Ground follower head inside spiral slot; shank retained by slider',('stepped-pin','cam-follower'),motion=motion,explode=(40,18*math.cos(a),18*math.sin(a)))
        for j,yy in enumerate((26,39)):
            screw = _socket_screw(120,8,yy,0,shaft=1.40,head=2.4)
            add(f'AT_074_Jaw_screw_{i+1}_{j+1}',_rotate(screw,a),STEEL,'jaws','Carrier retention screw aligned to bored jaw interface',('socket','pattern'),motion=motion,explode=exp)

    # Rear service cartridge.  Internal porous field geometry is intentionally
    # delivered as a watertight mesh, alongside the analytic interface frame.
    tray=_box(44,5,25,(45,-62,4),2)
    tray=tray.cut(_box(36,9,18,(45,-62,4),2))
    add('AT_080_Service_cartridge_frame',tray,BLACK,'cartridge','Removable service cartridge surrounding the field-geometry insert',('fillet','boolean-csg'),explode=(0,-27,0))
    v,f=gyroid_sheet_mesh((28,62,-63.5,-55.5,-4,12),resolution=(85,30,48),periods=(2.6,.65,1.35),thickness=.34)
    parts.append(mesh_part('AT_081_Implicit_gyroid_insert',v,f,TITANIUM,group='implicit',
                          role='Removable porous service / acoustic insert; not flow or acoustic optimized',
                          provenance='designed-concept',explode=np.array((0,-30,0)),
                          tags=('technique:implicit-tpms','technique:scalar-field','technique:flying-edges')))
    print(f'[{len(parts):03}] AT_081_Implicit_gyroid_insert: {len(f):,} triangles',flush=True)
    for i,x in enumerate((25,65)):
        # Radial fasteners at the sides of the cartridge.
        screw=_socket_screw(0,5,0,0).rotate((0,0,0),(0,0,1),-90).translate((x,-62,4))
        add(f'AT_082_Cartridge_screw_{i+1}',screw,STEEL,'cartridge','Removable cartridge retention screw',('pattern',),explode=(0,-35,0))

    # A true 3D BREP sweep routed clear of the input wheel.  This is a optional
    # sensor-cable envelope, without any claim of a complete electronic system.
    cable_path=[(8,-22,24),(-8,-25,28),(-19,-42,18),(-13,-60,-10),(7,-67,-31),(35,-68,-54),(44,-68,-65),(47,-65,-76)]
    add('AT_090_Spline_service_conduit',spline_sweep_tube(cable_path,2.35),POLYMER,'service',
        'Actual spline-swept service-cable jacket, terminating at two strain reliefs',('3d-spline-sweep','routing'),explode=(-18,-10,0),tolerance=.08,angular=.18)
    # Original helper used the wrong profile origin; this model exercises the
    # corrected continuous helix, not a segmented-cylinder approximation.
    helix=helical_sweep(x0=0,length=14,helix_radius=3.2,pitch=3.5,section_radius=.55)
    helix=helix.rotate((0,0,0),(0,1,0),45).translate((34,-68,-54))
    add('AT_091_Continuous_helical_strain_relief',helix,STEEL,'service','True continuous swept-helix strain-relief spring',('true-helical-sweep','brep-sweep'),explode=(-18,-10,0),tolerance=.07,angular=.20)
    connector=drafted_cylinder(-1,10,6,3).translate((0,-22,24))
    connector=connector.cut(_axial_cylinder(-3,15,2.55,-22,24))
    add('AT_092_Drafted_connector',connector,BLACK,'service','Drafted service connector around the spline conduit root',('drafted-extrusion','boolean-csg'),explode=(-18,-10,0))
    dock=_box(15,12,10,(47,-63,-75),1.0)
    dock=dock.cut(cq.Solid.makeCylinder(2.7,16,cq.Vector(47,-65,-82),cq.Vector(0,0,1)))
    add('AT_094_Base_cable_dock',dock,BLACK,'service','Cable termination dock connected to the base',('fillet','bore'),explode=(0,-10,0))
    # Three small insulated conductors demonstrate fine mesh parallel transport.
    for i in range(3):
        p=np.asarray([(6,-24+i*2,24),(-.5,-24+i*2,25),(-5,-24+i*2,27)])
        v,f=tube_mesh(p,.32,12)
        parts.append(mesh_part(f'AT_093_Conductor_{i+1}',v,f,COPPER,group='service',
                              role='Short explicit copper conductor inside connector, mesh routing',provenance='designed-concept',
                              explode=np.array((-18,-10,0)),tags=('technique:parallel-transport-tube','technique:fine-mesh')))

    # Geometry text, fully represented in the mesh and CAD exports.
    badge=_box(36,1.1,11,(48,-63.25,24),.5)
    add('AT_100_Identification_badge',badge,BLACK,'markings','Recessed identification plaque',('fillet',))
    plane=cq.Plane(origin=(48,-63.9,24),xDir=(1,0,0),normal=(0,-1,0))
    letters=cq.Workplane(plane).text('ATLAS',6.2,.16,font='DejaVu Sans',kind='bold',halign='center',valign='center',combine=False).val()
    add('AT_101_Geometric_brand_mark',letters,INK,'markings','Extruded vector lettering, not a composited image label',('text-extrusion',),tolerance=.04,angular=.18)
    # Fine engraved-style scale bars on the stationary front face.
    for i in range(45):
        a=math.radians(210+i*2)
        length=3.0 if i%5==0 else 1.55
        tick=_rotate(_box(.10,length,.30,(119.07,66.0,0)),a)
        add(f'AT_102_Radial_scale_{i+1:02}',tick,INK,'markings','Nominal geometric dial graduation',('radial-pattern',),explode=(49,0,0))

    common=dict(focal_length_mm=68,sensor_width_mm=36,f_stop=11,environment_strength=.26,
                background_strength=.65,light_size=1.45,light_intensity=1.12,floor_gap_mm=0,
                floor_roughness=.85,exposure=1.06)
    views={
        'hero':View(az=-39,el=24,target=(57,0,-8),scale=128,camera_distance_mm=790,
                    title='ATLAS / SELF-CENTERING INSPECTION FIXTURE',**common),
        'front':View(az=-12,el=11,target=(100,0,0),scale=89,camera_distance_mm=600,
                     title='THREE JAWS / ONE GEOMETRIC CONSTRAINT',**common),
        'cutaway':View(az=-27,el=24,target=(76,0,-7),scale=114,camera_distance_mm=720,
                       hide=('housing','face','face_fasteners','markings','handwheel'),
                       title='SPIRAL CAM / GUIDED FOLLOWERS / SUPPORT',**common),
        'exploded':View(az=-34,el=27,target=(76,-2,-9),scale=166,camera_distance_mm=975,explode=1,
                        title='ATLAS / EXPLODED ASSEMBLY',**common),
        'cam_macro':View(az=-6,el=12,target=(106,0,1),scale=69,camera_distance_mm=465,
                         hide=('housing','face','face_fasteners','markings','handwheel','service','cartridge','implicit','base','feet','support','lattice','drive','standoffs'),
                         title='REAL SPIRAL SLOTS / CONSTRAINT-DRIVEN JAWS',**common),
        'materials_macro':View(az=-75,el=12,target=(47,-41,-12),scale=73,camera_distance_mm=450,
                               title='FREEFORM BREP / TPMS / ANALYTIC LATTICE',**common),
        'gear_macro':View(az=147,el=17,target=(26,24,2),scale=92,camera_distance_mm=565,
                         hide=('housing','gear_guard','service','markings','handwheel','face','face_fasteners','jaws','followers','standoffs','cam','base','feet','support','lattice','cartridge','implicit','drive','bearings'),
                         title='44:16 INVOLUTE INPUT / TRUE SHAFT CONNECTIONS',**common),
        'lattice_macro':View(az=-48,el=21,target=(54,0,-72),scale=39,camera_distance_mm=265,
                            hide=tuple(sorted({p.group for p in parts}-{'lattice','base','feet'})),
                            title='ANALYTIC BCC SUPPORT / ISOLATED INSPECTION',**common),
        'engineering':View(az=-35,el=25,target=(59,0,-10),scale=126,projection='orthographic',floor=False),
    }
    metadata={
        'truth_intent':'concept','design':asdict(SPEC),'geometry_benchmark':True,
        'description':'A manually actuated self-centering fixture for small-part inspection and assembly.',
        'geometry_techniques':sorted({t.split(':',1)[1] for p in parts for t in p.tags if t.startswith('technique:')}),
        'motion_model':'Prescribed exact Archimedean cam kinematics. No motor, friction, force, contact solver or load capacity is claimed.',
        'interface_notes':[
            'Nominal 28–48 mm aperture measured between the three tangent pad planes.',
            'Follower radius 3 mm in a 3.18 mm normal-offset cam slot: nominal 0.18 mm flank clearance.',
            'Follower shanks are guided by the stationary face; jaws move only radially.',
            'Fine input: 44/16 teeth, module 2.5 mm, 75 mm centers; prescribed conjugate rotation with sampled involute flanks and unqualified generated root transitions.',
            'Static pose is unloaded and contains no workpiece; motion never clips through an inserted specimen.',
            'BCC support is an intersecting analytic compound. The gyroid insert is native field geometry and omitted from analytic STEP.',
            'Service conduit/connector are original packaging envelopes, not functional sensor electronics.',
            'No friction, clamp force, fatigue, manufacturing tolerance, acoustic or filtration performance is established.',
        ],
        'drawing_groups':['base','housing','face'],
    }
    return Assembly('atlas_fixture',parts,MATERIALS,views,metadata,pose)


__all__=['SPEC','JAW_ANGLES','build','pose','cam_angle','jaw_travel','spiral_centerline']
