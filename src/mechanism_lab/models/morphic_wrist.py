"""CYBR Morphic Wrist: a modern-geometry benchmark assembly.

This is an original 2-DOF biomimetic wrist/forearm cartridge intended to stress
Mechanism Lab's geometry pipeline rather than inflate complexity with repeated
gears.  The neutral assembly deliberately mixes analytic OpenCascade BREP and
high-resolution routed wire meshes.  It is a concept / geometry benchmark, not a
load-qualified medical, prosthetic, or industrial actuator.

Showcased geometry includes:
- multi-section freeform BREP lofts and hollow lofted shells
- 3D interpolation-spline BREP sweeps and conformal routed channels
- true helical OpenCascade sweeps
- drafted extrusions
- subtractive CSG windows, bores, patterned vents and conformal channels
- BREP BCC additive-manufacturing lattice
- revolved axisymmetric encoder / seal geometry
- toroidal gland geometry
- ribs / gussets, chamfers, fillets and patterned fasteners
- mixed analytic CAD and fine continuous-wire mesh geometry
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import cadquery as cq
import numpy as np

from ..advanced_geometry import (
    bcc_lattice,
    drafted_cylinder,
    helical_sweep,
    lofted_elliptic_shell,
    lofted_solid,
    spline_sweep_tube,
    toroidal_groove,
)
from ..core import Assembly, Material, View, cad_part, mesh_part
from ..geometry import tube_mesh


@dataclass(frozen=True)
class WristSpec:
    shell_length_mm: float = 118.0
    joint_center_x_mm: float = 128.0
    output_length_mm: float = 48.0
    shell_wall_mm: float = 2.4
    sma_wire_diameter_mm: float = 0.20
    sma_fibers_per_bank: int = 8
    sma_banks: int = 4
    lattice_strut_mm: float = 0.95
    yaw_bore_mm: float = 12.0
    pitch_bore_mm: float = 10.0


SPEC = WristSpec()


MATERIALS = [
    Material('Black hard-anodized aluminium', (0.022, 0.027, 0.034), .68, .27,
             ior=1.48, coat=.10, coat_rough=.19, anisotropy=.08,
             microfinish='anodized', material_source='original benchmark finish'),
    Material('Bead-blasted aluminium', (0.46, 0.50, 0.55), .88, .23,
             ior=1.46, coat=.05, coat_rough=.18, anisotropy=.08,
             microfinish='bead-blasted', material_source='original benchmark finish'),
    Material('Titanium alloy', (0.37, 0.40, 0.44), .93, .19,
             ior=1.52, coat=.02, coat_rough=.17, anisotropy=.12,
             microfinish='machined', material_source='original benchmark material assignment'),
    Material('Ground stainless steel', (0.55, 0.58, 0.61), .96, .15,
             ior=1.50, coat=.025, coat_rough=.14, anisotropy=.20,
             microfinish='turned', material_source='original benchmark material assignment'),
    Material('Nitinol actuator wire', (0.42, 0.46, 0.50), .94, .21,
             ior=1.50, coat=.015, coat_rough=.19,
             microfinish='drawn-wire', material_source='0.20 mm source-guided concept wire'),
    Material('Copper conductor / bus', (0.69, 0.27, 0.075), .96, .18,
             ior=1.50, coat=.02, coat_rough=.16,
             microfinish='copper-wire', material_source='original benchmark conductor routing'),
    Material('Bronze bearing / guide', (0.52, 0.31, 0.10), .82, .24,
             ior=1.48, coat=.02, coat_rough=.20,
             microfinish='machined', material_source='original benchmark bearing assignment'),
    Material('PEEK structural polymer', (0.19, 0.14, 0.075), .02, .37,
             ior=1.62, coat=.05, coat_rough=.25,
             microfinish='polymer', material_source='original benchmark polymer assignment'),
    Material('Elastomer overmold', (0.018, 0.021, 0.024), .0, .62,
             ior=1.48, coat=.0, coat_rough=.45,
             microfinish='polymer', material_source='original benchmark overmold'),
    Material('PCB substrate', (0.025, 0.12, 0.075), .0, .39,
             ior=1.55, coat=.08, coat_rough=.25,
             microfinish='polymer', material_source='original benchmark electronics'),
    Material('Ceramic electrical isolator', (0.71, 0.70, 0.64), .0, .28,
             ior=1.70, coat=.12, coat_rough=.16,
             microfinish='none', material_source='original benchmark isolator'),
]


def _soft(shape, r=.35):
    return cq.Workplane(obj=shape).edges().fillet(r).val()


def _cylinder_x(x0, x1, y, z, r):
    return cq.Workplane('YZ', origin=(x0, y, z)).circle(r).extrude(x1 - x0).val()


def _annulus_x(x0, x1, y, z, ro, ri):
    return cq.Workplane('YZ', origin=(x0, y, z)).circle(ro).circle(ri).extrude(x1 - x0).val()


def _radial_cylinder(center, direction, radius, length):
    return cq.Solid.makeCylinder(radius, length, cq.Vector(*center), cq.Vector(*direction))


def _socket_fastener_x(x0, length, y, z, shaft=1.55, head=2.8):
    sh = _cylinder_x(x0, x0 + length, y, z, shaft)
    hd = _cylinder_x(x0 - 2.5, x0, y, z, head)
    socket = cq.Workplane('YZ', origin=(x0 - 2.7, y, z)).polygon(6, 2.15).extrude(1.15).val()
    return _soft(sh.fuse(hd).cut(socket), .14)


def _mesh_wire(name, points, radius, material, group, role, provenance='designed-concept', sides=12):
    v, f = tube_mesh(points, radius, sides)
    return mesh_part(name, v, f, material, group=group, role=role, provenance=provenance,
                     tags=('technique:mixed-mesh-routing',))


def _revolved_encoder_hub():
    # Profile lives in the XY plane and is revolved around world X.
    return (
        cq.Workplane('XY')
        .moveTo(139.0, 7.4)
        .lineTo(143.0, 7.4)
        .lineTo(145.5, 9.3)
        .lineTo(150.0, 9.3)
        .lineTo(150.0, 12.2)
        .lineTo(139.0, 12.2)
        .close()
        .revolve(360.0, (0, 0), (1, 0))
        .val()
    )


def build() -> Assembly:
    parts = []

    def add_cad(name, shape, material, group, role, *, tags=(), provenance='designed-concept', explode=(0, 0, 0), tolerance=.030, angular=.050):
        parts.append(cad_part(name, shape, material, tolerance=tolerance, angular=angular,
                              group=group, role=role, provenance=provenance,
                              explode=np.asarray(explode, float), tags=tuple(tags)))

    # ------------------------------------------------------------------
    # 1. Freeform hollow exoskeleton: true multi-section BREP loft + CSG.
    # ------------------------------------------------------------------
    shell_sections = [
        (0.0, 36.0, 31.0, 0.0, 0.0),
        (24.0, 38.0, 32.5, 0.8, 1.0),
        (58.0, 36.5, 31.0, -0.5, 1.8),
        (92.0, 34.0, 29.0, 0.7, 0.5),
        (118.0, 31.0, 27.0, 0.0, 0.0),
    ]
    shell = lofted_elliptic_shell(shell_sections, SPEC.shell_wall_mm)

    # Four large inspection/lightening windows: real subtractive BREP booleans.
    for cut in (
        cq.Workplane('XY').box(56, 28, 19).translate((61, 0, 27)).val(),
        cq.Workplane('XY').box(56, 28, 19).translate((61, 0, -27)).val(),
        cq.Workplane('XY').box(56, 18, 35).translate((61, 34, 0)).val(),
        cq.Workplane('XY').box(56, 18, 35).translate((61, -34, 0)).val(),
    ):
        shell = shell.cut(cut)

    # Patterned radial vents on the proximal and distal shell bands.
    for x in (20.0, 103.0):
        for j in range(8):
            a = math.tau * (j + .5) / 8
            y, z = 38.0 * math.cos(a), 33.0 * math.sin(a)
            d = (0.0, -math.cos(a), -math.sin(a))
            shell = shell.cut(_radial_cylinder((x, y, z), d, 1.8, 12.0))

    # Two conformal spline channels cut through the shell wall.
    coolant_paths = [
        [(10, 30.5, 7), (35, 32.8, 11), (67, 31.8, 14), (96, 28.8, 10), (112, 26.5, 5)],
        [(10, -30.5, -7), (35, -32.8, -11), (67, -31.8, -14), (96, -28.8, -10), (112, -26.5, -5)],
    ]
    for p in coolant_paths:
        shell = shell.cut(spline_sweep_tube(p, 2.05))
    add_cad('MW_01_Freeform_exoskeleton', shell, 0, 'shell',
            'Hollow asymmetric freeform forearm exoskeleton with patterned windows, vents and conformal channels',
            tags=('technique:loft','technique:hollow-shell','technique:boolean-csg','technique:pattern','technique:conformal-channel'),
            explode=(-16, 0, 0))

    # Channel liners stay analytic BREP spline sweeps instead of mesh tubes.
    for i, p in enumerate(coolant_paths):
        add_cad(f'MW_02_Conformal_channel_liner_{i+1:02}', spline_sweep_tube(p, 1.25), 5, 'thermal',
                'Conformal routed copper thermal/electrical service liner',
                tags=('technique:3d-spline-sweep','technique:conformal-routing'), explode=(0, (i*2-1)*15, 0))

    # ------------------------------------------------------------------
    # 2. Additive lattice + organic central load spine.
    # ------------------------------------------------------------------
    lattice = bcc_lattice(origin=(25, -14, -11), cells=(3, 2, 2), pitch=(19, 14, 11),
                          strut_radius=SPEC.lattice_strut_mm, node_radius=1.15)
    add_cad('MW_03_BCC_lattice_core', lattice, 2, 'lattice',
            'Analytic BREP body-centred-cubic load-spreading lattice insert',
            tags=('technique:brep-lattice','technique:additive-manufacturing'), explode=(0, 0, 25), tolerance=.040)

    spine = lofted_solid([
        (12, 13.5, 9.5, 0, 0),
        (42, 16.0, 11.0, 0, 0),
        (76, 14.5, 10.0, 0, 1.0),
        (108, 11.5, 9.0, 0, 0),
    ])
    # Long central relief plus asymmetric side pockets make this an organic ribbed member.
    spine = spine.cut(_cylinder_x(8, 112, 0, 0, 5.8))
    for yy in (-11.0, 11.0):
        spine = spine.cut(cq.Workplane('XY').box(54, 9, 13).translate((62, yy, 0)).val())
    add_cad('MW_04_Organic_load_spine', spine, 1, 'structure',
            'Lofted central load spine with subtractive relief pockets',
            tags=('technique:freeform-loft','technique:boolean-csg','technique:ribbed-structure'), explode=(0, 0, -24))

    # Four tapered structural ribs from the spine toward the shell.
    for i, a in enumerate((math.pi/4, 3*math.pi/4, 5*math.pi/4, 7*math.pi/4)):
        y, z = 19*math.cos(a), 15*math.sin(a)
        rib = cq.Workplane('YZ', origin=(64, 0, 0)).moveTo(-2.3, -2.0).lineTo(y, z).lineTo(y*.72, z*.72).close().extrude(6, taper=4).val()
        add_cad(f'MW_05_Drafted_rib_{i+1:02}', rib, 1, 'structure',
                'Drafted load-transfer rib', tags=('technique:drafted-extrusion','technique:gusset-rib'))

    # ------------------------------------------------------------------
    # 3. Service collar: draft + true helix + toroidal gland.
    # ------------------------------------------------------------------
    collar = drafted_cylinder(2.0, 16.0, 34.2, -2.2).cut(_cylinder_x(1, 20, 0, 0, 31.3))
    gland = toroidal_groove(31.8, 1.05, center=(13.0, 0, 0), axis=(1, 0, 0))
    collar = collar.cut(gland)
    add_cad('MW_06_Drafted_service_collar', collar, 2, 'service',
            'Drafted service collar with analytic toroidal gland',
            tags=('technique:draft','technique:torus','technique:revolved-gland'), explode=(-18, 0, 0))

    helix_ridge = helical_sweep(x0=3.0, length=13.0, helix_radius=34.35, pitch=3.2, section_radius=.62)
    add_cad('MW_07_Helical_service_thread', helix_ridge, 3, 'service',
            'True OpenCascade helical service-thread/ridge geometry',
            tags=('technique:true-helix','technique:brep-sweep'), explode=(-18, 0, 0), tolerance=.035)

    # ------------------------------------------------------------------
    # 4. Nested freeform wrist cradle / distal output architecture.
    # ------------------------------------------------------------------
    cradle_outer = lofted_solid([
        (106, 31, 27, 0, 0),
        (119, 36, 32, 0, 0),
        (132, 35, 31, 0, 0),
        (140, 29, 26, 0, 0),
    ])
    cradle_inner = lofted_solid([
        (108, 23, 19, 0, 0),
        (119, 27, 23, 0, 0),
        (132, 26, 22, 0, 0),
        (138, 22, 18, 0, 0),
    ])
    cradle = cradle_outer.cut(cradle_inner)
    yaw_bore = cq.Solid.makeCylinder(SPEC.yaw_bore_mm/2, 82, cq.Vector(126, 0, -41), cq.Vector(0, 0, 1))
    cradle = cradle.cut(yaw_bore)
    # Side optical/access windows leave a double-arch yoke rather than a simple ring.
    cradle = cradle.cut(cq.Workplane('XY').box(27, 46, 20).translate((125, 0, 25)).val())
    cradle = cradle.cut(cq.Workplane('XY').box(27, 46, 20).translate((125, 0, -25)).val())
    add_cad('MW_08_Freeform_yaw_cradle', cradle, 2, 'joint',
            'Nested freeform hollow yaw cradle with through-bearing bore and access arches',
            tags=('technique:nested-loft','technique:hollow-shell','technique:boolean-csg','technique:bearing-bore'), explode=(12, 0, 0))

    output = lofted_solid([
        (126, 20, 18, 0, 0),
        (140, 24, 20, 0, 0),
        (158, 27, 19, 0.5, 0),
        (176, 24, 16, 0, 0),
    ])
    output = output.cut(_cylinder_x(124, 178, 0, 0, 8.8))
    pitch_bore = cq.Solid.makeCylinder(SPEC.pitch_bore_mm/2, 72, cq.Vector(143, -36, 0), cq.Vector(0, 1, 0))
    output = output.cut(pitch_bore)
    # Palm-interface lightening windows.
    for zz in (-10, 10):
        output = output.cut(cq.Workplane('XY').box(24, 16, 9).translate((163, 0, zz)).val())
    add_cad('MW_09_Lofted_output_yoke', output, 1, 'output',
            'Asymmetric lofted distal output yoke with orthogonal pitch bore and lightening pockets',
            tags=('technique:freeform-loft','technique:orthogonal-bore','technique:lightweighting'), explode=(30, 0, 0))

    # Ground pivot pins and bronze bearing sleeves on orthogonal axes.
    yaw_pin = cq.Solid.makeCylinder(5.7, 76, cq.Vector(126, 0, -38), cq.Vector(0, 0, 1))
    add_cad('MW_10_Yaw_pivot_pin', yaw_pin, 3, 'joint', 'Ground yaw pivot pin', tags=('technique:turned-bearing-interface',), explode=(0, 0, 28))
    for i, z in enumerate((-24.0, 19.0)):
        sleeve = cq.Solid.makeCylinder(8.0, 5.0, cq.Vector(126, 0, z), cq.Vector(0, 0, 1)).cut(
            cq.Solid.makeCylinder(5.8, 5.5, cq.Vector(126, 0, z-.2), cq.Vector(0, 0, 1)))
        add_cad(f'MW_11_Yaw_bearing_{i+1:02}', sleeve, 6, 'bearings', 'Bronze yaw bearing sleeve', tags=('technique:precision-fit',))

    pitch_pin = cq.Solid.makeCylinder(4.7, 62, cq.Vector(143, -31, 0), cq.Vector(0, 1, 0))
    add_cad('MW_12_Pitch_pivot_pin', pitch_pin, 3, 'joint', 'Ground pitch pivot pin', tags=('technique:turned-bearing-interface',), explode=(0, 25, 0))
    for i, y in enumerate((-21.0, 16.0)):
        sleeve = cq.Solid.makeCylinder(7.0, 5.0, cq.Vector(143, y, 0), cq.Vector(0, 1, 0)).cut(
            cq.Solid.makeCylinder(4.8, 5.5, cq.Vector(143, y-.2, 0), cq.Vector(0, 1, 0)))
        add_cad(f'MW_13_Pitch_bearing_{i+1:02}', sleeve, 6, 'bearings', 'Bronze pitch bearing sleeve', tags=('technique:precision-fit',))

    # ------------------------------------------------------------------
    # 5. Revolved encoder hub and electronics architecture.
    # ------------------------------------------------------------------
    add_cad('MW_14_Revolved_encoder_hub', _revolved_encoder_hub(), 2, 'sensor',
            'Revolved multi-step magnetic encoder hub', tags=('technique:revolve','technique:stepped-profile'), explode=(18, 0, 0))
    encoder_ring = _annulus_x(146.0, 149.2, 0, 0, 15.0, 12.3)
    add_cad('MW_15_Encoder_magnet_ring', encoder_ring, 10, 'sensor', 'Encoder magnet/ceramic ring envelope', tags=('technique:annular-fit',))

    pcb = _soft(cq.Workplane('XY').box(2.0, 46, 27).translate((99, 0, -18)).val(), .7)
    add_cad('MW_16_Control_PCB', pcb, 9, 'electronics', 'Curated PCB substrate envelope mounted inside forearm', tags=('technique:electronics-integration',), explode=(0, -28, -10))
    for i, (yy, zz, mat) in enumerate(((-12,-18,5),(0,-18,10),(12,-18,5),(-7,-10,5),(7,-10,5))):
        chip = _soft(cq.Workplane('XY').box(2.6, 7.0, 5.0).translate((97.2, yy, zz)).val(), .35)
        add_cad(f'MW_17_PCB_component_{i+1:02}', chip, mat, 'electronics', 'PCB component / conductor package envelope', tags=('technique:electronics-integration',), explode=(0, -28, -10))

    # ------------------------------------------------------------------
    # 6. BREP routed tendons, flexures and strain-relieved cable paths.
    # ------------------------------------------------------------------
    tendon_paths = [
        [(34, 18, 12), (72, 20, 15), (106, 22, 17), (132, 20, 14), (154, 14, 10)],
        [(34,-18, 12), (72,-20, 15), (106,-22, 17), (132,-20, 14), (154,-14, 10)],
        [(34, 18,-12), (72, 20,-15), (106, 22,-17), (132, 20,-14), (154, 14,-10)],
        [(34,-18,-12), (72,-20,-15), (106,-22,-17), (132,-20,-14), (154,-14,-10)],
    ]
    for i, path in enumerate(tendon_paths):
        add_cad(f'MW_18_Spline_tendon_{i+1:02}', spline_sweep_tube(path, .78), 3, 'tendons',
                '3D spline-swept tendon/cable path', tags=('technique:3d-spline-sweep','technique:tendon-routing'), explode=(0, (i-1.5)*10, 0))

    flexure_paths = [
        [(112, 24, 8), (119, 31, 12), (128, 33, 8), (136, 27, 3)],
        [(112,-24, 8), (119,-31, 12), (128,-33, 8), (136,-27, 3)],
        [(112, 24,-8), (119, 31,-12), (128, 33,-8), (136, 27,-3)],
        [(112,-24,-8), (119,-31,-12), (128,-33,-8), (136,-27,-3)],
    ]
    for i, path in enumerate(flexure_paths):
        add_cad(f'MW_19_Spline_flexure_{i+1:02}', spline_sweep_tube(path, 1.15), 2, 'flexures',
                'Curved spline-swept preload/flexure member', tags=('technique:3d-spline-sweep','technique:compliant-member'))

    # ------------------------------------------------------------------
    # 7. Antagonistic SMA banks: fine mixed-mesh geometry with real routing.
    # ------------------------------------------------------------------
    bank_centers = [(0, 17), (0, -17), (17, 0), (-17, 0)]
    wire_r = SPEC.sma_wire_diameter_mm * .5
    for b, (by, bz) in enumerate(bank_centers):
        # Insulating anchor rails are analytic BREP, wires remain continuous fine meshes.
        anchor0 = _soft(_cylinder_x(21, 26, by, bz, 3.3), .22)
        anchor1 = _soft(_cylinder_x(105, 111, by*.86, bz*.86, 3.6), .22)
        add_cad(f'MW_20_SMA_fixed_anchor_{b+1:02}', anchor0, 7, 'sma_anchors', 'SMA bank fixed insulating anchor', tags=('technique:fiber-anchor',))
        add_cad(f'MW_21_SMA_distal_anchor_{b+1:02}', anchor1, 7, 'sma_anchors', 'SMA bank distal insulating anchor', tags=('technique:fiber-anchor',))
        for f in range(SPEC.sma_fibers_per_bank):
            o = (f - (SPEC.sma_fibers_per_bank-1)/2) * .48
            # Offset each fiber perpendicular to the dominant bank direction.
            if abs(by) > abs(bz):
                sy, sz = by, bz + o
                ey, ez = by*.86, bz*.86 + o*.75
            else:
                sy, sz = by + o, bz
                ey, ez = by*.86 + o*.75, bz*.86
            pts = [(26, sy, sz), (52, sy*1.03, sz*1.03), (82, ey*1.05, ez*1.05), (105, ey, ez)]
            parts.append(_mesh_wire(f'MW_22_SMA_B{b+1}_F{f+1:02}', pts, wire_r, 4, 'sma_fibers',
                                    'Continuous 0.20 mm antagonistic SMA concept fiber', provenance='source-guided-concept', sides=10))

    # Four copper bus routes are true BREP spline sweeps, distinct from fine fibers.
    bus_paths = [
        [(18, 22, 20), (35, 29, 22), (69, 28, 20), (93, 23, 18)],
        [(18,-22, 20), (35,-29, 22), (69,-28, 20), (93,-23, 18)],
        [(18, 22,-20), (35, 29,-22), (69, 28,-20), (93, 23,-18)],
        [(18,-22,-20), (35,-29,-22), (69,-28,-20), (93,-23,-18)],
    ]
    for i,p in enumerate(bus_paths):
        add_cad(f'MW_23_Copper_bus_{i+1:02}', spline_sweep_tube(p, .72), 5, 'electrical',
                'Analytic spline-routed copper bus / strain-relief path', tags=('technique:3d-spline-sweep','technique:electrical-routing'))

    # ------------------------------------------------------------------
    # 8. Distal elastomer overmold: second independent hollow loft family.
    # ------------------------------------------------------------------
    boot_outer = lofted_elliptic_shell([
        (139, 30.0, 25.0, 0, 0),
        (151, 31.0, 24.0, 0.5, 0),
        (166, 28.0, 20.0, 0, 0),
        (176, 25.5, 17.5, 0, 0),
    ], 1.8)
    # Split/cut the boot so it visually reads as a compliant protective half-shell.
    boot_outer = boot_outer.cut(cq.Workplane('XY').box(42, 70, 28).translate((158, 0, 24)).val())
    add_cad('MW_24_Lofted_elastomer_boot', boot_outer, 8, 'overmold',
            'Independent freeform hollow elastomer protective overmold',
            tags=('technique:loft','technique:hollow-shell','technique:multi-material-overmold'), explode=(25, 0, 18))

    # ------------------------------------------------------------------
    # 9. Patterned service fasteners and connector architecture.
    # ------------------------------------------------------------------
    for i in range(8):
        a = math.tau * i / 8
        y, z = 29.2*math.cos(a), 25.0*math.sin(a)
        add_cad(f'MW_25_Service_fastener_{i+1:02}', _socket_fastener_x(-1.5, 8.5, y, z), 3, 'fasteners',
                'Patterned socket-head service fastener', tags=('technique:pattern','technique:fastener-detail'))

    connector = cq.Workplane('YZ', origin=(74, -31, -7)).rect(15, 12).extrude(12, taper=5).val()
    connector = _soft(connector, .8)
    add_cad('MW_26_Drafted_connector_shell', connector, 7, 'connector', 'Drafted strain-relieved connector shell', tags=('technique:drafted-extrusion','technique:connector-detail'), explode=(0,-20,0))
    for i in range(6):
        yy = -35 + (i%3)*4
        zz = -10 + (i//3)*5
        add_cad(f'MW_27_Connector_pin_{i+1:02}', _cylinder_x(84, 88, yy, zz, .72), 5, 'connector', 'Copper connector contact pin', tags=('technique:pattern','technique:electrical-routing'))

    views = {
        'hero': View(
            az=218, el=18, scale=86, target=(91, 0, 0),
            focal_length_mm=62, sensor_width_mm=36, camera_distance_mm=410,
            f_stop=5.6, environment_strength=.20, background_strength=.80,
            light_size=1.55, light_intensity=1.0, floor_gap_mm=1.5, floor_roughness=.88, exposure=.97,
            title='CYBR MORPHIC WRIST / MODERN GEOMETRY BENCHMARK',
            note='Freeform BREP shell / lattice / spline routing / helical service geometry / antagonistic SMA banks',
        ),
        'cutaway': View(
            az=220, el=16, scale=78, target=(91, 0, 0),
            focal_length_mm=66, sensor_width_mm=36, camera_distance_mm=385,
            f_stop=6.3, environment_strength=.21, background_strength=.78,
            light_size=1.60, light_intensity=1.02, floor_gap_mm=1.5, floor_roughness=.90, exposure=1.0,
            hide=('shell','overmold'),
            title='MORPHIC WRIST / INTERNAL ARCHITECTURE',
            note='Analytic lattice, load spine, orthogonal joint support, routed tendons, SMA banks and electronics',
        ),
        'geometry_macro': View(
            az=205, el=13, scale=49, target=(125, 8, 3),
            focal_length_mm=78, sensor_width_mm=36, camera_distance_mm=270,
            f_stop=5.0, environment_strength=.22, background_strength=.76,
            light_size=1.70, light_intensity=1.05, floor=False, exposure=1.02,
            hide=('shell','overmold','electronics'),
            title='FREEFORM JOINT / ROUTING MACRO',
            note='Nested lofted cradle, spline tendons/flexures, precision sleeves and revolved encoder hub',
        ),
        'lattice_macro': View(
            az=232, el=21, scale=42, target=(55, 0, 0),
            focal_length_mm=82, sensor_width_mm=36, camera_distance_mm=250,
            f_stop=5.6, environment_strength=.23, background_strength=.75,
            light_size=1.72, light_intensity=1.05, floor=False, exposure=1.03,
            hide=('shell','overmold','sma_fibers'),
            title='BREP LATTICE / ORGANIC SPINE',
            note='Analytic OpenCascade BCC lattice plus lofted load spine and drafted structural ribs',
        ),
        'exploded': View(
            az=220, el=20, scale=128, target=(88, 0, 0), explode=1.0,
            focal_length_mm=68, sensor_width_mm=36, camera_distance_mm=560,
            f_stop=9.0, environment_strength=.22, background_strength=.80,
            light_size=1.55, light_intensity=1.0, floor=False, exposure=.98,
            title='MORPHIC WRIST / EXPLODED GEOMETRY STUDY',
            note='Exploded offsets are visualization-only; this is not an assembly procedure',
        ),
        'engineering': View(
            az=45, el=18, scale=120, target=(90,0,0), projection='orthographic', floor=False,
            title='MORPHIC WRIST / ORTHOGRAPHIC INSPECTION', note='Original concept / units mm',
        ),
    }

    technique_manifest = [
        'multi-section freeform BREP loft',
        'hollow lofted shell / subtractive wall construction',
        '3D interpolation-spline BREP sweep',
        'true OpenCascade helical sweep',
        'drafted extrusion',
        'boolean CSG windows / pockets / bores',
        'patterned radial vents / fasteners',
        'conformal routed channels',
        'analytic BREP BCC lattice',
        'revolved stepped profile',
        'analytic toroidal gland',
        'precision bearing interfaces',
        'organic lofted load path / ribs',
        'mixed analytic BREP and continuous fine-wire mesh',
        'multi-material overmold architecture',
        'electronics / connector integration',
    ]
    metadata = {
        'truth_intent': 'concept',
        'design': asdict(SPEC),
        'geometry_benchmark': True,
        'geometry_techniques': technique_manifest,
        'fidelity': 'Original geometry benchmark assembly. No vendor or anatomical hardware is claimed.',
        'motion_model': 'Neutral static geometry benchmark. No SMA contraction or 2-DOF motion is faked in this revision.',
        'assumptions': [
            'The freeform shell, cradle, load spine, lattice, tendon routes, flexures, overmold and service geometry are original design geometry.',
            'Nitinol wire diameter is source-guided, but force, thermal response, fatigue, hysteresis and joint torque are not solved here.',
            'Bearing, seal, thread, connector and electronics envelopes are concept geometry and are not tolerance-qualified vendor reproductions.',
            'The BCC lattice is an analytic CAD benchmark; no topology optimization, stress sizing or additive-process qualification is claimed.',
        ],
        'safety': [
            'Do not fabricate or load this concept without independent structural, thermal, electrical, bearing, fastener and fatigue analysis.',
            'Guard hot SMA fibers and moving joints in any later physical prototype.',
        ],
    }
    return Assembly('morphic_wrist', parts, MATERIALS, views, metadata=metadata)


__all__ = ['SPEC', 'MATERIALS', 'build']
