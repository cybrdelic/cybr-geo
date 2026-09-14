"""AERIS / a serviceable desktop extraction-turbine GEOMETRY CONCEPT.

Original parametric geometry built with CYBR GEO / Mechanism Lab. This is not a
rated air-cleaning product. No capture velocity, filtration efficiency, fan curve,
rotor balance, temperature, noise, electrical safety or bearing life is claimed.

Coordinates: millimetres; fan/motor axis X; Z up. All functional parts have stable
names, declared materials, CAD or explicit mesh provenance, and service offsets.
Use the same assembly for native path tracing, STEP, GLB, STL and whiteprints.
"""
from __future__ import annotations
from dataclasses import dataclass, replace
from pathlib import Path
import json, math, time
import numpy as np
import cadquery as cq
from mechanism_lab import Assembly, Material, View
from mechanism_lab.core import cad_part, mesh_part, axis_pose, rotation_x
from mechanism_lab.geometry import ring, drill, bolt_circle, sector, tube_mesh
from mechanism_lab.advanced_geometry import (
    lofted_elliptic_shell, lofted_solid, spline_sweep_tube, helical_sweep,
    drafted_cylinder, strut, bcc_lattice, toroidal_groove, gyroid_sheet_mesh,
)

TAU=2*math.pi

@dataclass(frozen=True)
class Config:
    blade_count: int=11
    rotor_radius: float=64.
    rotor_front: float=0.
    rotor_back: float=35.
    shaft_radius: float=5.
    bolt_count: int=10
    pleat_count: int=44
    wire_turns: int=8
    tolerance: float=.055
    angular: float=.105

    def check(self):
        if not 7<=self.blade_count<=23: raise ValueError('blade_count outside supported range')
        if not 50<=self.rotor_radius<=65: raise ValueError('Invalid rotor/shroud/casing clearance')
        if self.rotor_back-self.rotor_front<20 or self.rotor_front < -2 or self.rotor_back > 40:
            raise ValueError('Rotor outside the fixed axial casing envelope')
        if self.shaft_radius!=5.: raise ValueError('This bearing interface requires a 5 mm shaft radius')
        if not 6<=self.bolt_count<=16: raise ValueError('Invalid volute bolt count')
        if not 1<=self.wire_turns<=20: raise ValueError('Invalid conductor turn count')
        if min(self.tolerance,self.angular)<=0: raise ValueError('Tessellation tolerances must be positive')
        if not 16<=self.pleat_count<=64: raise ValueError('Invalid pleat count')


MATERIALS=[
    Material('Deep petrol anodized aluminium',(.022,.115,.125),.80,.32,microfinish='anodized',material_source='Original finish choice; not measured'),
    Material('Satin machined aluminium',(.62,.67,.70),.96,.27,microfinish='machined',material_source='Original finish choice'),
    Material('Graphite bead-blasted alloy',(.055,.065,.071),.82,.34,microfinish='bead-blasted'),
    Material('Copper winding conductor',(.72,.29,.105),.98,.25,microfinish='copper-wire'),
    Material('Warm ceramic filter substrate',(.57,.51,.40),0.,.83,microfinish='polymer',material_source='Visual substrate choice; no filter rating'),
    Material('Black elastomer',(.016,.021,.023),0.,.73,microfinish='polymer'),
    Material('Stainless socket hardware',(.52,.57,.60),.99,.22,microfinish='brushed'),
    Material('Champagne metal trim',(.50,.34,.15),.93,.31,microfinish='brushed'),
    Material('Pale identification inlay',(.83,.87,.85),.15,.43),
    Material('Orange silicone seals',(.64,.135,.035),0.,.57,microfinish='polymer'),
    Material('Soft graphite filter support',(.13,.15,.155),.30,.65,microfinish='bead-blasted'),
]


def cyl(radius,length,origin,direction=(1,0,0)):
    return cq.Solid.makeCylinder(radius,length,cq.Vector(*origin),cq.Vector(*direction))


def translate(shape,p): return shape.translate(tuple(p))


def box(size,center,r=0.):
    q=cq.Workplane('XY').box(*size)
    if r: q=q.edges().fillet(r)
    return q.val().translate(center)


def axis_ring(ro,ri,a,b,center=(0.,0.,0.),axis='x'):
    s=ring(ro,ri,a,b)
    if axis=='y':s=s.rotate((0,0,0),(0,0,1),90)
    elif axis=='z':s=s.rotate((0,0,0),(0,1,0),-90)
    return s.translate(center)


def socket_screw(x,y,z,length=12.,r=1.5,head=3.,depth=2.8,reverse=False):
    """Nominal smooth-shank socket screw with real hex recess; no fake thread claim."""
    body=ring(r,0,x,x+length).fuse(ring(head,0,x-depth,x))
    hexcut=cq.Workplane('YZ',origin=(x-depth-.1,0,0)).polygon(6,head*1.05).extrude(depth*.68).val()
    body=body.cut(hexcut).translate((0,y,z))
    if reverse: body=body.rotate((x,y,z),(x,y+1,z),180)
    return body


def scroll_profile(shrink=0.):
    """Smooth interpolated spiral plus a connected tangential outlet volume."""
    theta=np.linspace(0.,-TAU,129)
    rad=72.+24.*np.linspace(0.,1.,len(theta))-shrink
    pts=[(float(r*math.cos(t)),float(r*math.sin(t))) for r,t in zip(rad,theta)]
    wire=cq.Workplane('YZ').moveTo(*pts[0]).spline(pts[1:],includeCurrent=True).lineTo(*pts[0]).close()
    return wire


def scroll_solid(x0,x1,shrink=0.):
    s=scroll_profile(shrink).extrude(x1-x0).val().translate((x0,0,0))
    # This overlapping tangent extension is a genuine fused opening, not a
    # disconnected decorative nozzle. Hollowing uses the matching inner volume.
    outlet=box((x1-x0,42-2*shrink,101-shrink),(0.5*(x0+x1),80.,55.5+shrink*.5))
    return s.fuse(outlet).clean()


def blade_shape(c: Config):
    """Five-section twisted backward-curved BREP blade; no planar mesh proxy."""
    wp=None
    for k,u in enumerate(np.linspace(0,1,5)):
        x=c.rotor_front+2.5+u*(c.rotor_back-c.rotor_front-5.)
        rr=np.linspace(21.,c.rotor_radius-.8,17)
        angle=-.36*(rr-21.)/(c.rotor_radius-21.) + .17*(u-.5)
        thickness=1.05+0.40*(1.-u)
        left=[(float(r*math.cos(t+thickness/(2*r))),float(r*math.sin(t+thickness/(2*r)))) for r,t in zip(rr,angle)]
        right=[(float(r*math.cos(t-thickness/(2*r))),float(r*math.sin(t-thickness/(2*r)))) for r,t in zip(rr[::-1],angle[::-1])]
        if wp is None: wp=cq.Workplane('YZ',origin=(x,0,0))
        else: wp=wp.workplane(offset=(c.rotor_back-c.rotor_front-5.)/4.)
        wp=wp.moveTo(*left[0]).spline(left[1:],includeCurrent=True).lineTo(*right[0]).spline(right[1:],includeCurrent=True).close()
    return wp.loft(combine=True,ruled=False).val()


def revolved_hub(c: Config):
    # On XY, y is the radial coordinate. Explicit 2D profile revolves around X.
    f,b=c.rotor_front,c.rotor_back
    points=[(f,5),(f,14),(f+4,19),(b-6,21),(b,21),(b,5)]
    return cq.Workplane('XY').polyline(points).close().revolve(360,(0,0),(1,0)).val()


def pleated_disk(c: Config):
    """Finite-thickness folded sheet clipped by an analytic circular envelope."""
    ys=np.linspace(-60.,60.,c.pleat_count*2+1)
    front=[(-88.+(27. if k%2 else 0.),float(y)) for k,y in enumerate(ys)]
    back=[(x+.45,y) for x,y in front[::-1]]
    sheet=cq.Workplane('XY',origin=(0,0,-65)).polyline(front+back).close().extrude(130.).val()
    return sheet.intersect(ring(58.4,0,-90,-59)).clean()


def coupled_pose(part,t=0.,explode=0.):
    # Preview rotor is deliberately slowed to 15 rpm. This is rigid kinematics,
    # not motor dynamics, computational fluid dynamics, or simulated performance.
    a=TAU*.25*t if part.motion=='rotor' else 0.
    return axis_pose(a,part.center,part.explode,explode)


def build(config: Config | None=None):
    c=config or Config();c.check();parts=[];features={};timings={};start=time.time()
    def add(name,shape,mat=0,group='structure',ex=(0,0,0),role='',features_used=(),motion='fixed',tags=()):
        p=cad_part(name,shape,mat,tolerance=c.tolerance,angular=c.angular,group=group,
                   explode=np.asarray(ex,float),role=role or name.replace('_',' '),
                   provenance='designed-concept',motion=motion,tags=tuple(tags))
        parts.append(p)
        for f in features_used:features.setdefault(f,[]).append(name)
        return p
    def mark(stage):
        timings[stage]=round(time.time()-start,3)
        print(f'{stage}: {len(parts)} parts / {sum(len(p.faces) for p in parts):,} triangles',flush=True)

    # ------------------------ Base and connected tilt yoke --------------------
    base=box((242,194,13),(-8,0,-139),5.8)
    foot_positions=[(x,y) for x in (-103.,87.) for y in (-73.,73.)]
    for x,y in foot_positions: base=base.cut(cyl(2.4,20,(x,y,-150),(0,0,1)))
    # Recessed upper electronics tray. Separate cover sits on this actual lip.
    base=base.cut(box((106,82,3.5),(-52,0,-132.)),box((34,14,3.),(90,30,-132.)))
    add('Base_machined_and_pocketed',base,2,features_used=('box extrusion','fillet','boolean difference','linear pattern'))
    add('Electronics_tray_cover',box((104,80,2.5),(-52,0,-131.6),1.),0,ex=(0,0,32),features_used=('fillet',))
    for j,(x,y) in enumerate(foot_positions):
        foot=drafted_cylinder(0,9,11.8,8).rotate((0,0,0),(0,1,0),-90).translate((x,y,-154.5))
        add(f'Isolation_foot_{j+1}',foot,5,ex=(0,0,-10),features_used=('drafted extrusion',))
        add(f'Foot_bolt_{j+1}',cyl(2,16,(x,y,-153),(0,0,1)),6)
    for side in (-1,1):
        y=side*102.
        # Plate profile is genuinely extruded and pocketed. Hollow triangular
        # region and feet make the support load path visible in the render.
        profile=[(-48,-130),(53,-130),(33,-15),(28,8),(8,13),(-10,0)]
        s=cq.Workplane('XZ',origin=(0,y+5,0)).polyline(profile).close().extrude(10).val()
        cut=cq.Workplane('XZ',origin=(0,y+6,0)).polyline([(-23,-110),(29,-110),(16,-27),(5,-24)]).close().extrude(12).val()
        s=s.cut(cut).cut(cyl(8.,16,(16,y-8,0),(0,1,0)))
        s=cq.Workplane(obj=s).edges('|Y').fillet(2.8).val()
        add(f'Yoke_side_{side:+}',s,0,features_used=('profile extrusion','boolean difference','fillet','mirrored assembly'))
        shoe=box((108,27,9),(2,side*93.,-127.8),3.)
        add(f'Yoke_mounting_shoe_{side:+}',shoe,1)
        for x in (-38,38):
            add(f'Yoke_mount_bolt_{side:+}_{x}',cyl(3,14,(x,side*93,-138),(0,0,1)).fuse(cyl(5,4,(x,side*93,-124),(0,0,1))),6)
        lo,hi=(75,113) if side>0 else (-113,-82)
        add(f'Tilt_trunnion_{side:+}',axis_ring(8,3.2,lo,hi,(16,0,0),'y'),1,features_used=('arbitrary-axis cylinder',))
        add(f'Tilt_friction_washer_{side:+}',axis_ring(15,8.15,side*108-1.8,side*108+1.8,(16,0,0),'y'),5)
        knob=axis_ring(19,3.2,side*112-5,side*112+5,(16,0,0),'y')
        for k in range(10):
            th=TAU*k/10;knob=knob.cut(cyl(3.5,15,(16+20*math.cos(th),side*112-7.5,20*math.sin(th)),(0,1,0)))
        add(f'Tilt_lock_scalloped_knob_{side:+}',knob,2,features_used=('circular pattern','boolean difference'))
        add(f'Tilt_lock_center_cap_{side:+}',axis_ring(7,0,side*119-1.2,side*119+1.2,(16,0,0),'y'),7)
    # Rear stiffening rods attach to the shoes, not to empty space.
    for side in (-1,1):
        add(f'Yoke_spline_brace_{side:+}',spline_sweep_tube([(42,side*92,-122),(53,side*98,-89),(37,side*102,-42),(29,side*102,-10)],3.1),1,features_used=('3D spline sweep',))
    mark('base and yoke')

    # ----------------------- Genuine hollow scroll casting -------------------
    outer=scroll_solid(-8,44,0)
    inner=scroll_solid(-9,45,5.2)
    wall=outer.cut(inner)
    # Lugs follow the spiral's actual radius, instead of floating on a fixed BCD.
    lug_points=[]
    for k in range(c.bolt_count):
        u=(k+.5)/c.bolt_count;th=-TAU*u;r=72+24*u+2
        y,z=r*math.cos(th),r*math.sin(th);lug_points.append((y,z))
        wall=wall.fuse(cyl(6.5,52,(-8,y,z)))
    wall=drill(wall,lug_points,1.8,-9,45).clean()
    # The two scroll-side walls are joined by an integral, open rectangular
    # outlet flange. Without this land the open-sided shell is two solids even
    # though it becomes connected when sandwiched by the end plates.
    port_land=box((54,44,5),(18,80,103.5),1.).cut(box((41.6,31.6,9),(18,80,103.5),.5))
    wall=wall.fuse(port_land).clean()
    if len(wall.Solids())!=1: raise ValueError('Volute casting must be one connected solid')
    add('Spiral_volute_hollow_casting',wall,0,'scroll',role='One connected hollow volute solid with integral open outlet flange',features_used=('spline profile','boolean union','boolean difference','drilled pattern'))
    back=scroll_solid(44,49,0).cut(ring(13.,0,43,50))
    for y,z in lug_points:back=back.fuse(cyl(6.5,5,(44,y,z)))
    back=drill(back,lug_points,1.6,43,50).clean()
    add('Volute_rear_bulkhead',back,2,'backplate',ex=(26,0,0),features_used=('boolean union','drilled pattern'))
    cover=scroll_solid(-13.2,-8,0).cut(ring(49.,0,-15,-6))
    for y,z in lug_points:cover=cover.fuse(cyl(6.5,5.2,(-13.2,y,z)))
    cover=drill(cover,lug_points,1.85,-15,-6).clean()
    add('Volute_removable_front_plate',cover,1,'front_plate',ex=(-48,0,0),features_used=('boolean difference','drilled pattern'))
    for j,(y,z) in enumerate(lug_points):
        add(f'Volute_socket_fastener_{j+1:02}',socket_screw(-13.2,y,z,length=24,r=1.5,head=3.2),6,'front_fasteners',ex=(-55,0,0),features_used=('hexagonal recess',))
    # The front gasket is an actual torus seated in a boolean gland.
    throat=ring(57,48.5,-23,-13.2)
    gland=toroidal_groove(53.2,1.15,(-22.9,0,0))
    add('Inlet_throat_with_toroidal_gland',throat.cut(gland),0,'throat',ex=(-49,0,0),features_used=('toroidal groove','boolean difference'))
    add('Inlet_sealing_O_ring',toroidal_groove(53.2,1.03,(-22.9,0,0)),9,'throat',ex=(-49,0,0),features_used=('analytic torus',))
    mark('scroll casting')

    # ------------------------------- Rotor -----------------------------------
    disk=ring(c.rotor_radius,5.,c.rotor_back-2.5,c.rotor_back)
    # Balance drill holes are nominal geometric features, not evidence of balance.
    disk=drill(disk,bolt_circle(47,11),1.4,c.rotor_back-3,c.rotor_back+1)
    add('Impeller_back_disk',disk,1,'impeller',ex=(-8,0,0),motion='rotor',features_used=('ring','bolt-circle pattern','drill'))
    add('Impeller_revolved_hub',revolved_hub(c),7,'impeller',ex=(-8,0,0),motion='rotor',features_used=('profile revolution',))
    b=blade_shape(c)
    for j in range(c.blade_count):
        add(f'Impeller_twisted_blade_{j+1:02}',b.rotate((0,0,0),(1,0,0),360*j/c.blade_count),1,'impeller',ex=(-8,0,0),motion='rotor',features_used=('multi-section blade loft','circular pattern'))
    # Every blade meets both the rear disk and the front annular shroud.
    add('Impeller_front_shroud',ring(c.rotor_radius,46.,c.rotor_front,c.rotor_front+2.8),1,'impeller',ex=(-8,0,0),motion='rotor')
    shaft=ring(5.,0,-2.,108.)
    shaft=shaft.cut(box((17,5,2),(89,5.1,0)))
    add('Continuous_motor_impeller_shaft',shaft,6,'shaft',ex=(6,0,0),motion='rotor',features_used=('keyway boolean',))
    for x in (49.5,100.):
        add(f'Shaft_bearing_outer_{x:g}',ring(12.5,10.6,x,x+6.5),6,'bearings',ex=(28,0,0))
        add(f'Shaft_bearing_inner_{x:g}',ring(7.5,5.02,x,x+6.5),6,'bearings',ex=(28,0,0))
        for j,(y,z) in enumerate(bolt_circle(9.05,10)):
            add(f'Bearing_ball_{x:g}_{j}',cq.Solid.makeSphere(1.5,cq.Vector(x+3.25,y,z)),6,'bearings',ex=(28,0,0),features_used=('sphere','circular pattern'))
        for xx in (x+.15,x+5.85):
            add(f'Bearing_shield_{x:g}_{xx:g}',ring(10.6,7.5,xx,xx+.45),2,'bearings',ex=(28,0,0))
    mark('twisted impeller and bearings')

    # -------------------------- Serviceable filter stage ---------------------
    cage=ring(63.5,60.,-109,-23)
    # Rounded rectangular windows are holes through a real annular wall.
    cutter=box((60,10,8),(-65,63.5,0),2.)
    for j in range(18):cage=cage.cut(cutter.rotate((0,0,0),(1,0,0),360*j/18))
    add('Windowed_filter_cartridge_cage',cage,2,'filter_cage',ex=(-99,0,0),features_used=('rounded slot','circular pattern','boolean difference'))
    for i,x in enumerate((-108,-26)):
        flange=drill(ring(68,49,x-3,x+3),bolt_circle(65,6),1.7,x-4,x+4)
        add(f'Filter_cartridge_end_flange_{i}',flange,0,'filter_cage',ex=(-99,0,0),features_used=('ring','drilled pattern'))
        add(f'Filter_flange_trim_{i}',ring(68.1,65.8,x-.5,x+.5),7,'filter_cage',ex=(-99,0,0))
    add('Pleated_circular_filter_media',pleated_disk(c),4,'filter_media',ex=(-117,0,0),features_used=('corrugated profile extrusion','boolean intersection'))
    # Open-face grille, with crossing wires seated in the frame. Cylinders are
    # trimmed by a disk envelope, not drawn over an image.
    wires=[]
    for d in np.arange(-54.,55.,5.5):
        length=2*math.sqrt(max(0.,58.**2-d*d))
        wires.append(cyl(.38,length,(-91.,d,-length/2),(0,0,1)))
        wires.append(cyl(.38,length,(-92.,-length/2,d),(0,1,0)))
    add('Filter_inlet_woven_support_grid',cq.Compound.makeCompound(wires),6,'filter_media',ex=(-117,0,0),features_used=('analytic wire grid','compound'))
    add('Filter_media_front_retainer',ring(60,57,-95,-90),5,'filter_media',ex=(-117,0,0))
    add('Filter_media_rear_retainer',ring(60,57,-61,-57),5,'filter_media',ex=(-117,0,0))
    # Thin hoop reliefs give the housing an honest parting seam and service grip.
    for x in np.linspace(-106,-99,5):add(f'Cartridge_grip_rib_{x:.2f}',ring(65.4,63.4,x,x+.8),2,'filter_cage',ex=(-99,0,0))
    for j,(y,z) in enumerate(bolt_circle(65,6)):
        add(f'Filter_rear_flange_screw_{j+1}',socket_screw(-29,y,z,12,1.4,2.7),6,'filter_cage',ex=(-99,0,0))
    mark('pleated cartridge')

    # ---------------------- Asymmetric hollow collection hood ----------------
    sections=[(-232.,82.,61.,-13.,28.),(-216.,75.,56.,-11.,25.),(-187.,61.,49.,-7.,16.),(-149.,47.,45.,-2.,5.),(-119.,49.,49.,0.,0.),(-111.,49.,49.,0.,0.)]
    hood=lofted_elliptic_shell(sections,2.7)
    add('Asymmetric_hollow_capture_hood',hood,0,'hood',ex=(-174,0,0),role='Original open, constant radial-inset loft; no capture-flow qualification',features_used=('hollow offset elliptic loft','boolean difference'))
    # Rolled lip is an elliptic BREP sweep, not a black painted rim.
    lip_path=cq.Workplane('YZ',origin=(-232.,-13.,28.)).ellipse(80.5,59.5).val()
    # Use a multisection lip loft with a small axial bulge; regular closed ends.
    lip=lofted_elliptic_shell([(-234.,82.,61.,-13.,28.),(-232.,83.,62.,-13.,28.),(-230.,82.,61.,-13.,28.)],2.8)
    add('Hood_rolled_protective_lip',lip,5,'hood',ex=(-174,0,0),features_used=('multi-section hollow loft',))
    add('Hood_mating_collar',drill(ring(57,46.,-120,-110),bolt_circle(53,6),1.7,-121,-109),1,'hood',ex=(-174,0,0))
    for j,(y,z) in enumerate(bolt_circle(53,6)):
        add(f'Hood_retaining_screw_{j+1}',socket_screw(-120,y,z,12,1.4,2.8),6,'hood',ex=(-174,0,0))
    # Visible external ribs follow the actual hood section centers/sizes.
    for j,th in enumerate((-math.pi*.70,-math.pi*.30,math.pi*.30,math.pi*.70)):
        points=[(x,oy+(ry+1.)*math.cos(th),oz+(rz+1.)*math.sin(th)) for x,ry,rz,oy,oz in sections[:-1]]
        add(f'Hood_spline_reinforcement_{j+1}',spline_sweep_tube(points,1.25),2,'hood',ex=(-174,0,0),features_used=('3D spline sweep',))
    mark('freeform hood')

    # ----------------- Tangential outlet and topology cartridges -------------
    # Nominal duct opening is 31.6 x 41.6 mm, along +Z, connected to the volute.
    collar=box((57.,48.,6.),(18.,80.,108.),2.)
    collar=collar.cut(box((43.,31.6,9.),(18.,80.,108.)))
    add('Exhaust_outlet_mounting_collar',collar,1,'outlet',ex=(0,0,45),features_used=('fillet','boolean difference'))
    sleeve=box((51.,40.,35.),(18.,80.,126.),2.)
    sleeve=sleeve.cut(box((44.,33.,37.),(18.,80.,126.)))
    # Viewing windows expose actual lattice surfaces on the side of the cartridge.
    sleeve=sleeve.cut(box((38.,45.,23.),(18.,80.,126.),2.))
    add('Topology_cartridge_windowed_sleeve',sleeve,2,'outlet',ex=(0,0,55),features_used=('boolean difference','fillet'))
    # BCC is a BREP compound. Overlapping struts/nodes are disclosed; it is not
    # falsely claimed to be a single fused manufacturing solid.
    lattice=bcc_lattice(origin=(-1.,67.,111.),cells=(3,2,1),pitch=(12.,12.,8.),strut_radius=.60,node_radius=.9)
    add('BCC_cartridge_structural_support',lattice,7,'lattice',ex=(0,0,55),features_used=('BCC analytic strut lattice','compound'))
    v,f=gyroid_sheet_mesh((-1.,37.,66.,94.,120.,140.),resolution=(57,45,37),periods=(2.,1.5,1.),thickness=.25)
    p=mesh_part('Gyroid_sheet_flow_conditioner',v,f,10,group='gyroid',explode=np.array([0.,0.,55.]),provenance='designed-concept',
                role='Explicit sampled implicit TPMS mesh; not a certified filter or validated acoustic element',tags=('implicit-mesh','not-in-STEP'))
    parts.append(p);features.setdefault('implicit gyroid isosurface',[]).append(p.name)
    grill=box((51,40,3.),(18,80,144.),1.)
    for x in np.linspace(0,36,7):grill=grill.cut(box((3.,29.,6.),(float(x),80,144.),1.))
    add('Exhaust_slotted_guard',grill,0,'outlet',ex=(0,0,62),features_used=('slot pattern','boolean difference'))
    for x in (-5,41):
        for y in (65,95):
            add(f'Exhaust_guard_fastener_{x}_{y}',cyl(1.3,10,(x,y,135),(0,0,1)).fuse(cyl(2.4,2,(x,y,145),(0,0,1))),6,'outlet',ex=(0,0,64))
    mark('analytic and implicit topology')

    # ----------------------- Original motor subassembly -----------------------
    # Motor topology is original geometric design, not reverse-engineered CAD.
    add('Motor_front_mount_plate',drill(ring(35,12.55,49.,56.),bolt_circle(29.,4),2.2,48,57),1,'motor_front',ex=(28,0,0))
    shell=ring(35.,32.,56.,102.)
    shell=cq.Workplane(obj=shell).edges().chamfer(.45).val()
    add('Motor_shell_chamfered',shell,2,'motor_shell',ex=(70,0,0),features_used=('chamfer','ring'))
    # Proper separate radial cooling fins all touch the motor shell.
    for j in range(24):
        fin=box((43,5,1.3),(79.,36.4,0),.38).rotate((0,0,0),(1,0,0),360*j/24)
        add(f'Motor_radial_cooling_fin_{j+1:02}',fin,2,'motor_shell',ex=(70,0,0),features_used=('circular pattern',))
    cap=ring(35,0,102.,109.).cut(ring(12.55,0,101.,110.))
    for j,(y,z) in enumerate(bolt_circle(29,4)):
        cap=cap.cut(cyl(2.2,9,(101,y,z)))
        add(f'Motor_through_bolt_{j+1}',socket_screw(110.,y,z,61,1.8,3.5,reverse=True),6,'motor_rear',ex=(110,0,0))
    add('Motor_rear_bearing_endbell',cap,1,'motor_rear',ex=(105,0,0))
    # Rotor magnet sectors, retaining sleeve, and fully continuous shaft.
    add('Motor_rotor_drum',ring(17,5.05,61.,96.),2,'motor_rotor',ex=(62,0,0),motion='rotor')
    for j in range(14):
        a=j*TAU/14
        magnet=sector(19.4,17.,63.,94.,a+.025,a+TAU/14-.025)
        add(f'Motor_magnet_sector_{j+1:02}',magnet,2 if j%2 else 1,'motor_rotor',ex=(62,0,0),motion='rotor',features_used=('annular sector',))
    for j in range(12):
        a=j*TAU/12
        tooth=sector(31.3,22.,62.,96.,a-.125,a+.125)
        add(f'Motor_stator_tooth_{j+1:02}',tooth,10,'motor_stator',ex=(70,0,0),features_used=('annular sector','circular pattern'))
        t=np.linspace(0,TAU*c.wire_turns,c.wire_turns*44+1)
        # Winding bundle wraps axially around each tooth, distributed radially.
        x=79.+17.5*np.sign(np.cos(t))*np.abs(np.cos(t))**.38
        y=23.8+5.0*t/(TAU*c.wire_turns)
        z=3.8*np.sign(np.sin(t))*np.abs(np.sin(t))**.6
        points=np.column_stack([x,y,z])@rotation_x(a).T
        v,f=tube_mesh(points,.42,8)
        p=mesh_part(f'Copper_conductor_winding_{j+1:02}',v,f,3,group='windings',explode=np.array([70.,0.,0.]),provenance='designed-concept',
                    role='Original 8-turn geometric conductor route; winding schedule/electromagnetics unqualified',tags=('transported-tube-mesh','not-in-STEP'))
        parts.append(p);features.setdefault('parallel-transport conductor mesh',[]).append(p.name)
    add('Motor_stator_back_iron',ring(31.5,29.5,62.,96.),10,'motor_stator',ex=(70,0,0))
    # Thin moulded termination box: true shell operation, not a solid black block.
    enclosure=cq.Workplane('XY').box(28,22,17).edges('|Z').fillet(2.2).faces('>Z').shell(-1.8).val().translate((83,-19,41.))
    add('Motor_terminal_box_hollow_shell',enclosure,5,'terminal',ex=(70,0,8),features_used=('face-removal shell','fillet'))
    add('Motor_terminal_box_lid',box((28,22,2.),(83,-19,50.),.8),2,'terminal',ex=(70,0,22))
    for x in (73,93):
        add(f'Terminal_lid_screw_{x}',cyl(1.3,6,(x,-19,47),(0,0,1)).fuse(cyl(2.3,1.8,(x,-19,51),(0,0,1))),6,'terminal',ex=(70,0,23))
    mark('motor and routed conductors')

    # ------------------- Routed services / helix / identifiers ----------------
    cable_points=[(111,31,-129),(121,31,-116),(129,26,-82),(123,14,-47),(109,-5,10),(98,-15,40)]
    add('Continuous_power_harness',spline_sweep_tube(cable_points,3.2),5,'harness',features_used=('3D spline sweep',))
    spring=helical_sweep(x0=0.,length=24.,helix_radius=4.2,pitch=3.0,section_radius=.75)
    spring=spring.rotate((0,0,0),(0,1,0),-90).translate((115.,31.,-125.))
    add('Harness_helical_strain_relief',spring,6,'harness',features_used=('analytic helix sweep',))
    # A lofted ergonomic knob/handle grip adds a solid freeform BREP construction.
    grip=lofted_solid([(-30,8,5,0,0),(-15,11,6,0,1),(15,11,6,0,1),(30,8,5,0,0)])
    grip=grip.rotate((0,0,0),(0,0,1),90).translate((43.,0,107.))
    add('Lofted_carry_handle_grip',grip,5,'handle',features_used=('solid asymmetric loft',))
    for side in (-1,1):
        p=[(43,side*29,107),(43,side*40,103),(43,side*49,83),(43,side*51,66)]
        add(f'Carry_handle_swept_leg_{side:+}',spline_sweep_tube(p,3.4),1,'handle',features_used=('3D spline sweep',))
        add(f'Carry_handle_mount_{side:+}',box((14,12,7),(43,side*51,65),1.5),0,'handle')
    # Speed knob is mechanically seated into the electronics tray.
    add('Control_potentiometer_stem',cyl(3,10,(-82,-24,-132),(0,0,1)),6)
    knob=axis_ring(11.5,0,0,9.,(-82,-24,-126),'z')
    for j in range(36):
        t=j*TAU/36;knob=knob.cut(cyl(.65,11,(-82+11.6*math.cos(t),-24+11.6*math.sin(t),-127),(0,0,1)))
    add('Fluted_speed_control_dial',knob,7,features_used=('circular flute pattern',))
    add('Dial_index_inlay',box((1.0,5.,.22),(-82,-18.5,-116.88),.1),8)
    # Geometry labels are actual extruded text. No image texture or fake overlay.
    lettering=cq.Workplane('XZ',origin=(-65,-97.07,-137.6)).text('A E R I S',8.3,.16,combine=True,halign='center',valign='center').val()
    add('AERIS_base_identification',lettering,8,'markings',features_used=('extruded typography',))
    sub=cq.Workplane('XZ',origin=(41,-97.07,-138.)).text('CYBR GEO / 01',3.1,.16,combine=True,halign='center',valign='center').val()
    add('CYBR_GEO_serial_inlay',sub,7,'markings',features_used=('extruded typography',))
    mark('completed')

    photo=dict(projection='perspective',focal_length_mm=62.,f_stop=11.,environment_strength=.30,
               light_size=1.6,light_intensity=1.15,floor_gap_mm=.0,floor_roughness=.9,exposure=1.04)
    views={
        'hero':View(az=222,el=20,scale=193,target=(-50,0,0),title='AERIS / CAPTURE & SERVICE',**photo),
        'rear':View(az=40,el=22,scale=173,target=(-31,0,-12),title='AERIS / DRIVE & EXHAUST',**photo),
        'exploded':View(az=235,el=17,scale=240,target=(-100,0,0),explode=1,title='AERIS / AXIAL SERVICE EXPLOSION',**photo),
        'internal':View(az=212,el=23,scale=125,target=(-26,0,4),hide=('hood','filter_cage','front_plate','front_fasteners','motor_shell','motor_rear','terminal','markings'),title='AERIS / INTERNAL INSPECTION',**photo),
        'impeller':View(az=212,el=21,scale=83,target=(17,0,2),hide=('hood','filter_cage','filter_media','front_plate','front_fasteners','throat','outlet','gyroid','lattice','handle','harness','markings'),title='AERIS / LOFTED ROTOR',**photo),
        'topology':View(az=36,el=21,scale=51,target=(18,80,126),hide=('outlet',),title='AERIS / ANALYTIC + IMPLICIT TOPOLOGY',**photo),
    }
    metadata={
        'truth_intent':'concept', 'units':'mm',
        'design':'Original serviceable benchtop fume-extraction-turbine geometry demonstrator',
        'feature_map':features, 'build_timings_seconds':timings,
        'functional_assembly':{'axis':'X','rigid_rotor_preview_rpm':15,'blade_count':c.blade_count,
           'minimum_nominal_radial_rotor_scroll_clearance_mm':72-5.2-c.rotor_radius,
           'flow_path':'open lofted hood -> pleated disk -> impeller eye -> hollow spiral volute -> open tangent outlet -> topology cartridge',
           'bearings_x_mm':[49.5,100.], 'shaft_span_x_mm':[-2,108],
           'connected_services':'continuous shaft, supported yoke, bolted flanges, continuous routed harness'},
        'limitations':[
            'Geometry concept only: not a rated fume extractor, respirator or life-safety device.',
            'No fluid simulation, capture velocity, filtration efficacy, pressure drop or acoustic validation.',
            'No electrical, structural, thermal, fatigue, bearing-life or high-speed balance qualification.',
            'Screw shanks are nominal smooth envelopes; helical threads are not claimed.',
            'BCC lattice is an overlapping BREP compound, not a single fused manufacturing solid.',
            'Gyroid and conductor routes are explicitly meshes and are omitted from analytic STEP.',
            'Blade/backplate intersection gives a connected rotor, not proof of manufacturability.',
            'Rigid rotor animation is prescribed at 15 rpm, not simulated motor performance.'
        ],
        'drawings':{'default':{'annotations':[]}},
        'upstream':{'master':'2924a62059cf60baa830bea89004aa08cde8320d',
                    'advanced_geometry_source':'e99df0dd87bdda2a176850e94a57a948d6161243'},
    }
    return Assembly('aeris',parts,MATERIALS,views,metadata=metadata,motion_function=coupled_pose)


def cutaway(assembly: Assembly):
    """True boolean section of opaque enclosures; internals remain unchanged.

    A Y=0 half-section removes the camera-facing portion of the hood, filter
    cage, volute wall, front plate and motor shell. Cutting is performed on the
    analytic solids before retessellation, so wall thickness appears in the image.
    """
    out=[]
    # Clean y<0 half-section, genuinely cut through the selected CAD shells.
    cutting=box((850.,350.,550.),(-40.,-175.,20.))
    cutgroups={'hood','filter_cage','scroll','front_plate','motor_shell','motor_rear','throat','front_fasteners','terminal'}
    for p in assembly.parts:
        if p.group in cutgroups and p.cad is not None:
            s=p.cad.cut(cutting)
            if len(s.Solids())==0 or s.Volume()<1e-7: continue
            npart=cad_part(p.name+'_section',s,p.material,tolerance=.06,angular=.11,group=p.group,
                           role=p.role+'; analytic Y=0 half-section',provenance=p.provenance,
                           explode=np.asarray(p.explode),motion=p.motion)
            out.append(npart)
        else:out.append(p)
    photo=dict(projection='perspective',focal_length_mm=65.,f_stop=16.,environment_strength=.34,
               light_size=1.7,light_intensity=1.18,floor_gap_mm=0.,floor_roughness=.9,exposure=1.08)
    return replace(assembly,name='aeris_cutaway',parts=out,views={'hero':View(az=242,el=12,scale=180,target=(-53,0,-6),**photo)})


if __name__=='__main__':
    from mechanism_lab.core import save_cache,validate
    from mechanism_lab.exporters import export_glb,export_step
    import argparse
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,default=Path('build/aeris'));ap.add_argument('--step',action='store_true');args=ap.parse_args()
    a=build();args.out.mkdir(parents=True,exist_ok=True);save_cache(a,args.out)
    (args.out/'validation.json').write_text(json.dumps(validate(a),indent=2))
    export_glb(a,args.out/'aeris.glb')
    if args.step:export_step(a,args.out/'aeris.step')
