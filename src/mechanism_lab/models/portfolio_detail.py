"""High-density reference geometry for the portfolio machine.

This file intentionally adds functional-looking second-order geometry instead of random
greebles: fasteners belong to flanges, conduits terminate at manifolds, optical mounts
sit on rails, housing ribs connect real rings, and biome service hardware connects to
the vessel/base. It is a geometric fidelity layer over portfolio_machine.py.
"""
from __future__ import annotations

import math
import numpy as np
import cadquery as cq

from ..core import cad_part, mesh_part
from ..geometry import ring, tube_mesh


def _box(size, center, r=0.0):
    q = cq.Workplane("XY").box(*map(float, size)).translate(tuple(map(float, center)))
    if r > 0:
        try:
            q = q.edges().fillet(float(r))
        except Exception:
            pass
    return q.val()


def _cyl_z(radius, height, center):
    x, y, z = map(float, center)
    return cq.Solid.makeCylinder(float(radius), float(height),
                                 cq.Vector(x, y, z-height/2), cq.Vector(0,0,1))


def _cyl_x(radius, length, center):
    x, y, z = map(float, center)
    return cq.Solid.makeCylinder(float(radius), float(length),
                                 cq.Vector(x-length/2, y, z), cq.Vector(1,0,0))


def _sphere(radius, center):
    return cq.Solid.makeSphere(float(radius), cq.Vector(*map(float, center)))


def _cone_z(r1, r2, height, center):
    x, y, z = map(float, center)
    return cq.Solid.makeCone(float(r1), float(r2), float(height),
                             cq.Vector(x, y, z-height/2), cq.Vector(0,0,1))


def _part(parts, name, shape, mat, group, role, explode=(0,0,0), motion="fixed", tol=.16, ang=.11):
    parts.append(cad_part(name, shape, mat, tolerance=tol, angular=ang,
                          group=group, role=role, provenance="designed-concept",
                          explode=np.asarray(explode,float), motion=motion))


def _tube(parts, name, points, radius, mat, group, role, explode=(0,0,0), sides=10):
    v,f=tube_mesh(np.asarray(points,float), radius=float(radius), sides=int(sides))
    parts.append(mesh_part(name,v,f,mat,group=group,role=role,
                           provenance="designed-concept",explode=np.asarray(explode,float)))


def _bolt_x(parts, prefix, x, radius, count, bolt_r=5.0, length=10.0, phase=0.0, mat=4, group="fasteners"):
    for i in range(count):
        a=phase+math.tau*i/count
        y,z=radius*math.cos(a),radius*math.sin(a)
        _part(parts,f"{prefix}_{i+1:02}",_cyl_x(bolt_r,length,(x,y,z)),mat,group,
              "Axial flange fastener")


def _bolt_z(parts, prefix, z, radius, count, bolt_r=5.0, height=10.0, phase=0.0, mat=4, group="fasteners"):
    for i in range(count):
        a=phase+math.tau*i/count
        x,y=radius*math.cos(a),radius*math.sin(a)
        _part(parts,f"{prefix}_{i+1:02}",_cyl_z(bolt_r,height,(x,y,z)),mat,group,
              "Deck/ring fastener")


def _pipe_bundle(parts, prefix, paths, radius=5.0, mat=4, group="service"):
    for i,path in enumerate(paths):
        _tube(parts,f"{prefix}_{i+1:02}",path,radius,mat,group,
              "Routed power / coolant / data conduit",sides=12)


def _radial_box(center_r, angle, size, z):
    x=center_r*math.cos(angle);y=center_r*math.sin(angle)
    shape=_box(size,(x,y,z),4)
    return shape.rotate((x,y,z),(x,y,z+1),math.degrees(angle))


def add_scenes_detail(parts):
    # Dense annular service architecture around the habitat.
    for i,a in enumerate(np.linspace(0,math.tau,20,endpoint=False)):
        r=494
        x,y=r*math.cos(a),r*math.sin(a)
        _part(parts,f"SCD_Service_panel_{i+1:02}",
              _radial_box(r,a,(84,34,92),148),1,"detail",
              "Radial habitat service panel",explode=(18*math.cos(a),18*math.sin(a),0))
        _part(parts,f"SCD_Panel_latch_{i+1:02}",
              _cyl_z(6,12,(x,y,190)),4,"fasteners","Service panel quarter-turn latch")
    _bolt_z(parts,"SCD_Inner_ring_bolt",246,420,24,bolt_r=5.5,height=10)

    # Three nested dome seals/clamp rings.
    for j,(ro,ri,z,mat) in enumerate([(443,425,247,0),(432,418,258,4),(426,410,269,1)]):
        _part(parts,f"SCD_Dome_clamp_ring_{j+1}",ring(ro,ri,z,z+16),mat,"support",
              "Layered mechanical dome seal/clamp",explode=(0,0,36+j*12))

    # Articulated double-bar service arms and hydraulic actuators.
    arm_sets=[
        [(-470,330,150),(-520,330,500),(-350,250,745)],
        [(470,-330,150),(520,-330,500),(360,-260,740)],
    ]
    for ai,chain in enumerate(arm_sets):
        sign=-1 if ai==0 else 1
        for li,(p0,p1) in enumerate(zip(chain[:-1],chain[1:])):
            p0=np.asarray(p0,float);p1=np.asarray(p1,float)
            for off in (-18,18):
                q0=p0+np.array([0,off,0]);q1=p1+np.array([0,off,0])
                _tube(parts,f"SCD_Arm_{ai+1}_link_{li+1}_{off:+}",[q0,q1],11,1,"support",
                      "Twin structural arm member",explode=(0,sign*30,10*li))
            mid=(p0+p1)/2
            _part(parts,f"SCD_Arm_{ai+1}_link_plate_{li+1}",_box((56,72,32),mid,7),0,"support",
                  "Arm link reinforcement block")
        for ji,p in enumerate(chain):
            _part(parts,f"SCD_Arm_{ai+1}_joint_shell_{ji+1}",_sphere(48 if ji<2 else 38,p),1,"support",
                  "Machined arm pivot housing")
            _part(parts,f"SCD_Arm_{ai+1}_joint_pin_{ji+1}",_cyl_x(18,92,p),4,"support",
                  "Arm pivot pin")
        # Visible actuator.
        p0=np.asarray(chain[0],float)+np.array([0,-sign*34,58])
        p1=np.asarray(chain[1],float)+np.array([0,-sign*34,-30])
        _tube(parts,f"SCD_Arm_{ai+1}_hydraulic_body",[p0,(p0+p1)*.52],17,0,"service",
              "Hydraulic/service actuator body")
        _tube(parts,f"SCD_Arm_{ai+1}_hydraulic_rod",[(p0+p1)*.52,p1],8,2,"service",
              "Polished actuator rod")

    # Functional-looking manifolds and hoses terminating at the vessel.
    paths=[]
    for k,z in enumerate((115,138,161,184)):
        paths.append([(520,-260+k*18,z),(570,-220+k*12,z+8),(505,-80+k*10,z+12),(442,-20+k*8,235+k*4)])
    _pipe_bundle(parts,"SCD_Right_service",paths,5.5,4)

    # More convincing mesa/rock field inside the habitat.
    rock_data=[
        (-230,-120,95,62,16),(-205,-105,155,48,10),(-150,110,112,55,18),
        (165,-90,135,70,22),(205,-68,205,48,14),(220,115,100,42,16),
        (30,175,78,36,12),(-10,-190,60,34,10),(95,95,70,28,8)
    ]
    for i,(x,y,h,r1,r2) in enumerate(rock_data):
        _part(parts,f"SCD_Rock_spire_{i+1:02}",_cone_z(r1,r2,h,(x,y,300+h/2)),11,"environment",
              "Layered desert rock spire",explode=(0,0,100))
        if i%2==0:
            _part(parts,f"SCD_Rock_cap_{i+1:02}",_cone_z(r2*1.15,r2*.65,h*.38,(x+6,y-4,300+h*1.03)),
                  11,"environment","Eroded rock cap",explode=(0,0,100))
    # Small perimeter lights.
    for i,a in enumerate(np.linspace(0,math.tau,12,endpoint=False)):
        x,y=385*math.cos(a),385*math.sin(a)
        _part(parts,f"SCD_Rim_light_{i+1:02}",_cyl_z(8,26,(x,y,278)),4,"detail",
              "Dome-rim instrumentation/light pod")


def add_geo_detail(parts):
    # End-bell bolt circles and concentric service rings.
    _bolt_x(parts,"GED_Front_bolt",338,238,18,bolt_r=6,length=14)
    _bolt_x(parts,"GED_Rear_bolt",-338,238,18,bolt_r=6,length=14,phase=math.pi/18)
    for side,x,sgn in [("F",348,1),("R",-348,-1)]:
        _part(parts,f"GED_{side}_outer_trim",ring(302,282,x-8,x+8),1,"housing",
              "Outer end-bell trim ring",explode=(sgn*90,0,0))
        _part(parts,f"GED_{side}_brass_trim",ring(264,252,x-12,x+12),4,"housing",
              "Warm-metal end-bell trim ring",explode=(sgn*100,0,0))
        _part(parts,f"GED_{side}_bearing_face",ring(148,86,x-20,x+20),1,"bearings",
              "Bearing cartridge face",explode=(sgn*150,0,0))
        _bolt_x(parts,f"GED_{side}_bearing_bolt",x+sgn*22,116,12,bolt_r=4.5,length=10)

    # Real rounded copper hairpin loops around the exposed stator.
    n=18
    for i in range(n):
        a=math.tau*i/n
        b=a+math.tau/(n*2.45)
        r=232
        path=[]
        path.extend([(-238,r*math.cos(a),r*math.sin(a)),
                     (-270,r*math.cos(a),r*math.sin(a))])
        for u in np.linspace(0,1,6)[1:]:
            aa=a+(b-a)*u
            path.append((-286,r*math.cos(aa),r*math.sin(aa)))
        path.extend([(238,r*math.cos(b),r*math.sin(b)),
                     (260,r*math.cos(b),r*math.sin(b))])
        _tube(parts,f"GED_Hairpin_{i+1:02}",path,7.2,3,"windings",
              "Rounded exposed copper stator hairpin",explode=(0,22*math.cos(a),22*math.sin(a)),sides=12)

    # Lamination reveal and cage clamps.
    for j,x in enumerate(np.linspace(-245,245,11)):
        _part(parts,f"GED_Lamination_reveal_{j+1:02}",ring(214,207,x-2,x+2),2,"stator",
              "Visible lamination stack edge")
    for i,a in enumerate(np.linspace(0,math.tau,12,endpoint=False)):
        y,z=287*math.cos(a),287*math.sin(a)
        _part(parts,f"GED_Cage_clamp_{i+1:02}",_box((90,42,48),(0,287,0),5).rotate((0,0,0),(1,0,0),math.degrees(a)),
              1,"housing","Axial cage clamp over exposed core")
        # Through-rod bolt heads on both ends.
        for x,sgn in [(-316,-1),(316,1)]:
            _part(parts,f"GED_Tie_head_{i+1:02}_{'R' if sgn>0 else 'L'}",
                  _cyl_x(10,18,(x,y,z)),4,"fasteners","Tie-rod end nut / washer")

    # Top junction box, lid, cable glands and terminals.
    _part(parts,"GED_Terminal_box",_box((250,210,120),(25,0,350),14),0,"service",
          "Generator terminal / control box",explode=(0,0,95))
    _part(parts,"GED_Terminal_lid",_box((220,184,20),(25,0,420),7),1,"service",
          "Bolted terminal box lid",explode=(0,0,120))
    for i,y in enumerate((-70,0,70)):
        _part(parts,f"GED_Cable_gland_{i+1}",_cyl_x(18,44,(160,y,350)),4,"service",
              "Cable gland")
        _tube(parts,f"GED_Cable_{i+1}",[(182,y,350),(265,y,330),(330,y*1.1,285)],9,16,"service",
              "Power/control cable exiting junction box",sides=12)
    for x in (-65,25,115):
        for y in (-72,72):
            _part(parts,f"GED_Lid_bolt_{x}_{y}",_cyl_z(5.5,12,(x,y,432)),4,"fasteners",
                  "Terminal lid fastener")

    # Vertical mounts and triangulated support brackets.
    for x in (-390,390):
        for y in (-220,220):
            _part(parts,f"GED_Pedestal_{x}_{y}",_box((92,76,185),(x,y,-145),8),0,"mounts",
                  "Heavy generator pedestal",explode=(0,0,-65))
            _tube(parts,f"GED_Brace_{x}_{y}",[(x,y,-70),(x*.78,y*.84,75)],12,1,"mounts",
                  "Diagonal generator support brace")

    # Service manifold on the left side, matching the reference pipe bundle.
    _part(parts,"GED_Service_manifold",_box((190,120,100),(-470,-310,-40),10),0,"service",
          "External coolant/power service manifold",explode=(-55,-25,0))
    paths=[]
    for i,z in enumerate((-100,-65,-30,5)):
        paths.append([(-365,-270,z),(-420,-300,z),(-505,-300+i*9,z+12),(-565,-250+i*12,z+28)])
    _pipe_bundle(parts,"GED_Service_pipe",paths,6.5,4)


def add_light_detail(parts):
    # Round precision rails instead of relying only on the rectangular base.
    for y in (-148,148):
        _part(parts,f"LID_Rail_{y:+}",_cyl_x(12,1320,(0,y,-105)),2,"rails",
              "Ground round optical-bench rail")
        for x in (-520,-310,-110,110,310,520):
            _part(parts,f"LID_Rail_clamp_{x}_{y:+}",_box((48,48,46),(x,y,-104),5),0,"mounts",
                  "Optical rail clamp")
            _part(parts,f"LID_Clamp_knob_{x}_{y:+}",_cyl_z(11,18,(x,y,-72)),4,"fasteners",
                  "Rail clamp locking knob")

    # Emitter barrel: focus/retaining rings and rear service collar.
    for j,(x,ro,ri,mat) in enumerate([(-635,112,94,1),(-606,108,91,4),(-575,102,86,0),(-530,90,74,1)]):
        _part(parts,f"LID_Emitter_ring_{j+1}",ring(ro,ri,x-9,x+9),mat,"optics",
              "Emitter retaining/focus ring",explode=(-45,0,0))
    _bolt_x(parts,"LID_Emitter_bolt",-635,102,12,bolt_r=4,length=8)
    for a in np.linspace(0,math.tau,8,endpoint=False):
        y,z=103*math.cos(a),103*math.sin(a)
        _part(parts,f"LID_Emitter_fin_{int(round(math.degrees(a))):03}",
              _box((72,10,30),(-570,103,0),3).rotate((-570,0,0),(-569,0,0),math.degrees(a)),
              1,"optics","Emitter heat-sink rib")

    # Tall prism tower receives cross-bracing, stages and micrometer adjusters.
    for z in (20,120,220,320):
        _part(parts,f"LID_Tower_cross_stage_{z}",_box((86,260,24),(-150,0,z),4),1,"support",
              "Prism tower cross-stage")
    for y in (-128,128):
        _tube(parts,f"LID_Tower_brace_{y:+}",[(-175,y,-80),(-150,y,330)],8,4,"support",
              "Tower tension/support rod")
    for i,(x,y,z) in enumerate([(-220,-145,145),(-220,145,145),(-80,-145,145),(-80,145,145)]):
        _part(parts,f"LID_Prism_corner_post_{i+1}",_cyl_z(10,250,(x,y,z)),2,"support",
              "Prism enclosure corner post")
        _part(parts,f"LID_Prism_adjuster_{i+1}",_cyl_x(8,58,(x-20,y,z+40)),4,"fasteners",
              "Prism micrometer adjuster")

    # Relay optic gets retaining rings and vertical cage.
    for z,r in [(-58,72),(-30,62),(240,62),(260,74)]:
        _part(parts,f"LID_Relay_ring_{z:+}",_cyl_z(r,12,(-20,0,z)),1 if abs(z)<250 else 4,"optics",
              "Relay optic retaining ring")
    for a in np.linspace(0,math.tau,6,endpoint=False):
        x=-20+72*math.cos(a);y=72*math.sin(a)
        _part(parts,f"LID_Relay_post_{int(round(math.degrees(a))):03}",_cyl_z(6,330,(x,y,100)),4,"support",
              "Relay optic cage post")

    # Steering mirror kinematic gimbal and adjuster screws.
    _part(parts,"LID_Mirror_outer_gimbal",ring(116,93,250,278),1,"optics",
          "Outer steering-mirror gimbal",explode=(20,0,35))
    _part(parts,"LID_Mirror_inner_gimbal",ring(91,75,282,304),4,"optics",
          "Inner steering-mirror gimbal",explode=(30,0,45))
    for a in (0,math.pi/2,math.pi,3*math.pi/2):
        y,z=120*math.cos(a),40+120*math.sin(a)
        _part(parts,f"LID_Mirror_adjuster_{int(a*100):03}",_cyl_x(8,65,(225,y,z)),4,"fasteners",
              "Kinematic mirror adjuster")

    # Output lens barrel with nested stages and bolt ring.
    for j,(x,ro,ri,mat) in enumerate([(500,104,78,1),(535,118,92,0),(575,126,98,4),(615,112,88,1)]):
        _part(parts,f"LID_Output_ring_{j+1}",ring(ro,ri,x-10,x+10),mat,"optics",
              "Output lens barrel retaining ring",explode=(35,0,0))
    _bolt_x(parts,"LID_Output_bolt",585,111,12,bolt_r=4.2,length=9)

    # Cable tray and deliberate routed services.
    _part(parts,"LID_Cable_tray",_box((1220,46,28),(0,-205,-155),5),0,"service",
          "Optical system cable tray",explode=(0,-25,-20))
    paths=[
        [(-580,-205,-145),(-430,-205,-120),(-180,-205,-100),(-150,-120,30)],
        [(-40,-205,-145),(20,-205,-100),(250,-205,-80),(290,-115,-20)],
        [(320,-205,-145),(430,-205,-120),(560,-180,-90),(570,-100,-30)],
    ]
    _pipe_bundle(parts,"LID_Cable",paths,5.2,16)


def add_elements_detail(parts):
    half=410
    # Heavy machined corner shoes, cross-ties and panel clamps.
    for x in (-half-28,half+28):
        for y in (-half-28,half+28):
            for z in (-half+36,0,half-36):
                _part(parts,f"ELD_Corner_shoe_{x}_{y}_{z}",_box((76,76,44),(x,y,z),8),1,"frame",
                      "Containment-frame machined clamp")
                _part(parts,f"ELD_Shoe_bolt_{x}_{y}_{z}",_cyl_z(7,14,(x,y,z+28)),4,"fasteners",
                      "Containment clamp bolt")
    for z in (-330,330):
        for y in (-438,438):
            _part(parts,f"ELD_X_rail_{z}_{y}",_box((760,28,34),(0,y,z),4),1,"frame",
                  "Containment cross rail")
    # Top gantry/actuator seen in the sheet.
    _part(parts,"ELD_Top_gantry",_box((620,82,72),(0,0,500),10),0,"service",
          "Top thermal/fluid gantry",explode=(0,0,90))
    _part(parts,"ELD_Top_slider",_box((180,126,86),(110,0,500),9),1,"service",
          "Top gantry sliding actuator",explode=(30,0,100))
    _part(parts,"ELD_Top_rod",_cyl_x(18,690,(0,0,500)),2,"service",
          "Gantry guide/actuator rod",explode=(0,0,105))
    # Side circulation manifold and pipes.
    _part(parts,"ELD_Manifold",_box((125,250,170),(-500,0,-205),12),0,"service",
          "Cooling/circulation manifold",explode=(-65,0,0))
    paths=[]
    for i,z in enumerate((-260,-205,-150,-95)):
        paths.append([(-442,-120+i*70,z),(-495,-120+i*70,z),(-540,-80+i*60,z+30),(-515,0+i*32,-5+i*52)])
    _pipe_bundle(parts,"ELD_Coolant_line",paths,7,4)

    # Replace spherical feel with many faceted/tapered rock shards around the core.
    rng=np.random.default_rng(821)
    for i in range(24):
        ang=rng.uniform(-1.2,1.2)
        x=rng.uniform(-210,190);y=rng.uniform(-90,70);z=rng.uniform(-260,240)
        r1=rng.uniform(28,64);r2=rng.uniform(7,28);h=rng.uniform(50,130)
        shape=_cone_z(r1,r2,h,(x,y,z))
        try:
            shape=shape.rotate((x,y,z),(x,y+1,z),math.degrees(ang))
        except Exception:
            pass
        _part(parts,f"ELD_Rock_shard_{i+1:02}",shape,8,"terrain",
              "Angular basalt shard in thermal interaction core",explode=(0,0,35),tol=.22,ang=.16)

    # Fire veins branch across the left face.
    fire_paths=[
        [(-280,-125,300),(-250,-130,220),(-275,-132,125),(-230,-128,30),(-260,-124,-85),(-220,-115,-230)],
        [(-170,-122,245),(-205,-128,170),(-170,-129,95),(-195,-122,10)],
        [(-320,-105,100),(-270,-120,40),(-305,-118,-40)],
        [(-120,-115,155),(-145,-120,75),(-110,-118,-5),(-140,-112,-105)],
    ]
    for i,path in enumerate(fire_paths):
        _tube(parts,f"ELD_Fire_branch_{i+1}",path,8 if i else 12,7,"fire",
              "Branching hot-element / lava vein",explode=(-25,0,0),sides=10)

    # Water sheets: multiple close parallel strands create a sheet silhouette.
    for band,base_x in enumerate((105,175,250)):
        for s in range(4):
            dx=s*10-15
            path=[(base_x+dx,-130,310),(base_x-8+dx,-136,200),(base_x+16+dx,-138,85),
                  (base_x-10+dx,-132,-40),(base_x+8+dx,-120,-235)]
            _tube(parts,f"ELD_Water_sheet_{band+1}_{s+1}",path,7.5,6,"water",
                  "Parallel water ribbon forming a falling sheet",explode=(28,0,0),sides=10)

    # Lower service deck detail.
    for i,x in enumerate(np.linspace(-320,320,9)):
        _part(parts,f"ELD_Base_panel_{i+1:02}",_box((62,32,92),(x,-430,-356),5),1,"detail",
              "Lower service panel")
        _part(parts,f"ELD_Base_latch_{i+1:02}",_cyl_x(5,12,(x,-449,-356)),4,"fasteners",
              "Lower service-panel latch")


def add_materials_detail(parts):
    # More layered sample rack: back frames, clamps and support fingers.
    sample_x=[-210,-125,-35,60,150,235]
    sample_h=[410,450,470,450,405,360]
    for i,(x,h) in enumerate(zip(sample_x,sample_h)):
        _part(parts,f"MAD_Sample_back_frame_{i+1}",_box((36,70,h+54),(x+8,74,-70+h/2),5),1,"samples",
              "Metal-backed sample cassette",explode=(0,35+i*6,65+i*12))
        _part(parts,f"MAD_Sample_lower_clamp_{i+1}",_box((92,86,42),(x,-35,-46),6),0,"mounts",
              "Non-destructive lower sample clamp")
        _part(parts,f"MAD_Sample_upper_clamp_{i+1}",_box((84,80,36),(x,-22,-70+h-12),6),4,"mounts",
              "Upper sample retaining clamp",explode=(0,25,35))
        for j,z in enumerate(np.linspace(0,h-70,4)):
            _part(parts,f"MAD_Sample_edge_bolt_{i+1}_{j+1}",_cyl_x(5,18,(x+70,-50,-35+z)),4,"fasteners",
                  "Sample cassette edge fastener")
    # Stacked tongue/groove boards on the primary wood specimens.
    for i,x in enumerate((-125,-35,60)):
        for j in range(5):
            y=-5-j*10
            z=80+j*32
            _part(parts,f"MAD_Board_layer_{i+1}_{j+1}",_box((250-j*14,18,26),(x,y,z),3),9 if i<2 else 10,"samples",
                  "Layered flooring/material board specimen",explode=(0,15+j*3,20+j*6))

    # Large inspection wheel gets a toothed/ribbed edge and hub bolts.
    for i,a in enumerate(np.linspace(0,math.tau,20,endpoint=False)):
        y=118*math.cos(a);z=-30+118*math.sin(a)
        _part(parts,f"MAD_Wheel_tooth_{i+1:02}",_box((42,24,38),(374,y,z),4).rotate((374,0,-30),(375,0,-30),math.degrees(a)),
              1,"mechanism","Inspection wheel rim rib")
    _bolt_x(parts,"MAD_Wheel_hub_bolt",384,68,10,bolt_r=4.5,length=10)

    # Replace single-stick arm silhouette with two parallel structural links.
    joints=[
        np.array([310.,220.,20.]),np.array([380.,220.,310.]),
        np.array([190.,120.,520.]),np.array([30.,65.,410.])
    ]
    for i,(p0,p1) in enumerate(zip(joints[:-1],joints[1:])):
        for off in (-18,18):
            a=p0+np.array([0,off,0]);b=p1+np.array([0,off,0])
            _tube(parts,f"MAD_Arm_twin_{i+1}_{off:+}",[a,b],10,1,"arm",
                  "Twin material-handler structural link",explode=(0,30,10*i))
        _part(parts,f"MAD_Arm_joint_cover_{i+1}",_sphere(48,p1),0,"arm",
              "Arm joint housing")
        _part(parts,f"MAD_Arm_joint_pin_{i+1}",_cyl_x(15,92,p1),4,"arm",
              "Arm joint pin")

    # Linear-drive screw, end bearings and cable chain.
    _part(parts,"MAD_Lead_screw",_cyl_x(10,840,(0,-250,-175)),2,"mechanism",
          "Visible precision lead screw")
    for x in (-430,430):
        _part(parts,f"MAD_Lead_bearing_{x:+}",_box((72,84,76),(x,-250,-175),8),0,"mechanism",
              "Lead-screw end bearing")
    chain=[]
    for i,x in enumerate(np.linspace(-410,250,18)):
        z=-205+20*math.sin(i*.35)
        _part(parts,f"MAD_Cable_chain_{i+1:02}",_box((34,28,22),(x,255,z),4),16,"service",
              "Articulated cable-chain segment")


def add_forest_detail(parts):
    # Vessel foundation panels and external life-support manifold.
    for i,a in enumerate(np.linspace(0,math.tau,18,endpoint=False)):
        r=410;x,y=r*math.cos(a),r*math.sin(a)
        _part(parts,f"FOD_Base_panel_{i+1:02}",_radial_box(r,a,(72,30,104),-28),1,"detail",
              "Biome-base service panel")
        _part(parts,f"FOD_Base_latch_{i+1:02}",_cyl_z(5.5,12,(x,y,20)),4,"fasteners",
              "Biome service panel latch")
    for i,a in enumerate(np.linspace(0,math.tau,12,endpoint=False)):
        x,y=432*math.cos(a),432*math.sin(a)
        _part(parts,f"FOD_Vessel_post_{i+1:02}",_cyl_z(8,545,(x,y,320)),4,"support",
              "External vessel tie/support post",explode=(18*math.cos(a),18*math.sin(a),0))
        _part(parts,f"FOD_Vessel_top_clamp_{i+1:02}",_sphere(15,(x,y,590)),1,"support",
              "Upper vessel clamp")

    # Detailed branch skeleton and clustered foliage for the four hero trees.
    trees=[
        (-145,10,160,270,82),(105,55,180,230,70),(5,-125,150,185,58),(205,-45,140,150,48)
    ]
    for ti,(x,y,z,h,cr) in enumerate(trees):
        trunk_top=np.array([x,y,z+h],float)
        for bi,(ang,tilt,scale) in enumerate([
            (.2,.55,.75),(1.35,.48,.66),(2.6,.60,.70),(3.75,.50,.62),(5.1,.58,.68)
        ]):
            start=trunk_top-np.array([0,0,h*(.18+.08*(bi%2))])
            end=start+np.array([math.cos(ang)*cr*1.35,math.sin(ang)*cr*1.35,cr*tilt])
            _tube(parts,f"FOD_Tree_{ti+1}_branch_{bi+1}",[start,end],7 if ti<2 else 5,9,"biome",
                  "Tree branch skeleton",explode=(0,0,70),sides=9)
            for ci,t in enumerate((.55,.82,1.0)):
                p=start+(end-start)*t
                _part(parts,f"FOD_Tree_{ti+1}_branch_crown_{bi+1}_{ci+1}",
                      _sphere(cr*(.34-.05*ci),p),12,"biome","Clustered branch foliage",
                      explode=(0,0,72),tol=.55,ang=.25)

    # More angular terrain/rock hierarchy.
    rock_data=[
        (-230,90,205,70,20,105),(-170,130,265,62,18,90),(140,95,220,68,16,110),
        (225,-45,185,58,14,85),(50,150,300,48,12,80),(-45,-155,190,52,15,78),
        (120,-160,170,44,11,65),(-260,-60,170,55,18,72)
    ]
    for i,(x,y,z,r1,r2,h) in enumerate(rock_data):
        _part(parts,f"FOD_Rock_spire_{i+1:02}",_cone_z(r1,r2,h,(x,y,z+h/2)),8,"biome",
              "Angular forest terrain rock",explode=(0,0,55),tol=.24,ang=.18)

    # Pump/filtration interconnects that visibly terminate at the biome base.
    paths=[
        [(-440,-255,40),(-500,-210,30),(-470,-120,10),(-395,-70,30)],
        [(440,255,60),(505,210,50),(470,120,20),(395,80,40)],
        [(440,-255,90),(510,-210,80),(475,-120,40),(395,-75,55)],
        [(-420,250,100),(-490,210,90),(-460,130,55),(-390,85,65)],
    ]
    _pipe_bundle(parts,"FOD_Life_support_pipe",paths,7,4)

    # Mechanical inspection arm twin-link detail.
    joints=[
        np.array([-430.,285.,40.]),np.array([-480.,285.,360.]),
        np.array([-310.,240.,650.]),np.array([-160.,150.,660.])
    ]
    for i,(p0,p1) in enumerate(zip(joints[:-1],joints[1:])):
        for off in (-16,16):
            a=p0+np.array([0,off,0]);b=p1+np.array([0,off,0])
            _tube(parts,f"FOD_Arm_twin_{i+1}_{off:+}",[a,b],10,1,"support",
                  "Twin biome inspection-arm link",explode=(-25,20,10*i))
        _part(parts,f"FOD_Arm_pin_{i+1}",_cyl_x(15,88,p1),4,"support",
              "Inspection-arm pivot pin")
