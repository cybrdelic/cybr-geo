"""Original ARX-6 belt design using GEO's shared native construction services.

Adapted directly from cybrdelic/cybr-arx6-belt at commit
 aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2,
 ARX-6-P5-Engineering/integration/generate_cad_p2.py.
Geometry, placement, material assumptions and mass reserves retain source values.
The original source's nominal prototype / vendor-envelope qualification applies.
Importing this recipe does not generate files or alter the source checkout.
This adaptation follows CYBR GEO's GPL-2.0-only licensing.
"""
from __future__ import annotations

import hashlib
import math
from pathlib import Path

import cadquery as cq
import numpy as np

from mechanism_lab.native_construction import NativeConstruction, NativeInterfaceRequirements, read_specification, transform_shape, shape_mass_properties

SOURCE_COMMIT = 'aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2'
SOURCE_GENERATOR_SHA256 = 'ae3b03b11f533c59a3507d18d93f9b0f7df7a804a8e52ced44b1d3f577d55e37'


def validate_native_spec(spec):
    """Reject edits to retained fixed bench geometry before invoking CAD.

    These source dimensions are shared with the contact builder. The authored
    foot, pedestal, fastener lengths and upstream frame are a coupled design;
    accepting a changed fixture without constructing all those changes would
    create disagreement between native geometry and contact data.
    """
    expected = dict(top_y_mm=0, thickness_mm=40,
        through_bores=[dict(center_xz_mm=[0, 0], diameter_mm=110),
                       dict(center_xz_mm=[110, 0], diameter_mm=46)],
        base_foot_bounds_xz_mm=[[-95, 205], [-95, 95]], base_foot_thickness_mm=12,
        mount_bolt_centers_xz_mm=[[-75, 45], [-75, -45], [30, 84], [30, -84], [175, 45], [175, -45]])
    bench = spec.get('bench_interface')
    if not isinstance(bench, dict):
        raise ValueError('Native design requires bench_interface')
    for key, value in expected.items():
        if bench.get(key) != value:
            raise ValueError(f'Unsupported bench_interface.{key} edit: revise the native foot, pedestal and mounting construction together')


def nominal_interface_requirements(parts, specification, transmissions):
    """The source's 52 nominal fit constraints, evaluated before build acceptance.

    Adapted from integration/validate_p5_mechanics.py at SOURCE_COMMIT. These
    retain the source geometry, pair selection and 1e-5 mm3 kernel allowance.
    Native in-memory solids replace STEP readback; the measurements use GEO's
    shared constraint collector. These are not production tolerance checks.
    """
    requirements = NativeInterfaceRequirements(dict(repository='https://github.com/cybrdelic/cybr-arx6-belt',
        commit=SOURCE_COMMIT, source='ARX-6-P5-Engineering/integration/validate_p5_mechanics.py',
        adaptation='Original 52 interface pairs measured on constructed native solids before acceptance'))
    S, T = specification, transmissions
    def read(name):
        shape = parts[name]
        return shape.val() if hasattr(shape, 'val') else shape
    def cyl(r, z0, z1):
        return cq.Solid.makeCylinder(r, z1-z0, cq.Vector(0, 0, z0))
    check = requirements.noninterference
    transform = transform_shape
    for size in ['large','small']:
        f=S['folded_transmission'][size];large=size=='large';cent=f['center_distance_mm'];belt=read(size+'_folded_timing_belt')
        for other,shift in [('driven_pulley',False),('drive_pulley',True),('carrier_deck',False),('front_cover',False),('housing_rails',False)]:
            a=read(size+'_folded_'+other);a=a.translate((0,cent,0)) if shift else a
            check(size+'_belt_vs_'+other,belt,a)
        z=f['pulley_center_z_mm'];w=f['belt_width_mm'];key=cq.Workplane('XY').box(4 if large else 2,4 if large else 2,w+3).translate((5.8 if large else 3.3,0,z)).val()
        check(size+'_input_key_vs_keyed_shaft',key,read(size+'_folded_input_shaft'))
        check(size+'_input_key_vs_pulley',key,read(size+'_folded_drive_pulley'))
        check(size+'_input_shaft_vs_pulley',read(size+'_folded_input_shaft'),read(size+'_folded_drive_pulley'))
        check(size+'_input_bearing_vs_cover',read(size+'_folded_input_support_bearing').translate((0,cent,0)),read(size+'_folded_front_cover'))
    for t in T:
        j=t['joint'];size='large' if int(j[1])<4 else 'small';large=size=='large';f=S['folded_transmission'][size];theta=t['clocking_degrees'];z=f['pulley_center_z_mm'];w=f['belt_width_mm']
        shaft=read(j+'_clocked_output_shaft');key=cq.Workplane('XY').box(6 if large else 2,6 if large else 2,w+3).translate((9.5 if large else 3.8,0,z)).val().rotate((0,0,0),(0,0,1),theta)
        pulley=read(size+'_folded_driven_pulley').rotate((0,0,0),(0,0,1),theta)
        check(j+'_output_key_vs_clocked_shaft',key,shaft)
        check(j+'_output_key_vs_clocked_pulley',key,pulley)
        check(j+'_shaft_vs_pulley',shaft,pulley)
        check(j+'_shaft_flange_vs_front_cover',shaft,read(size+'_front_bearing_cover'))
    # Exact table service cutouts and J1 base interface, in body/world mm.
    # validate_native_spec rejects changed fixture values until all authored
    # interfaces are revised together.
    bench=cq.Workplane('XZ').box(1000,1000,40).translate((0,-20,0)).val()
    for x,r in [(0,55),(110,23)]:bench=bench.cut(cq.Solid.makeCylinder(r,60,cq.Vector(x,-50,0),cq.Vector(0,1,0)))
    t=T[0];f=S['folded_transmission']['large'];theta=t['clocking_degrees'];origin=t['origin_mm'];axis=t['axis'];cent=f['center_distance_mm']
    base=transform(read('bench_mount_plate'),(0,0,0),(0,1,0))
    for part in ['carrier_deck','housing_rails','front_cover']:
        sh=transform(read('large_folded_'+part).rotate((0,0,0),(0,0,1),theta),origin,axis)
        check('J1_'+part+'_vs_table_with_two_real_bores',sh,bench)
        check('J1_'+part+'_vs_base_foot',sh,base)
    for part,z0,z1,r in [('gearhead',-104,-34,21),('motor',-34,22.1,20)]:
        sh=transform(cyl(r,z0,z1).translate((0,cent,0)).rotate((0,0,0),(0,0,1),theta),origin,axis)
        check('J1_'+part+'_vs_table_with_two_real_bores',sh,bench)
        check('J1_'+part+'_vs_base_foot',sh,base)
    if len(requirements.checks) != 52:
        raise ValueError('The retained source interface contract requires exactly 52 nominal checks')
    return requirements


def build(spec, output):
    """Generate exact authored native parts and body-local mass records.

    ``spec`` is the original specification object or its JSON path. ``output``
    receives generated artifacts only; existing unrelated files are preserved.
    Returns NativeBuild. The historical ``jaw`` body and separate ``left_jaw``
    assembly remain explicit, for the model adapter to map to the XML bodies.
    """
    SPEC, specification_identity = read_specification(spec)
    validate_native_spec(SPEC)
    RHO = {'aluminum':2700., 'steel':7850., 'polymer':1150., 'rubber':1100.}
    construction = NativeConstruction(output, RHO,
        body_ids=['fixed','turret','upper','fore','roll','bend','tool','jaw'],
        provenance=dict(repository='https://github.com/cybrdelic/cybr-arx6-belt',
            commit=SOURCE_COMMIT, generator_sha256=SOURCE_GENERATOR_SHA256,
            **specification_identity,
            adapted_recipe_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            adaptation='Original geometry recipe; shared native placement and mass accounting'))
    PARTS, META, BODIES = construction.parts, construction.part_metadata, construction.body_records
    MESHES = construction.meshes
    addpart, place, ppart = construction.addpart, construction.place, construction.place_part
    trans_shape, inertia_shape = transform_shape, shape_mass_properties
    primitive_records = []
    def cyl(r,z0,z1):
        return cq.Workplane('XY').workplane(offset=z0).circle(r).extrude(z1-z0)
    def ring(ro,ri,z0,z1):
        return cyl(ro,z0,z1).cut(cyl(ri,z0-1,z1+1))
    def holes(s,pcd,n,d,z0,z1,phase=0):
        for i in range(n):
            a=phase+2*math.pi*i/n
            s=s.cut(cyl(d/2,z0,z1).translate((pcd/2*math.cos(a),pcd/2*math.sin(a),0)))
        return s
    def drill(s,points,axis,d,L=500):
        for pt in points:s=s.cut(trans_shape(cyl(d/2,-L/2,L/2),pt,axis))
        return s

    def build_support(size):
        large=size=='large'
        d,D,B,sep=(20,42,12,34) if large else (12,28,8,20)
        ro=27 if large else 18
        # Bearings end at front z=B/2 and rear z=-sep-B/2.
        rear=-sep-B/2; front=B/2
        core=ring(ro,D/2-1.5,rear,front)
        core=core.cut(cyl(D/2,rear-1,rear+B)).cut(cyl(D/2,front-B,front+1))
        pcd=48 if large else 31
        core=holes(core,pcd,4,3.3 if large else 2.2,rear-1,front+1,math.pi/4)
        addpart(size+'_bearing_housing',core)
        covers=[]
        for end,z in [('rear',rear-4),('front',front)]:
            s=ring(36 if large and end=='rear' else 25 if end=='rear' else ro,18 if large else 12,z,z+4)
            s=holes(s,pcd,4,(2.5 if large else 1.6) if end=='rear' else (3.3 if large else 2.2),z-1,z+5,math.pi/4)
            if end=='front':s=holes(s,pcd,4,5.5 if large else 3.8,z+(.9 if large else 1.8),z+5,math.pi/4)
            if end=='rear':
                # Long cage screws enter from the gearbox side and thread into
                # this cover; no proud head may foul the J1 pedestal bore.
                s=holes(s,64 if large else 42,4,2.5 if large else 2.05,z-1,z+5,math.pi/4)
                # Rear outer ring has0.20mm axial float; front outer ring locates.
                s=s.cut(cyl(D/2+.2,z+3.8,z+4.1))
            addpart(size+'_'+end+'_bearing_cover',s);covers.append(s)
        extension=SPEC['brakes'][size]['stack_extension_mm']
        tail=SPEC['folded_transmission'][size]['output_shaft_tail_z_mm']
        flangeR=23 if large else 14
        shoulderR=13.5 if large else 7.5
        # 0.30 mm nominal running clearance between stationary cover and output flange.
        # The outer mating face remains front+12; no link datum is moved.
        shaft=cyl(d/2,tail,front).union(cyl(shoulderR,front,front+4.3)).union(cyl(flangeR,front+4.3,front+12))
        shaft=shaft.cut(cyl(4 if large else 2,tail-1,front+13))
        # Ring groove for DIN 471 is nominal; final groove width/root must be checked
        # against the selected circlip drawing before release.
        groove_w=1.3 if large else 1.1
        shaft=shaft.cut(ring(d/2+.1,9.5 if large else 5.75,rear-.1-groove_w,rear-.1))
        # Rear wrist tail is 8 mm so the compact brake can accept a keyed hub.
        # The 12 mm bearing seats and A12 retaining groove remain unchanged.
        if not large: shaft=shaft.cut(ring(6.1,4.,tail-1,-26.))
        # Key seats: 6x6x36 large, 2x2x20 small. Matching keys/hubs below.
        kw,kh,kz,kl=(6.,3.5,-75.,46.) if large else (2.,1.2,-55.,30.)
        kr=10. if large else 4.
        shaft=shaft.cut(cq.Workplane('XY').box(kh+1,kw,kl).translate((kr-kh/2+.5,0,kz)))
        # Pulley key seat is joint-clocked during drive placement, independently of the brake key.
        shaft=holes(shaft,36 if large else 21,4,4.2 if large else 2.5,front+3,front+13,math.pi/4)
        addpart(size+'_output_shaft',shaft,'steel')
        spacer=ring(d/2+2,d/2+.15,rear+B,front-B)
        addpart(size+'_inner_race_spacer',spacer,'steel')
        return {'d':d,'D':D,'B':B,'sep':sep,'rear':rear,'front':front}

    SUP={k:build_support(k) for k in ['large','small']}

    # DIN471 installed envelopes: ring s=1.2 (A20), 1.0 (A12), not free-state IDs.
    # 0.10mm is a nominal shim only. Select thickness after measuring the complete
    # bearing/shaft stack; do not impose this as an unmeasured bearing preload.
    for size in ['large','small']:
        large=size=='large';d=20 if large else 12;rear=-40 if large else -24
        th=1.2 if large else 1.0;z1=rear-.1;z0=z1-th
        inner=9.5 if large else 5.75;outer=12.1 if large else 7.55
        ret=ring(outer,inner,z0,z1)
        ret=ret.cut(cq.Workplane('XY').box(2.0,10,th+2).translate((0,-outer,z0+th/2)))
        ex=2.2 if large else 1.8;ey=-(outer-.2)
        for x in [-ex,ex]:
            ret=ret.union(cyl(1.8 if large else 1.5,z0,z1).translate((x,ey,0)))
            ret=ret.cut(cyl(1.0 if large else .85,z0-1,z1+1).translate((x,ey,0)))
        addpart(size+'_DIN471_installed_retainer',ret,'steel','Purchased A20/A12 installed clearance envelope; not a free-state supplier drawing')
        addpart(size+'_selective_inner_race_shim',ring(d/2+1.3,d/2+.05,rear-.1,rear),'steel','Nominal 0.10mm selective shim: inspect endplay and choose thickness during assembly')


    # Structural link tubes with matching bolted interfaces.
    def tube(name,w,h,y0,y1,t):
        s=cq.Workplane('XZ').rect(w,h).rect(w-2*t,h-2*t).extrude(y1-y0).translate((0,y1,0))
        ys=[48,58,62,78,92,108,182,198,220,232] if name.startswith('upper') else [48,58,77,93]
        s=drill(s,[(x,y,0) for x in [-20,20] for y in ys],(0,0,1),5.5)
        if name.startswith('fore'):
            s=drill(s,[(0,136,0)],(0,0,1),6.3) # J4 index-sensor cable/body service port
            s=drill(s,[(0,194,z) for z in [-22,22]],(1,0,0),4.5)
            s=drill(s,[(23,y,0) for y in [190,198]],(0,0,1),6)
        return addpart(name,s)
    tube('upper_link_tube',60,44,42,238,2)
    tube('fore_link_tube',60,60,42,204,2)
    upper_ear=cyl(27,-27,-22).union(cq.Workplane('XY').box(60,64,5).translate((0,32,-24.5)))
    upper_ear=holes(upper_ear,36,4,5.5,-28,-21,math.pi/4)
    upper_ear=drill(upper_ear,[(x,y,0) for x in [-20,20] for y in [48,58]],(0,0,1),5.5)
    addpart('upper_root_ear',upper_ear)
    fore_ear=cyl(27,22,27).union(cq.Workplane('XY').box(60,35,5).translate((0,17.5,24.5)))
    fore_ear=fore_ear.union(cq.Workplane('XY').box(60,12,13).translate((0,36,28.5))).union(cq.Workplane('XY').box(60,23,5).translate((0,53.5,32.5)))
    fore_ear=holes(fore_ear,36,4,5.5,21,28,math.pi/4)
    fore_ear=drill(fore_ear,[(x,y,0) for x in [-20,20] for y in [48,58]],(0,0,1),5.5)
    addpart('fore_root_ear',fore_ear)

    def large_mount(cy,zm,ymin):
        s=cq.Workplane('XY').box(72,cy+36-ymin,14).translate((0,(cy+36+ymin)/2,zm))
        s=s.cut(cyl(27.025,zm-8,zm+8).translate((0,cy,0)))
        s=s.cut(cq.Workplane('XY').box(.8,12,18).translate((0,cy+32,zm)))
        s=drill(s,[(0,cy+31,zm+d) for d in [-3.5,3.5]],(1,0,0),4.5)
        return s
    upper_mount=large_mount(280,62,210)
    for x in [-20,20]:upper_mount=upper_mount.union(cq.Workplane('XY').box(14,28,33).translate((x,224,38.5)))
    upper_mount=drill(upper_mount,[(x,y,0) for x in [-20,20] for y in [220,232]],(0,0,1),5.5)
    addpart('upper_elbow_housing_clamp',upper_mount)
    for name,groups in [('upper_nut_strip',[(-17,[48,58]),(17,[220,232])]),('fore_nut_strip',[(25,[48,58])])]:
        s=None
        for z,ys in groups:
            for x in [-20,20]:
                a=cq.Workplane('XY').box(12,max(ys)-min(ys)+20,6).translate((x,(min(ys)+max(ys))/2,z))
                a=drill(a,[(x,y,0) for y in ys],(0,0,1),4.2)
                s=a if s is None else s.union(a)
        addpart(name,s,'steel')

    fore_clamp=cq.Workplane('XY').box(54,14,54).translate((0,194,0))
    fore_clamp=drill(fore_clamp,[(0,194,0)],(0,1,0),36.05)
    fore_clamp=fore_clamp.cut(cq.Workplane('XY').box(12,18,.8).translate((24,194,0)))
    fore_clamp=drill(fore_clamp,[(23,y,0) for y in [190,198]],(0,0,1),3.4)
    fore_clamp=drill(fore_clamp,[(0,194,z) for z in [-22,22]],(1,0,0),3.3)
    addpart('fore_roll_housing_clamp',fore_clamp)
    shim=None
    for x in [-27.5,27.5]:
        ss=cq.Workplane('XY').box(1,14,54).translate((x,194,0))
        ss=drill(ss,[(0,194,z) for z in [-22,22]],(1,0,0),4.5)
        shim=ss if shim is None else shim.union(ss)
    addpart('fore_clamp_side_shims',shim,'steel','Two1mm shims close the clamp-to-tube wall gap')

    # Base service interface: retain central bore and add a real folded-motor bore.
    # A bolted capsule foot spans both holes; separate underside washers leave
    # the folded carrier below the 40 mm bench unobstructed.
    BASE_BOLTS=[(-75,-45),(-75,45),(30,-84),(30,84),(175,-45),(175,45)]
    base=cyl(95,0,12).union(cyl(95,0,12).translate((110,0,0))).union(cq.Workplane('XY').box(110,190,12).translate((55,0,6)))
    base=base.cut(cyl(55,-1,13)).cut(cyl(23,-1,13).translate((110,0,0)))
    for x,y in BASE_BOLTS:base=base.cut(cyl(4.25,-1,13).translate((x,y,0)))
    base=holes(base,130,4,5,-1,13,0)
    addpart('bench_mount_plate',base)
    addpart('bench_backing_washer',ring(14,4.25,-47,-40),'steel','Six separate underside spreader washers preserve service access to the folded carrier')
    addpart('bench_mount_bolt',cyl(4,-52,16).union(cyl(7,16,21)),'steel','Nominal M8 bolt envelope; head and through-hole geometry, no thread helix')
    addpart('bench_mount_nut',ring(7,4.05,-53,-47),'steel','Nominal M8 retaining nut envelope; torque and locking method require assembly specification')
    pedestal=ring(45,27.025,12,64).cut(cyl(36.2,11,20.2))
    pedestal=pedestal.union(ring(80,36.2,12,20))
    pedestal=holes(pedestal,130,4,6.6,11,21,0)
    pedestal=pedestal.cut(cq.Workplane('XY').box(.8,20,56).translate((0,37,38)))
    pedestal=drill(pedestal,[(0,37,z) for z in [30,52]],(1,0,0),5.5)
    addpart('pedestal_sleeve',pedestal)
    turret=large_mount(144,-62,78)
    turret=turret.union(cq.Workplane('XY').box(80,8,134).translate((0,82,-22)))
    f=18/math.sqrt(2)
    turret=drill(turret,[(x,82,z) for x in [-f,f] for z in [-f,f]],(0,1,0),5.5)
    turret=drill(turret,[(x,82,z) for x in [-25,25] for z in [20,35]],(0,1,0),5.5)
    addpart('turret_yoke',turret)

    # A hollow wrist fork leaves the coincident bend/tool center free of a solid shaft.
    fork=cq.Workplane('XY').box(42,6,76).translate((0,-37,0))
    for zz in [-34,34]:
        fork=fork.union(cq.Workplane('XY').box(42,54,8).translate((0,-7,zz)))
        bore=cyl(14.5 if zz<0 else 14,zz-6,zz+6)
        fork=fork.cut(bore)
    fork=holes(fork,32,4,1.6,29,39,math.pi/4)
    fork=fork.cut(cyl(18.025,-38.1,-37))
    f=10.5/math.sqrt(2)
    fork=drill(fork,[(x,-37,z) for x in [-f,f] for z in [-f,f]],(0,1,0),3.4)
    fork=fork.union(ring(25,18.025,-61,-49))
    for x in [-21,21]:fork=fork.union(cq.Workplane('XY').box(6,8,15).translate((x,0,-42.5)))
    fork=fork.cut(cq.Workplane('XY').box(.8,10,14).translate((0,22,-55)))
    fork=drill(fork,[(0,21,z) for z in [-58,-52]],(1,0,0),3.4)
    addpart('wrist_roll_fork',fork)
    cradle=cq.Workplane('XY').box(63,4,58).translate((0,6,0))
    for zz in [-26.5,26.5]:
        p=cq.Workplane('XY').box(36,32,5).translate((0,0,zz))
        p=holes(p,21,4,3.4,zz-4,zz+4,math.pi/4)
        if zz>0:p=p.cut(cyl(3.3,zz-4,zz+4))
        cradle=cradle.union(p)
    for xx in [-28.5,28.5]:cradle=cradle.union(cq.Workplane('XY').box(6,151.6,8).translate((xx,83.8,0)))
    cl=trans_shape(ring(30,18.025,-6,6),(0,159.6,0),(0,1,0))
    cl=cl.cut(cq.Workplane('XY').box(.8,16,14).translate((0,159.6,25)))
    cl=drill(cl,[(0,y,25) for y in [156.6,162.6]],(1,0,0),3.4)
    cradle=cradle.union(cl)
    # The bridge overlaps the ear locally; recut mating holes after the union and
    # provide head recesses open to the bridge edge for the two upper screws.
    for zz in [-26.5,26.5]:cradle=holes(cradle,21,4,3.4,zz-3,zz+3,math.pi/4)
    cradle=holes(cradle,21,4,5.5,-24,-20.8,math.pi/4)
    cradle=drill(cradle,[(0,6,0)],(0,1,0),23.0,L=6) # extended J6 motor passes through the bridge
    addpart('wrist_bend_cradle',cradle)
    stub=cyl(3,18,29).union(cyl(8,29,30)).union(cyl(6,30,42))
    stub=stub.cut(ring(6.1,5.75,38.1,39.2))
    addpart('wrist_idle_trunnion',stub,'steel','Custom M6x1 thread on18..29; journal12k5; circlip groove inspect')
    for name,z,t in [('inner',29.2,.8),('outer',38,2)]:
        addpart('idle_bearing_'+name+'_cover',holes(ring(18,12,z,z+t),32,4,2.2,z-1,z+t+1,math.pi/4),'steel')

    # Gripper bridge, guide carriage and replaceable fingers.
    bridge=cq.Workplane('XY').box(106,22.9,32).translate((0,197.05,0))
    bridge=bridge.cut(cq.Workplane('XY').box(98,30,28).translate((0,204.6,0)))
    bridge=drill(bridge,[(0,193,z) for z in [-10,10]],(1,0,0),4)
    bridge=drill(bridge,[(x,193,z) for x in [-51,51] for z in [-10,10]],(0,1,0),2.5)
    f=10.5/math.sqrt(2)
    bridge=drill(bridge,[(x,187.6,z) for x in [-f,f] for z in [-f,f]],(0,1,0),3.4,L=4.2)
    bridge=drill(bridge,[(x,188.65,z) for x in [-f,f] for z in [-f,f]],(0,1,0),5.5,L=2.1)
    # Two shaft-support posts: front bearing supports integral pinion stub;
    # rear post clamps the Pololu rectangular gearmotor at its gearbox shoulder.
    frontpost=cq.Workplane('XY').box(8,21.4,5).translate((0,196.3,6.5))
    frontpost=frontpost.cut(cyl(3.5,3.5,9.5).translate((0,201.5,0)))
    frontpost=drill(frontpost,[(0,193,10)],(1,0,0),4.4)
    bridge=bridge.union(frontpost)
    rearpost=cq.Workplane('XY').box(34,16.9,8).translate((0,200.05,-12))
    for x in [-14,14]:rearpost=rearpost.union(cq.Workplane('XY').box(6,5,8).translate((x,191.1,-12)))
    rearpost=drill(rearpost,[(0,201.5,-12)],(1,0,0),1.6)
    rearpost=drill(rearpost,[(0,193,-10)],(1,0,0),4.4)
    bridge=bridge.union(rearpost)
    bridge=bridge.cut(cq.Workplane('XY').box(12.15,10.15,16).translate((0,201.5,-12)))
    bridge=drill(bridge,[(x,191,-15) for x in [-20,20]],(0,0,1),2.5,L=6)
    addpart('gripper_bridge',bridge)
    finger=cq.Workplane('XY').box(8,6.4,27.6).translate((5.8,193,0)).union(cq.Workplane('XY').box(8,35.8,20).translate((5.8,214.1,0)))
    finger=drill(finger,[(5.8,193,z) for z in [-10,10]],(1,0,0),6)
    right=finger.cut(cq.Workplane('XY').box(8.5,2.8,6.2).translate((5.8,206.2,0)))
    right=drill(right,[(6,202.5,0)],(0,1,0),1.6,L=9)
    left=finger.cut(cq.Workplane('XY').box(8.5,2.8,6.2).translate((5.8,196.8,0)))
    left=drill(left,[(6,200.5,0)],(0,1,0),1.6,L=9)
    addpart('right_jaw_carriage',right)
    addpart('left_jaw_carriage',left.mirror('YZ'))
    addpart('finger_pad',cq.Workplane('XY').box(1.8,24,18.4).translate((.9,219.2,0)),'rubber','Bonded replaceable silicone insert; bond strength and friction require coupon test')
    addpart('jaw_guide_bushing',ring(3,2.01,-4,4),'polymer')
    addpart('pinion_front_bushing',ring(3.5,1.51,4,9),'steel','Custom oil-impregnated bronze sleeve; CAD density conservatively approximated by steel')

    def spur_gear(m=.3,z=20,b=6,pressure=20):
        """Involute flank nominal with radial root closure; root fillet not released."""
        rp=m*z/2; rb=rp*math.cos(math.radians(pressure)); ra=rp+m; rr=rp-1.25*m
        inv=lambda r: math.sqrt(max(0,(r/rb)**2-1))-math.atan(math.sqrt(max(0,(r/rb)**2-1)))
        half=math.pi/(2*z)
        pts=[]
        for i in range(z):
            a=2*math.pi*i/z
            for r in np.linspace(rr,max(rr,rb),2):
                phi=a-half-inv(rp)
                pts.append((r*math.cos(phi),r*math.sin(phi)))
            for r in np.linspace(max(rr,rb),ra,8):
                phi=a-half-inv(rp)+inv(r)
                pts.append((r*math.cos(phi),r*math.sin(phi)))
            for r in np.linspace(ra,max(rr,rb),8):
                phi=a+half+inv(rp)-inv(r)
                pts.append((r*math.cos(phi),r*math.sin(phi)))
            pts.append((rr*math.cos(a+half+inv(rp)),rr*math.sin(a+half+inv(rp))))
        clean=[]
        for p in pts:
            if not clean or math.dist(p,clean[-1])>1e-7:clean.append(p)
        s=cq.Workplane('XY').polyline(clean).close().extrude(b)
        # Defined 0.06 mm root blend, below the base circle; the active involute is
        # unchanged. A manufacturing process and tooth inspection are still needed.
        root_edges=[]
        for edge in s.val().Edges():
            vv=edge.Vertices();cc=edge.Center()
            if len(vv)==2 and abs(vv[0].Z-vv[1].Z)>b-1e-6 and abs(math.hypot(cc.x,cc.y)-rr)<1e-6:
                root_edges.append(edge)
        if len(root_edges)!=2*z:raise RuntimeError('Unexpected pinion root topology')
        s=s.newObject(root_edges).fillet(.06)
        return s
    pin=spur_gear().translate((0,0,-3)).union(cyl(3.5,-7,-3)).union(cyl(1.5,3,10))
    # Blind shaft socket: manufacturer3mmDprofile must be gauged before broaching.
    # Physical blind D socket, 3 mm nominal shaft and 2.50 mm across-flat.
    # +0.025 radial clearance; factory shaft gauge remains a release gate.
    dsocket=cyl(1.525,-7.1,1).intersect(cq.Workplane('XY').box(10,10,10).translate((-3.975,0,-3)))
    pin=pin.cut(dsocket)
    pin=drill(pin,[(0,-2,-5)],(0,1,0),1.6,L=5)
    addpart('gripper_pinion_m0p3_z20',pin,'steel','Sampled involute with 0.06mm root blend, physical 3.05mm/2.55mm D socket and front journal; active flank metrology, shaft tolerance and process qualification required')
    def rack():
        m=.3;p=math.pi*m;h=m;hf=1.25*m;face=6.;n=50;L=n*p
        s=cq.Workplane('XY').box(L,2.6,face).translate((0,-2.6/2-hf,face/2))
        pitchhalf=p/4;dh=h*math.tan(math.radians(20));df=hf*math.tan(math.radians(20))
        for i in range(n):
            x=(i+.5)*p-L/2
            tooth=cq.Workplane('XY').polyline([(x-pitchhalf-df,-hf),(x-pitchhalf+dh,h),(x+pitchhalf-dh,h),(x+pitchhalf+df,-hf)]).close().extrude(face)
            s=s.union(tooth)
        tongue=cq.Workplane('XY').box(8,2.6,face).translate((L/2+4,-2.6/2-hf,face/2))
        tongue=drill(tongue,[(L/2+4,-1,face/2)],(0,1,0),2.2,L=10)
        return s.union(tongue)
    addpart('gripper_rack_m0p3',rack(),'steel','Custom nominal rack profile; grind and inspect backlash before assembly')


    # ----- P2 replaceable brake cassette components -----
    for size in ['large','small']:
        large=size=='large'; b=SPEC['brakes'][size]
        zback,zfront=(-104.,-48.) if large else (-74.,-32.)
        R,ri,th=(54.,10.2,4.) if large else (27.5,4.2,4.)
        front=ring(R,22.2 if large else 8.5,zfront,zfront+4)
        front=holes(front,64 if large else 42,4,3.3 if large else 2.7,zfront-1,zfront+5,math.pi/4)
        back=ring(R,ri,zback,zback+th)
        back=holes(back,64 if large else 42,4,3.3 if large else 2.7,zback-1,zback+th+1,math.pi/4)
        for angle in [math.pi/4+k*math.pi/2 for k in range(4)]:
            pc=50 if large else 24.5
            pt=(pc*math.cos(angle),pc*math.sin(angle),0)
            for objkey in ['front','back']:
                obj=front if objkey=='front' else back
                obj=obj.cut(cyl(2.1 if large else 1.25,zback-1,zfront+5).translate(pt))
                if objkey=='front':front=obj
                else:back=obj
        back=holes(back,b['mount_pcd_mm'],b['mount_count'],b['mount_clearance_mm'],zback-1,zback+th+1)
        addpart(size+'_brake_front_adapter',front)
        addpart(size+'_brake_back_adapter',back)
        addpart(size+'_brake_guard_strut',ring(4 if large else 2.5,2.1 if large else 1.25,zback+th,zfront),'steel')
        case0=-88 if large else -63; case1=case0+b['length_mm']
        addpart(size+'_brake_vendor_envelope',ring(b['OD_mm']/2,b['ID_mm']/2,case0,case1),'steel',
           'CONFIGURED VENDOR ENVELOPE, not an internal manufacturing model; supplier drawing approval required')
        bore,hubR,keyW,keyD=(10.,15.,6.,3.5) if large else (4.,6.4,2.,1.2)
        hub=ring(hubR,bore+.01,case0,case1+3)
        hub=hub.union(ring(21 if large else 8,bore+.01,case1+1,case1+3))
        hub=holes(hub,36 if large else 13.5,4,3.3 if large else 1.8,case1+.9,case1+3.1,math.pi/4)
        hub=hub.cut(cq.Workplane('XY').box(keyD+1,keyW+.04,b['length_mm']+5).translate((bore+keyD/2-.5,0,(case0+case1)/2)))
        addpart(size+'_brake_keyed_hub',hub,'steel','Custom keyed adapter to configured brake armature; vendor fastening pattern not released')
        addpart(size+'_brake_shaft_key',cq.Workplane('XY').box(6 if large else 2,6 if large else 2,46 if large else 30).translate((9.5 if large else 3.8,0,-75 if large else -55)),'steel')
        zz,dt,root,tip=(-97,3,22,43) if large else (-69,2,10,18)
        cam=ring(root,bore+.015,zz,zz+dt)
        a=.11 if large else .13
        pts=[(r*math.cos(t),r*math.sin(t)) for r,t in [(root-.5,-a),(tip,-a),(tip,a),(root-.5,a)]]
        cam=cam.union(cq.Workplane('XY').workplane(offset=zz).polyline(pts).close().extrude(dt))
        cam=cam.cut(cq.Workplane('XY').box(keyD+1,keyW+.04,dt+2).translate((bore+keyD/2-.5,0,zz+dt/2)))
        addpart(size+'_integral_steel_stop_target',cam,'steel','Integral steel target/stop tab; angular datum and stop impact strength require inspection')
        pinr=2 if large else 1.5
        addpart(size+'_stop_pin',cyl(pinr,zback+1,zz+dt+1),'steel','Retained dowel pin envelope; not an impact-energy-rated stop')
        addpart(size+'_index_sensor_envelope',cyl(2.5,tip+1,tip+17),'steel','M5 radial index sensor envelope; catalog part selection remains open')

    # Lined split clamp: 4.0 mm cable plus 0.5 mm radial liner = 5.0 mm hardware bore.
    cb=cq.Workplane('XY').box(16,7,7)
    cb=drill(cb,[(0,0,0)],(0,1,0),5.,L=10)
    cb=drill(cb,[(x,0,0) for x in [-5.5,5.5]],(0,0,1),2.2,L=10)
    low=cb.intersect(cq.Workplane('XY').box(30,12,7).translate((0,0,-3.5)))
    high=cb.intersect(cq.Workplane('XY').box(30,12,7).translate((0,0,3.5)))
    addpart('gripper_cable_clamp_base',low)
    addpart('gripper_cable_clamp_cap',high)
    addpart('gripper_cable_clamp_liner',trans_shape(ring(2.5,2.,-3.5,3.5),axis=(0,1,0)),'rubber','Split liner; compression and cable pull tests required')
    plate=cq.Workplane('XY').box(50,12,3).translate((0,191,-18))
    plate=drill(plate,[(x,191,-18) for x in [-20,20]],(0,0,1),3.3,L=6)
    plate=drill(plate,[(x,191,-18) for x in [-5.5,5.5]],(0,0,1),1.6,L=6)
    addpart('gripper_cable_clamp_mount',plate)

    # P5 compact shoulder counterbalance: fixed (-65,-40) relative shoulder;
    # moving (0,70) local upper arm. Attachment plates use actual drilled interfaces.
    parent=cq.Workplane('XY').box(110,6,76).translate((-20,89,54))
    parent=drill(parent,[(x,89,z) for x in [-25,25] for z in [20,35]],(0,1,0),5.5,L=10)
    for z in [55,65,79,89]:
        cheek=cq.Workplane('XY').box(18,30,4).translate((-65,100,z))
        cheek=drill(cheek,[(-65,104,z)],(0,0,1),4.2,L=6)
        parent=parent.union(cheek)
    addpart('shoulder_spring_fixed_clevis',parent)
    child=cq.Workplane('XY').box(58,28,5).translate((0,70,24.5))
    child=drill(child,[(x,y,24.5) for x in [-20,20] for y in [62,78]],(0,0,1),5.5,L=8)
    child=child.union(cq.Workplane('XY').box(16,10,67).translate((0,70,56)))
    for z in [60,84]:
        child=child.cut(cq.Workplane('XY').box(24,20,6).translate((0,76,z)))
        child=drill(child,[(0,70,z)],(0,0,1),4.2,L=16)
    addpart('shoulder_spring_moving_clevis',child)
    addpart('spring_eye_pin',cyl(2,-8,8),'steel','4 mm retained pin; washers/cotters and bearing stress to be qualified')
    strip=cq.Workplane('XY').box(55,28,4).translate((0,70,18))
    strip=drill(strip,[(x,y,18) for x in [-20,20] for y in [62,78]],(0,0,1),4.2,L=8)
    addpart('shoulder_spring_nut_plate',strip,'steel')
    # CSG_DYNAMIC_SPRING_BEGIN
    # Canonical helical spring BREP at q=0 installed length. Renderer changes pitch
    # with the actual anchor motion. Hook formation and fatigue remain unqualified.
    ss=SPEC['counterbalance']; L0=ss['reference_length_m']*1000
    coilLength=L0-18; pitch=coilLength/ss['active_turns']; rr=ss['mean_coil_diameter_mm']/2
    helix=cq.Wire.makeHelix(pitch,coilLength,rr)
    profile=cq.Plane(origin=(rr,0,0),xDir=(1,0,0),normal=(0,rr,pitch/(2*math.pi)))
    coil=cq.Workplane(profile).circle(ss['wire_diameter_mm']/2).sweep(cq.Workplane(obj=helix),isFrenet=True).translate((0,0,9))
    for ze in [0,L0]:
        eye=cq.Workplane(obj=cq.Solid.makeTorus(3.6,1.4)).rotate((0,0,0),(1,0,0),90).translate((0,0,ze))
        pa=np.array([3.6,0.,ze]);pb=np.array([rr,0.,9. if ze==0 else L0-9.]);vv=pb-pa
        tail=trans_shape(cyl(1.4,0,float(np.linalg.norm(vv))),pa,vv)
        for pp in [pa,pb]:tail=tail.union(cq.Workplane('XY').sphere(1.4).translate(tuple(pp)))
        coil=coil.union(eye).union(tail)
    addpart('shoulder_extension_spring',coil,'steel','Proposed coil/eye geometry; initial tension, hook form and fatigue require spring supplier review')


    # CSG_DYNAMIC_SPRING_END
    # P5 elbow counterbalance in the existing link envelope: no mast past elbow.
    elbow_fixed=cq.Workplane('XY').box(60,32,6).translate((0,190,-25))
    elbow_fixed=elbow_fixed.union(cq.Workplane('XY').box(44,16,6).translate((-22,190,-25)))
    elbow_fixed=drill(elbow_fixed,[(x,y,-25) for x in [-20,20] for y in [182,198]],(0,0,1),5.5,L=10)
    elbow_fixed=elbow_fixed.union(cq.Workplane('XY').box(16,10,59).translate((-30,190,-54.5)))
    for z in [-55,-79]:
        elbow_fixed=elbow_fixed.cut(cq.Workplane('XY').box(24,20,6).translate((-30,196,z)))
        elbow_fixed=drill(elbow_fixed,[(-30,190,z)],(0,0,1),4.2,L=16)
    addpart('elbow_spring_fixed_clevis',elbow_fixed)
    fixednut=None
    for x in [-20,20]:
        a=cq.Workplane('XY').box(12,32,6).translate((x,190,-17))
        a=drill(a,[(x,y,-17) for y in [182,198]],(0,0,1),4.2,L=10)
        fixednut=a if fixednut is None else fixednut.union(a)
    addpart('elbow_fixed_anchor_nut_strips',fixednut,'steel')
    # Forked back lever attaches to the existing root-ear flange at its true holes.
    elbow_child=cyl(27,-36,-30).union(cq.Workplane('XY').box(24,45,6).translate((0,-22.5,-33)))
    elbow_child=elbow_child.union(cq.Workplane('XY').box(58,70,6).translate((0,35,-33)))
    elbow_child=drill(elbow_child,[(x,y,-33) for x in [-20,20] for y in [48,58]],(0,0,1),5.5,L=10)
    elbow_child=holes(elbow_child,36,4,5.5,-37,-29,math.pi/4)
    elbow_child=elbow_child.union(cq.Workplane('XY').box(16,10,54).translate((0,-45,-57)))
    for z in [-55,-79]:
        elbow_child=elbow_child.cut(cq.Workplane('XY').box(24,20,6).translate((0,-39,z)))
        elbow_child=drill(elbow_child,[(0,-45,z)],(0,0,1),4.2,L=16)
    addpart('elbow_spring_moving_clevis',elbow_child)
    nut=None
    for xx in [-20,20]:
        nn=cq.Workplane('XY').box(12,32,6).translate((xx,53,-25))
        nn=drill(nn,[(xx,y,-25) for y in [48,58]],(0,0,1),4.2,L=10)
        nut=nn if nut is None else nut.union(nn)
    addpart('elbow_moving_anchor_nut_plate',nut,'steel','Short lever bolts into negative-wall nut strips at existing forearm tube holes; not floating at the joint')
    # CSG_DYNAMIC_SPRING_BEGIN
    ss=SPEC['elbow_counterbalance'];L0=ss['reference_length_m']*1000;lead=ss['hook_span_total_mm']/2
    coilLength=L0-2*lead;pitch=coilLength/ss['active_turns'];rr=ss['mean_coil_diameter_mm']/2;wr=ss['wire_diameter_mm']/2
    helix=cq.Wire.makeHelix(pitch,coilLength,rr)
    profile=cq.Plane(origin=(rr,0,0),xDir=(1,0,0),normal=(0,rr,pitch/(2*math.pi)))
    coil=cq.Workplane(profile).circle(wr).sweep(cq.Workplane(obj=helix),isFrenet=True).translate((0,0,lead))
    for ze in [0,L0]:
        eye=cq.Workplane(obj=cq.Solid.makeTorus(3.1,wr)).rotate((0,0,0),(1,0,0),90).translate((0,0,ze))
        pa=np.array([3.1,0.,ze]);pb=np.array([rr,0.,lead if ze==0 else L0-lead]);vv=pb-pa
        tail=trans_shape(cyl(wr,0,float(np.linalg.norm(vv))),pa,vv)
        for pp in [pa,pb]:tail=tail.union(cq.Workplane('XY').sphere(wr).translate(tuple(pp)))
        coil=coil.union(eye).union(tail)
    addpart('elbow_extension_spring',coil,'steel','Proposed extension spring: geometry and mass computed; winding initial tension, hook and fatigue require supplier qualification')
    # CSG_DYNAMIC_SPRING_END



    def install_brake(parent,child,size,origin,axis,label,child_origin):
        large=size=='large'; b=SPEC['brakes'][size]
        ppart(parent,size+'_brake_front_adapter',origin,axis)
        for k in range(4):
            a=math.pi/4+k*math.pi/2;rad=50 if large else 24.5
            shape=PARTS[size+'_brake_guard_strut'].translate((rad*math.cos(a),rad*math.sin(a),0))
            place(parent,label+'_brake_strut_'+str(k),trans_shape(shape,origin,axis),'steel')
        place(parent,label+'_brake_stator',trans_shape(PARTS[size+'_brake_vendor_envelope'],origin,axis),'steel',
              b['mass_kg']*.9,'Vendor brake mass split: 90% stator, 10% armature engineering assumption')
        z=-50.4 if large else -37.3
        arm=ring(40 if large else 17.5,15.1 if large else 6.5,z,z+1)
        arm=holes(arm,36 if large else 13.5,4,3.3 if large else 1.8,z-.1,z+1.1,math.pi/4)
        place(child,label+'_brake_armature',trans_shape(arm,child_origin,axis),'steel',b['mass_kg']*.1,
              '10% vendor brake mass allocation; rotational inertia not measured')
        ppart(child,size+'_brake_keyed_hub',child_origin,axis)
        ppart(child,size+'_brake_shaft_key',child_origin,axis)
        i=int(label[1:])-1; sign=[-1,-1,1,1,-1,1][i]
        qhome=[-.34,.20,1.50,0,1.20,3.14][i];phi0=math.pi/2-sign*qhome
        # Clock only the integral tab, not the keyway: the shaft key remains at +X.
        zz,dt,root,tip=(-97,3,22,43) if large else (-69,2,10,18)
        bore,kw,kd=(10.,6.,3.5) if large else (4.,2.,1.2)
        half=.11 if large else .13
        pts=[(rr*math.cos(aa),rr*math.sin(aa)) for rr,aa in [(root-.5,phi0-half),(tip,phi0-half),(tip,phi0+half),(root-.5,phi0+half)]]
        target=ring(root,bore+.015,zz,zz+dt).union(cq.Workplane('XY').workplane(offset=zz).polyline(pts).close().extrude(dt))
        target=target.cut(cq.Workplane('XY').box(kd+1,kw+.04,dt+2).translate((bore+kd/2-.5,0,zz+dt/2)))
        addpart(label+'_clocked_stop_target',target,'steel','Joint-specific tab angle; shaft key datum stays +X')
        place(child,label+'_integral_stop_target',trans_shape(target,child_origin,axis),'steel')
        r=32 if large else 15;pr=2 if large else 1.5;half=.11 if large else .13
        low,high=sorted([phi0+sign*q for q in SPEC['joint_hard_limits_rad'][i]])
        backplate=PARTS[size+'_brake_back_adapter']
        for k,ang in enumerate([low-half-math.asin(pr/r),high+half+math.asin(pr/r)]):
            pin=PARTS[size+'_stop_pin'].translate((r*math.cos(ang),r*math.sin(ang),0))
            backplate=backplate.cut(cyl(pr,-110,-60).translate((r*math.cos(ang),r*math.sin(ang),0)))
            place(parent,label+'_stop_pin_'+str(k),trans_shape(pin,origin,axis),'steel')
        addpart(label+'_stop_pin_backplate',backplate,'aluminum','Joint-specific retained stop dowel holes; impact strength unqualified')
        place(parent,label+'_brake_back_adapter',trans_shape(backplate,origin,axis))
        zz=-95.5 if large else -68
        sensor=trans_shape(PARTS[size+'_index_sensor_envelope'],(0,0,zz),(0,1,0))
        place(parent,label+'_index_sensor',trans_shape(sensor,origin,axis),'steel',.004,'M5 sensor mass allowance; not selected catalog hardware')
        tip=43 if large else 18
        block=cq.Workplane('XY').box(10,7,8).translate((0,tip+10,zz))
        block=drill(block,[(0,tip+10,zz)],(0,1,0),5.1,L=10)
        # Support lug reaches the cassette backplate, rather than floating in space.
        zb=-100 if large else -70
        foot=cq.Workplane('XY').box(10,8,abs(zz-zb)+4).translate((0,tip+7,(zz+zb)/2+2))
        block=block.union(foot)
        block=drill(block,[(0,tip+10,zz)],(0,1,0),5.1,L=20)
        block=drill(block,[(xx,tip+7,zb+2) for xx in [-3,3]],(0,0,1),2.2,L=12)
        addpart(label+'_index_sensor_mount',block,'aluminum','Bolted radial M5 sensor mount; keeps the sensing face unobstructed')
        place(parent,label+'_sensor_mount',trans_shape(block,origin,axis))
        case0=-88 if large else -63;back=-100 if large else -70
        for k in range(b['mount_count']):
            a=2*math.pi*k/b['mount_count'];pc=b['mount_pcd_mm']/2
            spacer=ring(3.5 if large else 2.5,b['mount_clearance_mm']/2,back,case0).translate((pc*math.cos(a),pc*math.sin(a),0))
            place(parent,label+'_brake_spacer_'+str(k),trans_shape(spacer,origin,axis),'steel')

    # P5 folded transmission, actual nominal tooth geometry and adjustable mounting.
    # Canonical stage lies along +Y; each joint rotates it to the chosen packing direction.
    TRANSMISSIONS=[];P5_PROXIES=[];P5_CHECKS=[]
    def capsule(radius,centres,z0,z1):
        return cyl(radius,z0,z1).union(cyl(radius,z0,z1).translate((0,centres,0))).union(cq.Workplane('XY').box(2*radius,centres,z1-z0).translate((0,centres/2,(z0+z1)/2)))
    def slot_hole(radius,travel,z0,z1):
        return cyl(radius,z0,z1).translate((0,-travel/2,0)).union(cyl(radius,z0,z1).translate((0,travel/2,0))).union(cq.Workplane('XY').box(2*radius,travel,z1-z0).translate((0,0,(z0+z1)/2)))
    def belt_path(s,r,c):
        h=math.pi*r
        if s<h:
            a=math.pi+s/r;return np.array([r*math.cos(a),r*math.sin(a)]),np.array([math.cos(a),math.sin(a)])
        s-=h
        if s<c:return np.array([r,s]),np.array([1.,0.])
        s-=c
        if s<h:
            a=s/r;return np.array([r*math.cos(a),c+r*math.sin(a)]),np.array([math.cos(a),math.sin(a)])
        s-=h;return np.array([-r,c-s]),np.array([-1.,0.])
    def build_folded_stage(size):
        large=size=='large';f=SPEC['folded_transmission'][size];pitch=f['pitch_mm'];nt=f['pulley_teeth'];w=f['belt_width_mm'];c=f['center_distance_mm'];pr=pitch*nt/(2*math.pi);zc=f['pulley_center_z_mm'];front=f['gearhead_front_z_mm'];toothr=1.20 if large else .70;groover=1.35 if large else .82;off=.60 if large else .35
        def pulley(bore,input=False):
            phase=(math.pi/nt if round(c/pitch*2)%2 else 0.) if input else 0.
            p=cyl(pr-(.57 if large else .38),zc-w/2-.2,zc+w/2+.2)
            for k in range(nt):
                a=phase+2*math.pi*k/nt;p=p.cut(cyl(groover,zc-w/2-1,zc+w/2+1).translate(((pr-off)*math.cos(a),(pr-off)*math.sin(a),0)))
            for z in [zc-w/2-1.4,zc+w/2+.2]:p=p.union(cyl(pr+2.8,z,z+1.2))
            hubr=15 if large and not input else 11 if large else 8 if not input else 6
            p=p.union(cyl(hubr,zc-w/2-3.,front-2.))
            p=p.cut(cyl(bore+.015,zc-w/2-4,front+1))
            kw=6 if large and not input else 4 if large else 2
            kd=3.5 if large and not input else 2.3 if large else 1.2
            p=p.cut(cq.Workplane('XY').box(kd+1,kw+.04,w+10).translate((bore+kd/2-.5,0,zc)))
            return p
        addpart(size+'_folded_driven_pulley',pulley(10 if large else 4),'aluminum','Custom 1:1 synchronous pulley, rounded nominal tooth section; supplier cutter and fatigue qualification required')
        addpart(size+'_folded_drive_pulley',pulley(6 if large else 3.5,True),'aluminum','Gearhead-side keyed pulley; explicit tooth phase matches finite closed belt')
        back=capsule(pr+(2.30 if large else 1.50),c,zc-w/2,zc+w/2).cut(capsule(pr-(.40 if large else .25),c,zc-w/2-1,zc+w/2+1))
        for k in range(f['belt_teeth']):
            pt,n= belt_path(k*pitch,pr,c);pt=pt-off*n
            back=back.union(cyl(toothr,zc-w/2,zc+w/2).translate((pt[0],pt[1],0)))
        addpart(size+'_folded_timing_belt',back,'rubber','Actual 80/75-tooth closed nominal belt geometry; mass uses homogeneous elastomer approximation')
        # Carrier plate integrates the brake attachment and slotted reducer plate.
        rr=36 if large else 21;outR=56 if large else 29.5
        deck=capsule(rr,c,front-4,front).union(cyl(outR,front-4,front))
        deck=deck.cut(cyl(16 if large else 8.5,front-5,front+1))
        deck=deck.cut(slot_hole(14.2 if large else 7.7,4,front-5,front+1).translate((0,c,0)))
        pcd=35 if large else 19;bn=4 if large else 6
        for k in range(bn):
            a=(math.pi/4 if large else 0)+2*math.pi*k/bn
            pt=(pcd/2*math.cos(a),c+pcd/2*math.sin(a),0)
            deck=deck.cut(slot_hole(2.25 if large else 1.25,4,front-5,front+1).translate(pt))
        # Bolts into retained brake backplate: output load-bearing connection.
        for k in range(4):
            a=math.pi/4+k*math.pi/2;r=50 if large else 24.5
            deck=deck.cut(cyl(2.2 if large else 1.4,front-5,front+1).translate((r*math.cos(a),r*math.sin(a),0)))
        addpart(size+'_folded_carrier_deck',deck)
        # Removable perimeter enclosure: the cover and rails support the input bearing.
        cover0=zc-w/2-(14.5 if large else 13.5);cover1=cover0+3
        inner=pr+4.5;outer=inner+3
        wall=capsule(outer,c,cover1,front-4).cut(capsule(inner,c,cover1-1,front-3))
        # Windows retain continuous top/bottom rails while making the belt inspectable.
        window=cq.Workplane('XY').box(2*outer+4,max(8,c-2*outer),max(4,front-4-cover1-8)).translate((0,c/2,(cover1+front-4)/2))
        wall=wall.cut(window)
        addpart(size+'_folded_housing_rails',wall)
        cover=capsule(outer,c,cover0,cover1)
        cover=cover.cut(cyl(10.4 if large else 4.4,cover0-1,cover1+1))
        br=14 if large else 11;shaft=6 if large else 3.5;bl=8 if large else 6
        cover=cover.cut(cyl(br+.02,cover0-1,cover1+1).translate((0,c,0)))
        # Bearing pocket lip takes axial load without filling the radial bore.
        cover=cover.union(ring(br+3,br+.02,cover0-2,cover1+3).translate((0,c,0)))
        cover=cover.union(ring(br+3,shaft+1.,cover0-3.5,cover0-2).translate((0,c,0)))
        # Inspection window is genuinely cut from the plate, leaving two structural rails.
        cover=cover.cut(cq.Workplane('XY').box(2*max(4,pr-6),max(8,c-2*(pr+7)),6).translate((0,c/2,(cover0+cover1)/2)))
        addpart(size+'_folded_front_cover',cover)
        addpart(size+'_folded_input_support_bearing',ring(br,shaft,cover0-2,cover0-2+bl),'steel','Additional outboard bearing envelope; exact series, preload and life require supplier approval')
        inputshaft=cyl(shaft,cover0-3,front+10)
        inputshaft=inputshaft.cut(cq.Workplane('XY').box(4.1 if large else 2.1,4.04 if large else 2.04,w+4).translate((5.8 if large else 3.3,0,zc)))
        addpart(size+'_folded_input_shaft',inputshaft,'steel','Keyed input extension and outboard bearing journal; actual configured shaft/key interface must be approved')
        # Two opposed jackscrews adjust reducer axis in slotted mount ±2mm.
        block=cq.Workplane('XY').box(12 if large else 8,8,7).translate((0,c+rr-3,front+3.5))
        block=drill(block,[(0,c+rr-3,front+3.5)],(0,1,0),3.4 if large else 2.5,L=12)
        addpart(size+'_folded_tension_block',block)
        screw=trans_shape(cyl(2. if large else 1.5,-9,9).union(cyl(3.5 if large else 2.7,9,12)),(0,c+rr-4,front+3.5),(0,1,0))
        addpart(size+'_folded_tension_screw',screw,'steel','Nominal adjustment screw; thread helix omitted, locked after setting measured belt tension')
        return {'pitch_radius_mm':pr,'center_distance_mm':c,'cover_z_mm':[cover0,cover1],'bearing_z_mm':[cover0-2,cover0-2+bl],'nominal_tooth_tip_clearance_mm':groover-toothr,'gearhead_pilot_slide_mm':4.,'input_pulley_part':size+'_folded_drive_pulley','belt_part':size+'_folded_timing_belt'}
    STAGES={k:build_folded_stage(k) for k in ['large','small']}
    def drive(parent,child,size,origin,axis,label,ratio,child_offset=(0,0,0)):
        large=size=='large';f=SPEC['folded_transmission'][size];stage=STAGES[size];C=f['center_distance_mm'];front=f['gearhead_front_z_mm'];Lg=f['gearhead_length_mm'];Lm=f['motor_length_mm'];child_origin=tuple(np.array(origin)-np.array(child_offset))
        # Canonical +Y parallel-axis stage is clocked to avoid the real link structures.
        angles={'J1':-90.,'J2':math.degrees(math.atan2(-75.,-math.sqrt(C*C-75.*75.))) if large else 0.,'J3':0.,'J4':0.,'J5':90.,'J6':0.}
        # Rz(theta)*(0,C) = (-C*sin(theta), C*cos(theta)).
        theta=angles[label]
        if label=='J2':theta=math.degrees(math.atan2(75.,-math.sqrt(C*C-75.*75.)))
        def orient(sh,down=False):return trans_shape(sh.rotate((0,0,0),(0,0,1),theta),child_origin if down else origin,axis)
        offset=np.array([-C*math.sin(math.radians(theta)),C*math.cos(math.radians(theta)),0.])
        start=len(MESHES)
        for name in ['bearing_housing','rear_bearing_cover','front_bearing_cover']:ppart(parent,size+'_'+name,origin,axis)
        for name in ['inner_race_spacer','DIN471_installed_retainer','selective_inner_race_shim']:ppart(child,size+'_'+name,child_origin,axis)
        kw,kh,kr=(6.,3.5,10.) if large else (2.,1.2,4.)
        keyseat=cq.Workplane('XY').box(kh+1,kw+.04,34 if large else 23).translate((kr-kh/2+.5,0,f['pulley_center_z_mm'])).rotate((0,0,0),(0,0,1),theta)
        shaft=PARTS[size+'_output_shaft'].cut(keyseat)
        addpart(label+'_clocked_output_shaft',shaft,'steel','Brake key datum +X; pulley key seat clocked to the actual folded transmission')
        place(child,label+'_output_shaft',trans_shape(shaft,child_origin,axis),'steel')
        sp=SUP[size]
        for j in [0,-sp['sep']]:place(parent,label+'_bearing_'+str(j),trans_shape(ring(sp['D']/2,sp['d']/2,j-sp['B']/2,j+sp['B']/2),origin,axis),'steel',.068 if large else .0201,'Retained catalog bearing mass, annular envelope')
        install_brake(parent,child,size,origin,axis,label,child_origin)
        place(parent,label+'_gearhead',orient(cyl(21 if large else 11,front,front+Lg).translate((0,C,0))),'steel',.460 if large else .078,'Original full maxon gearhead envelope and catalog mass; now folded alongside output axis')
        place(parent,label+'_motor',orient(cyl(20 if large else 11,front+Lg,front+Lg+Lm).translate((0,C,0))),'steel',.390 if large else .110,'Original full maxon motor envelope and catalog mass; not rescaled')
        for name in ['carrier_deck','housing_rails','front_cover','tension_block','tension_screw','timing_belt']:
            part=size+'_folded_'+name;place(parent,label+'_'+name,orient(PARTS[part]),META[part]['material'])
        for name in ['drive_pulley','input_support_bearing','input_shaft']:
            part=size+'_folded_'+name;place(parent,label+'_'+name,orient(PARTS[part].translate((0,C,0))),META[part]['material'])
        part=size+'_folded_driven_pulley';place(child,label+'_driven_pulley',orient(PARTS[part],True),META[part]['material'])
        # Keys are separate real solids and can be removed in the inspection film.
        zc=f['pulley_center_z_mm'];w=f['belt_width_mm']
        outputkey=cq.Workplane('XY').box(6 if large else 2,6 if large else 2,w+3).translate((9.5 if large else 3.8,0,zc))
        place(child,label+'_pulley_output_key',orient(outputkey,True),'steel')
        inputkey=cq.Workplane('XY').box(4 if large else 2,4 if large else 2,w+3).translate((5.8 if large else 3.3,C,zc))
        place(parent,label+'_pulley_input_key',orient(inputkey),'steel')
        # Mount bolts through the actual slotted gearhead pattern.
        for k in range(4 if large else 6):
            a=(math.pi/4 if large else 0)+2*math.pi*k/(4 if large else 6);rad=17.5 if large else 9.5;pt=(rad*math.cos(a),C+rad*math.sin(a),0)
            bolt=cyl(2 if large else 1.,front-6,front+5).union(cyl(3.5 if large else 2.,front-8.5,front-6)).translate(pt)
            place(parent,label+'_gear_mount_screw_'+str(k),orient(bolt),'steel')
        for k in range(4):
            a=math.pi/4+k*math.pi/2;r=50 if large else 24.5;pt=(r*math.cos(a),r*math.sin(a),0)
            bolt=cyl(2 if large else 1.25,front-8,front+4).union(cyl(3.5 if large else 2.2,front-10.5,front-8)).translate(pt)
            place(parent,label+'_carrier_screw_'+str(k),orient(bolt),'steel')
        # Input inertia is joint-side because the belt is 1:1. Belt transport energy
        # is represented by m_belt * r_pitch^2, additional to housing-body inertia.
        ip=PARTS[size+'_folded_drive_pulley'];m,cc,I=inertia_shape(ip,RHO['aluminum']);bm=META[size+'_folded_timing_belt']['mass_kg']
        shm,shc,shI=inertia_shape(PARTS[size+'_folded_input_shaft'],RHO['steel'])
        km,kc,kI=inertia_shape(inputkey,RHO['steel']);rad2=float(kc[0]**2+(kc[1]-C*.001)**2)
        extra=float(I[2,2]+shI[2,2]+kI[2,2]+km*rad2+bm*(stage['pitch_radius_mm']*.001)**2)
        geometry_records=[{'type':'motor','origin_mm':origin,'axis':axis,'offset_local_mm':offset.tolist(),'z_mm':[front+Lg,front+Lg+Lm],'r_mm':20 if large else 11},{'type':'gearhead','origin_mm':origin,'axis':axis,'offset_local_mm':offset.tolist(),'z_mm':[front,front+Lg],'r_mm':21 if large else 11},{'type':'bearing_housing','origin_mm':origin,'axis':axis,'z_mm':[-40,10] if large else [-24,8],'r_mm':27 if large else 18}]
        primitive_records.append({'name':label,'parent_body':parent,'origin_mm':origin,'axis':axis,'segments':geometry_records})
        TRANSMISSIONS.append({'joint':label,'parent':parent,'child':child,'origin_mm':origin,'child_origin_mm':child_origin,'axis':axis,'input_offset_canonical_mm':[0,C,0],'clocking_degrees':theta,'drive_pulley_name':label+'_drive_pulley','driven_pulley_name':label+'_driven_pulley','ratio':1.,'pitch_radius_mm':stage['pitch_radius_mm'],'belt_mass_kg':bm,'extra_joint_armature_kg_m2':extra,'carrier_slot_travel_mm':4.,'nominal_tooth_tip_clearance_mm':stage['nominal_tooth_tip_clearance_mm'],'motor_envelope_mm':[2*(20 if large else 11),Lm],'gearhead_envelope_mm':[2*(21 if large else 11),Lg],'input_axis_local_offset_mm':offset.tolist(),'cover_z_mm':stage['cover_z_mm']})
        # Transform a point through the same construction used by the CAD placement.
        av=np.array(axis,float);av/=np.linalg.norm(av);zz=np.array([0.,0.,1.]);vv=np.cross(zz,av);an=math.acos(np.clip(zz@av,-1,1))
        if np.linalg.norm(vv)<1e-10:Q=np.eye(3) if av[2]>0 else np.diag([1.,-1.,-1.])
        else:
            vv/=np.linalg.norm(vv);K=np.array([[0,-vv[2],vv[1]],[vv[2],0,-vv[0]],[-vv[1],vv[0],0]]);Q=np.eye(3)+math.sin(an)*K+(1-math.cos(an))*(K@K)
        for seg in geometry_records[:2]:
            pts=[(np.array(origin)+Q@(offset+np.array([0,0,z]))).tolist() for z in seg['z_mm']]
            P5_PROXIES.append({'name':'P5_'+label+'_'+seg['type'],'body':parent,'type':'cylinder','fromto_mm':pts,'radius_mm':seg['r_mm']})
        for rec in MESHES[start:]:rec['mechanism']=label


    # Motor/reducer and bearing housing are fixed to upstream bodies. Output shafts
    # belong to downstream bodies. Transform shaft origins into the child frame
    # after collecting assemblies; frames coincide at q=0 except link translations.
    drive('fixed','turret','large',(0,60,0),(0,1,0),'J1',42.875)
    drive('turret','upper','large',(0,144,-45),(0,0,1),'J2',2197/27,(0,144,0))
    drive('upper','fore','large',(0,280,45),(0,0,-1),'J3',2197/27,(0,280,0))
    drive('fore','roll','small',(0,204,0),(0,1,0),'J4',41553/325,(0,260,0))
    drive('roll','bend','small',(0,0,-45),(0,0,1),'J5',41553/325)
    drive('bend','tool','small',(0,169.6,0),(0,1,0),'J6',41553/325)
    for name in ['bench_mount_plate','pedestal_sleeve']:ppart('fixed',name,axis=(0,1,0))
    for x,y in BASE_BOLTS:
        for name in ['bench_backing_washer','bench_mount_bolt','bench_mount_nut']:
            ppart('fixed',name,(x,0,-y),(0,1,0))
    ppart('turret','turret_yoke')
    for body in ['upper','fore']:
        ppart(body,body+'_link_tube')
        ppart(body,body+'_root_ear')
        ppart(body,body+'_nut_strip')
    ppart('upper','upper_elbow_housing_clamp')
    ppart('fore','fore_roll_housing_clamp')
    ppart('fore','fore_clamp_side_shims')
    ppart('roll','wrist_roll_fork')
    for name in ['inner','outer']:ppart('roll','idle_bearing_'+name+'_cover')
    place('roll','J5_idle_bearing',ring(14,6,30,38),'steel',.0201,'SKF6001-2RSH vendor mass; annular envelope inertia')
    ppart('bend','wrist_bend_cradle');ppart('bend','wrist_idle_trunnion')
    ppart('tool','gripper_bridge')
    place('tool','gripper_motor',cq.Workplane('XY').box(12,10,32).translate((0,201.5,-24)),mass=.012,note='Pololu envelope; mass engineering allowance')
    ppart('tool','gripper_pinion_m0p3_z20',(0,201.5,0))
    ppart('tool','pinion_front_bushing',(0,201.5,0))
    for z in [-10,10]:place('tool','guide_rod_'+str(z),trans_shape(cyl(2,-53,53),(0,193,z),(1,0,0)),'steel')
    # Canonical right jaw frame: inner contact planeX=0, pad centerY=0.
    ppart('jaw','right_jaw_carriage',(0,-219.2,0));ppart('jaw','finger_pad',(0,-219.2,0))
    for z in [-10,10]:ppart('jaw','jaw_guide_bushing',(5.8,193-219.2,z),(1,0,0))
    rack_right=PARTS['gripper_rack_m0p3'].mirror('XZ').translate((2-50*math.pi*.3/2,204.5-219.2,-3))
    place('jaw','upper_rack',rack_right,'steel')
    # Retain the separately authored handed jaw. Its mass is integrated from
    # these actual components below, rather than assuming a mirrored carriage.
    leftjaw=cq.Assembly(name='left_jaw')
    leftjaw.add(PARTS['left_jaw_carriage'],name='carriage')
    leftjaw.add(PARTS['finger_pad'].mirror('YZ'),name='pad')
    leftjaw.add(PARTS['gripper_rack_m0p3'].mirror('YZ').translate((50*math.pi*.3/2-2,198.5,-3)),name='lower_rack')
    for z in [-10,10]:leftjaw.add(trans_shape(PARTS['jaw_guide_bushing'],(-5.8,193,z),(1,0,0)),name='guide_bushing_'+str(z))
    construction.save_auxiliary_assembly('left_jaw', leftjaw)
    construction.place_assembly('jaw_opposed', leftjaw, {
        'left_jaw/carriage': META['left_jaw_carriage']['material'],
        'left_jaw/pad': META['finger_pad']['material'],
        'left_jaw/lower_rack': META['gripper_rack_m0p3']['material'],
        **{'left_jaw/guide_bushing_'+str(z): META['jaw_guide_bushing']['material'] for z in [-10,10]},
    }, offset_mm=(0,-219.2,0), component_names={
        'left_jaw/carriage': 'left_jaw_carriage', 'left_jaw/pad': 'finger_pad',
        'left_jaw/lower_rack': 'lower_rack',
        **{'left_jaw/guide_bushing_'+str(z): 'guide_bushing_'+str(z) for z in [-10,10]},
    }, provenance='Actual authored opposed jaw component CAD density; source Y-219.2 mm joint-local frame. CAD placements and inertia share the same located solids')


    ppart('tool','gripper_cable_clamp_mount')
    for name in ['base','cap','liner']:ppart('tool','gripper_cable_clamp_'+name,(0,191,-23))
    ppart('turret','shoulder_spring_fixed_clevis')
    ppart('upper','shoulder_spring_moving_clevis')
    ppart('upper','shoulder_spring_nut_plate')
    for z in [60,84]:
        ppart('turret','spring_eye_pin',(-65,104,z))
        ppart('upper','spring_eye_pin',(0,70,z))
    ms=META['shoulder_extension_spring']['mass_kg']
    for z in [.060,.084]:
        for b,c in [('turret',[-.065,.104,z]),('upper',[0,.070,z])]:
            BODIES[b].append({'name':'spring_mass_allocation','mass_kg':ms/2,'com_m':c,
                'inertia_at_com_kg_m2':(np.eye(3)*ms/2*.009**2/2).tolist(),
                'provenance':'Half spring mass at each actual anchor; flexible-body mass approximation'})


    ppart('upper','elbow_spring_fixed_clevis')
    ppart('upper','elbow_fixed_anchor_nut_strips')
    ppart('fore','elbow_spring_moving_clevis')
    ppart('fore','elbow_moving_anchor_nut_plate')
    for z in [-55,-79]:
        ppart('upper','spring_eye_pin',(-30,190,z))
        ppart('fore','spring_eye_pin',(0,-45,z))
    ms=META['elbow_extension_spring']['mass_kg']
    for z in [-.055,-.079]:
        for b,c in [('upper',[-.030,.190,z]),('fore',[0,-.045,z])]:
            BODIES[b].append({'name':'elbow_spring_mass_allocation','mass_kg':ms/2,'com_m':c,
                'inertia_at_com_kg_m2':(np.eye(3)*ms/2*.0075**2/2).tolist(),
                'provenance':'Half spring mass at each actual elbow anchor; flexible-body mass approximation'})

    # Wire/encoder/fastener reserve is explicit; not treated as measured hardware.
    reserve={'turret':(.10,[0,.09,-.04]),'upper':(.08,[0,.14,0]),'fore':(.06,[0,.122,0]),'roll':(.025,[0,0,-.025]),'bend':(.025,[0,.08,0]),'tool':(.06,[0,.195,0])}
    for b,(m,c) in reserve.items():BODIES[b].append({'name':'fastener_encoder_cable_allowance','mass_kg':m,'com_m':c,'inertia_at_com_kg_m2':(np.eye(3)*m*.02**2/6).tolist(),'provenance':'Engineering mass reserve'})


    return construction.finish(
        'P5 CAD-density compact parts, preserved catalog motor/brake mass, folded belt transmission, explicit cable/fastener reserves; supplier interfaces not released',
        {
            'packaging_envelopes.json': primitive_records,
            'folded-transmissions.json': TRANSMISSIONS,
            'p5_collision_proxies.json': {
                'records': P5_PROXIES,
                'joint_extra_armature_kg_m2': [r['extra_joint_armature_kg_m2'] for r in TRANSMISSIONS],
                'replace_names': [],
                'scope': 'Motor/reducer capsules from actual folded CAD. Structural proxy additions follow in collision review. No flexible belt contact; torque screened independently.'},
        }, native_requirements=nominal_interface_requirements(PARTS, SPEC, TRANSMISSIONS))
