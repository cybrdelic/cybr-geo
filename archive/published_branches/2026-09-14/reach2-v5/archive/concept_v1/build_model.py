from pathlib import Path
import math, json
import cadquery as cq
from cadquery import exporters

OUT = Path('/mnt/data/torquebias_diff')
STL = OUT/'stl'
STL.mkdir(parents=True, exist_ok=True)

# ---------------- helpers ----------------
def polar(r, deg):
    a = math.radians(deg)
    return (r*math.cos(a), r*math.sin(a))

def ring(od, id_, h, z=0):
    return (cq.Workplane('XY').circle(od/2).circle(id_/2).extrude(h)
            .translate((0,0,z-h/2)))

def disk_with_holes(od, h, bore, bolt_circle, bolt_dia, n, z=0, chamfer=0.8):
    s = cq.Workplane('XY').circle(od/2).extrude(h).translate((0,0,z-h/2))
    if bore > 0:
        s = s.cut(cq.Workplane('XY').circle(bore/2).extrude(h+4).translate((0,0,z-(h+4)/2)))
    for i in range(n):
        a = 2*math.pi*i/n
        x,y = bolt_circle/2*math.cos(a), bolt_circle/2*math.sin(a)
        hole = cq.Workplane('XY').center(x,y).circle(bolt_dia/2).extrude(h+4).translate((0,0,z-(h+4)/2))
        s = s.cut(hole)
    try:
        if chamfer:
            s = s.edges('|Z').chamfer(chamfer)
    except Exception:
        pass
    return s

def helical_gear(root_r, outer_r, width, teeth, twist_deg=16, bore=0, z=0):
    # Compound geometry: robust, visually accurate helical tooth sweep by lofting trapezoidal teeth.
    shapes = []
    base = cq.Workplane('XY').circle(root_r).extrude(width).translate((0,0,z-width/2))
    if bore:
        base = base.cut(cq.Workplane('XY').circle(bore/2).extrude(width+2).translate((0,0,z-(width+2)/2)))
    shapes.append(base.val())
    pitch = 360/teeth
    root_half = pitch*0.34
    tip_half = pitch*0.20
    for i in range(teeth):
        c = i*pitch
        pts0 = [polar(root_r*0.995,c-root_half), polar(outer_r,c-tip_half),
                polar(outer_r,c+tip_half), polar(root_r*0.995,c+root_half)]
        pts1 = []
        for p in pts0:
            r = math.hypot(p[0],p[1]); a = math.degrees(math.atan2(p[1],p[0])) + twist_deg
            pts1.append(polar(r,a))
        tooth = (cq.Workplane('XY').workplane(offset=z-width/2).polyline(pts0).close()
                 .workplane(offset=width).polyline(pts1).close().loft(combine=True))
        shapes.append(tooth.val())
    return cq.Workplane('XY').newObject([cq.Compound.makeCompound(shapes)])

def spline_shaft(core_r, spline_r, length, teeth, bore=0, z=0):
    shapes = []
    core = cq.Workplane('XY').circle(core_r).extrude(length).translate((0,0,z-length/2))
    if bore:
        core = core.cut(cq.Workplane('XY').circle(bore/2).extrude(length+2).translate((0,0,z-(length+2)/2)))
    shapes.append(core.val())
    pitch = 360/teeth
    for i in range(teeth):
        c = i*pitch
        hw = pitch*0.20
        pts = [polar(core_r*0.985,c-hw), polar(spline_r,c-hw*0.75), polar(spline_r,c+hw*0.75), polar(core_r*0.985,c+hw)]
        tooth = cq.Workplane('XY').polyline(pts).close().extrude(length).translate((0,0,z-length/2))
        shapes.append(tooth.val())
    return cq.Workplane('XY').newObject([cq.Compound.makeCompound(shapes)])

def clutch_stack(z, side=1, count=7):
    shapes=[]
    start = z
    for i in range(count):
        th=1.15
        od=66 if i%2==0 else 63
        id_=35 if i%2==0 else 32
        plate = ring(od,id_,th,start + side*i*1.65)
        # Six indexing slots / tabs for visual and mechanical distinction
        if i%2==0:
            for k in range(6):
                a=math.radians(k*60)
                x,y=od/2*math.cos(a),od/2*math.sin(a)
                tab=(cq.Workplane('XY').box(5,2.4,th, centered=(True,True,True))
                     .translate((x,y,start + side*i*1.65)))
                plate = plate.union(tab)
        shapes.append(plate.val())
    return cq.Workplane('XY').newObject([cq.Compound.makeCompound(shapes)])

def coil_spring(radius, wire_r, pitch, height, z0=0):
    helix = cq.Wire.makeHelix(pitch, height, radius)
    spring = cq.Workplane('XZ').center(radius,0).circle(wire_r).sweep(helix)
    return spring.translate((0,0,z0-height/2))

def ball_bearing(od, id_, width, balls=16, z=0):
    shapes=[]
    outer = ring(od, od-5, width, z)
    inner = ring(id_+5, id_, width, z)
    shapes += [outer.val(), inner.val()]
    race_r = (od/2 + id_/2)/2
    ball_r = min(2.4, (od-id_)/8)
    for i in range(balls):
        a=2*math.pi*i/balls
        x,y=race_r*math.cos(a), race_r*math.sin(a)
        b=cq.Workplane('XY').sphere(ball_r).translate((x,y,z))
        shapes.append(b.val())
    return cq.Workplane('XY').newObject([cq.Compound.makeCompound(shapes)])

def cage_housing():
    # 104 mm OD cage, 72 mm length, large windows and reinforced end rings.
    shell = cq.Workplane('XY').circle(52).circle(44).extrude(72).translate((0,0,-36))
    # Six large circumferential windows through the shell.
    for i in range(6):
        a=i*60
        cut=(cq.Workplane('XY').box(46,22,46, centered=(True,True,True))
             .translate((0,48,0)).rotate((0,0,0),(0,0,1),a))
        shell=shell.cut(cut)
    # end rings + smaller reinforcing rings
    left = ring(108,78,8,-36)
    right = ring(108,78,8,36)
    # 6 long ribs connect end rings, following the cage windows.
    ribs=[]
    for i in range(6):
        a=i*60+30
        rib=(cq.Workplane('XY').box(10,10,68, centered=(True,True,True))
             .translate((0,47,0)).rotate((0,0,0),(0,0,1),a))
        ribs.append(rib.val())
    comp = cq.Compound.makeCompound([shell.val(),left.val(),right.val()]+ribs)
    return cq.Workplane('XY').newObject([comp])

def carrier_end_plate(z):
    p=ring(86,34,5,z)
    for i in range(6):
        a=2*math.pi*i/6
        x,y=34*math.cos(a),34*math.sin(a)
        h=cq.Workplane('XY').center(x,y).circle(3).extrude(7).translate((0,0,z-3.5))
        p=p.cut(h)
    return p

# ---------------- assembly ----------------
components = {}
components['housing'] = cage_housing()
components['left_flange'] = disk_with_holes(104,10,32,82,8.5,8,z=-61)
components['right_flange'] = disk_with_holes(104,10,32,82,8.5,8,z=61)
components['left_hub'] = ring(56,28,20,-49)
components['right_hub'] = ring(56,28,20,49)
components['left_output_spline'] = spline_shaft(14,16,36,26,bore=0,z=-78)
components['right_output_spline'] = spline_shaft(14,16,36,26,bore=0,z=78)
components['left_bearing'] = ball_bearing(78,42,9,balls=18,z=-39)
components['right_bearing'] = ball_bearing(78,42,9,balls=18,z=39)
components['left_end_plate'] = carrier_end_plate(-31.5)
components['right_end_plate'] = carrier_end_plate(31.5)

# Side gears: two opposing, splined helical gears.
components['left_side_gear'] = helical_gear(18.5,24.5,18,24,twist_deg=15,bore=28,z=-10)
components['right_side_gear'] = helical_gear(18.5,24.5,18,24,twist_deg=-15,bore=28,z=10)

# Internal shaft cores through side gears.
components['left_internal_shaft'] = ring(28,0.1,42,-19)
components['right_internal_shaft'] = ring(28,0.1,42,19)

# Six pinions, alternating handedness and axial location. Parallel-axis arrangement.
for i in range(6):
    a=i*60
    rr=34.5
    x,y=rr*math.cos(math.radians(a)), rr*math.sin(math.radians(a))
    zz=-8 if i%2==0 else 8
    g=helical_gear(6.8,10.2,23,13,twist_deg=(-21 if i%2==0 else 21),bore=4.5,z=zz)
    components[f'pinion_{i+1}']=g.translate((x,y,0))
    # pinion axle
    axle=cq.Workplane('XY').circle(2.2).extrude(34).translate((x,y,-17))
    components[f'pinion_axle_{i+1}']=axle

components['left_clutch_stack'] = clutch_stack(-23.0, side=-1, count=7)
components['right_clutch_stack'] = clutch_stack(23.0, side=1, count=7)
components['left_preload_spring'] = coil_spring(30.0,1.0,2.8,10.5,z0=-27.5)
components['right_preload_spring'] = coil_spring(30.0,1.0,2.8,10.5,z0=27.5)

# Ring gear mounting flange / drive interface around carrier center.
ringgear = ring(121,96,10,0)
# Add 12 through holes on OD for ring gear bolting.
for i in range(12):
    a=2*math.pi*i/12
    x,y=54*math.cos(a),54*math.sin(a)
    h=cq.Workplane('XY').center(x,y).circle(2.7).extrude(14).translate((0,0,-7))
    ringgear=ringgear.cut(h)
components['ring_gear_mount'] = ringgear

# Six housing tie bolts visible between end plates.
for i in range(6):
    a=2*math.pi*i/6
    x,y=47*math.cos(a),47*math.sin(a)
    bolt=cq.Workplane('XY').circle(2.4).extrude(76).translate((x,y,-38))
    components[f'tie_bolt_{i+1}']=bolt

# Export individual STLs + STEP assembly
mesh_tol=0.22
for name, shape in components.items():
    exporters.export(shape, str(STL/f'{name}.stl'), tolerance=mesh_tol, angularTolerance=0.12)

assy=cq.Assembly(name='TORQUE_BIASING_DIFFERENTIAL')
for name, shape in components.items():
    assy.add(shape, name=name)

# Assembly STEP and per-part STEP for CAD reuse
exporters.export(assy.toCompound(), str(OUT/'torque_biasing_differential.step'))

# Export a few principal solids as standalone STEP too
for n in ['housing','left_flange','right_flange','left_side_gear','right_side_gear','ring_gear_mount']:
    exporters.export(components[n], str(OUT/f'{n}.step'))

manifest={
    'name':'CYBR Compact Torque-Biasing Differential Concept',
    'units':'mm',
    'overall_length_mm':166,
    'housing_od_mm':108,
    'ring_gear_mount_od_mm':121,
    'output_spline':'26 tooth visual spline',
    'side_gears':'24 tooth helical, opposite hand',
    'pinions':'6 x 13 tooth helical',
    'preload':'dual clutch packs + coil preload springs',
    'note':'Concept geometry derived from the prior generated presentation; not a production-qualified Torsen design.',
    'components':list(components.keys())
}
(OUT/'manifest.json').write_text(json.dumps(manifest,indent=2))
print(f'exported {len(components)} components')
