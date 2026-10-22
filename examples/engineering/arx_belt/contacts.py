"""ARX belt contact declarations using the shared envelope algorithms.

Adapted from integration/build_p5_collision_proxies.py at
cybrdelic/cybr-arx6-belt aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2.
The source's 2,095 default primitives are retained. They over-approximate
material around openings and can report false contacts; they are not exact
native CAD or evidence of hardware qualification.
"""
from __future__ import annotations

from copy import deepcopy
import math

from mechanism_lab.contact_geometry import ProxyBuilder, align_z, rotation_z, tabletop_tiles


def _stage(builder,spec,j,parent,size,origin,axis,theta):
 box,cylinder,ring=builder.box,builder.cylinder,builder.annulus
 large=size=='large';f=spec['folded_transmission'][size];C=f['center_distance_mm'];front=f['gearhead_front_z_mm'];Lg=f['gearhead_length_mm'];Lm=f['motor_length_mm'];pr=f['pitch_mm']*f['pulley_teeth']/(2*math.pi);zc=f['pulley_center_z_mm'];w=f['belt_width_mm'];Q=align_z(axis)@rotation_z(theta);Qa=align_z(axis)
 cylinder(j+'_motor',parent,[0,C,front+Lg],[0,C,front+Lg+Lm],20 if large else 11,Q,origin,j+'_motor')
 cylinder(j+'_gearhead',parent,[0,C,front],[0,C,front+Lg],21 if large else 11,Q,origin,j+'_gearhead')
 # Retained stationary casing material outside its real bearing and shaft bores.
 ring(j+'_bearing_housing',parent,21 if large else 14,27 if large else 18,-40 if large else -24,6 if large else 4,Qa,origin,native_part=size+'_bearing_housing')
 ring(j+'_rear_bearing_cover',parent,18 if large else 12,36 if large else 25,-44 if large else -28,-40 if large else -24,Qa,origin,native_part=size+'_rear_bearing_cover')
 ring(j+'_front_bearing_cover',parent,18 if large else 12,27 if large else 18,6 if large else 4,10 if large else 8,Qa,origin,native_part=size+'_front_bearing_cover')
 b=spec['brakes'][size];case0=-88 if large else -63
 ring(j+'_brake_stator',parent,b['ID_mm']/2,b['OD_mm']/2,case0,case0+b['length_mm'],Qa,origin,native_part=size+'_brake_vendor_envelope')
 ring(j+'_brake_front_adapter',parent,22.2 if large else 8.5,54 if large else 27.5,-48 if large else -32,-44 if large else -28,Qa,origin,native_part=size+'_brake_front_adapter')
 ring(j+'_brake_back_adapter',parent,10.2 if large else 4.2,54 if large else 27.5,-104 if large else -74,-100 if large else -70,Qa,origin,native_part=j+'_stop_pin_backplate')
 # Carrier is a drilled capsule deck; annular regions leave shaft openings.
 rr=36 if large else 21;outR=56 if large else 29.5;outHole=16 if large else 8.5;inHole=14.2 if large else 7.7
 ring(j+'_carrier_output',parent,outHole,outR,front-4,front,Q,origin,native_part=size+'_folded_carrier_deck')
 ring(j+'_carrier_input',parent,inHole,rr,front-4,front,Q,origin,center_xy_mm=(0,C),native_part=size+'_folded_carrier_deck')
 box(j+'_carrier_bridge',parent,[0,(outHole+C-inHole)/2,front-2],[rr,(C-inHole-outHole)/2,2],Q,origin,size+'_folded_carrier_deck','Conservative carrier web, excludes both pilot holes; fastener holes omitted')
 cover0=zc-w/2-(14.5 if large else 13.5);cover1=cover0+3;inner=pr+4.5;outer=inner+3;windowhalf=max(4,pr-6)
 ring(j+'_cover_output',parent,10.4 if large else 4.4,outer,cover0,cover1,Q,origin,native_part=size+'_folded_front_cover')
 ring(j+'_cover_input',parent,14.02 if large else 11.02,outer,cover0,cover1,Q,origin,center_xy_mm=(0,C),native_part=size+'_folded_front_cover')
 for sign in (-1,1):
  box(j+'_cover_side_'+str(sign),parent,[sign*(outer+windowhalf)/2,C/2,(cover0+cover1)/2],[(outer-windowhalf)/2,C/2,1.5],Q,origin,size+'_folded_front_cover')
 # Full annuli are conservative at their inner-facing halves; the native shell
 # has a capsule cavity. This can cause false contacts, never silent gaps.
 for which,y in [('output',0),('input',C)]:
  ring(j+'_housing_end_'+which,parent,inner,outer,cover1,front-4,Q,origin,center_xy_mm=(0,y),native_part=size+'_folded_housing_rails')
 for sign in (-1,1):
  for end,z in [('rear',cover1+2),('front',front-6)]:
   box(j+'_housing_side_'+str(sign)+'_'+end,parent,[sign*(inner+1.5),C/2,z],[1.5,C/2,2],Q,origin,size+'_folded_housing_rails')
 box(j+'_tension_block',parent,[0,C+rr-3,front+3.5],[6 if large else 4,4,3.5],Q,origin,size+'_folded_tension_block')
 cylinder(j+'_tension_screw',parent,[0,C+rr-13,front+3.5],[0,C+rr+8,front+3.5],3.5 if large else 2.7,Q,origin,size+'_folded_tension_screw')
 # Outboard input journal is enclosed; added bearing pocket protrusion matters.
 ring(j+'_input_bearing_pocket',parent,14.02 if large else 11.02,17 if large else 14,cover0-3.5,cover1+3,Q,origin,center_xy_mm=(0,C),native_part=size+'_folded_front_cover')


def build(spec, transmissions):
    """Return fresh proxy configuration from a spec and native-stage records.

    Stage frames come from the same transmission records as native CAD, rather
    than a second hardcoded frame list. This function performs no file I/O.
    """
    if len(transmissions) != 6 or {row['joint'] for row in transmissions} != {f'J{i}' for i in range(1, 7)}:
        raise ValueError('ARX belt contacts require the six named native transmission records')
    by_joint = {row['joint']: row for row in transmissions}
    transmissions = [by_joint[f'J{i}'] for i in range(1, 7)]
    for row in transmissions:
        value = row['extra_joint_armature_kg_m2']
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError('Transmission armature additions must be finite and nonnegative')
    builder = ProxyBuilder(prefix='P5_')
    box, cylinder, ring = builder.box, builder.cylinder, builder.annulus
    for index, row in enumerate(transmissions):
        _stage(builder, spec, row['joint'], row['parent'], 'large' if index < 3 else 'small',
               row['origin_mm'], row['axis'], row['clocking_degrees'])
    # Revised counterbalance plates and short clevises, with real coordinates.
    for name,body,c,h in [
     ('shoulder_fixed_plate','turret',[-20,89,54],[55,3,38]),
     ('shoulder_moving_plate','upper',[0,70,24.5],[29,14,2.5]),
     ('shoulder_moving_post','upper',[0,70,56],[8,5,33.5]),
     ('elbow_fixed_plate','upper',[-7,190,-25],[37,16,3]),
     ('elbow_fixed_post','upper',[-30,190,-54.5],[8,5,29.5]),
     ('elbow_moving_plate','fore',[0,35,-33],[29,35,3]),
     ('elbow_moving_lever','fore',[0,-22.5,-33],[12,22.5,3]),
     ('elbow_moving_post','fore',[0,-45,-57],[8,5,27])]:box(name,body,c,h)
    for z in [55,65,79,89]:box('shoulder_fixed_cheek_'+str(z),'turret',[-65,100,z],[9,15,2])
    cylinder('elbow_moving_root','fore',[0,0,-36],[0,0,-30],27)
    bench = spec['bench_interface']
    # align_z(+Y) maps canonical +Y to body -Z.
    holes = [dict(center_xy_mm=[hole['center_xz_mm'][0], -hole['center_xz_mm'][1]],
                  radius_mm=hole['diameter_mm'] / 2)
             for hole in bench['through_bores']]
    (xmin, xmax), (ymin, ymax) = bench['base_foot_bounds_xz_mm']
    radius = (ymax - ymin) / 2
    if ymin != -ymax or xmin != -radius:
        raise ValueError('ARX capsule foot must retain its source origin and X-aligned semicircular ends')
    axis = align_z((0, 1, 0))
    builder.capsule_plate_strips('base_foot', 'fixed', radius, xmax - radius,
                                0, bench['base_foot_thickness_mm'], holes,
                                rotation=axis, native_part='bench_mount_plate')
    ring('pedestal_main', 'fixed', 27.025, 45, 20.2, 64, axis, native_part='pedestal_sleeve')
    ring('pedestal_base', 'fixed', 36.2, 80, 12, 20, axis, native_part='pedestal_sleeve')
    top = bench['top_y_mm']
    bottom = top - bench['thickness_mm']
    for index, (x, z) in enumerate(bench['mount_bolt_centers_xz_mm']):
        cylinder('bench_washer_' + str(index), 'fixed', [x, bottom - 7, z], [x, bottom, z],
                 14, native_part='bench_backing_washer')
        cylinder('bench_bolt_' + str(index), 'fixed', [x, bottom - 13, z], [x, top + 21, z],
                 7, native_part='bench_mount_bolt')
    config={'records':builder.records,'replace_names':['pedestal_collision',*[f'P2_base_mount_{i}' for i in range(4)]],
     'replace_prefixes':['package_','P2_fixed_clevis','P2_moving_clevis','P2_elbow_anchor_','P2_elbow_moving_anchor'],
     'joint_extra_armature_kg_m2':[r['extra_joint_armature_kg_m2'] for r in transmissions],
     'scope':'External folded motor/reducer, output bearing/brake material, slotted carrier, windowed enclosure, revised counterbalance brackets, mounting foot and pedestal. Annuli use continuous overlapping conservative 24-sector boxes; no cavity-filling bearing or brake cylinders. Native BREP controls exact fitted interfaces. Flexible springs/belt contact, small fastener holes, internal shafts and vendor internals are outside proxy fidelity. No new body exclusions.',
     'approximation':'Conservative external material strips can report false contacts near curved or machined edges. Native BREP refinement is required before classifying an apparent overlap. Circular annulus sector maximum radial excess ro*(sec(pi/24)-1).',
     'source_generator':'generate_cad_p2.py','source_proxy_script':'build_p5_collision_proxies.py'}
    config['tabletop'] = dict(thickness_mm=bench['thickness_mm'], top_y_mm=top,
                              extent_xz_mm=[1000, 1000], holes=[dict(
                                  center_xz_mm=list(hole['center_xz_mm']), radius_mm=hole['diameter_mm'] / 2)
                                  for hole in bench['through_bores']])
    config['coverage_policy'] = dict(material_containment_claimed=False,
                                    conservative_overcoverage=True,
                                    native_brep_refinement_required_for_apparent_contact=True)
    config['source_provenance'] = dict(repository='https://github.com/cybrdelic/cybr-arx6-belt',
                                     commit='aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2',
                                     path='ARX-6-P5-Engineering/integration/build_p5_collision_proxies.py')
    return deepcopy(config)


def table_contacts(config):
    """Emit the source model's separate 1 mm table tiles without changing config."""
    return tabletop_tiles(config['tabletop'], name_prefix='P5_table_material_')
