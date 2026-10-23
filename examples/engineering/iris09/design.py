"""IRIS-09's authored geometry through shared GEO CAD services.

Direct geometric transcription of the pinned shader sections recorded in
source-geometry.json. No shader, image, film or render assets are copied. The
source is an animated optical study, with no manufactured design or license
grant supplied. This requested adaptation uses CYBR GEO's GPL-2.0-only license;
it does not relicense upstream files. There is deliberately no physics model,
EngineeringProject acceptance predicate or mechanically driven animation.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

import cadquery as cq
import numpy as np

from mechanism_lab.core import Assembly, Material, View, cad_part
from mechanism_lab.native_construction import NativeConstruction
from mechanism_lab.shape_dag import ShapeEvaluator, from_trees


HERE = Path(__file__).resolve().parent
SOURCE = json.loads((HERE / 'source-geometry.json').read_text(encoding='utf-8'))
QUALIFICATION = 'Source-derived visual geometry at a reference pose; no mechanical closure, dynamics or hardware qualification'
DENSITIES = {'aluminum_assumed': 2700., 'steel_assumed': 7850.,
             'brass_assumed': 8500., 'glass_assumed': 2500.}


def _node(op, args=(), *children):
    return dict(op=op, args=list(args), children=list(children))


def _transform(tree, matrix):
    return _node('transform', np.asarray(matrix).ravel().tolist(), tree)


def _translate(tree, xyz):
    matrix = np.eye(4)
    matrix[:3, 3] = xyz
    return _transform(tree, matrix)


def _frame(angle, xyz=(0, 0, 0)):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0, xyz[0]], [s, c, 0, xyz[1]],
                     [0, 0, 1, xyz[2]], [0, 0, 0, 1]], float)


def angles(time_s):
    if isinstance(time_s, bool) or not math.isfinite(time_s):
        raise ValueError('time_s must be finite')
    wave = math.cos(float(time_s) * math.pi / 10)
    return dict(hinge_radians=.45 + .45 * wave, actuator_radians=-.38 + .54 * (1 - wave))


def _scale(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError('scene_unit_mm must be finite and positive')
    return float(value)


def leaf_frame(index, hinge_radians, scene_unit_mm=50.):
    """Source hinge/XZ basis mapped to right-handed CAD XY, Z up."""
    leaf = SOURCE['leaf']
    azimuth = index * 2 * math.pi / leaf['count']
    r = leaf['hinge_radius'] * _scale(scene_unit_mm)
    return _frame(-(azimuth + hinge_radians), (r * math.cos(azimuth), -r * math.sin(azimuth), 0))


def leaf_tree(scene_unit_mm=50.):
    """Invariant local leaf = two circular prisms intersected by a canted slab.

    The shader's planes are H-t <= source_y - cant*local_z <= H. In CAD
    coordinates these become H-t <= z + cant*y <= H. The slab's normal
    thickness is t/sqrt(1+cant**2), preserving the source's *vertical* thickness.
    Finite cylinder heights and slab side extents contain the entire profile.
    The stationary world clip is deliberately absent from this rigid template.
    """
    s, row = _scale(scene_unit_mm), SOURCE['leaf']
    arcs = [row['front_arc_center_xz_radius'], row['trailing_arc_center_xz_radius']]
    bound = max(abs(v) + arc[2] for arc in arcs for v in arc[:2]) + 1
    disks = [_translate(_node('cylinder', [arc[2] * s, bound * s]),
                        (arc[0] * s, -arc[1] * s, 0)) for arc in arcs]
    cant, thickness = row['cant'], row['vertical_thickness']
    angle = -math.atan(cant)
    c, sn = math.cos(angle), math.sin(angle)
    slab_frame = np.array([[1, 0, 0, 0], [0, c, -sn, 0],
                          [0, sn, c, (row['height'] - thickness / 2) * s], [0, 0, 0, 1]])
    slab = _transform(_node('box', [bound * s, bound * s, thickness * s / (2 * math.sqrt(1 + cant * cant))]), slab_frame)
    return _node('intersection', [], _node('intersection', [], *disks), slab)


def leaf_dag(time_s=5., scene_unit_mm=50.):
    """One shared DAG holds invariant geometry and the separately clipped pose."""
    s = _scale(scene_unit_mm)
    angle = angles(time_s)['hinge_radians']
    invariant = leaf_tree(s)
    frame0 = leaf_frame(0, angle, s)
    clip = _node('cylinder', [SOURCE['leaf']['world_clip_radius'] * s, 4 * s])
    clipped_world = _node('intersection', [], _transform(invariant, frame0), clip)
    clipped_local = _transform(clipped_world, np.linalg.inv(frame0))
    trees = {'invariant_leaf': invariant, 'visual_leaf_local': clipped_local}
    for index in range(SOURCE['leaf']['count']):
        trees[f'visual_leaf_{index:02}'] = _transform(clipped_local, leaf_frame(index, angle, s))
        trees[f'invariant_leaf_{index:02}'] = _transform(invariant, leaf_frame(index, angle, s))
    return from_trees(trees, provenance=dict(source=SOURCE['source'], time_s=time_s,
                      scene_unit_mm=s, scope='Exact encoded leaf formulas; clipping creates a reference display solid'))


def rounded_radial_profile(center_r, center_z, half_r, half_z, corner, scale):
    """Exact solid of revolution of the source sdRoundBox2 radial profile.

    A profile extending beyond the axis is a disk only when its inner rounded
    corner is also behind the axis, as in the source objective floor/ear/screw.
    """
    inner, outer = center_r - half_r, center_r + half_r
    if inner < 0 and inner + corner > 1e-12:
        raise ValueError('Radial profile crosses the axis through a rounded corner')
    work = cq.Workplane('XY').circle(outer * scale)
    if inner > 0:
        work = work.circle(inner * scale)
    shape = work.extrude(2 * half_z * scale).translate((0, 0, (center_z - half_z) * scale))
    return shape.edges('%CIRCLE').fillet(corner * scale).val()


def rounded_box(source_center, source_half_size, corner, scale):
    # Source (x,y,z) -> CAD (x,-z,y).
    x, y, z = source_half_size
    cx, cy, cz = source_center
    shape = cq.Workplane('XY').box(2 * x * scale, 2 * z * scale, 2 * y * scale)
    if corner:
        shape = shape.edges().fillet(corner * scale)
    return shape.val().translate((cx * scale, -cz * scale, cy * scale))


def _rotate(shape, angle):
    return shape.rotate((0, 0, 0), (0, 0, 1), math.degrees(angle))


def _union(shapes):
    return shapes[0].fuse(*shapes[1:]) if len(shapes) > 1 else shapes[0]


def _overlaps_bounds(a, b):
    a, b = a.BoundingBox(), b.BoundingBox()
    return all(getattr(a, dim + 'max') >= getattr(b, dim + 'min') and
               getattr(b, dim + 'max') >= getattr(a, dim + 'min') for dim in ('x', 'y', 'z'))


def _radial_face(r, z, wr, hz, corner, scale):
    """The same rounded source profile in the radial/Z half-plane.

    Coplanar profile Booleans before revolution are substantially cheaper than
    repeated 3D unions of nearly tangent toroidal faces. The source's disk
    profile crosses the axis beyond its inner rounding, so the axis is straight.
    """
    inner, outer = max(0., r - wr), r + wr
    points = [(inner * scale, 0, (z - hz) * scale),
              (outer * scale, 0, (z - hz) * scale),
              (outer * scale, 0, (z + hz) * scale),
              (inner * scale, 0, (z + hz) * scale)]
    face = cq.Face.makeFromWires(cq.Wire.makePolygon(points, close=True))
    vertices = [v for v in face.Vertices() if inner > 0 or v.Center().x > 0]
    return face.fillet2D(corner * scale, vertices)


def _revolve_profile(shape):
    solids = [cq.Solid.revolve(face.outerWire(), face.innerWires(), 360., (0, 0, 0), (0, 0, 1))
              for face in shape.Faces()]
    return cq.Compound.makeCompound(solids)


def _cut_radial_profiles(profile, cuts, material):
    """Apply every intersectable cutter with conservative radial/Z pruning.

    Each input face lies in r/Z and is revolved through a full circle. Its
    radial interval and height interval therefore bound every occupied point.
    Cutter AABB radial bounds are conservative; a rejected cutter provably
    cannot meet the revolved profile. Small batches bound Boolean working RAM.
    """
    results = []
    for index, face in enumerate(profile.Faces()):
        b = face.BoundingBox()
        relevant = []
        for tool in cuts:
            t = tool.BoundingBox()
            near_x = max(t.xmin, -t.xmax, 0.)
            near_y = max(t.ymin, -t.ymax, 0.)
            near_r = math.hypot(near_x, near_y)
            far_r = math.hypot(max(abs(t.xmin), abs(t.xmax)), max(abs(t.ymin), abs(t.ymax)))
            if (t.zmax >= b.zmin and t.zmin <= b.zmax and near_r <= b.xmax and far_r >= b.xmin):
                relevant.append(tool)
        shape = _revolve_profile(face)
        print(f'Housing material {material}, radial region {index}: {len(relevant)} cutters', flush=True)
        for start in range(0, len(relevant), 20):
            shape = shape.cut(cq.Compound.makeCompound(relevant[start:start + 20]))
            print(f'  cuts {min(start + 20, len(relevant))}/{len(relevant)} complete', flush=True)
        results.append(shape)
    return cq.Compound.makeCompound(results)


def _track_tree(scale):
    row = SOURCE['housing']['track']
    r, w, h = row['radius'] * scale, row['half_width'] * scale, row['half_height'] * scale
    a, b = -row['center_angle'] - row['angle_half_range'], -row['center_angle'] + row['angle_half_range']
    ring = _node('cut', [], _node('cylinder', [r + w, h]), _node('cylinder', [r - w, h * 2]))
    reach = 4 * r
    wedge = _node('polygon', [[[0, 0], [reach * math.cos(a), reach * math.sin(a)],
                              [reach * math.cos(b), reach * math.sin(b)]], h * 2])
    track = _node('intersection', [], ring, wedge)
    for angle in (a, b):
        end = _translate(_node('cylinder', [w, h]), (r * math.cos(angle), r * math.sin(angle), 0))
        track = _node('union', [], track, end)
    return _translate(track, (0, 0, row['height'] * scale))


def housing_geometry(scene_unit_mm=50.):
    """Source rounded profiles and all explicitly modeled housing cuts/ears."""
    s, source = _scale(scene_unit_mm), SOURCE['housing']
    cuts = []
    row = source['flutes']
    flute = rounded_box((row['radial_center'], row['height'], 0), row['half_size_source_xyz'], row['corner_radius'], s)
    cuts.extend(_rotate(flute, -index * 2 * math.pi / row['count']) for index in range(row['count']))
    row = source['seam']
    cuts.append(rounded_radial_profile(row['radial_center'], row['height'], row['half_width'], row['half_height'], row['corner_radius'], s))
    row = source['spanner']
    for sign in (-1, 1):
        notch = rounded_box((sign * row['radial_center'], row['height'], 0), row['half_size_source_xyz'], row['corner_radius'], s)
        cuts.append(_rotate(notch, -row['angle']))
    cuts.append(ShapeEvaluator(from_trees({'track': _track_tree(s)})).evaluate())
    screw_row = source['screws']
    for index in range(screw_row['count']):
        angle = index * 2 * math.pi / screw_row['count']
        recess = rounded_radial_profile(0, screw_row['recess_height'], screw_row['recess_radius'],
                                       screw_row['recess_half_height'], screw_row['recess_corner'], s)
        cuts.append(recess.translate((screw_row['center_radius'] * s * math.cos(angle),
                                      -screw_row['center_radius'] * s * math.sin(angle), 0)))
    profiles = []
    for name, r, z, wr, hz, fillet, material in source['rounded_radial_profiles']:
        shape = _radial_face(r, z, wr, hz, fillet, s)
        profiles.append((name, shape, material))
    # Ears are unioned after the global track/notch cuts in bodyMap, then the
    # screw recesses are subtracted. Their disjoint angle sectors need no clip.
    ears = []
    row = source['ears']
    for index in range(row['count']):
        angle = -index * 2 * math.pi / row['count']
        ear = rounded_radial_profile(0, row['height'], row['radius'], row['half_height'], row['corner_radius'], s)
        ear = ear.translate((row['center_radius'] * s, 0, 0))
        bore = cq.Solid.makeCylinder(row['bore_radius'] * s, 4 * s, (row['bore_center_radius'] * s, 0, -2 * s))
        ears.append(_rotate(ear.cut(bore), angle))
    # A disjoint material partition preserves the source's occupied union and
    # avoids double-counting overlapping radial profiles in volume integrals.
    grouped, occupied = [], None
    for material in (3, 2, 5, 1):
        print(f'Building housing material {material}: profile union', flush=True)
        shape = _union([p[1] for p in profiles if p[2] == material])
        raw = shape
        higher_priority = occupied
        if occupied is not None:
            shape = shape.cut(occupied)
        occupied = raw if occupied is None else occupied.fuse(raw)
        # Partition the simple radial profiles before the shared subtraction.
        # (A\\B)\\C == (A\\C)\\(B\\C): this changes only Boolean ordering,
        # avoiding repeated unions of hundreds of already-fluted faces.
        shape = _cut_radial_profiles(shape.clean(), cuts, material)
        if material == 1:
            # Ears are authored after the global cuts. They still participate
            # in our explicit material partition: their overlap with lower
            # trim/objective floor must not be counted twice.
            other_material = _revolve_profile(higher_priority.clean())
            shape = shape.fuse(*[ear.cut(other_material) for ear in ears])
        if shape.Volume() > 0:
            grouped.append((f'housing_material_{material}', shape, material))
    row = screw_row
    for index in range(row['count']):
        angle = index * 2 * math.pi / row['count']
        screw = rounded_radial_profile(0, row['height'], row['radius'], row['half_height'], row['corner_radius'], s)
        slot = rounded_box((0, row['slot_height'], 0), row['slot_half_size_source_xyz'], 0, s)
        screw = screw.cut(_rotate(slot, -(row['slot_angle_base'] + angle * row['slot_angle_multiplier'])))
        screw = screw.translate((row['center_radius'] * s, 0, 0))
        grouped.append((f'screw_{index}', _rotate(screw, -angle), 4))
    return grouped


def handle_geometry(scene_unit_mm=50.):
    s, row = _scale(scene_unit_mm), SOURCE['handle']
    result = [('handle_arm', rounded_box(row['arm_center_source_xyz'], row['arm_half_size_source_xyz'], row['arm_corner'], s), 4)]
    x, z = row['pivot_center_x_y']
    pivot = rounded_radial_profile(0, z, row['pivot_radius'], row['pivot_half_height'], row['pivot_corner'], s).translate((x * s, 0, 0))
    result.append(('handle_pivot', pivot, 4))
    x, z = row['knob_center_x_y']
    knob = rounded_radial_profile(0, z, row['knob_mean_radius'], row['knob_half_height'], row['knob_corner'], s).translate((x * s, 0, 0))
    result.append(('knob_mean_radius_approximation', knob, 1))
    cap = rounded_radial_profile(0, row['cap_height'], row['cap_radius'], row['cap_half_height'], row['cap_corner'], s).translate((x * s, 0, 0))
    result.append(('knob_cap', cap, 4))
    return result


def lens_geometry(scene_unit_mm=50.):
    s = _scale(scene_unit_mm)
    trees = {}
    for index, (radius, base, edge) in enumerate(SOURCE['lenses']['sphere_radius_base_y_edge_radius']):
        center = base - math.sqrt(radius * radius - edge * edge)
        sphere = _translate(_node('sphere', [radius * s]), (0, 0, center * s))
        halfspace = _translate(_node('box', [2 * radius * s, 2 * radius * s, radius * s]), (0, 0, (base + radius) * s))
        trees[f'lens_{index}'] = _node('intersection', [], sphere, halfspace)
    evaluator = ShapeEvaluator(from_trees(trees))
    return [(name, evaluator.evaluate(name), 6) for name in trees]


@dataclass
class StudyBuild:
    native: object
    reference: Assembly
    invariant: Assembly
    report: dict


def build(output, *, scene_unit_mm=50., time_s=5.):
    """Export reference CAD and a distinct invariant template through GEO."""
    s = _scale(scene_unit_mm)
    motion = angles(time_s)
    parameters = dict(scene_unit_mm=s, time_s=float(time_s))
    provenance = dict(SOURCE['source'], adapted_recipe_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                      geometry_constants_sha256=hashlib.sha256((HERE / 'source-geometry.json').read_bytes()).hexdigest(),
                      input_spec_sha256=hashlib.sha256(json.dumps(parameters, sort_keys=True, separators=(',', ':')).encode()).hexdigest(),
                      scope=QUALIFICATION, units_assumption=f'1 source scene unit = {s:g} mm', density_assumptions_kg_m3=DENSITIES)
    construction = NativeConstruction(output, DENSITIES, provenance=provenance)
    materials = [Material('dark anodized reference', (.12, .14, .17), .7, .3),
                 Material('silver reference', (.58, .63, .68), .85, .24),
                 Material('brass reference', (.55, .31, .09), .8, .27),
                 Material('steel reference', (.48, .52, .56), .8, .25),
                 Material('black barrel reference', (.025, .03, .04), .3, .35),
                 Material('optical volume reference', (.20, .34, .42), 0, .08, opacity=.25),
                 Material('thin leaf reference', (.09, .11, .14), .65, .3)]
    densities = {1: 'aluminum_assumed', 2: 'aluminum_assumed', 3: 'brass_assumed', 4: 'steel_assumed', 5: 'aluminum_assumed', 6: 'glass_assumed', 7: 'steel_assumed'}
    rendered, invariant_parts, geometry_records = [], [], []
    snapshot, unclipped = cq.Assembly(name='IRIS09_visual_reference'), cq.Assembly(name='IRIS09_unclipped_profiles')
    body_frames = {}

    def place(name, local, world, body, material, note, frame=None):
        construction.addpart(name, local, densities[material], note)
        construction.place(body, name, local, densities[material], note=note)
        body_frames[body] = (np.eye(4) if frame is None else frame).tolist()
        rendered.append(cad_part(name, world, material=material - 1, tolerance=.06, angular=.12,
                                 group='leaves' if material == 7 else 'reference', provenance='designed-concept', role=note))
        snapshot.add(world, name=name, color=cq.Color(*materials[material - 1].color))
        geometry_records.append(dict(name=name, body=body, valid=bool(local.isValid()),
                                     solids=len(local.Solids()), volume_mm3=float(local.Volume()),
                                     note=note))

    for name, shape, material in housing_geometry(s) + lens_geometry(s):
        place(name, shape, shape, 'housing_reference', material, 'Source formula; assumed material/density; visual reference only')
    handle_frame = _frame(-motion['actuator_radians'])
    for name, local, material in handle_geometry(s):
        world = _rotate(local, -motion['actuator_radians'])
        note = 'Source handle geometry; prescribed pose without a solved drive path'
        if name.startswith('knob_mean'):
            note += f'; omitted radial knurl has amplitude {SOURCE["handle"]["knob_radius_amplitude"] * s:g} mm'
        place(name, local, world, 'handle_reference', material, note, handle_frame)
    dag = leaf_dag(time_s, s)
    evaluator = ShapeEvaluator(dag)
    invariant = evaluator.evaluate('invariant_leaf')
    visual_local = evaluator.evaluate('visual_leaf_local')
    construction.addpart('invariant_leaf_template', invariant, 'steel_assumed', 'Invariant two-disk/canted-slab geometry before stationary world clipping; no hinge hardware supplied')
    for index in range(SOURCE['leaf']['count']):
        name = f'visual_leaf_{index:02}'
        world = evaluator.evaluate(name)
        frame = leaf_frame(index, motion['hinge_radians'], s)
        place(name, visual_local, world, f'leaf_body_{index:02}', 7,
              'Pose-dependent world-clipped display leaf; its mass is not a physical invariant leaf inertia', frame)
        uncut = evaluator.evaluate(f'invariant_leaf_{index:02}')
        unclipped.add(uncut, name=f'invariant_leaf_{index:02}')
        invariant_parts.append(cad_part(f'invariant_leaf_{index:02}', uncut, material=6,
                                        tolerance=.06, angular=.12, provenance='designed-concept',
                                        role='Invariant untrimmed local profile at the reference placement'))
    sampled_clips = []
    for sample_time in (0., 5., 10.):
        measured = ShapeEvaluator(leaf_dag(sample_time, s))
        sampled_clips.append(dict(time_s=sample_time, **angles(sample_time),
                                  invariant_volume_mm3=measured.evaluate('invariant_leaf').Volume(),
                                  displayed_leaf_volume_mm3=measured.evaluate('visual_leaf_local').Volume()))
    report = dict(schema='cybrgeo-geometry-study/1', source=SOURCE['source'], parameters=parameters,
                  qualification=dict(geometry='reference CAD', kinematics='prescribed placement only',
                                     dynamics='not simulated', hardware='unqualified', engineering_acceptance=False),
                  coordinate_mapping=SOURCE['coordinate_mapping'], source_angles=motion,
                  body_frames_to_world=body_frames, density_assumptions_kg_m3=DENSITIES,
                  reference_parts=geometry_records,
                  leaf_vertical_thickness_mm=SOURCE['leaf']['vertical_thickness'] * s,
                  leaf_normal_thickness_mm=SOURCE['leaf']['vertical_thickness'] * s / math.sqrt(1 + SOURCE['leaf']['cant'] ** 2),
                  clipping_demonstration=sampled_clips,
                  retained_features=['nine circular-intersection/canted-slab leaves', '14 rounded radial housing profiles',
                                     '120 grip flutes', 'annular seam', 'two spanner notches', 'curved actuator slot',
                                     'three bored mounting ears', 'six recessed slotted screw heads', 'handle/pivot/cap',
                                     'two closed plano-convex optical volumes'],
                  limitations=['No solved pin-slot linkage, hinge pins, drive closure, force/torque path, contact or clearance proof',
                               'World clipping changes displayed leaf material; only the untrimmed template is a rigid shape',
                               f'Thumb knob uses source mean radius; omitted 28-lobe radial amplitude is {SOURCE["handle"]["knob_radius_amplitude"] * s:g} mm',
                               'Shader grain, engraving, wear, zero-thickness sensor surface, lighting and refraction are not CAD geometry',
                               'Rendered materials and densities are illustrative assumptions; material partition is for accounting only',
                               'Source sampled-clearance comment is not adopted as evidence'])
    construction.save_auxiliary_assembly('visual_reference', snapshot)
    construction.save_auxiliary_assembly('unclipped_profiles_reference', unclipped)
    native = construction.finish(QUALIFICATION, {'geometry-study.json': report, 'leaf-shape-dag.json': dag})
    views = {'hero': View(az=35, el=53, scale=1.95 * s, target=(0, 0, .22 * s),
                          title='IRIS-09 / source-derived CAD reference', note='Visual reference pose / no solved actuation or hardware qualification'),
             'top': View(az=0, el=90, scale=1.95 * s, target=(0, 0, .22 * s), projection='orthographic',
                         title='IRIS-09 / actual CAD top view', note='World-clipped leaf display at the selected source time')}
    assembly = Assembly('iris09_visual_reference', rendered, materials, views, metadata=report)
    invariant_views = {'hero': View(az=35, el=53, scale=3.25 * s, target=(0, 0, .26 * s),
                                   title='IRIS-09 / invariant local profiles', note='Outer world clip removed / original leaf material preserved')}
    return StudyBuild(native, assembly, Assembly('iris09_unclipped_profiles', invariant_parts, materials, invariant_views, metadata=report), report)


def main():
    import argparse
    from mechanism_lab.exporters import export_glb
    parser = argparse.ArgumentParser(description=QUALIFICATION)
    parser.add_argument('--output', type=Path, default=Path('outputs/iris09-reference'))
    parser.add_argument('--scene-unit-mm', type=float, default=50.)
    parser.add_argument('--time', type=float, default=5.)
    parser.add_argument('--render', action='store_true', help='Render actual CAD with the shared GEO renderer')
    args = parser.parse_args()
    result = build(args.output, scene_unit_mm=args.scene_unit_mm, time_s=args.time)
    export_glb(result.reference, args.output / 'reference.glb')
    export_glb(result.invariant, args.output / 'invariant-profiles.glb')
    if args.render:
        from mechanism_lab.render import render_still
        render_still(result.reference, args.output / 'reference.png', size=(1280, 960), intent='concept')
        render_still(result.reference, args.output / 'reference-top.png', view_name='top', size=(1280, 960), intent='concept')
        render_still(result.invariant, args.output / 'invariant-profiles.png', size=(1280, 960), intent='concept')
    print(json.dumps(dict(output=str(args.output.resolve()), parts=len(result.native.parts),
                         placements=len(result.native.placed_records), scope=QUALIFICATION), indent=2))


if __name__ == '__main__':
    main()
