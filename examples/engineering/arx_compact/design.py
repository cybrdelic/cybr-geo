"""Pinned ARX-6 compact CSG evaluated by GEO's shared geometry core.

This is a geometry-only example, deliberately separate from the belt-drive
EngineeringProject. It does not import the cinematic renderer, run captured
controls, invent masses, or qualify the authored mechanism.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re

import numpy as np

from mechanism_lab.core import Assembly, Material, View, cad_part
from mechanism_lab.shape_dag import ShapeEvaluator, from_trees


HERE = Path(__file__).resolve().parent
COMMIT = '0003382bd6a2e96f3bed065003efec35001982f7'
SOURCE = dict(repository='https://github.com/cybrdelic/cybr-arx6-compact', commit=COMMIT,
              design_sha256='48b3dcf993f78a9e1f386c8458644bb5829d627c895b88e58e616ae14aa565a0',
              native_builder_sha256='1b2e4f087634f8ff41ad450195a2a35ef3ef6445d1bc25efbad9c797f7fb2e50',
              geometry_py_sha256='60caf4eca51910f172cc9ce884ec795a70a54714ffb5bd1c366f94107669ee87',
              extracted_pose_sha256='d8cbe609a96572e99030c152cce89a726a2e681efc2ff97ce651e0a24bb07e07',
              capture_sha256='ba938e6a1e0a829c52435a5ef165e7364bf8cb072a604bb18ff0577edf6f1b33')
SCOPE = 'Pinned source CAD at captured t=0 placement; no engineering acceptance, dynamics or hardware qualification'
SOURCE_TO_CAD = np.array([[1., 0, 0, 0], [0, 0, -1., 0], [0, 1., 0, 0], [0, 0, 0, 1.]])


def load_source():
    """Require the pinned geometry bytes and an explicit extracted pose."""
    raw = (HERE / 'data/design.json').read_bytes()
    if hashlib.sha256(raw).hexdigest() != SOURCE['design_sha256']:
        raise ValueError('Pinned design.json hash mismatch; record a new source revision before adapting it')
    design = json.loads(raw)
    pose_raw = (HERE / 'data/initial-pose.json').read_bytes()
    if hashlib.sha256(pose_raw).hexdigest() != SOURCE['extracted_pose_sha256']:
        raise ValueError('Pinned extracted pose hash mismatch')
    pose = json.loads(pose_raw)
    if pose['commit'] != COMMIT or pose['capture_sha256'] != SOURCE['capture_sha256'] or pose['time_s'] != 0:
        raise ValueError('Initial pose provenance does not match the pinned source capture')
    if design['units'] != 'mm' or len(design['parts']) != 278:
        raise ValueError('Unexpected pinned design units or part count')
    return design, pose


def body_frames(pose):
    """Source body-local mm + SI world translation -> GEO mm, Z up.

    The capture stores [position_metres(3), row-major rotation(9)]. A rigid
    frame is required for every part; absent frames never become identities.
    """
    names, rows = pose['body_names'], pose['body_poses']
    if len(names) != len(rows) or len(set(names)) != len(names):
        raise ValueError('Initial body names and poses must be unique and aligned')
    frames = {}
    for name, row in zip(names, rows):
        values = np.asarray(row, float)
        if values.shape != (12,) or not np.isfinite(values).all():
            raise ValueError(f'Invalid body pose: {name}')
        frame = np.eye(4)
        frame[:3, :3] = values[3:].reshape(3, 3)
        frame[:3, 3] = values[:3] * 1000
        if not np.allclose(frame[:3, :3].T @ frame[:3, :3], np.eye(3), atol=1e-7) or not np.isclose(np.linalg.det(frame[:3, :3]), 1., atol=1e-7):
            raise ValueError(f'Body pose is not a proper rotation: {name}')
        frames[name] = SOURCE_TO_CAD @ frame
    if 'fixed_hardware' not in frames:
        raise ValueError('Missing fixed_hardware frame')
    frames['fixed'] = frames['fixed_hardware']
    return frames


def bounds(shape):
    box = shape.BoundingBox()
    return [[getattr(box, axis + edge) for axis in ('x', 'y', 'z')] for edge in ('min', 'max')]


def geometry_dag(design):
    roots = {str(p['id']): p['shape'] for p in design['parts']}
    if len(roots) != len(design['parts']):
        raise ValueError('Duplicate source part IDs')
    return from_trees(roots, provenance=dict(SOURCE, units='body-local millimetres', scope=SCOPE))


def _material(finish):
    # Explicit display assumptions, without assigning density or physical mass.
    colors = {'darkmetal': (.10, .12, .15), 'steel': (.45, .48, .52),
              'aluminum': (.56, .60, .64), 'black': (.025, .03, .04),
              'rubber': (.025, .03, .035), 'pcb': (.05, .22, .11),
              'copper': (.52, .23, .09)}
    color = colors.get(finish, (.28, .34, .40))
    metal = 0. if finish in ('rubber', 'pcb', 'black') else .65
    return Material(f'Source finish {finish} / illustrative', color, metal, .32)


def build(output):
    """Build all source records, preserving disconnected solids and diagnostics."""
    import cadquery as cq
    from mechanism_lab.exporters import export_glb, export_step
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    design, pose = load_source()
    frames, dag = body_frames(pose), geometry_dag(design)
    evaluator = ShapeEvaluator(dag)
    finishes = sorted({p['finish'] for p in design['parts']})
    materials = [_material(finish) for finish in finishes]
    parts, records = [], []
    local_dir = output / 'body-local-step'
    local_dir.mkdir(exist_ok=True)
    for index, row in enumerate(design['parts']):
        if row['body'] not in frames:
            raise ValueError(f'Missing source body pose: {row["body"]}')
        local = evaluator.evaluate(str(row['id']))
        if not local.Solids() or local.Volume() <= 0:
            raise ValueError(f'Source part {row["id"]} has no positive-volume solid')
        actual = np.asarray(bounds(local))
        authored = np.asarray(row['bounds_mm'], float)
        overrun = float(max(0., np.max(authored[0] - actual[0]), np.max(actual[1] - authored[1])))
        name = f'{row["id"]:03d}_' + re.sub(r'[^A-Za-z0-9_-]', '_', row['name'])
        local_file = local_dir / f'{name}.step'
        cq.exporters.export(local, str(local_file))
        world = evaluator.evaluate(str(row['id']), frame=frames[row['body']])
        parts.append(cad_part(name, world, material=finishes.index(row['finish']),
                              group=row['group'], role=row['role'], provenance='designed-concept',
                              tolerance=.12, angular=.16))
        records.append(dict(id=row['id'], name=row['name'], body=row['body'],
                            valid=bool(local.isValid()), solids=len(local.Solids()),
                            volume_mm3=float(local.Volume()), actual_bounds_mm=actual.tolist(),
                            authored_bounds_mm=authored.tolist(), bounds_overrun_mm=overrun,
                            source_status=row['status'], local_step=str(local_file.relative_to(output)),
                            local_step_sha256=hashlib.sha256(local_file.read_bytes()).hexdigest()))
        if (index + 1) % 25 == 0 or index == len(design['parts']) - 1:
            print(f'Native source parts: {index + 1}/{len(design["parts"])}', flush=True)
    limits = ['Source supplier interface envelopes do not expose reducer/bearing internal geometry',
              'Static source capture frame only; no captured controls or animation replay executed',
              'No masses, densities, inertias, forces, clearances, motion envelope or hardware qualification inferred',
              'Disconnected source solids and authored-bound overruns are retained and reported, not silently repaired',
              'Display finishes are illustrative; cinematic materials and renderer are not imported']
    report = dict(schema='cybrgeo-geometry-study/1', source=SOURCE, source_revision=design['revision'],
                  qualification=dict(geometry='actual source CSG evaluated as analytic CAD',
                                     kinematics='captured static placement', dynamics='not simulated',
                                     hardware='unqualified', engineering_acceptance=False),
                  input_pose_sha256=hashlib.sha256((HERE / 'data/initial-pose.json').read_bytes()).hexdigest(),
                  recipe_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  time_s=0, source_scope=design['scope'],
                  coordinate_mapping='Source body-local mm; captured translation m multiplied by1000; source(x,y,z) -> CAD(x,-z,y)',
                  body_frames_to_world={name: frame.tolist() for name, frame in frames.items()},
                  part_count=len(parts), dag_nodes=len(dag['nodes']), parts=records,
                  disconnected_parts=[r['id'] for r in records if r['solids'] != 1],
                  authored_bound_overruns=[r['id'] for r in records if r['bounds_overrun_mm'] > .01],
                  limitations=limits)
    extents = np.array([p.bounds for p in parts])
    low, high = extents[:, 0].min(axis=0), extents[:, 1].max(axis=0)
    target = tuple((low + high) / 2)
    size = float(np.max(high - low) * .68)
    views = dict(hero=View(az=38, el=23, scale=size, target=target,
                           title='ARX-6 compact / actual source CAD',
                           note='278 source parts / captured t=0 / geometry study only'),
                 side=View(az=0, el=0, scale=size, target=target, projection='orthographic',
                           title='ARX-6 compact / source reference side',
                           note='Static geometry; no mechanical or hardware acceptance'))
    assembly = Assembly('arx6_compact_source_reference', parts, materials, views, metadata=report)
    export_step(assembly, output / 'reference.step', individual=False)
    export_glb(assembly, output / 'reference.glb')
    report['artifacts'] = {name: dict(sha256=hashlib.sha256((output / name).read_bytes()).hexdigest(),
                                    bytes=(output / name).stat().st_size)
                           for name in ('reference.step', 'reference.glb')}
    (output / 'geometry-study.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    (output / 'shape-dag.json').write_text(json.dumps(dag, separators=(',', ':')) + '\n', encoding='utf-8')
    return assembly, report


def main():
    import argparse
    parser = argparse.ArgumentParser(description=SCOPE)
    parser.add_argument('--output', type=Path, default=Path('outputs/arx-compact-reference'))
    parser.add_argument('--render', action='store_true', help='Render actual CAD using GEO services')
    parser.add_argument('--render-only', action='store_true', help='Render the verified saved GLB without rebuilding CAD')
    args = parser.parse_args()
    if args.render_only:
        from mechanism_lab.importers import import_geometry
        report = json.loads((args.output / 'geometry-study.json').read_text(encoding='utf-8'))
        glb = args.output / 'reference.glb'
        if hashlib.sha256(glb.read_bytes()).hexdigest() != report['artifacts']['reference.glb']['sha256']:
            raise ValueError('Saved GLB does not match the source build report')
        assembly = import_geometry(glb, name='arx6_compact_source_reference')
        if len(assembly.parts) != report['part_count']:
            raise ValueError('Saved GLB part count differs from source report')
        low, high = assembly.bounds
        target, size = tuple((low + high) / 2), float(np.max(high - low) * .68)
        assembly.views = dict(hero=View(az=38, el=23, scale=size, target=target,
                                         title='ARX-6 compact / actual source CAD', note='278 source parts / captured t=0 / geometry study only'),
                              side=View(az=0, el=0, scale=size, target=target, projection='orthographic',
                                        title='ARX-6 compact / source reference side', note='Static geometry; no mechanical or hardware acceptance'))
        assembly.metadata['source_build_report'] = report
    else:
        assembly, report = build(args.output)
    if args.render or args.render_only:
        from mechanism_lab.render import render_still
        render_still(assembly, args.output / 'reference.png', size=(1280, 960), intent='concept')
        render_still(assembly, args.output / 'reference-side.png', view_name='side', size=(1280, 960), intent='concept')
    print(json.dumps(dict(output=str(args.output.resolve()), parts=report['part_count'],
                         disconnected_parts=report['disconnected_parts'],
                         authored_bound_overruns=report['authored_bound_overruns'], scope=SCOPE), indent=2))


if __name__ == '__main__':
    main()
