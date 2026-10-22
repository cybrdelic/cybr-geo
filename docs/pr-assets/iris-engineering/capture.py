"""Capture actual source-bound IRIS integrated poses; no simulation is run here."""
import argparse
from copy import deepcopy
import gzip
import hashlib
import json
import os
from pathlib import Path

import numpy as np

from mechanism_lab.core import pose_cad
from mechanism_lab.engineering_source import load_project, assert_project_source_identity
from mechanism_lab.physical_model import digest
from mechanism_lab.physical_replay import render_evidence, pose
from mechanism_lab.render import render_still, Studio, labelled


def save(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n', encoding='utf-8')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def relative(path):
    resolved = path.resolve()
    try:
        return Path(os.path.relpath(resolved, Path.cwd())).as_posix()
    except ValueError:
        # Windows cannot express a relative path across drives.
        return resolved.as_posix()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, default=Path('outputs/engineering-iris09-final'))
    parser.add_argument('--native', type=Path, help='Verified native output; defaults to the run receipt directory')
    parser.add_argument('--out', type=Path, default=Path('outputs/engineering-iris09-replay'))
    parser.add_argument('--full', action='store_true')
    parser.add_argument('--inspection', action='store_true', help='Also render a labeled transparent-rotor inspection still')
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    report = json.loads((args.run/'engineering.json').read_text(encoding='utf-8'))
    assert report['passed'], 'Only an accepted final bound run is packaged by this capture'
    trace_path = args.run/'simulation/trace.json.gz'
    with gzip.open(trace_path, 'rt', encoding='utf-8') as stream:
        trace = json.load(stream)
    project, sources = load_project('iris09')
    assert sources == report['source_files'] == trace['reference_source']['source_files']
    assert digest(trace['physical_source']) == trace['model_sha256'] == report['model_sha256']
    assert digest(trace['reference_source']) == trace['reference_sha256'] == report['simulation']['reference_sha256']
    assert digest(json.loads((args.run/'control-source.json').read_text())) == trace['control_sha256'] == report['control_sha256']
    native = args.native or Path(report['native']['directory'])
    assembly = project.replay(native, trace)
    metadata = assembly.metadata
    assembly.views['hero'].el = 67
    assembly.views['hero'].az = 235
    assembly.views['hero'].floor = False
    assembly.views['hero'].title = 'IRIS-derived / single-motor linkage'
    assembly.views['hero'].note = 'Native CAD at an actual integrated state; ideal pins, hardware unqualified'
    assembly.views['top'] = deepcopy(assembly.views['hero'])
    assembly.views['top'].el = 90
    frames = trace['frames']
    indices = sorted({0, len(frames)//4, len(frames)//2, 3*len(frames)//4, len(frames)-1})
    basis = np.asarray(metadata['world_basis_to_core'])
    expected, comparisons, maximum, cad_bound_error = [], 0, 0., 0.
    for part in assembly.parts:
        assert part.cad is not None and part.cad.isValid() and part.cad.Volume() > 0, part.name
        b = part.cad.BoundingBox()
        cad_bounds = np.array([[b.xmin,b.ymin,b.zmin],[b.xmax,b.ymax,b.zmax]])
        cad_bound_error = max(cad_bound_error, float(np.max(np.abs(cad_bounds-part.bounds))))
    for index in indices:
        frame, parts = frames[index], []
        for part in assembly.parts:
            state = frame['bodies'][part.motion]
            rot = np.asarray(state['rotation_row_major']).reshape(3,3)
            world = (part.vertices @ rot.T + np.asarray(state['position_m'])*1000) @ basis.T
            matrix = assembly.pose(part, frame['time_s'])
            actual = part.vertices @ matrix[:3,:3].T + matrix[:3,3]
            maximum = max(maximum, float(np.max(np.abs(world-actual))))
            comparisons += len(part.vertices)
            selected = sorted({0, len(part.vertices)//2, len(part.vertices)-1})
            parts.append(dict(name=part.name, vertex_indices=selected, world_vertices_mm=world[selected].tolist()))
            center = np.asarray(part.cad.Center().toTuple())
            independently_posed = (rot@center + np.asarray(state['position_m'])*1000)@basis.T
            np.testing.assert_allclose(pose_cad(part.cad,matrix).Center().toTuple(), independently_posed, atol=1e-7,rtol=0)
        expected.append(dict(frame_index=index,time_s=frame['time_s'],parts=parts))
    assert maximum < 1e-8 and cad_bound_error < .3
    prior = assembly.pose(assembly.parts[0],frames[len(frames)//2]['time_s'])
    assembly.motion_function = pose.bind(assembly)
    np.testing.assert_array_equal(prior,assembly.pose(assembly.parts[0],frames[len(frames)//2]['time_s']))
    replay = dict(schema='cybrgeo-native-replay/1', units='mm', up_axis='Z',
        interpolation='previous_recorded_state', times_s=metadata['recorded_times_s'],
        transforms_mm=metadata['recorded_body_transforms_mm'], part_bindings=metadata['recorded_part_bindings'],
        model_sha256=trace['model_sha256'],control_sha256=trace['control_sha256'],
        reference_sha256=trace['reference_sha256'],qualification=metadata['qualification'],
        visual_limitations=metadata.get('visual_limitations',[]))
    save(args.out/'native-three-fixture.json',dict(schema='cybrgeo-native-three-fixture/1',
        report=report,replay=replay,expected=expected,trace_file_sha256=sha(trace_path)))
    save(args.out/'replay-check.json',dict(schema='cybrgeo-native-replay-check/1',passed=True,
        model_sha256=trace['model_sha256'],control_sha256=trace['control_sha256'],reference_sha256=trace['reference_sha256'],
        trace_file_sha256=sha(trace_path),native_build_sha256=metadata['native_build_sha256'],
        native_source_binding_sha256=metadata['native_source_binding_sha256'],
        replay_state_sha256=metadata['replay_state_sha256'],display_mesh_sha256=metadata['display_mesh_sha256'],
        part_count=len(assembly.parts),analytical_parts_valid=True,sampled_times_s=[frames[i]['time_s'] for i in indices],
        vertex_comparisons=comparisons,max_vertex_transform_error_mm=maximum,
        max_native_mesh_bounds_difference_mm=cad_bound_error,bounds_tolerance_mm=.3,
        cache_rebind_exact=True,scope='Actual integrated poses and native CAD replay parity only; no clearance or hardware qualification'))
    midpoint = min(frames,key=lambda row:abs(row['time_s']-project.duration_s/2))['time_s']
    for filename,view,time in [('pose-start.png','hero',0),('pose-midpoint.png','hero',midpoint),('top-midpoint.png','top',midpoint)]:
        assembly.views[view].note = f'Actual integrated t={time:.4f} s / ideal pin model / hardware unqualified'
        render_still(assembly,args.out/filename,view_name=view,size=(1120,840),time_seconds=time,intent='inspection',supersample=1)
        print('Rendered '+filename,flush=True)
    if args.inspection:
        studio = Studio(assembly, size=(1120,840), floor=False, supersample=1)
        overrides = []
        try:
            studio.set_view(assembly.views['top'])
            studio.pose(midpoint)
            for part, actor in zip(assembly.parts, studio.actors):
                if part.motion == 'rotor':
                    actor.GetProperty().SetOpacity(.16)
                    overrides.append(dict(part=part.name, opacity=.16))
            labelled(studio.render(), 'IRIS-derived / linkage inspection',
                f'Actual integrated t={midpoint:.4f} s / rotor transparency added for inspection',
                'All native geometry retained / actual solved poses / ideal pins / hardware unqualified',
                tag='CYBR GEO / TRANSPARENT INSPECTION VIEW').save(args.out/'linkage-inspection.png')
        finally:
            studio.close()
        save(args.out/'linkage-inspection.json',dict(
            schema='cybrgeo-inspection-view/1', time_s=midpoint,
            trace_file_sha256=sha(trace_path), model_sha256=trace['model_sha256'],
            camera=dict(azimuth_deg=235,elevation_deg=90,image_size_px=[1120,840]),
            display_only_overrides=overrides, geometry_or_pose_changes=False,
            png_sha256=sha(args.out/'linkage-inspection.png')))
        print('Rendered linkage-inspection.png',flush=True)
    if args.full:
        receipt = render_evidence(assembly,args.out/'full',fps=8,size=(1120,840),
            still_times=[0.,2.,4.,6.,8.])
        save(args.out/'capture-inputs.json',dict(source_run=relative(args.run),native=relative(native),
            model_sha256=trace['model_sha256'],control_sha256=trace['control_sha256'],reference_sha256=trace['reference_sha256'],
            native_build_sha256=metadata['native_build_sha256'],native_source_binding_sha256=metadata['native_source_binding_sha256'],
            trace_file_sha256=sha(trace_path),engineering_file_sha256=sha(args.run/'engineering.json'),
            source_files=sources,frames=len(frames),media=receipt))
    assert_project_source_identity('iris09',sources)
    print(json.dumps(dict(passed=True,parts=len(assembly.parts),samples=len(expected),vertex_comparisons=comparisons,
        max_vertex_error_mm=maximum,max_cad_mesh_bounds_error_mm=cad_bound_error,full_media=args.full)),flush=True)


if __name__ == '__main__':
    main()
