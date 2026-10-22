"""Reproduce native replay checks/rendering from a fresh shared-runtime trace.

Run from repository root with native CAD dependencies:
python -m examples.engineering.arx_belt.verify_geometry --trace .../trace.json.gz
    --native outputs/engineering-arx-native --out outputs/arx-replay --render
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np

from mechanism_lab.physical_model import digest
from mechanism_lab.physical_replay import Y_UP_TO_Z_UP, pose, render_evidence
from .geometry import build_assembly
from .model import compile_model


def verify(assembly, result):
    indices = sorted({0,len(result['frames'])//4,len(result['frames'])//2,
                      3*len(result['frames'])//4,len(result['frames'])-1})
    maximum, comparisons = 0.,0
    # Independently evaluate source SI row-major transforms on display vertices;
    # there is no joint/FK/pivot term to silently apply twice.
    for index in indices:
        frame = result['frames'][index]
        for part in assembly.parts:
            record = frame['bodies'][part.motion]
            vertices = part.vertices[::max(1,len(part.vertices)//31)]
            expected = (vertices @ np.asarray(record['rotation_row_major']).reshape(3,3).T
                        + np.asarray(record['position_m'])*1000) @ Y_UP_TO_Z_UP.T
            transform = assembly.pose(part,frame['time_s'])
            actual = vertices @ transform[:3,:3].T + transform[:3,3]
            maximum = max(maximum,float(np.max(np.abs(expected-actual))))
            comparisons += len(vertices)
    rebound = pose.bind(assembly)
    midpoint = indices[len(indices)//2]
    for part in assembly.parts:
        np.testing.assert_array_equal(rebound(part,result['frames'][midpoint]['time_s']),
                                      assembly.pose(part,result['frames'][midpoint]['time_s']))
    # Native CAD and display meshes share local frame. Tessellation has bounded
    # approximation error; transformation of BRep bounds catches double-pivots.
    bound_error = 0.
    cad_parts = 0
    for part in assembly.parts:
        if part.cad is None:
            continue
        if not part.cad.isValid():
            raise ValueError(f'Invalid native replay CAD {part.name}')
        cad_parts += 1
        bb = part.cad.BoundingBox()
        exact = np.array([[bb.xmin,bb.ymin,bb.zmin],[bb.xmax,bb.ymax,bb.zmax]])
        bound_error = max(bound_error,float(np.max(np.abs(part.bounds-exact))))
    if maximum > 1e-8 or bound_error > .3:
        raise ValueError(f'Replay mismatch: {maximum} mm vertex, {bound_error} mm mesh/CAD bounds')
    return dict(schema='cybrgeo-native-replay-check/1',passed=True,
        model_sha256=result['model_sha256'],control_sha256=result['control_sha256'],
        display_mesh_sha256=assembly.metadata['display_mesh_sha256'],
        replay_state_sha256=assembly.metadata['replay_state_sha256'],
        parts=len(assembly.parts),valid_native_cad_parts=cad_parts,
        sampled_frame_indices=indices,sampled_times_s=[result['frames'][i]['time_s'] for i in indices],
        compared_vertices=comparisons,max_vertex_error_mm=maximum,
        max_native_cad_mesh_local_bounds_error_mm=bound_error,
        native_mesh_bounds_tolerance_mm=.3,cache_rebind_exact=True,
        free_workpiece_body='puck',body_bindings=assembly.metadata['body_mapping'],
        visual_limitations=assembly.metadata['visual_limitations'],
        scope='Native/display placement and recorded-state replay; no hardware qualification')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace',required=True,type=Path)
    parser.add_argument('--native',required=True,type=Path)
    parser.add_argument('--out',required=True,type=Path)
    parser.add_argument('--render',action='store_true')
    args=parser.parse_args()
    result=json.load(gzip.open(args.trace,'rt',encoding='utf-8'))
    if 'physical_source' not in result or 'task_parameters' not in result:
        physical,control,parameters=compile_model(native_directory=args.native)
        if digest(physical.to_source()) != result['model_sha256'] or digest(control) != result['control_sha256']:
            raise ValueError('Current example source does not match recorded simulation; regenerate the shared run')
        result.update(task_parameters=parameters,physical_source=physical.to_source())
    assembly=build_assembly(args.native,result)
    report=verify(assembly,result)
    report['trace_file_sha256']=hashlib.sha256(args.trace.read_bytes()).hexdigest()
    args.out.mkdir(parents=True,exist_ok=True)
    (args.out/'replay-check.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2),flush=True)
    if args.render:
        receipt=render_evidence(assembly,args.out,fps=8,size=(1120,840),still_times=[0,2.2,3.4,5,7.2,10])
        print(json.dumps({'rendered':receipt['stills'],'gif_frames':len(receipt['gif_frame_indices'])}),flush=True)


if __name__=='__main__':
    main()
