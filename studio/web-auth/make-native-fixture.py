"""Build independent SI-pose expectations for the actual native GLB replay probe."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import numpy as np
from examples.engineering.arx_belt.geometry import build_assembly


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace',type=Path,required=True)
    parser.add_argument('--native',type=Path,required=True)
    parser.add_argument('--report',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    result=json.load(gzip.open(args.trace,'rt',encoding='utf-8'))
    assembly=build_assembly(args.native,result,analytic=False)
    metadata=assembly.metadata
    replay=dict(schema='cybrgeo-native-replay/1',units='mm',up_axis='Z',
        interpolation='previous_recorded_state',times_s=metadata['recorded_times_s'],
        transforms_mm=metadata['recorded_body_transforms_mm'],part_bindings=metadata['recorded_part_bindings'],
        model_sha256=result['model_sha256'],control_sha256=result['control_sha256'],
        reference_sha256=result['reference_sha256'],qualification=metadata['qualification'],
        visual_limitations=metadata['visual_limitations'])
    indices=sorted({0,len(result['frames'])//4,len(result['frames'])//2,
                    3*len(result['frames'])//4,len(result['frames'])-1})
    expected=[]
    for index in indices:
        frame=result['frames'][index]
        parts=[]
        for part in assembly.parts:
            ii=sorted({0,len(part.vertices)//2,len(part.vertices)-1})
            body=frame['bodies'][part.motion]
            si=part.vertices[ii] @ np.array(body['rotation_row_major']).reshape(3,3).T/1000+body['position_m']
            # Source world is metres/Y-up. Expected core world is mm/Z-up.
            world=np.column_stack([si[:,0],-si[:,2],si[:,1]])*1000
            parts.append(dict(name=part.name,vertex_indices=ii,world_vertices_mm=world.tolist()))
        expected.append(dict(frame_index=index,time_s=frame['time_s'],parts=parts))
    args.out.mkdir(parents=True,exist_ok=True)
    (args.out/'replay.json').write_text(json.dumps(replay,separators=(',',':')),encoding='utf-8')
    fixture=dict(schema='cybrgeo-native-three-fixture/1',report=json.loads(args.report.read_text(encoding='utf-8')),
        replay=replay,expected=expected,trace_file_sha256=hashlib.sha256(args.trace.read_bytes()).hexdigest())
    (args.out/'native-three-fixture.json').write_text(json.dumps(fixture,separators=(',',':')),encoding='utf-8')
    print(json.dumps(dict(parts=len(assembly.parts),samples=len(expected),model_sha256=result['model_sha256'])),flush=True)


if __name__=='__main__':
    main()
