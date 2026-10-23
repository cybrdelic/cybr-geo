"""Read saved, hash-verified suite traces; never modify model/acceptance rules."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def walk(node):
    yield node
    for child in node['children']:
        yield from walk(child)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('directory',type=Path);args=parser.parse_args()
    root=args.directory;summary=json.loads((root/'summary.json').read_text());raw=(root/'inputs.json').read_bytes()
    if hashlib.sha256(raw).hexdigest()!=summary['input_sha256']:
        raise ValueError('Suite input snapshot digest mismatch')
    inputs=json.loads(raw)
    limits={}
    for node in walk(inputs['physical_source']['physics']):
        attrs=node['attributes']
        if node['tag']=='joint' and attrs.get('name') in ('jaw_left','jaw_right'):
            if attrs.get('type')!='slide':raise ValueError('Expected the authored prismatic jaw joint')
            limits[attrs['name']]=[float(x) for x in attrs['range'].split()]
    if set(limits)!={'jaw_left','jaw_right'}:raise ValueError('Both authored jaw bounds are required')
    rows=[]
    for row in summary['results']:
        trace_path=root/row['case']['id']/'trace.json.gz';trace_bytes=trace_path.read_bytes()
        if hashlib.sha256(trace_bytes).hexdigest()!=row['trace_sha256']:
            raise ValueError('Trace hash mismatch: '+str(trace_path))
        trace=json.loads(gzip.decompress(trace_bytes))
        jaws={}
        for name,(low,high) in limits.items():
            record=trace['metrics']['joints'][name]
            if record['samples']!=trace['steps']+1 or record['position_unit']!='m':
                raise ValueError('Incomplete per-step prismatic observation: '+name)
            minimum,maximum=record['minimum_position'],record['maximum_position']
            jaws[name]=dict(authored_range_m=[low,high],minimum_position=minimum,maximum_position=maximum,
                lower_margin_m=minimum['value']-low,upper_margin_m=high-maximum['value'],
                lower_bound_residual_m=max(0.,low-minimum['value']),
                upper_bound_residual_m=max(0.,maximum['value']-high),
                observer_samples=record['samples'],integration_steps=trace['steps'],
                peak_absolute_speed=record['peak_absolute_speed'])
        rows.append(dict(case=row['case']['id'],jaws=jaws,trace_sha256=row['trace_sha256'],
            model_sha256=trace['model_sha256'],control_sha256=trace['control_sha256'],reference_sha256=trace['reference_sha256'],
            runtime_implementation=trace['runtime_implementation'],
            native_build_sha256=summary['native_build_sha256'],native_source_binding_sha256=summary['native_source_binding_sha256']))
    maximum=max((jaw[field] for row in rows for jaw in row['jaws'].values()
        for field in ('lower_bound_residual_m','upper_bound_residual_m')),default=None)
    report=dict(schema='cybrgeo-arx-belt-gripper-travel-readback/1',case_run_count=len(rows),
        suite_summary_sha256=hashlib.sha256((root/'summary.json').read_bytes()).hexdigest(),input_sha256=summary['input_sha256'],
        native_build_sha256=summary['native_build_sha256'],native_source_binding_sha256=summary['native_source_binding_sha256'],
        source_files=summary['source_files'],maximum_observed_authored_bound_residual_m=maximum,scenarios=rows,
        readback_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        scope='Readback of pinned ARX belt P5 source-bound traces at aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2, including initial state and every integrated step. '
              'Observed residuals against authored slide ranges; no new acceptance tolerance and no claim about the distinct compact P6 design or latest ARX revisions.')
    (root/'gripper-travel.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n')
    print(json.dumps(dict(scenarios=len(rows),maximum_observed_authored_bound_residual_m=maximum)))


if __name__=='__main__':main()
