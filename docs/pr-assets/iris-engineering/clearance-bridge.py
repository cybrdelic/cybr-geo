"""Prove exact geometry/pose equivalence without relabeling an earlier audit."""
import argparse, gzip, hashlib, json, re
from pathlib import Path
from mechanism_lab.physical_model import digest

parser=argparse.ArgumentParser()
parser.add_argument('--audited-native',required=True,type=Path)
parser.add_argument('--audited-trace',required=True,type=Path)
parser.add_argument('--final-native',required=True,type=Path)
parser.add_argument('--final-trace',required=True,type=Path)
parser.add_argument('--audit',required=True,type=Path)
parser.add_argument('--audit-script',required=True,type=Path)
parser.add_argument('--out',required=True,type=Path)
args=parser.parse_args()
sha=lambda value:hashlib.sha256(value).hexdigest()

def load(native,trace_path):
    raw=(native/'native-build.json').read_bytes(); receipt=json.loads(raw)
    binding=json.loads((native/'native-source-binding.json').read_bytes())
    trace_bytes=trace_path.read_bytes(); trace=json.loads(gzip.decompress(trace_bytes))
    assert digest(trace['physical_source'])==trace['model_sha256']
    assert trace['physical_source']['provenance']['native_build_sha256']==digest(receipt)
    assert trace['physical_source']['provenance']['native_source_binding_sha256']==digest(binding)
    assert binding['native_build_sha256']==digest(receipt)
    assert binding['native_build_file_sha256']==sha(raw)
    for name,wanted in receipt['files_sha256'].items():
        assert sha((native/name).read_bytes())==wanted,(native,name)
    identity=dict(native_directory=str(native),trace_path=str(trace_path),trace_sha256=sha(trace_bytes),
        native_build_sha256=digest(receipt),native_source_binding_sha256=digest(binding),
        model_sha256=trace['model_sha256'],source_sha256=binding['source_sha256'],
        specification_sha256=binding['specification_sha256'])
    return receipt,trace,identity

def step_data(raw):
    # Discard only the STEP HEADER. Never renumber entities, round numbers,
    # reorder topology, or normalize geometric coordinates/placements.
    starts=list(re.finditer(rb'(?m)^DATA;\r?$',raw))
    assert len(starts)==1,'Expected one native STEP DATA section'
    start=starts[0].start()
    ends=list(re.finditer(rb'(?m)^ENDSEC;\r?$',raw[starts[0].end():]))
    assert len(ends)==1,'Expected one closing DATA ENDSEC'
    end=starts[0].end()+ends[0].end()
    return raw[start:end]

def entity_changes(a,b):
    def entities(raw):
        markers=list(re.finditer(rb'(?m)^#(\d+)\s*=',raw)); result={}
        for index,marker in enumerate(markers):
            end=markers[index+1].start() if index+1<len(markers) else len(raw)
            block=raw[marker.start():end]
            kind=re.match(rb'^#\d+\s*=\s*([A-Z][A-Z0-9_]*)\s*\(',block)
            result[marker.group(1).decode()]=(kind.group(1).decode() if kind else None,block)
        return result
    ea,eb=entities(a),entities(b); changed=[]
    for key in sorted(set(ea)|set(eb),key=int):
        ta,ra=ea.get(key,(None,b'')); tb,rb=eb.get(key,(None,b''))
        if ra!=rb:
            changed.append(dict(entity=key,audited_type=ta,final_type=tb,
                audited_entity_sha256=sha(ra),final_entity_sha256=sha(rb)))
    return changed

old,old_trace,old_id=load(args.audited_native,args.audited_trace)
new,new_trace,new_id=load(args.final_native,args.final_trace)
audit_bytes=args.audit.read_bytes(); audit=json.loads(audit_bytes)
for key in ('trace_sha256','native_build_sha256','native_source_binding_sha256','model_sha256'):
    assert audit['identity'][key]==old_id[key],key
assert audit['identity']['audit_script_sha256']==sha(args.audit_script.read_bytes())
old_steps=sorted(name for name in old['files_sha256'] if name.lower().endswith('.step'))
new_steps=sorted(name for name in new['files_sha256'] if name.lower().endswith('.step'))
checks=[dict(name='identical registered, placed and assembly STEP path set',passed=old_steps==new_steps)]
parts=json.loads((args.audited_native/'cad_parts.json').read_bytes())
placed=json.loads((args.audited_native/'placed-native-parts.json').read_bytes())['records']
required_steps={f'cad/{name}.step' for name in parts}|{row['step_file'] for row in placed}
assembly_steps={f'cad/{name}_subassembly.step' for name in old['body_ids']}
assert set(old_steps)==required_steps|assembly_steps,'Unclassified STEP export'
presentation_types={'MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION','STYLED_ITEM',
    'PRESENTATION_STYLE_ASSIGNMENT','SURFACE_STYLE_USAGE','SURFACE_SIDE_STYLE',
    'SURFACE_STYLE_FILL_AREA','FILL_AREA_STYLE','FILL_AREA_STYLE_COLOUR','COLOUR_RGB'}
steps=[]
for name in sorted(set(old_steps)&set(new_steps)):
    a=(args.audited_native/name).read_bytes(); b=(args.final_native/name).read_bytes()
    da,db=step_data(a),step_data(b)
    item=dict(path=name,audited_raw_sha256=sha(a),final_raw_sha256=sha(b),
        audited_data_sha256=sha(da),final_data_sha256=sha(db),data_identical=da==db,
        role='registered_or_placed' if name in required_steps else 'aggregate_assembly')
    if name in assembly_steps and da!=db:
        item['changed_entities']=entity_changes(da,db)
        item['only_presentation_entities_changed']=bool(item['changed_entities']) and all(
            change['audited_type'] in presentation_types and change['final_type'] in presentation_types
            for change in item['changed_entities'])
    steps.append(item)
checks.append(dict(name='exact STEP DATA bytes for every registered and placed native export',
    passed=bool(required_steps) and all(item['data_identical'] for item in steps if item['role']=='registered_or_placed'),count=len(required_steps)))
checks.append(dict(name='aggregate assembly DATA is unchanged or differs only in explicitly listed presentation entities',
    passed=all(item['data_identical'] or item.get('only_presentation_entities_changed',False)
        for item in steps if item['role']=='aggregate_assembly'),count=len(assembly_steps),
    exact_data_count=sum(item['data_identical'] for item in steps if item['role']=='aggregate_assembly')))
for name in ('placed-native-parts.json','cad_parts.json'):
    a=json.loads((args.audited_native/name).read_bytes()); b=json.loads((args.final_native/name).read_bytes())
    checks.append(dict(name='identical '+name,passed=digest(a)==digest(b),audited_sha256=digest(a),final_sha256=digest(b)))
old_layout=json.loads((args.audited_native/'mechanism-layout.json').read_bytes())
new_layout=json.loads((args.final_native/'mechanism-layout.json').read_bytes())
for key in ('frames','links','phi0','phi1'):
    checks.append(dict(name='identical authored '+key,passed=digest(old_layout[key])==digest(new_layout[key]),
        audited_sha256=digest(old_layout[key]),final_sha256=digest(new_layout[key])))
poses=lambda trace:[dict(time_s=frame['time_s'],bodies=frame['bodies']) for frame in trace['frames']]
a,b=poses(old_trace),poses(new_trace)
checks.append(dict(name='all recorded body poses and frame times exactly identical',passed=digest(a)==digest(b),
    audited_frames=len(a),final_frames=len(b),audited_sha256=digest(a),final_sha256=digest(b)))
checks.append(dict(name='original sampled audit passed',passed=audit['positive_separation'] and audit['no_material_intersections']))
passed=all(check['passed'] for check in checks)
result=dict(schema='cybrgeo-exact-native-pose-equivalence/1',passed=passed,
    scope='A separate exact-equivalence bridge. Original audit identities remain unchanged; only its geometric calculations apply to the final source-bound build if every check passes.',
    normalization='Registered/placed native STEP DATA bytes are compared exactly, excluding only HEADER metadata. Aggregate assembly DATA hashes are retained; any differences must be confined to the explicitly listed presentation entity types. No entity renumbering, rounding, topology reordering or coordinate normalization is performed.',
    aggregate_export_scope='The audit consumes placed native STEP files, not aggregate assemblies. No claim is made that aggregate exports are byte-identical. Any non-presentation aggregate change blocks reuse.',
    aggregate_presentation_entity_allowlist=sorted(presentation_types),
    audited=old_id,final=new_id,checks=checks,step_files=steps,
    audit=dict(path=str(args.audit),sha256=sha(audit_bytes),script_sha256=sha(args.audit_script.read_bytes()),
        pose_count=audit['pose_count'],sample_times_s=[pose['time_s'] for pose in audit['poses']]),
    bridge_script_sha256=sha(Path(__file__).read_bytes()),
    limits='The original finite sampled scope is unchanged. No new dynamics, continuous articulated-clearance or hardware-qualification claim follows from equivalence.')
args.out.parent.mkdir(parents=True,exist_ok=True); args.out.write_text(json.dumps(result,indent=2))
print(json.dumps({key:result[key] for key in ('passed','audited','final','checks')},indent=2))
raise SystemExit(0 if passed else 1)
