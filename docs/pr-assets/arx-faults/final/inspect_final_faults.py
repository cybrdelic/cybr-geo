"""Verify the final pinned-P5 fault packet with the Python standard library."""
import argparse
import copy
import gzip
import hashlib
import json
from pathlib import Path


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def digest(value):
    return sha(json.dumps(value,sort_keys=True,separators=(',', ':'),allow_nan=False).encode())


def replace_pointer(value, pointer, replacement):
    keys=pointer.strip('/').split('/')
    for key in keys[:-1]:
        value=value[int(key)] if isinstance(value,list) else value[key]
    key=int(keys[-1]) if isinstance(value,list) else keys[-1]
    original=value[key]; value[key]=replacement
    return original


def verify_public_derivation(root, manifest, original_packet=None):
    provenance=json.loads((root/manifest['publication_provenance']).read_bytes())
    assert provenance['schema']=='cybrgeo-public-evidence-path-derivation/1'
    assert set(provenance['derived_receipts'])=={
        'full-suite-execution.json','public-command-execution.json',
        'refinement-execution.json','refinement-execution-failed-spawn.json'}
    if original_packet:
        raw=(original_packet/'fault-evidence-manifest.json').read_bytes()
        assert sha(raw)==provenance['original_packet_manifest_sha256']
        assert len(raw)==provenance['original_packet_manifest_bytes']
        original_manifest=json.loads(raw)
        for name,record in original_manifest['files'].items():
            original=(original_packet/name).read_bytes()
            assert sha(original)==record['sha256'] and len(original)==record['bytes'],name
        for name,record in provenance['omitted_originals'].items():
            assert record['sha256']==original_manifest['files'][name]['sha256']
            assert record['bytes']==original_manifest['files'][name]['bytes']
    for name,record in provenance['derived_receipts'].items():
        public=json.loads((root/name).read_bytes())
        metadata=public.pop('public_derivation')
        assert metadata['schema']==provenance['schema']
        assert metadata['provenance_file']==manifest['publication_provenance']
        assert metadata['original_sha256']==record['original_sha256']
        assert metadata['original_bytes']==record['original_bytes']
        projection=copy.deepcopy(public)
        for change in record['substitutions']:
            assert replace_pointer(projection,change['pointer'],'<PATH-REDACTED>')==change['replacement']
        assert digest(projection)==record['unchanged_semantic_projection_sha256'],name
        if original_packet:
            raw=(original_packet/name).read_bytes()
            assert sha(raw)==record['original_sha256'] and len(raw)==record['original_bytes'],name
            original=json.loads(raw.decode('utf-8-sig'))
            for change in record['substitutions']:
                value=replace_pointer(original,change['pointer'],change['replacement'])
                assert sha(value.encode())==change['original_value_utf8_sha256']
                assert len(value.encode())==change['original_value_utf8_bytes']
            assert original==public,name
    return provenance


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--packet',type=Path,default=Path(__file__).resolve().parent)
    parser.add_argument('--extract-to',type=Path,help='Restore exact raw JSON receipts into a new directory')
    parser.add_argument('--original-packet',type=Path,help='Optional private original packet: verify receipt substitutions against original bytes')
    args=parser.parse_args(); root=args.packet
    manifest=json.loads((root/'fault-evidence-manifest.json').read_text())
    for name,record in manifest['files'].items():
        raw=(root/name).read_bytes()
        assert len(raw)==record['bytes'] and sha(raw)==record['sha256'],name
    provenance=verify_public_derivation(root,manifest,args.original_packet)
    raw_receipts={}
    for name,record in manifest['raw_receipts'].items():
        packed=(root/name).read_bytes(); raw=gzip.decompress(packed)
        assert len(packed)==record['compressed_bytes'] and sha(packed)==record['compressed_sha256'],name
        assert len(raw)==record['raw_bytes'] and sha(raw)==record['raw_sha256'],name
        raw_receipts[name.removesuffix('.gz')]=raw
    suites={}
    execution=json.loads((root/'refinement-execution.json').read_bytes())
    assert execution['returncode']==0 and execution['process_succeeded']
    assert execution['wrapper_sha256']==provenance['omitted_originals']['run_final_refinement.ps1']['sha256']
    assert execution['explicit_thread_environment']=={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'2','LP_NUM_THREADS':'2'}
    full_execution=json.loads((root/'full-suite-execution.json').read_bytes())
    assert full_execution['process_exit_code']==0 and full_execution['terminal_passed']
    assert full_execution['summary_sha256']==sha(raw_receipts['full-suite-summary.json'])
    public_execution=json.loads((root/'public-command-execution.json').read_bytes())
    assert public_execution['passed'] and all(stage['passed'] for stage in public_execution['stages'])
    binding=json.loads((root/'native-source-binding.json').read_text())
    native=json.loads((root/'native-build.json').read_text())
    for prefix,count in (('full-suite',49),('refined-suite',5)):
        inputs_raw=raw_receipts[prefix+'-inputs.json']; inputs=json.loads(inputs_raw)
        summary=json.loads(raw_receipts[prefix+'-summary.json'])
        compact=json.loads((root/(prefix+'-summary.json')).read_text())
        del compact['evidence_derivation']
        for row in compact['results']:
            link=row.pop('reference_source_link')
            reference=dict(source_files=inputs['source_files'],task_parameters=inputs['task_parameters'])
            assert digest(reference)==link['semantic_sha256']
            row['reference_source']=reference
        assert compact==summary
        assert summary['input_sha256']==sha(inputs_raw)
        assert summary['complete'] and summary['passed'] and not summary['errors']
        assert len(summary['results'])==count and all(row['expected_outcome_observed'] for row in summary['results'])
        assert summary['native_build_sha256']==digest(native)
        assert summary['native_source_binding_sha256']==digest(binding)
        assert summary['source_files']==binding['source_files']
        assert all(row['passed'] for row in summary['friction_prefix_comparisons'])
        travel=json.loads(raw_receipts[prefix+'-gripper-travel.json'])
        assert travel['suite_summary_sha256']==sha(raw_receipts[prefix+'-summary.json'])
        assert travel['case_run_count']==count and travel['input_sha256']==summary['input_sha256']
        assert travel['native_build_sha256']==digest(native) and travel['native_source_binding_sha256']==digest(binding)
        assert {r['case']:r['trace_sha256'] for r in travel['scenarios']}=={r['case']['id']:r['trace_sha256'] for r in summary['results']}
        for row in travel['scenarios']:
            for jaw in row['jaws'].values():
                assert jaw['observer_samples']==jaw['integration_steps']+1
        suites[prefix]=dict(rows=count,passed=True,ordinary=sum(not r['case']['deliberate_negative_control'] for r in summary['results']),
            adverse=sum(r['case']['deliberate_negative_control'] for r in summary['results']),
            maximum_jaw_bound_residual_m=travel['maximum_observed_authored_bound_residual_m'])
    if args.extract_to:
        target=args.extract_to.resolve(); target.mkdir(parents=True,exist_ok=False)
        for name,raw in raw_receipts.items():
            destination=target/name
            assert destination.resolve().is_relative_to(target)
            destination.write_bytes(raw)
    print(json.dumps(dict(verified_files=len(manifest['files']),source_revision=manifest['source_revision_label'],
        public_execution_derivatives=4, original_execution_bytes_verified=args.original_packet is not None,
        exact_numeric_receipts=len(raw_receipts),suites=suites),indent=2))


if __name__=='__main__': main()
