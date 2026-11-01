"""Package completed fault evidence, preserving exact raw receipts separately.

This is an evidence readback, not a simulation or an acceptance-rule change.
"""
from pathlib import Path
import copy
import gzip
import hashlib
import io
import json
import tarfile

ROOT = Path(__file__).resolve().parents[2]
STAGE = ROOT/'outputs/arx-fault-evidence-stage-v2'
DIAG = ROOT/'outputs/fault-diagnosis'
TARGETS = ('power_loss_3.2', 'weak_spring_double_delay_3.2', 'weak_spring_double_delay_7.2')
CURRENT_TARGETS = TARGETS+('power_loss_7.2',)


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def digest(value):
    return sha(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode())


def write(name, value):
    (STAGE/name).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n', encoding='utf-8', newline='\n')


def preserve(path, name, manifest):
    raw = path.read_bytes()
    compressed = gzip.compress(raw, mtime=0)
    assert gzip.decompress(compressed) == raw
    (STAGE/name).write_bytes(compressed)
    manifest[name] = dict(source_path=path.relative_to(ROOT).as_posix(), raw_sha256=sha(raw),
                          raw_bytes=len(raw), compressed_sha256=sha(compressed), compressed_bytes=len(compressed),
                          encoding='gzip; mtime=0; exact source bytes')


def suite(directory, prefix, manifest):
    raw = (directory/'summary.json').read_bytes()
    summary = json.loads(raw)
    inputs_raw = (directory/'inputs.json').read_bytes()
    assert summary['input_sha256'] == sha(inputs_raw)
    assert summary['complete'] and not summary['errors']
    inputs = json.loads(inputs_raw)
    preserve(directory/'inputs.json', prefix+'-inputs.json.gz', manifest)
    preserve(directory/'summary.json', prefix+'-summary.json.gz', manifest)
    preserve(directory/'gripper-travel.json', prefix+'-gripper-travel.json.gz', manifest)
    compact = copy.deepcopy(summary)
    expected_reference = dict(source_files=inputs['source_files'], task_parameters=inputs['task_parameters'])
    for row in compact['results']:
        assert row['reference_source'] == expected_reference
        row['reference_source_link'] = dict(input_file=prefix+'-inputs.json.gz',
            source_files_pointer='/source_files', task_parameters_pointer='/task_parameters',
            semantic_sha256=digest(row.pop('reference_source')))
    compact['evidence_derivation'] = dict(raw_summary_file=prefix+'-summary.json.gz', raw_summary_sha256=sha(raw),
        raw_summary_bytes=len(raw), operation='Only repeated results[].reference_source objects are replaced with exact input-file pointers; every other original field is preserved.',
        reconstruction='For each row, read the two indicated input fields into {source_files, task_parameters}; verify semantic_sha256, restore reference_source, and remove reference_source_link. The gzip file preserves exact original serialized bytes.')
    write(prefix+'-summary.json', compact)
    # Prove complete value-level reconstruction; exact raw bytes are retained above.
    reconstructed = copy.deepcopy(compact)
    del reconstructed['evidence_derivation']
    for row in reconstructed['results']:
        assert row.pop('reference_source_link')['semantic_sha256'] == digest(expected_reference)
        row['reference_source'] = expected_reference
    assert reconstructed == summary
    return summary


def check(row, name):
    return next(item for item in row['checks']+row['task_outcome_checks'] if item['name'] == name)


def comparison(full, refined, baseline):
    rows = []
    for summary, scope in ((full, 'final'), (refined, 'half-timestep')):
        for row in summary['results']:
            if row['case']['name'] not in CURRENT_TARGETS and row['case']['name'] not in ('nominal', 'dt1200', 'dt4800', 'weak_spring'):
                continue
            torque = check(row, 'estimated belt torque including rotor deceleration within service allowance')
            rows.append(dict(scope=scope, case=row['case'], timestep_s=row['timestep_s'], duration_s=row['duration_s'],
                passed=row['passed'], original_source_numeric_acceptance_passed=row['original_source_numeric_acceptance_passed'],
                task_outcomes_passed=row['task_outcomes_passed'], measurement_coverage_valid=row['measurement_coverage_valid'],
                scenario_valid=row['scenario_valid'], expected_fault_time_s=row['expected_fault_time_s'],
                actual_fault_time_s=row['actual_fault_time_s'], events=row['events'],
                peak_belt_Nm=torque['measured'], belt_allowance_Nm=torque['limit'],
                belt_allowance_margin_Nm=[b-a for a,b in zip(torque['measured'], torque['limit'])],
                max_fault_travel_deg=row['max_fault_travel_deg'],
                max_speed_after_fault_deadline_rad_s=row['max_speed_after_fault_deadline_rad_s'], payload=row['payload'],
                placement_checks=[item for item in row['checks']+row['task_outcome_checks'] if 'placement' in item['name']],
                model_sha256=row['model_sha256'], control_sha256=row['control_sha256'], reference_sha256=row['reference_sha256'],
                trace_sha256=row['trace_sha256'], native_build_sha256=summary['native_build_sha256'],
                native_source_binding_sha256=summary['native_source_binding_sha256'], runtime_implementation=row['runtime_implementation']))
    convergences = []
    for name in CURRENT_TARGETS:
        base = next(r for r in rows if r['scope']=='final' and r['case']['name']==name and r['case']['clock_scope']=='matched-phase')
        fine = next(r for r in rows if r['scope']=='half-timestep' and r['case']['name']==name)
        convergences.append(dict(case=name, coarse_timestep_s=base['timestep_s'], fine_timestep_s=fine['timestep_s'],
            peak_belt_delta_Nm=[b-a for a,b in zip(base['peak_belt_Nm'], fine['peak_belt_Nm'])],
            peak_belt_relative_delta=[(b-a)/a if a else None for a,b in zip(base['peak_belt_Nm'], fine['peak_belt_Nm'])],
            coarse_margin_Nm=base['belt_allowance_margin_Nm'], fine_margin_Nm=fine['belt_allowance_margin_Nm'],
            coarse_settled_speed_rad_s=base['max_speed_after_fault_deadline_rad_s'], fine_settled_speed_rad_s=fine['max_speed_after_fault_deadline_rad_s']))
    return dict(schema='cybrgeo-arx-fault-comparison/1', source_commit='aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2',
        baseline=baseline, current_rows=rows, matched_phase_refinement=convergences,
        qualification='Applies only to pinned ARX belt P5 at aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2, not latest ARX. Historical baseline uses exact 1c216fda shared runtime and retained native-derived physical/control/task snapshot; current cases use separately identified fresh native construction and frozen measured runtime. Values are not asserted to be the identical upstream WASM model. Numeric emergency stopping and free-payload retention remain separate. Timestep comparison is bounded evidence, not hardware qualification or an all-state motion envelope.')


def baseline_bundle(manifest):
    paths = [DIAG/name for name in ('diagnose_policy.py', 'diagnose_shared.py', 'diagnose_retiming.py',
             'capture_runtime.py', 'baseline-1c216fda.json', 'baseline-pinned-runtime-1c216fda.json')]
    paths += sorted((DIAG/'baseline_runtime').glob('*.py'))
    members = []
    stream = io.BytesIO()
    with tarfile.open(fileobj=stream, mode='w', format=tarfile.USTAR_FORMAT) as tar:
        for path in paths:
            raw = path.read_bytes(); name = path.relative_to(ROOT).as_posix()
            info = tarfile.TarInfo(name); info.size=len(raw); info.mtime=0; info.mode=0o644
            tar.addfile(info, io.BytesIO(raw))
            members.append(dict(path=name, sha256=sha(raw), bytes=len(raw)))
    raw = stream.getvalue(); compressed=gzip.compress(raw,mtime=0)
    (STAGE/'baseline-reproduction.tar.gz').write_bytes(compressed)
    manifest['baseline-reproduction.tar.gz'] = dict(raw_sha256=sha(raw), raw_bytes=len(raw),
        compressed_sha256=sha(compressed), compressed_bytes=len(compressed), members=members,
        encoding='USTAR with fixed member metadata; gzip mtime=0; original member bytes')
    source = DIAG/'policy-source-transfer-1-descent-1'
    (STAGE/'baseline-shared-summary.json').write_bytes((source/'summary.json').read_bytes())
    for name in TARGETS:
        preserve(source/(name+'--original-clock')/'diagnosis.json', 'baseline-shared-'+name+'.json.gz', manifest)
        upstream = ROOT.parent/'arx-belt-source/ARX-6-P5-Engineering/integration/results'/(name+'-metrics.json')
        raw = upstream.read_bytes()
        target = 'upstream-source-'+name+'.json'
        (STAGE/target).write_bytes(raw)
        manifest[target] = dict(source_repository='cybrdelic/cybr-arx6-belt',source_commit='aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2',
            source_path='ARX-6-P5-Engineering/integration/results/'+upstream.name, raw_bytes=len(raw), raw_sha256=sha(raw))
    return json.loads((source/'summary.json').read_text())


def main():
    STAGE.mkdir(exist_ok=False)
    manifest = {}
    full = suite(ROOT/'outputs/engineering-arx-faults-final','full-suite',manifest)
    refined = suite(ROOT/'outputs/engineering-arx-faults-refined','refined-suite',manifest)
    baseline = baseline_bundle(manifest)
    write('comparative-metrics.json', comparison(full,refined,baseline))
    full_travel=json.loads(gzip.decompress((STAGE/'full-suite-gripper-travel.json.gz').read_bytes()))
    fine_travel=json.loads(gzip.decompress((STAGE/'refined-suite-gripper-travel.json.gz').read_bytes()))
    write('gripper-travel.json', dict(schema='cybrgeo-arx-gripper-travel-index/1',
        full_suite=dict(cases=full_travel['case_run_count'],maximum_bound_residual_m=full_travel['maximum_observed_authored_bound_residual_m'],file='full-suite-gripper-travel.json.gz'),
        refined_suite=dict(cases=fine_travel['case_run_count'],maximum_bound_residual_m=fine_travel['maximum_observed_authored_bound_residual_m'],file='refined-suite-gripper-travel.json.gz'),
        native_build_sha256=full['native_build_sha256'],native_source_binding_sha256=full['native_source_binding_sha256'],
        scope=full_travel['scope']))
    (STAGE/'readback_final_suite.py').write_bytes((DIAG/'readback_final_suite.py').read_bytes())
    (STAGE/'inspect_fault_evidence.py').write_bytes((DIAG/'inspect_fault_evidence.py').read_bytes())
    (STAGE/'curate_faults_packet.py').write_bytes(Path(__file__).read_bytes())
    files={p.name:dict(bytes=p.stat().st_size,sha256=sha(p.read_bytes())) for p in STAGE.iterdir() if p.is_file()}
    write('fault-evidence-manifest.json',dict(schema='cybrgeo-arx-fault-evidence-packet/1',files=files,raw_receipts=manifest,
        full_count=len(full['results']),refined_count=len(refined['results']),
        policy_label='Retained diagnostic: 17.4-second clock with unchanged source posture. Full suite failed one ordinary case; not a final accepted default.',
        suite_acceptance=dict(full_suite=full['passed'],refined_suite=refined['passed']),
        source_repository='cybrdelic/cybr-arx6-belt', source_commit='aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2', source_revision_label='Pinned P5; not latest ARX',
        scope='Exact raw receipts retained losslessly; human-readable summaries are independently reconstruction-checked. No full traces included. Applies to the pinned P5 source only; latest-source integration is separate and pending.'))
    print(json.dumps(dict(files=[dict(name=p.name,bytes=p.stat().st_size) for p in STAGE.iterdir()],total_bytes=sum(p.stat().st_size for p in STAGE.iterdir())),indent=2))


if __name__=='__main__': main()
