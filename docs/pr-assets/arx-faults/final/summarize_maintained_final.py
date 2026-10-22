"""Read final maintained-suite receipts without changing acceptance or dynamics."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_suite(directory):
    summary = json.loads((directory / 'summary.json').read_text())
    inputs = json.loads((directory / 'inputs.json').read_text())
    assert summary['complete'] and not summary['errors']
    assert summary['input_sha256'] == sha(directory / 'inputs.json')
    assert set(summary['requested_cases']) == {row['case']['id'] for row in summary['results']}
    rows = []
    for row in summary['results']:
        saved = json.loads((directory / row['case']['id'] / 'checks.json').read_text())
        assert saved == row
        trace = directory / row['case']['id'] / 'trace.json.gz'
        assert sha(trace) == row['trace_sha256']
        torque = next(check for check in row['checks'] if check['name'] == 'estimated belt torque including rotor deceleration within service allowance')
        motor_torque = next(check for check in row['checks'] if check['name'] == 'motor-generated output torque within belt service allowance')
        rows.append(dict(case=row['case'], passed=row['passed'],
            expected_outcome_observed=row['expected_outcome_observed'],
            original_source_numeric_acceptance_passed=row['original_source_numeric_acceptance_passed'],
            task_outcomes_passed=row['task_outcomes_passed'], scenario_valid=row['scenario_valid'],
            measurement_coverage_valid=row['measurement_coverage_valid'],
            actual_fault_time_s=row['actual_fault_time_s'], expected_fault_time_s=row['expected_fault_time_s'],
            timestep_s=row['timestep_s'], duration_s=row['duration_s'],
            peak_belt_Nm=torque['measured'], belt_allowance_Nm=torque['limit'],
            belt_margin_Nm=[limit-value for value, limit in zip(torque['measured'], torque['limit'])],
            motor_generated_peak_Nm=motor_torque['measured'], motor_generated_allowance_Nm=motor_torque['limit'],
            motor_generated_margin_Nm=[limit-value for value, limit in zip(motor_torque['measured'], motor_torque['limit'])],
            max_speed_after_fault_deadline_rad_s=row['max_speed_after_fault_deadline_rad_s'],
            max_fault_travel_deg=row['max_fault_travel_deg'], payload=row['payload'],
            failed_checks=[check for check in row['checks']+row['task_outcome_checks'] if not check['passed']],
            placement_checks=[check for check in row['checks']+row['task_outcome_checks'] if 'placement' in check['name']],
            trace_sha256=row['trace_sha256'], model_sha256=row['model_sha256'],
            control_sha256=row['control_sha256'], reference_sha256=row['reference_sha256']))
    return dict(directory=str(directory), input_sha256=sha(directory/'inputs.json'),
        summary_sha256=sha(directory/'summary.json'), complete=summary['complete'], passed=summary['passed'],
        native_build_sha256=summary['native_build_sha256'], native_source_binding_sha256=summary['native_source_binding_sha256'],
        source_files=summary['source_files'], engine_files_sha256=summary['engine_files_sha256'],
        runner_sha256=summary['runner_sha256'], operating_policy=inputs['task_parameters']['operating_policy'],
        ordinary_rows=sum(not row['case']['deliberate_negative_control'] for row in rows),
        adverse_rows=sum(row['case']['deliberate_negative_control'] for row in rows),
        ordinary_passes=sum(row['passed'] for row in rows if not row['case']['deliberate_negative_control']),
        adverse_expected_rejections=sum(row['expected_outcome_observed'] for row in rows if row['case']['deliberate_negative_control']),
        friction_prefix_comparisons=summary['friction_prefix_comparisons'], rows=rows)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--full', type=Path, required=True)
    parser.add_argument('--refined', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args=parser.parse_args()
    full, refined = read_suite(args.full), read_suite(args.refined)
    for key in ('source_files','engine_files_sha256','runner_sha256','native_build_sha256','native_source_binding_sha256','operating_policy'):
        assert full[key] == refined[key], key
    coarse={row['case']['id']:row for row in full['rows']}
    convergence=[]
    for fine in refined['rows']:
        original=coarse[fine['case']['id'].removesuffix('--dt-0.5')]
        convergence.append(dict(case=fine['case']['id'],
            coarse_timestep_s=original['timestep_s'], fine_timestep_s=fine['timestep_s'],
            peak_belt_delta_Nm=[b-a for a,b in zip(original['peak_belt_Nm'],fine['peak_belt_Nm'])],
            coarse_belt_margin_Nm=original['belt_margin_Nm'], fine_belt_margin_Nm=fine['belt_margin_Nm'],
            coarse_settled_speed_rad_s=original['max_speed_after_fault_deadline_rad_s'],
            fine_settled_speed_rad_s=fine['max_speed_after_fault_deadline_rad_s'],
            coarse_placement=original['placement_checks'], fine_placement=fine['placement_checks']))
    old_path=Path(__file__).resolve().parents[2]/'docs/pr-assets/arx-faults/diagnostic-17.4s/full-suite-summary.json.gz'
    old_raw=gzip.decompress(old_path.read_bytes()); old=json.loads(old_raw)
    old_case=next(row for row in old['results'] if row['case']['id']=='power_loss_7.2--matched-phase')
    current_case=coarse['power_loss_7.2--matched-phase']
    diagnostic=dict(packet_path='docs/pr-assets/arx-faults/diagnostic-17.4s/full-suite-summary.json.gz',
        compressed_sha256=sha(old_path),raw_sha256=hashlib.sha256(old_raw).hexdigest(),
        native_build_sha256=old['native_build_sha256'],native_source_binding_sha256=old['native_source_binding_sha256'],
        engine_files_sha256=old['engine_files_sha256'],case=old_case['case'],
        old_max_speed_after_fault_deadline_rad_s=old_case['max_speed_after_fault_deadline_rad_s'],
        new_max_speed_after_fault_deadline_rad_s=current_case['max_speed_after_fault_deadline_rad_s'],
        old_max_fault_travel_deg=old_case['max_fault_travel_deg'],new_max_fault_travel_deg=current_case['max_fault_travel_deg'],
        old_failed_checks=[check for check in old_case['checks'] if not check['passed']],
        scope='Retained failed17.4s source-posture policy. Different frozen implementation identities are explicit; this before/after record is not a same-runtime controlled experiment. No historical receipt is relabeled or duplicated.')
    report=dict(schema='cybrgeo-arx-p5-maintained-policy-readback/1',
        source_commit='aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2',
        complete_expected_outcomes_passed=full['passed'] and refined['passed'],
        full_suite=full, refined_suite=refined, convergence=convergence, prior_policy_diagnostic=diagnostic,
        readback_script_sha256=sha(Path(__file__)),
        scope='Pinned ARX belt P5 with the explicit authored zero-lateral policy, not latest ARX. '
              'Exact source scenarios and numeric thresholds, original clock and matched phase, separate adverse task failures. '
              'Half-timestep checks cover nominal plus four critical ordinary cases only. '
              'Payload drop is reported separately and is not an ordinary source acceptance criterion. '
              'No hardware qualification or continuous whole-state-space guarantee.')
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n',encoding='utf-8',newline='\n')
    print(json.dumps(dict(passed=report['complete_expected_outcomes_passed'],
        ordinary=f"{full['ordinary_passes']}/{full['ordinary_rows']}",
        adverse=f"{full['adverse_expected_rejections']}/{full['adverse_rows']}",
        refined=len(refined['rows']), output=str(args.output)),indent=2))


if __name__ == '__main__':
    main()
