"""Source-rule and evidence-boundary regressions; no CAD construction needed."""
from copy import deepcopy
import importlib.util
import json
import gzip
import hashlib
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace

from mechanism_lab.reference_clock import PhaseClock

ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT/'examples/engineering/arx_belt'
SPEC = importlib.util.spec_from_file_location('arx_fault_requirements_test', EXAMPLE/'faults.py')
faults = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(faults)


def inputs():
    p = json.loads((EXAMPLE/'data/P2-baseline-parameters.json').read_text())
    p['integration'] = json.loads((EXAMPLE/'data/spec.json').read_text())
    p['motors']['belt_allowable_output_torque_Nm'] = [19.1984/1.5]*3+[3.7422/1.5]*3
    p['reference_clock'] = PhaseClock([0.,1.65,2.75,3.4,5.,6.65,7.75,8.4,10.],
                                     [0.,1.65,3.85,7.1,8.7,10.35,12.55,15.8,17.4]).source()
    steps = 10
    joints = {f'j{i+1}': dict(samples=steps+1, position_unit='rad',
        minimum_position=dict(value=(a+b)/2), maximum_position=dict(value=(a+b)/2))
        for i,(a,b) in enumerate(p['integration']['joint_hard_limits_rad'])}
    actuators = {f'motor{i+1}': dict(peak_output_torque_Nm=0., peak_current_A=0.,
        peak_motor_speed_rad_s=0., peak_voltage_V=0., rms_current_A=0.,
        transmitted_torque_estimate=dict(unit='Nm', samples=steps, peak_absolute=dict(value=0.))) for i in range(6)}
    frame = dict(time_s=0., mode='RUN', qpos=[0.]*15, qvel=[0.]*14, contacts=[],
        bodies={'puck': dict(position_m=p['workpiece_targets_m']['A'])},
        motors=[dict(current_A=0.) for _ in range(6)], rack_motor=dict(current_A=0.), brake_energy_J=[0.]*6)
    result = dict(steps=steps, start_time_s=0., duration_s=10., timestep_s=1/2400,
        frames=[deepcopy(frame),dict(deepcopy(frame),time_s=10.)], events=[],
        model_sha256='test-model',control_sha256='test-control',reference_sha256='test-reference',
        metrics=dict(monitoring_interval=dict(start_time_s=0.,integration_steps=steps),
            joints=joints,actuators=actuators,contacts=dict(samples=steps,initial_state_sampled=True,pairs=[]),
            rack_motor=dict(samples=steps,peak_current_A=.2),
            free_bodies={'puck':dict(max_imposed_generalized_force=0.,max_applied_wrench_component=0.)}))
    return p,result


class SourceFaultRequirementsTests(unittest.TestCase):
    def test_complete_source_scenarios_and_unchanged_thresholds(self):
        doc=faults.source_scenarios()
        self.assertEqual(28,len(doc['scenarios']))
        self.assertEqual({'no_counterbalance','power_loss_no_brakes','friction_loss','weak_grip'},
            {row['name'] for row in doc['scenarios'] if row['deliberate_negative_control']})
        self.assertEqual(.002,doc['thresholds']['settling_speed_rad_s'])
        self.assertEqual(1e-4,doc['thresholds']['structural_penetration_m'])
        self.assertEqual('4fe47f23e40981a5c1de524ff89fd9dbba85e826b799e4f7430337501524722d',doc['provenance']['lf_sha256'])

    def test_phase_mapping_preserves_original_clocks_and_watchdog_delay(self):
        p,_=inputs()
        rows=faults.scheduled_cases(p,['power_loss_3.2','weak_spring_double_delay_7.2','watchdog','nominal_30s'])
        by_id={row['id']:row for row in rows}
        self.assertEqual(3.2,by_id['power_loss_3.2--original-clock']['options']['motorOffTime'])
        self.assertAlmostEqual(6.1,by_id['power_loss_3.2--matched-phase']['options']['motorOffTime'])
        self.assertAlmostEqual(11.45,by_id['weak_spring_double_delay_7.2--matched-phase']['options']['motorOffTime'])
        self.assertAlmostEqual(2.25,by_id['watchdog--matched-phase']['options']['heartbeatUntil'])
        self.assertAlmostEqual(52.2,by_id['nominal_30s--same-clock-independent']['seconds'])

    def test_zero_current_does_not_hide_rotor_deceleration_belt_overload(self):
        p,result=inputs()
        self.assertTrue(all(row['passed'] for row in faults.nominal_checks(result,p)))
        result['metrics']['actuators']['motor4']['transmitted_torque_estimate']['peak_absolute']['value']=-3.2
        checks={row['name']:row for row in faults.nominal_checks(result,p)}
        self.assertTrue(checks['configured arm peak current ceilings']['passed'])
        self.assertFalse(checks['estimated belt torque including rotor deceleration within service allowance']['passed'])

    def test_missing_high_rate_observations_fail_closed(self):
        p,result=inputs()
        result['metrics'].pop('contacts')
        result['metrics']['rack_motor']['samples']=9
        result['metrics']['actuators']['motor2']['transmitted_torque_estimate']['samples']=9
        checks={row['name']:row for row in faults.nominal_checks(result,p)}
        self.assertFalse(checks['no structural proxy penetration over 0.1 mm']['passed'])
        self.assertFalse(checks['configured gripper peak current ceiling']['passed'])
        self.assertFalse(checks['estimated belt torque including rotor deceleration within service allowance']['passed'])

    def test_payload_contact_exclusion_does_not_hide_structural_contact(self):
        p,result=inputs()
        pairs=result['metrics']['contacts']['pairs']
        pairs.append(dict(geoms=['puck','finger_pad'],maximum_penetration_m=.002))
        self.assertTrue(all(row['passed'] for row in faults.ordinary_checks(result,p)))
        pairs.append(dict(geoms=['arm','bench'],maximum_penetration_m=.0002))
        self.assertFalse(next(row for row in faults.ordinary_checks(result,p) if row['name'].startswith('no structural'))['passed'])

    def test_earlier_safety_trip_cannot_count_as_requested_power_loss(self):
        p,result=inputs();case=faults.scheduled_cases(p,['power_loss_3.2'],clocks=['original-clock'])[0]
        result['duration_s']=case['seconds'];result['events']=[dict(t=2.,mode='EMERGENCY',reason='software-travel-limit')]
        result['source_fault_observation']=dict(time_s=2.,qpos_addresses=list(range(6)),dof_addresses=list(range(6)),
            q_before=[0.]*6,puck_position_before_m=p['workpiece_targets_m']['A'])
        result['frames'][-1]['time_s']=case['seconds'];result['frames'][-1]['mode']='EMERGENCY'
        evaluation=faults.evaluate_case(result,p,case)
        self.assertTrue(evaluation['numeric_acceptance_passed'])
        self.assertFalse(evaluation['scenario_valid'])
        self.assertFalse(evaluation['expected_outcome_observed'])

    def test_negative_control_requires_valid_measurement_coverage(self):
        p,result=inputs();case=faults.scheduled_cases(p,['friction_loss'],clocks=['original-clock'])[0]
        result['duration_s']=case['seconds'];result['metrics'].pop('contacts')
        evaluation=faults.evaluate_case(result,p,case)
        self.assertFalse(evaluation['measurement_coverage_valid'])
        self.assertFalse(evaluation['expected_outcome_observed'])

    def test_contact_negative_control_preserves_original_general_verdict(self):
        p,result=inputs();case=faults.scheduled_cases(p,['friction_loss'],clocks=['matched-phase'])[0]
        result['duration_s']=case['seconds']
        evaluation=faults.evaluate_case(result,p,case)
        self.assertTrue(evaluation['original_source_numeric_acceptance_passed'])
        self.assertFalse(evaluation['task_outcomes_passed'])
        self.assertTrue(evaluation['expected_outcome_observed'])

    def test_spawn_worker_compiles_and_uses_its_exact_bound_parameters(self):
        spec=importlib.util.spec_from_file_location('arx_fault_tool_test',ROOT/'tools/check_arx_faults.py')
        tool=importlib.util.module_from_spec(spec);spec.loader.exec_module(tool)
        p={'compiled':True};control={'drive':'test'};files={'project.py':'test'}
        physical=SimpleNamespace(provenance={})
        physical.to_source=lambda:dict(provenance=deepcopy(physical.provenance))
        project=SimpleNamespace(specification={'spec':True})
        calls=[]
        def compile_physics(specification,*,native_directory):
            calls.append((specification,native_directory));return physical,control,p
        project.compile_physics=compile_physics
        snapshot=dict(project='test',source_files=files,engine_files_sha256={},runner_sha256=tool._RUNNER_SHA256,native_directory=None,
            native_build_sha256=None,native_source_binding_sha256=None,
            physical_source={'provenance':{'design_source_files':files}},control=control,task_parameters=deepcopy(p))
        with patch.object(tool,'load_project',return_value=(project,files)),patch.object(tool,'_engine_identity',return_value={}),patch.object(tool,'_fault_module',return_value=faults):
            tool._initialize(snapshot)
        self.assertEqual([({'spec':True},None)],calls)
        self.assertIs(p,tool._WORKER[3])
        self.assertIsNot(snapshot['task_parameters'],tool._WORKER[3])

    def test_friction_prefix_rejects_earlier_state_change(self):
        spec=importlib.util.spec_from_file_location('arx_fault_prefix_test',ROOT/'tools/check_arx_faults.py')
        tool=importlib.util.module_from_spec(spec);spec.loader.exec_module(tool)
        frame=dict(time_s=0.,qpos=[0.],qvel=[0.],motors=[],rack_motor={},desired_q=[0.],bodies={})
        trace=dict(model_sha256='m',control_sha256='c',reference_sha256='r',timestep_s=.01,frames=[frame])
        rows=[dict(case=dict(id='nominal',name='nominal')),dict(case=dict(id='friction',name='friction_loss',options=dict(frictionChangeTime=2.)))]
        (ROOT/'outputs').mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=ROOT/'outputs') as directory:
            for row in rows:
                path=Path(directory)/row['case']['id'];path.mkdir()
                raw=gzip.compress(json.dumps(trace).encode(),mtime=0);(path/'trace.json.gz').write_bytes(raw)
                row['trace_sha256']=hashlib.sha256(raw).hexdigest()
            self.assertTrue(tool._friction_prefixes(directory,rows)[0]['passed'])
            trace['frames'][0]['qpos']=[.001]
            raw=gzip.compress(json.dumps(trace).encode(),mtime=0);(Path(directory)/'friction/trace.json.gz').write_bytes(raw)
            rows[1]['trace_sha256']=hashlib.sha256(raw).hexdigest()
            comparison=tool._friction_prefixes(directory,rows)[0]
            self.assertFalse(comparison['passed'])
            self.assertEqual(.001,comparison['maximum_qpos_qvel_absolute_difference'])


if __name__=='__main__':unittest.main()
