"""Exercise the real default routing without paid inference or browser fixtures."""
from __future__ import annotations

from contextlib import contextmanager
import http.client
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

STUDIO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STUDIO))
import codex_studio as studio
from mechanical_planner import mechanical_request
from planner_codec import decode, encode, planner_schema


def static_plan():
    return {
        'name': 'Fixture flange', 'units': 'mm', 'summary': 'Static fixture',
        'assumptions': ['Nominal original part.'], 'joints': [],
        'parts': [{
            'name': 'Flange',
            'body': {'kind': 'cylinder', 'size': [40, 40, 6],
                     'position': [0, 0, 0], 'rotation': [0, 0, 0]},
            'cuts': [], 'unions': [], 'fillet': 0,
            'color': [0.3, 0.4, 0.5], 'metalness': 0.8, 'roughness': 0.3,
        }],
    }


class PlanningTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.jobs = Path(temporary.name) / 'jobs'
        changed = patch.object(studio, 'JOBS', self.jobs)
        changed.start()
        self.addCleanup(changed.stop)

    def test_short_mechanical_prompts_use_real_compiler_without_subprocess(self):
        cases = [('Build a robot arm', 'compact_arm'),
                 ('belt driven arm', 'belt_arm'),
                 ('inspection head', 'inspection_head'),
                 ('pan-tilt gimbal', 'inspection_head'),
                 ('default', 'compact_arm')]
        with patch.object(studio.subprocess, 'run', side_effect=AssertionError('Inference was invoked')):
            for prompt, architecture in cases:
                with self.subTest(prompt=prompt):
                    plan = studio.plan_design(prompt)
                    self.assertEqual(plan['construction']['architecture'], 'component_graph')
                    self.assertEqual(plan['generation']['architecture'], architecture)
                    self.assertFalse(plan['generation']['inference_used'])
                    self.assertGreater(len(plan['parts']), 32)
                    self.assertTrue(plan['joints'])
                    self.assertTrue(plan['construction']['interfaces'])
                    self.assertEqual(json.loads(json.dumps(plan)), plan)
        saved = list(self.jobs.glob('*/recipe.json'))
        self.assertEqual(len(saved), len(cases))

    def test_explicit_parameters_reach_shared_compiler(self):
        with patch.object(studio.subprocess, 'run', side_effect=AssertionError('Inference was invoked')):
            plan = studio.plan_design(mechanism={'architecture': 'compact_arm', 'parameters': {'scale': 0.8}})
            self.assertEqual(plan['construction']['parameters']['scale'], 0.8)
            for parameters in ({'scale': 0.1}, {'scale': True}, {'unknown': 2}, {'scale': float('nan')}):
                with self.subTest(parameters=parameters), self.assertRaises(ValueError):
                    studio.plan_design(mechanism={'architecture': 'compact_arm', 'parameters': parameters})

    def test_unknown_mechanisms_and_reference_names_never_fall_back(self):
        with patch.object(studio.subprocess, 'run', side_effect=AssertionError('Inference was invoked')):
            for prompt in ('Build a crank mechanism', 'IRIS-09', 'ARX-6 belt architecture', 'robot arm 500 mm long'):
                with self.subTest(prompt=prompt), self.assertRaises(ValueError):
                    studio.plan_design(prompt)
            with self.assertRaisesRegex(ValueError, 'Primitive repair'):
                studio.plan_design('robot arm', repair='Move the shaft to an arbitrary point')
        self.assertFalse(self.jobs.exists())

    def test_static_parts_retain_codec_and_official_cli_path(self):
        plan = static_plan()
        self.assertIsNone(mechanical_request('Make a mounting flange'))
        self.assertIsNone(mechanical_request('Mounting bracket for a robot arm'))
        self.assertEqual(decode(encode(plan)), plan)
        schema = planner_schema(json.loads((STUDIO / 'recipe-schema.json').read_text(encoding='utf-8')))
        self.assertEqual(schema['properties']['joints']['maxItems'], 0)

        def completed(command, **kwargs):
            output = Path(command[command.index('-o') + 1])
            output.write_text(json.dumps(encode(plan)), encoding='utf-8')
            kwargs['stdout'].write(json.dumps({'type': 'turn.completed'}) + '\n')
            return SimpleNamespace(returncode=0)

        with patch.object(studio, 'executable', return_value=['codex-test']), \
                patch.object(studio.subprocess, 'run', side_effect=completed) as run:
            result = studio.plan_design('Make a mounting flange')
        self.assertEqual(result, plan)
        self.assertEqual(run.call_count, 1)
        command = run.call_args.args[0]
        self.assertIn('--output-schema', command)
        self.assertIn('--ignore-user-config', command)

    @contextmanager
    def server(self):
        server = ThreadingHTTPServer(('127.0.0.1', 0), studio.Handler)
        port = server.server_address[1]
        with patch.object(studio, 'PORT', port):
            worker = threading.Thread(target=server.serve_forever, daemon=True)
            worker.start()
            try:
                yield port
            finally:
                server.shutdown()
                server.server_close()
                worker.join(timeout=2)

    def post(self, port, payload, token=True, path='/api/plan'):
        connection = http.client.HTTPConnection('127.0.0.1', port, timeout=30)
        headers = {'Content-Type': 'application/json', 'Origin': f'http://127.0.0.1:{port}'}
        if token:
            headers['X-Studio-Token'] = studio.TOKEN
        connection.request('POST', path, json.dumps(payload), headers)
        response = connection.getresponse()
        result = response.status, json.loads(response.read())
        connection.close()
        return result

    def test_http_compiler_path_needs_csrf_but_not_login(self):
        with self.server() as port, \
                patch.object(studio, 'signed_in', side_effect=AssertionError('Login checked for constructive request')), \
                patch.object(studio.subprocess, 'run', side_effect=AssertionError('Inference was invoked')):
            status, result = self.post(port, {'mechanism': {'architecture': 'inspection_head'}})
            self.assertEqual(status, 200, result)
            self.assertEqual(result['provider'], 'constructive')
            self.assertEqual(result['plan']['construction']['architecture'], 'component_graph')
            self.assertEqual(result['plan']['generation']['architecture'], 'inspection_head')
            status, _ = self.post(port, {'prompt': 'robot arm'}, token=False)
            self.assertEqual(status, 403)

    def test_http_static_path_keeps_login_gate(self):
        with self.server() as port, patch.object(studio, 'signed_in', return_value=False), \
                patch.object(studio.subprocess, 'run', side_effect=AssertionError('Inference was invoked')):
            status, _ = self.post(port, {'prompt': 'Make a mounting flange'})
            self.assertEqual(status, 401)

    def test_canonical_document_uses_the_same_compiler_without_inference(self):
        from mechanism_lab.mechanism import load_example
        source = load_example('inspection_head')
        source['name'] = 'Custom documented component assembly'
        with patch.object(studio.subprocess, 'run', side_effect=AssertionError('Inference was invoked')):
            plan = studio.plan_design(mechanism_source=source)
        self.assertEqual(plan['construction']['mechanism'], source)
        self.assertEqual(plan['construction']['architecture'], 'component_graph')
        self.assertEqual(plan['generation']['architecture'], 'component_graph')
        self.assertTrue(plan['construction']['dynamics']['density_kg_m3'])
        self.assertTrue(plan['construction']['engineering']['actuators'])

    def test_alias_and_document_have_identical_physical_model_identity(self):
        from mechanism_lab.engineering_pipeline import compile_source
        from mechanism_lab.mechanism import load_example
        alias = studio.plan_design(mechanism={'architecture': 'compact_arm'})
        direct = studio.plan_design(mechanism_source=load_example('compact_arm'))
        bundled, _ = compile_source('compact_arm')
        for field in ('recipe_sha256', 'mechanism_sha256', 'model_sha256'):
            self.assertEqual(alias['construction'][field], direct['construction'][field])
            self.assertEqual(alias['construction'][field], bundled['construction'][field])

    def test_http_accepts_canonical_source_and_rejects_missing_physics(self):
        from mechanism_lab.mechanism import load_example
        source = load_example('inspection_head')
        with self.server() as port, \
                patch.object(studio, 'signed_in', side_effect=AssertionError('Login checked for source compilation')):
            status, result = self.post(port, {'mechanism_source': source})
            self.assertEqual(status, 200, result)
            self.assertEqual(result['plan']['construction']['mechanism'], source)
            del source['materials']
            status, _ = self.post(port, {'mechanism_source': source})
            self.assertEqual(status, 422)

    def test_http_engineering_retains_failed_analysis_without_authorizing_motion(self):
        from mechanism_lab.mechanism import load_example
        source = load_example('inspection_head')
        def evaluate(plan, output):
            return None, dict(plan=plan, passed=False, clearance=None, simulation=None,
                              stage='complete', errors=[dict(stage='simulation', message='Declared acceleration exceeded')])
        with self.server() as port, patch('mechanism_lab.engineering_pipeline.evaluate_plan', side_effect=evaluate):
            status, result = self.post(port, {'mechanism_source': source}, path='/api/engineer')
            self.assertEqual(status, 200, result)
            self.assertFalse(result['passed'])
            self.assertEqual(result['plan']['construction']['mechanism'], source)
            self.assertEqual(result['errors'][0]['message'], 'Declared acceleration exceeded')
            status, _ = self.post(port, {'mechanism_source': source}, token=False, path='/api/engineer')
            self.assertEqual(status, 403)


if __name__ == '__main__':
    unittest.main()
