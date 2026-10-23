"""Studio executes the shared engineering default; clients cannot upload code."""
import http.client
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import codex_studio as studio


@pytest.fixture
def server(tmp_path, monkeypatch):
    monkeypatch.setattr(studio, 'JOBS', tmp_path/'jobs')
    monkeypatch.setattr(studio, 'PROJECT_JOBS', {})
    monkeypatch.setattr(studio, 'signed_in', lambda: False)
    instance = ThreadingHTTPServer(('127.0.0.1', 0), studio.Handler)
    monkeypatch.setattr(studio, 'PORT', instance.server_address[1])
    worker = threading.Thread(target=instance.serve_forever, daemon=True)
    worker.start()
    yield instance.server_address[1]
    instance.shutdown()
    instance.server_close()
    worker.join(timeout=2)


def request(port, path, body=None, *, token=True):
    connection = http.client.HTTPConnection('127.0.0.1', port, timeout=10)
    headers = {'Origin': f'http://127.0.0.1:{port}'}
    if token:
        headers['X-Studio-Token'] = studio.TOKEN
    connection.request('GET' if body is None else 'POST', path,
                       None if body is None else json.dumps(body), headers)
    response = connection.getresponse()
    data, content_type = response.read(), response.getheader('Content-Type')
    status = response.status
    connection.close()
    return status, json.loads(data) if content_type == 'application/json' else data


def finished(port, job):
    for _ in range(100):
        status, record = request(port, job['status_url'])
        assert status == 200
        if record['status'] in ('succeeded', 'failed'):
            return record
        time.sleep(.01)
    raise AssertionError('Registered project job did not complete')


def test_default_discovery_and_plain_request_use_shared_project(server):
    status, result = request(server, '/api/examples')
    assert status == 200 and result['default_example'] == 'arx_belt'
    assert result['default_source'] == studio.project_selection()
    status, result = request(server, '/api/plan', {'prompt': 'Build a robot arm'})
    assert status == 200 and result == dict(plan=studio.project_selection(), provider='engineering-project')
    assert studio.plan_design() == studio.project_selection()


def test_no_client_paths_code_or_missing_csrf(server):
    assert request(server, '/api/project-jobs', {'project': 'arx_belt'}, token=False)[0] == 403
    for payload in ({'project': '../arbitrary.py'}, {'project': 'arx_belt', 'source': 'execute'}, []):
        assert request(server, '/api/project-jobs', payload)[0] == 400


def test_success_publishes_only_bound_recorded_assets(server, monkeypatch):
    calls = []
    def generate(project, directory):
        calls.append(project)
        directory.mkdir(parents=True)
        (directory/'assembly.glb').write_bytes(b'actual exporter result fixture')
        metadata = dict(recorded_times_s=[0., 1.], recorded_body_transforms_mm={'body': []},
                        recorded_part_bindings={'part': 'body'}, replay_state_sha256='states')
        return SimpleNamespace(metadata=metadata), dict(passed=True, model_sha256='physical',
            control_sha256='control', simulation={'reference_sha256': 'reference'},
            qualification={'hardware': 'not qualified'})
    monkeypatch.setattr('mechanism_lab.engineering_pipeline.generate_project', generate)
    status, job = request(server, '/api/project-jobs', {'project': 'arx_belt'})
    assert status == 202
    result = finished(server, job)
    assert result['status'] == 'succeeded' and calls == ['arx_belt']
    status, replay = request(server, result['assets']['replay'])
    assert status == 200 and replay['interpolation'] == 'previous_recorded_state'
    assert replay['schema'] == 'cybrgeo-native-replay/1' and replay['reference_sha256'] == 'reference'
    assert replay['model_sha256'] == 'physical' and replay['part_bindings'] == {'part': 'body'}
    assert request(server, result['assets']['glb'])[1] == b'actual exporter result fixture'
    assert request(server, job['status_url']+'/assets/control-source.json')[0] == 404


@pytest.mark.parametrize('exception', [False, True])
def test_failed_project_never_exposes_accepted_replay(server, monkeypatch, exception):
    def generate(project, directory):
        if exception:
            raise ValueError('Construction interface failed')
        return None, dict(passed=False, requirements={'passed': False})
    monkeypatch.setattr('mechanism_lab.engineering_pipeline.generate_project', generate)
    _, job = request(server, '/api/project-jobs', {'project': 'arx_belt'})
    result = finished(server, job)
    assert result['status'] == 'failed' and not result['assets']
    assert request(server, job['status_url']+'/assets/assembly.glb')[0] == 409
