"""Installed engineering examples remain usable without a source checkout."""
from importlib import metadata
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import sysconfig
import tomllib
from types import SimpleNamespace
import zipfile

import pytest

from mechanism_lab.engineering_source import is_engineering_project, project_directory


ROOT = Path(__file__).resolve().parents[1]
SHARE = Path('share/cybr-geo/examples/engineering')


def _project(directory):
    directory.mkdir(parents=True)
    (directory / 'project.json').write_text('{}', encoding='utf-8')
    return directory.resolve()


def test_explicit_project_and_checkout_take_precedence(monkeypatch, tmp_path):
    from mechanism_lab import core
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(core, 'project_root', lambda: tmp_path / 'checkout')
    explicit = _project(tmp_path / 'arx_belt')
    checkout = _project(tmp_path / 'checkout/examples/engineering/iris09')
    assert project_directory('arx_belt') == explicit
    assert project_directory(explicit / 'project.json') == explicit
    assert project_directory('iris09') == checkout


def test_interpreter_data_scheme_fallback_and_explicit_missing_path(monkeypatch, tmp_path):
    from mechanism_lab import core
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(core, 'project_root', lambda: tmp_path / 'checkout')
    monkeypatch.setattr(sysconfig, 'get_path', lambda name: str(tmp_path / 'prefix'))

    def no_distribution(name):
        raise metadata.PackageNotFoundError(name)

    monkeypatch.setattr(metadata, 'distribution', no_distribution)
    installed = _project(tmp_path / 'prefix' / SHARE / 'arx_belt')
    assert project_directory('arx_belt') == installed
    study = tmp_path / 'prefix' / SHARE / 'iris09'
    study.mkdir()
    (study / 'design.py').write_text('# Authored geometry study', encoding='utf-8')
    assert project_directory('iris09') == study.resolve()
    assert not is_engineering_project('iris09')
    # A missing user-authored path must not silently select a bundled recipe.
    assert project_directory('missing/arx_belt') != installed


def test_relative_record_locates_data_outside_site_packages(monkeypatch, tmp_path):
    from mechanism_lab import core
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(core, 'project_root', lambda: tmp_path / 'checkout')
    monkeypatch.setattr(sysconfig, 'get_path', lambda name: str(tmp_path / 'other-prefix'))
    installed = _project(tmp_path / 'user-prefix' / SHARE / 'arx_belt')
    site_packages = tmp_path / 'user-prefix/lib/python3.12/site-packages'
    site_packages.mkdir(parents=True)
    record = metadata.PackagePath('../../../' + SHARE.as_posix() + '/arx_belt/project.json')
    distribution = SimpleNamespace(files=[record], locate_file=lambda path: site_packages / path)
    monkeypatch.setattr(metadata, 'distribution', lambda name: distribution)
    assert project_directory('arx_belt') == installed


def _run(command, *, cwd, env):
    __tracebackhide__ = True
    completed = subprocess.run(command, cwd=cwd, env=env, text=True,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if completed.returncode:
        pytest.fail(completed.stdout, pytrace=False)
    return completed.stdout


def test_wheel_installs_exact_example_sources_and_loads_outside_checkout(tmp_path_factory):
    """Build and install the real wheel without network or dependency installs."""
    # A short leaf also keeps setuptools' nested wheel paths usable on Windows.
    tmp_path = tmp_path_factory.mktemp('wheel')
    staging = tmp_path / 'source'
    staging.mkdir()
    for name in ('pyproject.toml', 'README.md', 'LICENSE'):
        shutil.copy2(ROOT / name, staging / name)
    shutil.copytree(ROOT / 'src', staging / 'src',
                    ignore=shutil.ignore_patterns('__pycache__', '*.egg-info', '*.pyc'))
    shutil.copytree(ROOT / 'examples/engineering', staging / 'examples/engineering',
                    ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    config = tomllib.loads((staging / 'pyproject.toml').read_text(encoding='utf-8'))
    data_files = config['tool']['setuptools']['data-files']
    expected = {str(Path(destination) / source.name).replace('\\', '/'): source.read_bytes()
                for destination, patterns in data_files.items()
                for pattern in patterns for source in staging.glob(pattern)}
    for source in (staging / 'examples/engineering').rglob('*'):
        if source.is_file() and source.suffix in ('.py', '.json', '.gz', '.xml', '.md', '.mjs', '.glsl', '.png'):
            relative = SHARE / source.relative_to(staging / 'examples/engineering')
            assert relative.as_posix() in expected, f'Authored example file missing from wheel: {relative}'
    for name in ('arx_belt', 'iris09'):
        assert any(path.startswith(f'{SHARE.as_posix()}/{name}/') for path in expected)

    wheels = tmp_path / 'wheels'
    wheels.mkdir()
    env = dict(os.environ, PYTHONPATH='', PYTHONDONTWRITEBYTECODE='1', PIP_NO_INDEX='1')
    # The repository conftest intentionally sets this for source-based tests.
    # Installed behavior must be exercised without that checkout override.
    env.pop('MECHANISM_LAB_ROOT', None)
    _run([sys.executable, '-c',
          'import sys; from setuptools.build_meta import build_wheel; build_wheel(sys.argv[1])',
          str(wheels)], cwd=staging, env=env)
    wheel, = wheels.glob('*.whl')
    with zipfile.ZipFile(wheel) as archive:
        names = archive.namelist()
        for relative, payload in expected.items():
            matches = [name for name in names if name.endswith('.data/data/' + relative)]
            assert len(matches) == 1, relative
            assert archive.read(matches[0]) == payload
        assert not any(name.startswith('mechanism_lab/examples/') for name in names)
        package_metadata, = [name for name in names if name.endswith('.dist-info/METADATA')]
        assert any(line.startswith('Requires-Dist: mujoco')
                   for line in archive.read(package_metadata).decode().splitlines())

    target = tmp_path / 'installed'
    _run([sys.executable, '-m', 'pip', 'install', '--no-deps', '--no-compile',
          '--no-index', '--target', str(target), str(wheel)], cwd=tmp_path, env=env)
    for relative, payload in expected.items():
        assert (target / relative).read_bytes() == payload
    env['PYTHONPATH'] = str(target)
    # The subprocess imports installed code, and its cwd has no checkout assets.
    probe = '''
import json, pathlib, sys
import mechanism_lab.engineering_source as source
target = pathlib.Path(sys.argv[1]).resolve()
assert pathlib.Path(source.__file__).resolve().is_relative_to(target)
resolved = {}
for design in (target / 'share/cybr-geo/examples/engineering').glob('*/design.py'):
    directory = source.project_directory(design.parent.name)
    assert directory == design.parent.resolve(), (directory, design.parent)
    assert source.is_engineering_project(design.parent.name) == (directory / 'project.json').is_file()
for descriptor in (target / 'share/cybr-geo/examples/engineering').glob('*/project.json'):
    directory = source.project_directory(descriptor.parent.name)
    assert directory == descriptor.parent.resolve(), (directory, descriptor.parent)
    project, identity = source.load_project(descriptor.parent.name)
    assert project.name == descriptor.parent.name
    assert 'project.json' in identity and 'project.py' in identity
    resolved[project.name] = str(directory)
assert 'arx_belt' in resolved
print(json.dumps(resolved))
'''
    output = _run([sys.executable, '-c', probe, str(target)], cwd=tmp_path, env=env)
    assert json.loads(output.strip().splitlines()[-1])['arx_belt'] == str(
        (target / SHARE / 'arx_belt').resolve())
