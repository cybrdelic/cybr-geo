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
        if source.is_file() and source.suffix in ('.py', '.json', '.gz', '.xml', '.md', '.mjs', '.glsl', '.png', '.brep'):
            relative = SHARE / source.relative_to(staging / 'examples/engineering')
            assert relative.as_posix() in expected, f'Authored example file missing from wheel: {relative}'
    for name in ('arx_belt', 'iris09', 'fourbar_lift'):
        assert any(path.startswith(f'{SHARE.as_posix()}/{name}/') for path in expected)
    compact_data = f'{SHARE.as_posix()}/arx_compact/data/'
    for filename in ('print-first-spec.json', 'print-first-inputs.json',
                     'print-first-upper.brep', 'print-first-fore.brep', 'refined/source.json'):
        assert compact_data + filename in expected, f'Latest compact input missing from wheel: {filename}'
    assert any(path.startswith(compact_data + 'refined/cad/') and path.endswith('.brep')
               for path in expected), 'Nested Refined BREP replacements missing from wheel'

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
    probe_directory = tmp_path / 'probe'
    probe_directory.mkdir()
    output = _run([sys.executable, '-c', probe, str(target)], cwd=probe_directory, env=env)
    assert json.loads(output.strip().splitlines()[-1])['arx_belt'] == str(
        (target / SHARE / 'arx_belt').resolve())

    # Exercise the installed share-data clients, including their direct-path
    # sibling imports. This is bounded native input/property readback, not a
    # second full Refined build or another six-article construction run.
    native_probe = '''
import json, math, pathlib, runpy, sys
import cadquery as cq
from mechanism_lab.engineering_source import project_directory, is_engineering_project
import mechanism_lab.part_geometry as geometry
target = pathlib.Path(sys.argv[1]).resolve()
assert pathlib.Path(geometry.__file__).resolve().is_relative_to(target)
directory = project_directory('arx_compact')
assert directory == target / 'share/cybr-geo/examples/engineering/arx_compact'
assert not is_engineering_project('arx_compact'), 'Geometry studies must not acquire a dynamics registration'
def entries(root):
    return {p.relative_to(root).as_posix(): (p.is_dir(), p.stat().st_size, p.stat().st_mtime_ns)
            for p in root.rglob('*')}
installed_before, cwd_before = entries(target), entries(pathlib.Path.cwd())
refined = runpy.run_path(str(directory / 'refined.py'), run_name='installed_refined_probe')
print_first = runpy.run_path(str(directory / 'print_first.py'), run_name='installed_print_first_probe')
assert entries(target) == installed_before, 'Import changed installed files'
assert entries(pathlib.Path.cwd()) == cwd_before, 'Import wrote output into the caller directory'
manifest, payloads = refined['load_bundle']()
design, pose, provenance, overrides, changes, inherited = refined['source_inputs'](manifest, payloads)
assert manifest['commit'] == refined['COMMIT'] == print_first['SOURCE']['commit']
assert len(overrides) == len(changes) > 0
assert all((directory / 'data/refined' / row['path']).is_file() for row in overrides)
inputs = print_first['verified_inputs']()
spec, *_ = print_first['specification']()
assert spec['schema'] == 'cybrgeo-print-first-test-articles/1'
reopened = []
for record in inputs['inputs']:
    path = directory / 'data' / record['file']
    solid = cq.Shape.importBrep(str(path))
    assert solid.isValid() and len(solid.Solids()) == 1
    assert math.isfinite(solid.Volume()) and solid.Volume() > 0
    reopened.append(record['file'])
assert set(reopened) == {'print-first-upper.brep', 'print-first-fore.brep'}
records = {str(row['id']): row for row in design['parts']}
resolver = geometry.PartGeometryResolver(refined['geometry_dag'](design, source=provenance), records,
    overrides=overrides, asset_root=directory / 'data/refined', source=provenance)
# Housing 2 carries an actual fillet/chamfer/envelope-clip replacement. Reading
# that installed BREP must reproduce its archived native volume/mass/tensor,
# rather than quietly evaluating its unmodified CSG record.
part = '2'
assert resolver.describe(part)['kind'] == 'brep'
measured = resolver.mass_properties(part, density_kg_m3=refined['DENSITIES'][records[part]['material']])
residuals = refined['property_residuals'](measured, changes[part]['refined'])
assert measured['valid'] and measured['solid_count'] == 1
assert all(row['passed'] for row in residuals.values()), residuals
assert entries(target) == installed_before, 'Read-only source checks changed installed files'
assert entries(pathlib.Path.cwd()) == cwd_before, 'Read-only source checks wrote output'
print(json.dumps(dict(commit=manifest['commit'], verified_bundle_files=len(payloads),
    refined_replacements=len(overrides), reopened_print_breps=sorted(reopened),
    checked_refined_part=part, property_residuals=residuals, import_wrote_files=False)))
'''
    output = _run([sys.executable, '-c', native_probe, str(target)], cwd=probe_directory, env=env)
    smoke = json.loads(output.strip().splitlines()[-1])
    assert smoke['checked_refined_part'] == '2'
    assert len(smoke['reopened_print_breps']) == 2 and not smoke['import_wrote_files']
