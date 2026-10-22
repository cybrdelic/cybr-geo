"""Trusted local engineering recipes using one construction/physics protocol.

Examples supply authored geometry and tasks. They do not supply alternative
CAD, mass-accounting, contact, motor, solver or rendering engines.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import importlib
import importlib.abc
import importlib.util
import json
import os
from pathlib import Path
import sys
import threading
import types
from typing import Callable


@dataclass
class EngineeringProject:
    name: str
    specification: dict
    construct: Callable
    compile_physics: Callable
    reference: Callable
    replay: Callable
    requirements: Callable


def project_directory(source):
    """Resolve an explicit project, checkout example, or installed example.

    Wheels install the maintained example tree as data files under ``share``.
    Distribution-root and RECORD lookup also support pip's ``--target`` and
    user schemes, whose data location can differ from the normal prefix.
    """
    from importlib import metadata
    import sysconfig

    from .core import project_root
    path = Path(source).expanduser()
    if path.exists():
        if path.is_file() and path.name == 'project.json':
            path = path.parent
        return path.resolve()

    path = project_root() / 'examples' / 'engineering' / str(source)
    if path.is_file() and path.name == 'project.json':
        path = path.parent
    if path.is_dir():
        return path.resolve()

    # Installed lookup is only for a recipe name, never an unresolved path.
    name = str(source)
    if name not in ('', '.', '..') and Path(name).name == name:
        relative = Path('share/cybr-geo/examples/engineering') / name
        try:
            distribution = metadata.distribution('cybr-geo')
        except metadata.PackageNotFoundError:
            distribution = None
        # pip --target places data beside packages, but its installed RECORD
        # need not retain those data entries after relocating the wheel.
        if distribution:
            installed = Path(distribution.locate_file(relative))
            if any((installed / entrypoint).is_file() for entrypoint in ('project.json', 'design.py')):
                return installed.resolve()
        files = (distribution.files or []) if distribution else []
        # Geometry studies have design.py but intentionally no project.json:
        # locating them must not register a dynamics/acceptance implementation.
        for entrypoint in ('project.json', 'design.py'):
            marker = relative / entrypoint
            for item in files:
                if item.parts[-len(marker.parts):] == marker.parts:
                    installed = Path(distribution.locate_file(item))
                    if installed.is_file():
                        return installed.parent.resolve()
        installed = Path(sysconfig.get_path('data')) / relative
        if any((installed / entrypoint).is_file() for entrypoint in ('project.json', 'design.py')):
            return installed.resolve()
    return path.resolve()


def is_engineering_project(source):
    return (project_directory(source) / 'project.json').is_file()


_SOURCE_EXCLUDES = frozenset(('.git', '__pycache__', '.venv', 'node_modules', 'output', 'outputs', 'evidence'))
_SNAPSHOT_LOCK = threading.RLock()


def _project_snapshot(path):
    """Bound the executable source closure to this project, never its workspace.

    Authored Python (including nested modules), project.json and all data/**
    files participate. Runtime outputs/caches are excluded. Other input assets
    must live under data/ to participate in the accepted project identity.
    """
    root = Path(path).resolve()
    snapshot = {}
    for directory, children, filenames in os.walk(root, followlinks=False):
        location = Path(directory).relative_to(root)
        inside_data = bool(location.parts) and location.parts[0] == 'data'
        children[:] = sorted(name for name in children if inside_data or name not in _SOURCE_EXCLUDES)
        for name in children:
            child = Path(directory) / name
            if child.is_symlink():
                raise ValueError('Project source directories must not be symbolic links')
        for name in sorted(filenames):
            file = Path(directory) / name
            relative = file.relative_to(root)
            if not (relative.as_posix() == 'project.json' or file.suffix == '.py' or relative.parts[0] == 'data'):
                continue
            if file.is_symlink() or not file.resolve().is_relative_to(root):
                raise ValueError('Project source files must remain inside their declared project')
            snapshot[relative.as_posix()] = file.read_bytes()
    return snapshot


def project_source_identity(source):
    """Current bounded source/data identity, suitable for acceptance readback."""
    return {name: hashlib.sha256(payload).hexdigest()
            for name, payload in _project_snapshot(project_directory(source)).items()}


def assert_project_source_identity(source, expected):
    """Reject code/data edits after load before accepting generated outputs."""
    current = project_source_identity(source)
    if current != expected:
        changed = sorted(name for name in set(current) | set(expected) if current.get(name) != expected.get(name))
        raise ValueError('Engineering project source changed after load: ' + ', '.join(changed))
    return current


class _SnapshotLoader(importlib.abc.Loader):
    def __init__(self, path, payload):
        self.path, self.payload = str(path), payload

    def create_module(self, spec):
        return None

    def exec_module(self, module):
        # Compile the captured bytes directly. Timestamp/size validated pyc
        # files can otherwise execute stale code after a rapid same-size edit.
        module.__file__ = self.path
        exec(compile(self.payload, self.path, 'exec', dont_inherit=True), module.__dict__)


class _SnapshotFinder(importlib.abc.MetaPathFinder):
    def __init__(self):
        self.packages = {}

    def find_spec(self, fullname, path=None, target=None):
        package, _, suffix = fullname.partition('.')
        entry = self.packages.get(package)
        if entry is None or not suffix:
            return None
        root, sources = entry
        relative = suffix.replace('.', '/')
        for filename, is_package in ((relative + '/__init__.py', True), (relative + '.py', False)):
            if filename in sources:
                loader = _SnapshotLoader(root / filename, sources[filename])
                return importlib.util.spec_from_loader(fullname, loader,
                    origin=str(root / filename), is_package=is_package)
        if any(name.startswith(relative + '/') for name in sources):
            # Namespace packages still resolve every imported child through
            # this snapshot; no normal filesystem source loader is used.
            spec = importlib.machinery.ModuleSpec(fullname, loader=None, is_package=True)
            spec.submodule_search_locations = []
            return spec
        raise ModuleNotFoundError(f'{fullname} is outside the captured engineering source snapshot')


_SNAPSHOT_FINDER = _SnapshotFinder()


def load_project(source):
    """Execute an immutable snapshot of explicitly requested trusted local code.

    The package namespace includes its source/data digest. Relative imports use
    captured source bytes even when stale bytecode files exist on disk. Call
    assert_project_source_identity again before accepting the run's outputs.
    """
    path = project_directory(source)
    snapshot = _project_snapshot(path)
    descriptor = json.loads(snapshot['project.json'].decode('utf-8-sig'))
    if descriptor.get('schema') != 'cybrgeo-engineering-project/1':
        raise ValueError('Expected cybrgeo-engineering-project/1')
    identity = {name: hashlib.sha256(payload).hexdigest() for name, payload in snapshot.items()}
    namespace = json.dumps([str(path), identity], sort_keys=True, separators=(',', ':')).encode()
    package_name = 'cybrgeo_project_' + hashlib.sha256(namespace).hexdigest()
    with _SNAPSHOT_LOCK:
        _SNAPSHOT_FINDER.packages[package_name] = (path, {name: data for name, data in snapshot.items() if name.endswith('.py')})
        if _SNAPSHOT_FINDER not in sys.meta_path:
            sys.meta_path.insert(0, _SNAPSHOT_FINDER)
        if package_name not in sys.modules:
            package = types.ModuleType(package_name)
            package.__path__ = []
            package.__package__ = package_name
            package.__spec__ = importlib.machinery.ModuleSpec(package_name, loader=None, is_package=True)
            sys.modules[package_name] = package
        module = importlib.import_module(package_name + '.project')
        project = module.create()
    if not isinstance(project, EngineeringProject):
        raise TypeError('Project factory must return EngineeringProject')
    assert_project_source_identity(path, identity)
    return project, identity


def verify_native_build(directory, specification, *, recipe_sha256=None):
    """Check the actual native outputs before using their inertias or replay."""
    from .physical_model import digest
    root = Path(directory).resolve()
    receipt = json.loads((root / 'native-build.json').read_text(encoding='utf-8'))
    if receipt.get('schema') != 'cybrgeo-native-build/1':
        raise ValueError('Expected a completed native build')
    if receipt['provenance']['input_spec_sha256'] != digest(specification):
        raise ValueError('Native build belongs to a different design specification')
    if recipe_sha256 and receipt['provenance']['adapted_recipe_sha256'] != recipe_sha256:
        raise ValueError('Native build belongs to a different authored geometry recipe')
    core_sha256 = hashlib.sha256(Path(__file__).with_name('native_construction.py').read_bytes()).hexdigest()
    if receipt['provenance'].get('native_construction_sha256') != core_sha256:
        raise ValueError('Native build belongs to a different construction implementation')
    for relative, expected in receipt['files_sha256'].items():
        path = (root / relative).resolve()
        if not path.is_relative_to(root) or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError(f'Native artifact differs from its completed build: {relative}')
    return receipt
