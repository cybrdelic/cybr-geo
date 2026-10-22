"""Accepted project identities must describe the exact Python bytes executed."""
import hashlib
import json
import os
import py_compile

import pytest

from mechanism_lab.engineering_source import (
    assert_project_source_identity, load_project, project_source_identity,
)


def fixture_project(root):
    (root/'data/nested').mkdir(parents=True)
    (root/'helpers').mkdir()
    (root/'project.json').write_text(json.dumps({'schema':'cybrgeo-engineering-project/1'}))
    (root/'data/nested/parameters.json').write_text('{"mass":1}')
    (root/'helpers/__init__.py').write_text('')
    (root/'helpers/value.py').write_text('VALUE = "old"\n')
    (root/'design.py').write_text('def construct():\n from .helpers.value import VALUE\n return VALUE\n')
    (root/'project.py').write_text(
        'import json\nfrom pathlib import Path\n'
        'from mechanism_lab.engineering_source import EngineeringProject\n'
        'from .helpers.value import VALUE\nfrom .design import construct\n'
        'def create():\n'
        ' data=json.loads((Path(__file__).parent/"data/nested/parameters.json").read_text())\n'
        ' return EngineeringProject("test", {"value":VALUE, **data}, construct, None, None, None, None)\n')
    return root


def test_rapid_same_size_same_mtime_edit_executes_new_snapshot_not_old_pyc(tmp_path):
    root = fixture_project(tmp_path)
    value = root/'helpers/value.py'
    old_stat = value.stat()
    py_compile.compile(str(value), doraise=True)
    first, old_identity = load_project(root)
    value.write_text('VALUE = "new"\n')
    os.utime(value, ns=(old_stat.st_atime_ns, old_stat.st_mtime_ns))
    assert value.stat().st_size == old_stat.st_size
    second, new_identity = load_project(root)
    assert first.specification['value'] == 'old'
    assert second.specification['value'] == second.construct() == 'new'
    assert old_identity['helpers/value.py'] != new_identity['helpers/value.py']
    assert new_identity['helpers/value.py'] == hashlib.sha256(value.read_bytes()).hexdigest()
    assert first.construct.__module__ != second.construct.__module__


def test_post_load_data_and_code_changes_are_rejected(tmp_path):
    root = fixture_project(tmp_path)
    project, identity = load_project(root)
    assert assert_project_source_identity(root, identity) == identity
    (root/'data/nested/parameters.json').write_text('{"mass":2}')
    with pytest.raises(ValueError, match='data/nested/parameters.json'):
        assert_project_source_identity(root, identity)
    assert project.specification['mass'] == 1
    _, updated = load_project(root)
    (root/'helpers/value.py').write_text('VALUE = "new"\n')
    with pytest.raises(ValueError, match='helpers/value.py'):
        assert_project_source_identity(root, updated)


def test_lazy_relative_import_uses_captured_source_bytes(tmp_path):
    root = fixture_project(tmp_path)
    # This module is not imported by project.py; its first import happens after
    # the source edit and still must execute the original captured bytes.
    (root/'late.py').write_text('VALUE = "old"\n')
    (root/'design.py').write_text('def construct():\n from .late import VALUE\n return VALUE\n')
    project, identity = load_project(root)
    (root/'late.py').write_text('VALUE = "new"\n')
    assert project.construct() == 'old'
    with pytest.raises(ValueError, match='late.py'):
        assert_project_source_identity(root, identity)


def test_source_identity_is_recursive_but_excludes_generated_outputs(tmp_path):
    root = fixture_project(tmp_path/'project')
    before = project_source_identity(root)
    for directory in ('outputs', '__pycache__', 'evidence'):
        (root/directory).mkdir()
        (root/directory/'generated.py').write_text('ignored = True')
    (tmp_path/'workspace_elsewhere.py').write_text('unrelated = True')
    assert project_source_identity(root) == before
    (root/'data/nested/additional.dat').write_bytes(b'authored input')
    assert 'data/nested/additional.dat' in project_source_identity(root)
    (root/'data/outputs').mkdir()
    (root/'data/outputs/fixture.json').write_text('{}')
    assert 'data/outputs/fixture.json' in project_source_identity(root)
