"""Portable release packaging, content inventory and safe source selection."""
from __future__ import annotations
from pathlib import Path
import hashlib
import json
import zipfile

SKIP_DIRS = {'.git', '.build', '.venv', '__pycache__', '.pytest_cache', '.ruff_cache', 'dist'}
SKIP_SUFFIXES = {'.pyc', '.pyo', '.ttf', '.otf', '.woff', '.woff2'}


def release_files(root):
    root = Path(root).resolve()
    for path in sorted(root.rglob('*')):
        relative = path.relative_to(root)
        if any(part in SKIP_DIRS or part.endswith('.egg-info') for part in relative.parts):
            continue
        if not path.is_file() or path.is_symlink():
            continue
        if path.suffix.lower() in SKIP_SUFFIXES or '.partial.' in path.name:
            continue
        yield path


def digest(path):
    hasher = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            hasher.update(block)
    return hasher.hexdigest()


def inventory(root):
    root = Path(root).resolve()
    rows = []
    for path in release_files(root):
        rel = path.relative_to(root).as_posix()
        if rel == 'validation/release_manifest.json':
            continue
        rows.append({'path': rel, 'bytes': path.stat().st_size, 'sha256': digest(path)})
    return {'schema': 1, 'files': rows, 'file_count': len(rows),
            'total_bytes': sum(row['bytes'] for row in rows),
            'manifest_self_excluded': True}


def pack(root, output, source_only=False):
    root = Path(root).resolve()
    output = Path(output).expanduser().resolve()
    # Prohibit self-inclusion; dist is explicitly excluded from source selection.
    if output.is_relative_to(root) and 'dist' not in output.relative_to(root).parts:
        raise ValueError('Place release archives outside the project, or inside dist/.')
    output.parent.mkdir(parents=True, exist_ok=True)
    report = inventory(root)
    (root/'validation').mkdir(exist_ok=True)
    (root/'validation/release_manifest.json').write_text(json.dumps(report, indent=2)+'\n')
    selected = list(release_files(root))
    if source_only:
        def needed(path):
            rel = path.relative_to(root)
            if rel.parts[0] in ('media', 'outputs'):
                return False
            if rel.parts[0] == 'assets':
                return (len(rel.parts)>3 and rel.parts[1:3] == ('differential_v3','geometry')
                        and (path.name.endswith('_parts.npz') or path.name.endswith('_parts.json')
                             or path.name == 'reference_manifest.json'))
            return rel.as_posix() not in ('README.md', 'index.html', 'validation/release_manifest.json')
        selected = [path for path in selected if needed(path)]
    with zipfile.ZipFile(output, 'w', zipfile.ZIP_DEFLATED, compresslevel=4) as archive:
        for path in selected:
            rel = path.relative_to(root).as_posix()
            # Reproducible timestamps independent of extraction/machine modification time.
            info = zipfile.ZipInfo('cybr-mechanism-lab/'+rel, (2026,9,9,0,0,0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = (0o100644 << 16)
            archive.writestr(info, path.read_bytes(), compress_type=zipfile.ZIP_DEFLATED, compresslevel=4)
        if source_only:
            archive.writestr('cybr-mechanism-lab/README.md',
                '# CYBR Mechanism Lab — regeneration kit\n\n'
                'This smaller archive contains source, build tooling, documentation, tests, '
                'and exact preserved differential mesh inputs. Existing videos, images, and '
                'historical CAD exports are in the full release, not this kit.\n\n'
                'Install with `python -m pip install -e ".[dev]"`. See `docs/REPRODUCE.md`.\n'
                'Run `lab doctor`, `lab build m8325s --step`, `lab render m8325s`, '
                '`lab blueprint m8325s`, and `python tools/build_release_media.py`.\n\n'
                'New geometry recipes use the same render/export/media/whiteprint APIs. '
                'No image generator is used. This remains a visual and kinematic study, '
                'not a released motor/differential manufacturing design.\n')
    return {'archive': str(output), 'bytes': output.stat().st_size,
            'sha256': digest(output), 'source_only': source_only,
            'files': len(selected)+(1 if source_only else 0)}
