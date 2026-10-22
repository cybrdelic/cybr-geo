"""Actual P5 Refined BREP selection through GEO's shared part resolver.

Source: cybrdelic/cybr-arx6-compact at 85bd6d2ce5b7c567d08b3ffccf2a9be46005aabf,
revisions/p5-refined/source/{cad_core,refine_cad,export_refined_native,frames}.py.
The original exporter executes output-writing code at import. This client reads
only its verified data/BREP and reuses GEO services; it never imports that code.
Archived body placement is a static comparison, not rerun dynamics acceptance.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path, PurePosixPath
import time

import numpy as np

from mechanism_lab.part_geometry import PartGeometryResolver, native_properties
from mechanism_lab.physical_model import digest
if __package__:
    from . import design as _design
    from .design import PINNED_BODY_ALIASES, build_records, geometry_dag
else:  # Installed share-data scripts are also usable via runpy or a direct path.
    import importlib.util
    _spec = importlib.util.spec_from_file_location('_cybrgeo_compact_geometry', Path(__file__).with_name('design.py'))
    _design = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(_design)
    PINNED_BODY_ALIASES = _design.PINNED_BODY_ALIASES
    build_records, geometry_dag = _design.build_records, _design.geometry_dag


COMMIT = '85bd6d2ce5b7c567d08b3ffccf2a9be46005aabf'
REPOSITORY = 'https://github.com/cybrdelic/cybr-arx6-compact'
DATA = Path(__file__).resolve().parent / 'data' / 'refined'
# Exact source refine_cad.py declarations, converted kg/mm3 -> kg/m3.
# Changed source records use only aluminum/steel. Unknown materials fail;
# the source generator's silent aluminum fallback is intentionally not used.
DENSITIES = {'aluminum': 2700., 'steel': 7850., 'brass': 8500., 'rubber': 1100., 'polymer': 1250.}
TOLERANCES = {'relative': 2e-7, 'volume_mm3': 1e-5, 'mass_kg': 1e-10,
              'com_mm': 1e-5, 'inertia_kg_m2': 1e-12, 'bounds_mm': 1e-5}
CONSTRUCTION_CODE_PATHS = {'examples/engineering/arx_compact/refined.py': Path(__file__).resolve()}


def construction_code_hashes():
    return {**_design.construction_code_hashes(),
            **{name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in CONSTRUCTION_CODE_PATHS.items()}}


_LOADED_CODE_HASHES = construction_code_hashes()


def require_current_construction(expected=None):
    _design.require_current_construction()
    current = construction_code_hashes()
    if current != _LOADED_CODE_HASHES or expected is not None and current != expected:
        raise ValueError('Construction code changed; reload the Refined client and rebuild before accepting output')
    return dict(current)


def load_bundle(directory=DATA):
    """Read byte-pinned inputs; source identity is evidence, not a signature."""
    directory = Path(directory).resolve()
    manifest = json.loads((directory / 'source.json').read_text(encoding='utf-8'))
    if (manifest.get('schema') != 'cybrgeo-source-bundle/1'
            or manifest.get('repository') != REPOSITORY or manifest.get('commit') != COMMIT):
        raise ValueError('Unexpected Refined source bundle identity')
    payloads = {}
    for relative, row in manifest['files'].items():
        path = PurePosixPath(relative)
        if (path.is_absolute() or '\\' in relative or ':' in relative
                or any(p in ('', '.', '..') for p in relative.split('/'))):
            raise ValueError('Source bundle filenames must be confined relative paths')
        target = directory.joinpath(*path.parts)
        if not target.resolve().is_relative_to(directory) or target.is_symlink():
            raise ValueError('Source bundle file escaped its directory')
        raw = target.read_bytes()
        if (hashlib.sha256(raw).hexdigest() != row['sha256']
                or hashlib.sha1(b'blob ' + str(len(raw)).encode() + b'\0' + raw).hexdigest() != row['git_blob_sha']):
            raise ValueError('Source bundle byte identity mismatch: ' + relative)
        payloads[relative] = raw
    return manifest, payloads


def source_inputs(manifest, payloads, *, frame_index=300):
    """Select an actual saved frame, without source interpolation or animation."""
    if (manifest.get('repository') != REPOSITORY or manifest.get('commit') != COMMIT
            or set(payloads) != set(manifest['files'])
            or any(hashlib.sha256(raw).hexdigest() != manifest['files'][name]['sha256']
                   for name, raw in payloads.items())):
        raise ValueError('Refined input byte identity differs from its source bundle')
    design = json.loads(payloads['design.json'])
    capture = json.loads(payloads['capture.json'])
    refinement = json.loads(payloads['cad-refinement.json'])
    tessellation = json.loads(payloads['tessellation.json'])
    inherited_masses = json.loads(payloads['mass-properties.json'])
    if (isinstance(frame_index, bool) or not isinstance(frame_index, int)
            or not 0 <= frame_index < len(capture['frames'])
            or len(capture['frames']) != len(capture['bodies'])):
        raise ValueError('Choose an existing source capture frame index')
    pose = dict(body_names=capture['bodyNames'], body_poses=capture['bodies'][frame_index],
                time_s=capture['frames'][frame_index][0], source_frame_index=frame_index,
                capture_sha256=manifest['files']['capture.json']['sha256'], commit=COMMIT)
    if refinement['original_source_sha256'] != manifest['files']['design.json']['sha256']:
        raise ValueError('Refinement report names a different original design')
    records = {str(row['id']): row for row in design['parts']}
    changes = {str(row['id']): row for row in refinement['parts'] if row['operations']}
    exports = {str(row['id']): row for row in tessellation['parts']}
    if (len(records) != len(design['parts']) or len(changes) != len(refinement['parts'])
            or len(exports) != len(tessellation['parts'])
            or set(changes) != set(exports) or not set(exports) <= records.keys()
            or tessellation['part_instances'] != len(records)
            or tessellation['changed_instances'] != len(exports)
            or refinement['modified_parts'] != len(changes)):
        raise ValueError('Refined selection, exported BREP set and source part records disagree')
    actual_breps = {name for name in payloads if name.startswith('cad/') and name.endswith('.brep')}
    declared_breps = {f'cad/{int(key):03d}.brep' for key in exports}
    if actual_breps != declared_breps:
        raise ValueError('Source bundle must retain exactly the exported BREP set')
    overrides = []
    for key, export in exports.items():
        record, change = records[key], changes[key]
        relative = f'cad/{int(key):03d}.brep'
        if (change['name'] != record['name'] or change['material'] != record['material']
                or not change.get('valid') or change.get('solid_count') != 1
                or export['brep_sha256'] != manifest['files'][relative]['sha256']):
            raise ValueError('Refined BREP report binding mismatch: ' + key)
        overrides.append(dict(part=key, record_sha256=digest(record), path=relative,
            sha256=export['brep_sha256'], units='mm', frame='body-local',
            provenance=dict(upstream_path=manifest['files'][relative]['upstream_path'],
                refinement_record_sha256=digest(change), tessellation_record_sha256=digest(export))))
    source = dict(repository=REPOSITORY, commit=COMMIT,
                  source_files=manifest['files'], source_bundle_record_sha256=digest(manifest),
                  operation_sources=manifest.get('operation_sources', {}),
                  scope='Refined geometry at archived compact capture placement; no new dynamics')
    return design, pose, source, overrides, changes, inherited_masses


def property_residuals(actual, archived):
    result = {}
    for key in ('volume_mm3', 'mass_kg', 'com_mm', 'inertia_kg_m2', 'bounds_mm'):
        measured, expected = np.asarray(actual[key], float), np.asarray(archived[key], float)
        if measured.shape != expected.shape or not np.isfinite(expected).all():
            raise ValueError('Invalid archived property: ' + key)
        result[key] = dict(maximum_absolute_error=float(np.max(np.abs(measured - expected))),
            absolute_tolerance=TOLERANCES[key], relative_tolerance=TOLERANCES['relative'],
            passed=bool(np.allclose(measured, expected, rtol=TOLERANCES['relative'], atol=TOLERANCES[key])))
    return result


def original_properties(resolver, key, *, density_kg_m3):
    """Match refine_cad.refine(): original=solid(build(p['shape'])).clean().

    This source normalization affects kernel integration of split faces. It is
    applied to the original comparison shape only, not to replacement BREPs or
    to the unchanged parts used for display/export. Tolerances stay unchanged.
    """
    solids = resolver.resolve(key).Solids()
    if len(solids) != 1:
        raise ValueError('Source refinement requires one original solid: ' + key)
    return native_properties(solids[0].clean(), density_kg_m3=density_kg_m3)


def reconcile(design, source, overrides, changes, inherited_masses, directory):
    identity = require_current_construction()
    """Recompute changed geometry; never overwrite inherited assembly masses."""
    from mechanism_lab import native_construction
    dag = geometry_dag(design, source=source)
    parts = {str(row['id']): row for row in design['parts']}
    original = PartGeometryResolver(dag, parts, source=source)
    refined = PartGeometryResolver(dag, parts, overrides=overrides, asset_root=directory, source=source)
    rows, body_deltas = [], {}
    for key, change in changes.items():
        record = parts[key]
        if record['material'] not in DENSITIES:
            raise ValueError('No explicit source density: ' + record['material'])
        density = DENSITIES[record['material']]
        before = original_properties(original, key, density_kg_m3=density)
        after = refined.mass_properties(key, density_kg_m3=density)
        before_residual = property_residuals(before, change['original'])
        after_residual = property_residuals(after, change['refined'])
        delta = after['mass_kg'] - before['mass_kg']
        body_deltas[record['body']] = body_deltas.get(record['body'], 0.) + delta
        rows.append(dict(id=record['id'], body=record['body'], name=record['name'],
            material=record['material'], density_kg_m3=density,
            source_mass_override_kg=record.get('mass_override_kg'),
            original_measured=before, refined_measured=after,
            archived_refinement=change, original_residuals=before_residual,
            refined_residuals=after_residual, geometry_density_mass_delta_kg=delta,
            passed=(before['valid'] and after['valid'] and before['solid_count'] == after['solid_count'] == 1
                    and all(r['passed'] for r in (*before_residual.values(), *after_residual.values())))))
    require_current_construction(identity)
    return dict(schema='cybrgeo-native-property-reconciliation/1', source=source,
        recipe_sha256=identity['examples/engineering/arx_compact/refined.py'],
        mass_integration_sha256=identity['src/mechanism_lab/native_construction.py'],
        construction_code_sha256=dict(identity),
        geometry_resolution=refined.report(),
        passed=all(row['passed'] for row in rows), changed_parts=len(rows), parts=rows,
        geometry_density_mass_delta_kg=sum(body_deltas.values()),
        geometry_density_mass_delta_by_body_kg=body_deltas,
        inherited_mass_properties=inherited_masses, inherited_mass_properties_modified=False,
        source_mass_overrides_used=False, property_tolerances=TOLERANCES,
        original_comparison_normalization="refine_cad.refine: solid(build(part['shape'])).clean()",
        scope='Kernel property reconciliation for selected native geometry; homogeneous source densities only',
        limitations=['Inherited assembly mass/inertia and captured dynamics were not updated or requalified.',
            'Body mass deltas are geometric comparisons, not replacements for inherited mass overrides or tensors.',
            'Component/reducer envelopes remain envelopes, not recovered supplier internal geometry.',
            'No new fit, clearance, load, thermal, fatigue, manufacturing or hardware qualification.'])


def build(output, *, source_directory=DATA, frame_index=300):
    identity = require_current_construction()
    manifest, payloads = load_bundle(source_directory)
    design, pose, source, overrides, changes, inherited = source_inputs(manifest, payloads, frame_index=frame_index)
    assembly, report = build_records(output, design, pose, source=source,
        body_aliases=PINNED_BODY_ALIASES, brep_overrides=overrides, asset_root=source_directory)
    reconciliation = reconcile(design, source, overrides, changes, inherited, source_directory)
    output = Path(output)
    reconciliation['geometry_artifacts'] = report['artifacts']
    reconciliation['geometry_study_sha256'] = hashlib.sha256((output / 'geometry-study.json').read_bytes()).hexdigest()
    require_current_construction(identity)
    (output / 'property-reconciliation.json').write_text(json.dumps(reconciliation, indent=2) + '\n', encoding='utf-8')
    if not reconciliation['passed']:
        raise ValueError('Refined native property reconciliation failed; inspect property-reconciliation.json')
    require_current_construction(identity)
    return assembly, report, reconciliation


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('outputs/arx-compact-refined'))
    parser.add_argument('--source-directory', type=Path, default=DATA)
    parser.add_argument('--frame-index', type=int, default=300)
    parser.add_argument('--render', action='store_true')
    args = parser.parse_args()
    identity = require_current_construction()
    started = time.perf_counter()
    assembly, report, reconciliation = build(args.output, source_directory=args.source_directory, frame_index=args.frame_index)
    built = time.perf_counter()
    media = {}
    if args.render:
        from mechanism_lab.render import render_still
        render_still(assembly, args.output / 'reference.png', size=(1280, 960), intent='concept')
        media['reference.png'] = hashlib.sha256((args.output / 'reference.png').read_bytes()).hexdigest()
    require_current_construction(identity)
    result = dict(schema='cybrgeo-geometry-client-run/1', output=str(args.output.resolve()), parts=report['part_count'],
        construction_code_sha256=dict(identity),
        changed_parts=reconciliation['changed_parts'], captured_time_s=report['time_s'],
        geometry_density_mass_delta_kg=reconciliation['geometry_density_mass_delta_kg'],
        build_and_reconciliation_seconds=built-started, render_seconds=time.perf_counter()-built if args.render else 0.,
        media_sha256=media, property_reconciliation_sha256=hashlib.sha256((args.output / 'property-reconciliation.json').read_bytes()).hexdigest(),
        passed=reconciliation['passed'], dynamics_rerun=False, hardware_qualified=False)
    (args.output / 'client-run.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
