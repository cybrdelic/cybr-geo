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
import math
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


def current_body_properties(design, source, overrides, directory, inherited_masses):
    """Aggregate the selected native solids in their actual body-local frames.

    Two views retain the distinction between geometry-density properties and
    source-declared envelope masses. An override scales the native shape's
    uniform-density tensor through the shared mass integrator; it is not a
    recovered supplier tensor. Missing physical materials stay unresolved,
    rather than inheriting a display finish or an archived body residual.
    """
    from mechanism_lab.native_construction import aggregate_mass_properties, shape_mass_properties

    identity = require_current_construction()
    parts = {str(row['id']): row for row in design['parts']}
    resolver = PartGeometryResolver(geometry_dag(design, source=source), parts,
        overrides=overrides, asset_root=directory, source=source)
    groups = {}
    rows = []
    for key, record in parts.items():
        body = record['body']
        if not isinstance(body, str) or not body:
            raise ValueError('Native properties require an explicit body identity')
        group = groups.setdefault(body, dict(part_ids=[], missing_material_part_ids=[],
            disconnected_part_ids=[], density=[], declared=[]))
        group['part_ids'].append(record['id'])
        shape = resolver.resolve(key)
        solid_count = len(shape.Solids())
        if not shape.isValid() or not solid_count or not math.isfinite(shape.Volume()) or shape.Volume() <= 0:
            raise ValueError('Current body properties require valid positive-volume native parts: ' + key)
        if solid_count != 1:
            group['disconnected_part_ids'].append(record['id'])
        override = record.get('mass_override_kg')
        if override is not None and (isinstance(override, bool)
                or not isinstance(override, (int, float))
                or not math.isfinite(override) or override <= 0):
            raise ValueError('Source mass override must be null or finite positive kilograms: ' + key)
        row = dict(id=record['id'], name=record['name'], body=body,
            material=record['material'], source_status=record.get('status'),
            source_mass_override_kg=override, geometry_source=resolver.describe(key),
            frame='source body-local', valid=True, solid_count=solid_count)
        if record['material'] not in DENSITIES:
            group['missing_material_part_ids'].append(record['id'])
            row.update(resolved=False, reason='No explicit source physical density for material ' + record['material'],
                       density_properties=None, declared_mass_properties=None)
            rows.append(row)
            continue
        density = DENSITIES[record['material']]
        density_properties = native_properties(shape, density_kg_m3=density)
        mass = density_properties['mass_kg']
        center = np.asarray(density_properties['com_mm']) * .001
        tensor = np.asarray(density_properties['inertia_kg_m2'])
        component = dict(name=record['name'], part_id=record['id'], material=record['material'],
            mass_kg=mass, com_m=center.tolist(), inertia_at_com_kg_m2=tensor.tolist(),
            provenance='Current selected native geometry at explicit source homogeneous density')
        group['density'].append(component)
        declared = dict(component)
        if override is not None:
            mass, center, tensor = shape_mass_properties(shape, density, override)
            declared.update(mass_kg=mass, com_m=center.tolist(), inertia_at_com_kg_m2=tensor.tolist(),
                provenance='Source-declared mass; uniform selected-native-envelope tensor scaling; not supplier inertia')
        group['declared'].append(declared)
        row.update(resolved=True, density_properties=density_properties,
            declared_mass_properties=dict(mass_kg=mass, com_m=center.tolist(),
                inertia_at_com_kg_m2=tensor.tolist(), override_applied=override is not None,
                policy='explicit source mass with uniform-envelope inertia' if override is not None else 'source homogeneous density'))
        rows.append(row)

    bodies = {}
    for body, group in groups.items():
        complete = not group['missing_material_part_ids']
        density = aggregate_mass_properties(group['density'], 'Current source-density native components') if group['density'] else None
        declared = aggregate_mass_properties(group['declared'], 'Current native components with explicit source envelope-mass overrides') if group['declared'] else None
        archive = inherited_masses.get(body)
        comparison = dict(archive_present=archive is not None, complete_current_mass_inputs=complete,
            comparable=bool(archive is not None and complete and declared is not None))
        if comparison['comparable']:
            comparison.update(mass_delta_kg=declared['mass_kg'] - archive['mass_kg'],
                com_delta_m=(np.asarray(declared['com_m']) - np.asarray(archive['com_m'])).tolist(),
                inertia_tensor_delta_kg_m2=(np.asarray(declared['inertia_tensor_kg_m2'])
                    - np.asarray(archive['inertia_tensor_kg_m2'])).tolist(),
                scope='Numerical difference only; archive inventory/model assumptions may differ')
        bodies[body] = dict(part_ids=group['part_ids'], missing_material_part_ids=group['missing_material_part_ids'],
            disconnected_part_ids=group['disconnected_part_ids'], mass_inputs_complete=complete,
            records_are_partial_subtotals=not complete, source_density=density,
            declared_envelope_masses=declared, archival_comparison=comparison)
    require_current_construction(identity)
    return dict(schema='cybrgeo-current-native-body-properties/1', source=source,
        construction_code_sha256=identity, geometry_resolution=resolver.report(),
        frame='Original source body-local millimetres integrated to kg, m and kg m2; no capture/world transform applied',
        aggregation='Shared native_construction.shape_mass_properties and aggregate_mass_properties; parallel-axis sum about body CoM',
        physical_density_kg_m3=dict(DENSITIES),
        mass_override_policy='Retain declared source mass; scale current native uniform-envelope inertia. No inferred supplier internals.',
        part_count=len(rows), resolved_parts=sum(row['resolved'] for row in rows),
        unresolved_part_ids=[row['id'] for row in rows if not row['resolved']],
        mass_inputs_complete=all(row['resolved'] for row in rows),
        source_mass_override_count=sum(row['source_mass_override_kg'] is not None for row in rows),
        parts=rows, bodies=bodies, inherited_mass_properties_sha256=digest(inherited_masses),
        inherited_mass_properties_modified=False, dynamics_rerun=False,
        limitations=['Missing material inputs make affected body records partial subtotals, unusable as complete solver bodies.',
            'Native properties are current computed values; archived body records and captured dynamics remain separate and unchanged.',
            'Declared rigid-body allocation and purchased-envelope mass distributions have not been independently qualified.',
            'Disconnected source solids remain explicit; no fit, continuous clearance, strength, thermal or hardware qualification follows.'])


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


def corrected_archival_bodies(design, reconciliation, source_corrected):
    """Apply actual Refined native deltas without discarding archive assumptions.

    This is the source verify_geometry.py correction through GEO's shared
    component replacement service, using freshly measured native part values.
    The separate all-part inventory reports missing inputs for a full rebuild.
    """
    from mechanism_lab.native_construction import replace_mass_components
    identity = require_current_construction()
    if not reconciliation['passed']:
        raise ValueError('Body correction requires passing measured native part reconciliation')
    old = reconciliation['inherited_mass_properties']
    if set(source_corrected) != set(old):
        raise ValueError('Corrected source body inventory differs from the original archive')
    parts = {row['name']: row for row in design['parts']}
    if len(parts) != len(design['parts']):
        raise ValueError('Source body correction requires unique part names')
    consumed = set()
    bodies, comparisons = {}, {}
    for body, base in old.items():
        replacements = []
        for row in reconciliation['parts']:
            name = row['name']
            if name not in base['components']:
                continue
            if (name in consumed or parts[name]['body'] != body
                    or parts[name].get('mass_override_kg') is not None):
                raise ValueError('Corrected native part has ambiguous body or purchased-mass binding: ' + name)
            consumed.add(name)
            values = {}
            for field, measured in (('before', 'original_measured'), ('after', 'refined_measured')):
                value = row[measured]
                values[field] = dict(name=name, mass_kg=value['mass_kg'],
                    com_m=(np.asarray(value['com_mm'])*.001).tolist(),
                    inertia_at_com_kg_m2=value['inertia_kg_m2'],
                    provenance='Measured source-native ' + measured + '; part record ' + str(row['id']))
            replacements.append(values)
        actual = replace_mass_components(base, replacements,
            status='Actual Refined native component deltas applied to retained archived body assumptions')
        expected = source_corrected[body]
        if actual['components'] != expected['components']:
            raise ValueError('Corrected source component inventory differs for body ' + body)
        residuals = {}
        for field, atol in (('mass_kg', TOLERANCES['mass_kg']), ('com_m', TOLERANCES['com_mm']*.001),
                            ('inertia_tensor_kg_m2', TOLERANCES['inertia_kg_m2'])):
            a, b = np.asarray(actual[field], float), np.asarray(expected[field], float)
            if a.shape != b.shape or not np.isfinite(b).all():
                raise ValueError('Invalid corrected source property: ' + field)
            residuals[field] = dict(maximum_absolute_error=float(np.max(np.abs(a-b))),
                absolute_tolerance=atol, relative_tolerance=TOLERANCES['relative'],
                passed=bool(np.allclose(a, b, atol=atol, rtol=TOLERANCES['relative'])))
        bodies[body] = actual
        comparisons[body] = dict(replaced_components=[row['before']['name'] for row in replacements],
            residuals=residuals, passed=all(row['passed'] for row in residuals.values()))
    if consumed != {row['name'] for row in reconciliation['parts']}:
        raise ValueError('Every changed native part must occur in exactly one archived body')
    require_current_construction(identity)
    source = reconciliation['source']
    return dict(schema='cybrgeo-native-corrected-archival-body-properties/1',
        source=source, construction_code_sha256=identity,
        algorithm_source=source['operation_sources']['revisions/p5-refined/source/verify_geometry.py'],
        original_mass_properties_sha256=digest(old), source_corrected_properties_sha256=digest(source_corrected),
        native_reconciliation_sha256=digest(reconciliation),
        passed=all(row['passed'] for row in comparisons.values()),
        replaced_parts=len(consumed), bodies=bodies, source_comparisons=comparisons,
        original_archived_properties_modified=False, retained_unmodified_component_assumptions=True,
        all_parts_reintegrated=False, dynamics_rerun=False,
        scope='Current native part spatial-inertia corrections to the actual source body model, verified against published corrected records',
        limitations=['Unreplaced component mass/distribution assumptions remain inherited; this is not the all-part native-density rebuild.',
            'Corrected body properties do not revalidate archived capture dynamics, control loads, contacts or hardware.'])


def build(output, *, source_directory=DATA, frame_index=300):
    identity = require_current_construction()
    manifest, payloads = load_bundle(source_directory)
    design, pose, source, overrides, changes, inherited = source_inputs(manifest, payloads, frame_index=frame_index)
    assembly, report = build_records(output, design, pose, source=source,
        body_aliases=PINNED_BODY_ALIASES, brep_overrides=overrides, asset_root=source_directory)
    reconciliation = reconcile(design, source, overrides, changes, inherited, source_directory)
    current_properties = current_body_properties(design, source, overrides, source_directory, inherited)
    corrected = corrected_archival_bodies(design, reconciliation, json.loads(payloads['refined-mass-properties.json']))
    output = Path(output)
    reconciliation['geometry_artifacts'] = report['artifacts']
    reconciliation['geometry_study_sha256'] = hashlib.sha256((output / 'geometry-study.json').read_bytes()).hexdigest()
    require_current_construction(identity)
    (output / 'current-native-body-properties.json').write_text(json.dumps(current_properties, indent=2) + '\n', encoding='utf-8')
    reconciliation['current_native_body_properties'] = dict(
        file='current-native-body-properties.json',
        sha256=hashlib.sha256((output / 'current-native-body-properties.json').read_bytes()).hexdigest(),
        resolved_parts=current_properties['resolved_parts'],
        unresolved_part_ids=current_properties['unresolved_part_ids'],
        mass_inputs_complete=current_properties['mass_inputs_complete'])
    (output / 'native-corrected-archival-body-properties.json').write_text(json.dumps(corrected, indent=2) + '\n', encoding='utf-8')
    reconciliation['native_corrected_archival_body_properties'] = dict(
        file='native-corrected-archival-body-properties.json',
        sha256=hashlib.sha256((output / 'native-corrected-archival-body-properties.json').read_bytes()).hexdigest(),
        passed=corrected['passed'], replaced_parts=corrected['replaced_parts'],
        source_corrected_file_sha256=manifest['files']['refined-mass-properties.json']['sha256'])
    (output / 'property-reconciliation.json').write_text(json.dumps(reconciliation, indent=2) + '\n', encoding='utf-8')
    if not reconciliation['passed'] or not corrected['passed']:
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
