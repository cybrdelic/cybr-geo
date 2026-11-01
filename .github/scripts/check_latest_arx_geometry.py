"""Bound fresh Refined/P6B client builds and independently reopen their CAD.

This is source-selected geometry and nominal property evidence only. It does
not rerun compact dynamics, qualify a printed arm, or establish load capacity.
CI must use the default fresh-build mode. --check-existing is explicit local
curation: it never records a fresh construction or substitutes for a CI build.
"""
from __future__ import annotations

import argparse
import faulthandler
import hashlib
import importlib.metadata
from io import BytesIO
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[2]
COMMIT = '85bd6d2ce5b7c567d08b3ffccf2a9be46005aabf'
SCOPE = ('Pinned source geometry, homogeneous-density properties and nominal CAD interfaces; '
         'no dynamics rerun, printed transmission integration, whole-arm load rating or hardware qualification')
PRINT_PARTS = {'upper-frame-proximal', 'upper-frame-distal', 'upper-frame-lap-coupler',
               'fore-frame-reinforced', 'gripper-pinion-tooth-trial', 'gripper-rack-tooth-trial'}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def sha256(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def artifact(directory, relative, expected):
    # Legacy report paths use the producer's native separators. Accept either
    # spelling while preventing path escape in all three supported desktop OSes.
    relative = relative.replace('\\', '/')
    require(':' not in relative and not relative.startswith('/')
            and all(part not in ('', '.', '..') for part in relative.split('/')),
            'Artifact must use a confined relative path: ' + relative)
    path = directory.joinpath(*relative.split('/'))
    require(path.resolve().is_relative_to(directory.resolve()), 'Artifact path escaped its build')
    require(path.is_file() and sha256(path) == expected, 'Artifact byte identity mismatch: ' + relative)
    return path


def versions():
    return dict(system=platform.system(), machine=platform.machine(), python=sys.version,
                packages={name: importlib.metadata.version(name)
                          for name in ('cadquery', 'cadquery-ocp', 'numpy', 'trimesh')})


def close(actual, expected, label, *, rtol=2e-7, atol=1e-8):
    import numpy as np
    actual, expected = np.asarray(actual, float), np.asarray(expected, float)
    require(actual.shape == expected.shape and np.isfinite(actual).all()
            and np.isfinite(expected).all() and np.allclose(actual, expected, rtol=rtol, atol=atol),
            'Native readback mismatch: ' + label)


def exchange_properties(shape, density, default_properties, epsilon, error_budget):
    """Independent, uniform BREP/STEP quadrature; never changes producer masses.

    OCCT's adaptive Gauss overload estimates relative integration error.
    Compare both representations at two
    requested accuracies; an estimate alone is not an equivalence proof.
    """
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    from mechanism_lab.native_construction import validate_inertia
    props = GProp_GProps()
    # Use the same unshifted reference and three-argument adaptive overload for
    # every representation/part; no per-part integration-method selection.
    error = float(BRepGProp.VolumeProperties_s(shape.wrapped, props, epsilon))
    # Eps requests per-face integration accuracy; OCCT returns an estimate for
    # the whole shape. Report both without pretending they are identical. The
    # existing property budget bounds the estimate and every convergence test.
    require(math.isfinite(error) and 0 <= error <= error_budget,
            'Adaptive integration error estimate exceeds the existing property budget')
    volume = float(props.Mass())
    require(math.isfinite(volume) and volume > 0, 'Invalid integrated native volume')
    center, matrix = props.CentreOfMass(), props.MatrixOfInertia()
    # Native coordinates are mm: volume mm^3 -> m^3 uses 1e-9;
    # volume second moments mm^5 -> m^5 use 1e-15, then multiply kg/m^3.
    tensor = [[matrix.Value(i, j) * density * 1e-15 for j in (1, 2, 3)] for i in (1, 2, 3)]
    validate_inertia(tensor)
    measured = dict(default_properties, volume_mm3=volume, mass_kg=volume*density*1e-9,
                    com_mm=[center.X(), center.Y(), center.Z()], inertia_kg_m2=tensor)
    return dict(properties=measured, requested_relative_error=epsilon,
                estimated_relative_error=error, estimate_meets_request=error <= epsilon,
                relative_error_acceptance_budget=error_budget)


def verify_refined_body_reports(directory, report, design, source, payloads, code, records):
    """Check physical-input receipts without promoting corrected archives to dynamics."""
    import numpy as np
    from examples.engineering.arx_compact import refined
    from mechanism_lab.native_construction import aggregate_mass_properties, validate_inertia
    from mechanism_lab.physical_model import digest

    current_link = report['current_native_body_properties']
    corrected_link = report['native_corrected_archival_body_properties']
    current = read_json(artifact(directory, current_link['file'], current_link['sha256']))
    corrected = read_json(artifact(directory, corrected_link['file'], corrected_link['sha256']))
    require(current['schema'] == 'cybrgeo-current-native-body-properties/1'
            and corrected['schema'] == 'cybrgeo-native-corrected-archival-body-properties/1',
            'Unexpected Refined body-property schemas')
    require(current['source'] == corrected['source'] == source
            and current['construction_code_sha256'] == corrected['construction_code_sha256'] == code,
            'Refined body properties have stale source or construction bindings')
    require(current['geometry_resolution'] == report['geometry_resolution']
            and current['physical_density_kg_m3'] == refined.DENSITIES,
            'Refined body properties have different geometry or physical densities')
    original = report['inherited_mass_properties']
    require(current['inherited_mass_properties_sha256'] == digest(original)
            and current['inherited_mass_properties_modified'] is False
            and current['dynamics_rerun'] is False and corrected['dynamics_rerun'] is False,
            'Refined body properties overwrite or promote archived evidence')
    source_parts = {str(row['id']): row for row in design['parts']}
    parts = {str(row['id']): row for row in current['parts']}
    require(len(parts) == len(current['parts']) == current['part_count'] == len(source_parts)
            and set(parts) == set(source_parts) == set(records), 'Incomplete current body inventory')
    missing = [row['id'] for row in design['parts'] if row['material'] not in refined.DENSITIES]
    require(current['unresolved_part_ids'] == current_link['unresolved_part_ids'] == missing
            and current['resolved_parts'] == current_link['resolved_parts'] == len(parts) - len(missing)
            and current['mass_inputs_complete'] is (not missing)
            and current_link['mass_inputs_complete'] is (not missing)
            and current['source_mass_override_count'] == sum(row.get('mass_override_kg') is not None
                for row in design['parts']), 'Missing densities or mass overrides were obscured')
    selected = {row['part']: row for row in current['geometry_resolution']['parts']}
    for key, part in parts.items():
        record = source_parts[key]
        require(all(part[field] == record[field] for field in ('id', 'name', 'body', 'material'))
                and part['source_status'] == record.get('status')
                and part['source_mass_override_kg'] == record.get('mass_override_kg')
                and part['geometry_source'] == selected[key]
                and part['frame'] == 'source body-local'
                and part['valid'] is True and part['solid_count'] == records[key]['solids'],
                'Current native component binding differs: ' + key)
        resolved = record['material'] in refined.DENSITIES
        require(part['resolved'] is resolved, 'Missing physical material was silently resolved: ' + key)
        if not resolved:
            require(part['density_properties'] is None and part['declared_mass_properties'] is None,
                    'Unresolved physical material acquired mass properties: ' + key)
            continue
        density, declared = part['density_properties'], part['declared_mass_properties']
        require(density['density_kg_m3'] == refined.DENSITIES[record['material']]
                and density['valid'] is True and density['solid_count'] == part['solid_count'],
                'Current density/native topology differs: ' + key)
        close(density['volume_mm3'], records[key]['volume_mm3'], 'Current body volume ' + key)
        close(density['mass_kg'], density['volume_mm3'] * density['density_kg_m3'] * 1e-9,
              'Current density mass ' + key, atol=1e-12)
        override = record.get('mass_override_kg')
        mass = override if override is not None else density['mass_kg']
        require(declared['override_applied'] is (override is not None), 'Mass override policy differs: ' + key)
        close(declared['mass_kg'], mass, 'Declared envelope mass ' + key, atol=1e-12)
        close(declared['com_m'], np.asarray(density['com_mm']) * .001, 'Body-local component CoM ' + key)
        close(declared['inertia_at_com_kg_m2'], np.asarray(density['inertia_kg_m2']) * mass / density['mass_kg'],
              'Uniform envelope inertia scaling ' + key, atol=1e-12)
        validate_inertia(density['inertia_kg_m2'])
        validate_inertia(declared['inertia_at_com_kg_m2'])
    require(set(current['bodies']) == {row['body'] for row in design['parts']}, 'Current body allocation differs')
    for body, value in current['bodies'].items():
        members = [row for row in current['parts'] if row['body'] == body]
        unresolved = [row['id'] for row in members if not row['resolved']]
        require(value['part_ids'] == [row['id'] for row in members]
                and value['missing_material_part_ids'] == unresolved
                and value['disconnected_part_ids'] == [row['id'] for row in members if row['solid_count'] != 1]
                and value['mass_inputs_complete'] is (not unresolved)
                and value['records_are_partial_subtotals'] is bool(unresolved),
                'Current body completeness/allocation differs: ' + body)
        for field in ('source_density', 'declared_envelope_masses'):
            aggregate = value[field]
            resolved = [row for row in members if row['resolved']]
            if not resolved:
                require(aggregate is None, 'Unknown-only body has an invented mass: ' + body)
                continue
            components = aggregate['components']
            require(len(components) == len(resolved)
                    and [row['part_id'] for row in components] == [row['id'] for row in resolved],
                    'Current aggregate component membership differs: ' + body)
            for component, part in zip(components, resolved):
                props = part['density_properties'] if field == 'source_density' else part['declared_mass_properties']
                require(component['name'] == part['name'] and component['material'] == part['material'],
                        'Current aggregate component identity differs: ' + body)
                center = np.asarray(props['com_mm']) * .001 if field == 'source_density' else props['com_m']
                tensor = props['inertia_kg_m2'] if field == 'source_density' else props['inertia_at_com_kg_m2']
                close(component['mass_kg'], props['mass_kg'], 'Aggregate component mass ' + body, atol=1e-12)
                close(component['com_m'], center, 'Aggregate component CoM ' + body)
                close(component['inertia_at_com_kg_m2'], tensor, 'Aggregate component inertia ' + body, atol=1e-12)
            recomputed = aggregate_mass_properties(components, aggregate['status'])
            for property_name in ('mass_kg', 'com_m', 'inertia_tensor_kg_m2', 'fullinertia_mujoco_kg_m2'):
                close(aggregate[property_name], recomputed[property_name], 'Current aggregate ' + property_name,
                      atol=1e-12)
    source_corrected = json.loads(payloads['refined-mass-properties.json'])
    unattached = {key: value for key, value in report.items() if key not in {
        'geometry_artifacts', 'geometry_study_sha256', 'current_native_body_properties',
        'native_corrected_archival_body_properties'}}
    require(corrected['native_reconciliation_sha256'] == digest(unattached)
            and corrected['original_mass_properties_sha256'] == digest(original)
            and corrected['source_corrected_properties_sha256'] == digest(source_corrected)
            and corrected_link['source_corrected_file_sha256'] == source['source_files']['refined-mass-properties.json']['sha256']
            and corrected['algorithm_source'] == source['operation_sources']['revisions/p5-refined/source/verify_geometry.py'],
            'Corrected body source/native reconciliation identity differs')
    require(corrected['original_archived_properties_modified'] is False
            and corrected['retained_unmodified_component_assumptions'] is True
            and corrected['all_parts_reintegrated'] is False
            and corrected['passed'] is True and corrected_link['passed'] is True,
            'Corrected bodies overstate scope or fail source comparison')
    require(set(corrected['bodies']) == set(corrected['source_comparisons']) == set(source_corrected) == set(original),
            'Corrected body inventory differs')
    recomputed = refined.corrected_archival_bodies(design, unattached, source_corrected)
    require(corrected == recomputed, 'Corrected body receipt differs from measured native component replacements')
    require(corrected['replaced_parts'] == corrected_link['replaced_parts'] == len(report['parts']),
            'Corrected body replacement inventory differs')
    return dict(current_native_body_properties_sha256=current_link['sha256'],
        corrected_archival_body_properties_sha256=corrected_link['sha256'],
        current_native_parts=len(parts), resolved_native_parts=current['resolved_parts'],
        unresolved_physical_material_part_ids=missing, source_mass_override_count=current['source_mass_override_count'],
        complete_native_bodies=[body for body, value in current['bodies'].items() if value['mass_inputs_complete']],
        partial_native_bodies=[body for body, value in current['bodies'].items() if not value['mass_inputs_complete']],
        corrected_source_bodies=len(corrected['bodies']), corrected_native_parts=corrected['replaced_parts'],
        dynamics_rerun=False, scope='Bound native component receipts and full-tensor aggregation; corrected archive assumptions retained')


def verify_refined(directory):
    import cadquery as cq
    import trimesh
    from examples.engineering.arx_compact import design as client, refined
    from mechanism_lab import native_construction, part_geometry, shape_dag
    from mechanism_lab.physical_model import digest

    study = read_json(directory / 'geometry-study.json')
    report = read_json(directory / 'property-reconciliation.json')
    manifest, payloads = refined.load_bundle()
    design, pose, source, overrides, changes, inherited = refined.source_inputs(manifest, payloads)
    require(study['schema'] == 'cybrgeo-geometry-study/1'
            and report['schema'] == 'cybrgeo-native-property-reconciliation/1', 'Unexpected Refined schemas')
    require(source['commit'] == COMMIT and study['source'] == report['source'] == source,
            'Refined source pins or byte-bound source bundle differ')
    require(study['input_design_record_sha256'] == digest(design)
            and study['input_pose_record_sha256'] == digest(pose), 'Refined input records differ')
    require(study['recipe_sha256'] == sha256(Path(client.__file__))
            and report['recipe_sha256'] == sha256(Path(refined.__file__))
            and report['mass_integration_sha256'] == sha256(Path(native_construction.__file__)),
            'Refined report was produced by different construction code')
    resolver_code = part_geometry.require_current_construction()
    client_code = client.require_current_construction()
    refined_code = refined.require_current_construction()
    require(set(resolver_code) == {'src/mechanism_lab/part_geometry.py',
            'src/mechanism_lab/shape_dag.py', 'src/mechanism_lab/native_construction.py'}
            and set(client_code) == set(resolver_code) | {'examples/engineering/arx_compact/design.py'}
            and set(refined_code) == set(client_code) | {'examples/engineering/arx_compact/refined.py'},
            'Refined construction-code binding omitted a required implementation')
    require(study['construction_code_sha256'] == client_code
            and report['construction_code_sha256'] == refined_code,
            'Refined reports do not bind current loaded construction code')
    require(study['part_count'] == len(study['parts']) == len(design['parts']) == 278
            and study['brep_override_count'] == len(overrides) == len(changes) == 123,
            'Incomplete Refined source geometry')
    require(study['captured_body_count'] == 10 and study['time_s'] == pose['time_s'],
            'Unexpected archived placement')
    qualification = study['qualification']
    require(qualification['engineering_acceptance'] is False and qualification['dynamics'] == 'not simulated'
            and qualification['hardware'] == 'unqualified', 'Refined report overstates qualification')
    require(report['passed'] is True and report['changed_parts'] == len(report['parts']) == 123
            and report['inherited_mass_properties_modified'] is False
            and report['source_mass_overrides_used'] is False
            and report['inherited_mass_properties'] == inherited,
            'Refined reconciliation is incomplete or promotes inherited dynamics')
    require(report['property_tolerances'] == refined.TOLERANCES, 'Refined property tolerances changed')
    require(report['geometry_study_sha256'] == sha256(directory / 'geometry-study.json')
            and report['geometry_artifacts'] == study['artifacts'], 'Refined artifact/report binding mismatch')
    require(study['geometry_resolution'] == report['geometry_resolution'], 'Conflicting geometry resolutions')
    resolution = study['geometry_resolution']
    require(resolution['construction_code_sha256'] == resolver_code,
            'Refined resolver report does not bind current loaded construction code')
    require(resolution['implementation_sha256'] == sha256(Path(part_geometry.__file__))
            and resolution['shape_evaluator_sha256'] == sha256(Path(shape_dag.__file__)),
            'Refined geometry resolver code differs')
    selected = {row['part']: row for row in resolution['parts']}
    require(len(selected) == 278 and sum(row['kind'] == 'brep' for row in selected.values()) == 123
            and sum(row['kind'] == 'csg' for row in selected.values()) == 155,
            'Refined BREP/CSG selection differs from the source')
    for override in overrides:
        require(selected[override['part']]['override'] == override, 'Refined BREP binding differs')
    changed = {str(row['id']): row for row in report['parts']}
    records = {str(row['id']): row for row in study['parts']}
    require(set(changed) == set(changes) and set(records) == set(selected), 'Refined part identities differ')
    body_properties = verify_refined_body_reports(directory, report, design, source, payloads, refined_code, records)
    for relative, identity in study['artifacts'].items():
        path = artifact(directory, relative, identity['sha256'])
        require(path.stat().st_size == identity['bytes'], 'Refined artifact byte length differs')
    scene = trimesh.load(directory / 'reference.glb', force='scene')
    require(len(scene.graph.nodes_geometry) == 278, 'Refined GLB does not contain all source parts')
    reopened, changed_mass, exchange_mass = [], [], []
    default_step_discrepancies, source_method_discrepancies = [], []
    for index, (key, record) in enumerate(records.items()):
        path = artifact(directory, record['local_step'], record['local_step_sha256'])
        shape = cq.importers.importStep(str(path)).val()
        valid, solids, volume = bool(shape.isValid()), len(shape.Solids()), float(shape.Volume())
        require(valid and record['valid'] and solids == record['solids'] and solids > 0,
                'Refined STEP is invalid or changed recorded topology: ' + key)
        close(volume, record['volume_mm3'], 'Refined volume ' + key)
        exchange = None
        if key in changed:
            row = changed[key]
            require(row['passed'] and row['archived_refinement'] == changes[key],
                    'Refined changed-part record failed: ' + key)
            for state in ('original', 'refined'):
                residuals = refined.property_residuals(row[state + '_measured'], changes[key][state])
                require(residuals == row[state + '_residuals']
                        and all(value['passed'] for value in residuals.values()),
                        'Refined property acceptance differs: ' + key)
            density = row['density_kg_m3']
            measured = part_geometry.native_properties(shape, density_kg_m3=density)
            expected = row['refined_measured']
            require(valid and solids == 1, 'Changed Refined STEP must remain one valid solid: ' + key)
            native = cq.Shape.importBrep(BytesIO(payloads[selected[key]['override']['path']]))
            native_default = part_geometry.native_properties(native, density_kg_m3=density)
            native_report = refined.property_residuals(native_default, expected)
            require(all(value['passed'] for value in native_report.values()),
                    'Source-method BREP properties differ from accepted archive reconciliation: ' + key)
            error_budget = refined.TOLERANCES['relative']
            native_coarse = exchange_properties(native, density, native_default, 1e-14, error_budget)
            native_fine = exchange_properties(native, density, native_default, 1e-15, error_budget)
            step_coarse = exchange_properties(shape, density, measured, 1e-14, error_budget)
            step_fine = exchange_properties(shape, density, measured, 1e-15, error_budget)
            comparisons = {
                'native_convergence': refined.property_residuals(native_fine['properties'], native_coarse['properties']),
                'step_convergence': refined.property_residuals(step_fine['properties'], step_coarse['properties']),
                'native_step_equivalence': refined.property_residuals(step_fine['properties'], native_fine['properties']),
            }
            require(all(value['passed'] for group in comparisons.values() for value in group.values()),
                    'Uniform native/STEP quadrature convergence or exchange equivalence failed: ' + key)
            default_residuals = refined.property_residuals(measured, expected)
            method_residuals = refined.property_residuals(native_fine['properties'], expected)
            if not all(value['passed'] for value in default_residuals.values()):
                default_step_discrepancies.append(record['id'])
            if not all(value['passed'] for value in method_residuals.values()):
                source_method_discrepancies.append(record['id'])
            exchange = dict(native_default=native_default, step_default=measured,
                native_coarse=native_coarse, native_fine=native_fine,
                step_coarse=step_coarse, step_fine=step_fine, comparisons=comparisons,
                source_method_to_report=native_report, default_step_to_report=default_residuals,
                adaptive_source_to_report=method_residuals,
                diagnostic_scope='Default-versus-adaptive residuals are retained, not substituted for the archived source-method acceptance')
            changed_mass.append(native_default['mass_kg'])
            exchange_mass.append(step_fine['properties']['mass_kg'])
        reopened.append(dict(id=record['id'], valid=valid, solids=solids, volume_mm3=volume,
                             local_step=record['local_step'], sha256=record['local_step_sha256'], exchange=exchange))
        if (index + 1) % 25 == 0 or index + 1 == len(records):
            print(f'Reopened Refined STEP parts: {index + 1}/{len(records)}', flush=True)
    close(math.fsum(row['geometry_density_mass_delta_kg'] for row in report['parts']),
          report['geometry_density_mass_delta_kg'], 'Refined summed mass delta', atol=1e-10)
    return dict(passed=True, source_commit=COMMIT, source_bundle_sha256=digest(manifest),
                construction_code_sha256=refined_code,
                source_files=len(manifest['files']), source_parts=278, brep_overrides=123, csg_parts=155,
                reconciled_parts=123, changed_geometry_mass_kg=math.fsum(changed_mass),
                adaptive_exchange_mass_kg=math.fsum(exchange_mass),
                exchange_method=dict(api='BRepGProp.VolumeProperties_s: three-argument adaptive overload',
                    integration='Uniform adaptive 2D Gauss for both source BREP and reopened STEP',
                    reference_point_mm=[0, 0, 0], only_closed=False, skip_shared=False,
                    requested_relative_errors=[1e-14, 1e-15], acceptance_tolerances=refined.TOLERANCES,
                    density_units='kg/m3', native_length_units='mm', mass_conversion=1e-9, inertia_conversion=1e-15,
                    scope='Converged exchange properties only; archived/default-method producer masses are unchanged'),
                default_step_report_discrepancy_parts=default_step_discrepancies,
                source_integration_method_discrepancy_parts=source_method_discrepancies,
                geometry_density_mass_delta_kg=report['geometry_density_mass_delta_kg'],
                disconnected_parts=study['disconnected_parts'], authored_bound_overruns=study['authored_bound_overruns'],
                native_readback=reopened, geometry_study_sha256=sha256(directory / 'geometry-study.json'),
                property_reconciliation_sha256=sha256(directory / 'property-reconciliation.json'),
                body_property_receipts=body_properties,
                artifact_hashes=study['artifacts'], scope=SCOPE)


def verify_print_first(directory):
    import cadquery as cq
    import numpy as np
    from examples.engineering.arx_compact import print_first
    from mechanism_lab import native_construction
    from mechanism_lab.manufacturing_joints import native_bounds
    from mechanism_lab.native_construction import shape_mass_properties, aggregate_mass_properties
    from mechanism_lab.physical_model import digest

    receipt = read_json(directory / 'native-build.json')
    plan = read_json(directory / 'print-first-plan.json')
    parts = read_json(directory / 'cad_parts.json')
    masses = read_json(directory / 'mass_properties.json')
    interfaces = read_json(directory / 'native-interface-requirements.json')
    spec, identity, *specification = print_first.specification()
    gears = specification[-1]
    require(receipt['schema'] == 'cybrgeo-native-build/1' and plan['schema'] == 'cybrgeo-print-first-plan/1',
            'Unexpected print-first schemas')
    require(receipt['parts'] == receipt['placements'] == len(parts) == 6 and set(parts) == PRINT_PARTS,
            'Incomplete print-first source articles')
    provenance = receipt['provenance']
    require(provenance['source'] == print_first.SOURCE and provenance['source']['commit'] == COMMIT
            and provenance['source_inputs'] == print_first.verified_inputs(), 'Print-first source pins differ')
    require(provenance['native_construction_sha256'] == sha256(Path(native_construction.__file__)),
            'Print-first native construction code differs')
    code_hashes = print_first.construction_code_hashes()
    require(set(code_hashes) == {
        'examples/engineering/arx_compact/print_first.py',
        'src/mechanism_lab/manufacturing_joints.py',
        'src/mechanism_lab/manufacturing_gears.py',
        'src/mechanism_lab/native_construction.py',
    } and provenance['construction_code_sha256'] == code_hashes,
        'Print-first receipt does not bind all four current construction files')
    require(provenance['input_spec_sha256'] == identity['input_spec_sha256']
            and read_json(directory / 'input-spec.json') == spec, 'Print-first input is not the source default')
    for relative, expected in receipt['files_sha256'].items():
        artifact(directory, relative, expected)
    require(plan['gear_pair'] == gears.to_source() and plan['source_module_scaling'] == gears.scaling_from(module_mm=.3, teeth=20),
            'Print-first tooth geometry and pitch scaling disagree')
    require(plan['source_module_scaling']['torque_ratio_for_same_rack_force'] == 4,
            'Coarse prototype silently inherited original gripper torque scaling')
    require(len(plan['print_files_sha256']) == 12, 'Missing native print-local files')
    for relative, expected in plan['print_files_sha256'].items():
        artifact(directory, relative, expected)
    require(interfaces['schema'] == 'cybrgeo-native-interfaces/1' and interfaces['passed']
            and len(interfaces['checks']) == 3, 'Missing nominal splice acceptance')
    for row in interfaces['checks']:
        require(row['pass'] and row['clearance_passed']
                and row['intersection_volume_mm3'] <= row['volume_tolerance_mm3']
                and row['minimum_distance_mm'] + row['distance_kernel_allowance_mm'] >= row['required_minimum_clearance_mm'],
                'Nominal splice acceptance failed: ' + row['name'])
    records = {row['name']: row for row in plan['parts']}
    require(set(records) == PRINT_PARTS and len(plan['parts']) == 6, 'Print-first plan identities differ')
    placements = read_json(directory / 'placed-native-parts.json')['records']
    require({row['name'] for row in placements} == PRINT_PARTS and len(placements) == 6,
            'Print-first native placements are incomplete')
    bodies, print_bounds = {}, {}
    # Arithmetic/kernel allowance at the nominal Z=0 plate; not a printer fit
    # or fabrication tolerance and far smaller than the explicit 5 mm margin.
    coordinate_allowance_mm = 1e-6
    for name, record in records.items():
        shape = cq.importers.importStep(str(directory / 'cad' / (name + '.step'))).val()
        require(shape.isValid() and len(shape.Solids()) == 1 and record['valid_single_solid']
                and record['step_roundtrip_valid'] and record['step_volume_relative_error'] <= 1e-8,
                'Invalid reopened print article: ' + name)
        close(shape.Volume(), parts[name]['volume_mm3'], 'Print volume ' + name)
        density = spec['material']['density_kg_m3']
        mass, center, tensor = shape_mass_properties(shape, density)
        close(mass, parts[name]['mass_kg'], 'Print mass ' + name, atol=1e-10)
        bounds = native_bounds(shape)
        measured_bounds = [[getattr(bounds, axis + edge) for axis in ('x', 'y', 'z')] for edge in ('min', 'max')]
        close(measured_bounds, record['source_bounds_mm'], 'Print source bounds ' + name, atol=1e-5)
        require(max(record['print_size_mm']) <= spec['planning_cube_mm'], 'Print planning envelope exceeded')
        local = cq.importers.importStep(str(directory / 'print-local' / (name + '.step'))).val()
        require(local.isValid() and len(local.Solids()) == 1, 'Invalid print-local STEP: ' + name)
        close(local.Volume(), shape.Volume(), 'Print-local volume ' + name)
        local_box = native_bounds(local)
        local_bounds = [[getattr(local_box, axis + edge) for axis in ('x', 'y', 'z')] for edge in ('min', 'max')]
        require(min(local_bounds[0]) >= -coordinate_allowance_mm
                and max(local_bounds[1]) <= spec['planning_cube_mm'] + coordinate_allowance_mm,
                'Actual print-local STEP exceeds the declared planning cube: ' + name)
        close(local_bounds, np.asarray(measured_bounds) + record['print_translation_mm'],
              'Print-local bounds ' + name, atol=1e-5)
        close(local_bounds, record['print_occupied_bounds_mm'],
              'Declared occupied print bounds ' + name, atol=1e-5)
        print_bounds[name] = local_bounds
        close(local.Center().toTuple(), np.asarray(shape.Center().toTuple()) + record['print_translation_mm'],
              'Print-local translation ' + name, atol=1e-5)
        placed = next(row for row in placements if row['name'] == name)
        bodies.setdefault(placed['body'], []).append(dict(name=name, mass_kg=mass, com_m=center.tolist(),
            inertia_at_com_kg_m2=tensor.tolist(), provenance='Independent STEP density integral'))
    require(set(bodies) == set(masses) == set(receipt['body_ids']) and len(bodies) == 4,
            'Print-first assembly mass coverage differs')
    for body, rows in bodies.items():
        measured = aggregate_mass_properties(rows)
        for key, tolerance in (('mass_kg', 1e-10), ('com_m', 1e-8), ('inertia_tensor_kg_m2', 1e-12)):
            close(measured[key], masses[body][key], 'Print body ' + body + '/' + key, atol=tolerance)
    return dict(passed=True, source_commit=COMMIT, source=provenance['source'],
                construction_code_sha256=code_hashes,
                source_input_sha256={row['file']:row['sha256'] for row in provenance['source_inputs']['inputs']},
                input_spec_sha256=identity['input_spec_sha256'], native_build_sha256=digest(receipt),
                native_build_canonical_sha256=digest(receipt),
                native_build_file_sha256=sha256(directory / 'native-build.json'),
                native_build_sha256_encoding='Canonical JSON via physical_model.digest; raw file bytes use native_build_file_sha256',
                parts=6, placements=6, mass_bodies=4, native_step_readbacks=6, print_local_step_readbacks=6,
                verified_native_artifacts=len(receipt['files_sha256']), verified_print_artifacts=12,
                nominal_interface_checks=interfaces['checks'], source_module_scaling=plan['source_module_scaling'],
                print_local_bounds_mm=print_bounds, print_coordinate_kernel_allowance_mm=coordinate_allowance_mm,
                solid_continuum_mass_kg=math.fsum(row['mass_kg'] for row in parts.values()), scope=SCOPE)


def run(args):
    require(math.isfinite(args.timeout) and args.timeout > 0, 'Timeout must be finite and positive')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    projects = ['refined', 'print-first'] if args.only is None else [args.only]
    locations = {'refined': (args.refined_output or output / 'refined').resolve(),
                 'print-first': (args.print_first_output or output / 'print-first').resolve()}
    require(args.check_existing or not (args.refined_output or args.print_first_output),
            'External output locations are supported only by explicit --check-existing')
    if not args.check_existing:
        for name in projects:
            directory = locations[name]
            require(not directory.exists() or not any(directory.iterdir()),
                    'Fresh CI construction requires an empty project output: ' + str(directory))
    env = os.environ.copy()
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    env['PYTHONPATH'] = str(ROOT / 'src') + os.pathsep + str(ROOT) + (os.pathsep + env['PYTHONPATH'] if env.get('PYTHONPATH') else '')
    started = time.perf_counter()
    deadline = started + args.timeout
    status = dict(schema='cybrgeo-latest-arx-ci/1', passed=False, source_commit=COMMIT,
                  constructed_fresh=not args.check_existing, timeout_s=args.timeout, versions=versions(),
                  helper_sha256=sha256(Path(__file__)), projects=projects, stages=[], scope=SCOPE)
    # An interrupted rerun must not leave an earlier completed receipt looking
    # like the current command's result. Project CAD is never removed here.
    write_json(output / 'ci-latest-arx.json', status)
    try:
        for name in projects:
            module = 'examples.engineering.arx_compact.' + name.replace('-', '_')
            stages = [] if args.check_existing else [
                ('build', [sys.executable, '-B', '-u', '-m', module, '--output', str(locations[name])])]
            stages.append(('readback', [sys.executable, '-B', '-u', str(Path(__file__).resolve()),
                '--readback', name, '--output', str(locations[name]),
                '--evidence', str(output / (name + '-readback.json'))]))
            for action, command in stages:
                stage = dict(name=name + '-' + action, command=command, elapsed_s=0, passed=False)
                status['stages'].append(stage)
                log = output / (stage['name'] + '.log')
                stage_started = time.perf_counter()
                print('Starting ' + stage['name'], flush=True)
                try:
                    remaining = deadline - time.perf_counter()
                    require(remaining > 0, 'Shared geometry deadline expired')
                    with log.open('w', encoding='utf-8') as stream:
                        result = subprocess.run(command, cwd=ROOT, env=env, stdout=stream,
                            stderr=subprocess.STDOUT, timeout=remaining)
                    stage['returncode'] = result.returncode
                    result.check_returncode()
                    stage['passed'] = True
                finally:
                    stage['elapsed_s'] = time.perf_counter() - stage_started
                    if log.exists():
                        print(log.read_text(encoding='utf-8', errors='replace')[-10000:], flush=True)
                    print(json.dumps(stage), flush=True)
        status['passed'] = True
    except Exception as error:
        status['error'] = type(error).__name__ + ': ' + str(error)
        raise
    finally:
        status['elapsed_s'] = time.perf_counter() - started
        status['reports'] = {name: str(output / (name + '-readback.json')) for name in projects}
        write_json(output / 'ci-latest-arx.json', status)
        print(json.dumps(status, indent=2), flush=True)


def main():
    faulthandler.enable()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('outputs/portable-latest-arx'))
    parser.add_argument('--timeout', type=float, help='One deadline in seconds for all builds/readbacks; required for supervised runs')
    parser.add_argument('--check-existing', action='store_true', help='Local readback only; never records fresh construction')
    parser.add_argument('--refined-output', type=Path)
    parser.add_argument('--print-first-output', type=Path)
    parser.add_argument('--only', choices=('refined', 'print-first'), help='Select one client for local diagnosis')
    parser.add_argument('--readback', choices=('refined', 'print-first'), help=argparse.SUPPRESS)
    parser.add_argument('--evidence', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.readback:
        require(args.evidence is not None, 'Readback requires an evidence path')
        args.evidence.unlink(missing_ok=True)
        started = time.perf_counter()
        result = (verify_refined if args.readback == 'refined' else verify_print_first)(args.output.resolve())
        result.update(elapsed_s=time.perf_counter() - started, versions=versions())
        write_json(args.evidence, result)
        print(json.dumps({key: value for key, value in result.items() if key != 'native_readback'}, indent=2), flush=True)
    else:
        if args.timeout is None:
            parser.error('--timeout is required; choose the workflow budget from measured build/readback times')
        run(args)


if __name__ == '__main__':
    main()
