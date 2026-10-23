"""Bound verification for this new consumer, using shared GEO construction.

This is a finite native-solid audit of the actual integrated frames. It does
not generate motion and does not establish continuous-time clearance.
"""
from copy import deepcopy
import gzip
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np


def _read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify(output, *, sample_count=65):
    import cadquery as cq
    import mujoco
    import trimesh
    from mechanism_lab.core import pose_cad
    from mechanism_lab.engineering_source import load_project, verify_native_build
    from mechanism_lab.native_construction import shape_mass_properties
    from mechanism_lab.physical_model import PhysicalModel, digest

    if isinstance(sample_count, bool) or not isinstance(sample_count, int) or sample_count < 2:
        raise ValueError('Sample count must be an integer of at least two')
    output = Path(output).resolve()
    project, source_files = load_project('fourbar_lift')
    report = _read(output/'engineering.json')
    native = output/'native'
    receipt = verify_native_build(native, project.specification, source_files=source_files)
    assert report['passed'] and report['source_files'] == source_files
    assert report['native_build_sha256'] == digest(receipt)
    with gzip.open(output/'simulation'/'trace.json.gz', 'rt', encoding='utf-8') as stream:
        trace = json.load(stream)
    assert project.requirements(trace, trace['task_parameters']) == report['requirements'] == trace['requirements']
    physical_source = _read(output/'physical-source.json')
    control_source = _read(output/'control-source.json')
    assert physical_source == trace['physical_source']
    assert digest(physical_source) == report['model_sha256'] == trace['model_sha256']
    assert digest(control_source) == report['control_sha256'] == trace['control_sha256']
    assert PhysicalModel.from_source(physical_source).xml() == (output/'model.xml').read_text(encoding='utf-8')
    assert report['simulation'] == {key:value for key,value in trace.items()
        if key not in ('frames','task_parameters','physical_source','requirements')}
    # Replay independently rechecks source/native/mass bindings and imports
    # every placed STEP. Its authority remains the common native_replay module.
    assembly = project.replay(native, trace)
    coverage = _read(output/'fourbar_lift.coverage.json')
    aggregate = cq.importers.importStep(str(output/'fourbar_lift.step')).val()
    glb = trimesh.load(output/'assembly.glb', force='scene', process=False)
    assert not coverage['mesh_only_components_not_in_STEP']
    assert len(coverage['analytic_components']) == len(assembly.parts) == len(glb.graph.nodes_geometry) == 15
    assert aggregate.isValid() and len(aggregate.Solids()) == 15
    model = mujoco.MjModel.from_xml_path(str(output/'model.xml'))
    assert model.nu == 1 and model.njnt == 3 and model.neq == 1 and model.ngeom == 0

    native_records = _read(native/'placed-native-parts.json')['records']
    masses = _read(native/'mass_properties.json')
    densities = receipt['material_densities_kg_m3']
    shapes = {}
    mass_checks = []
    for row in native_records:
        shape = cq.importers.importStep(str(native/row['step_file'])).val()
        assert shape.isValid() and shape.Volume() > 0
        shapes[(row['body'], row['name'])] = shape
        mass, center, tensor = shape_mass_properties(shape, densities[row['material']])
        source = next(item for item in masses[row['body']]['components'] if item['name'] == row['name'])
        assert np.isclose(mass, source['mass_kg'], rtol=1e-8, atol=1e-12)
        assert np.allclose(center, source['com_m'], rtol=1e-8, atol=1e-12)
        assert np.allclose(tensor, source['inertia_at_com_kg_m2'], rtol=1e-8, atol=1e-12)
        mass_checks.append(dict(body=row['body'], name=row['name'], mass_kg=mass,
            mass_difference_kg=mass-source['mass_kg'],
            maximum_inertia_difference_kg_m2=float(np.max(np.abs(tensor-source['inertia_at_com_kg_m2'])))))

    indices = sorted({int(index) for index in np.linspace(0, len(trace['frames'])-1, sample_count, dtype=int)})
    pairs = [(first, second) for first,second in itertools.combinations(shapes,2) if first[0]!=second[0]]
    audit = {first[0]+'__'+first[1]+' / '+second[0]+'__'+second[1]:dict(
        parts=[list(first),list(second)], minimum_gap_mm=float('inf'), maximum_overlap_mm3=0., exact_checks=0)
        for first,second in pairs}
    broad = exact = 0
    for index in indices:
        frame = trace['frames'][index]
        posed, bounds = {}, {}
        for key,shape in shapes.items():
            body = frame['bodies'][key[0]]
            matrix = np.eye(4)
            matrix[:3,:3] = np.asarray(body['rotation_row_major']).reshape(3,3)
            matrix[:3,3] = np.asarray(body['position_m'])*1000
            placed = pose_cad(shape, matrix)
            posed[key] = placed
            box = placed.BoundingBox()
            bounds[key] = np.array([[box.xmin,box.ymin,box.zmin],[box.xmax,box.ymax,box.zmax]])
        for first,second in pairs:
            row = audit[first[0]+'__'+first[1]+' / '+second[0]+'__'+second[1]]
            a,b = bounds[first],bounds[second]
            lower = float(np.linalg.norm(np.maximum(0., np.maximum(a[0]-b[1],b[0]-a[1]))))
            # Broad-phase rejection is a valid nonintersection certificate;
            # only nearby bodies need the exact native Boolean operation.
            if lower>.2:
                broad += 1
                continue
            overlap = float(posed[first].intersect(posed[second]).Volume())
            distance = float(posed[first].distance(posed[second]))
            row['exact_checks'] += 1
            row['minimum_gap_mm'] = min(row['minimum_gap_mm'], distance)
            row['maximum_overlap_mm3'] = max(row['maximum_overlap_mm3'], overlap)
            assert overlap <= 1e-6, (frame['time_s'], first,second,overlap)
            exact += 1
    for row in audit.values():
        if row['minimum_gap_mm'] == float('inf'):
            row['minimum_gap_mm'] = None
            row['broad_phase_minimum_gap_exceeds_mm'] = .2
    measured_gaps = [row['minimum_gap_mm'] for row in audit.values() if row['minimum_gap_mm'] is not None]
    result = dict(schema='cybrgeo-fourbar-lift-readback/1', passed=True,
        source_files=source_files, model_sha256=report['model_sha256'],
        native_build_sha256=report['native_build_sha256'], native_source_binding_sha256=report['native_source_binding_sha256'],
        native_parts=receipt['parts'], placed_parts=receipt['placements'], native_artifacts=len(receipt['files_sha256']),
        aggregate_step_solids=len(aggregate.Solids()), glb_nodes=len(glb.graph.nodes_geometry),
        moving_mass_kg={key:row['mass_kg'] for key,row in masses.items() if key!='fixed'},
        native_step_mass_checks=mass_checks,
        sampled_clearance=dict(frame_indices=indices, times_s=[trace['frames'][i]['time_s'] for i in indices],
            body_pair_checks=len(pairs)*len(indices), broad_phase_nonintersections=broad,
            exact_native_pair_checks=exact, minimum_exact_gap_mm=min(measured_gaps),
            maximum_allowed_overlap_mm3=1e-6, pairs=list(audit.values()),
            scope='Finite actual integrated native-solid poses; no interpolation or continuous-time assembly proof'),
        files_sha256={name:_sha(output/name) for name in ('engineering.json','physical-source.json','model.xml','assembly.glb',
            'fourbar_lift.step','fourbar_lift.coverage.json','simulation/trace.json.gz')},
        consumer_path=['engineering_pipeline.generate_project','NativeConstruction','PinnedLink','PhysicalModel',
            'CausalSimulation','native_replay.build_assembly','exporters.export_step'],
        qualification='Native property/exchange checks and bounded ideal-pin dynamics; no measured manufacturing process or hardware rating')
    (output/'fourbar-readback.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    return result


def causal_controls(output):
    from mechanism_lab.causal_runtime import CausalSimulation
    from mechanism_lab.engineering_source import load_project, verify_native_build, verify_native_source_binding
    from mechanism_lab.physical_model import digest
    output = Path(output).resolve()
    project, source_files = load_project('fourbar_lift')
    native = output/'native'
    native_receipt = verify_native_build(native,project.specification,source_files=source_files)
    binding = verify_native_source_binding(native,project.specification,source_files,receipt=native_receipt)
    records = []
    for name, options, disconnected in (('power-disabled', {'power':False}, False),
                                         ('coupler-disconnected', {}, True)):
        physical,control,parameters = project.compile_physics(project.specification, native_directory=output/'native')
        physical.provenance.update(native_build_sha256=digest(native_receipt),
            native_source_binding_sha256=digest(binding),design_source_files=source_files)
        if disconnected:
            equality = physical.root.find('equality')
            equality.remove(equality.find("connect[@name='coupler_closure']"))
            physical.provenance['adverse_case'] = 'Numerically disconnected coupler closure; diagnostic control, not an alternative hardware assembly'
        runtime = CausalSimulation(physical,control,lambda t:project.reference(t,parameters),options=options,
            reference_source=dict(source_files=source_files,task_parameters=parameters))
        trace = runtime.run(project.duration_s, output=output/'controls'/name)
        outcome = project.requirements(trace, parameters)
        trace.update(physical_source=physical.to_source(),task_parameters=parameters,requirements=outcome)
        (output/'controls'/name/'trace.json.gz').write_bytes(gzip.compress(
            json.dumps(trace,separators=(',',':'),allow_nan=False).encode(),mtime=0))
        assert not outcome['passed'], name+' must falsify the nominal task'
        if disconnected:
            assert 'coupler_closure' not in trace['metrics']['constraints']
        else:
            assert trace['metrics']['actuators']['crank_motor']['peak_current_A']==0.
        records.append(dict(name=name, expected='nominal task rejected', observed_passed=outcome['passed'],
            outcome=outcome, model_sha256=trace['model_sha256'], control_sha256=trace['control_sha256'],
            options=trace['options'], steps=trace['steps'],events=trace['events'],
            trace_sha256=_sha(output/'controls'/name/'trace.json.gz')))
    receipt = dict(schema='cybrgeo-fourbar-causal-controls/1', passed=True, cases=records,
        native_build_sha256=digest(native_receipt),native_source_binding_sha256=digest(binding),source_files=source_files,
        scope='Initialization power-off and removed numerical pin constraint; neither is a qualified emergency-stop test')
    (output/'causal-controls.json').write_text(json.dumps(receipt,indent=2)+'\n',encoding='utf-8')
    return receipt


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('outputs/fourbar_lift'))
    parser.add_argument('--build',action='store_true')
    parser.add_argument('--controls',action='store_true')
    parser.add_argument('--clearance-samples',type=int,default=65)
    args = parser.parse_args()
    if args.build:
        from mechanism_lab.engineering_pipeline import generate_project
        from mechanism_lab.exporters import export_step
        assembly,report = generate_project('fourbar_lift',output=args.output)
        if not report['passed']:
            raise SystemExit('Nominal four-bar task failed')
        export_step(assembly,args.output/'fourbar_lift.step')
    result = verify(args.output,sample_count=args.clearance_samples)
    if args.controls:
        causal_controls(args.output)
    audit = result['sampled_clearance']
    print(json.dumps(dict(passed=result['passed'], native_parts=result['native_parts'],
        sampled_frames=len(audit['frame_indices']), pair_checks=audit['body_pair_checks'],
        exact_checks=audit['exact_native_pair_checks'], minimum_exact_gap_mm=audit['minimum_exact_gap_mm']),indent=2))


if __name__ == '__main__':
    main()
