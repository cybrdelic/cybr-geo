"""ARX belt native geometry adapter; physical replay lives in shared core.

Placement conventions follow pinned aabdf8c capture_csg.py and
export_p5_assembly.py. This adapter does not reproduce their commanded FK or
cosmetic drive phase: all body placement comes from the new integrated trace.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import cadquery as cq
import numpy as np

from mechanism_lab.core import Material, pose_cad
from mechanism_lab.physical_model import PhysicalModel, digest
from mechanism_lab.physical_replay import Y_UP_TO_Z_UP, build_assembly as bind_meshes
from mechanism_lab.engineering_source import verify_native_build, verify_native_source_binding


def _record(shape, body, name, material):
    vertices, faces = shape.tessellate(.15, .20)
    return dict(body=body, name=name, material=material,
                vertices_mm=[v.toTuple() for v in vertices], triangles=faces,
                provenance='imported-analytic-cad')


def _quaternion_rotation(values):
    q = np.asarray(values, float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError('Scene geom requires a finite quaternion')
    w,x,y,z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def build_assembly(native_directory, simulation_result, *, analytic=True):
    """Return native core Assembly; ``analytic=True`` retains all placed BReps.

    Caller must supply task_parameters and physical_source from the same run.
    No archived film, default trajectory, baseline scene or welded puck fallback
    is used. CAD records are body-local mm; simulation uses SI Y-up.
    """
    directory = Path(native_directory).resolve()
    native_path = directory/'geometry-meshes.json'
    document = json.loads(native_path.read_text(encoding='utf-8'))
    if 'task_parameters' not in simulation_result or 'physical_source' not in simulation_result:
        raise ValueError('ARX replay requires this run\'s task_parameters and physical_source')
    parameters = simulation_result['task_parameters']
    reference = simulation_result.get('reference_source')
    if (not isinstance(reference,dict) or digest(reference) != simulation_result.get('reference_sha256')
            or reference.get('task_parameters') != parameters):
        raise ValueError('Replay task parameters must match the bound simulation reference source')
    source_files = reference.get('source_files', {})
    if not source_files:
        raise ValueError('Native replay requires the complete authored project source identity')
    native_receipt = verify_native_build(directory, parameters['integration'], source_files=source_files)
    source_binding = verify_native_source_binding(directory, parameters['integration'], source_files, receipt=native_receipt)
    source = simulation_result['physical_source']
    if digest(source) != simulation_result['model_sha256']:
        raise ValueError('Replay physical source does not match simulation model identity')
    physical = PhysicalModel.from_source(source)
    if physical.provenance.get('native_build_sha256') != digest(native_receipt):
        raise ValueError('Native build receipt is not bound to this simulation physical source')
    if physical.provenance.get('native_source_binding_sha256') != digest(source_binding):
        raise ValueError('Native construction source binding is not bound to this simulation physical source')
    physical.require_free_body('puck')
    masses = json.loads((directory/'mass_properties.json').read_text(encoding='utf-8'))
    expected_masses = {body:masses[body] for body in ('turret','upper','fore','roll','bend','tool')}
    if 'jaw' in masses:
        expected_masses['jaw_left'] = masses['jaw']
        # Check the precise declared model adapter's derived opposite-jaw
        # tensor. It is a reflection assumption, not a fresh left-jaw CAD mass.
        reflected = deepcopy(masses['jaw'])
        mirror = np.diag([-1.,1.,1.])
        reflected['com_m'] = (mirror @ np.asarray(reflected['com_m'])).tolist()
        tensor = mirror @ np.asarray(reflected['inertia_tensor_kg_m2']) @ mirror
        reflected['inertia_tensor_kg_m2'] = tensor.tolist()
        reflected['fullinertia_mujoco_kg_m2'] = [tensor[0,0],tensor[1,1],tensor[2,2],tensor[0,1],tensor[0,2],tensor[1,2]]
        reflected['components'] = []
        reflected['provenance'] = 'Reflection of the native authored opposite jaw mass tensor about joint-local X'
        expected_masses['jaw_right'] = masses.get('jaw_opposed', reflected)
    else:
        expected_masses.update({body:masses[body] for body in ('jaw_left','jaw_right') if body in masses})
    if digest(expected_masses) != digest(physical.mass_records):
        raise ValueError('Native mapped/derived mass records differ from the simulation source')
    cads = {}
    if analytic:
        placements = json.loads((directory/'placed-native-parts.json').read_text(encoding='utf-8'))['records']
        for row in placements:
            path = (directory/row['step_file']).resolve()
            if not path.is_relative_to(directory):
                raise ValueError('Placed STEP reference escapes native output directory')
            cads[f"{row['body']}__{row['name']}"] = cq.importers.importStep(str(path)).val()
        expected = {f"{row['body']}__{row['name']}" for row in document['meshes']}
        if set(cads) != expected:
            raise ValueError('Placed analytic CAD identities do not match native display meshes')
    # Source positive-X jaw is named jaw_left in the physical model. The other
    # carriage is handed (different rack slot), so a whole-jaw mirror is wrong.
    carriage = cq.importers.importStep(str(directory/'cad/left_jaw_carriage.step')).val().translate((0,-219.2,0))
    document['meshes'].append(_record(carriage, 'opposed_jaw', 'left_jaw_carriage', 'aluminum'))
    cads['opposed_jaw__left_jaw_carriage'] = carriage
    for row in list(document['meshes']):
        if row['body'] != 'jaw' or 'right_jaw_carriage' in row['name']:
            continue
        other = deepcopy(row)
        other['body'] = 'opposed_jaw'
        vertices = np.asarray(row['vertices_mm'], float)
        if row['name'] == 'upper_rack':
            other['name'] = 'lower_rack'
            matrix = np.diag([-1.,-1.,1.])
            offset = np.array([0.,-35.4,0.])
            other['vertices_mm'] = (vertices @ matrix.T + offset).tolist()
            if analytic:
                shape = cads[f"jaw__{row['name']}"]
                cads['opposed_jaw__lower_rack'] = shape.rotate((0,0,0),(0,0,1),180).translate(tuple(offset))
        else:
            other['vertices_mm'] = (vertices * [-1,1,1]).tolist()
            other['triangles'] = np.asarray(row['triangles'])[:,[0,2,1]].tolist()
            if analytic:
                cads[f"opposed_jaw__{other['name']}"] = cads[f"jaw__{row['name']}"] .mirror('YZ')
        document['meshes'].append(other)
    # The free workpiece is visible geometry from the exact run parameters. Its
    # position is only the integrated puck body's position, never tool-relative.
    puck = parameters['puck']
    shape = (cq.Workplane('XZ').circle(puck['outer_radius_m']*1000)
             .circle(puck['bore_radius_m']*1000).extrude(puck['height_m']*500,both=True).val())
    document['meshes'].append(_record(shape,'puck','free_workpiece','workpiece'))
    cads['puck__free_workpiece'] = shape
    # Render exactly the authored nest cylinders from this run's worldbody.
    for name in ('nest_A_base','nest_A_seat','nest_B_base','nest_B_seat'):
        geom = physical.root.find(f"worldbody/geom[@name='{name}']")
        if geom is None or geom.get('type') != 'cylinder' or any(key in geom.attrib for key in ('euler','axisangle','xyaxes','zaxis','fromto')):
            raise ValueError(f'{name} needs an explicit worldbody cylinder with quaternion orientation')
        radius,half = [float(x)*1000 for x in geom.get('size').split()]
        shape = cq.Workplane('XY').circle(radius).extrude(half*2).val().translate((0,0,-half))
        transform = np.eye(4)
        transform[:3,:3] = _quaternion_rotation([float(x) for x in geom.get('quat','1 0 0 0').split()])
        transform[:3,3] = [float(x)*1000 for x in geom.get('pos','0 0 0').split()]
        shape = pose_cad(shape,transform)
        material = 'rubber' if name.endswith('base') else 'aluminum'
        document['meshes'].append(_record(shape,'world',name,material))
        cads[f'world__{name}'] = shape
    palette = {
        'aluminum': Material('Aluminum source appearance',(.66,.70,.75),.65,.32),
        'steel': Material('Steel source appearance',(.26,.29,.33),.75,.30),
        'rubber': Material('Rubber source appearance',(.10,.11,.12),0,.72),
        'polymer': Material('Polymer source appearance',(.23,.27,.31),0,.60),
        'workpiece': Material('Free aluminum workpiece',(.72,.63,.38),.5,.36),
    }
    mapping = {row['body']:row['body'] for row in document['meshes']}
    mapping.update(fixed='fixed_hardware',jaw='jaw_left',opposed_jaw='jaw_right')
    assembly = bind_meshes(document,mapping,simulation_result,
        name='ARX belt / integrated native geometry', materials=palette,
        world_rotation=Y_UP_TO_Z_UP, metadata={
            'native_geometry_file_sha256':hashlib.sha256(native_path.read_bytes()).hexdigest(),
            'native_source':native_receipt['provenance'],
            'visual_limitations':[
                'Four deforming spring coil visuals are omitted; spring forces remain in the physical model.',
                'Internal input shafts, drive pulleys, pinion and belt teeth retain native local phase; separate rotor/belt material states are not recorded.',
                'Flexible cable harness is omitted; contacts use the separate declared conservative proxy model.',
                'Workbench surface is omitted from the display; its authored physical contacts remain active.',
                'Workpiece bore is native visual geometry; its physical contact primitive remains the declared cylinder.',
                'Display finishes are source appearance, not a measured material or manufacturing qualification.',
            ]})
    assembly.views['hero'].floor = False
    for part in assembly.parts:
        if part.name in cads:
            part.cad = cads[part.name]
            part.provenance = 'imported-analytic-cad'
    if analytic and any(part.cad is None for part in assembly.parts):
        raise ValueError('Analytic replay must retain a BRep for every display part')
    return assembly
