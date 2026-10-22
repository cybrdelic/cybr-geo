"""Reusable CAD/assembly -> MJCF -> actual MuJoCo rigid-body simulation.

Geometry uses mm/Z-up; physics uses SI.  No architecture names occur here.
Exact B-rep mass integrals (or explicitly identified watertight mesh integrals)
and declared material densities are the inertial inputs. Motors are ideal
effort-limited outputs, not vendor-qualified electrical/thermal models.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import replace
import gzip
import hashlib
import json
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

from .constructed import KinematicGraph, _check_construction_identity


def _mujoco():
    try:
        import mujoco
    except ImportError as exc:
        raise RuntimeError('MuJoCo is required: install the declared cybr-geo dependencies. No prescribed-motion fallback is used.') from exc
    return mujoco


def _finite(value, name, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or (positive and value <= 0):
        raise ValueError(f'{name} must be finite' + (' and positive' if positive else ''))
    return float(value)


def _vec(value, name, length=3):
    result = np.asarray(value, dtype=float)
    if result.shape != (length,) or not np.isfinite(result).all():
        raise ValueError(f'{name} must contain {length} finite numbers')
    return result


def _fmt(value):
    return ' '.join(format(float(x), '.15g') for x in np.asarray(value).reshape(-1))


def _recipe(assembly):
    recipe = assembly.metadata.get('construction_recipe')
    if not isinstance(recipe, dict):
        raise ValueError('A mechanical recipe with parts and joints is required for dynamics')
    _check_construction_identity(recipe)
    if (recipe.get('construction') or {}).get('mechanism'):
        from .construction import validate_constructed_plan
        validate_constructed_plan(recipe)
    if {p.name for p in assembly.parts} != {p['name'] for p in recipe['parts']}:
        raise ValueError('Dynamics geometry and mechanical recipe part names disagree')
    return recipe


def _mesh_digest(part):
    digest = hashlib.sha256()
    for value, dtype in ((part.vertices, '<f8'), (part.faces, '<i8')):
        array = np.asarray(value, dtype=dtype)
        digest.update(str(array.shape).encode())
        digest.update(array.tobytes())
    return digest.hexdigest()


def _geometry_digest(assembly):
    return hashlib.sha256(json.dumps({p.name: _mesh_digest(p) for p in assembly.parts}, sort_keys=True).encode()).hexdigest()


def geometry_integrals(assembly):
    """Density-independent volume, CoM and inertia about CoM, in mm units."""
    result = {}
    cached = assembly.metadata.get('geometry_mass_integrals', {})
    for part in assembly.parts:
        shape = getattr(part, 'cad', None)
        if shape is None and isinstance(getattr(assembly, 'cad', None), dict):
            shape = assembly.cad.get(part.name)
        if shape is not None:
            row = dict(volume_mm3=float(shape.Volume()), com_mm=list(shape.Center().toTuple()),
                       inertia_mm5=shape.matrixOfInertia(shape), source='analytic B-rep volume integrals')
        elif part.name in cached and cached[part.name].get('mesh_sha256') == _mesh_digest(part):
            row = deepcopy(cached[part.name])
        else:
            import trimesh
            mesh = trimesh.Trimesh(part.vertices, part.faces, process=True)
            if not mesh.is_watertight or not mesh.is_winding_consistent or mesh.volume <= 0:
                raise ValueError(f'{part.name}: mass properties require analytic CAD or a closed consistently wound mesh')
            row = dict(volume_mm3=float(mesh.volume), com_mm=mesh.center_mass.tolist(),
                       inertia_mm5=mesh.moment_inertia.tolist(), source='tessellated watertight mesh integrals')
        inertia = np.asarray(row['inertia_mm5'], float)
        if inertia.shape != (3, 3) or not np.isfinite(inertia).all() or np.linalg.eigvalsh(inertia).min() <= 0:
            raise ValueError(f'{part.name}: invalid inertia tensor')
        _finite(row['volume_mm3'], 'volume', True)
        _vec(row['com_mm'], 'center of mass')
        row['mesh_sha256'] = _mesh_digest(part)
        result[part.name] = row
    return result


def _settings(recipe, overrides):
    config = deepcopy((recipe.get('construction') or {}).get('dynamics') or {})
    if (recipe.get('construction') or {}).get('mechanism') and overrides:
        if any(key not in config or value != config[key] for key, value in overrides.items()):
            raise ValueError('Canonical physical settings must be changed in the mechanism source, not backend overrides')
    config.update(deepcopy(overrides or {}))
    return config


def _part_properties(assembly, config):
    integrals = geometry_integrals(assembly)
    densities = config.get('density_kg_m3', {})
    default_density = _finite(config.get('default_density_kg_m3', 2700), 'default density', True)
    result = {}
    for name, row in integrals.items():
        density = _finite(densities.get(name, default_density), f'{name} density', True)
        result[name] = dict(mass=float(row['volume_mm3']) * 1e-9 * density,
                            com=np.array(row['com_mm']) * .001,
                            inertia=np.asarray(row['inertia_mm5']) * density * 1e-15,
                            density_kg_m3=density, source=row['source'])
    payload = config.get('payload')
    if payload and _finite(payload.get('mass_kg', 0), 'payload mass') > 0:
        target = result.get(payload.get('part'))
        if target is None:
            raise ValueError('Payload attachment part is absent')
        mass = float(payload['mass_kg'])
        # Canonical sources retain both component-relative offset and the
        # resolved reference-world center used by world-authored body geometry.
        com = _vec(payload['com_world_mm'] if 'com_world_mm' in payload else payload['com_mm'], 'payload center') * .001
        old_mass, old_com = target['mass'], target['com']
        combined = (old_mass * old_com + mass * com) / (old_mass + mass)
        def shift(weight, delta):
            return weight * (np.dot(delta, delta) * np.eye(3) - np.outer(delta, delta))
        intrinsic = np.asarray(payload.get('inertia_kg_m2', np.zeros((3, 3))), float)
        if intrinsic.shape != (3, 3) or not np.isfinite(intrinsic).all() or np.linalg.eigvalsh(intrinsic).min() < -1e-12:
            raise ValueError('Payload inertia must be a positive semidefinite 3x3 tensor')
        target.update(mass=old_mass + mass, com=combined,
                      inertia=target['inertia'] + intrinsic + shift(old_mass, old_com - combined) + shift(mass, com - combined))
    return result


def _proxy(parent, spec, name, mass=None):
    """Only explicit primitive proxies create contacts; mesh visuals never do."""
    from scipy.spatial.transform import Rotation
    kind = spec.get('kind')
    size = _vec(spec['size'], 'collision proxy size') * .001
    if np.any(size <= 0) or kind not in ('box', 'cylinder', 'sphere'):
        raise ValueError('Collision proxies require positive box/cylinder/sphere dimensions')
    actual = size / 2 if kind == 'box' else ([size[0] / 2, size[2] / 2] if kind == 'cylinder' else [size[0] / 2])
    quat = Rotation.from_euler('xyz', spec.get('rotation', [0, 0, 0]), degrees=True).as_quat()
    attrs = dict(name=name, type=kind, size=_fmt(actual), pos=_fmt(_vec(spec.get('position', [0, 0, 0]), 'proxy position') * .001),
                 quat=_fmt(quat[[3, 0, 1, 2]]), friction=_fmt(spec.get('friction', [.7, .005, .0001])),
                 rgba=_fmt(spec.get('rgba', [.8, .5, .15, .45])), contype='1', conaffinity='1',
                 solref=_fmt(spec.get('solref', [.006, 1])), solimp=_fmt(spec.get('solimp', [.99, .99, .001])))
    if mass is not None:
        attrs['mass'] = str(_finite(mass, 'free body mass', True))
    return ET.SubElement(parent, 'geom', **attrs)


def export_mjcf(assembly, output, *, timestep=.002, settings=None):
    """Export the shared assembly graph without any preset-specific behavior.

    Returns a manifest. Canonical mechanisms own all physical settings in
    their source. For legacy assemblies, `settings` can declare densities,
    payload, actuators, primitive contact proxies and free environment objects;
    these settings are written to the manifest.
    """
    import trimesh
    recipe = _recipe(assembly)
    graph = KinematicGraph([p.name for p in assembly.parts], recipe['joints'], (recipe.get('construction') or {}).get('motion'))
    config = _settings(recipe, settings)
    dt = _finite(timestep, 'timestep', True)
    if not .00005 <= dt <= .01:
        raise ValueError('Dynamics timestep must be within [0.00005, 0.01] seconds')
    requested_dt = dt
    has_drives = any(joint.get('drive') for joint in graph.joints)
    # This is a numerical approximation to the declared ideal transmission,
    # not a measured belt stiffness. Low-inertia motor shafts coupled to heavy
    # outputs need a fine step to avoid MuJoCo's soft-equality regularization
    # dominating the motion. Step-halving convergence is tested separately.
    if has_drives:
        dt = min(dt, .0001)
    root = Path(output)
    root.mkdir(parents=True, exist_ok=True)
    meshes = root / 'meshes'
    meshes.mkdir(exist_ok=True)
    props = _part_properties(assembly, config)
    xml = ET.Element('mujoco', model=assembly.name)
    ET.SubElement(xml, 'compiler', angle='radian', inertiafromgeom='false', fusestatic='false')
    gravity = _vec(config.get('gravity_mm_s2', [0, 0, -9810]), 'gravity') * .001
    option = ET.SubElement(xml, 'option', timestep=str(dt), gravity=_fmt(gravity), integrator='implicitfast', iterations='100', tolerance='1e-10')
    ET.SubElement(option, 'flag', energy='enable')
    asset = ET.SubElement(xml, 'asset')
    world = ET.SubElement(xml, 'worldbody')
    bodies, joint_units = {}, {}
    by_part = {part.name: part for part in assembly.parts}
    actuators = {item['joint']: item for item in config.get('actuators', [])}
    unknown = set(actuators) - set(graph.controls)
    if unknown:
        raise ValueError(f'Actuators must target independent joints: {sorted(unknown)}')
    generated_meshes = []
    for index, name in enumerate(graph.part_order):
        part, joint = by_part[name], graph.by_child.get(name)
        parent = bodies.get(joint.get('parent')) if joint else None
        body = ET.SubElement(world if parent is None else parent, 'body', name=name, pos='0 0 0')
        bodies[name] = body
        physical = props[name]
        inertia = physical['inertia']
        ET.SubElement(body, 'inertial', pos=_fmt(physical['com']), mass=str(physical['mass']),
                      fullinertia=_fmt([inertia[0, 0], inertia[1, 1], inertia[2, 2], inertia[0, 1], inertia[0, 2], inertia[1, 2]]))
        if joint and joint['type'] != 'fixed':
            scale = math.pi / 180 if joint['type'] == 'revolute' else .001
            joint_units[joint['name']] = scale
            spec = actuators.get(joint['name'], {})
            damping = _finite(spec.get('viscous_damping', .01 if joint['type'] == 'revolute' else 1), 'viscous damping')
            if damping < 0:
                raise ValueError('Viscous damping cannot be negative')
            ET.SubElement(body, 'joint', name=joint['name'], type='hinge' if joint['type'] == 'revolute' else 'slide',
                          pos=_fmt(np.asarray(joint['origin']) * .001), axis=_fmt(joint['axis']),
                          limited='true', range=_fmt(np.asarray([joint['min'], joint['max']]) * scale), damping=str(damping))
        mesh_name = f'part_{index:04d}'
        file = f'meshes/{mesh_name}.obj'
        trimesh.Trimesh(part.vertices * .001, part.faces, process=False).export(root / file)
        generated_meshes.append(root / file)
        ET.SubElement(asset, 'mesh', name=mesh_name, file=file)
        material = assembly.materials[part.material]
        ET.SubElement(body, 'geom', name=f'visual_{index:04d}', type='mesh', mesh=mesh_name,
                      contype='0', conaffinity='0', group='2', rgba=_fmt([*material.color, 1]))
    equality = ET.SubElement(xml, 'equality')
    for joint in graph.joints:
        drive = joint.get('drive')
        if drive:
            ratio = drive['ratio'] * joint_units[joint['name']] / joint_units[drive['joint']]
            offset = drive['offset'] * joint_units[joint['name']]
            ET.SubElement(equality, 'joint', joint1=joint['name'], joint2=drive['joint'],
                          polycoef=_fmt([offset, ratio, 0, 0, 0]), solref=_fmt([2 * dt, 1]), solimp='.9999 .9999 .001')
    motor_parent = ET.SubElement(xml, 'actuator')
    motor_limits = {}
    for name in graph.controls:
        joint, spec = graph.by_name[name], actuators.get(name, {})
        key = 'effort_limit_Nmm' if joint['type'] == 'revolute' else 'effort_limit_N'
        if key not in spec:
            raise ValueError(f'{name}: dynamics requires explicit {key}; no motor rating is inferred')
        limit = _finite(spec[key], f'{name} effort limit', True) * (.001 if joint['type'] == 'revolute' else 1)
        motor_limits[name] = limit
        ET.SubElement(motor_parent, 'motor', name=f'motor_{name}', joint=name, gear='1',
                      ctrllimited='true', ctrlrange=_fmt([-limit, limit]))
    contacts = config.get('contacts') or {}
    contact_geoms = list(contacts.get('geoms', []))
    # Canonical component proxies are opt-in. Declaring a primitive does not
    # establish that contacts against all neighboring mechanical parts are valid.
    if contacts.get('enable_component_proxies', False):
        if not config.get('contact_policy', {}).get('enabled', False):
            raise ValueError('Canonical contact policy disables component proxies; explicitly revise the policy before enabling them')
        for item in config.get('collision_proxies', []):
            contact_geoms.extend({**primitive, 'part': item['part']} for primitive in item['primitives'])
    for index, spec in enumerate(contact_geoms):
        if spec.get('part') not in bodies:
            raise ValueError('Collision proxy refers to unknown part')
        _proxy(bodies[spec['part']], spec, f'contact_{index}')
    if contacts.get('ground'):
        ET.SubElement(world, 'geom', name='environment_ground', type='plane', size='2 2 .01',
                      pos=_fmt(_vec(contacts.get('ground_position_mm', [0, 0, 0]), 'ground') * .001),
                      friction=_fmt(contacts.get('friction', config.get('contact_policy', {}).get('friction', [.7, .005, .0001]))))
    free_names = []
    for index, item in enumerate(contacts.get('free_bodies', [])):
        name = item.get('name', f'free_payload_{index}')
        if name in bodies or name in free_names:
            raise ValueError('Free environment body name must be unique')
        free_names.append(name)
        body = ET.SubElement(world, 'body', name=name, pos=_fmt(_vec(item.get('position_mm', [0, 0, 100]), 'free body position') * .001))
        ET.SubElement(body, 'freejoint', name=f'free_{name}')
        # Free environment inertia uses the declared analytic primitive and
        # mass, unlike the CAD volume integrals used for constructed bodies.
        geom = _proxy(body, item['shape'], f'environment_{index}', mass=item['mass_kg'])
        shape = item['shape']
        size = np.asarray(shape['size']) * .001
        mass = float(item['mass_kg'])
        if shape['kind'] == 'box':
            diagonal = mass / 12 * np.array([size[1] ** 2 + size[2] ** 2, size[0] ** 2 + size[2] ** 2, size[0] ** 2 + size[1] ** 2])
        elif shape['kind'] == 'sphere':
            diagonal = np.full(3, .4 * mass * (size[0] / 2) ** 2)
        else:
            radius, height = size[0] / 2, size[2]
            diagonal = mass * np.array([(3 * radius ** 2 + height ** 2) / 12] * 2 + [radius ** 2 / 2])
        ET.SubElement(body, 'inertial', pos=geom.get('pos'), quat=geom.get('quat'), mass=str(mass), diaginertia=_fmt(diagonal))
    ET.indent(xml)
    model_path = root / 'model.xml'
    ET.ElementTree(xml).write(model_path, encoding='utf-8', xml_declaration=True)
    manifest = dict(schema='cybrgeo.mujoco/1', model='model.xml', units='SI (m, kg, s, rad, N, Nm)',
                    recipe_sha256=(recipe.get('construction') or {}).get('recipe_sha256'), settings=config,
                    mechanism_sha256=(recipe.get('construction') or {}).get('mechanism_sha256'),
                    model_sha256=(recipe.get('construction') or {}).get('model_sha256'),
                    geometry_sha256=_geometry_digest(assembly),
                    solver_settings=dict(requested_timestep_s=requested_dt, effective_timestep_s=dt,
                                         driven_constraint_max_timestep_s=.0001,
                                         equality_time_constant_s=2 * dt, equality_impedance=.9999,
                                         reason='Resolve ideal-drive equality regularization for small rotor / large load inertia; not physical belt elasticity'),
                    controls=list(graph.controls), motor_limits_SI=motor_limits,
                    joint_native_to_SI=joint_units, free_bodies=free_names,
                    mass_properties={name: {**p, 'com': p['com'].tolist(), 'inertia': p['inertia'].tolist()} for name, p in props.items()},
                    contact_policy='explicit primitive proxies and environment only; all visual CAD mesh contacts disabled',
                    scope='Rigid bodies, gravity, joint constraints and ideal effort-limited actuators; no hardware qualification',
                    files_sha256={str(path.relative_to(root)).replace('\\', '/'): hashlib.sha256(path.read_bytes()).hexdigest()
                                  for path in [model_path, *generated_meshes]})
    (root / 'model.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest


def load_model(directory):
    """Reopen MJCF from disk after checking every declared asset hash."""
    mj = _mujoco()
    root = Path(directory).resolve()
    info = json.loads((root / 'model.json').read_text())
    model_path = (root / info['model']).resolve()
    if not model_path.is_relative_to(root) or info['model'] not in info['files_sha256']:
        raise ValueError('MuJoCo model must be a declared local artifact')
    for relative, digest in info['files_sha256'].items():
        file = (root / relative).resolve()
        if not file.is_relative_to(root) or not file.is_file() or hashlib.sha256(file.read_bytes()).hexdigest() != digest:
            raise ValueError(f'MuJoCo asset hash mismatch: {relative}')
    return mj.MjModel.from_xml_path(str(model_path)), info


def _mapping(model, graph):
    """Reduced independent coordinates -> all hinge/slide velocities."""
    mj = _mujoco()
    controls = list(graph.controls)
    matrix = np.zeros((model.nv, len(controls)))
    columns = {}
    factors = {j['name']: math.pi / 180 if j['type'] == 'revolute' else .001 for j in graph.joints}
    for name in graph.drive_order:
        joint = graph.by_name[name]
        if joint['type'] == 'fixed':
            continue
        drive = joint.get('drive')
        column = np.zeros(len(controls))
        if drive:
            column = columns[drive['joint']] * drive['ratio'] * factors[name] / factors[drive['joint']]
        else:
            column[controls.index(name)] = 1
        columns[name] = column
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
        matrix[model.jnt_dofadr[jid]] = column
    return matrix, factors


def simulate(assembly, *, seconds=8, timestep=.002, output_hz=20, effort_scale=1,
             settings=None, output=None, controller=True):
    """Integrate MuJoCo equations, with saturated computed-torque tracking.

    Requested demo motion is a command only. Recorded body poses come from
    mjData after integration, including gravity, inertia, coupling, constraints
    and declared contacts. The controller knows the nominal model exactly.
    `effort_scale` is an explicit experimental control-cap perturbation; it
    does not change the canonical actuator rating in the model manifest.
    """
    mj = _mujoco()
    seconds = _finite(seconds, 'simulation duration', True)
    output_hz = _finite(output_hz, 'output frequency', True)
    effort_scale = _finite(effort_scale, 'effort scale')
    if seconds > 60 or output_hz > 200 or not 0 <= effort_scale <= 10:
        raise ValueError('Simulation bounds: duration<=60s, output<=200Hz, effort scale in [0,10]')
    if output is None:
        with tempfile.TemporaryDirectory(prefix='cybrgeo-mj-') as directory:
            return simulate(assembly, seconds=seconds, timestep=timestep, output_hz=output_hz,
                            effort_scale=effort_scale, settings=settings, output=directory, controller=controller)
    manifest = export_mjcf(assembly, output, timestep=timestep, settings=settings)
    model, reopened = load_model(output)
    recipe = _recipe(assembly)
    graph = KinematicGraph([p.name for p in assembly.parts], recipe['joints'], (recipe.get('construction') or {}).get('motion'))
    data = mj.MjData(model)
    mapping, factors = _mapping(model, graph)
    ids = {name: mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name) for name in factors if graph.by_name[name]['type'] != 'fixed'}
    qadr = {name: int(model.jnt_qposadr[jid]) for name, jid in ids.items()}
    vadr = {name: int(model.jnt_dofadr[jid]) for name, jid in ids.items()}
    controls = list(graph.controls)
    for name, value in graph.resolve(graph.demo_values(0)).items():
        if name in qadr:
            data.qpos[qadr[name]] = value * factors[name]
    for index, item in enumerate(manifest['settings'].get('contacts', {}).get('free_bodies', [])):
        name = item.get('name', f'free_payload_{index}')
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, f'free_{name}')
        start = int(model.jnt_dofadr[jid])
        data.qvel[start:start + 3] = _vec(item.get('velocity_mm_s', [0, 0, 0]), 'free linear velocity') * .001
        data.qvel[start + 3:start + 6] = _vec(item.get('angular_velocity_rad_s', [0, 0, 0]), 'free angular velocity')
    mj.mj_forward(model, data)
    scale = np.array([factors[name] for name in controls])
    task = graph.motion_task
    peak_rates = task.peak_rates()
    actuator_specs = {item['joint']: item for item in manifest['settings'].get('actuators', [])}
    for index, name in enumerate(controls):
        unit = 'deg' if graph.by_name[name]['type'] == 'revolute' else 'mm'
        spec = actuator_specs[name]
        speed = peak_rates[name]['speed']
        accel = peak_rates[name]['acceleration']
        for actual, field in ((speed, f'max_speed_{unit}_s'), (accel, f'max_acceleration_{unit}_s2')):
            if field in spec and actual > _finite(spec[field], field, True) + 1e-12:
                raise ValueError(f'{name}: source motion period {task.period_s:g}s exceeds {field}; lengthen the source motion period')
    limits = np.array([manifest['motor_limits_SI'][name] for name in controls]) * effort_scale
    if len(controls):
        model.actuator_ctrlrange[:] = np.stack([-limits, limits], axis=1)
    mass = np.zeros((model.nv, model.nv))
    body_ids = {p.name: mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, p.name) for p in assembly.parts}
    all_bodies = {**body_ids, **{name: mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, name) for name in manifest['free_bodies']}}
    samples, saturation = [], np.zeros(len(controls), dtype=int)
    peak_contacts, limit_violation, drive_error = 0, 0., 0.
    per_joint_limit_error = {name: 0. for name in ids}
    per_drive_error = {name: 0. for name in ids if graph.by_name[name].get('drive')}
    steps = int(math.ceil(seconds / model.opt.timestep))
    actual_dt = seconds / steps
    model.opt.timestep = actual_dt
    next_sample = 0.
    max_error = np.zeros(len(controls))
    integration_trace = dict(joint_names=list(ids), control_names=controls, times_s=[], joint_values=[],
                             requested_control_values=[], sampling='every_integration_step_including_initial_and_final')
    sum_error = np.zeros(len(controls))
    peak_effort = np.zeros(len(controls))
    peak_speed = np.zeros(len(controls))
    peak_acceleration = np.zeros(len(controls))
    applied = np.zeros(len(controls))
    config = manifest['settings']
    bandwidth = _finite(config.get('controller_bandwidth_rad_s', 8), 'controller bandwidth', True)
    if bandwidth > 40:
        raise ValueError('Controller bandwidth exceeds 40 rad/s')
    for step in range(steps + 1):
        time = step * actual_dt
        command = task.sample(time)
        target = np.array([command['values'][name] for name in controls]) * scale
        target_velocity = np.array([command['velocities'][name] for name in controls]) * scale
        target_accel = np.array([command['accelerations'][name] for name in controls]) * scale
        integration_trace['times_s'].append(time)
        integration_trace['joint_values'].append([float(data.qpos[qadr[name]] / factors[name]) for name in ids])
        integration_trace['requested_control_values'].append((target / scale).tolist())
        current = np.array([data.qpos[qadr[name]] for name in controls])
        velocity = np.array([data.qvel[vadr[name]] for name in controls])
        acceleration_now = np.array([data.qacc[vadr[name]] for name in controls])
        if len(controls):
            peak_speed = np.maximum(peak_speed, np.abs(velocity) / scale)
            # At initialization qacc still represents zero applied effort, so
            # count acceleration only after the controller's first real step.
            if step:
                peak_acceleration = np.maximum(peak_acceleration, np.abs(acceleration_now) / scale)
        error = np.abs(target - current) / scale if len(controls) else np.array([])
        max_error = np.maximum(max_error, error)
        sum_error += error ** 2
        peak_contacts = max(peak_contacts, data.ncon)
        for name, jid in ids.items():
            value = data.qpos[qadr[name]] / factors[name]
            joint = graph.by_name[name]
            per_joint_limit_error[name] = max(per_joint_limit_error[name], joint['min'] - value, value - joint['max'])
            limit_violation = max(limit_violation, per_joint_limit_error[name])
            drive = joint.get('drive')
            if drive:
                expected = data.qpos[qadr[drive['joint']]] / factors[drive['joint']] * drive['ratio'] + drive['offset']
                per_drive_error[name] = max(per_drive_error[name], abs(value - expected))
                drive_error = max(drive_error, per_drive_error[name])
        if step == steps or time >= next_sample - 1e-10:
            poses = {}
            for name, bid in all_bodies.items():
                transform = np.eye(4)
                transform[:3, :3] = data.xmat[bid].reshape(3, 3)
                transform[:3, 3] = data.xpos[bid] * 1000
                poses[name] = transform.tolist()
            contact_force = np.zeros(6)
            normal_force = 0.
            for contact_id in range(data.ncon):
                mj.mj_contactForce(model, data, contact_id, contact_force)
                normal_force += abs(contact_force[0])
            samples.append(dict(time_s=time, requested_values=dict(zip(controls, (target / scale).tolist())),
                                simulated_values=dict(zip(controls, (current / scale).tolist())),
                                joint_values={name: float(data.qpos[qadr[name]] / factors[name]) for name in ids},
                                applied_efforts_SI=dict(zip(controls, applied.tolist())), body_poses_mm=poses,
                                energy_J=data.energy.tolist(), contacts=int(data.ncon),
                                summed_normal_contact_force_N=normal_force))
            next_sample += 1 / output_hz
        if step == steps:
            break
        if controller and len(controls):
            mj.mj_fullM(model, data, mass)
            reduced_mass = mapping.T @ mass @ mapping
            acceleration = target_accel + bandwidth ** 2 * (target - current) + 2 * bandwidth * (target_velocity - velocity)
            demand = reduced_mass @ acceleration + mapping.T @ (data.qfrc_bias - data.qfrc_passive)
            applied = np.clip(demand, -limits, limits)
            saturation += np.abs(demand) > limits + 1e-12
        else:
            applied = np.zeros(len(controls))
        if len(controls):
            data.ctrl[:] = applied
        peak_effort = np.maximum(peak_effort, np.abs(applied))
        mj.mj_step(model, data)
        mj.mj_forward(model, data)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all() or any(w.number for w in data.warning):
            raise RuntimeError(f'MuJoCo numerical warning or nonfinite state at {time:.6f}s')
    metrics = {name: dict(unit='deg' if graph.by_name[name]['type'] == 'revolute' else 'mm',
                         max_tracking_error=float(max_error[i]), rms_tracking_error=float(math.sqrt(sum_error[i] / (steps + 1))),
                         peak_applied_effort_SI=float(peak_effort[i]), effort_limit_SI=float(limits[i]),
                         declared_effort_limit_SI=float(manifest['motor_limits_SI'][name]),
                         max_requested_speed=peak_rates[name]['speed'],
                         max_requested_acceleration=peak_rates[name]['acceleration'],
                         max_simulated_speed=float(peak_speed[i]), max_simulated_acceleration=float(peak_acceleration[i]),
                         saturation_fraction=float(saturation[i] / steps)) for i, name in enumerate(controls)}
    thresholds = np.array([config.get('tracking_tolerance_deg', 1) if graph.by_name[name]['type'] == 'revolute'
                           else config.get('tracking_tolerance_mm', .5) for name in controls])
    tracking_pass = bool(np.all(max_error <= thresholds))
    operating_limits = True
    for i, name in enumerate(controls):
        if peak_effort[i] > manifest['motor_limits_SI'][name] + 1e-9:
            operating_limits = False
        unit = 'deg' if graph.by_name[name]['type'] == 'revolute' else 'mm'
        spec = actuator_specs[name]
        for value, key in ((peak_speed[i], f'max_speed_{unit}_s'), (peak_acceleration[i], f'max_acceleration_{unit}_s2')):
            if key in spec and value > spec[key] + 1e-9:
                operating_limits = False
    result = dict(schema='cybrgeo.simulation/1', backend='MuJoCo', backend_version=mj.__version__,
                  recipe_sha256=manifest['recipe_sha256'], seconds=seconds, timestep_s=actual_dt,
                  mechanism_sha256=manifest['mechanism_sha256'], model_sha256=manifest['model_sha256'],
                  geometry_sha256=manifest['geometry_sha256'],
                  motion_task=task.to_dict(),
                  integration_steps=steps, output_hz=output_hz, controls=controls, metrics=metrics,
                  tracking_passed=tracking_pass, maximum_joint_limit_violation_native=limit_violation,
                  operating_limits_passed=bool(operating_limits),
                  run_settings=dict(controller_enabled=bool(controller), effort_scale=effort_scale,
                                    effort_scale_scope='Experimental controller effort cap multiplier, not a revised canonical actuator rating',
                                    controller_bandwidth_rad_s=bandwidth, timestep_s=actual_dt),
                  maximum_drive_constraint_error_native=drive_error, maximum_contacts=peak_contacts,
                  joint_limit_violations_native=per_joint_limit_error, drive_constraint_errors_native=per_drive_error,
                  contact_policy=manifest['contact_policy'], model=manifest, samples=samples,
                  integration_state_trace=integration_trace,
                  assumptions=['Fixed-base rigid bodies with exact CAD/declared mesh mass integrals and declared densities.',
                               'Ideal output motors with symmetric effort limits; no electrical, thermal, backlash or flexibility model.',
                               'Computed-torque PD uses the exact nominal MuJoCo mass/bias model; disturbance robustness is not established.',
                               'Driven joints use a stiff finite-compliance MuJoCo equality; constraint error is reported.',
                               'Payload without an explicit inertia tensor is a point mass at its declared center.',
                               'Sampled body poses are actual integrated states; replay interpolates stored poses, not a contact certificate.',
                               'No hardware qualification, vendor rating, or collision-free operation is established by tracking success.'])
    if effort_scale != 1:
        result['assumptions'].append(
            f'Experimental effort_scale={effort_scale:g} changes the applied controller cap only; '
            'the declared source actuator limits remain unchanged and are still checked.')
    trace_bytes = gzip.compress(json.dumps(integration_trace, separators=(',', ':')).encode(), mtime=0)
    trace_path = Path(output) / 'integration-state-trace.json.gz'
    trace_path.write_bytes(trace_bytes)
    result['integration_trace_artifact'] = dict(path=trace_path.name, sha256=hashlib.sha256(trace_bytes).hexdigest(),
                                               states=len(integration_trace['times_s']), sampling=integration_trace['sampling'])
    saved = {key: value for key, value in result.items() if key != 'integration_state_trace'}
    (Path(output) / 'simulation.json').write_text(json.dumps(saved, separators=(',', ':')) + '\n')
    return result


class SimulatedMotion:
    def __init__(self, result):
        self.samples = result['samples']
        self.times = np.array([sample['time_s'] for sample in self.samples])

    def __call__(self, part, time_seconds=0, explode=0):
        from scipy.spatial.transform import Rotation, Slerp
        t = float(np.clip(time_seconds, self.times[0], self.times[-1]))
        index = min(max(int(np.searchsorted(self.times, t, side='right')) - 1, 0), len(self.times) - 2)
        a, b = [np.asarray(self.samples[i]['body_poses_mm'][part.name]) for i in (index, index + 1)]
        weight = (t - self.times[index]) / (self.times[index + 1] - self.times[index])
        result = np.eye(4)
        result[:3, :3] = Slerp([0, 1], Rotation.from_matrix(np.stack([a[:3, :3], b[:3, :3]])))(weight).as_matrix()
        result[:3, 3] = (1 - weight) * a[:3, 3] + weight * b[:3, 3] + np.asarray(part.explode) * explode
        return result


def simulated_assembly(assembly, result):
    recipe = _recipe(assembly)
    expected = (recipe.get('construction') or {}).get('recipe_sha256')
    if result.get('recipe_sha256') != expected or result.get('backend') != 'MuJoCo':
        raise ValueError('Simulation does not belong to this mechanical recipe')
    for field in ('model_sha256', 'mechanism_sha256'):
        if result.get(field) != (recipe.get('construction') or {}).get(field):
            raise ValueError('Simulation does not belong to this mechanical source/model')
    if result.get('geometry_sha256') != _geometry_digest(assembly):
        raise ValueError('Simulation geometry does not match the replay assembly')
    if not hasattr(assembly, 'motion_function'):
        raise TypeError('Use the GEOAssembly-to-Lab adapter before producing animated artifacts')
    return replace(assembly, motion_function=SimulatedMotion(result),
                   metadata={**assembly.metadata, 'motion_source': 'MuJoCo integrated rigid-body dynamics'})


def plot_simulation(result, output):
    """Standalone SVG curves from integrated records; no plotting dependency."""
    controls = result['controls']
    if not controls:
        raise ValueError('Tracking plot requires at least one controlled coordinate')
    width, height = 1200, 115 + 180 * len(controls)
    svg = ET.Element('svg', xmlns='http://www.w3.org/2000/svg', width=str(width), height=str(height), viewBox=f'0 0 {width} {height}')
    ET.SubElement(svg, 'rect', width='100%', height='100%', fill='#f7f8fa')
    def label(x, y, text, size=13, color='#263344'):
        ET.SubElement(svg, 'text', x=str(x), y=str(y), fill=color, **{'font-size': str(size), 'font-family': 'sans-serif'}).text = text
    label(35, 30, f'MuJoCo {result["backend_version"]} · integrated joint motion and actuator effort', 22)
    label(35, 55, 'Nominal rigid-body model; no hardware qualification. Dashed: requested. Blue: integrated. Orange: applied effort.')
    times = np.array([s['time_s'] for s in result['samples']])
    for index, name in enumerate(controls):
        unit = result['metrics'][name]['unit']
        requested = np.array([s['requested_values'][name] for s in result['samples']])
        integrated = np.array([s['simulated_values'][name] for s in result['samples']])
        effort = np.array([s['applied_efforts_SI'][name] for s in result['samples']])
        limit = result['metrics'][name]['effort_limit_SI']
        for col, curves in enumerate(([(requested, '#778595', '6 4'), (integrated, '#1664b4', None)],
                                      [(effort, '#b45222', None), (np.full_like(times, limit), '#929ca8', '3 4'),
                                       (np.full_like(times, -limit), '#929ca8', '3 4')])):
            x, y, w, h = 80 + 590 * col, 105 + 180 * index, 480, 115
            low = min(float(np.min(values)) for values, _, _ in curves)
            high = max(float(np.max(values)) for values, _, _ in curves)
            pad = max((high - low) * .08, 1e-5)
            low, high = low - pad, high + pad
            label(x, y - 12, f'{name} ({unit})' if col == 0 else f'Applied effort ({"Nm" if unit == "deg" else "N"}); limits ±{limit:.3g}')
            for frac in (0, .5, 1):
                yy = y + frac * h
                ET.SubElement(svg, 'line', x1=str(x), x2=str(x + w), y1=str(yy), y2=str(yy), stroke='#d5dbe2')
                label(x - 65, yy + 4, f'{high - frac * (high - low):.3g}', 11)
            for values, color, dash in curves:
                xs = x + (times - times[0]) / max(times[-1] - times[0], 1e-10) * w
                ys = y + (high - values) / (high - low) * h
                attrs = dict(points=' '.join(f'{xx:.2f},{yy:.2f}' for xx, yy in zip(xs, ys)), fill='none', stroke=color)
                attrs['stroke-width'] = '1.8'
                if dash:
                    attrs['stroke-dasharray'] = dash
                ET.SubElement(svg, 'polyline', **attrs)
            label(x, y + h + 20, f'{times[0]:g} s', 11)
            label(x + w - 30, y + h + 20, f'{times[-1]:g} s', 11)
    output = Path(output)
    ET.indent(svg)
    content = ET.tostring(svg, encoding='unicode')
    if output.suffix.lower() == '.png':
        import cairosvg
        cairosvg.svg2png(bytestring=content.encode(), write_to=str(output))
    elif output.suffix.lower() == '.svg':
        output.write_text(content, encoding='utf-8')
    else:
        raise ValueError('Tracking plots support .svg or .png output')
    return output
