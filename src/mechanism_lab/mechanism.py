"""Canonical mechanical source, independent of CAD and simulation backends.

This is a deliberately bounded component catalog, not a general constraint
solver. Unsupported topology/frames fail before geometry is constructed. New
components extend this one source contract; examples contain no executable code.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import re

from .component_catalog import component_definition
from .motion import DEFAULT_SOURCE as DEFAULT_MOTION, validate_motion_source
from pathlib import Path

SCHEMA = 'cybrgeo-mechanism/2'
EXAMPLES = Path(__file__).with_name('mechanisms')
MATERIAL_ROLES = ('frame', 'shaft', 'bushing', 'motor', 'belt', 'pad')


def _canonical(value):
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return value
    if isinstance(value, (float, int)):
        return format(0. if abs(value) < .5e-9 else value, '.9f')
    if isinstance(value, dict):
        return {key: _canonical(value[key]) for key in sorted(value)}
    return [_canonical(item) for item in value]


def mechanism_digest(source):
    """Bind the complete physical source, including assumptions; not a certificate."""
    data = json.dumps(_canonical(source), ensure_ascii=False, separators=(',', ':'))
    return hashlib.sha256(data.encode()).hexdigest()


def _object(value, required, optional=(), where='source'):
    if not isinstance(value, dict):
        raise ValueError(f'{where} must be an object')
    missing, extra = set(required)-set(value), set(value)-set(required)-set(optional)
    if missing or extra:
        raise ValueError(f'{where}: missing fields {sorted(missing)}, unknown fields {sorted(extra)}')


def _number(value, lo, hi, where):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lo <= value <= hi:
        raise ValueError(f'{where} must be finite within [{lo}, {hi}]')
    return float(value)


def _vector(value, where, limit=10000):
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f'{where} must be a three-element vector')
    return [_number(v, -limit, limit, where) for v in value]


def load_example(name='compact_arm'):
    """Load a data-only example by portable basename (never an arbitrary path)."""
    if not isinstance(name, str) or not re.fullmatch(r'[a-z][a-z0-9_]{0,63}', name):
        raise ValueError(f'Unknown architecture {name!r}')
    path = EXAMPLES / (name + '.json')
    if not path.is_file():
        raise ValueError(f'Unknown architecture {name!r}; no external example descriptor')
    return json.loads(path.read_text(encoding='utf-8'))


def validate_mechanism(source):
    """Return an isolated JSON source after enforcing the supported design domain.

    Component frames are reference-pose world coordinates in millimetres. Scale
    changes reusable hardware dimensions only; locations remain explicit. A
    compatibility loader scales both when adapting the old uniform-scale input.
    """
    from .construction import register_builtin_components
    register_builtin_components()
    try:
        json.dumps(source, allow_nan=False)
    except (TypeError, ValueError) as exc:
        raise ValueError('Mechanism source must contain finite JSON data only') from exc
    _object(source, ('schema', 'name', 'units', 'provenance', 'dimensions', 'materials',
                     'material_roles', 'tolerances', 'components', 'actuators', 'loads',
                     'contacts', 'manufacturing'), ('motion',))
    if source['schema'] != SCHEMA or source['units'] != 'mm':
        raise ValueError(f'Expected {SCHEMA} with millimetre units')
    for key in ('name', 'provenance'):
        if not isinstance(source[key], str) or not source[key].strip():
            raise ValueError(f'{key} must be a nonempty string')
    _object(source['dimensions'], ('hardware_scale',), where='dimensions')
    scale = _number(source['dimensions']['hardware_scale'], .5, 2., 'hardware_scale')
    if not isinstance(source['materials'], dict) or not source['materials']:
        raise ValueError('materials must contain explicit physical material assumptions')
    for name, material in source['materials'].items():
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]{0,63}', name):
            raise ValueError('Material IDs must be portable identifiers')
        _object(material, ('density_kg_m3', 'provenance'), ('allowable_shear_MPa',), f'materials.{name}')
        _number(material['density_kg_m3'], 100, 23000, f'{name}.density_kg_m3')
        if 'allowable_shear_MPa' in material:
            _number(material['allowable_shear_MPa'], 1, 1000, f'{name}.allowable_shear_MPa')
        if not isinstance(material['provenance'], str) or not material['provenance']:
            raise ValueError(f'{name}: material provenance is required')
    _object(source['material_roles'], MATERIAL_ROLES, where='material_roles')
    if not all(isinstance(name, str) and name in source['materials'] for name in source['material_roles'].values()):
        raise ValueError('Every material role must reference an explicit material')
    if 'allowable_shear_MPa' not in source['materials'][source['material_roles']['shaft']]:
        raise ValueError('Shaft material requires an explicit nominal allowable_shear_MPa assumption')
    _object(source['tolerances'], ('journal', 'guide'), where='tolerances')
    for name, fit in source['tolerances'].items():
        _object(fit, ('radial_clearance_mm', 'shaft_radius_tolerance_mm',
                      'bore_radius_tolerance_mm', 'minimum_clearance_mm'), where=f'tolerances.{name}')
        clearance = _number(fit['radial_clearance_mm'], .12, .4, f'{name}.radial_clearance_mm')
        shaft = _number(fit['shaft_radius_tolerance_mm'], 0, .1, f'{name}.shaft_radius_tolerance_mm')
        bore = _number(fit['bore_radius_tolerance_mm'], 0, .1, f'{name}.bore_radius_tolerance_mm')
        required = _number(fit['minimum_clearance_mm'], .001, .3, f'{name}.minimum_clearance_mm')
        if clearance - shaft - bore < required:
            raise ValueError(f'{name} tolerance stack closes the running clearance: {clearance-shaft-bore:g} mm < {required:g} mm')
    components = source['components']
    if not isinstance(components, list) or not 1 <= len(components) <= 9:
        raise ValueError('components requires 1 to 9 reusable instances')
    ids, ordered, pending = set(), [], list(components)
    for component in components:
        _object(component, ('id', 'type', 'parent', 'parameters', 'drive'), where='component')
        name = component['id']
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]{0,39}', name) or name in ids:
            raise ValueError('Component IDs must be unique portable identifiers')
        ids.add(name)
        if component['parent'] is not None and not isinstance(component['parent'], str):
            raise ValueError('Component parent must be a component ID or null')
        component_definition(component['type'])
    roots = [c for c in components if c['parent'] is None]
    if len(roots) != 1 or not component_definition(roots[0]['type'])['root_allowed']:
        raise ValueError('Exactly one component with a supported grounded interface is required')
    while pending:
        ready = [c for c in pending if c['parent'] is None or c['parent'] in {v['id'] for v in ordered}]
        if not ready:
            raise ValueError('Component parent graph contains a cycle or missing parent')
        for component in ready:
            ordered.append(component)
            pending.remove(component)
    children = {name: [c for c in components if c['parent'] == name] for name in ids}
    used_actuators = []
    if not isinstance(source['actuators'], dict):
        raise ValueError('actuators must be an object of named design assumptions')
    by_id = {c['id']: c for c in components}
    context = dict(scale=scale, by_id=by_id, children=children, source=source)
    for component in ordered:
        used_actuators.extend(component_definition(component['type'])['validator'](component, context))
    names = [name for name, _ in used_actuators]
    if any(not isinstance(name, str) for name in names):
        raise ValueError('Actuator references must be string IDs')
    if len(set(names)) != len(names) or set(names) != set(source['actuators']):
        raise ValueError('Every independent actuator must be referenced exactly once')
    for name, kind in used_actuators:
        actuator = source['actuators'][name]
        effort = 'effort_limit_Nmm' if kind == 'revolute' else 'effort_limit_N'
        speed = 'max_speed_deg_s' if kind == 'revolute' else 'max_speed_mm_s'
        acceleration = 'max_acceleration_deg_s2' if kind == 'revolute' else 'max_acceleration_mm_s2'
        _object(actuator, (effort, speed, acceleration, 'provenance'), where='actuators.'+name)
        _number(actuator[effort], .01, 1e6 if kind == 'revolute' else 1e4, name+'.'+effort)
        _number(actuator[speed], .01, 90 if kind == 'revolute' else 100, name+'.'+speed)
        _number(actuator[acceleration], .01, 360 if kind == 'revolute' else 1000, name+'.'+acceleration)
        if not isinstance(actuator['provenance'], str) or not actuator['provenance']:
            raise ValueError('Actuator provenance must distinguish assumptions from rated vendor hardware')
    _object(source['loads'], ('gravity_mm_s2', 'payload', 'duty_cycle'), where='loads')
    _vector(source['loads']['gravity_mm_s2'], 'gravity_mm_s2', 20000)
    _number(source['loads']['duty_cycle'], .001, 1, 'duty_cycle')
    payload = source['loads']['payload']
    _object(payload, ('component', 'mass_kg', 'com_offset_mm'), where='payload')
    if payload['component'] not in ids:
        raise ValueError('Payload must attach to an existing component output')
    _number(payload['mass_kg'], 0, 10, 'payload.mass_kg')
    _vector(payload['com_offset_mm'], 'payload.com_offset_mm', 500)
    _object(source['contacts'], ('model', 'friction', 'enable_component_proxies', 'ground',
                                'ground_position_mm', 'geoms', 'free_bodies'), where='contacts')
    if source['contacts']['model'] != 'component_safe_primitives':
        raise ValueError('Only component_safe_primitives contact is supported; no hull-filled bores')
    friction = source['contacts']['friction']
    if not isinstance(friction, list) or len(friction) != 3:
        raise ValueError('Contact friction requires sliding, torsional, rolling coefficients')
    for value in friction:
        _number(value, 0, 2, 'contacts.friction')
    for key in ('enable_component_proxies', 'ground'):
        if not isinstance(source['contacts'][key], bool):
            raise ValueError(f'contacts.{key} must be boolean')
    _vector(source['contacts']['ground_position_mm'], 'contacts.ground_position_mm')
    if source['contacts']['geoms']:
        raise ValueError('Authored part collision geoms require material-containment validation; use component-owned proxies')
    if not isinstance(source['contacts']['geoms'], list):
        raise ValueError('contacts.geoms must be a list')
    free_bodies = source['contacts']['free_bodies']
    if not isinstance(free_bodies, list) or len(free_bodies) > 16:
        raise ValueError('contacts.free_bodies requires at most 16 environment bodies')
    body_names = set()
    for body in free_bodies:
        _object(body, ('name', 'position_mm', 'velocity_mm_s', 'angular_velocity_rad_s', 'mass_kg', 'shape'), where='free_body')
        if not isinstance(body['name'], str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]{0,39}', body['name']) or body['name'] in body_names:
            raise ValueError('Free body names must be unique portable identifiers')
        body_names.add(body['name'])
        for key in ('position_mm', 'velocity_mm_s', 'angular_velocity_rad_s'):
            _vector(body[key], 'free_body.'+key, 100 if key == 'angular_velocity_rad_s' else 10000)
        _number(body['mass_kg'], .00001, 100, 'free_body.mass_kg')
        shape = body['shape']
        _object(shape, ('kind', 'size', 'position', 'rotation'), ('friction',), 'free_body.shape')
        if shape['kind'] not in ('box', 'sphere', 'cylinder'):
            raise ValueError('Free body collision shape must be an analytic primitive')
        size = _vector(shape['size'], 'free_body.shape.size', 2000)
        if min(size) <= 0 or (shape['kind'] == 'sphere' and len(set(size)) != 1) or (shape['kind'] == 'cylinder' and size[0] != size[1]):
            raise ValueError('Free body dimensions must be positive and match primitive symmetry')
        _vector(shape['position'], 'free_body.shape.position')
        _vector(shape['rotation'], 'free_body.shape.rotation', 360)
        if 'friction' in shape:
            if not isinstance(shape['friction'], list) or len(shape['friction']) != 3:
                raise ValueError('Free body friction requires three values')
            for value in shape['friction']:
                _number(value, 0, 2, 'free_body.shape.friction')
    _object(source['manufacturing'], ('process', 'threads', 'qualification'), where='manufacturing')
    if any(not isinstance(v, str) or not v for v in source['manufacturing'].values()):
        raise ValueError('Manufacturing assumptions must be nonempty strings')
    result = copy.deepcopy(source)
    result['components'] = copy.deepcopy(ordered)
    result['motion'] = validate_motion_source(source.get('motion', DEFAULT_MOTION), source['actuators'])
    return result


def validate_rotary_component(component, context):
    name, parameters = component['id'], component['parameters']
    scale, by_id, children = context['scale'], context['by_id'], context['children']
    if len(children[name]) > 1:
        raise ValueError('rotary_cartridge supports a serial chain; branching requires a branch interface component')
    _object(parameters, ('origin_mm', 'axis', 'limit_deg'), where=name+'.parameters')
    origin = _vector(parameters['origin_mm'], name+'.origin_mm')
    axis = _vector(parameters['axis'], name+'.axis', 1)
    if axis not in ([1., 0., 0.], [0., 1., 0.], [0., 0., 1.]):
        raise ValueError('rotary_cartridge supports positive cardinal axes only')
    _number(parameters['limit_deg'], 1, 18, name+'.limit_deg')
    if component['parent'] is None:
        if axis != [0., 0., 1.] or origin[:2] != [0., 0.] or not 65*scale <= origin[2] <= 140*scale:
            raise ValueError('Root stand requires +Z axis, x=y=0 and height 65..140 times hardware_scale')
    elif by_id[component['parent']]['type'] != 'rotary_cartridge':
        raise ValueError('Rotary cartridge input requires a rotary cartridge output flange')
    else:
        previous = by_id[component['parent']]['parameters']['origin_mm']
        if math.dist(origin, previous) < 70*scale:
            raise ValueError('Connected cartridge datums must be separated by at least 70 times hardware_scale mm')
    _object(component['drive'], ('type', 'actuator'), where=name+'.drive')
    if component['drive']['type'] not in ('direct', 'equal_pulley_belt'):
        raise ValueError('Unsupported rotary transmission; no motion-only substitute is generated')
    return [(component['drive']['actuator'], 'revolute')]


def validate_gripper_component(component, context):
    name, parameters = component['id'], component['parameters']
    scale, by_id, children = context['scale'], context['by_id'], context['children']
    _object(parameters, ('travel_mm',), where=name+'.parameters')
    _number(parameters['travel_mm'], 2, min(14, 14*scale), name+'.travel_mm')
    parent = by_id.get(component['parent'])
    if not parent or parent['type'] != 'rotary_cartridge' or parent['parameters']['axis'] != [1, 0, 0] or children[name]:
        raise ValueError('parallel_gripper must terminate a +X rotary cartridge output')
    _object(component['drive'], ('type', 'left', 'right'), where=name+'.drive')
    if component['drive']['type'] != 'independent_linear':
        raise ValueError('parallel_gripper supports independent linear actuators; no fictional screw relation')
    return [(component['drive'][side], 'prismatic') for side in ('left', 'right')]
