"""Lower constructive plans to analytic CAD and reusable, cache-safe kinematics.

The input is authored in millimetres, Z-up, in the reference world frame.
Geometry, joints and mating datums come from ``construction.compile_plan``.
This module performs prescribed rigid-body kinematics, not a dynamics solver.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import replace
import math
from typing import Mapping

import numpy as np

from .core import Assembly, Material, View, cad_part


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{label} must be finite')
    return float(value)


def _vector(value, label):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f'{label} must have three coordinates')
    result = np.array([_number(v, label) for v in value])
    if np.max(np.abs(result)) > 10000:
        raise ValueError(f'{label} exceeds the 10000 mm/degrees construction envelope')
    return result


def _check_construction_identity(plan):
    """Keep construction claims attached to their source recipe on every path."""
    if plan.get('construction') is None:
        return
    from .construction import recipe_digest
    construction = plan['construction']
    if not isinstance(construction, dict) or construction.get('schema') != 'cybrgeo-construction/1':
        raise ValueError('Unsupported construction schema')
    if construction.get('recipe_sha256') != recipe_digest(plan):
        raise ValueError('Construction no longer matches the geometry or joints. Regenerate from the mechanism plan before building.')
    if 'mechanism' in construction:
        from .construction import validate_constructed_plan
        validate_constructed_plan(plan)
    if len(plan.get('joints', [])) != len(plan.get('parts', [])):
        raise ValueError('Every constructed part must belong to the load and joint hierarchy')


class KinematicGraph:
    """Acyclic part hierarchy and actuator graph, validated before making solids.

    Each joint operates on world-authored reference geometry. Its transform is
    inherited from the parent, then composed with its own reference-frame joint
    transform. Driven values are resolved from their source, never animated
    independently. Explicit out-of-range commands fail instead of being clipped.
    """

    def __init__(self, part_names, joints, motion=None):
        self.part_names = tuple(part_names)
        if len(set(self.part_names)) != len(self.part_names) or not self.part_names:
            raise ValueError('Construction requires unique, nonempty part names')
        if not isinstance(joints, list) or len(joints) > 512:
            raise ValueError('Construction requires a joint list of at most 512 entries')
        self.joints = deepcopy(joints)
        self.by_name = {}
        self.by_child = {}
        names = set(self.part_names)
        for joint in self.joints:
            name = joint.get('name')
            child, parent = joint.get('child'), joint.get('parent')
            if not isinstance(name, str) or not name or name in self.by_name:
                raise ValueError('Joint names must be nonempty and unique')
            if child not in names or child in self.by_child or parent == child or (parent is not None and parent not in names):
                raise ValueError(f'{name}: invalid or duplicate child/parent')
            kind = joint.get('type')
            if kind not in ('fixed', 'revolute', 'prismatic'):
                raise ValueError(f'{name}: invalid joint type')
            origin = _vector(joint.get('origin'), f'{name} origin')
            axis = _vector(joint.get('axis'), f'{name} axis')
            if np.linalg.norm(axis) < 1e-8:
                raise ValueError(f'{name}: zero joint axis')
            joint['origin'] = origin.tolist()
            joint['axis'] = (axis / np.linalg.norm(axis)).tolist()
            low, high, value = (_number(joint.get(key), f'{name} {key}') for key in ('min', 'max', 'value'))
            if low > high or not low <= value <= high or max(abs(low), abs(high)) > (36000 if kind == 'revolute' else 10000):
                raise ValueError(f'{name}: invalid limits/reference value')
            if kind == 'fixed' and (low != 0 or high != 0 or value != 0 or joint.get('drive')):
                raise ValueError(f'{name}: a fixed attachment cannot have motion or a drive')
            self.by_name[name] = joint
            self.by_child[child] = joint

        self.part_order = self._ordered(self.part_names, lambda name: self.by_child.get(name, {}).get('parent'), 'part hierarchy')
        for joint in self.joints:
            drive = joint.get('drive')
            if drive is None:
                continue
            if not isinstance(drive, dict) or drive.get('joint') not in self.by_name:
                raise ValueError(f"{joint['name']}: unknown drive source")
            source = self.by_name[drive['joint']]
            if source['type'] == 'fixed':
                raise ValueError(f"{joint['name']}: drive source is fixed")
            ratio = _number(drive.get('ratio'), 'drive ratio')
            offset = _number(drive.get('offset'), 'drive offset')
            if ratio == 0:
                raise ValueError(f"{joint['name']}: zero drive ratio")
            a, b = source['min'] * ratio + offset, source['max'] * ratio + offset
            if min(a, b) < joint['min'] - 1e-8 or max(a, b) > joint['max'] + 1e-8:
                raise ValueError(f"{joint['name']}: follower limits do not cover source travel")
        self.drive_order = self._ordered(self.by_name, lambda name: (self.by_name[name].get('drive') or {}).get('joint'), 'drive graph')
        self.controls = tuple(j['name'] for j in self.joints if j['type'] != 'fixed' and j.get('drive') is None)
        from .motion import MotionTask
        self.motion_task = MotionTask.from_dict(motion) if motion is not None else MotionTask.from_joints(self.joints)
        if set(self.motion_task.waypoints[0]) != set(self.controls):
            raise ValueError('Motion task must cover the graph independent controls exactly')
        for point in self.motion_task.waypoints:
            self.resolve(point)

    @staticmethod
    def _ordered(nodes, parent, label):
        ordered, marks = [], {}

        def visit(node):
            if marks.get(node) == 1:
                raise ValueError(f'Cycle in {label}')
            if marks.get(node) == 2:
                return
            marks[node] = 1
            predecessor = parent(node)
            if predecessor is not None:
                visit(predecessor)
            marks[node] = 2
            ordered.append(node)

        for node in nodes:
            visit(node)
        return tuple(ordered)

    def resolve(self, values: Mapping[str, float] | None = None):
        values = values or {}
        for name, value in values.items():
            if name not in self.controls:
                raise ValueError(f'{name}: command must target an independent movable joint')
            number = _number(value, name)
            joint = self.by_name[name]
            if not joint['min'] <= number <= joint['max']:
                raise ValueError(f"{name}: command {number} outside [{joint['min']}, {joint['max']}]")
        result = {}
        for name in self.drive_order:
            joint = self.by_name[name]
            drive = joint.get('drive')
            result[name] = (result[drive['joint']] * drive['ratio'] + drive['offset']) if drive else float(values.get(name, joint['value']))
            if not joint['min'] - 1e-8 <= result[name] <= joint['max'] + 1e-8:
                raise ValueError(f'{name}: derived drive command outside travel')
        return result

    def transforms(self, values=None):
        resolved = self.resolve(values)
        transforms = {}
        for name in self.part_order:
            joint = self.by_child.get(name)
            local = np.eye(4)
            if joint is None:
                transforms[name] = local
                continue
            value = resolved[joint['name']]
            axis, origin = np.array(joint['axis']), np.array(joint['origin'])
            if joint['type'] == 'revolute':
                angle = math.radians(value)
                x, y, z = axis
                cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                rotation = math.cos(angle) * np.eye(3) + (1 - math.cos(angle)) * np.outer(axis, axis) + math.sin(angle) * cross
                local[:3, :3] = rotation
                local[:3, 3] = origin - rotation @ origin
            elif joint['type'] == 'prismatic':
                local[:3, 3] = axis * value
            transforms[name] = (transforms[joint['parent']] @ local) if joint['parent'] is not None else local
        return transforms

    def demo_values(self, time_seconds):
        """Evaluate the source-bound task; legacy plans use an 8-second task.

        These are requested positions. Execution authorization and actual
        integrated motion are separate products of the engineering pipeline.
        """
        return self.motion_task.sample(time_seconds)['values']

    def report(self):
        roots = [name for name in self.part_names if name not in self.by_child or self.by_child[name]['parent'] is None]
        return {'scope': 'Construction graph and prescribed kinematics; no simulated dynamics or hardware qualification.',
                'parts': len(self.part_names), 'joints': len(self.joints), 'independent_controls': list(self.controls),
                'roots': roots, 'acyclic_part_hierarchy': True, 'acyclic_drives': True,
                'drive_ranges_contained': True,
                'drives': [{'follower': j['name'], **j['drive']} for j in self.joints if j.get('drive')],
                'part_parents': {name: self.by_child.get(name, {}).get('parent') for name in self.part_names}}


class ConstructedMotion:
    def __init__(self, graph, values=None):
        self.graph = graph
        self.values = None if values is None else dict(values)
        if self.values is not None:
            graph.resolve(self.values)
        self._time = None
        self._transforms = None

    def __call__(self, part, time_seconds=0, explode=0):
        if self._transforms is None or time_seconds != self._time:
            commands = self.graph.demo_values(time_seconds) if self.values is None else self.values
            self._transforms = self.graph.transforms(commands)
            self._time = time_seconds
        transform = self._transforms[part.name].copy()
        transform[:3, 3] += np.asarray(part.explode) * explode
        return transform


class _PoseFactory:
    def bind(self, assembly):
        plan = assembly.metadata['construction_recipe']
        _check_construction_identity(plan)
        graph = KinematicGraph([p.name for p in assembly.parts], plan['joints'], (plan.get('construction') or {}).get('motion'))
        selected = assembly.metadata.get('selected_joint_values')
        values = None if selected is None else {name: selected[name] for name in graph.controls}
        return ConstructedMotion(graph, values)


# registry.load() calls .bind() on a cache hit. The graph lives in the manifest,
# never in a non-serializable closure or a stale module-global assembly.
pose = _PoseFactory()


def articulation(assembly, values):
    """Return a shallow assembly copy at a commanded pose; geometry is shared."""
    plan = assembly.metadata['construction_recipe']
    graph = KinematicGraph([p.name for p in assembly.parts], plan['joints'], (plan.get('construction') or {}).get('motion'))
    graph.resolve(values)
    metadata = {**assembly.metadata, 'selected_joint_values': graph.resolve(values)}
    return replace(assembly, metadata=metadata, motion_function=ConstructedMotion(graph, values))


def primitive(spec):
    """Construct a primitive using exactly the browser Studio coordinate rules."""
    import cadquery as cq
    size = _vector(spec.get('size'), 'primitive size')
    position = _vector(spec.get('position'), 'primitive position')
    rotation = _vector(spec.get('rotation'), 'primitive rotation')
    if np.any(size <= 0):
        raise ValueError('Primitive dimensions must be positive')
    kind = spec.get('kind')
    if kind == 'box':
        shape = cq.Workplane('XY').box(*size).val()
    elif kind == 'cylinder':
        shape = cq.Workplane('XY').circle(size[0] / 2).extrude(size[2]).translate((0, 0, -size[2] / 2)).val()
    elif kind == 'sphere':
        shape = cq.Workplane('XY').sphere(size[0] / 2).val()
    else:
        raise ValueError(f'Unknown construction primitive: {kind}')
    for angle, axis in zip(rotation, np.eye(3)):
        if angle:
            shape = shape.rotate((0, 0, 0), tuple(axis), float(angle))
    return shape.translate(tuple(position))


def build_from_plan(plan):
    """Compile one common recipe to native CAD after validating its whole graph."""
    plan = deepcopy(plan)
    if plan.get('units') != 'mm' or not isinstance(plan.get('parts'), list) or not 1 <= len(plan['parts']) <= 512:
        raise ValueError('Construction requires 1-512 parts in millimetres')
    graph = KinematicGraph([part['name'] for part in plan['parts']], plan.get('joints', []), (plan.get('construction') or {}).get('motion'))
    graph.resolve()
    _check_construction_identity(plan)
    # Validate primitive and material scalar inputs before invoking the kernel.
    for part in plan['parts']:
        if not isinstance(part.get('cuts'), list) or not isinstance(part.get('unions'), list):
            raise ValueError(f"{part['name']}: expected boolean operation lists")
        for spec in [part['body'], *part['unions'], *part['cuts']]:
            if spec.get('kind') not in ('box', 'cylinder', 'sphere') or np.any(_vector(spec.get('size'), 'size') <= 0):
                raise ValueError('Invalid construction primitive')
            _vector(spec.get('position'), 'position')
            _vector(spec.get('rotation'), 'rotation')
        color = _vector(part.get('color'), 'color')
        roughness = _number(part.get('roughness'), 'roughness')
        metalness = _number(part.get('metalness'), 'metalness')
        fillet = _number(part.get('fillet', 0), 'fillet')
        if np.any(color < 0) or np.any(color > 1) or not 0 < roughness <= 1 or not 0 <= metalness <= 1 or not 0 <= fillet <= 25:
            raise ValueError('Invalid construction material/fillet')
    materials, lookup, parts = [], {}, []
    roles = (plan.get('construction') or {}).get('part_roles', {})
    for spec in plan['parts']:
        shape = primitive(spec['body'])
        for item in spec['unions']:
            shape = shape.fuse(primitive(item))
        for item in spec['cuts']:
            shape = shape.cut(primitive(item))
        if spec.get('fillet', 0):
            import cadquery as cq
            shape = cq.Workplane(obj=shape).edges().fillet(spec['fillet']).val()
        if not shape.isValid() or len(shape.Solids()) != 1 or not math.isfinite(shape.Volume()) or shape.Volume() <= .001:
            raise ValueError(f"{spec['name']}: expected one connected valid positive-volume solid")
        key = (*spec['color'], spec['metalness'], spec['roughness'])
        if key not in lookup:
            lookup[key] = len(materials)
            materials.append(Material(f'Construction finish {len(materials) + 1}', tuple(spec['color']), spec['metalness'], spec['roughness']))
        joint = graph.by_child.get(spec['name'])
        center = np.array(joint['origin'] if joint else [0, 0, 0])
        parts.append(cad_part(spec['name'], shape, lookup[key], tolerance=.06, angular=.12,
                              group=spec.get('group', 'construction'), motion=joint['name'] if joint else 'fixed',
                              center=center, role=spec.get('role', roles.get(spec['name'], 'Original constructed mechanism component')),
                              provenance=spec.get('provenance', 'designed-concept'),
                              explode=np.array(spec.get('explode', [0, 0, 0]), float)))
    metadata = {'truth_intent': 'concept', 'construction_recipe': plan,
                'construction': plan.get('construction', {}), 'kinematics': graph.report(),
                'qualification': {'kinematics': 'prescribed from construction graph',
                                  'dynamics': 'not simulated', 'hardware': 'not qualified'}}
    assembly = Assembly(plan['name'], parts, materials, metadata=metadata, motion_function=ConstructedMotion(graph))
    # Preserve density-independent analytic inertia through the mesh cache.
    # Dynamics later applies declared physical materials, never preview colors.
    from .dynamics import geometry_integrals
    assembly.metadata['geometry_mass_integrals'] = geometry_integrals(assembly)
    bounds = assembly.bounds
    center = tuple((bounds[0] + bounds[1]) / 2)
    scale = max(float(np.max(bounds[1] - bounds[0])) * .65, 20)
    assembly.views = {'hero': View(az=40, el=22, scale=scale, target=center, title=plan['name']),
                      'front': View(az=90, el=0, scale=scale, target=center, projection='orthographic'),
                      'top': View(az=0, el=89, scale=scale, target=center, projection='orthographic')}
    return assembly


def build(architecture='compact_arm', parameters=None):
    from .engineering_pipeline import compile_source
    plan, _ = compile_source(architecture, parameters)
    assembly = build_from_plan(plan)
    assembly.name = architecture
    return assembly
