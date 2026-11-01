"""Component inventory, recorded motion and mass accounting for shared consumers.

An assembly owns identities and body membership. Render, native export and
collision adapters ask it for the same placement; none reconstruct a rig from
part names. Recorded poses remain reference motion, never a dynamics result.
Optical material records and mechanical mass records remain independent.
"""
from dataclasses import dataclass
from copy import deepcopy
import math
import hashlib
from pathlib import Path
import numpy as np
from .physical_model import inertia_tensor, digest


def require_component_export(purpose='manufacturing'):
    """One preconstruction/export policy; no diagnostic manufacturing bypass."""
    if purpose != 'inspection':
        from .qualification import QualificationError
        raise QualificationError(
            'ComponentAssembly is diagnostic: manufacturing export requires '
            'QualifiedNativeConstruction and release_project with qualified physical evidence; '
            'request purpose=inspection explicitly for diagnostic output')
    return dict(purpose='inspection', hardware_qualified=False)


@dataclass(frozen=True)
class RackPinionMotion:
    parent_body: object
    rack_body: object
    rotating_ids: tuple
    reference_coordinate_m: float
    phase_rad: float
    pitch_radius_m: float
    pivot_m: tuple
    rack_bodies: object = None

    def __post_init__(self):
        values = [self.reference_coordinate_m, self.phase_rad, self.pitch_radius_m, *self.pivot_m]
        if len(self.pivot_m) != 3 or not np.isfinite(values).all() or self.pitch_radius_m <= 0:
            raise ValueError('Rack/pinion requires finite coordinates and positive pitch radius')
        if not self.rotating_ids or len(set(self.rotating_ids)) != len(self.rotating_ids):
            raise ValueError('Rack/pinion requires unique rotating component IDs')
        signs = dict(self.rack_bodies or {self.rack_body: 1.})
        if self.rack_body not in signs or signs[self.rack_body] != 1.:
            raise ValueError('Driving rack must have positive unit coordinate direction')
        if not all(np.isfinite(v) and v != 0 for v in signs.values()):
            raise ValueError('Rack coordinate directions must be finite and nonzero')
        object.__setattr__(self, 'rack_bodies', signs)
        object.__setattr__(self, 'rotating_ids', tuple(self.rotating_ids))
        object.__setattr__(self, 'pivot_m', tuple(self.pivot_m))

    def coordinate(self, body_poses):
        return float((np.linalg.inv(body_poses[self.parent_body])@body_poses[self.rack_body])[0, 3])

    def angle(self, coordinate_m):
        return self.phase_rad - (coordinate_m-self.reference_coordinate_m)/self.pitch_radius_m

    def transform(self, coordinate_m):
        angle = self.angle(coordinate_m)
        c, s = math.cos(angle), math.sin(angle)
        rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
        result = np.eye(4)
        result[:3, :3] = rotation
        pivot = np.asarray(self.pivot_m)
        result[:3, 3] = pivot-rotation@pivot
        return result


class ComponentAssembly:
    """One component/body authority, with explicit recorded-pose provenance.

    ``frames`` maps frame labels to world transforms for every body (metres).
    ``body_parents`` is a rooted forest. Components store body-local geometry.
    Optional rack overrides are diagnostic coordinates, not solved dynamics.
    """
    def __init__(self, parts, body_parents, frames, *, connections=(), motions=(), drives=(), selections=None):
        parts = tuple(parts)
        self.parts = {p['id']: deepcopy(p) for p in parts}
        if len(self.parts) != len(parts):
            raise ValueError('Duplicate component IDs')
        self.body_parents = dict(body_parents)
        bodies = set(self.body_parents)
        if any(p['body_index'] not in bodies for p in self.parts.values()):
            raise ValueError('Every component requires a declared body')
        for body in bodies:
            seen = set()
            cursor = body
            while cursor is not None:
                if cursor in seen or cursor not in bodies:
                    raise ValueError('Body topology is cyclic or references a missing parent')
                seen.add(cursor)
                cursor = self.body_parents[cursor]
        self.frames = {}
        for frame, rows in frames.items():
            if set(rows) != bodies:
                raise ValueError('Reference pose must cover the complete body inventory')
            parsed = {}
            for body, value in rows.items():
                matrix = np.array(value, dtype=float, copy=True)
                if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
                    or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-10)
                    or not np.allclose(matrix[:3, :3].T@matrix[:3, :3], np.eye(3), atol=2e-6)
                    or not np.isclose(np.linalg.det(matrix[:3, :3]), 1., atol=2e-6)):
                    raise ValueError('Body pose must be a finite rigid transform')
                matrix.setflags(write=False)
                parsed[body] = matrix
            self.frames[str(frame)] = parsed
        if not self.frames:
            raise ValueError('At least one reference pose is required')
        self.connections = deepcopy(list(connections))
        for edge in self.connections:
            if any(i not in self.parts for i in [edge['a'], edge['b'], *edge.get('hardware', [])]):
                raise ValueError('Connection references a missing component')
        self.motions = tuple(motions)
        self._rotation = {}
        self._racks = {}
        for motion in self.motions:
            if motion.parent_body not in bodies or any(b not in bodies for b in motion.rack_bodies):
                raise ValueError('Motion references a missing body')
            for pid in motion.rotating_ids:
                if pid not in self.parts or pid in self._rotation:
                    raise ValueError('Motion requires present, exclusively owned component IDs')
                if self.parts[pid]['body_index'] != motion.parent_body:
                    raise ValueError('Rotating part geometry must use its motion parent frame')
                self._rotation[pid] = motion
            for body in motion.rack_bodies:
                if body in self._racks:
                    raise ValueError('Rack body has multiple coordinate owners')
                self._racks[body] = motion
        self._descendants = {b: frozenset(c for c in bodies if self._under(c, b)) for b in bodies}
        self.drives = deepcopy(list(drives))
        for drive in self.drives:
            axis, origin = np.asarray(drive['axis'], float), np.asarray(drive['origin_m'], float)
            if (drive['parent_body'] not in bodies or drive['child_body'] not in bodies
                or self.body_parents[drive['child_body']] != drive['parent_body']
                or axis.shape != (3,) or origin.shape != (3,) or not np.isfinite(origin).all()
                or not np.isfinite(axis).all() or not np.isclose(np.linalg.norm(axis), 1.)):
                raise ValueError('Drive requires a body edge, unit axis and finite local origin')
        self.selections = deepcopy(selections or {})
        for selection in self.selections.values():
            if (any(pid not in self.parts for pid in selection['ids'])
                or len(selection['ids']) != len(set(selection['ids']))
                or selection.get('relative_body') is not None and selection['relative_body'] not in bodies):
                raise ValueError('Assembly selection references duplicate or missing components/body')

    def _under(self, body, ancestor):
        while body is not None:
            if body == ancestor:
                return True
            body = self.body_parents[body]
        return False

    def coordinate(self, frame, motion):
        poses = self.frames[str(frame)]
        return motion.coordinate(poses)

    def rack_coordinate(self, frame, motion_index=0):
        return self.coordinate(frame, self.motions[motion_index])

    def angle(self, frame, motion_index=0, *, rack_coordinate_m=None):
        motion = self.motions[motion_index]
        q = self.coordinate(frame, motion) if rack_coordinate_m is None else rack_coordinate_m
        return motion.angle(q)

    def body_pose(self, frame, body, *, rack_coordinate_m=None):
        poses = self.frames[str(frame)]
        result = poses[body].copy()
        motion = self._racks.get(body)
        if rack_coordinate_m is not None and motion is not None:
            delta = float(rack_coordinate_m)-self.coordinate(frame, motion)
            result[:3, 3] += poses[motion.parent_body][:3, 0]*delta*motion.rack_bodies[body]
        return result

    def pose(self, frame, component_id, *, rack_coordinate_m=None):
        motion = self._rotation.get(component_id)
        if motion is None:
            return self.body_pose(frame, self.parts[component_id]['body_index'], rack_coordinate_m=rack_coordinate_m)
        q = self.coordinate(frame, motion) if rack_coordinate_m is None else float(rack_coordinate_m)
        return self.frames[str(frame)][motion.parent_body]@motion.transform(q)

    def local_pose(self, frame, component_id, relative_body, *, rack_coordinate_m=None):
        return np.linalg.inv(self.body_pose(frame, relative_body, rack_coordinate_m=rack_coordinate_m))@self.pose(frame, component_id, rack_coordinate_m=rack_coordinate_m)

    def select_ids(self, group=None):
        return tuple(pid for pid, row in self.parts.items() if group is None or row.get('group') == group)

    @property
    def revision(self):
        root=Path(__file__).parent
        files=['assembly_model.py','assembly_native.py','assembly_physics.py','blender_assembly.py','native_construction.py','physical_model.py']
        return digest(dict(document=self.manifest(),implementation={n:hashlib.sha256((root/n).read_bytes()).hexdigest() for n in files if (root/n).exists()}))

    def require_export(self, purpose='manufacturing'):
        """Diagnostic assembly export must be explicitly requested as inspection.

        This adapter cannot mint manufacturing authority. Qualified CAD and
        release stay on QualifiedNativeConstruction/release_project, including
        their existing revision-bound physical evidence requirements.
        """
        return {**require_component_export(purpose), 'assembly_revision': self.revision}

    def manifest(self):
        return dict(schema='cybrgeo-component-assembly/1', units='m',
                    parts=[deepcopy(self.parts[i]) for i in sorted(self.parts)],
                    bodies=[dict(id=body, parent=parent) for body,parent in self.body_parents.items()],
                    reference_frames=[dict(frame=frame, body_poses=[dict(body=body, matrix=value.tolist())
                        for body,value in poses.items()]) for frame,poses in self.frames.items()],
                    motions=[dict(kind='rack_pinion', parent_body=m.parent_body, rack_body=m.rack_body,
                        rotating_ids=list(m.rotating_ids), reference_coordinate_m=m.reference_coordinate_m,
                        phase_rad=m.phase_rad, pitch_radius_m=m.pitch_radius_m, pivot_m=list(m.pivot_m),
                        racks=[dict(body=b, direction=d) for b,d in m.rack_bodies.items()]) for m in self.motions],
                    drives=deepcopy(self.drives), selections=deepcopy(self.selections),
                    connection_graph=deepcopy(self.connections), reference_motion_only=True,
                    whole_robot_qualified=False)

    @classmethod
    def from_document(cls, document):
        if document.get('schema') != 'cybrgeo-component-assembly/1':
            raise ValueError('Expected cybrgeo-component-assembly/1')
        if document.get('units') != 'm':
            raise ValueError('Component assembly body poses must explicitly use metres (units=m)')
        motions = []
        for row in document['motions']:
            if row['kind'] != 'rack_pinion':
                raise ValueError('Unsupported component motion')
            motions.append(RackPinionMotion(row['parent_body'], row['rack_body'], tuple(row['rotating_ids']),
                row['reference_coordinate_m'], row['phase_rad'], row['pitch_radius_m'], tuple(row['pivot_m']),
                {r['body']:r['direction'] for r in row['racks']}))
        return cls(document['parts'], {r['id']:r['parent'] for r in document['bodies']},
            {r['frame']:{p['body']:p['matrix'] for p in r['body_poses']} for r in document['reference_frames']},
            connections=document['connection_graph'], motions=motions,
            drives=document.get('drives', ()), selections=document.get('selections'))

    @staticmethod
    def mass_deltas(old_records, new_records):
        """Parallel-axis mass deltas from explicit mechanical records, not color.

        Records: id, body_index, mass_kg, com_m, inertia_com_kg_m2.
        An absent old record means a newly added component, not zero old density.
        """
        old = {r['id']: r for r in old_records}
        new = {r['id']: r for r in new_records}
        bodies, changes = {}, []
        def props(row):
            if row is None:
                return 0., np.zeros(3), np.zeros((3, 3))
            return inertia_tensor(dict(row, inertia_tensor_kg_m2=row['inertia_com_kg_m2']))
        for pid in sorted(old.keys() | new.keys()):
            before, after = old.get(pid), new.get(pid)
            om, oc, oi = props(before)
            nm, nc, ni = props(after)
            row = after or before
            changes.append(dict(id=pid, body_index=row['body_index'], name=row.get('name', str(pid)),
                old_mass_kg=om, new_mass_kg=nm, delta_mass_kg=nm-om,
                new_com_m=nc.tolist(), new_inertia_com_kg_m2=ni.tolist()))
            for sign, record, mass, com, inertia in [(-1, before, om, oc, oi), (1, after, nm, nc, ni)]:
                if record is None:
                    continue
                body = bodies.setdefault(record['body_index'], dict(delta_mass_kg=0.,
                    delta_first_moment_kg_m=np.zeros(3), delta_inertia_origin_kg_m2=np.zeros((3, 3))))
                body['delta_mass_kg'] += sign*mass
                body['delta_first_moment_kg_m'] += sign*mass*com
                body['delta_inertia_origin_kg_m2'] += sign*(inertia+mass*(np.dot(com, com)*np.eye(3)-np.outer(com, com)))
        return dict(part_deltas=changes, body_deltas={b:{k:v.tolist() if isinstance(v, np.ndarray) else v
                    for k,v in row.items()} for b,row in bodies.items()})

    def gravity_increment(self, frame, body_deltas, drives=None, *, gravity=(0., 0., -9.81)):
        """Quasistatic recorded-pose torque; no capacity or dynamics implication.

        Drives explicitly identify parent_body, child_body, axis, origin_m.
        Mass deltas are body-local. Internal reference spin is not inferred.
        """
        result = []
        for drive in self.drives if drives is None else drives:
            parent = self.body_pose(frame, drive['parent_body'])
            axis = parent[:3, :3]@np.asarray(drive['axis'])
            origin = parent[:3, :3]@np.asarray(drive['origin_m'])+parent[:3, 3]
            torque = 0.
            for body in self._descendants[drive['child_body']]:
                delta = body_deltas.get(body, body_deltas.get(str(body)))
                if delta is None:
                    continue
                pose = self.body_pose(frame, body)
                first = pose[:3, :3]@np.asarray(delta['delta_first_moment_kg_m'])+(pose[:3, 3]-origin)*delta['delta_mass_kg']
                torque += float(np.dot(axis, np.cross(first, gravity)))
            result.append(torque)
        return result
