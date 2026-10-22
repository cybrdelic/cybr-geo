"""Pinned planar links: one geometry record for native holes and solver closure.

The loop is closed by forces at two material pin axes, never a time function or
an assignment to the follower coordinate. Bearings are ideal revolutes in the
solver. Declared nominal CAD clearance is not a bearing-contact or wear model.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
import math
import re
import xml.etree.ElementTree as ET

import numpy as np


def _positive(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and positive')
    return float(value)


def _frame(value):
    frame = np.asarray(value, dtype=float)
    if (frame.shape != (4, 4) or not np.isfinite(frame).all()
            or not np.allclose(frame[3], [0, 0, 0, 1], rtol=0, atol=1e-12)
            or not np.allclose(frame[:3, :3].T @ frame[:3, :3], np.eye(3), rtol=0, atol=1e-12)
            or not np.isclose(np.linalg.det(frame[:3, :3]), 1, rtol=0, atol=1e-12)
            or not np.allclose(frame[:3, 2], [0, 0, 1], rtol=0, atol=1e-12)):
        raise ValueError('Planar link frames require rigid Z-axis poses in mm')
    return frame


def _point(value):
    point = np.asarray(value, dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError('Pin anchor must contain three finite body-local millimetres')
    return point


def _fmt(values):
    return ' '.join(f'{v:.16g}' for v in values)


def _model_frame(physical, name):
    """Resolve the supported explicit planar qpos0 subtree, without a solver.

    Attachment happens before CAD inertia binding, so compiling an incomplete
    MuJoCo model to discover its pose is inappropriate. Ambiguous/defaulted
    transforms and axes are rejected instead of silently assuming a frame.
    """
    parents = {child: parent for parent in physical.root.iter() for child in parent}
    body, chain = physical.bodies[name], []
    while body.tag == 'body':
        chain.append(body)
        body = parents[body]
    if body.tag != 'worldbody':
        raise ValueError('Pinned links require explicit body ancestry without MJCF frame wrappers')
    frame = np.eye(4)
    default_joints = physical.root.findall('.//default/joint')
    for body in reversed(chain):
        if (set(body.attrib) & {'class', 'childclass', 'euler', 'axisangle', 'xyaxes', 'zaxis'}):
            raise ValueError('Pinned link frames require explicit pos/quat transforms without default classes')
        position = np.fromstring(body.get('pos', '0 0 0'), sep=' ')
        quat = np.fromstring(body.get('quat', '1 0 0 0'), sep=' ')
        if (position.shape != (3,) or quat.shape != (4,) or not np.isfinite(position).all()
                or not np.isfinite(quat).all() or np.linalg.norm(quat) <= 0):
            raise ValueError('Invalid explicit MJCF body pose')
        w, x, y, z = quat / np.linalg.norm(quat)
        local = np.eye(4)
        local[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
        local[:3, 3] = position * 1000
        frame = frame @ local
        _frame(frame)
        if body.find('freejoint') is not None:
            raise ValueError('A pinned planar loop cannot have free ancestor motion')
        for joint in body.findall('joint'):
            if joint.get('class') or any(key in row.attrib and key not in joint.attrib
                for row in default_joints for key in ('type', 'axis')):
                raise ValueError('Pinned link joint type and axis must be explicit when defaulted')
            axis = np.fromstring(joint.get('axis', '0 0 1'), sep=' ')
            if joint.get('type', 'hinge') != 'hinge' or axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) <= 0:
                raise ValueError('Pinned planar loop ancestors require fixed or parallel hinge motion')
            world_axis = frame[:3, :3] @ (axis / np.linalg.norm(axis))
            if not np.allclose(np.abs(world_axis), [0, 0, 1], atol=1e-12, rtol=0):
                raise ValueError('Pinned planar loop hinge axes must stay parallel to Z')
    return frame


@dataclass(frozen=True)
class PinnedLink:
    """Manufacturable nominal strap and its two planar pin-axis attachments.

    Frames describe the model's qpos0 pose. The child rod frame is at the input
    pin, with +X pointing to the output pin. Its constant length is measured
    once at construction; MuJoCo solves the closed loop throughout motion.
    """
    name: str
    input_body: str
    output_body: str
    input_anchor_mm: tuple
    output_anchor_mm: tuple
    length_mm: float
    input_relative_yaw_rad: float
    width_mm: float
    thickness_mm: float
    pin_diameter_mm: float
    diametral_clearance_mm: float
    minimum_wall_mm: float
    input_frame_mm: tuple
    output_frame_mm: tuple

    def __post_init__(self):
        for identity in (self.name, self.input_body, self.output_body):
            if not isinstance(identity, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_.-]*', identity):
                raise ValueError('Link and body identities must be portable MJCF identifiers')
        if len({self.name, self.input_body, self.output_body}) != 3:
            raise ValueError('A pinned link must connect two distinct existing bodies')
        first, second = _frame(self.input_frame_mm), _frame(self.output_frame_mm)
        a, b = _point(self.input_anchor_mm), _point(self.output_anchor_mm)
        for key in ('length_mm', 'width_mm', 'thickness_mm', 'pin_diameter_mm',
                    'diametral_clearance_mm', 'minimum_wall_mm'):
            _positive(getattr(self, key), key)
        bore = self.pin_diameter_mm + self.diametral_clearance_mm
        if self.width_mm < bore + 2*self.minimum_wall_mm or self.length_mm < bore + self.minimum_wall_mm:
            raise ValueError('Link dimensions cannot contain the declared bores and minimum material wall')
        yaw = self.input_relative_yaw_rad
        if isinstance(yaw, bool) or not math.isfinite(yaw):
            raise ValueError('Pinned link yaw must be finite')
        authored_delta = self.length_mm * np.array([math.cos(yaw), math.sin(yaw), 0.])
        actual_delta = first[:3, :3].T @ (second[:3, :3] @ b + second[:3, 3] - first[:3, :3] @ a - first[:3, 3])
        if not np.allclose(authored_delta, actual_delta, atol=1e-8, rtol=0):
            raise ValueError('Pinned link length/yaw differs from its declared material pin axes')

    @classmethod
    def from_frames(cls, name, input_body, output_body, input_frame_mm, output_frame_mm,
                    input_anchor_mm, output_anchor_mm, *, width_mm, thickness_mm,
                    pin_diameter_mm, diametral_clearance_mm, minimum_wall_mm):
        for identity in (name, input_body, output_body):
            if not isinstance(identity, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_.-]*', identity):
                raise ValueError('Link and body identities must be portable MJCF identifiers')
        if len({name, input_body, output_body}) != 3:
            raise ValueError('A pinned link must connect two distinct existing bodies')
        first, second = _frame(input_frame_mm), _frame(output_frame_mm)
        a, b = _point(input_anchor_mm), _point(output_anchor_mm)
        world_a, world_b = first[:3, :3] @ a + first[:3, 3], second[:3, :3] @ b + second[:3, 3]
        delta = first[:3, :3].T @ (world_b - world_a)
        if abs(delta[2]) > 1e-8:
            raise ValueError('Planar pin centres must share an axial plane at construction')
        length = _positive(float(np.linalg.norm(delta)), 'Pin centre distance')
        dimensions = [width_mm, thickness_mm, pin_diameter_mm, diametral_clearance_mm, minimum_wall_mm]
        for label, value in zip(('width', 'thickness', 'pin diameter', 'diametral clearance', 'minimum wall'), dimensions):
            _positive(value, label)
        bore = pin_diameter_mm + diametral_clearance_mm
        if width_mm < bore + 2 * minimum_wall_mm or length < bore + minimum_wall_mm:
            raise ValueError('Link dimensions cannot contain the declared bores and minimum material wall')
        return cls(name, input_body, output_body, tuple(a), tuple(b), length,
                   math.atan2(delta[1], delta[0]), *map(float, dimensions),
                   tuple(map(tuple, first)), tuple(map(tuple, second)))

    def source(self):
        return dict(schema='cybrgeo-pinned-link/1', **asdict(self),
                    scope='Rigid strap with nominal bored pin interfaces; ideal bearing axes in dynamics')

    def native_shape(self):
        """Actual strap in rod-local mm, including both clearance bores."""
        import cadquery as cq
        bore = (self.pin_diameter_mm + self.diametral_clearance_mm) / 2
        shape = cq.Workplane('XY').center(self.length_mm / 2, 0).slot2D(
            self.length_mm + self.width_mm, self.width_mm).extrude(self.thickness_mm)
        cutters = cq.Workplane('XY').pushPoints([(0, 0), (self.length_mm, 0)]).circle(bore).extrude(self.thickness_mm)
        return shape.cut(cutters).val().translate((0, 0, -self.thickness_mm / 2))

    def attach(self, physical, *, damping_Nm_s_rad=0., solref=(.002, 1.), solimp=(.99, .999, .00001)):
        """Attach a passive rod hinge and a point closure at the opposite bore.

        CAD inertia must be bound separately with PhysicalModel.bind_inertias.
        No mass, motor, infinite torque, or undocumented collision exclusion is
        invented here. Both anchor sites remain explicit in serialized MJCF.
        """
        if (not math.isfinite(damping_Nm_s_rad) or damping_Nm_s_rad < 0
                or isinstance(damping_Nm_s_rad, bool)):
            raise ValueError('Pin damping must be finite and nonnegative')
        bodies = physical.bodies
        if self.name in bodies or self.input_body not in bodies or self.output_body not in bodies:
            raise ValueError('Pinned link requires two existing bodies and a new rod identity')
        for body, authored in ((self.input_body, self.input_frame_mm), (self.output_body, self.output_frame_mm)):
            if not np.allclose(_model_frame(physical, body), authored, atol=1e-7, rtol=0):
                raise ValueError(f'Pinned link qpos0 frame differs from the physical body: {body}')
        if len(solref) != 2 or len(solimp) != 3 or not np.isfinite([*solref, *solimp]).all():
            raise ValueError('Explicit finite constraint solver parameters required')
        identities = {self.name + suffix for suffix in ('_hinge', '_tip', '_socket', '_closure')}
        if any(item.get('name') in identities for item in physical.root.iter()):
            raise ValueError('Pinned link generated identities already exist')
        rod = ET.SubElement(bodies[self.input_body], 'body', name=self.name,
            pos=_fmt(np.asarray(self.input_anchor_mm) * .001),
            quat=_fmt([math.cos(self.input_relative_yaw_rad / 2), 0, 0, math.sin(self.input_relative_yaw_rad / 2)]))
        # A model-wide joint default must not move the material pin axis or add
        # an undeclared spring, friction, inertia or reference displacement.
        ET.SubElement(rod, 'joint', name=self.name + '_hinge', type='hinge', axis='0 0 1',
                      pos='0 0 0', ref='0', springref='0', stiffness='0', springdamper='0 0',
                      armature='0', frictionloss='0', limited='false', actuatorfrclimited='false',
                      damping=f'{damping_Nm_s_rad:.16g}')
        ET.SubElement(rod, 'site', name=self.name + '_tip', pos=_fmt([self.length_mm * .001, 0, 0]), size='.0002')
        ET.SubElement(bodies[self.output_body], 'site', name=self.name + '_socket',
                      pos=_fmt(np.asarray(self.output_anchor_mm) * .001), size='.0002')
        equalities = physical.root.find('equality')
        if equalities is None:
            equalities = ET.SubElement(physical.root, 'equality')
        # Serialize the standard impedance midpoint/power too, so omitted
        # trailing values cannot inherit an unrelated model default.
        ET.SubElement(equalities, 'connect', name=self.name + '_closure',
                      site1=self.name + '_tip', site2=self.name + '_socket',
                      active='true', solref=_fmt(solref), solimp=_fmt([*solimp, .5, 2.]))
        physical.provenance.setdefault('pinned_links', []).append(self.source())
        return rod
