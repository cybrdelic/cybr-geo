"""Shared editable rigid-body model, independent of a mechanism or controller.

The MJCF tree is the physical IR: named bodies, local frames, inertias, joints,
transmissions, contacts and initialization survive lowering without flattening.
Native CAD binds mass records to this tree before simulation. Display meshes
never silently become collision hulls or replace authored physical mass.

Inertia/proxy lowering is adapted from cybrdelic/cybr-arx6-belt,
aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2,
ARX-6-P5-Engineering/integration/build_p5_model.py. See NOTICE-SOURCES.md.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def _fmt(values):
    return ' '.join(f'{float(x):.13g}' for x in np.asarray(values).ravel())


def _proxy_attributes(row):
    """Validate and lower a contact record without changing its provenance."""
    def vector(key, length):
        value = np.asarray(row[key], float)
        if value.shape != (length,) or not np.isfinite(value).all():
            raise ValueError(f'Contact {key} requires {length} finite values')
        return value

    if not isinstance(row, dict) or not isinstance(row.get('name'), str) or not row['name']:
        raise ValueError('Contact geometry requires a name')
    if not isinstance(row.get('body'), str) or not row['body']:
        raise ValueError('Contact geometry requires a body provenance label')
    kind = row.get('type')
    if kind not in ('box', 'cylinder', 'capsule', 'sphere'):
        raise ValueError(f'Unsupported contact primitive {kind}')
    if kind == 'box':
        vector('center_mm', 3)
        if np.any(vector('half_size_mm', 3) <= 0):
            raise ValueError('Contact half sizes must be positive')
        if 'quat_wxyz' in row and not np.isclose(np.linalg.norm(vector('quat_wxyz', 4)), 1., rtol=1e-8, atol=1e-10):
            raise ValueError('Contact quaternion must be normalized')
    else:
        radius = float(row['radius_mm'])
        if not math.isfinite(radius) or radius <= 0:
            raise ValueError('Contact radius must be positive and finite')
        if kind == 'sphere':
            vector('center_mm', 3)
        else:
            ends = np.asarray(row['fromto_mm'], float)
            if ends.shape != (2, 3) or not np.isfinite(ends).all() or np.linalg.norm(ends[1] - ends[0]) <= 1e-12:
                raise ValueError('Contact endpoints must be finite and distinct')
    attrs = dict(name=row['name'], type=kind, rgba=row.get('rgba', '.42 .46 .49 1'))
    if kind == 'box':
        attrs.update(pos=_fmt(np.asarray(row['center_mm']) * .001),
                     size=_fmt(np.asarray(row['half_size_mm']) * .001))
        if 'quat_wxyz' in row:
            attrs['quat'] = _fmt(row['quat_wxyz'])
    elif kind in ('cylinder', 'capsule'):
        attrs.update(fromto=_fmt(np.asarray(row['fromto_mm']) * .001), size=_fmt([row['radius_mm'] * .001]))
    else:
        attrs.update(pos=_fmt(np.asarray(row['center_mm']) * .001), size=_fmt([row['radius_mm'] * .001]))
    return attrs


def _same_numeric_attribute(actual, expected):
    try:
        left = np.asarray(actual.split(), dtype=float)
        right = np.asarray(expected.split(), dtype=float)
    except (AttributeError, TypeError, ValueError):
        return False
    return (left.shape == right.shape and left.size > 0
            and np.isfinite(left).all() and np.isfinite(right).all()
            and np.allclose(left, right, rtol=1e-12, atol=1e-15))


def _encode(element):
    return dict(tag=element.tag, attributes=dict(element.attrib),
                children=[_encode(child) for child in element])


def _decode(node):
    if set(node) != {'tag', 'attributes', 'children'}:
        raise ValueError('Physical tree node requires tag, attributes and children')
    if node['tag'] in {'include', 'plugin'} or 'file' in node['attributes']:
        raise ValueError('Physical source must be self-contained; external files/plugins require an explicit asset resolver')
    element = ET.Element(node['tag'], node['attributes'])
    for child in node['children']:
        element.append(_decode(child))
    return element


def inertia_tensor(record):
    """Reject nonphysical CAD tensors rather than ask MuJoCo to repair them."""
    mass = float(record['mass_kg'])
    com = np.asarray(record['com_m'], float)
    if 'inertia_tensor_kg_m2' in record:
        tensor = np.asarray(record['inertia_tensor_kg_m2'], float)
    else:
        xx, yy, zz, xy, xz, yz = record['fullinertia_mujoco_kg_m2']
        tensor = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
    if (not math.isfinite(mass) or mass <= 0 or com.shape != (3,)
            or not np.isfinite(com).all() or tensor.shape != (3, 3)
            or not np.isfinite(tensor).all() or not np.allclose(tensor, tensor.T, atol=1e-14, rtol=1e-12)):
        raise ValueError('Mass properties require positive finite mass, finite CoM and symmetric full inertia')
    eigen = np.linalg.eigvalsh(tensor)
    if eigen[0] <= 0 or eigen[-1] > sum(eigen[:2]) + 1e-10:
        raise ValueError('Nonphysical CAD inertia tensor')
    return mass, com, tensor


class PhysicalModel:
    """One source tree shared by construction, simulation, replay and export."""

    def __init__(self, root, *, provenance=None):
        self.root = deepcopy(root)
        if self.root.tag != 'mujoco':
            raise ValueError('Expected a MuJoCo physical model')
        _decode(_encode(self.root))  # The XML and JSON entry points have one asset policy.
        names = [item.get('name') for item in self.root.findall('.//body')]
        if any(not name for name in names) or len(set(names)) != len(names):
            raise ValueError('Physical bodies require unique, nonempty names')
        self.provenance = deepcopy(provenance or {})
        self.mass_records = {}
        self.contact_records = []

    @classmethod
    def from_xml(cls, xml, **kwargs):
        if '<!DOCTYPE' in xml or '<!ENTITY' in xml:
            raise ValueError('External XML entities are unsupported')
        return cls(ET.fromstring(xml), **kwargs)

    @classmethod
    def from_source(cls, source):
        if source.get('schema') != 'cybrgeo-physical/1':
            raise ValueError('Expected cybrgeo-physical/1')
        model = cls(_decode(source['physics']), provenance=source['provenance'])
        model.mass_records = deepcopy(source.get('mass_records', {}))
        model.contact_records = deepcopy(source.get('contact_records', []))
        # CAD records are provenance for the physical tree, not a second set
        # of inertias that may silently diverge when the source is edited.
        for name, record in model.mass_records.items():
            mass, com, tensor = inertia_tensor(record)
            body = model.bodies.get(name)
            inertial = None if body is None else body.find('inertial')
            if inertial is None:
                raise ValueError(f'CAD mass record has no physical inertial: {name}')
            expected = np.r_[mass, com, tensor[0, 0], tensor[1, 1], tensor[2, 2],
                             tensor[0, 1], tensor[0, 2], tensor[1, 2]]
            actual = np.fromstring(' '.join(inertial.get(key, '') for key in
                                            ('mass', 'pos', 'fullinertia')), sep=' ')
            if actual.shape != expected.shape or not np.allclose(actual, expected, rtol=1e-12, atol=1e-15):
                raise ValueError(f'CAD mass record differs from physical inertia: {name}')
        model._validate_contact_records()
        return model

    def _validate_contact_records(self):
        """Records describe named geoms; they are not independent model inputs.

        Historical records retain authoring body aliases without serializing
        the alias map. An unresolved label is provenance only; the physical
        XML tree is authoritative for attachment. A label naming an actual
        body (or world/worldbody) must match that geom's immediate parent.
        """
        if not isinstance(self.contact_records, list):
            raise ValueError('Contact records must be a list')
        parents = {child: parent for parent in self.root.iter() for child in parent}
        geoms = {}
        for geom in self.root.iter('geom'):
            geoms.setdefault(geom.get('name'), []).append(geom)
        names = set()
        bodies = self.bodies
        for row in self.contact_records:
            expected = _proxy_attributes(row)
            name = row['name']
            if name in names:
                raise ValueError(f'Duplicate contact record: {name}')
            names.add(name)
            matches = geoms.get(name, [])
            if len(matches) != 1:
                raise ValueError(f'Contact record requires one named physical geom: {name}')
            geom = matches[0]
            parent = parents[geom]
            if parent.tag not in ('body', 'worldbody'):
                raise ValueError(f'Contact record is not attached to a physical body: {name}')
            label = row['body']
            required_parent = self.root.find('worldbody') if label in ('world', 'worldbody') else bodies.get(label)
            if required_parent is not None and parent is not required_parent:
                raise ValueError(f'Contact record differs from physical body attachment: {name}')
            if geom.get('type') != expected['type']:
                raise ValueError(f'Contact record differs from physical geometry type: {name}')
            # Alternative orientation/shape attributes cannot silently change
            # the meaning of otherwise matching size/position values.
            geometry_keys = {'pos', 'size', 'quat', 'fromto', 'axisangle', 'euler', 'xyaxes', 'zaxis', 'mesh', 'hfield'}
            if (set(geom.attrib) & geometry_keys) != (set(expected) & geometry_keys):
                raise ValueError(f'Contact record differs from physical geometry attributes: {name}')
            for key in ('pos', 'size', 'quat', 'fromto', 'rgba'):
                if key in expected and not _same_numeric_attribute(geom.get(key), expected[key]):
                    raise ValueError(f'Contact record differs from physical geometry {key}: {name}')

    @property
    def bodies(self):
        return {item.get('name'): item for item in self.root.findall('.//body')}

    def bind_inertias(self, records, mapping):
        """Bind CAD bodies explicitly; no reliance on render-part enumeration."""
        for physical_name, cad_name in mapping.items():
            body = self.bodies.get(physical_name)
            if body is None or cad_name not in records:
                raise ValueError(f'Missing physical/CAD body binding: {physical_name} -> {cad_name}')
            row = records[cad_name]
            mass, com, tensor = inertia_tensor(row)
            inertial = body.find('inertial')
            if inertial is None:
                inertial = ET.SubElement(body, 'inertial')
            inertial.attrib.clear()
            inertial.attrib.update(mass=f'{mass:.15g}', pos=_fmt(com),
                fullinertia=_fmt([tensor[0, 0], tensor[1, 1], tensor[2, 2],
                                  tensor[0, 1], tensor[0, 2], tensor[1, 2]]))
            self.mass_records[physical_name] = deepcopy(row)

    def set_joint(self, name, **attributes):
        joint = self.root.find(f".//joint[@name='{name}']")
        if joint is None:
            raise ValueError(f'Unknown joint {name}')
        for key, value in attributes.items():
            joint.set(key, value if isinstance(value, str) else _fmt(value))

    def add_proxy(self, row, *, body_aliases=None):
        attrs = _proxy_attributes(row)
        if self.root.find(f".//geom[@name='{row['name']}']") is not None:
            raise ValueError(f'Duplicate contact geometry {row["name"]}')
        body = (body_aliases or {}).get(row.get('body'), row.get('body'))
        container = self.root.find('worldbody') if body in ('world', 'worldbody') else self.bodies.get(body)
        if container is None:
            raise ValueError(f'Unknown body {body!r} for contact geometry {row["name"]!r}')
        ET.SubElement(container, 'geom', **attrs)
        self.contact_records.append(deepcopy(row))

    def remove_geoms(self, *, names=(), prefixes=()):
        removed = []
        for container in [self.root.find('worldbody'), *self.bodies.values()]:
            for geom in list(container.findall('geom')):
                name = geom.get('name', '')
                if name in names or any(name.startswith(prefix) for prefix in prefixes):
                    container.remove(geom)
                    removed.append(name)
        self.contact_records = [r for r in self.contact_records if r['name'] not in removed]
        return removed

    def require_free_body(self, name):
        body = self.bodies.get(name)
        if body is None or (body.find('freejoint') is None and body.find("joint[@type='free']") is None):
            raise ValueError(f'{name} must remain an independent free body')
        for weld in self.root.findall('./equality/weld'):
            if name in (weld.get('body1'), weld.get('body2')):
                raise ValueError(f'{name} may not be welded to a manipulator')

    def to_source(self):
        return dict(schema='cybrgeo-physical/1', physics=_encode(self.root),
                    provenance=deepcopy(self.provenance), mass_records=deepcopy(self.mass_records),
                    contact_records=deepcopy(self.contact_records))

    def xml(self):
        root = deepcopy(self.root)
        ET.indent(root)
        return ET.tostring(root, encoding='unicode') + '\n'

    def write(self, directory):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        source = self.to_source()
        (directory / 'physical-source.json').write_text(json.dumps(source, indent=2), encoding='utf-8')
        (directory / 'model.xml').write_text(self.xml(), encoding='utf-8')
        return digest(source)
