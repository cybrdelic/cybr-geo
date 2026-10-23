"""Lossless, bounded transport for the planner; the CAD recipe contract is unchanged."""
import copy
import math

KINDS = ('box', 'cylinder', 'sphere')


def planner_schema(canonical):
    schema = copy.deepcopy(canonical)
    # Mechanical plans use the shared constructive compiler before this codec.
    # Keep the full recipe schema for imports, but inference may emit static parts
    # only; it cannot independently invent a mechanism's joints or drive ratios.
    schema['properties']['joints']['maxItems'] = 0
    primitive = {'type': 'array', 'items': {'type': 'number'}, 'minItems': 10, 'maxItems': 10}
    part = schema['properties']['parts']['items']['properties']
    part['body'] = copy.deepcopy(primitive)
    for field in ('cuts', 'unions'):
        part[field]['items'] = copy.deepcopy(primitive)
    schema['properties']['copies'] = {
        'type': 'array', 'maxItems': 31, 'items': {
            'type': 'object', 'additionalProperties': False,
            'properties': {'source': {'type': 'string'}, 'name': {'type': 'string'},
                           'offset': {'type': 'array', 'items': {'type': 'number'}, 'minItems': 3, 'maxItems': 3}},
            'required': ['source', 'name', 'offset']}}
    schema['required'].append('copies')
    return schema


def primitive(value):
    if not isinstance(value, list) or len(value) != 10 or any(
            isinstance(n, bool) or not isinstance(n, (int, float)) or not math.isfinite(n)
            or abs(n) > 10000 for n in value):
        raise ValueError('Invalid compact CAD primitive')
    kind = value[0]
    if kind not in (0, 1, 2) or any(n <= 0 for n in value[1:4]):
        raise ValueError('Invalid compact CAD dimensions or kind')
    return {'kind': KINDS[int(kind)], 'size': value[1:4], 'position': value[4:7], 'rotation': value[7:10]}


def decode(wire):
    plan = copy.deepcopy(wire)
    parts = plan.get('parts')
    copies = plan.pop('copies', [])
    if not isinstance(parts, list) or not isinstance(copies, list) or not 1 <= len(parts) + len(copies) <= 32:
        raise ValueError('Expected 1–32 CAD parts including copies')
    names = {}
    for part in parts:
        name = part.get('name')
        if not isinstance(name, str) or name in names:
            raise ValueError('Duplicate or missing CAD part name')
        part['body'] = primitive(part['body'])
        for field, limit in (('cuts', 48), ('unions', 24)):
            if not isinstance(part[field], list) or len(part[field]) > limit:
                raise ValueError('CAD operation budget exceeded')
            part[field] = [primitive(item) for item in part[field]]
        names[name] = part
    for item in copies:
        source, name, offset = item['source'], item['name'], item['offset']
        if source not in names or not isinstance(name, str) or name in names:
            raise ValueError('Invalid CAD copy reference or name')
        if not isinstance(offset, list) or len(offset) != 3 or any(
                isinstance(n, bool) or not isinstance(n, (int, float)) or not math.isfinite(n)
                or abs(n) > 10000 for n in offset):
            raise ValueError('Invalid CAD copy translation')
        part = copy.deepcopy(names[source])
        part['name'] = name
        for shape in [part['body'], *part['cuts'], *part['unions']]:
            shape['position'] = [a+b for a, b in zip(shape['position'], offset)]
            if any(abs(n) > 10000 for n in shape['position']):
                raise ValueError('Translated CAD part exceeds size limit')
        parts.append(part)
        names[name] = part
    return plan


def encode(plan):
    wire = copy.deepcopy(plan)
    for part in wire['parts']:
        def pack(shape):
            return [KINDS.index(shape['kind']), *shape['size'], *shape['position'], *shape['rotation']]
        part['body'] = pack(part['body'])
        for field in ('cuts', 'unions'):
            part[field] = [pack(item) for item in part[field]]
    wire['copies'] = []
    return wire
