"""Route mechanical intent into paired construction, before primitive CAD planning.

The supported architectures are original studies. They are not reconstructions of
IRIS-09 or either ARX-6 reference package. The compiler owns all dimensions shared
by mating parts, joint pivots, motion and transmission relationships.
"""
from __future__ import annotations

import math
from pathlib import Path
import re
import sys


ARCHITECTURES = ('compact_arm', 'belt_arm', 'inspection_head')
_MECHANICAL = re.compile(
    r'\b(?:mechanical|mechanism|robot(?:ic)?|arm|manipulator|actuator|gripper|'
    r'gear(?:ed|s)?|belt|pulley|motor|engine|linkage|piston|tendon|gimbal|'
    r'pan[ -]?tilt|iris|arx(?:[ -]?6)?|hinge|drawer|slider|sliding|'
    r'rotating|articulated|moving|servo|clutch|pendulum|crank|valve)\b', re.I)
_ARM = re.compile(r'\b(?:arm|manipulator)\b|\bpick[ -]and[ -]place\b', re.I)
_HEAD = re.compile(r'\binspection[ -](?:head|wrist)\b|\bpan[ -]?tilt\b|\bgimbal\b', re.I)
_STATIC = re.compile(r'\b(?:bracket|flange|enclosure|cover|mounting plate)\b', re.I)


def _request(architecture, parameters):
    if architecture not in ARCHITECTURES:
        raise ValueError('Choose compact_arm, belt_arm or inspection_head.')
    if parameters is None:
        parameters = {}
    if not isinstance(parameters, dict) or len(parameters) > 16:
        raise ValueError('Mechanical parameters must be a bounded JSON object.')
    for name, value in parameters.items():
        if (not isinstance(name, str) or isinstance(value, bool)
                or not isinstance(value, (int, float)) or not math.isfinite(value)):
            raise ValueError('Mechanical parameters must have finite numeric values.')
    return {'architecture': architecture, 'parameters': dict(parameters)}


def mechanical_request(prompt, mechanism=None):
    """Return a compiler request, None for static CAD, or reject unsupported motion.

    Explicit structured parameters are passed to the compiler, whose domain
    validation is authoritative. Prose dimensional requirements are never silently
    discarded. Unknown mechanisms cannot fall through to arbitrary solid generation.
    """
    if not isinstance(prompt, str) or len(prompt) > 6000:
        raise ValueError('Use a design request of at most 6000 characters.')
    if mechanism is not None:
        if (not isinstance(mechanism, dict)
                or set(mechanism) - {'architecture', 'parameters'}):
            raise ValueError('A mechanism request contains architecture and parameters only.')
        return _request(mechanism.get('architecture'), mechanism.get('parameters'))
    text = prompt.strip()
    slug = text.lower().replace('-', '_').replace(' ', '_')
    if slug in ARCHITECTURES:
        return _request(slug, {})
    if not text or text.lower() in ('default', 'default assembly', 'mechanical assembly'):
        return _request('compact_arm', {})
    if re.search(r'\b(?:iris(?:[ -]?09)?|arx(?:[ -]?6)?)\b', text, re.I):
        raise ValueError('IRIS-09 and ARX-6 are separate reference designs, not included '
                         'reconstructions. Choose an original compact_arm, belt_arm or '
                         'inspection_head study explicitly.')
    # "A bracket for a robot arm" asks for the static bracket, whereas "an arm
    # with a bracket" selects the arm architecture. Keep the requested subject.
    static = _STATIC.search(text)
    mechanical = _MECHANICAL.search(text)
    if static and (mechanical is None or static.start() < mechanical.start()):
        return None
    architecture = None
    if _HEAD.search(text):
        architecture = 'inspection_head'
    elif _ARM.search(text):
        architecture = 'belt_arm' if re.search(r'\bbelt\b', text, re.I) else 'compact_arm'
    if architecture:
        if re.search(r'\d(?:\.\d+)?\s*(?:mm|cm|m|inches|inch|degrees|deg|kg|n|nm)\b', text, re.I):
            raise ValueError('Use mechanism.parameters for dimensional or motion changes; '
                             'prose dimensions are not silently substituted with defaults. '
                             'See studio/README.md for the bounded parameter contract.')
        return _request(architecture, {})
    if _MECHANICAL.search(text):
        raise ValueError('That mechanical architecture is not supported yet. Choose a '
                         'robot arm, belt arm or inspection head, or provide a structured '
                         'mechanism request. Mechanical requests never use arbitrary solids.')
    return None


def compile_request(request, prompt=''):
    """Compile without inference, credentials, network access or generated code."""
    # An installed package is preferred; the source checkout is the supported
    # development fallback for `python studio/codex_studio.py`.
    source = Path(__file__).resolve().parents[1] / 'src'
    if source.is_dir() and str(source) not in sys.path:
        sys.path.insert(0, str(source))
    from mechanism_lab.construction import compile_plan
    plan = compile_plan(request['architecture'], request['parameters'])
    if not isinstance(plan, dict) or not plan.get('parts') or not plan.get('construction'):
        raise RuntimeError('The construction compiler did not return an assembly contract.')
    plan['generation'] = {
        'mode': 'constructive',
        'architecture': request['architecture'],
        'parameters': dict(request['parameters']),
        'prompt': prompt,
        'inference_used': False,
        'reference_reproduction': False,
        'interpretation': ('The prompt selects an original architecture; only the '
                           'structured parameters customize its geometry. Additional '
                           'requested features are not synthesized by this preset route.'),
    }
    return plan
