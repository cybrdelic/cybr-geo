"""Explicit registration boundary for reusable, trusted Python components.

JSON sources select installed components; they cannot import code or execute
expressions. A validator returns independent (actuator ID, joint kind) pairs.
An expander receives the construction builder, source instance and parent output
datum, and returns input/output part IDs, output_datum, actuator bindings and
capture probes. It must construct actual interfaces, not claim certification.
"""
from __future__ import annotations

import re

_COMPONENTS = {}


def register_component(name, *, validator, expander, revision, provenance, root_allowed=False):
    """Register a trusted component implementation; existing names cannot be replaced."""
    if not isinstance(name, str) or not re.fullmatch(r'[a-z][a-z0-9_]{0,63}', name):
        raise ValueError('Component type must be a portable lowercase identifier')
    if name in _COMPONENTS:
        raise ValueError(f'Component {name!r} is already registered')
    if (not callable(validator) or not callable(expander)
            or not isinstance(revision, str) or not revision
            or not isinstance(provenance, str) or not provenance
            or not isinstance(root_allowed, bool)):
        raise ValueError('Component registration requires validator, expander, revision and provenance')
    _COMPONENTS[name] = dict(validator=validator, expander=expander, revision=revision,
                             provenance=provenance, root_allowed=bool(root_allowed))


def component_definition(name):
    if not isinstance(name, str) or name not in _COMPONENTS:
        raise ValueError(f'Unsupported reusable component {name!r}')
    return dict(_COMPONENTS[name])


def component_catalog():
    """Return serializable component capability identities, not executable hooks."""
    return {name: {key: value for key, value in definition.items()
                   if key not in ('validator', 'expander')}
            for name, definition in _COMPONENTS.items()}
