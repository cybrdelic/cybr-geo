"""Validate source-bound, explicitly routed mechanical service load cases.

This validates topology and provenance of a *declared* service load path. It
does not estimate forces, prove capacity, or qualify fabricated hardware.
Callers must supply source cases derived from their authoritative design and
must separately check the signed service envelope and measured capacities.
"""

from collections.abc import Mapping
import math


class LoadPathRouteError(ValueError):
    """A service case or its route does not match the source assembly."""


# Units are part of each mode so a torque cannot silently become a force.
LOAD_MODES = frozenset({
    'force_N', 'axial_force_N', 'radial_force_N', 'shear_force_N',
    'torque_Nm', 'bending_moment_Nm',
})
_CASE_FIELDS = frozenset({'source_part', 'reaction_part', 'load_mode', 'demand'})
_ROUTE_FIELDS = frozenset({'parts', 'interfaces'})
_FORCE_MODES = frozenset(mode for mode in LOAD_MODES if mode.endswith('force_N'))


def _record(value, fields, label):
    if not isinstance(value, Mapping) or set(value) != fields:
        raise LoadPathRouteError(f'{label} must contain exactly {sorted(fields)}')
    return value


def _case(value, label, parts):
    _record(value, _CASE_FIELDS, label)
    source, reaction = value['source_part'], value['reaction_part']
    if (not isinstance(source, str) or not isinstance(reaction, str)
            or source not in parts or reaction not in parts or source == reaction):
        raise LoadPathRouteError(f'{label} needs distinct, declared source and reaction parts')
    if not isinstance(value['load_mode'], str) or value['load_mode'] not in LOAD_MODES:
        raise LoadPathRouteError(f'{label} has an unsupported physical load mode')
    demand = value['demand']
    if (isinstance(demand, bool) or not isinstance(demand, (int, float))
            or not math.isfinite(demand) or demand <= 0):
        raise LoadPathRouteError(f'{label} requires a finite positive service demand')


def _positive(value, label):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value <= 0):
        raise LoadPathRouteError(f'{label} requires a finite positive value')
    return float(value)


def _declared_interface_capacity(name, interface):
    """Screen available scalar force declarations without inventing a fit load."""
    present = {'demand_N', 'design_capacity_N'} & set(interface)
    if not present:
        return None
    if present != {'demand_N', 'design_capacity_N'}:
        raise LoadPathRouteError(f'Interface {name} has an incomplete force/capacity declaration')
    demand = _positive(interface['demand_N'], f'Interface {name} demand')
    capacity = _positive(interface['design_capacity_N'], f'Interface {name} capacity')
    if demand > capacity:
        raise LoadPathRouteError(f'Interface {name} has less declared capacity than demand')
    return demand


def _accounted_nonload_clearances(clearances, interfaces):
    """A source-plan disposition for pairs that must remain separated in use.

    Clearance does not automatically mean non-load-bearing: a bearing with
    running play may still carry reaction. The caller must deliberately name
    each excluded pair in the source plan. Retained contacts can never be
    classified as non-load-bearing clearances.
    """
    if not isinstance(clearances, (set, frozenset, list, tuple)) or any(
            not isinstance(name, str) for name in clearances):
        raise LoadPathRouteError('Nonload clearances must be an explicit source-bound list')
    names = set(clearances)
    if len(names) != len(clearances) or not names <= set(interfaces):
        raise LoadPathRouteError('Nonload clearance inventory has duplicate or unknown interfaces')
    for name in names:
        row = interfaces[name]
        if row.get('role', 'clearance') != 'clearance':
            raise LoadPathRouteError(f'Load-bearing contact {name} cannot be a nonload clearance')
    return names


def _check_modeled_nonload_demands(nonload, modeled_interface_demands):
    if modeled_interface_demands is None:
        return
    if not isinstance(modeled_interface_demands, Mapping):
        raise LoadPathRouteError('Modeled interface demands must be an exact named mapping')
    for name in nonload:
        demand = modeled_interface_demands.get(name, 0.)
        if (isinstance(demand, bool) or not isinstance(demand, (int, float))
                or not math.isfinite(demand) or demand < 0):
            raise LoadPathRouteError(f'Nonload clearance {name} has invalid modeled demand')
        if demand > 0:
            raise LoadPathRouteError(
                f'Nonload clearance {name} actually carries modeled connector or spring load')


def validate_service_case_plan(source_cases, *, parts, interfaces,
                               unloaded_clearances=(), loads=None):
    """Preflight source case and interface classifications before CAD export.

    This checks only authored declarations. Signed service routes and measured
    hardware capacities must still be checked separately at release.
    """
    if not isinstance(source_cases, Mapping) or not source_cases:
        raise LoadPathRouteError('Source-bound service cases must be a nonempty mapping')
    if not isinstance(parts, (set, frozenset, Mapping)) or not parts:
        raise LoadPathRouteError('Declared parts must be a nonempty set or mapping')
    if not isinstance(interfaces, Mapping) or not interfaces:
        raise LoadPathRouteError('Declared interfaces must be a nonempty mapping')
    part_names = set(parts)
    for name, interface in interfaces.items():
        if not isinstance(name, str) or not name:
            raise LoadPathRouteError('Interface names must be nonempty strings')
        if not isinstance(interface, Mapping) or 'parts' not in interface:
            raise LoadPathRouteError(f'Interface {name} lacks declared endpoints')
        pair = interface['parts']
        if (not isinstance(pair, (list, tuple)) or len(pair) != 2
                or any(not isinstance(part, str) for part in pair)
                or pair[0] == pair[1] or not set(pair) <= part_names):
            raise LoadPathRouteError(f'Interface {name} needs two distinct declared endpoints')

    nonload = _accounted_nonload_clearances(unloaded_clearances, interfaces)
    for name, source in source_cases.items():
        if not isinstance(name, str) or not name:
            raise LoadPathRouteError('Service case names must be nonempty strings')
        _case(source, f'Source case {name}', part_names)
    if loads is not None:
        if not isinstance(loads, Mapping) or not set(loads) <= set(source_cases):
            raise LoadPathRouteError('Every active actuator needs a source-bound service case')
        for load_name, load in loads.items():
            case = source_cases[load_name]
            if not isinstance(case, Mapping):
                raise LoadPathRouteError(f'Active load {load_name} needs a source-bound torque case')
            if (not isinstance(load, Mapping) or case.get('load_mode') != 'torque_Nm'
                    or case.get('source_part') != load.get('part')):
                raise LoadPathRouteError(f'Active load {load_name} lacks its physical torque case')
            target = _positive(load.get('demand_Nm'), f'Active load {load_name} torque demand')
            capacity = _positive(load.get('design_capacity_Nm'), f'Active load {load_name} torque capacity')
            sourced = _positive(case.get('demand'), f'Active load {load_name} sourced torque')
            if target > capacity or sourced > target:
                raise LoadPathRouteError(f'Active load {load_name} understates sourced torque demand')
    return nonload


def validate_load_path_routes(cases, *, source_cases, parts, interfaces,
                              unloaded_clearances=(), loads=None,
                              modeled_interface_demands=None):
    """Return the exact interface coverage of signed service-case routes.

    ``source_cases`` maps each source-bound case name to its immutable
    ``source_part``, ``reaction_part``, ``load_mode`` and positive ``demand``.
    ``cases`` repeats that metadata and adds nonempty ``routes``. Each route
    lists simple ordered ``parts`` and their corresponding ``interfaces``.
    Every planned interface must occur on a route or be explicitly classified
    as a non-load-bearing clearance in the source plan. Routes may share
    members when they represent parallel load paths. Scalar force/capacity
    declarations are screened when supplied, but no torque is converted into
    force without a source-grounded lever or transmission model. The caller
    must establish that source cases represent real applied loads and physical
    reactions.
    """
    nonload = validate_service_case_plan(source_cases, parts=parts,
        interfaces=interfaces, unloaded_clearances=unloaded_clearances,
        loads=loads)
    _check_modeled_nonload_demands(nonload, modeled_interface_demands)
    if not isinstance(cases, Mapping) or set(cases) != set(source_cases):
        raise LoadPathRouteError('Signed service cases must exactly match source-bound cases')
    part_names = set(parts)
    covered = set()
    for name, source in source_cases.items():
        signed = cases[name]
        _record(signed, _CASE_FIELDS | {'routes'}, f'Signed case {name}')
        _case({key: signed[key] for key in _CASE_FIELDS}, f'Signed case {name}', part_names)
        if any(signed[key] != source[key] for key in _CASE_FIELDS):
            raise LoadPathRouteError(f'Signed case {name} changes its source-bound load')
        routes = signed['routes']
        if not isinstance(routes, list) or not routes:
            raise LoadPathRouteError(f'Signed case {name} needs at least one route')
        seen_routes = set()
        for index, route in enumerate(routes):
            label = f'Signed case {name} route {index}'
            _record(route, _ROUTE_FIELDS, label)
            members, mates = route['parts'], route['interfaces']
            if (not isinstance(members, list) or not isinstance(mates, list)
                    or len(members) < 2 or len(mates) != len(members) - 1
                    or any(not isinstance(member, str) for member in members)
                    or any(not isinstance(mate, str) for mate in mates)
                    or len(set(members)) != len(members)
                    or len(set(mates)) != len(mates)
                    or members[0] != source['source_part']
                    or members[-1] != source['reaction_part']
                    or not set(members) <= part_names
                    or not set(mates) <= set(interfaces)):
                raise LoadPathRouteError(f'{label} is cyclic, unknown, or lacks the declared reaction')
            identity = (tuple(members), tuple(mates))
            if identity in seen_routes:
                raise LoadPathRouteError(f'{label} duplicates a service route')
            seen_routes.add(identity)
            for first, second, mate in zip(members, members[1:], mates):
                if set(interfaces[mate]['parts']) != {first, second}:
                    raise LoadPathRouteError(f'{label} is disconnected at {mate}')
                force_budget = _declared_interface_capacity(mate, interfaces[mate])
                if (source['load_mode'] in _FORCE_MODES and force_budget is not None
                        and source['demand'] > force_budget):
                    raise LoadPathRouteError(
                        f'{label} exceeds planned force demand at {mate}')
            covered.update(mates)
    if covered & nonload:
        raise LoadPathRouteError('A routed load interface cannot also be a nonload clearance')
    if covered | nonload != set(interfaces):
        missing = sorted(set(interfaces) - covered - nonload)
        raise LoadPathRouteError(f'Signed service routes omit declared interfaces: {missing}')
    return frozenset(covered)


def validate_mechanical_load_paths(paths, *, loads, parts, interfaces,
                                   source_cases=None, unloaded_clearances=(),
                                   modeled_interface_demands=None):
    """Validate old single routes or newer source-bound branched cases.

    Existing signed single-route evidence remains readable. It is accepted
    only in projects without source service cases; it does not gain the stronger
    source/reaction binding of a newly authored case. New projects provide
    ``source_cases`` in their source-bound qualification plan and sign exact
    repeated case metadata plus ``routes``. No fallback to legacy parsing is
    allowed once a source case inventory exists.
    """
    if source_cases is not None:
        return validate_load_path_routes(paths, source_cases=source_cases,
            parts=parts, interfaces=interfaces,
            unloaded_clearances=unloaded_clearances, loads=loads,
            modeled_interface_demands=modeled_interface_demands)
    if not isinstance(loads, Mapping) or not isinstance(paths, Mapping) or set(paths) != set(loads):
        raise LoadPathRouteError('Signed load paths must cover every declared active load')
    if not isinstance(parts, Mapping) or not isinstance(interfaces, Mapping):
        raise LoadPathRouteError('Signed load paths require declared parts and interfaces')
    nonload = _accounted_nonload_clearances(unloaded_clearances, interfaces)
    _check_modeled_nonload_demands(nonload, modeled_interface_demands)
    covered = set()
    for name, route in paths.items():
        _record(route, _ROUTE_FIELDS, f'{name} signed load path')
        members, mates = route['parts'], route['interfaces']
        if (not isinstance(members, list) or not isinstance(mates, list)
                or len(members) < 2 or len(mates) != len(members)-1
                or any(not isinstance(member, str) for member in members)
                or any(not isinstance(mate, str) for mate in mates)
                or len(set(members)) != len(members) or len(set(mates)) != len(mates)
                or members[0] != loads[name].get('part')
                or not set(members) <= set(parts) or not set(mates) <= set(interfaces)):
            raise LoadPathRouteError(f'Signed {name} path lacks a distinct reaction or has invalid members')
        for first, second, mate in zip(members, members[1:], mates):
            if set(interfaces[mate]['parts']) != {first, second}:
                raise LoadPathRouteError(f'Signed {name} path is disconnected at {mate}')
            _declared_interface_capacity(mate, interfaces[mate])
        covered.update(mates)
    if covered & nonload:
        raise LoadPathRouteError('A routed load interface cannot also be a nonload clearance')
    if covered | nonload != set(interfaces):
        raise LoadPathRouteError('Signed load paths omit a declared load-bearing interface')
    return frozenset(covered)
