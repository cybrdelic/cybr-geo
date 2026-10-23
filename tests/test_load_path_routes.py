"""Small topology tests for source-bound branched service load paths."""

from copy import deepcopy

import pytest

from mechanism_lab.load_path_routes import (
    LoadPathRouteError, validate_load_path_routes, validate_mechanical_load_paths,
    validate_service_case_plan,
)


def _network():
    parts = {name: {} for name in ('output', 'left', 'right', 'frame', 'keeper')}
    interfaces = {
        'output_left': {'parts': ['output', 'left']},
        'left_frame': {'parts': ['left', 'frame']},
        'output_right': {'parts': ['output', 'right']},
        'right_frame': {'parts': ['right', 'frame']},
        'output_keeper': {'parts': ['output', 'keeper']},
    }
    source_cases = {
        'output_torque': {'source_part': 'output', 'reaction_part': 'frame',
                          'load_mode': 'torque_Nm', 'demand': 1.0},
        'axial_retention': {'source_part': 'output', 'reaction_part': 'keeper',
                            'load_mode': 'axial_force_N', 'demand': 3.0},
    }
    cases = {
        name: {**source, 'routes': routes}
        for (name, source), routes in zip(source_cases.items(), (
            [
                {'parts': ['output', 'left', 'frame'],
                 'interfaces': ['output_left', 'left_frame']},
                {'parts': ['output', 'right', 'frame'],
                 'interfaces': ['output_right', 'right_frame']},
            ],
            [{'parts': ['output', 'keeper'], 'interfaces': ['output_keeper']}],
        ))
    }
    return parts, interfaces, source_cases, cases


def _validate(parts, interfaces, source_cases, cases):
    return validate_load_path_routes(cases, source_cases=source_cases,
                                     parts=parts, interfaces=interfaces)


def test_parallel_routes_and_separate_retention_case_cover_exact_interfaces():
    network = _network()
    assert _validate(*network) == frozenset(network[1])


@pytest.mark.parametrize('fault', [
    'missing_branch', 'unknown_interface', 'disconnected_edge', 'cycle',
    'wrong_source', 'wrong_reaction', 'duplicate_route', 'empty_routes',
    'missing_case', 'invented_case', 'changed_source_binding',
    'changed_demand', 'unsupported_mode', 'zero_demand', 'nonfinite_demand',
])
def test_invalid_service_routes_fail_closed(fault):
    parts, interfaces, source_cases, cases = _network()
    cases = deepcopy(cases)
    torque = cases['output_torque']
    if fault == 'missing_branch':
        torque['routes'].pop()
    elif fault == 'unknown_interface':
        torque['routes'][1]['interfaces'][0] = 'imaginary_joint'
    elif fault == 'disconnected_edge':
        torque['routes'][1]['interfaces'] = ['output_left', 'right_frame']
    elif fault == 'cycle':
        torque['routes'][1] = {'parts': ['output', 'right', 'output', 'frame'],
                               'interfaces': ['output_right', 'output_right', 'left_frame']}
    elif fault == 'wrong_source':
        torque['routes'][0]['parts'] = ['left', 'output', 'frame']
    elif fault == 'wrong_reaction':
        torque['routes'][0] = {'parts': ['output', 'left'], 'interfaces': ['output_left']}
    elif fault == 'duplicate_route':
        torque['routes'].append(deepcopy(torque['routes'][0]))
    elif fault == 'empty_routes':
        torque['routes'] = []
    elif fault == 'missing_case':
        cases.pop('axial_retention')
    elif fault == 'invented_case':
        cases['unbound_retention'] = deepcopy(cases['axial_retention'])
    elif fault == 'changed_source_binding':
        torque['source_part'] = 'right'
    elif fault == 'changed_demand':
        torque['demand'] = 2.0
    elif fault == 'unsupported_mode':
        torque['load_mode'] = 'imaginary_load'
    elif fault == 'zero_demand':
        torque['demand'] = 0.0
    elif fault == 'nonfinite_demand':
        torque['demand'] = float('inf')
    with pytest.raises(LoadPathRouteError):
        _validate(parts, interfaces, source_cases, cases)


def test_unrepresented_keeper_cannot_hide_in_torque_path():
    parts, interfaces, source_cases, cases = _network()
    source_cases.pop('axial_retention')
    cases.pop('axial_retention')
    with pytest.raises(LoadPathRouteError, match='omit declared interfaces'):
        _validate(parts, interfaces, source_cases, cases)


def test_reaction_must_be_a_distinct_declared_part():
    parts, interfaces, source_cases, cases = _network()
    source_cases['output_torque']['reaction_part'] = 'output'
    cases['output_torque']['reaction_part'] = 'output'
    with pytest.raises(LoadPathRouteError, match='distinct'):
        _validate(parts, interfaces, source_cases, cases)


def test_invalid_interface_inventory_fails_before_route_coverage():
    parts, interfaces, source_cases, cases = _network()
    interfaces['left_frame']['parts'] = ['left', 'missing_part']
    with pytest.raises(LoadPathRouteError, match='declared endpoints'):
        _validate(parts, interfaces, source_cases, cases)


def test_source_bound_branches_account_for_noncontact_clearance_without_routing_force():
    parts, interfaces, source_cases, cases = _network()
    interfaces['optical_gap'] = {'parts': ['left', 'right'],
                                 'nominal_clearance_mm': 2.,
                                 'required_clearance_mm': 1.,
                                 'demand_N': 1., 'design_capacity_N': 2.}
    for interface in interfaces.values():
        interface.setdefault('demand_N', 4.)
        interface.setdefault('design_capacity_N', 5.)
    loads = {'output_torque': {'part': 'output', 'demand_Nm': 1.,
                               'design_capacity_Nm': 1.5}}
    covered = validate_mechanical_load_paths(cases, loads=loads, parts=parts,
        interfaces=interfaces, source_cases=source_cases,
        unloaded_clearances=['optical_gap'])
    assert covered == frozenset(set(interfaces) - {'optical_gap'})
    with pytest.raises(LoadPathRouteError, match='actually carries modeled connector or spring load'):
        validate_mechanical_load_paths(cases, loads=loads, parts=parts,
            interfaces=interfaces, source_cases=source_cases,
            unloaded_clearances=['optical_gap'],
            modeled_interface_demands={'optical_gap': .2})
    # The disposition comes from the source plan; it cannot relabel a retained
    # contact or conceal a pair already named on a structural load route.
    interfaces['optical_gap']['role'] = 'tested_retention_contact'
    with pytest.raises(LoadPathRouteError, match='cannot be a nonload clearance'):
        validate_mechanical_load_paths(cases, loads=loads, parts=parts,
            interfaces=interfaces, source_cases=source_cases,
            unloaded_clearances=['optical_gap'])
    interfaces['optical_gap'].pop('role')
    with pytest.raises(LoadPathRouteError, match='also be a nonload clearance'):
        validate_load_path_routes(cases, source_cases=source_cases,
            parts=parts, interfaces=interfaces,
            unloaded_clearances=['output_left', 'optical_gap'])


def test_force_case_screens_each_routed_interface_without_unit_converting_torque():
    parts, interfaces, source_cases, cases = _network()
    for interface in interfaces.values():
        interface.update(demand_N=5., design_capacity_N=6.)
    assert _validate(parts, interfaces, source_cases, cases) == frozenset(interfaces)
    interfaces['output_keeper']['demand_N'] = 2.
    with pytest.raises(LoadPathRouteError, match='planned force demand'):
        _validate(parts, interfaces, source_cases, cases)
    interfaces['output_keeper']['demand_N'] = 5.
    interfaces['output_left']['demand_N'] = .1
    # A 1 Nm torque is not 1 N at the interface; the signed service envelope
    # must independently supply a physical interface-force demand.
    assert _validate(parts, interfaces, source_cases, cases) == frozenset(interfaces)
    interfaces['output_left']['design_capacity_N'] = .05
    with pytest.raises(LoadPathRouteError, match='less declared capacity'):
        _validate(parts, interfaces, source_cases, cases)


def test_legacy_single_route_stays_readable_but_cannot_masquerade_as_source_cases():
    parts = {'motor': {}, 'shaft': {}, 'frame': {}}
    interfaces = {'coupling': {'parts': ['motor', 'shaft']},
                  'bearing': {'parts': ['shaft', 'frame']}}
    loads = {'drive': {'part': 'motor'}}
    legacy = {'drive': {'parts': ['motor', 'shaft', 'frame'],
                        'interfaces': ['coupling', 'bearing']}}
    assert validate_mechanical_load_paths(legacy, loads=loads, parts=parts,
        interfaces=interfaces) == frozenset(interfaces)
    sourced = {'drive': {'source_part': 'motor', 'reaction_part': 'frame',
                         'load_mode': 'torque_Nm', 'demand': 1.}}
    with pytest.raises(LoadPathRouteError):
        validate_mechanical_load_paths(legacy, loads=loads, parts=parts,
            interfaces=interfaces, source_cases=sourced)
    branched = {'drive': {**sourced['drive'], 'routes': [legacy['drive']]}}
    loads['drive'].update(demand_Nm=1., design_capacity_Nm=2.)
    assert validate_mechanical_load_paths(branched, loads=loads, parts=parts,
        interfaces=interfaces, source_cases=sourced) == frozenset(interfaces)
    loads['drive']['demand_Nm'] = .5
    with pytest.raises(LoadPathRouteError, match='understates sourced torque'):
        validate_mechanical_load_paths(branched, loads=loads, parts=parts,
            interfaces=interfaces, source_cases=sourced)


def test_source_service_cases_preflight_before_any_signed_route_exists():
    parts, interfaces, source_cases, _ = _network()
    assert validate_service_case_plan(source_cases, parts=parts,
        interfaces=interfaces) == set()
    source_cases['output_torque']['reaction_part'] = 'invented_frame'
    with pytest.raises(LoadPathRouteError, match='distinct, declared'):
        validate_service_case_plan(source_cases, parts=parts,
            interfaces=interfaces)
