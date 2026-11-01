"""A shared phase clock changes time, not geometry, forces or acceptance rules."""
from copy import deepcopy
import hashlib
import math
from pathlib import Path

import pytest

from mechanism_lab import reference_clock
from mechanism_lab.reference_clock import PhaseClock


def quintic_reference(time_s, parameters):
    """Periodic outward/return quintic, zero velocity and acceleration at joins."""
    local = time_s % 2.
    u, sign, start = (local, 1., 0.) if local < 1. else (local-1., -1., 1.)
    amplitude = parameters['amplitude']
    q = start + sign*(10*u**3 - 15*u**4 + 6*u**5)
    qd = sign*(30*u**2 - 60*u**3 + 30*u**4)
    qdd = sign*(60*u - 180*u**2 + 120*u**3)
    return dict(q=[amplitude*q], qd=[amplitude*qd], qdd=[amplitude*qdd],
                stage=int(local >= 1), auxiliary=parameters['auxiliary'])


def bind(clock=None, reference=quintic_reference, parameters=None, **changes):
    options = dict(probe_s=1e-7, position_tolerance=1e-10,
                   velocity_tolerance=1e-9, acceleration_tolerance=2e-5)
    options.update(changes)
    return (clock or PhaseClock((0, 1, 2), (0, 2, 5))).bind(reference,
        parameters or dict(amplitude=2., auxiliary=dict(gap=.03, names=['one'])), **options)


def test_exact_landmarks_inverse_negative_and_later_cycles():
    clock = PhaseClock([0, 1, 2], [0, 2, 5])
    assert clock.source_period_s == 2 and clock.physical_period_s == 5
    assert clock.rates == (.5, 1/3)
    for source, physical in ((0, 0), (1, 2), (2, 5), (-1, -3), (-2, -5),
                             (5, 12), (8, 20), (.5, 1), (1.5, 3.5)):
        assert clock.physical_time(source) == pytest.approx(physical)
        assert clock.source_time(physical) == pytest.approx(source)
    for physical in (-17.342, -5.01, -.0001, 0., 1.999, 2., 3.9, 5., 72.64):
        assert clock.physical_time(clock.source_time(physical)) == pytest.approx(physical)
    assert clock.rate(-1.) == 1/3
    assert clock.rate(2.) == 1/3
    assert clock.rate(5.) == .5


def test_quintic_chain_rule_inside_intervals_and_repeated_cycle():
    clock = PhaseClock((0, 1, 2), (0, 2, 5))
    parameters = dict(amplitude=2., auxiliary={'gap': .03})
    transformed = bind(clock, parameters=parameters)
    for physical in (.3, 1.1, 2.3, 4.7, -.3, 8.2):
        expected = quintic_reference(clock.source_time(physical), parameters)
        actual = transformed(physical)
        assert actual['q'] == pytest.approx(expected['q'])
        assert actual['qd'] == pytest.approx([x*clock.rate(physical) for x in expected['qd']])
        assert actual['qdd'] == pytest.approx([x*clock.rate(physical)**2 for x in expected['qdd']])
        assert actual['stage'] == expected['stage'] and actual['auxiliary'] == parameters['auxiliary']
        delta = 1e-5
        before, after = transformed(physical-delta), transformed(physical+delta)
        numeric_velocity = (after['q'][0]-before['q'][0])/(2*delta)
        numeric_acceleration = (after['qd'][0]-before['qd'][0])/(2*delta)
        assert actual['qd'][0] == pytest.approx(numeric_velocity, abs=2e-8)
        assert actual['qdd'][0] == pytest.approx(numeric_acceleration, abs=3e-8)
    for physical in (0, 2, 5, -5):
        assert transformed(physical)['qd'] == [0.]
        assert transformed(physical)['qdd'] == [0.]
        assert transformed(physical-1e-7)['q'] == pytest.approx(transformed(physical+1e-7)['q'], abs=1e-12)


def test_binding_does_not_mutate_caller_or_callback_data():
    parameters = dict(amplitude=2., auxiliary=dict(gap=.03, names=['one']))
    original = deepcopy(parameters)
    transformed = bind(parameters=parameters)
    result = transformed(.5)
    result['auxiliary']['names'].append('two')
    result['q'][0] = 700
    assert parameters == original
    assert transformed(.5)['auxiliary']['names'] == ['one']
    # Later caller edits do not alter the already validated callback parameters.
    parameters['amplitude'] = 30
    assert transformed(.5)['q'] == pytest.approx(quintic_reference(.25, original)['q'])


def test_receipt_binds_exact_knots_helper_bytes_and_join_measurements():
    clock = PhaseClock((0, 1, 2), (0, 2, 5))
    transformed = bind(clock)
    source = clock.source()
    assert source['source_times_s'] == [0., 1., 2.]
    assert source['physical_times_s'] == [0., 2., 5.]
    assert source['implementation_sha256'] == hashlib.sha256(Path(reference_clock.__file__).read_bytes()).hexdigest()
    assert clock.identity() == PhaseClock([0, 1, 2], [0, 2, 5]).identity()
    assert clock.identity() != PhaseClock([0, 1, 2], [0, 2, 6]).identity()
    validation = transformed.validation_report
    assert validation['passed'] and len(validation['joins']) == 2
    assert all(len(row['measurements']) == 2 for row in validation['joins'])
    assert all(row['rate_changes'] for row in validation['joins'])
    assert validation['joins'][0]['measurements'][0]['maximum_absolute_acceleration'] > 0
    assert transformed.source()['clock'] == source
    assert PhaseClock.from_source(source) == clock
    for key, replacement in (('schema', 'old'), ('rates', [1., 1.]),
                             ('implementation_sha256', '0'*64), ('physical_period_s', 9.)):
        corrupted = deepcopy(source)
        corrupted[key] = replacement
        with pytest.raises(ValueError, match='identity differs'):
            PhaseClock.from_source(corrupted)
    validation['passed'] = False
    assert transformed.validation_report['passed']


@pytest.mark.parametrize('source,physical', [([], []), ([0], [0]), ([0, 1], [0, 1, 2]),
    ([1, 2], [0, 1]), ([0, 1], [1, 2]), ([0, 1, 1], [0, 1, 2]),
    ([0, 1, 2], [0, 2, 1]), ([0, -1], [0, 1]), ([0, True], [0, 1]),
    ([0, math.nan], [0, 1]), ([0, 1], [0, math.inf]), ('01', [0, 1]),
    ([0, 1e200], [0, 1e-100]), ([0, 1e-200], [0, 1e100])])
def test_invalid_or_unrepresentable_knots_fail(source, physical):
    with pytest.raises(ValueError):
        PhaseClock(source, physical)


@pytest.mark.parametrize('time', [True, math.nan, math.inf, '1', None])
def test_nonfinite_or_nonnumeric_time_fails(time):
    clock = PhaseClock((0, 1, 2), (0, 2, 5))
    for operation in (clock.source_time, clock.physical_time, clock.rate, bind(clock)):
        with pytest.raises(ValueError):
            operation(time)


@pytest.mark.parametrize('options', [dict(probe_s=0), dict(probe_s=.25), dict(probe_s=True),
    dict(position_tolerance=math.nan), dict(velocity_tolerance=-1), dict(acceleration_tolerance=0)])
def test_explicit_probe_and_tolerances_are_required_and_bounded(options):
    with pytest.raises(ValueError):
        bind(**options)
    with pytest.raises(TypeError):
        PhaseClock((0, 1), (0, 1)).bind(quintic_reference, {})


def test_nonstationary_rate_change_rejected_even_for_continuous_periodic_curve():
    def sinusoid(t, p):
        return dict(q=[math.sin(math.pi*t)], qd=[math.pi*math.cos(math.pi*t)],
                    qdd=[-math.pi**2*math.sin(math.pi*t)])
    with pytest.raises(ValueError, match='nonstationary'):
        bind(reference=sinusoid)
    # An unchanged clock rate permits moving joins and still checks continuity.
    same_rate = PhaseClock((0, .5, 2), (0, 1, 4))
    assert bind(same_rate, reference=sinusoid, velocity_tolerance=1e-5,
                acceleration_tolerance=1e-5)(1)['q'] == pytest.approx([1.])


def test_nonzero_stationary_acceleration_is_not_a_safe_rate_change():
    def cosine(t, p):
        return dict(q=[math.cos(math.pi*t)], qd=[-math.pi*math.sin(math.pi*t)],
                    qdd=[-math.pi**2*math.cos(math.pi*t)])
    with pytest.raises(ValueError, match='nonstationary'):
        bind(reference=cosine, velocity_tolerance=1e-5)


@pytest.mark.parametrize('derivative', ['qd', 'qdd'])
@pytest.mark.parametrize('knot', [0., 1.])
def test_unchanged_rate_rejects_exact_knot_derivative_spikes(derivative, knot):
    clock = PhaseClock((0, 1, 2), (0, 1, 2))

    def spike(t, parameters):
        result = dict(q=[0.], qd=[0.], qdd=[0.])
        # For the periodic join, keep the explicit endpoint equal to zero's
        # center. Only center-to-side comparisons can detect this spike.
        if t == knot or (knot == 0 and t == 2):
            result[derivative] = [7.]
        return result

    with pytest.raises(ValueError, match='discontinuous'):
        clock.bind(spike, {}, probe_s=1e-6, position_tolerance=1e-10,
                   velocity_tolerance=1e-10, acceleration_tolerance=1e-10)


def test_position_discontinuity_and_nonperiodic_endpoint_are_rejected():
    def jump(t, p):
        result = quintic_reference(t, p)
        result['q'][0] += .1 if 1 <= t < 2 else 0
        return result
    with pytest.raises(ValueError, match='discontinuous'):
        bind(reference=jump)

    def not_periodic(t, p):
        return dict(q=[t], qd=[1.], qdd=[0.])
    with pytest.raises(ValueError, match='period endpoints'):
        bind(PhaseClock((0, 2), (0, 2)), reference=not_periodic)


@pytest.mark.parametrize('bad', [dict(q=[0.], qd=[], qdd=[0.]),
    dict(q=[0.], qd=[0., 1.], qdd=[0.]), dict(q=[False], qd=[0.], qdd=[0.]),
    dict(q=[0.], qd=[math.nan], qdd=[0.]), dict(q=[0.], qd=[0.]),
    dict(q=[0.], qd=1., qdd=[0.]), [], dict(q='a', qd=[0.], qdd=[0.])])
def test_invalid_callback_vectors_are_rejected_before_binding(bad):
    with pytest.raises(ValueError):
        bind(reference=lambda t, p: bad)


def test_each_sample_checks_shape_and_coordinate_count_after_binding():
    def malformed_interior(t, p):
        result = quintic_reference(t, p)
        if .4 < t < .6:
            result['qdd'] = [1., 2.]
        return result
    transformed = bind(reference=malformed_interior)
    with pytest.raises(ValueError, match='identical coordinate counts'):
        transformed(1.)

    def different_interior(t, p):
        result = quintic_reference(t, p)
        if .4 < t < .6:
            for key in ('q', 'qd', 'qdd'):
                result[key] *= 2
        return result
    transformed = bind(reference=different_interior)
    with pytest.raises(ValueError, match='count changed'):
        transformed(1.)
