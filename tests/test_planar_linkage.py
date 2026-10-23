"""The same analytic closure must support whole-interval motion bounds."""
import json
import math
import random

import pytest

from mechanism_lab.planar_linkage import PlanarFourBar


def _separation(first, second):
    return math.dist(first, second)


def _wrapped_difference(first, second):
    return math.atan2(math.sin(first - second), math.cos(first - second))


def _cross(left, right):
    return left[0] * right[1] - left[1] * right[0]


def test_authored_fourbar_closes_and_bounds_full_crank_turn():
    fourbar = PlanarFourBar((0, 0), (100, 0), 25, 90, 60)
    bound = fourbar.angular_rate_bounds(0, math.tau)
    assert json.loads(json.dumps(bound)) == bound
    point_bound = fourbar.point_motion_bound('coupler', 20., 0, math.tau)
    assert json.loads(json.dumps(point_bound)) == point_bound
    assert bound['source_sha256'] == fourbar.source_sha256
    assert bound['output_pivot_distance_interval_mm'] == pytest.approx((75, 125))
    assert bound['minimum_coupler_rocker_sine_lower_bound'] == pytest.approx(
        math.sqrt(1 - .5625 ** 2), rel=1e-10)
    assert bound['maximum_rocker_angular_rate_rad_per_input_rad'] < .505
    assert bound['maximum_coupler_angular_rate_rad_per_input_rad'] < .337
    for sample in range(97):
        pose = fourbar.solve(math.tau * sample / 96)
        a, b = pose['crank_pin_mm'], pose['rocker_pin_mm']
        assert _separation(fourbar.input_pivot_mm, a) == pytest.approx(25)
        assert _separation(a, b) == pytest.approx(90)
        assert _separation(fourbar.output_pivot_mm, b) == pytest.approx(60)
        if sample < 96:
            next_pose = fourbar.solve(math.tau * sample / 96 + 1e-7)
            for angle, limit in (
                ('rocker_angle_rad', 'maximum_rocker_angular_rate_rad_per_input_rad'),
                ('coupler_angle_rad', 'maximum_coupler_angular_rate_rad_per_input_rad'),
            ):
                rate = abs(_wrapped_difference(next_pose[angle], pose[angle])) / 1e-7
                assert rate <= bound[limit] * (1 + 1e-5)


@pytest.mark.parametrize('branch', [-1, 1])
def test_translated_rotated_pivots_phase_and_branch(branch):
    bearing_angle = .73
    pivot = (-123., 47.)
    fixed = (pivot[0] + 100 * math.cos(bearing_angle),
             pivot[1] + 100 * math.sin(bearing_angle))
    fourbar = PlanarFourBar(pivot, fixed, 25, 90, 60, input_phase_rad=-.41,
                           branch=branch)
    bound = fourbar.angular_rate_bounds(-1.2, 4.7)
    assert bound['output_pivot_distance_interval_mm'][0] >= 75 - 1e-10
    assert bound['output_pivot_distance_interval_mm'][1] <= 125 + 1e-10
    for input_angle in (-1.2, -.3, .6, 2.1, 4.7):
        pose = fourbar.solve(input_angle)
        a, b = pose['crank_pin_mm'], pose['rocker_pin_mm']
        ray = (fixed[0] - a[0], fixed[1] - a[1])
        flank = (b[0] - a[0], b[1] - a[1])
        assert branch * _cross(ray, flank) > 0
        assert _separation(a, b) == pytest.approx(90)
        assert _separation(fixed, b) == pytest.approx(60)
        assert pose['crank_angle_rad'] == pytest.approx(input_angle - .41)


def test_randomized_finite_differences_respect_interval_bounds():
    rng = random.Random(82031)
    accepted = 0
    for _ in range(80):
        ground = rng.uniform(55, 130)
        crank = rng.uniform(5, 35)
        # Every crank angle is strictly inside the two-circle reach annulus.
        inner_distance = abs(ground - crank)
        outer_distance = ground + crank
        coupler = rng.uniform(outer_distance * .72, outer_distance * .95)
        rocker_min = max(outer_distance - coupler + 2, coupler - inner_distance + 2)
        rocker_max = coupler + inner_distance - 2
        if rocker_min >= rocker_max:
            continue
        rocker = rng.uniform(rocker_min, rocker_max)
        heading, phase = rng.uniform(-math.pi, math.pi), rng.uniform(-2, 2)
        origin = (rng.uniform(-200, 200), rng.uniform(-200, 200))
        fixed = (origin[0] + ground * math.cos(heading),
                 origin[1] + ground * math.sin(heading))
        mechanism = PlanarFourBar(origin, fixed, crank, coupler, rocker, phase,
                                  rng.choice((-1, 1)))
        accepted += 1
        start = rng.uniform(-3, 1)
        end = start + rng.uniform(.1, math.tau)
        bound = mechanism.angular_rate_bounds(start, end)
        for _ in range(12):
            theta = rng.uniform(start, end - 1e-6)
            first, second = mechanism.solve(theta), mechanism.solve(theta + 1e-7)
            for angle, limit in (
                ('rocker_angle_rad', 'maximum_rocker_angular_rate_rad_per_input_rad'),
                ('coupler_angle_rad', 'maximum_coupler_angular_rate_rad_per_input_rad'),
            ):
                rate = abs(_wrapped_difference(second[angle], first[angle])) / 1e-7
                assert rate <= bound[limit] * (1 + 1e-5)
    assert accepted >= 50


def test_source_bound_point_travel_for_each_rigid_body():
    mechanism = PlanarFourBar((13, -7), (113, -7), 25, 90, 60, .21, -1)
    angle_interval = (-.7, 3.4)
    radius = 125.
    for body in ('crank', 'coupler', 'rocker'):
        bound = mechanism.point_motion_bound(body, radius, *angle_interval)
        assert bound['source_sha256'] == mechanism.source_sha256
        assert bound['input_interval_rad'] == list(angle_interval)
        assert bound['maximum_point_radius_mm'] == radius
        for sample in range(80):
            theta = angle_interval[0] + (angle_interval[1] - angle_interval[0]) * sample / 80
            poses = (mechanism.solve(theta), mechanism.solve(theta + 1e-6))
            points = []
            for pose in poses:
                if body == 'crank':
                    pivot, angle = mechanism.input_pivot_mm, pose['crank_angle_rad']
                elif body == 'coupler':
                    pivot, angle = pose['crank_pin_mm'], pose['coupler_angle_rad']
                else:
                    pivot, angle = mechanism.output_pivot_mm, pose['rocker_angle_rad']
                points.append((pivot[0] + radius * math.cos(angle + .38),
                               pivot[1] + radius * math.sin(angle + .38)))
            assert math.dist(*points) / 1e-6 <= (
                bound['maximum_point_speed_mm_per_input_rad'] * (1 + 1e-5))
    assert mechanism.point_motion_bound('coupler', 0, *angle_interval)[
        'maximum_point_speed_mm_per_input_rad'] == pytest.approx(25.)
    with pytest.raises(ValueError, match='Body'):
        mechanism.point_motion_bound('frame', 1, *angle_interval)
    with pytest.raises(ValueError, match='nonnegative'):
        mechanism.point_motion_bound('rocker', -1, *angle_interval)


def test_interval_rejects_hidden_unreachable_and_change_point_configurations():
    # Endpoints close but the intervening crank angle is too far from O2.
    mechanism = PlanarFourBar((0, 0), (100, 0), 25, 60, 50)
    mechanism.solve(.01)
    mechanism.solve(math.tau - .01)
    with pytest.raises(ValueError, match='Full input interval'):
        mechanism.angular_rate_bounds(.01, math.tau - .01)
    with pytest.raises(ValueError, match='change-point'):
        PlanarFourBar((0, 0), (100, 0), 25, 75, 50).angular_rate_bounds(math.pi, math.pi)


@pytest.mark.parametrize('kwargs', [
    {'input_pivot_mm': (100, 0)},
    {'crank_length_mm': 0},
    {'coupler_length_mm': float('nan')},
    {'rocker_length_mm': -1},
    {'input_phase_rad': float('inf')},
    {'branch': 0},
    {'branch': True},
])
def test_invalid_source_is_rejected(kwargs):
    fields = dict(input_pivot_mm=(0, 0), output_pivot_mm=(100, 0),
                  crank_length_mm=25, coupler_length_mm=90,
                  rocker_length_mm=60)
    fields.update(kwargs)
    with pytest.raises(ValueError):
        PlanarFourBar(**fields)


def test_bounds_are_tied_to_geometry_and_ordered_finite_interval():
    first = PlanarFourBar((0, 0), (100, 0), 25, 90, 60)
    revised = PlanarFourBar((0, 0), (100, 0), 25, 91, 60)
    assert first.source_sha256 != revised.source_sha256
    assert first.angular_rate_bounds(0, math.tau)['source_sha256'] != revised.source_sha256
    with pytest.raises(ValueError, match='must not precede'):
        first.angular_rate_bounds(1, 0)
    with pytest.raises(ValueError, match='finite'):
        first.angular_rate_bounds(0, math.inf)
    with pytest.raises(ValueError, match='finite'):
        first.solve(float('nan'))
