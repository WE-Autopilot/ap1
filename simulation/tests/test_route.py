"""Route planner and follower tests. Run: python3 -m pytest simulation/tests"""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from route import Edge, RouteFollower, length, map_edges, nearest_edge, plan_route, vehicle_geometry


def line(a, b, step=1.0):
    a, b = np.array(a, float), np.array(b, float)
    n = max(int(round(np.linalg.norm(b - a) / step)), 1)
    return a + (b - a) * np.linspace(0.0, 1.0, n + 1)[:, None]


def road(u, v, a, b, road_id, junction=False):
    return Edge(u, v, line(a, b), (road_id, 0), -1, junction)


CORNERS = [(0, 0), (100, 0), (100, 100), (0, 100)]


def square():
    """One-way ring of four 100 m roads through nodes 0-1-2-3."""
    return [road(i, (i + 1) % 4, CORNERS[i], CORNERS[(i + 1) % 4], i + 1) for i in range(4)]


def square_with_spur():
    """The ring, plus a dead-end road 5 out from node 1 and back through a junction."""
    return square() + [road(1, 4, (100, 0), (150, 0), 5),
                       road(4, 1, (150, 0), (100, 0), 6, junction=True)]


def test_short_route_is_a_closed_loop_from_the_start_point():
    pts, covered, total = plan_route(square(), 0, 10, short=True)
    assert pts[0] == pytest.approx(pts[-1])
    assert pts[0] == pytest.approx([10, 0])
    assert length(pts) == pytest.approx(400.0)
    assert (covered, total) == (4, 4)


def test_short_route_skips_roads_it_does_not_need():
    pts, covered, total = plan_route(square_with_spur(), 0, 0, short=True)
    assert (covered, total) == (4, 5)
    assert pts[:, 0].max() == pytest.approx(100.0)


def test_full_route_covers_every_road_and_still_closes():
    pts, covered, total = plan_route(square_with_spur(), 0, 0)
    assert (covered, total) == (5, 5)
    assert pts[:, 0].max() == pytest.approx(150.0)
    assert pts[0] == pytest.approx(pts[-1])


def test_route_has_no_jumps_or_repeated_points():
    pts, _, _ = plan_route(square_with_spur(), 0, 10)
    steps = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    assert steps.max() < 1.5
    assert steps.min() > 0.0


def test_route_that_cannot_get_home_is_an_error():
    edges = [road(0, 1, (0, 0), (100, 0), 1)]
    with pytest.raises(ValueError, match='back to the start'):
        plan_route(edges, 0, 0, short=True)


def test_nearest_edge_stays_on_the_requested_lane_and_off_junctions():
    edges = [road(0, 1, (0, 0), (100, 0), 1),
             road(0, 1, (0, 0), (100, 0), 2),
             road(0, 1, (0, 0), (100, 0), 1, junction=True)]
    assert nearest_edge(edges, 42.0, 0.3, 1, -1) == (0, 42)
    with pytest.raises(ValueError, match='try another spawn point'):
        nearest_edge(edges, 42.0, 0.3, 1, -2)      # no such lane
    with pytest.raises(ValueError, match='try another spawn point'):
        nearest_edge(edges[2:], 42.0, 0.3, 1, -1)  # only a junction


def fake_waypoint(x, y, road_id, lane_id=-1, junction=False, following=()):
    location = SimpleNamespace(x=x, y=y)
    return SimpleNamespace(transform=SimpleNamespace(location=location), road_id=road_id,
                           section_id=0, lane_id=lane_id, is_junction=junction,
                           next_until_lane_end=lambda _step: list(following))


def test_map_edges_merges_nodes_by_position_and_keeps_the_end_point():
    a = fake_waypoint(0, 0, 1, following=[fake_waypoint(1, 0, 1)])
    b = fake_waypoint(2, 0, 1)
    b_nearby = fake_waypoint(2.2, 0, 2)
    c = fake_waypoint(5, 0, 2)
    carla_map = SimpleNamespace(get_topology=lambda: [(a, b), (b_nearby, c)])
    first, second = map_edges(carla_map)
    assert first.v == second.u                      # 0.2 m apart: the same node
    assert first.pts[-1] == pytest.approx([2, 0])   # next_until_lane_end stopped short
    assert first.road == (1, 0)


def fake_vehicle(wheelbase):
    class Position:
        def distance(self, _other):
            return wheelbase
    wheel = SimpleNamespace(position=Position(), max_steer_angle=70.0)
    physics = SimpleNamespace(wheels=[wheel] * 4)
    return SimpleNamespace(get_physics_control=lambda: physics)


@pytest.mark.parametrize('reported', [2.9, 290.0])   # older CARLA reports cm
def test_vehicle_geometry_gives_metres(reported):
    assert vehicle_geometry(fake_vehicle(reported)) == (pytest.approx(2.9), 70.0)


def drive(follower, x, y, yaw_deg, dt=0.01, max_ticks=100_000, wheelbase=2.9, max_steer_deg=70.0):
    """A kinematic bicycle in CARLA axes (+yaw and +steer turn right). Returns top speed."""
    v, top = 0.0, 0.0
    for _ in range(max_ticks):
        throttle, steer, brake = follower.step(x, y, yaw_deg, v, dt)
        v = max(v + (3.0 * throttle - 6.0 * brake) * dt, 0.0)
        top = max(top, v)
        yaw = math.radians(yaw_deg)
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw_deg += math.degrees(v / wheelbase * math.tan(math.radians(steer * max_steer_deg))) * dt
        if follower.remaining < 1.0 and v < 0.05:
            return top, x, y
    pytest.fail(f'never finished; {follower.remaining:.0f} m left')


@pytest.mark.parametrize('dt', [0.01, 0.02])
def test_follower_finishes_the_loop_on_route_and_under_the_cap(dt):
    pts, _, _ = plan_route(square(), 0, 0, short=True)
    speed = 10.0 / 3.6
    follower = RouteFollower(pts, speed, 2.9, 70.0, 4.5, 5.0)
    top, x, y = drive(follower, *pts[0], 0.0, dt=dt)  # raises if it ever leaves the route
    assert top <= speed * 1.02
    assert math.hypot(x - pts[-1][0], y - pts[-1][1]) < 2.0


def test_follower_steers_right_toward_a_target_on_the_right():
    follower = RouteFollower(line((0, 0), (0, 50)), 2.7, 2.9, 70.0, 4.5, 5.0)     # +y is right in CARLA
    assert follower.step(0.0, 0.0, 0.0, 1.0, 0.01)[1] > 0.0


def test_follower_steering_rate_does_not_depend_on_the_tick():
    for dt in (0.01, 0.02):
        follower = RouteFollower(line((0, 0), (0, 50)), 2.7, 2.9, 70.0, 4.5, 5.0)
        steer = follower.step(0.0, 0.0, 0.0, 1.0, dt)[1]
        assert steer == pytest.approx(RouteFollower.MAX_STEER_RATE * dt)


def test_follower_off_route_limit_is_the_given_one():
    follower = RouteFollower(line((0, 0), (100, 0)), 2.7, 2.9, 70.0, 4.5, 20.0)
    follower.step(0.0, 10.0, 0.0, 1.0, 0.01)   # 10 m off is fine with a 20 m limit


def test_follower_reports_leaving_the_route():
    follower = RouteFollower(line((0, 0), (100, 0)), 2.7, 2.9, 70.0, 4.5, 5.0)
    with pytest.raises(RuntimeError, match='off the route'):
        follower.step(0.0, 10.0, 0.0, 1.0, 0.01)
