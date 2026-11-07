"""
The route planner and follower. Needs only numpy: map_edges() and
vehicle_geometry() read the CARLA objects they are handed, and everything else
works on plain arrays of (x, y) points in CARLA coordinates.
"""
import heapq
import math
from dataclasses import dataclass

import numpy as np


@dataclass
class Edge:
    """One lane from a junction to the next, as dense (x, y) points in CARLA coordinates."""
    u: int
    v: int
    pts: np.ndarray
    road: tuple          # (road_id, section_id): covering one lane covers the road
    lane: int
    junction: bool


def length(pts):
    return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())


def map_edges(carla_map):
    """The map's lane graph from CARLA's topology; nodes are merged by position."""
    nodes = []

    def node(loc):
        for i, (x, y) in enumerate(nodes):
            if math.hypot(x - loc.x, y - loc.y) < 0.5:
                return i
        nodes.append((loc.x, loc.y))
        return len(nodes) - 1

    edges = []
    for a, b in carla_map.get_topology():
        pts = [(w.transform.location.x, w.transform.location.y)
               for w in [a] + a.next_until_lane_end(1.0)]
        end = b.transform.location
        if math.hypot(pts[-1][0] - end.x, pts[-1][1] - end.y) > 0.05:
            pts.append((end.x, end.y))
        edges.append(Edge(node(a.transform.location), node(end), np.array(pts),
                          (a.road_id, a.section_id), a.lane_id, a.is_junction))
    return edges


def dijkstra(edges, src):
    """Shortest distances from src and the edge used to reach each node."""
    adj = {}
    for i, e in enumerate(edges):
        adj.setdefault(e.u, []).append(i)
    dist, via, heap = {src: 0.0}, {}, [(0.0, src)]
    while heap:
        d, n = heapq.heappop(heap)
        if d > dist[n]:
            continue
        for i in adj.get(n, []):
            m = edges[i].v
            nd = d + length(edges[i].pts)
            if nd < dist.get(m, math.inf):
                dist[m], via[m] = nd, i
                heapq.heappush(heap, (nd, m))
    return dist, via


def path_to(edges, via, src, dst):
    path = []
    while dst != src:
        path.append(via[dst])
        dst = edges[via[dst]].u
    return path[::-1]


def plan_route(edges, start_edge, start_idx, short=False):
    """
    A loop from point start_idx of start_edge back to the same point. Greedily
    drives to the nearest road not yet driven until all are covered, then heads
    home. With short, it adds no roads and just heads home.
    Returns (points Nx2, roads covered, roads total).
    """
    e0 = edges[start_edge]
    home = e0.u
    roads = {e.road for e in edges if not e.junction}

    pieces = [e0.pts[start_idx:]]
    covered = {e0.road}
    cur = e0.v

    def drive(path):
        for i in path:
            pieces.append(edges[i].pts)
            if not edges[i].junction:
                covered.add(edges[i].road)

    while covered != roads and not short:
        dist, via = dijkstra(edges, cur)
        pick = min(((dist[e.u] + length(e.pts), i) for i, e in enumerate(edges)
                    if not e.junction and e.road not in covered and e.u in dist),
                   default=(0, None))[1]
        if pick is None:
            break
        drive(path_to(edges, via, cur, edges[pick].u) + [pick])
        cur = edges[pick].v

    dist, via = dijkstra(edges, cur)
    if home not in dist:
        raise SystemExit('the route can\'t get back to the start; try another --spawn-point')
    drive(path_to(edges, via, cur, home))
    pieces.append(e0.pts[:start_idx + 1])

    pts = [pieces[0]]
    for p in pieces[1:]:
        pts.append(p[1:] if len(p) and np.hypot(*(p[0] - pts[-1][-1])) < 0.1 else p)
    return np.vstack(pts), len(covered), len(roads)


def nearest_edge(edges, x, y, road_id, lane_id):
    """(edge index, point index) on the lane closest to (x, y)."""
    best = (math.inf, None, None)
    for i, e in enumerate(edges):
        if e.junction or (e.road[0], e.lane) != (road_id, lane_id):
            continue
        d = np.hypot(e.pts[:, 0] - x, e.pts[:, 1] - y)
        if d.min() < best[0]:
            best = (d.min(), i, int(d.argmin()))
    return best[1], best[2]


def vehicle_geometry(vehicle):
    """(wheelbase m, max steer deg) from the vehicle's physics."""
    wheels = vehicle.get_physics_control().wheels
    wb = wheels[0].position.distance(wheels[2].position)
    return (wb / 100.0 if wb > 20.0 else wb), wheels[0].max_steer_angle   # older CARLA: cm


class RouteFollower:
    """Pure pursuit for steering, PI for speed. Works in CARLA coordinates."""

    KP, KI = 0.6, 0.15
    MAX_THROTTLE = 0.75
    MAX_STEER_STEP = 0.03     # per tick
    ACCEL = 0.8               # m/s^2, how fast the speed target ramps up (~3.5 s to 10 km/h)
    STOP_DECEL = 1.0          # m/s^2, slowing for the end of the route

    def __init__(self, pts, speed, wheelbase, max_steer_deg, lookahead_m, max_off_route_m):
        self.pts = pts
        self.s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
        self.speed, self.wheelbase, self.max_steer = speed, wheelbase, max_steer_deg
        self.lookahead_m, self.max_off_route_m = lookahead_m, max_off_route_m
        self.i = 0
        self.integral = 0.0
        self.steer = 0.0
        self.v_ref = 0.0

    @property
    def remaining(self):
        return self.s[-1] - self.s[self.i]

    def step(self, x, y, yaw_deg, v, dt):
        """(throttle, steer, brake) for this tick; raises RuntimeError if off the route."""
        # Search forward only, so overlapping parts of the route can't be confused.
        ahead = slice(self.i, min(self.i + 40, len(self.pts)))
        d = np.hypot(self.pts[ahead, 0] - x, self.pts[ahead, 1] - y)
        self.i += int(d.argmin())
        if d.min() > self.max_off_route_m:
            raise RuntimeError(f'{d.min():.1f} m off the route at ({x:.1f}, {y:.1f})')

        k = min(int(np.searchsorted(self.s, self.s[self.i] + self.lookahead_m)), len(self.pts) - 1)
        dx, dy = self.pts[k, 0] - x, self.pts[k, 1] - y
        yaw = math.radians(yaw_deg)
        lx = math.cos(yaw) * dx + math.sin(yaw) * dy
        ly = -math.sin(yaw) * dx + math.cos(yaw) * dy      # + = right (CARLA)
        alpha = math.atan2(ly, lx)
        delta = math.atan2(2.0 * self.wheelbase * math.sin(alpha), max(math.hypot(lx, ly), 1.0))
        target = float(np.clip(math.degrees(delta) / self.max_steer, -1.0, 1.0))
        self.steer += float(np.clip(target - self.steer, -self.MAX_STEER_STEP, self.MAX_STEER_STEP))

        if self.remaining < 1.0:
            return 0.0, self.steer, 1.0
        # The speed target ramps up at ACCEL rather than jumping to the cruise speed.
        target = min(self.speed, math.sqrt(2.0 * self.STOP_DECEL * self.remaining))
        self.v_ref = min(self.v_ref + self.ACCEL * dt, target)
        err = self.v_ref - v
        if abs(err) < 0.5:  # integrate only near the target, so the run-up can't wind it up
            self.integral = float(np.clip(self.integral + err * dt, -1.0, 1.0))
        u = self.KP * err + self.KI * self.integral
        if v >= self.speed:  # the cap is a cap: never push past it
            u = min(u, 0.0)
        return (min(u, self.MAX_THROTTLE), self.steer, 0.0) if u >= 0 else (0.0, self.steer, min(-u, 1.0))
