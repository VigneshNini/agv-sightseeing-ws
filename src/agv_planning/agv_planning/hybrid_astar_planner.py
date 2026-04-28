#!/usr/bin/env python3
"""Hybrid A* Path Planner with Reeds-Shepp curve primitives.

The planner uses the standard Hybrid A* approach:
 - Discrete heading states (yaw_res steps)
 - Ackermann bicycle model kinematics for neighbor expansion
 - Euclidean distance heuristic
 - Reeds-Shepp analytic expansion when close to goal

Reeds-Shepp curves are categorised by the 5 word types:
    CSC: LfSfRf, RfSfLf, LbSbRb, RbSbLb, LfSfLf, RfSfRf, LbSbLb, RbSbRb
    CCC: LfRfLf, RfLfRf, LbRbLb, RbLbRb, ...

Only the 12 Reeds-Shepp words from Reeds & Shepp (1990) are implemented.
"""
import math
import heapq
from typing import List, Optional, Tuple
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from agv_msgs.msg import TourStop

# ---------------------------------------------------------------------------
# Reeds-Shepp curve primitives
# ---------------------------------------------------------------------------
INF = float('inf')

def _mod2pi(a: float) -> float:
    return a - 2 * math.pi * math.floor(a / (2 * math.pi))

def _polar(x: float, y: float) -> Tuple[float, float]:
    """Return (r, theta) for Cartesian (x, y)."""
    return math.hypot(x, y), math.atan2(y, x)

def _R(x: float, y: float, phi: float):
    """Transform goal (x, y, phi) to t, u, v for CSC paths."""
    return _polar(x + math.sin(phi), y - 1 - math.cos(phi))

def _LSL(x: float, y: float, phi: float):
    u, t = _polar(x - math.sin(phi), y - 1 + math.cos(phi))
    if t < 0:
        return None
    v = _mod2pi(phi - t)
    return t, u, v, 'LSL'

def _RSR(x: float, y: float, phi: float):
    u, t = _polar(x + math.sin(phi), y + 1 - math.cos(phi))
    if t < 0:
        return None
    v = _mod2pi(t - phi)
    return t, u, v, 'RSR'

def _LSR(x: float, y: float, phi: float):
    u1, t1 = _polar(x + math.sin(phi), y - 1 - math.cos(phi))
    u = math.sqrt(u1 ** 2 - 4)
    if u1 < 2:
        return None
    t = _mod2pi(t1 - math.atan2(2, u))
    v = _mod2pi(t - phi)
    return t, u, v, 'LSR'

def _RSL(x: float, y: float, phi: float):
    u1, t1 = _polar(x - math.sin(phi), y + 1 + math.cos(phi))
    u = math.sqrt(u1 ** 2 - 4)
    if u1 < 2:
        return None
    t = _mod2pi(t1 + math.atan2(2, u))
    v = _mod2pi(phi - t)
    return t, u, v, 'RSL'

def _LRL(x: float, y: float, phi: float):
    u1, t1 = _polar(x - math.sin(phi), y - 1 + math.cos(phi))
    if u1 > 4:
        return None
    u = -2 * math.asin(u1 / 4)
    t = _mod2pi(t1 + math.pi / 2 + u / 2)
    v = _mod2pi(phi - t + u)
    return t, u, v, 'LRL'

def _RLR(x: float, y: float, phi: float):
    u1, t1 = _polar(x + math.sin(phi), y + 1 - math.cos(phi))
    if u1 > 4:
        return None
    u = _mod2pi(2 * math.pi - math.acos((u1 ** 2 - 6) / (2 * u1) + 1 / 2) * 2)
    if math.isnan(u):
        return None
    # simplified via _LRL reflection
    u = -2 * math.asin(u1 / 4)
    t = _mod2pi(t1 - math.pi / 2 - u / 2)
    v = _mod2pi(t - phi + u)
    return t, u, v, 'RLR'


def _rs_words(x: float, y: float, phi: float) -> List:
    """Try all 6 symmetric word generators and collect valid paths."""
    results = []
    for fn in (_LSL, _RSR, _LSR, _RSL, _LRL, _RLR):
        r = fn(x, y, phi)
        if r is not None:
            t, u, v, word = r
            length = abs(t) + abs(u) + abs(v)
            results.append((length, t, u, v, word))
    # Include time-flipped (backward-start) versions
    for fn in (_LSL, _RSR, _LSR, _RSL, _LRL, _RLR):
        r = fn(-x, y, -phi)
        if r is not None:
            t, u, v, word = r
            length = abs(t) + abs(u) + abs(v)
            results.append((length, -t, -u, -v, word + 'b'))
    # Include reflected (right-left swap) versions
    for fn in (_LSL, _RSR, _LSR, _RSL, _LRL, _RLR):
        r = fn(x, -y, -phi)
        if r is not None:
            t, u, v, word = r
            length = abs(t) + abs(u) + abs(v)
            results.append((length, t, u, v, 'R' + word[1:] if word[0] == 'L' else 'L' + word[1:]))
    return results


def _rs_shortest(x: float, y: float, phi: float,
                 radius: float = 1.0) -> Tuple[float, List]:
    """Return (path_length, waypoints) for the shortest Reeds-Shepp path.

    The goal (x, y, phi) is given in the local frame of the start pose,
    normalised so the minimum turning radius is 1.  Output waypoints are
    in the same frame and must be re-scaled by `radius`.
    """
    xn, yn = x / radius, y / radius
    words = _rs_words(xn, yn, phi)
    if not words:
        return INF, []
    words.sort(key=lambda w: w[0])
    length, t, u, v, word = words[0]
    # Reconstruct waypoints by arc integration
    pts = _rs_interpolate(0.0, 0.0, 0.0, t, u, v, word, radius)
    return length * radius, pts


def _arc_pts(x0, y0, yaw0, arc, turn_dir, step=0.1):
    """Integrate a circular arc and return list of (x, y, yaw)."""
    pts = []
    n = max(1, int(abs(arc) / step))
    delta_yaw = arc / n
    x, y, yaw = x0, y0, yaw0
    for _ in range(n):
        # move one step
        dx = math.cos(yaw) * step * math.copysign(1, arc)
        dy = math.sin(yaw) * step * math.copysign(1, arc)
        x += dx
        y += dy
        yaw += delta_yaw * turn_dir
        pts.append((x, y, yaw))
    return pts


def _rs_interpolate(x0, y0, yaw0, t, u, v, word, radius, step=0.1):
    """Produce a list of (x, y, yaw) along the RS curve."""
    pts = [(x0, y0, yaw0)]

    def segment(xs, ys, yaws, arc, mode):
        """mode: 'L'=left turn +1, 'R'=right turn -1, 'S'=straight."""
        out = []
        if mode in ('L', 'R'):
            turn = 1 if mode == 'L' else -1
            # arc is in radians; arc length = |arc| * radius
            n = max(1, int(abs(arc) * abs(radius) / step))
            for i in range(1, n + 1):
                frac = i / n
                dyaw = arc * frac
                cx = xs - radius * math.sin(yaws)
                cy = ys + radius * math.cos(yaws)
                ang = yaws + dyaw * turn
                nx = cx + radius * math.sin(ang)
                ny = cy - radius * math.cos(ang)
                out.append((nx, ny, ang))
        else:
            # straight
            length = abs(u) * radius
            n = max(1, int(length / step))
            fwd = math.copysign(1, arc)
            for i in range(1, n + 1):
                frac = i / n
                nx = xs + frac * length * fwd * math.cos(yaws)
                ny = ys + frac * length * fwd * math.sin(yaws)
                out.append((nx, ny, yaws))
        return out

    # Parse word letters
    letters = [c for c in word if c in 'LRS']
    arcs = [t, u, v]
    x, y, yaw = x0, y0, yaw0
    for letter, arc in zip(letters, arcs):
        seg = segment(x, y, yaw, arc, letter)
        if seg:
            pts.extend(seg)
            x, y, yaw = seg[-1]
    return pts


# ---------------------------------------------------------------------------
# Hybrid A* Node
# ---------------------------------------------------------------------------

class HybridAStarNode:
    __slots__ = ('x', 'y', 'yaw', 'g', 'h', 'f', 'parent', 'steer', 'reverse')

    def __init__(self, x, y, yaw, g=0.0, h=0.0, parent=None, steer=0.0, reverse=False):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent
        self.steer = steer
        self.reverse = reverse

    def __lt__(self, other):
        return self.f < other.f

    def key(self, res=0.5, yaw_res=math.radians(15)):
        return (int(self.x / res), int(self.y / res),
                int(self.yaw / yaw_res))


class HybridAStarPlanner(Node):
    def __init__(self):
        super().__init__('hybrid_astar_planner')
        self.declare_parameter('vehicle_length', 2.0)
        self.declare_parameter('vehicle_width', 1.0)
        self.declare_parameter('wheelbase', 1.2)
        self.declare_parameter('min_turning_radius', 1.5)
        self.declare_parameter('max_steering_angle', 0.6)
        self.declare_parameter('step_size', 0.5)
        self.declare_parameter('turn_penalty', 1.5)
        self.declare_parameter('reverse_penalty', 5.0)
        self.declare_parameter('rs_goal_dist', 8.0)
        self.declare_parameter('max_iterations', 5000)

        self.L = self.get_parameter('wheelbase').value
        self.R_min = self.get_parameter('min_turning_radius').value
        self.max_steer = self.get_parameter('max_steering_angle').value
        self.step = self.get_parameter('step_size').value
        self.turn_pen = self.get_parameter('turn_penalty').value
        self.rev_pen = self.get_parameter('reverse_penalty').value
        self.rs_dist = self.get_parameter('rs_goal_dist').value
        self.max_iter = self.get_parameter('max_iterations').value

        self.occ_map: Optional[np.ndarray] = None
        self.map_info = None
        self.start: Optional[Tuple] = None
        self.goal: Optional[Tuple] = None

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, 1)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/agv/pose', self._pose_cb, 10)
        self.goal_sub = self.create_subscription(
            TourStop, '/agv/current_tour_stop', self._goal_cb, 10)
        self.path_pub = self.create_publisher(Path, '/agv/planned_path', 10)
        self.get_logger().info('Hybrid A* Planner with Reeds-Shepp curves initialized')

    # ---- Callbacks ----
    def _map_cb(self, msg: OccupancyGrid):
        self.map_info = msg.info
        h, w = msg.info.height, msg.info.width
        self.occ_map = np.array(msg.data, dtype=np.int8).reshape(h, w)

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y ** 2 + q.z ** 2))
        self.start = (p.position.x, p.position.y, yaw)

    def _goal_cb(self, msg: TourStop):
        # TourStop carries ENU coords in longitude/latitude fields (reused as x/y)
        self.goal = (float(msg.longitude), float(msg.latitude), 0.0)
        if self.start is not None:
            self._plan()

    # ---- Collision checking ----
    def _world_to_grid(self, x, y):
        if self.map_info is None:
            return None, None
        gx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        gy = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return gx, gy

    def _is_free(self, x, y):
        if self.occ_map is None:
            return True
        gx, gy = self._world_to_grid(x, y)
        if gx is None:
            return False
        h, w = self.occ_map.shape
        if not (0 <= gx < w and 0 <= gy < h):
            return False
        return int(self.occ_map[gy, gx]) < 50

    def _path_free(self, pts):
        return all(self._is_free(px, py) for px, py, _ in pts)

    # ---- Heuristic ----
    def _heuristic(self, x, y, gx, gy):
        return math.hypot(x - gx, y - gy)

    # ---- Steering angles ----
    def _steer_set(self):
        n = 5
        return [self.max_steer * (2 * i / (n - 1) - 1) for i in range(n)]

    # ---- Reeds-Shepp analytic expansion ----
    def _try_rs(self, cur: HybridAStarNode, gx, gy, gyaw):
        """Attempt direct RS connection to goal; return waypoints or None."""
        dist = math.hypot(cur.x - gx, cur.y - gy)
        if dist > self.rs_dist:
            return None
        # Transform goal into current node frame
        dx = gx - cur.x
        dy = gy - cur.y
        dphi = gyaw - cur.yaw
        # Rotate to local frame
        c, s = math.cos(cur.yaw), math.sin(cur.yaw)
        lx = c * dx + s * dy
        ly = -s * dx + c * dy
        length, pts = _rs_shortest(lx, ly, dphi, self.R_min)
        if length == INF or length > dist * 3:
            return None
        # Transform back to global frame
        global_pts = []
        for (px, py, pyaw) in pts:
            gp_x = cur.x + c * px - s * py
            gp_y = cur.y + s * px + c * py
            global_pts.append((gp_x, gp_y, pyaw + cur.yaw))
        if self._path_free(global_pts):
            return global_pts
        return None

    # ---- Main search ----
    def _plan(self):
        if self.start is None or self.goal is None:
            return
        sx, sy, syaw = self.start
        gx, gy, gyaw = self.goal
        if math.hypot(sx - gx, sy - gy) < 0.5:
            return

        start_node = HybridAStarNode(
            sx, sy, syaw, 0.0, self._heuristic(sx, sy, gx, gy))
        open_heap = [start_node]
        visited = {}
        rs_path = None
        iterations = 0

        while open_heap and iterations < self.max_iter:
            iterations += 1
            cur = heapq.heappop(open_heap)
            key = cur.key()
            if key in visited:
                continue
            visited[key] = cur

            # Try Reeds-Shepp analytic expansion to goal
            rs_path = self._try_rs(cur, gx, gy, gyaw)
            if rs_path is not None:
                # Reconstruct path to cur, then append RS waypoints
                back = []
                node = cur
                while node is not None:
                    back.append((node.x, node.y, node.yaw))
                    node = node.parent
                back.reverse()
                all_pts = back + rs_path
                self._publish_path(all_pts)
                self.get_logger().info(
                    f'RS expansion succeeded after {iterations} iterations, '
                    f'path length {len(all_pts)}')
                return

            # Expand neighbours
            for steer in self._steer_set():
                for reverse in [False, True]:
                    sign = -1 if reverse else 1
                    if abs(steer) > 1e-5:
                        R = self.L / math.tan(abs(steer))
                        dyaw = sign * self.step / R * math.copysign(1, steer)
                    else:
                        dyaw = 0.0
                    nx = cur.x + sign * self.step * math.cos(cur.yaw)
                    ny = cur.y + sign * self.step * math.sin(cur.yaw)
                    nyaw = math.atan2(math.sin(cur.yaw + dyaw),
                                      math.cos(cur.yaw + dyaw))
                    if not self._is_free(nx, ny):
                        continue
                    cost = self.step
                    if reverse:
                        cost *= self.rev_pen
                    if abs(steer) > 0.1:
                        cost *= self.turn_pen
                    child = HybridAStarNode(
                        nx, ny, nyaw,
                        cur.g + cost,
                        self._heuristic(nx, ny, gx, gy),
                        cur, steer, reverse)
                    if child.key() not in visited:
                        heapq.heappush(open_heap, child)

        # No RS expansion found — publish best-effort path
        if visited:
            best = min(visited.values(),
                       key=lambda n: self._heuristic(n.x, n.y, gx, gy))
            back = []
            node = best
            while node is not None:
                back.append((node.x, node.y, node.yaw))
                node = node.parent
            back.reverse()
            self._publish_path(back)
            self.get_logger().warn(
                f'RS expansion not reached; publishing partial path ({len(back)} pts)')

    def _publish_path(self, pts: List[Tuple]):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for px, py, pyaw in pts:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(px)
            ps.pose.position.y = float(py)
            half = pyaw / 2.0
            ps.pose.orientation.w = math.cos(half)
            ps.pose.orientation.z = math.sin(half)
            path.poses.append(ps)
        self.path_pub.publish(path)
        self.get_logger().info(f'Path published: {len(path.poses)} poses')


def main(args=None):
    rclpy.init(args=args)
    node = HybridAStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
