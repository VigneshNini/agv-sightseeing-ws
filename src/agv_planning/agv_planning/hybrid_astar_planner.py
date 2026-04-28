#!/usr/bin/env python3
"""Hybrid A* Path Planner for Ackermann vehicles."""
import math
import heapq
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from agv_msgs.msg import TourStop


class HybridAStarNode:
    def __init__(self, x, y, yaw, g=0, h=0, parent=None, steer=0.0, rev=False):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent
        self.steer = steer
        self.reverse = rev

    def __lt__(self, other):
        return self.f < other.f

    def key(self, res=0.5, yaw_res=15):
        return (int(self.x/res), int(self.y/res), int(math.degrees(self.yaw)/yaw_res))


class HybridAStarPlanner(Node):
    def __init__(self):
        super().__init__('hybrid_astar_planner')
        self.declare_parameter('vehicle_length', 2.0)
        self.declare_parameter('vehicle_width', 1.0)
        self.declare_parameter('wheelbase', 1.2)
        self.declare_parameter('max_steering_angle', 0.6)
        self.declare_parameter('step_size', 0.5)
        self.declare_parameter('turn_penalty', 1.5)
        self.declare_parameter('reverse_penalty', 5.0)
        self.declare_parameter('obstacle_clearance', 0.5)

        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steer = self.get_parameter('max_steering_angle').value
        self.step = self.get_parameter('step_size').value
        self.turn_penalty = self.get_parameter('turn_penalty').value
        self.rev_penalty = self.get_parameter('reverse_penalty').value

        self.map = None
        self.map_info = None
        self.start = None
        self.goal = None

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/agv/pose', self.pose_callback, 10)
        self.goal_sub = self.create_subscription(TourStop, '/agv/current_tour_stop', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, '/agv/planned_path', 10)
        self.get_logger().info('Hybrid A* Planner initialized')

    def map_callback(self, msg: OccupancyGrid):
        self.map_info = msg.info
        w, h = msg.info.width, msg.info.height
        self.map = np.array(msg.data, dtype=np.int8).reshape(h, w)

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2 + q.z**2))
        self.start = (p.position.x, p.position.y, yaw)

    def goal_callback(self, msg: TourStop):
        self.goal = (float(msg.longitude), float(msg.latitude), 0.0)
        if self.start is not None:
            self._plan()

    def _world_to_grid(self, x, y):
        if self.map_info is None:
            return None, None
        gx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        gy = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return gx, gy

    def _is_valid(self, x, y):
        if self.map is None:
            return True
        gx, gy = self._world_to_grid(x, y)
        if gx is None:
            return False
        h, w = self.map.shape
        if not (0 <= gx < w and 0 <= gy < h):
            return False
        return self.map[gy, gx] < 50

    def _heuristic(self, x, y, gx, gy):
        return math.sqrt((x-gx)**2 + (y-gy)**2)

    def _steer_angles(self):
        n = 5
        return [self.max_steer * (i - n//2) / (n//2) for i in range(n)]

    def _plan(self):
        if self.start is None or self.goal is None:
            return
        sx, sy, syaw = self.start
        gx, gy, gyaw = self.goal
        dist = math.sqrt((sx-gx)**2 + (sy-gy)**2)
        if dist < 0.5:
            return
        start_node = HybridAStarNode(sx, sy, syaw, 0, self._heuristic(sx, sy, gx, gy))
        open_heap = [start_node]
        visited = {}
        best = None
        max_iter = 2000
        iterations = 0

        while open_heap and iterations < max_iter:
            iterations += 1
            cur = heapq.heappop(open_heap)
            key = cur.key()
            if key in visited:
                continue
            visited[key] = cur
            dist_to_goal = math.sqrt((cur.x-gx)**2 + (cur.y-gy)**2)
            if dist_to_goal < self.step * 2:
                best = cur
                break
            if best is None or dist_to_goal < math.sqrt((best.x-gx)**2 + (best.y-gy)**2):
                best = cur
            for steer in self._steer_angles():
                for reverse in [False]:
                    dx = self.step * math.cos(cur.yaw) * (-1 if reverse else 1)
                    dy = self.step * math.sin(cur.yaw) * (-1 if reverse else 1)
                    if abs(steer) > 1e-4:
                        R = self.wheelbase / math.tan(steer)
                        dyaw = self.step / R * (-1 if reverse else 1)
                    else:
                        dyaw = 0.0
                    nx = cur.x + dx
                    ny = cur.y + dy
                    nyaw = math.atan2(math.sin(cur.yaw + dyaw), math.cos(cur.yaw + dyaw))
                    if not self._is_valid(nx, ny):
                        continue
                    cost = self.step
                    if reverse:
                        cost *= self.rev_penalty
                    if abs(steer) > 0.1:
                        cost *= self.turn_penalty
                    g = cur.g + cost
                    h = self._heuristic(nx, ny, gx, gy)
                    child = HybridAStarNode(nx, ny, nyaw, g, h, cur, steer, reverse)
                    nkey = child.key()
                    if nkey not in visited:
                        heapq.heappush(open_heap, child)

        if best is None:
            return
        poses = []
        node = best
        while node is not None:
            poses.append((node.x, node.y, node.yaw))
            node = node.parent
        poses.reverse()
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for px, py, pyaw in poses:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = px
            ps.pose.position.y = py
            cy, sy2 = math.cos(pyaw/2), math.sin(pyaw/2)
            ps.pose.orientation.w = cy
            ps.pose.orientation.z = sy2
            path.poses.append(ps)
        self.path_pub.publish(path)
        self.get_logger().info(f'Path planned: {len(path.poses)} poses')


def main(args=None):
    rclpy.init(args=args)
    node = HybridAStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
