#!/usr/bin/env python3
"""MPC Local Planner - Model Predictive Control for trajectory tracking."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from agv_msgs.msg import VehicleCommand, ObstacleArray


class MPCLocalPlanner(Node):
    def __init__(self):
        super().__init__('mpc_local_planner')
        self.declare_parameter('horizon_steps', 20)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('max_speed', 2.5)
        self.declare_parameter('min_speed', 0.0)
        self.declare_parameter('max_steering', 0.6)
        self.declare_parameter('max_steering_rate', 0.3)
        self.declare_parameter('max_accel', 1.5)
        self.declare_parameter('max_decel', 2.0)
        self.declare_parameter('weight_tracking', 10.0)
        self.declare_parameter('weight_speed', 1.0)
        self.declare_parameter('weight_steering', 0.5)
        self.declare_parameter('weight_obstacle', 100.0)
        self.declare_parameter('wheelbase', 1.2)

        self.N = self.get_parameter('horizon_steps').value
        self.dt = self.get_parameter('dt').value
        self.max_v = self.get_parameter('max_speed').value
        self.min_v = self.get_parameter('min_speed').value
        self.max_steer = self.get_parameter('max_steering').value
        self.max_steer_rate = self.get_parameter('max_steering_rate').value
        self.max_a = self.get_parameter('max_accel').value
        self.max_d = self.get_parameter('max_decel').value
        self.w_track = self.get_parameter('weight_tracking').value
        self.w_speed = self.get_parameter('weight_speed').value
        self.w_steer = self.get_parameter('weight_steering').value
        self.w_obs = self.get_parameter('weight_obstacle').value
        self.L = self.get_parameter('wheelbase').value

        self.robot_state = np.zeros(4)  # x, y, yaw, v
        self.ref_path = None
        self.obstacles = []
        self.prev_steer = 0.0
        self.prev_speed = 0.0

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/agv/pose', self.pose_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/agv/planned_path', self.path_callback, 10)
        self.obs_sub = self.create_subscription(
            ObstacleArray, '/agv/tracked_obstacles', self.obs_callback, 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/agv/vehicle_command', 10)
        self.create_timer(self.dt, self.compute_control)
        self.get_logger().info('MPC Local Planner initialized')

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2 + q.z**2))
        self.robot_state[0] = p.position.x
        self.robot_state[1] = p.position.y
        self.robot_state[2] = yaw

    def path_callback(self, msg: Path):
        self.ref_path = msg.poses

    def obs_callback(self, msg: ObstacleArray):
        self.obstacles = msg.obstacles

    def _bicycle_model(self, state, v, steer, dt):
        x, y, yaw, _ = state
        beta = math.atan2(0.5 * self.L * math.tan(steer), self.L)
        nx = x + v * math.cos(yaw + beta) * dt
        ny = y + v * math.sin(yaw + beta) * dt
        nyaw = yaw + v / self.L * math.sin(beta) * 2 * dt
        nyaw = math.atan2(math.sin(nyaw), math.cos(nyaw))
        return np.array([nx, ny, nyaw, v])

    def _find_nearest_ref(self, state):
        if not self.ref_path:
            return None, 0
        min_dist = float('inf')
        idx = 0
        for i, pose in enumerate(self.ref_path):
            dx = pose.pose.position.x - state[0]
            dy = pose.pose.position.y - state[1]
            d = math.sqrt(dx*dx + dy*dy)
            if d < min_dist:
                min_dist = d
                idx = i
        return self.ref_path[idx], idx

    def _compute_cost(self, state, v, steer, ref_idx):
        cost = 0.0
        s = state.copy()
        for k in range(self.N):
            s = self._bicycle_model(s, v, steer, self.dt)
            ri = min(ref_idx + k + 1, len(self.ref_path) - 1)
            ref = self.ref_path[ri]
            dx = s[0] - ref.pose.position.x
            dy = s[1] - ref.pose.position.y
            cost += self.w_track * (dx**2 + dy**2)
            target_v = self.max_v * 0.5
            cost += self.w_speed * (v - target_v)**2
            cost += self.w_steer * steer**2
            for obs in self.obstacles:
                odx = s[0] - obs.position.x
                ody = s[1] - obs.position.y
                odist = math.sqrt(odx**2 + ody**2)
                if odist < 3.0:
                    cost += self.w_obs / max(odist, 0.1)
        return cost

    def compute_control(self):
        if self.ref_path is None or len(self.ref_path) < 2:
            return
        _, ref_idx = self._find_nearest_ref(self.robot_state)
        if ref_idx >= len(self.ref_path) - 1:
            cmd = VehicleCommand()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.speed = 0.0
            cmd.brake = 1.0
            self.cmd_pub.publish(cmd)
            return
        best_cost = float('inf')
        best_v = 0.0
        best_steer = 0.0
        steers = np.linspace(-self.max_steer, self.max_steer, 7)
        speeds = np.linspace(self.min_v, self.max_v * 0.5, 5)
        for v in speeds:
            for steer in steers:
                if abs(steer - self.prev_steer) > self.max_steer_rate * self.dt * 10:
                    continue
                cost = self._compute_cost(self.robot_state, v, steer, ref_idx)
                if cost < best_cost:
                    best_cost = cost
                    best_v = v
                    best_steer = steer
        self.prev_steer = best_steer
        self.prev_speed = best_v
        cmd = VehicleCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.speed = float(best_v)
        cmd.steering_angle = float(best_steer)
        cmd.brake = 0.0
        cmd.mode = 'auto'
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MPCLocalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
