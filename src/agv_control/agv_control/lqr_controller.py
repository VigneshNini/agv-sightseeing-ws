#!/usr/bin/env python3
"""LQR Lateral Controller for Ackermann vehicle path tracking."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from agv_msgs.msg import VehicleCommand
from geometry_msgs.msg import PoseWithCovarianceStamped


class LQRController(Node):
    def __init__(self):
        super().__init__('lqr_controller')
        self.declare_parameter('wheelbase', 1.2)
        self.declare_parameter('max_steering', 0.6)
        self.declare_parameter('lqr_q_lat', 10.0)
        self.declare_parameter('lqr_q_yaw', 5.0)
        self.declare_parameter('lqr_r', 1.0)
        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('control_rate', 20.0)

        self.L = self.get_parameter('wheelbase').value
        self.max_steer = self.get_parameter('max_steering').value
        self.Q = np.diag([self.get_parameter('lqr_q_lat').value, self.get_parameter('lqr_q_yaw').value])
        self.R = np.array([[self.get_parameter('lqr_r').value]])
        self.lookahead = self.get_parameter('lookahead_distance').value

        self.path = None
        self.state = None  # x, y, yaw, v

        self.path_sub = self.create_subscription(Path, '/agv/planned_path', self.path_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/agv/pose', self.pose_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/agv/odometry', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/agv/lqr_command', 10)
        rate = self.get_parameter('control_rate').value
        self.create_timer(1.0/rate, self.control_loop)
        self.speed = 0.0
        self.get_logger().info('LQR Controller initialized')

    def path_callback(self, msg: Path):
        self.path = msg.poses

    def odom_callback(self, msg: Odometry):
        self.speed = msg.twist.twist.linear.x

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2+q.z**2))
        self.state = [p.position.x, p.position.y, yaw, self.speed]

    def _solve_lqr(self, A, B, Q, R, iterations=100):
        P = Q.copy()
        for _ in range(iterations):
            Pn = A.T @ P @ A - A.T @ P @ B @ np.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A + Q
            if np.max(np.abs(Pn - P)) < 1e-6:
                break
            P = Pn
        K = np.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A
        return K

    def _find_nearest_point(self, x, y):
        if not self.path:
            return None, 0
        min_d = float('inf')
        idx = 0
        for i, pose in enumerate(self.path):
            dx = pose.pose.position.x - x
            dy = pose.pose.position.y - y
            d = math.sqrt(dx*dx + dy*dy)
            if d < min_d:
                min_d = d
                idx = i
        return self.path[idx], idx

    def control_loop(self):
        if self.state is None or self.path is None or len(self.path) < 2:
            return
        x, y, yaw, v = self.state
        ref, idx = self._find_nearest_point(x, y)
        if idx >= len(self.path) - 1:
            cmd = VehicleCommand()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.speed = 0.0; cmd.brake = 1.0
            self.cmd_pub.publish(cmd)
            return
        ri = min(idx + 1, len(self.path) - 1)
        ref_next = self.path[ri]
        ref_yaw = math.atan2(
            ref_next.pose.position.y - ref.pose.position.y,
            ref_next.pose.position.x - ref.pose.position.x)
        dx = x - ref.pose.position.x
        dy = y - ref.pose.position.y
        lat_err = -math.sin(ref_yaw) * dx + math.cos(ref_yaw) * dy
        yaw_err = math.atan2(math.sin(yaw - ref_yaw), math.cos(yaw - ref_yaw))
        v_safe = max(v, 0.1)
        dt = 0.05
        A = np.array([[1.0, v_safe*dt], [0.0, 1.0]])
        B = np.array([[v_safe*dt**2/2], [dt]])
        K = self._solve_lqr(A, B, self.Q, self.R)
        state_err = np.array([lat_err, yaw_err])
        steer_fb = float(-K @ state_err)
        steer = max(-self.max_steer, min(self.max_steer, steer_fb))
        cmd = VehicleCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.steering_angle = steer
        cmd.speed = 1.5
        cmd.mode = 'auto'
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LQRController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
