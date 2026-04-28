#!/usr/bin/env python3
"""Velocity Profiler - computes speed profile along path with acceleration constraints."""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray


class VelocityProfiler(Node):
    def __init__(self):
        super().__init__('velocity_profiler')
        self.declare_parameter('max_speed', 2.5)
        self.declare_parameter('max_accel', 1.5)
        self.declare_parameter('max_decel', 2.0)
        self.declare_parameter('max_curvature_speed', 1.0)
        self.declare_parameter('curvature_lookahead', 5)

        self.max_v = self.get_parameter('max_speed').value
        self.max_a = self.get_parameter('max_accel').value
        self.max_d = self.get_parameter('max_decel').value
        self.curv_v = self.get_parameter('max_curvature_speed').value
        self.lookahead = self.get_parameter('curvature_lookahead').value

        self.path_sub = self.create_subscription(Path, '/agv/planned_path', self.path_callback, 10)
        self.profile_pub = self.create_publisher(Float32MultiArray, '/agv/velocity_profile', 10)
        self.get_logger().info('Velocity Profiler initialized')

    def _path_length(self, poses, i, j):
        total = 0.0
        for k in range(i, j):
            p1, p2 = poses[k].pose.position, poses[k+1].pose.position
            total += math.sqrt((p2.x-p1.x)**2 + (p2.y-p1.y)**2)
        return total

    def _curvature(self, poses, i):
        n = len(poses)
        if i == 0 or i >= n-1:
            return 0.0
        p0, p1, p2 = poses[i-1].pose.position, poses[i].pose.position, poses[i+1].pose.position
        ax, ay = p1.x-p0.x, p1.y-p0.y
        bx, by = p2.x-p1.x, p2.y-p1.y
        cross = ax*by - ay*bx
        dot = ax*bx + ay*by
        if dot == 0:
            return 0.0
        return abs(cross) / (dot**2 + cross**2)**0.5

    def path_callback(self, msg: Path):
        n = len(msg.poses)
        if n < 2:
            return
        speeds = [self.max_v] * n
        speeds[-1] = 0.0
        for i in range(n):
            curv = self._curvature(msg.poses, i)
            if curv > 0.1:
                speeds[i] = min(speeds[i], self.curv_v)
        for i in range(n-2, -1, -1):
            p1 = msg.poses[i].pose.position
            p2 = msg.poses[i+1].pose.position
            ds = math.sqrt((p2.x-p1.x)**2 + (p2.y-p1.y)**2)
            if ds > 0:
                v_dec = math.sqrt(speeds[i+1]**2 + 2*self.max_d*ds)
                speeds[i] = min(speeds[i], v_dec)
        for i in range(1, n):
            p1 = msg.poses[i-1].pose.position
            p2 = msg.poses[i].pose.position
            ds = math.sqrt((p2.x-p1.x)**2 + (p2.y-p1.y)**2)
            if ds > 0:
                v_acc = math.sqrt(speeds[i-1]**2 + 2*self.max_a*ds)
                speeds[i] = min(speeds[i], v_acc)
        out = Float32MultiArray()
        out.data = [float(v) for v in speeds]
        self.profile_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityProfiler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
