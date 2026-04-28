#!/usr/bin/env python3
"""Wheel Odometry Node - Ackermann steering odometry from wheel encoders."""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
import tf2_ros


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')
        self.declare_parameter('wheel_radius', 0.25)
        self.declare_parameter('wheelbase', 1.2)
        self.declare_parameter('track_width', 0.8)
        self.declare_parameter('encoder_resolution', 4096)
        self.declare_parameter('publish_rate', 50.0)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value
        self.encoder_resolution = self.get_parameter('encoder_resolution').value

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.prev_ticks = None
        self.last_time = None

        self.pub = self.create_publisher(Odometry, '/agv/wheel_odometry', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.enc_sub = self.create_subscription(
            Float32MultiArray, '/agv/encoder_ticks', self.encoder_callback, 10)
        self.steering_sub = self.create_subscription(
            Float32MultiArray, '/agv/steering_angle', self.steering_callback, 10)
        self.steering_angle = 0.0
        self.get_logger().info('Wheel Odometry Node initialized')

    def steering_callback(self, msg: Float32MultiArray):
        if msg.data:
            self.steering_angle = msg.data[0]

    def encoder_callback(self, msg: Float32MultiArray):
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9
        if len(msg.data) < 4:
            return
        ticks = list(msg.data)
        if self.prev_ticks is None or self.last_time is None:
            self.prev_ticks = ticks
            self.last_time = now_sec
            return
        dt = now_sec - self.last_time
        if dt <= 0:
            return
        dticks = [ticks[i] - self.prev_ticks[i] for i in range(4)]
        meters_per_tick = 2.0 * math.pi * self.wheel_radius / self.encoder_resolution
        left_dist = (dticks[0] + dticks[1]) / 2.0 * meters_per_tick
        right_dist = (dticks[2] + dticks[3]) / 2.0 * meters_per_tick
        dist = (left_dist + right_dist) / 2.0
        if abs(self.steering_angle) > 1e-4:
            R = self.wheelbase / math.tan(self.steering_angle)
            dtheta = dist / R
        else:
            dtheta = (right_dist - left_dist) / self.track_width
        self.x += dist * math.cos(self.yaw + dtheta / 2)
        self.y += dist * math.sin(self.yaw + dtheta / 2)
        self.yaw += dtheta
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.vx = dist / dt * math.cos(self.yaw)
        self.vy = dist / dt * math.sin(self.yaw)
        self.wz = dtheta / dt
        self.prev_ticks = ticks
        self.last_time = now_sec
        self._publish(now)

    def _publish(self, now):
        stamp = now.to_msg()
        cy, sy = math.cos(self.yaw/2), math.sin(self.yaw/2)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.w = cy
        odom.pose.pose.orientation.z = sy
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.wz
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.01
        self.pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.w = cy
        t.transform.rotation.z = sy
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
