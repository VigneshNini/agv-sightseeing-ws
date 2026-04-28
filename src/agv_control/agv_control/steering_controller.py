#!/usr/bin/env python3
"""Steering Controller Node - servo-based steering angle control."""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from agv_msgs.msg import VehicleCommand


class SteeringControllerNode(Node):
    def __init__(self):
        super().__init__('steering_controller')
        self.declare_parameter('max_steering_angle', 0.6)
        self.declare_parameter('steering_rate_limit', 0.5)
        self.declare_parameter('servo_center_pwm', 1500)
        self.declare_parameter('servo_range_pwm', 500)
        self.declare_parameter('control_rate', 50.0)

        self.max_steer = self.get_parameter('max_steering_angle').value
        self.rate_limit = self.get_parameter('steering_rate_limit').value
        self.center_pwm = self.get_parameter('servo_center_pwm').value
        self.range_pwm = self.get_parameter('servo_range_pwm').value

        self.target_angle = 0.0
        self.current_angle = 0.0
        self.last_time = self.get_clock().now().nanoseconds * 1e-9

        self.cmd_sub = self.create_subscription(
            VehicleCommand, '/agv/vehicle_command', self.cmd_callback, 10)
        self.angle_sub = self.create_subscription(
            Float32, '/agv/actual_steering', self.angle_callback, 10)
        self.pwm_pub = self.create_publisher(Float32, '/agv/steering_pwm', 10)
        self.angle_pub = self.create_publisher(Float32, '/agv/steering_angle_cmd', 10)
        rate = self.get_parameter('control_rate').value
        self.create_timer(1.0/rate, self.control_loop)
        self.get_logger().info('Steering Controller Node initialized')

    def cmd_callback(self, msg: VehicleCommand):
        self.target_angle = max(-self.max_steer, min(self.max_steer, msg.steering_angle))

    def angle_callback(self, msg: Float32):
        self.current_angle = msg.data

    def control_loop(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0:
            return
        diff = self.target_angle - self.current_angle
        max_change = self.rate_limit * dt
        if abs(diff) > max_change:
            cmd_angle = self.current_angle + math.copysign(max_change, diff)
        else:
            cmd_angle = self.target_angle
        cmd_angle = max(-self.max_steer, min(self.max_steer, cmd_angle))
        norm = cmd_angle / self.max_steer
        pwm = self.center_pwm + int(norm * self.range_pwm)

        pwm_msg = Float32()
        pwm_msg.data = float(pwm)
        self.pwm_pub.publish(pwm_msg)

        angle_msg = Float32()
        angle_msg.data = float(cmd_angle)
        self.angle_pub.publish(angle_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SteeringControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
