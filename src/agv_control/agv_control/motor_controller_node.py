#!/usr/bin/env python3
"""Motor Controller Node - PID-based motor speed control."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from agv_msgs.msg import VehicleCommand


class PIDController:
    def __init__(self, kp, ki, kd, max_out=1.0, min_out=-1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_out, self.min_out = max_out, min_out
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint, measurement, dt):
        error = setpoint - measurement
        self.integral += error * dt
        self.integral = max(-10.0, min(10.0, self.integral))
        derivative = (error - self.prev_error) / max(dt, 1e-6)
        self.prev_error = error
        output = self.kp*error + self.ki*self.integral + self.kd*derivative
        return max(self.min_out, min(self.max_out, output))


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('ki', 0.5)
        self.declare_parameter('kd', 0.1)
        self.declare_parameter('max_throttle', 1.0)
        self.declare_parameter('wheel_radius', 0.25)
        self.declare_parameter('control_rate', 50.0)

        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        max_t = self.get_parameter('max_throttle').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        self.fl_pid = PIDController(kp, ki, kd, max_t, -max_t)
        self.fr_pid = PIDController(kp, ki, kd, max_t, -max_t)
        self.rl_pid = PIDController(kp, ki, kd, max_t, -max_t)
        self.rr_pid = PIDController(kp, ki, kd, max_t, -max_t)

        self.target_speed = 0.0
        self.actual_speed = 0.0
        self.last_time = self.get_clock().now().nanoseconds * 1e-9

        self.cmd_sub = self.create_subscription(
            VehicleCommand, '/agv/vehicle_command', self.cmd_callback, 10)
        self.speed_sub = self.create_subscription(
            Float32, '/agv/actual_speed', self.speed_callback, 10)
        self.throttle_pub = self.create_publisher(Float32, '/agv/throttle_cmd', 10)
        rate = self.get_parameter('control_rate').value
        self.create_timer(1.0/rate, self.control_loop)
        self.get_logger().info('Motor Controller Node initialized')

    def cmd_callback(self, msg: VehicleCommand):
        self.target_speed = msg.speed

    def speed_callback(self, msg: Float32):
        self.actual_speed = msg.data

    def control_loop(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0:
            return
        throttle = self.fl_pid.compute(self.target_speed, self.actual_speed, dt)
        out = Float32()
        out.data = float(throttle)
        self.throttle_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
