#!/usr/bin/env python3
"""Joystick Teleop Node - joystick-based teleoperation for AGV."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from agv_msgs.msg import VehicleCommand
from std_msgs.msg import Bool


class JoystickTeleopNode(Node):
    AXIS_SPEED = 1      # Left stick vertical
    AXIS_STEER = 3      # Right stick horizontal
    BTN_ESTOP = 0       # A button
    BTN_DEADMAN = 4     # LB button (deadman switch)
    BTN_MODE_AUTO = 1   # B button
    BTN_MODE_MANUAL = 2 # X button

    def __init__(self):
        super().__init__('joystick_teleop_node')
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('max_steering', 0.5)
        self.declare_parameter('deadman_required', True)
        self.declare_parameter('joy_topic', '/joy')

        self.max_speed = self.get_parameter('max_speed').value
        self.max_steer = self.get_parameter('max_steering').value
        self.deadman_req = self.get_parameter('deadman_required').value

        self.deadman_active = False
        self.emergency = False
        self.mode = 'manual'

        joy_topic = self.get_parameter('joy_topic').value
        self.joy_sub = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/agv/teleop_command', 10)
        self.estop_pub = self.create_publisher(Bool, '/agv/emergency_stop_cmd', 10)
        self.get_logger().info('Joystick Teleop Node initialized')

    def joy_callback(self, msg: Joy):
        axes = msg.axes
        buttons = msg.buttons

        if len(buttons) > self.BTN_DEADMAN:
            self.deadman_active = bool(buttons[self.BTN_DEADMAN])
        if len(buttons) > self.BTN_ESTOP and buttons[self.BTN_ESTOP]:
            self.emergency = not self.emergency
            estop = Bool()
            estop.data = self.emergency
            self.estop_pub.publish(estop)
            self.get_logger().info(f'Emergency stop: {self.emergency}')
        if len(buttons) > self.BTN_MODE_AUTO and buttons[self.BTN_MODE_AUTO]:
            self.mode = 'auto'
            self.get_logger().info('Switched to AUTO mode')
        if len(buttons) > self.BTN_MODE_MANUAL and buttons[self.BTN_MODE_MANUAL]:
            self.mode = 'manual'
            self.get_logger().info('Switched to MANUAL mode')

        if self.deadman_req and not self.deadman_active:
            cmd = VehicleCommand()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.speed = 0.0; cmd.brake = 1.0; cmd.mode = self.mode
            self.cmd_pub.publish(cmd)
            return

        speed_raw = axes[self.AXIS_SPEED] if len(axes) > self.AXIS_SPEED else 0.0
        steer_raw = -axes[self.AXIS_STEER] if len(axes) > self.AXIS_STEER else 0.0

        cmd = VehicleCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.speed = float(speed_raw * self.max_speed)
        cmd.steering_angle = float(steer_raw * self.max_steer)
        cmd.brake = 1.0 if self.emergency else 0.0
        cmd.emergency_stop = self.emergency
        cmd.mode = self.mode
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
