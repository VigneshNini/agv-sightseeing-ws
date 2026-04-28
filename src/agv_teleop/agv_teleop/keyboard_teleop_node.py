#!/usr/bin/env python3
"""Keyboard Teleop Node - keyboard-based teleoperation for AGV.
Keys:
  w/s: increase/decrease speed
  a/d: steer left/right
  space: brake
  e: emergency stop toggle
  q: quit
"""
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from agv_msgs.msg import VehicleCommand
from std_msgs.msg import Bool

MSG = """
AGV Keyboard Teleop
-------------------
w/s : Forward/Backward
a/d : Steer Left/Right
SPACE : Brake / Stop
e : Toggle Emergency Stop
q : Quit
"""

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('speed_step', 0.1)
        self.declare_parameter('max_steering', 0.5)
        self.declare_parameter('steer_step', 0.05)

        self.max_speed = self.get_parameter('max_speed').value
        self.speed_step = self.get_parameter('speed_step').value
        self.max_steer = self.get_parameter('max_steering').value
        self.steer_step = self.get_parameter('steer_step').value

        self.speed = 0.0
        self.steer = 0.0
        self.emergency = False
        self.settings = None

        self.cmd_pub = self.create_publisher(VehicleCommand, '/agv/teleop_command', 10)
        self.estop_pub = self.create_publisher(Bool, '/agv/emergency_stop_cmd', 10)
        self.create_timer(0.1, self.publish_cmd)
        self.create_timer(0.05, self.read_key)
        print(MSG)
        self.get_logger().info('Keyboard Teleop Node initialized')
        try:
            self.settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
        except Exception:
            pass

    def _get_key(self):
        try:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if rlist:
                return sys.stdin.read(1)
        except Exception:
            pass
        return None

    def read_key(self):
        key = self._get_key()
        if key is None:
            self.speed *= 0.8
            self.steer *= 0.8
            return
        if key == 'w':
            self.speed = min(self.speed + self.speed_step, self.max_speed)
        elif key == 's':
            self.speed = max(self.speed - self.speed_step, -self.max_speed)
        elif key == 'a':
            self.steer = max(self.steer - self.steer_step, -self.max_steer)
        elif key == 'd':
            self.steer = min(self.steer + self.steer_step, self.max_steer)
        elif key == ' ':
            self.speed = 0.0
            self.steer = 0.0
        elif key == 'e':
            self.emergency = not self.emergency
            msg = Bool(); msg.data = self.emergency
            self.estop_pub.publish(msg)
            self.get_logger().info(f'Emergency: {self.emergency}')
        elif key in ['q', '\x03']:
            raise SystemExit

    def publish_cmd(self):
        cmd = VehicleCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.speed = float(self.speed)
        cmd.steering_angle = float(self.steer)
        cmd.brake = 1.0 if self.emergency else 0.0
        cmd.emergency_stop = self.emergency
        cmd.mode = 'manual'
        self.cmd_pub.publish(cmd)

    def destroy_node(self):
        if self.settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
