#!/usr/bin/env python3
"""Emergency Stop Node - hardware emergency stop interface."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from agv_msgs.srv import EmergencyStop


class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')
        self.declare_parameter('estop_gpio_pin', 18)
        self.declare_parameter('active_low', True)
        self.declare_parameter('poll_rate', 20.0)

        self.estop_active = False
        self.gpio_pin = self.get_parameter('estop_gpio_pin').value
        self.active_low = self.get_parameter('active_low').value

        self.hw_pub = self.create_publisher(Bool, '/agv/hw_emergency_stop', 10)
        self.estop_sub = self.create_subscription(Bool, '/agv/emergency', self.emergency_callback, 10)
        self.estop_cli = self.create_client(EmergencyStop, '/agv/emergency_stop')

        rate = self.get_parameter('poll_rate').value
        self.create_timer(1.0/rate, self.poll_hardware)
        self.get_logger().info(f'Emergency Stop Node initialized (GPIO pin {self.gpio_pin})')

    def emergency_callback(self, msg: Bool):
        self.estop_active = msg.data
        if msg.data:
            self._activate_hw_estop()

    def _activate_hw_estop(self):
        self.get_logger().error('EMERGENCY STOP ACTIVATED')

    def _read_gpio(self):
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            val = GPIO.input(self.gpio_pin)
            return not val if self.active_low else val
        except ImportError:
            return False
        except Exception as e:
            self.get_logger().debug(f'GPIO read error: {e}')
            return False

    def poll_hardware(self):
        hw_estop = self._read_gpio()
        if hw_estop != self.estop_active:
            self.estop_active = hw_estop
            if hw_estop:
                self.get_logger().error('Hardware E-Stop pressed!')
        msg = Bool()
        msg.data = hw_estop
        self.hw_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
