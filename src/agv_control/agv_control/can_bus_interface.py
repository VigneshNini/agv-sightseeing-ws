#!/usr/bin/env python3
"""CAN Bus Interface - interfaces with vehicle CAN bus for actuation."""
import struct
import rclpy
from rclpy.node import Node
from agv_msgs.msg import VehicleCommand
from std_msgs.msg import Float32MultiArray


class CANBusInterface(Node):
    def __init__(self):
        super().__init__('can_bus_interface')
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('steering_can_id', 0x201)
        self.declare_parameter('throttle_can_id', 0x202)
        self.declare_parameter('brake_can_id', 0x203)
        self.declare_parameter('status_can_id', 0x180)
        self.declare_parameter('max_steering_angle', 0.6)
        self.declare_parameter('max_speed', 2.5)

        self.can_iface = self.get_parameter('can_interface').value
        self.steer_id = self.get_parameter('steering_can_id').value
        self.throttle_id = self.get_parameter('throttle_can_id').value
        self.brake_id = self.get_parameter('brake_can_id').value
        self.max_steer = self.get_parameter('max_steering_angle').value
        self.max_speed = self.get_parameter('max_speed').value

        self.cmd_sub = self.create_subscription(
            VehicleCommand, '/agv/vehicle_command', self.cmd_callback, 10)
        self.feedback_pub = self.create_publisher(
            Float32MultiArray, '/agv/can_feedback', 10)
        self.can_socket = None
        self._init_can()
        self.get_logger().info(f'CAN Bus Interface initialized on {self.can_iface}')

    def _init_can(self):
        try:
            import socket
            self.can_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.can_socket.bind((self.can_iface,))
            self.can_socket.setblocking(False)
            self.get_logger().info('CAN socket connected')
        except Exception as e:
            self.get_logger().warn(f'CAN socket unavailable (simulation mode): {e}')
            self.can_socket = None

    def _send_can_frame(self, can_id, data_bytes):
        if self.can_socket is None:
            return
        try:
            import socket
            frame = struct.pack('=IB3x8s', can_id, len(data_bytes),
                                data_bytes + b'\x00' * (8 - len(data_bytes)))
            self.can_socket.send(frame)
        except Exception as e:
            self.get_logger().debug(f'CAN send error: {e}')

    def cmd_callback(self, msg: VehicleCommand):
        if msg.emergency_stop:
            self._send_can_frame(self.brake_id, struct.pack('Hh', 1000, 0))
            self._send_can_frame(self.throttle_id, struct.pack('Hh', 0, 0))
            return
        steer_norm = max(-1.0, min(1.0, msg.steering_angle / self.max_steer))
        steer_raw = int(steer_norm * 32767)
        speed_norm = max(0.0, min(1.0, msg.speed / self.max_speed))
        throttle_raw = int(speed_norm * 1000)
        brake_raw = int(max(0.0, min(1.0, msg.brake)) * 1000)
        self._send_can_frame(self.steer_id, struct.pack('h6x', steer_raw))
        self._send_can_frame(self.throttle_id, struct.pack('H6x', throttle_raw))
        self._send_can_frame(self.brake_id, struct.pack('H6x', brake_raw))

    def destroy_node(self):
        if self.can_socket:
            self.can_socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CANBusInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
