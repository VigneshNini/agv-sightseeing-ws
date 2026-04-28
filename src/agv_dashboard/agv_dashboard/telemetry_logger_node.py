#!/usr/bin/env python3
"""Telemetry Logger Node - logs AGV telemetry to CSV files."""
import csv
import os
import time
import rclpy
from rclpy.node import Node
from agv_msgs.msg import AGVStatus
from std_msgs.msg import String


class TelemetryLoggerNode(Node):
    def __init__(self):
        super().__init__('telemetry_logger_node')
        self.declare_parameter('log_dir', '/var/log/agv')
        self.declare_parameter('log_rate', 1.0)
        self.declare_parameter('max_log_size_mb', 100)

        self.log_dir = self.get_parameter('log_dir').value
        self.max_size = self.get_parameter('max_log_size_mb').value * 1024 * 1024
        os.makedirs(self.log_dir, exist_ok=True)

        timestamp = time.strftime('%Y%m%d_%H%M%S')
        self.log_file = os.path.join(self.log_dir, f'agv_telemetry_{timestamp}.csv')
        self.csv_file = open(self.log_file, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'timestamp', 'battery_percent', 'speed_mps', 'heading_deg',
            'mode', 'current_stop', 'emergency_stop',
            'latitude', 'longitude', 'system_status'
        ])
        self.csv_file.flush()

        self.latest_status = None
        self.latest_state = 'IDLE'

        self.status_sub = self.create_subscription(AGVStatus, '/agv/status', self.status_cb, 10)
        self.state_sub = self.create_subscription(String, '/agv/behavior_state', self.state_cb, 10)

        rate = self.get_parameter('log_rate').value
        self.create_timer(1.0/rate, self.log_telemetry)
        self.get_logger().info(f'Telemetry Logger writing to {self.log_file}')

    def status_cb(self, msg: AGVStatus):
        self.latest_status = msg

    def state_cb(self, msg: String):
        self.latest_state = msg.data

    def log_telemetry(self):
        if self.latest_status is None:
            return
        if os.path.getsize(self.log_file) > self.max_size:
            self.get_logger().warn(f'Log file size limit reached: {self.log_file}')
            return
        msg = self.latest_status
        ts = time.strftime('%Y-%m-%d %H:%M:%S')
        self.writer.writerow([
            ts,
            round(float(msg.battery_percent), 2),
            round(float(msg.speed_mps), 3),
            round(float(msg.heading_deg), 2),
            msg.mode,
            msg.current_stop,
            msg.emergency_stop,
            round(float(msg.latitude), 6),
            round(float(msg.longitude), 6),
            msg.system_status,
        ])
        self.csv_file.flush()

    def destroy_node(self):
        if not self.csv_file.closed:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
