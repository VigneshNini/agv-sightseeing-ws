#!/usr/bin/env python3
"""
Fake LiDAR Publisher — AGV Sightseeing Workspace
Publishes simulated sensor_msgs/LaserScan on /scan at 10 Hz.
Run without any physical hardware to test and visualize in RViz2.

Usage:
    source /opt/ros/humble/setup.bash
    python3 scripts/fake_lidar_publisher.py
"""

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header


class FakeLidarPublisher(Node):
    def __init__(self):
        super().__init__('fake_lidar_publisher')

        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)  # 10 Hz
        self.angle = 0.0  # moving obstacle angle

        self.get_logger().info('=' * 55)
        self.get_logger().info('  Fake LiDAR publisher started!')
        self.get_logger().info('  Topic  : /scan')
        self.get_logger().info('  Rate   : 10 Hz')
        self.get_logger().info('  Range  : 0.1 m — 15.0 m')
        self.get_logger().info('  Beams  : 360 (1 degree resolution)')
        self.get_logger().info('  Open RViz2 and add a LaserScan display')
        self.get_logger().info('  Set Fixed Frame to: laser_frame')
        self.get_logger().info('=' * 55)

    def publish_scan(self):
        msg = LaserScan()

        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'

        # Scan parameters — 360 degrees, 1 degree resolution
        num_readings = 360
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = (2.0 * math.pi) / num_readings
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 15.0

        ranges = []
        for i in range(num_readings):
            angle_deg = i  # 0 to 359 degrees
            r = 12.0  # default background range

            # ── Wall ahead (front, ~0 degrees) ──────────────────────────────
            if 350 <= angle_deg or angle_deg <= 10:
                r = 8.0 + 0.05 * math.sin(math.radians(angle_deg * 5))

            # ── Wall on the right (~90 degrees) ─────────────────────────────
            elif 80 <= angle_deg <= 100:
                r = 5.0 + 0.03 * math.cos(math.radians(angle_deg * 3))

            # ── Wall on the left (~270 degrees) ─────────────────────────────
            elif 260 <= angle_deg <= 280:
                r = 5.0 + 0.03 * math.cos(math.radians(angle_deg * 3))

            # ── Moving obstacle (sweeps full circle) ─────────────────────────
            obstacle_deg = int(math.degrees(self.angle)) % 360
            if abs(angle_deg - obstacle_deg) <= 3:
                r = 3.0  # obstacle at 3 m

            # ── Add small random noise (realistic sensor noise) ──────────────
            noise = 0.02 * math.sin(time.time() * 10 + i)
            r = max(msg.range_min, min(msg.range_max, r + noise))

            ranges.append(r)

        msg.ranges = ranges
        msg.intensities = [100.0] * num_readings  # uniform intensity

        self.publisher_.publish(msg)

        # Advance the moving obstacle
        self.angle += 0.05
        if self.angle > 2.0 * math.pi:
            self.angle = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = FakeLidarPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down fake LiDAR publisher.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
