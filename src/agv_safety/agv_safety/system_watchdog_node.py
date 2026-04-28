#!/usr/bin/env python3
"""System Watchdog Node - monitors node heartbeats and system health."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Header
from agv_msgs.msg import AGVStatus


class SystemWatchdogNode(Node):
    MONITORED_TOPICS = {
        '/agv/pose': 5.0,
        '/agv/odometry': 5.0,
        '/agv/obstacles': 5.0,
        '/agv/vehicle_command': 10.0,
        '/agv/safety_level': 5.0,
    }

    def __init__(self):
        super().__init__('system_watchdog_node')
        self.declare_parameter('watchdog_rate', 5.0)
        self.declare_parameter('alert_timeout', 5.0)

        self.last_seen = {}
        self.alert_timeout = self.get_parameter('alert_timeout').value
        self.subscribers = {}
        self.health_pub = self.create_publisher(String, '/agv/system_health', 10)
        self.watchdog_ok_pub = self.create_publisher(Bool, '/agv/watchdog_ok', 10)

        for topic in self.MONITORED_TOPICS:
            self.last_seen[topic] = self.get_clock().now()
            self.subscribers[topic] = self.create_subscription(
                Header if topic.endswith('/header') else String,
                topic, self._make_callback(topic), 10)

        rate = self.get_parameter('watchdog_rate').value
        self.create_timer(1.0/rate, self.watchdog_check)
        self.get_logger().info(f'System Watchdog monitoring {len(self.MONITORED_TOPICS)} topics')

    def _make_callback(self, topic):
        def callback(msg):
            self.last_seen[topic] = self.get_clock().now()
        return callback

    def watchdog_check(self):
        now = self.get_clock().now()
        failed = []
        for topic, timeout in self.MONITORED_TOPICS.items():
            elapsed = (now.nanoseconds - self.last_seen[topic].nanoseconds) * 1e-9
            if elapsed > timeout:
                failed.append(f'{topic}({elapsed:.1f}s)')
        health = String()
        ok = Bool()
        if failed:
            health.data = f'DEGRADED: {", ".join(failed)}'
            ok.data = False
            self.get_logger().warn(f'Watchdog: {health.data}')
        else:
            health.data = 'OK'
            ok.data = True
        self.health_pub.publish(health)
        self.watchdog_ok_pub.publish(ok)


def main(args=None):
    rclpy.init(args=args)
    node = SystemWatchdogNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
