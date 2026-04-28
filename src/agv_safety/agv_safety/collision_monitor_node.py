#!/usr/bin/env python3
"""Collision Monitor Node - monitors obstacle proximity for collision warnings."""
import math
import rclpy
from rclpy.node import Node
from agv_msgs.msg import ObstacleArray
from std_msgs.msg import String, Float32
from agv_msgs.srv import EmergencyStop


class CollisionMonitorNode(Node):
    def __init__(self):
        super().__init__('collision_monitor_node')
        self.declare_parameter('collision_threshold', 0.5)
        self.declare_parameter('warning_threshold', 2.0)
        self.declare_parameter('time_to_collision_thresh', 3.0)

        self.coll_thresh = self.get_parameter('collision_threshold').value
        self.warn_thresh = self.get_parameter('warning_threshold').value
        self.ttc_thresh = self.get_parameter('time_to_collision_thresh').value

        self.current_speed = 0.0
        self.obs_sub = self.create_subscription(
            ObstacleArray, '/agv/tracked_obstacles', self.obs_callback, 10)
        self.speed_sub = self.create_subscription(
            Float32, '/agv/actual_speed', self.speed_callback, 10)
        self.alert_pub = self.create_publisher(String, '/agv/collision_alert', 10)
        self.estop_cli = self.create_client(EmergencyStop, '/agv/emergency_stop')
        self.get_logger().info('Collision Monitor Node initialized')

    def speed_callback(self, msg: Float32):
        self.current_speed = msg.data

    def obs_callback(self, msg: ObstacleArray):
        min_dist = float('inf')
        min_obs = None
        for obs in msg.obstacles:
            d = math.sqrt(obs.position.x**2 + obs.position.y**2 + obs.position.z**2)
            if d < min_dist:
                min_dist = d
                min_obs = obs
        alert = String()
        if min_dist < self.coll_thresh:
            alert.data = f'COLLISION_IMMINENT:{min_dist:.2f}m'
            self.get_logger().error(f'Collision imminent! Obstacle at {min_dist:.2f}m')
            if self.estop_cli.service_is_ready():
                req = EmergencyStop.Request()
                req.activate = True
                req.reason = f'Collision imminent: obstacle at {min_dist:.2f}m'
                self.estop_cli.call_async(req)
        elif min_dist < self.warn_thresh:
            if self.current_speed > 0.1:
                ttc = min_dist / self.current_speed
                if ttc < self.ttc_thresh:
                    alert.data = f'COLLISION_WARNING:{min_dist:.2f}m,TTC:{ttc:.1f}s'
                else:
                    alert.data = f'OBSTACLE_NEAR:{min_dist:.2f}m'
            else:
                alert.data = f'OBSTACLE_NEAR:{min_dist:.2f}m'
        else:
            alert.data = 'CLEAR'
        self.alert_pub.publish(alert)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
