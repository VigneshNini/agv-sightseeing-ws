#!/usr/bin/env python3
"""Safety Arbiter Node - central safety coordinator monitoring all safety signals."""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from agv_msgs.msg import ObstacleArray, VehicleCommand, AGVStatus
from agv_msgs.srv import EmergencyStop


class SafetyLevel:
    SAFE = 'SAFE'
    CAUTION = 'CAUTION'
    WARNING = 'WARNING'
    EMERGENCY = 'EMERGENCY'


class SafetyArbiterNode(Node):
    def __init__(self):
        super().__init__('safety_arbiter_node')
        self.declare_parameter('emergency_stop_distance', 0.5)
        self.declare_parameter('warning_distance', 2.0)
        self.declare_parameter('caution_distance', 5.0)
        self.declare_parameter('max_speed_caution', 1.0)
        self.declare_parameter('watchdog_timeout', 2.0)

        self.estop_dist = self.get_parameter('emergency_stop_distance').value
        self.warn_dist = self.get_parameter('warning_distance').value
        self.caution_dist = self.get_parameter('caution_distance').value
        self.max_caution_v = self.get_parameter('max_speed_caution').value
        self.watchdog_to = self.get_parameter('watchdog_timeout').value

        self.safety_level = SafetyLevel.SAFE
        self.emergency_active = False
        self.obstacle_distances = []
        self.hw_estop = False
        self.cmd_in = None
        self.last_cmd_time = None

        self.obs_sub = self.create_subscription(
            ObstacleArray, '/agv/tracked_obstacles', self.obs_callback, 10)
        self.cmd_sub = self.create_subscription(
            VehicleCommand, '/agv/vehicle_command', self.cmd_callback, 10)
        self.hw_estop_sub = self.create_subscription(
            Bool, '/agv/hw_emergency_stop', self.hw_estop_callback, 10)
        self.estop_sub = self.create_subscription(
            Bool, '/agv/emergency_stop_cmd', self.estop_cmd_callback, 10)

        self.cmd_pub = self.create_publisher(VehicleCommand, '/agv/safe_command', 10)
        self.safety_pub = self.create_publisher(String, '/agv/safety_level', 10)
        self.estop_pub = self.create_publisher(Bool, '/agv/emergency', 10)

        self.estop_srv = self.create_service(EmergencyStop, '/agv/emergency_stop', self.estop_service)

        self.create_timer(0.05, self.safety_loop)
        self.get_logger().info('Safety Arbiter Node initialized')

    def obs_callback(self, msg: ObstacleArray):
        self.obstacle_distances = []
        for obs in msg.obstacles:
            d = math.sqrt(obs.position.x**2 + obs.position.y**2)
            self.obstacle_distances.append(d)

    def cmd_callback(self, msg: VehicleCommand):
        self.cmd_in = msg
        self.last_cmd_time = self.get_clock().now()

    def hw_estop_callback(self, msg: Bool):
        self.hw_estop = msg.data
        if msg.data:
            self.emergency_active = True

    def estop_cmd_callback(self, msg: Bool):
        if msg.data:
            self.emergency_active = True
        else:
            self.emergency_active = False

    def estop_service(self, request, response):
        self.emergency_active = request.activate
        response.success = True
        response.message = f'Emergency stop {"activated" if request.activate else "deactivated"}: {request.reason}'
        self.get_logger().warn(response.message)
        return response

    def _compute_safety_level(self):
        if self.emergency_active or self.hw_estop:
            return SafetyLevel.EMERGENCY
        if self.obstacle_distances:
            min_dist = min(self.obstacle_distances)
            if min_dist < self.estop_dist:
                return SafetyLevel.EMERGENCY
            elif min_dist < self.warn_dist:
                return SafetyLevel.WARNING
            elif min_dist < self.caution_dist:
                return SafetyLevel.CAUTION
        if self.last_cmd_time is not None:
            elapsed = (self.get_clock().now().nanoseconds - self.last_cmd_time.nanoseconds) * 1e-9
            if elapsed > self.watchdog_to:
                return SafetyLevel.WARNING
        return SafetyLevel.SAFE

    def safety_loop(self):
        self.safety_level = self._compute_safety_level()
        safety_msg = String()
        safety_msg.data = self.safety_level
        self.safety_pub.publish(safety_msg)
        estop_msg = Bool()
        estop_msg.data = self.safety_level == SafetyLevel.EMERGENCY
        self.estop_pub.publish(estop_msg)
        if self.cmd_in is None:
            return
        safe_cmd = VehicleCommand()
        safe_cmd.header = self.cmd_in.header
        safe_cmd.mode = self.cmd_in.mode
        if self.safety_level == SafetyLevel.EMERGENCY:
            safe_cmd.speed = 0.0
            safe_cmd.brake = 1.0
            safe_cmd.emergency_stop = True
            safe_cmd.steering_angle = self.cmd_in.steering_angle
        elif self.safety_level == SafetyLevel.WARNING:
            safe_cmd.speed = min(self.cmd_in.speed, 0.5)
            safe_cmd.brake = 0.0
            safe_cmd.steering_angle = self.cmd_in.steering_angle
        elif self.safety_level == SafetyLevel.CAUTION:
            safe_cmd.speed = min(self.cmd_in.speed, self.max_caution_v)
            safe_cmd.brake = 0.0
            safe_cmd.steering_angle = self.cmd_in.steering_angle
        else:
            safe_cmd = self.cmd_in
        self.cmd_pub.publish(safe_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyArbiterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
