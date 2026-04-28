#!/usr/bin/env python3
"""Behavior Tree Node - mission behavior tree for AGV tour management.
States: IDLE -> NAVIGATE -> DWELL -> NEXT_STOP -> TOUR_COMPLETE
"""
import math
import enum
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from agv_msgs.msg import TourStop, AGVStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from agv_msgs.srv import EmergencyStop


class State(enum.Enum):
    IDLE = 'IDLE'
    NAVIGATE = 'NAVIGATE'
    DWELL = 'DWELL'
    NEXT_STOP = 'NEXT_STOP'
    TOUR_COMPLETE = 'TOUR_COMPLETE'
    EMERGENCY = 'EMERGENCY'


class BehaviorTreeNode(Node):
    def __init__(self):
        super().__init__('behavior_tree_node')
        self.declare_parameter('arrival_threshold', 1.5)
        self.declare_parameter('start_on_init', True)

        self.state = State.IDLE
        self.current_stop = None
        self.robot_pose = None
        self.dwell_start = None
        self.emergency = False

        self.arrival_thresh = self.get_parameter('arrival_threshold').value

        self.state_pub = self.create_publisher(String, '/agv/behavior_state', 10)
        self.status_pub = self.create_publisher(AGVStatus, '/agv/status', 10)

        self.stop_sub = self.create_subscription(TourStop, '/agv/current_tour_stop', self.stop_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/agv/pose', self.pose_callback, 10)
        self.emergency_sub = self.create_subscription(String, '/agv/emergency', self.emergency_callback, 10)

        self.create_timer(0.5, self.tick)
        self.get_logger().info('Behavior Tree Node initialized')
        if self.get_parameter('start_on_init').value:
            self.state = State.NAVIGATE

    def stop_callback(self, msg: TourStop):
        self.current_stop = msg

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.robot_pose = msg

    def emergency_callback(self, msg: String):
        if msg.data:
            self.emergency = True
            self.state = State.EMERGENCY

    def _distance_to_stop(self):
        if self.robot_pose is None or self.current_stop is None:
            return float('inf')
        rx = self.robot_pose.pose.pose.position.x
        ry = self.robot_pose.pose.pose.position.y
        tx = self.current_stop.longitude
        ty = self.current_stop.latitude
        return math.sqrt((rx-tx)**2 + (ry-ty)**2)

    def tick(self):
        now = self.get_clock().now()
        if self.state == State.IDLE:
            pass
        elif self.state == State.NAVIGATE:
            if self.current_stop is None:
                pass
            elif self._distance_to_stop() < self.arrival_thresh:
                self.get_logger().info(f'Arrived at: {self.current_stop.name}')
                self.state = State.DWELL
                self.dwell_start = now
        elif self.state == State.DWELL:
            if self.dwell_start is not None and self.current_stop is not None:
                elapsed = (now.nanoseconds - self.dwell_start.nanoseconds) * 1e-9
                if elapsed >= self.current_stop.dwell_time:
                    self.state = State.NEXT_STOP
        elif self.state == State.NEXT_STOP:
            self.state = State.NAVIGATE
        elif self.state == State.TOUR_COMPLETE:
            pass
        elif self.state == State.EMERGENCY:
            pass

        state_msg = String()
        state_msg.data = self.state.value
        self.state_pub.publish(state_msg)

        status = AGVStatus()
        status.header.stamp = now.to_msg()
        status.mode = self.state.value
        status.emergency_stop = self.emergency
        if self.current_stop:
            status.current_stop = self.current_stop.name
        if self.robot_pose:
            status.latitude = float(self.robot_pose.pose.pose.position.y)
            status.longitude = float(self.robot_pose.pose.pose.position.x)
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
