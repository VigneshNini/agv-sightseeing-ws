#!/usr/bin/env python3
"""Passenger Display Node - sends content to passenger-facing screens."""
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from agv_msgs.msg import TourStop, AGVStatus


class PassengerDisplayNode(Node):
    def __init__(self):
        super().__init__('passenger_display_node')
        self.declare_parameter('display_topic', '/agv/display_content')
        self.declare_parameter('refresh_rate', 1.0)

        self.current_stop = None
        self.agv_status = None
        self.behavior_state = 'IDLE'

        display_topic = self.get_parameter('display_topic').value

        self.stop_sub = self.create_subscription(TourStop, '/agv/current_tour_stop', self.stop_callback, 10)
        self.status_sub = self.create_subscription(AGVStatus, '/agv/status', self.status_callback, 10)
        self.state_sub = self.create_subscription(String, '/agv/behavior_state', self.state_callback, 10)
        self.display_pub = self.create_publisher(String, display_topic, 10)
        self.screen_pub = self.create_publisher(String, '/agv/screen_data', 10)

        rate = self.get_parameter('refresh_rate').value
        self.create_timer(1.0/rate, self.update_display)
        self.get_logger().info('Passenger Display Node initialized')

    def stop_callback(self, msg: TourStop):
        self.current_stop = msg

    def status_callback(self, msg: AGVStatus):
        self.agv_status = msg

    def state_callback(self, msg: String):
        self.behavior_state = msg.data

    def update_display(self):
        data = {
            'state': self.behavior_state,
            'battery': 0,
            'speed': 0.0,
            'stop_name': 'Loading...',
            'stop_desc': '',
            'stop_num': 0,
            'total_stops': 5,
        }
        if self.agv_status:
            data['battery'] = int(self.agv_status.battery_percent)
            data['speed'] = round(float(self.agv_status.speed_mps), 1)
        if self.current_stop:
            data['stop_name'] = self.current_stop.name
            data['stop_desc'] = self.current_stop.description
            data['stop_num'] = self.current_stop.stop_id + 1
        if self.behavior_state == 'NAVIGATE':
            text = f'Heading to: {data["stop_name"]}'
        elif self.behavior_state == 'DWELL':
            text = f'Welcome to {data["stop_name"]}!\n{data["stop_desc"]}'
        elif self.behavior_state == 'TOUR_COMPLETE':
            text = 'Thank you for joining our sightseeing tour!'
        else:
            text = 'AGV Sightseeing Vehicle'
        display_msg = String()
        display_msg.data = text
        self.display_pub.publish(display_msg)
        screen_msg = String()
        screen_msg.data = json.dumps(data)
        self.screen_pub.publish(screen_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PassengerDisplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
