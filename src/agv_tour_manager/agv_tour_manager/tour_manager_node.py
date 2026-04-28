#!/usr/bin/env python3
"""Tour Manager Node - orchestrates the sightseeing tour schedule."""
import math
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool
from agv_msgs.msg import TourStop, AGVStatus
from agv_msgs.srv import SetTourStop


class TourManagerNode(Node):
    def __init__(self):
        super().__init__('tour_manager_node')
        self.declare_parameter('tour_config_file', '')
        self.declare_parameter('default_dwell_time', 30.0)
        self.declare_parameter('arrival_threshold', 1.5)
        self.declare_parameter('loop_tour', False)

        self.dwell_time = self.get_parameter('default_dwell_time').value
        self.arrival_thresh = self.get_parameter('arrival_threshold').value
        self.loop_tour = self.get_parameter('loop_tour').value

        self.tour_stops = []
        self.current_idx = 0
        self.dwell_start = None
        self.tour_active = False
        self.behavior_state = 'IDLE'

        config = self.get_parameter('tour_config_file').value
        if config:
            self._load_config(config)
        else:
            self._default_stops()

        self.stop_pub = self.create_publisher(TourStop, '/agv/current_tour_stop', 10)
        self.audio_pub = self.create_publisher(String, '/agv/play_audio', 10)
        self.display_pub = self.create_publisher(String, '/agv/display_content', 10)
        self.tour_idx_pub = self.create_publisher(Int32, '/agv/tour_stop_index', 10)

        self.state_sub = self.create_subscription(String, '/agv/behavior_state', self.state_callback, 10)
        self.start_sub = self.create_subscription(Bool, '/agv/start_tour', self.start_callback, 10)
        self.stop_sub_cmd = self.create_subscription(Int32, '/agv/goto_stop', self.goto_callback, 10)
        self.set_stop_srv = self.create_service(SetTourStop, '/agv/set_tour_stop', self.set_stop_callback)

        self.create_timer(1.0, self.manage_tour)
        self.get_logger().info(f'Tour Manager initialized with {len(self.tour_stops)} stops')

    def _default_stops(self):
        self.tour_stops = [
            {'stop_id': 0, 'name': 'Entrance Gate', 'latitude': 0.0, 'longitude': 0.0,
             'dwell_time': 30.0, 'audio_file': 'entrance.mp3',
             'description': 'Welcome! This is the main entrance of the park.'},
            {'stop_id': 1, 'name': 'Historic Fountain', 'latitude': 20.0, 'longitude': 15.0,
             'dwell_time': 45.0, 'audio_file': 'fountain.mp3',
             'description': 'Built in 1824, this fountain is a historic landmark.'},
            {'stop_id': 2, 'name': 'Rose Garden', 'latitude': 40.0, 'longitude': 10.0,
             'dwell_time': 60.0, 'audio_file': 'rose_garden.mp3',
             'description': 'Over 500 species of roses bloom here year-round.'},
            {'stop_id': 3, 'name': 'Art Museum', 'latitude': 50.0, 'longitude': 30.0,
             'dwell_time': 90.0, 'audio_file': 'museum.mp3',
             'description': 'Home to a world-class collection of modern art.'},
            {'stop_id': 4, 'name': 'Viewpoint Terrace', 'latitude': 30.0, 'longitude': 50.0,
             'dwell_time': 60.0, 'audio_file': 'viewpoint.mp3',
             'description': 'Enjoy a panoramic view of the entire park from here.'},
        ]

    def _load_config(self, config_file):
        try:
            with open(config_file, 'r') as f:
                data = yaml.safe_load(f)
            self.tour_stops = data.get('tour_stops', [])
            self.get_logger().info(f'Loaded {len(self.tour_stops)} stops from {config_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to load tour config: {e}')
            self._default_stops()

    def state_callback(self, msg: String):
        self.behavior_state = msg.data
        if msg.data == 'DWELL' and self.dwell_start is None:
            self.dwell_start = self.get_clock().now()
            self._trigger_stop_content()
        elif msg.data == 'NAVIGATE':
            self.dwell_start = None

    def start_callback(self, msg: Bool):
        self.tour_active = msg.data
        if msg.data:
            self.get_logger().info('Tour started!')
            self._publish_current_stop()
        else:
            self.get_logger().info('Tour paused.')

    def goto_callback(self, msg: Int32):
        idx = msg.data
        if 0 <= idx < len(self.tour_stops):
            self.current_idx = idx
            self._publish_current_stop()

    def set_stop_callback(self, request, response):
        for i, stop in enumerate(self.tour_stops):
            if stop['stop_id'] == request.stop_id:
                self.current_idx = i
                self._publish_current_stop()
                response.success = True
                response.message = f'Navigating to {stop["name"]}'
                return response
        response.success = False
        response.message = f'Stop {request.stop_id} not found'
        return response

    def _publish_current_stop(self):
        if not self.tour_stops:
            return
        s = self.tour_stops[self.current_idx]
        msg = TourStop()
        msg.stop_id = s['stop_id']
        msg.name = s['name']
        msg.latitude = float(s['latitude'])
        msg.longitude = float(s['longitude'])
        msg.dwell_time = float(s.get('dwell_time', self.dwell_time))
        msg.audio_file = s.get('audio_file', '')
        msg.description = s.get('description', '')
        self.stop_pub.publish(msg)
        idx_msg = Int32()
        idx_msg.data = self.current_idx
        self.tour_idx_pub.publish(idx_msg)

    def _trigger_stop_content(self):
        if not self.tour_stops:
            return
        s = self.tour_stops[self.current_idx]
        audio = String()
        audio.data = s.get('audio_file', '')
        if audio.data:
            self.audio_pub.publish(audio)
        display = String()
        display.data = f'Stop {self.current_idx+1}/{len(self.tour_stops)}: {s["name"]}\n{s.get("description", "")}'
        self.display_pub.publish(display)

    def manage_tour(self):
        if not self.tour_active:
            return
        if self.behavior_state == 'NEXT_STOP':
            self.current_idx = (self.current_idx + 1) % len(self.tour_stops) if self.loop_tour else min(self.current_idx + 1, len(self.tour_stops) - 1)
            self._publish_current_stop()
        elif self.behavior_state == 'TOUR_COMPLETE':
            self.get_logger().info('Tour complete!')
            self.tour_active = False
        else:
            self._publish_current_stop()


def main(args=None):
    rclpy.init(args=args)
    node = TourManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
