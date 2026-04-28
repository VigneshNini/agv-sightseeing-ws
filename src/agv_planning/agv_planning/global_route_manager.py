#!/usr/bin/env python3
"""Global Route Manager - manages tour stops and computes routes between them."""
import math
import yaml
import rclpy
from rclpy.node import Node
from agv_msgs.msg import TourStop
from agv_msgs.srv import SetTourStop
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Int32


class GlobalRouteManager(Node):
    def __init__(self):
        super().__init__('global_route_manager')
        self.declare_parameter('tour_config_file', '')
        self.declare_parameter('arrival_threshold', 1.5)

        self.tour_stops = []
        self.current_stop_idx = 0
        self.robot_pose = None
        self.arrival_threshold = self.get_parameter('arrival_threshold').value

        config_file = self.get_parameter('tour_config_file').value
        if config_file:
            self._load_tour(config_file)
        else:
            self._load_default_tour()

        self.stop_pub = self.create_publisher(TourStop, '/agv/current_tour_stop', 10)
        self.path_pub = self.create_publisher(Path, '/agv/global_path', 10)
        self.stop_idx_pub = self.create_publisher(Int32, '/agv/tour_stop_index', 10)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/agv/pose', self.pose_callback, 10)
        self.set_stop_srv = self.create_service(
            SetTourStop, '/agv/set_tour_stop', self.set_stop_callback)
        self.create_timer(1.0, self.update_route)
        self.get_logger().info(f'Global Route Manager initialized with {len(self.tour_stops)} stops')

    def _load_default_tour(self):
        """Load a default sightseeing tour."""
        self.tour_stops = [
            {'stop_id': 0, 'name': 'Entrance Gate', 'latitude': 0.0, 'longitude': 0.0,
             'dwell_time': 30.0, 'description': 'Welcome to the sightseeing tour!'},
            {'stop_id': 1, 'name': 'Historic Fountain', 'latitude': 20.0, 'longitude': 15.0,
             'dwell_time': 45.0, 'description': 'The 200-year-old central fountain.'},
            {'stop_id': 2, 'name': 'Rose Garden', 'latitude': 40.0, 'longitude': 10.0,
             'dwell_time': 60.0, 'description': 'Over 500 varieties of roses.'},
            {'stop_id': 3, 'name': 'Art Museum', 'latitude': 50.0, 'longitude': 30.0,
             'dwell_time': 90.0, 'description': 'World-class art collection.'},
            {'stop_id': 4, 'name': 'Viewpoint Terrace', 'latitude': 30.0, 'longitude': 50.0,
             'dwell_time': 60.0, 'description': 'Panoramic view of the park.'},
        ]

    def _load_tour(self, config_file):
        try:
            with open(config_file, 'r') as f:
                data = yaml.safe_load(f)
            self.tour_stops = data.get('tour_stops', [])
        except Exception as e:
            self.get_logger().error(f'Failed to load tour config: {e}')
            self._load_default_tour()

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.robot_pose = msg

    def set_stop_callback(self, request, response):
        for i, stop in enumerate(self.tour_stops):
            if stop['stop_id'] == request.stop_id:
                self.current_stop_idx = i
                response.success = True
                response.message = f'Set to stop: {stop["name"]}''
                self._publish_current_stop()
                return response
        response.success = False
        response.message = f'Stop {request.stop_id} not found'
        return response

    def _dist_to_stop(self, stop):
        if self.robot_pose is None:
            return float('inf')
        rx = self.robot_pose.pose.pose.position.x
        ry = self.robot_pose.pose.pose.position.y
        dx = stop['longitude'] - rx
        dy = stop['latitude'] - ry
        return math.sqrt(dx*dx + dy*dy)

    def _publish_current_stop(self):
        if not self.tour_stops:
            return
        stop = self.tour_stops[self.current_stop_idx]
        msg = TourStop()
        msg.stop_id = stop['stop_id']
        msg.name = stop['name']
        msg.latitude = float(stop['latitude'])
        msg.longitude = float(stop['longitude'])
        msg.dwell_time = float(stop.get('dwell_time', 30.0))
        msg.description = stop.get('description', '')
        msg.audio_file = stop.get('audio_file', '')
        self.stop_pub.publish(msg)

        idx_msg = Int32()
        idx_msg.data = self.current_stop_idx
        self.stop_idx_pub.publish(idx_msg)

    def _publish_path_to_stop(self):
        if not self.tour_stops or self.robot_pose is None:
            return
        stop = self.tour_stops[self.current_stop_idx]
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        rx = self.robot_pose.pose.pose.position.x
        ry = self.robot_pose.pose.pose.position.y
        tx = float(stop['longitude'])
        ty = float(stop['latitude'])
        n_points = max(2, int(math.sqrt((tx-rx)**2 + (ty-ry)**2) / 0.5))
        for i in range(n_points + 1):
            t = i / n_points
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = rx + t * (tx - rx)
            pose.pose.position.y = ry + t * (ty - ry)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)

    def update_route(self):
        self._publish_current_stop()
        self._publish_path_to_stop()


def main(args=None):
    rclpy.init(args=args)
    node = GlobalRouteManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
