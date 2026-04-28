#!/usr/bin/env python3
"""Map Server Node - serves 2D occupancy grid map from file."""
import os
import math
import yaml
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from std_srvs.srv import Trigger


class MapServerNode(Node):
    def __init__(self):
        super().__init__('map_server_node')
        self.declare_parameter('map_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 1.0)

        self.frame_id = self.get_parameter('frame_id').value
        map_file = self.get_parameter('map_file').value
        self.map_msg = None

        if map_file and os.path.exists(map_file):
            self._load_map(map_file)
        else:
            self._create_default_map()

        self.pub = self.create_publisher(OccupancyGrid, '/map', 1)
        self.get_map_srv = self.create_service(GetMap, '/get_map', self.get_map_callback)
        self.reload_srv = self.create_service(Trigger, '/map_server/reload', self.reload_callback)
        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self.publish_map)
        self.get_logger().info('Map Server Node initialized')

    def _create_default_map(self):
        """Create a simple default empty map."""
        w, h = 200, 200
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = self.frame_id
        self.map_msg.info.resolution = 0.1
        self.map_msg.info.width = w
        self.map_msg.info.height = h
        self.map_msg.info.origin.position.x = -10.0
        self.map_msg.info.origin.position.y = -10.0
        self.map_msg.info.origin.orientation.w = 1.0
        self.map_msg.data = [0] * (w * h)
        self.get_logger().info('Created default 20x20m empty map')

    def _load_map(self, yaml_file):
        """Load map from YAML+PGM file pair."""
        try:
            with open(yaml_file, 'r') as f:
                meta = yaml.safe_load(f)
            image_file = meta.get('image', '')
            if not os.path.isabs(image_file):
                image_file = os.path.join(os.path.dirname(yaml_file), image_file)
            resolution = meta.get('resolution', 0.05)
            origin = meta.get('origin', [0, 0, 0])
            negate = meta.get('negate', 0)
            occ_thresh = meta.get('occupied_thresh', 0.65)
            free_thresh = meta.get('free_thresh', 0.196)
            with open(image_file, 'rb') as f:
                content = f.read()
            lines = content.split(b'\n')
            idx = 0
            while lines[idx].startswith(b'#') or lines[idx].startswith(b'P'):
                idx += 1
            w, h = map(int, lines[idx].decode().split())
            idx += 1
            max_val = int(lines[idx].decode())
            idx += 1
            pixels = list(content[content.index(lines[idx]):])
            self.map_msg = OccupancyGrid()
            self.map_msg.header.frame_id = self.frame_id
            self.map_msg.info.resolution = resolution
            self.map_msg.info.width = w
            self.map_msg.info.height = h
            self.map_msg.info.origin.position.x = origin[0]
            self.map_msg.info.origin.position.y = origin[1]
            self.map_msg.info.origin.orientation.w = 1.0
            data = []
            for px in pixels[:w*h]:
                val = px / max_val
                if negate:
                    val = 1 - val
                if val >= occ_thresh:
                    data.append(100)
                elif val <= free_thresh:
                    data.append(0)
                else:
                    data.append(-1)
            self.map_msg.data = data
            self.get_logger().info(f'Map loaded: {w}x{h}, res={resolution}m')
        except Exception as e:
            self.get_logger().error(f'Failed to load map: {e}')
            self._create_default_map()

    def publish_map(self):
        if self.map_msg:
            self.map_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.map_msg)

    def get_map_callback(self, request, response):
        if self.map_msg:
            response.map = self.map_msg
        return response

    def reload_callback(self, request, response):
        map_file = self.get_parameter('map_file').value
        if map_file and os.path.exists(map_file):
            self._load_map(map_file)
            response.success = True
            response.message = 'Map reloaded'
        else:
            response.success = False
            response.message = 'No valid map file specified'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MapServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
