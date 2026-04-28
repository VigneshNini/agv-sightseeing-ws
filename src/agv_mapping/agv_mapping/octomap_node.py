#!/usr/bin/env python3
"""OctoMap Node - 3D occupancy mapping from point clouds."""
import math
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry


class OctomapNode(Node):
    def __init__(self):
        super().__init__('octomap_node')
        self.declare_parameter('resolution', 0.2)
        self.declare_parameter('max_range', 20.0)
        self.declare_parameter('hit_prob', 0.7)
        self.declare_parameter('miss_prob', 0.4)
        self.declare_parameter('clamp_min', 0.12)
        self.declare_parameter('clamp_max', 0.97)

        self.res = self.get_parameter('resolution').value
        self.max_range = self.get_parameter('max_range').value
        hit = self.get_parameter('hit_prob').value
        miss = self.get_parameter('miss_prob').value
        self.log_hit = math.log(hit / (1 - hit))
        self.log_miss = math.log(miss / (1 - miss))
        self.log_min = math.log(self.get_parameter('clamp_min').value / (1 - self.get_parameter('clamp_min').value))
        self.log_max = math.log(self.get_parameter('clamp_max').value / (1 - self.get_parameter('clamp_max').value))

        self.voxels = {}  # key -> log_odds
        self.robot_pos = [0.0, 0.0, 0.0]

        self.cloud_sub = self.create_subscription(
            PointCloud2, '/velodyne_points', self.cloud_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/agv/odometry', self.odom_callback, 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/agv/occupancy_grid', 1)
        self.create_timer(2.0, self.publish_grid)
        self.get_logger().info('OctoMap Node initialized')

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self.robot_pos = [p.x, p.y, p.z]

    def _bresenham_3d(self, start, end):
        """Generate voxel cells along ray using 3D Bresenham."""
        cells = []
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        steps = max(abs(int(dx/self.res)), abs(int(dy/self.res)), 1)
        for i in range(steps):
            t = i / steps
            x = start[0] + t * dx
            y = start[1] + t * dy
            key = (int(x / self.res), int(y / self.res))
            cells.append(key)
        return cells

    def cloud_callback(self, msg: PointCloud2):
        step = msg.point_step
        data = msg.data
        fields = {f.name: f.offset for f in msg.fields}
        rx, ry = self.robot_pos[0], self.robot_pos[1]
        for i in range(0, min(len(data), step * 500), step):
            x = struct.unpack_from('f', data, i + fields.get('x', 0))[0]
            y = struct.unpack_from('f', data, i + fields.get('y', 4))[0]
            z = struct.unpack_from('f', data, i + fields.get('z', 8))[0]
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            r = math.sqrt((x-rx)**2 + (y-ry)**2)
            if r > self.max_range:
                continue
            wx, wy = rx + x, ry + y
            hit_key = (int(wx / self.res), int(wy / self.res))
            self.voxels[hit_key] = max(self.log_min, min(self.log_max,
                self.voxels.get(hit_key, 0) + self.log_hit))
            for key in self._bresenham_3d([rx, ry], [wx, wy])[:-1]:
                self.voxels[key] = max(self.log_min, min(self.log_max,
                    self.voxels.get(key, 0) + self.log_miss))

    def publish_grid(self):
        if not self.voxels:
            return
        keys = list(self.voxels.keys())
        xs = [k[0] for k in keys]
        ys = [k[1] for k in keys]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        w = max_x - min_x + 1
        h = max_y - min_y + 1
        if w * h > 100000:
            return
        grid_data = [-1] * (w * h)
        for (kx, ky), logodds in self.voxels.items():
            ix = kx - min_x
            iy = ky - min_y
            prob = 1.0 / (1.0 + math.exp(-logodds))
            grid_data[iy * w + ix] = int(prob * 100)
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.res
        msg.info.width = w
        msg.info.height = h
        msg.info.origin.position.x = min_x * self.res
        msg.info.origin.position.y = min_y * self.res
        msg.info.origin.orientation.w = 1.0
        msg.data = grid_data
        self.grid_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OctomapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
