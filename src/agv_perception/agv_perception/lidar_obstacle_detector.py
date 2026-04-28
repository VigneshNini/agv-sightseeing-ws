#!/usr/bin/env python3
"""LiDAR Obstacle Detector - Euclidean cluster extraction on point clouds."""
import math
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from agv_msgs.msg import Obstacle, ObstacleArray
from geometry_msgs.msg import Point, Vector3


class LidarObstacleDetector(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_detector')
        self.declare_parameter('max_range', 30.0)
        self.declare_parameter('min_range', 0.3)
        self.declare_parameter('ground_removal_height', -0.3)
        self.declare_parameter('cluster_tolerance', 0.5)
        self.declare_parameter('min_cluster_size', 10)
        self.declare_parameter('max_cluster_size', 5000)
        self.declare_parameter('voxel_size', 0.1)

        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value
        self.ground_h = self.get_parameter('ground_removal_height').value
        self.cluster_tol = self.get_parameter('cluster_tolerance').value
        self.min_pts = self.get_parameter('min_cluster_size').value
        self.max_pts = self.get_parameter('max_cluster_size').value
        self.voxel_size = self.get_parameter('voxel_size').value

        self.sub = self.create_subscription(
            PointCloud2, '/velodyne_points', self.cloud_callback, 10)
        self.pub = self.create_publisher(ObstacleArray, '/agv/obstacles', 10)
        self.obstacle_id = 0
        self.get_logger().info('LiDAR Obstacle Detector initialized')

    def _parse_cloud(self, msg: PointCloud2):
        pts = []
        step = msg.point_step
        data = msg.data
        fields = {f.name: f.offset for f in msg.fields}
        xo = fields.get('x', 0)
        yo = fields.get('y', 4)
        zo = fields.get('z', 8)
        for i in range(0, len(data), step):
            x = struct.unpack_from('f', data, i + xo)[0]
            y = struct.unpack_from('f', data, i + yo)[0]
            z = struct.unpack_from('f', data, i + zo)[0]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            r = math.sqrt(x*x + y*y)
            if r < self.min_range or r > self.max_range:
                continue
            if z < self.ground_h:
                continue
            pts.append([x, y, z])
        return np.array(pts) if pts else np.zeros((0, 3))

    def _euclidean_cluster(self, pts):
        if len(pts) == 0:
            return []
        n = len(pts)
        visited = np.zeros(n, dtype=bool)
        clusters = []
        tol2 = self.cluster_tol ** 2
        for i in range(n):
            if visited[i]:
                continue
            cluster = [i]
            queue = [i]
            visited[i] = True
            while queue:
                cur = queue.pop()
                diffs = pts[:, :2] - pts[cur, :2]
                dists2 = (diffs * diffs).sum(axis=1)
                neighbors = np.where((dists2 < tol2) & (~visited))[0]
                for nb in neighbors:
                    visited[nb] = True
                    cluster.append(nb)
                    queue.append(nb)
            if self.min_pts <= len(cluster) <= self.max_pts:
                clusters.append(cluster)
        return clusters

    def cloud_callback(self, msg: PointCloud2):
        pts = self._parse_cloud(msg)
        if len(pts) < self.min_pts:
            arr = ObstacleArray()
            arr.header = msg.header
            self.pub.publish(arr)
            return
        clusters = self._euclidean_cluster(pts)
        obs_array = ObstacleArray()
        obs_array.header = msg.header
        for cluster_idx in clusters:
            cluster_pts = pts[cluster_idx]
            center = cluster_pts.mean(axis=0)
            min_pt = cluster_pts.min(axis=0)
            max_pt = cluster_pts.max(axis=0)
            size = max_pt - min_pt
            obs = Obstacle()
            obs.position = Point(x=float(center[0]), y=float(center[1]), z=float(center[2]))
            obs.size = Vector3(x=float(size[0]), y=float(size[1]), z=float(size[2]))
            obs.confidence = min(1.0, len(cluster_idx) / 100.0)
            obs.id = self.obstacle_id
            self.obstacle_id += 1
            vol = size[0] * size[1] * size[2]
            if vol < 0.1:
                obs.type = 'small_object'
            elif size[2] > 1.5:
                obs.type = 'person'
            elif vol < 5.0:
                obs.type = 'vehicle'
            else:
                obs.type = 'unknown'
            obs_array.obstacles.append(obs)
        self.pub.publish(obs_array)


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
