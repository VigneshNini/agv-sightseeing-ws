#!/usr/bin/env python3
"""Driveable Surface Node - classifies driveable vs non-driveable surface from point clouds."""
import math
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


class DriveableSurfaceNode(Node):
    def __init__(self):
        super().__init__('driveable_surface_node')
        self.declare_parameter('ground_height_threshold', 0.15)
        self.declare_parameter('slope_threshold', 0.2)
        self.declare_parameter('min_ground_points', 50)
        self.declare_parameter('roi_radius', 5.0)

        self.ground_thresh = self.get_parameter('ground_height_threshold').value
        self.slope_thresh = self.get_parameter('slope_threshold').value
        self.min_pts = self.get_parameter('min_ground_points').value
        self.roi_radius = self.get_parameter('roi_radius').value

        self.sub = self.create_subscription(
            PointCloud2, '/velodyne_points', self.cloud_callback, 10)
        self.pub = self.create_publisher(Bool, '/agv/is_driveable', 10)
        self.get_logger().info('Driveable Surface Node initialized')

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
            if r < self.roi_radius:
                pts.append([x, y, z])
        return np.array(pts) if pts else np.zeros((0, 3))

    def _estimate_ground_plane(self, pts):
        """RANSAC ground plane estimation."""
        if len(pts) < 4:
            return None, None
        best_inliers = 0
        best_normal = None
        best_d = None
        for _ in range(20):
            idx = np.random.choice(len(pts), 3, replace=False)
            p0, p1, p2 = pts[idx[0]], pts[idx[1]], pts[idx[2]]
            v1 = p1 - p0
            v2 = p2 - p0
            normal = np.cross(v1, v2)
            if np.linalg.norm(normal) < 1e-6:
                continue
            normal = normal / np.linalg.norm(normal)
            d = -normal @ p0
            dists = np.abs(pts @ normal + d)
            inliers = (dists < self.ground_thresh).sum()
            if inliers > best_inliers:
                best_inliers = inliers
                best_normal = normal
                best_d = d
        return best_normal, best_d

    def cloud_callback(self, msg: PointCloud2):
        pts = self._parse_cloud(msg)
        result = Bool()
        if len(pts) < self.min_pts:
            result.data = False
            self.pub.publish(result)
            return
        normal, d = self._estimate_ground_plane(pts)
        if normal is None:
            result.data = False
            self.pub.publish(result)
            return
        vertical = np.array([0, 0, 1])
        slope = abs(float(np.dot(normal, vertical)))
        result.data = slope > (1.0 - self.slope_thresh)
        self.pub.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = DriveableSurfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
