#!/usr/bin/env python3
"""LiDAR Odometry Node - ICP scan matching for incremental pose estimation."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import struct


class LidarOdometryNode(Node):
    def __init__(self):
        super().__init__('lidar_odometry_node')
        self.declare_parameter('max_icp_iterations', 50)
        self.declare_parameter('icp_convergence_threshold', 1e-4)
        self.declare_parameter('max_correspondence_distance', 1.0)
        self.declare_parameter('voxel_size', 0.2)

        self.max_iters = self.get_parameter('max_icp_iterations').value
        self.conv_thresh = self.get_parameter('icp_convergence_threshold').value
        self.max_corr_dist = self.get_parameter('max_correspondence_distance').value
        self.voxel_size = self.get_parameter('voxel_size').value

        self.prev_cloud = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.pub = self.create_publisher(Odometry, '/agv/lidar_odometry', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.sub = self.create_subscription(
            PointCloud2, '/velodyne_points', self.cloud_callback, 10)
        self.get_logger().info('LiDAR Odometry Node initialized')

    def _parse_pointcloud2(self, msg: PointCloud2):
        """Extract XY points from PointCloud2 message."""
        points = []
        point_step = msg.point_step
        data = msg.data
        x_offset = next((f.offset for f in msg.fields if f.name == 'x'), 0)
        y_offset = next((f.offset for f in msg.fields if f.name == 'y'), 4)
        z_offset = next((f.offset for f in msg.fields if f.name == 'z'), 8)
        for i in range(0, len(data), point_step):
            x = struct.unpack_from('f', data, i + x_offset)[0]
            y = struct.unpack_from('f', data, i + y_offset)[0]
            z = struct.unpack_from('f', data, i + z_offset)[0]
            if math.isfinite(x) and math.isfinite(y) and abs(z) < 1.0:
                points.append([x, y])
        return np.array(points) if points else np.zeros((0, 2))

    def _voxel_downsample(self, points):
        if len(points) == 0:
            return points
        keys = np.floor(points / self.voxel_size).astype(int)
        unique_keys = np.unique(keys, axis=0)
        result = []
        for k in unique_keys:
            mask = np.all(keys == k, axis=1)
            result.append(points[mask].mean(axis=0))
        return np.array(result)

    def _icp_2d(self, source, target):
        """Simple 2D ICP implementation."""
        src = source.copy()
        T = np.eye(3)
        for _ in range(self.max_iters):
            dists = np.linalg.norm(src[:, None, :] - target[None, :, :], axis=2)
            idx = np.argmin(dists, axis=1)
            min_dists = dists[np.arange(len(src)), idx]
            mask = min_dists < self.max_corr_dist
            if mask.sum() < 3:
                break
            src_m = src[mask]
            tgt_m = target[idx[mask]]
            src_c = src_m.mean(axis=0)
            tgt_c = tgt_m.mean(axis=0)
            src_n = src_m - src_c
            tgt_n = tgt_m - tgt_c
            H = src_n.T @ tgt_n
            U, _, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = Vt.T @ U.T
            t = tgt_c - R @ src_c
            dT = np.eye(3)
            dT[:2, :2] = R
            dT[:2, 2] = t
            T = dT @ T
            src = (R @ src.T).T + t
            if np.linalg.norm(t) < self.conv_thresh:
                break
        return T

    def cloud_callback(self, msg: PointCloud2):
        pts = self._parse_pointcloud2(msg)
        if len(pts) < 10:
            return
        pts = self._voxel_downsample(pts)
        if self.prev_cloud is None or len(self.prev_cloud) < 10:
            self.prev_cloud = pts
            return
        T = self._icp_2d(pts, self.prev_cloud)
        dx = T[0, 2]
        dy = T[1, 2]
        dtheta = math.atan2(T[1, 0], T[0, 0])
        self.x += dx * math.cos(self.yaw) - dy * math.sin(self.yaw)
        self.y += dx * math.sin(self.yaw) + dy * math.cos(self.yaw)
        self.yaw += dtheta
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.prev_cloud = pts
        self._publish(msg.header.stamp)

    def _publish(self, stamp):
        cy, sy = math.cos(self.yaw/2), math.sin(self.yaw/2)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.w = cy
        odom.pose.pose.orientation.z = sy
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.02
        self.pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'lidar_odom'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.w = cy
        t.transform.rotation.z = sy
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = LidarOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
