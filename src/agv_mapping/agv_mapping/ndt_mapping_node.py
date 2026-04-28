#!/usr/bin/env python3
"""NDT Mapping Node - Normal Distributions Transform based mapping."""
import math
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class NDTMappingNode(Node):
    def __init__(self):
        super().__init__('ndt_mapping_node')
        self.declare_parameter('voxel_size', 1.0)
        self.declare_parameter('max_range', 50.0)
        self.declare_parameter('ndt_resolution', 1.0)
        self.declare_parameter('ndt_step_size', 0.1)
        self.declare_parameter('ndt_max_iterations', 30)

        self.voxel_size = self.get_parameter('voxel_size').value
        self.max_range = self.get_parameter('max_range').value
        self.ndt_res = self.get_parameter('ndt_resolution').value

        self.global_map = {}  # voxel_key -> list of points
        self.pose = np.zeros(3)  # x, y, yaw
        self.prev_cloud = None
        self.initialized = False

        self.cloud_sub = self.create_subscription(
            PointCloud2, '/velodyne_points', self.cloud_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/agv/odometry', self.odom_callback, 10)
        self.map_pub = self.create_publisher(PointCloud2, '/agv/map_cloud', 1)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.map_timer = self.create_timer(5.0, self.publish_map)
        self.get_logger().info('NDT Mapping Node initialized')

    def _parse_cloud(self, msg: PointCloud2):
        pts = []
        step = msg.point_step
        data = msg.data
        fields = {f.name: f.offset for f in msg.fields}
        for i in range(0, len(data), step):
            x = struct.unpack_from('f', data, i + fields.get('x', 0))[0]
            y = struct.unpack_from('f', data, i + fields.get('y', 4))[0]
            z = struct.unpack_from('f', data, i + fields.get('z', 8))[0]
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                r = math.sqrt(x*x + y*y)
                if r < self.max_range:
                    pts.append([x, y, z])
        return np.array(pts) if pts else np.zeros((0, 3))

    def odom_callback(self, msg: Odometry):
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.pose[2] = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2 + q.z**2))
        self.initialized = True

    def cloud_callback(self, msg: PointCloud2):
        if not self.initialized:
            return
        pts = self._parse_cloud(msg)
        if len(pts) == 0:
            return
        cy, sy = math.cos(self.pose[2]), math.sin(self.pose[2])
        R = np.array([[cy, -sy], [sy, cy]])
        world_pts = (R @ pts[:, :2].T).T + self.pose[:2]
        for i, wp in enumerate(world_pts):
            key = (int(wp[0] / self.voxel_size), int(wp[1] / self.voxel_size))
            if key not in self.global_map:
                self.global_map[key] = []
            self.global_map[key].append([wp[0], wp[1], pts[i, 2]])
        if len(self.global_map) % 100 == 0:
            self.get_logger().info(f'Map size: {len(self.global_map)} voxels')

    def publish_map(self):
        if not self.global_map:
            return
        all_pts = []
        for pts in self.global_map.values():
            if pts:
                all_pts.append(pts[len(pts)//2])
        if not all_pts:
            return
        pts_arr = np.array(all_pts, dtype=np.float32)
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.height = 1
        msg.width = len(pts_arr)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(pts_arr)
        msg.data = pts_arr.tobytes()
        msg.is_dense = True
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NDTMappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
