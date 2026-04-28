#!/usr/bin/env python3
"""Camera Perception Node - lane detection and obstacle detection from camera images."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from agv_msgs.msg import ObstacleArray


class CameraPerceptionNode(Node):
    def __init__(self):
        super().__init__('camera_perception_node')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('lane_detection_enabled', True)
        self.declare_parameter('obstacle_detection_enabled', True)
        self.declare_parameter('confidence_threshold', 0.5)

        self.lane_detection = self.get_parameter('lane_detection_enabled').value
        self.obstacle_detection = self.get_parameter('obstacle_detection_enabled').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value

        cam_topic = self.get_parameter('camera_topic').value
        self.image_sub = self.create_subscription(Image, cam_topic, self.image_callback, 10)
        self.lane_pub = self.create_publisher(String, '/agv/lane_status', 10)
        self.cam_obs_pub = self.create_publisher(ObstacleArray, '/agv/camera_obstacles', 10)
        self.get_logger().info('Camera Perception Node initialized')

    def _convert_image(self, msg: Image):
        """Convert ROS Image to numpy array."""
        dtype = np.uint8
        n_channels = 3 if msg.encoding in ('rgb8', 'bgr8') else 1
        img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, n_channels)
        return img

    def _detect_lanes(self, img):
        """Detect lane markings using edge-based approach."""
        gray = img.mean(axis=2).astype(np.uint8) if img.ndim == 3 else img
        h, w = gray.shape
        roi = gray[h//2:, :]
        sobelx = np.gradient(roi.astype(float), axis=1)
        sobely = np.gradient(roi.astype(float), axis=0)
        edges = np.sqrt(sobelx**2 + sobely**2)
        thresh = edges.max() * 0.3
        edge_mask = edges > thresh
        left_count = edge_mask[:, :w//2].sum()
        right_count = edge_mask[:, w//2:].sum()
        if left_count > 100 and right_count > 100:
            return 'both_lanes'
        elif left_count > 100:
            return 'left_lane_only'
        elif right_count > 100:
            return 'right_lane_only'
        return 'no_lanes'

    def image_callback(self, msg: Image):
        try:
            img = self._convert_image(msg)
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed: {e}')
            return
        if self.lane_detection:
            lane_status = self._detect_lanes(img)
            lane_msg = String()
            lane_msg.data = lane_status
            self.lane_pub.publish(lane_msg)
        if self.obstacle_detection:
            arr = ObstacleArray()
            arr.header = msg.header
            self.cam_obs_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = CameraPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
