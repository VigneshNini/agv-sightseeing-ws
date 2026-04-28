#!/usr/bin/env python3
"""Object Tracker Node - SORT-style multi-object tracker with Kalman filters."""
import numpy as np
import rclpy
from rclpy.node import Node
from agv_msgs.msg import Obstacle, ObstacleArray
from geometry_msgs.msg import Vector3


class KalmanTrack:
    """Single object Kalman filter track."""
    _count = 0

    def __init__(self, obs: Obstacle):
        KalmanTrack._count += 1
        self.id = KalmanTrack._count
        self.hits = 1
        self.age = 0
        self.time_since_update = 0
        dt = 0.1
        self.F = np.eye(6)
        self.F[0, 3] = self.F[1, 4] = self.F[2, 5] = dt
        self.H = np.zeros((3, 6))
        self.H[0, 0] = self.H[1, 1] = self.H[2, 2] = 1.0
        self.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1])
        self.R = np.diag([0.5, 0.5, 0.5])
        self.P = np.eye(6) * 10.0
        self.x = np.array([
            obs.position.x, obs.position.y, obs.position.z,
            obs.velocity.x, obs.velocity.y, obs.velocity.z
        ])
        self.size = [obs.size.x, obs.size.y, obs.size.z]
        self.type = obs.type

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.age += 1
        self.time_since_update += 1

    def update(self, obs: Obstacle):
        z = np.array([obs.position.x, obs.position.y, obs.position.z])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P
        self.hits += 1
        self.time_since_update = 0
        self.size = [obs.size.x, obs.size.y, obs.size.z]
        self.type = obs.type

    def to_obstacle(self) -> Obstacle:
        from geometry_msgs.msg import Point
        obs = Obstacle()
        obs.id = self.id
        obs.position = Point(x=float(self.x[0]), y=float(self.x[1]), z=float(self.x[2]))
        obs.velocity = Vector3(x=float(self.x[3]), y=float(self.x[4]), z=float(self.x[5]))
        obs.size = Vector3(x=self.size[0], y=self.size[1], z=self.size[2])
        obs.type = self.type
        obs.confidence = min(1.0, self.hits / 10.0)
        return obs


class ObjectTrackerNode(Node):
    def __init__(self):
        super().__init__('object_tracker_node')
        self.declare_parameter('max_age', 5)
        self.declare_parameter('min_hits', 3)
        self.declare_parameter('iou_threshold', 0.3)

        self.max_age = self.get_parameter('max_age').value
        self.min_hits = self.get_parameter('min_hits').value
        self.iou_thresh = self.get_parameter('iou_threshold').value

        self.tracks = []
        self.sub = self.create_subscription(
            ObstacleArray, '/agv/obstacles', self.obs_callback, 10)
        self.pub = self.create_publisher(ObstacleArray, '/agv/tracked_obstacles', 10)
        self.get_logger().info('Object Tracker Node initialized')

    def _distance(self, track: KalmanTrack, obs: Obstacle) -> float:
        dx = track.x[0] - obs.position.x
        dy = track.x[1] - obs.position.y
        dz = track.x[2] - obs.position.z
        return (dx*dx + dy*dy + dz*dz) ** 0.5

    def _associate(self, detections):
        if not self.tracks or not detections:
            return [], list(range(len(self.tracks))), list(range(len(detections)))
        dist_matrix = np.array([[self._distance(t, d) for d in detections] for t in self.tracks])
        matched_t, matched_d = [], []
        unmatched_t = list(range(len(self.tracks)))
        unmatched_d = list(range(len(detections)))
        while dist_matrix.min() < 3.0:
            ti, di = np.unravel_index(dist_matrix.argmin(), dist_matrix.shape)
            matched_t.append(ti)
            matched_d.append(di)
            if ti in unmatched_t:
                unmatched_t.remove(ti)
            if di in unmatched_d:
                unmatched_d.remove(di)
            dist_matrix[ti, :] = 1e9
            dist_matrix[:, di] = 1e9
        return list(zip(matched_t, matched_d)), unmatched_t, unmatched_d

    def obs_callback(self, msg: ObstacleArray):
        for t in self.tracks:
            t.predict()
        detections = msg.obstacles
        matches, unmatched_tracks, unmatched_dets = self._associate(detections)
        for ti, di in matches:
            self.tracks[ti].update(detections[di])
        for di in unmatched_dets:
            self.tracks.append(KalmanTrack(detections[di]))
        self.tracks = [t for t in self.tracks if t.time_since_update <= self.max_age]
        arr = ObstacleArray()
        arr.header = msg.header
        for t in self.tracks:
            if t.hits >= self.min_hits or t.time_since_update == 0:
                arr.obstacles.append(t.to_obstacle())
        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
