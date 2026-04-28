#!/usr/bin/env python3
"""Object Tracker Node — SORT-style multi-object tracker.

Implements:
  - Per-object Kalman filters (state: [px, py, pz, vx, vy, vz])
  - Hungarian algorithm (scipy.optimize.linear_sum_assignment) for
    optimal detection-to-track assignment by 3D Euclidean distance
"""
import numpy as np
from scipy.optimize import linear_sum_assignment
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from agv_msgs.msg import Obstacle, ObstacleArray


# ---------------------------------------------------------------------------
# Single-target Kalman track
# ---------------------------------------------------------------------------

class KalmanTrack:
    """Constant-velocity 3D Kalman filter for one obstacle track."""
    _id_counter = 0

    def __init__(self, obs: Obstacle, dt: float = 0.1):
        KalmanTrack._id_counter += 1
        self.id = KalmanTrack._id_counter
        self.hits = 1
        self.age = 0
        self.time_since_update = 0
        self.type = obs.type

        # State: [px, py, pz, vx, vy, vz]
        self.x = np.array([
            obs.position.x, obs.position.y, obs.position.z,
            obs.velocity.x, obs.velocity.y, obs.velocity.z,
        ], dtype=float)

        # Constant velocity transition
        self.F = np.eye(6)
        self.F[0, 3] = self.F[1, 4] = self.F[2, 5] = dt

        # Observation: px, py, pz only
        self.H = np.zeros((3, 6))
        self.H[0, 0] = self.H[1, 1] = self.H[2, 2] = 1.0

        # Noise covariances
        q_pos, q_vel = 0.01, 0.1
        self.Q = np.diag([q_pos, q_pos, q_pos, q_vel, q_vel, q_vel])
        self.R = np.diag([0.25, 0.25, 0.25])
        self.P = np.eye(6) * 10.0

        # Cache obstacle size
        self.size = np.array([obs.size.x, obs.size.y, obs.size.z])

    # ---- Kalman predict ----
    def predict(self) -> None:
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.age += 1
        self.time_since_update += 1

    # ---- Kalman update ----
    def update(self, obs: Obstacle) -> None:
        z = np.array([obs.position.x, obs.position.y, obs.position.z])
        innov = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innov
        self.P = (np.eye(6) - K @ self.H) @ self.P
        self.hits += 1
        self.time_since_update = 0
        self.type = obs.type
        self.size = np.array([obs.size.x, obs.size.y, obs.size.z])

    # ---- Serialise to ROS message ----
    def to_obstacle(self) -> Obstacle:
        obs = Obstacle()
        obs.id = self.id
        obs.position = Point(x=float(self.x[0]),
                             y=float(self.x[1]),
                             z=float(self.x[2]))
        obs.velocity = Vector3(x=float(self.x[3]),
                               y=float(self.x[4]),
                               z=float(self.x[5]))
        obs.size = Vector3(x=float(self.size[0]),
                           y=float(self.size[1]),
                           z=float(self.size[2]))
        obs.type = self.type
        obs.confidence = float(min(1.0, self.hits / 10.0))
        return obs

    # ---- Predicted 3D position ----
    @property
    def pos(self) -> np.ndarray:
        return self.x[:3]


# ---------------------------------------------------------------------------
# Hungarian assignment
# ---------------------------------------------------------------------------

def _build_cost_matrix(tracks: list, dets: list) -> np.ndarray:
    """Return T×D matrix of 3D Euclidean distances."""
    T, D = len(tracks), len(dets)
    cost = np.full((T, D), fill_value=1e9)
    for i, track in enumerate(tracks):
        for j, det in enumerate(dets):
            dp = np.array([det.position.x, det.position.y, det.position.z])
            cost[i, j] = np.linalg.norm(track.pos - dp)
    return cost


def _hungarian_assign(tracks: list, dets: list,
                      max_dist: float = 3.0):
    """Optimal assignment using the Hungarian algorithm.

    Returns:
        matches:        list of (track_idx, det_idx)
        unmatched_trk:  list of track indices without a detection
        unmatched_det:  list of detection indices without a track
    """
    if not tracks or not dets:
        return [], list(range(len(tracks))), list(range(len(dets)))

    cost = _build_cost_matrix(tracks, dets)
    row_ind, col_ind = linear_sum_assignment(cost)

    matches = []
    unmatched_trk = list(range(len(tracks)))
    unmatched_det = list(range(len(dets)))

    for ti, di in zip(row_ind, col_ind):
        if cost[ti, di] > max_dist:
            continue   # distance too large — treat as unmatched
        matches.append((ti, di))
        if ti in unmatched_trk:
            unmatched_trk.remove(ti)
        if di in unmatched_det:
            unmatched_det.remove(di)

    return matches, unmatched_trk, unmatched_det


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------

class ObjectTrackerNode(Node):
    def __init__(self):
        super().__init__('object_tracker_node')
        self.declare_parameter('max_age', 5)
        self.declare_parameter('min_hits', 3)
        self.declare_parameter('max_association_dist', 3.0)
        self.declare_parameter('dt', 0.1)

        self.max_age = self.get_parameter('max_age').value
        self.min_hits = self.get_parameter('min_hits').value
        self.max_dist = self.get_parameter('max_association_dist').value
        self.dt = self.get_parameter('dt').value

        self.tracks: list[KalmanTrack] = []

        self.sub = self.create_subscription(
            ObstacleArray, '/agv/obstacles', self._obs_cb, 10)
        self.pub = self.create_publisher(
            ObstacleArray, '/agv/tracked_obstacles', 10)
        self.get_logger().info(
            'SORT Object Tracker initialized '
            '(Kalman filter + Hungarian assignment via scipy)')

    def _obs_cb(self, msg: ObstacleArray):
        # 1. Kalman predict all tracks
        for t in self.tracks:
            t.predict()

        # 2. Hungarian assignment
        dets = list(msg.obstacles)
        matches, unmatched_trk, unmatched_det = _hungarian_assign(
            self.tracks, dets, self.max_dist)

        # 3. Update matched tracks
        for ti, di in matches:
            self.tracks[ti].update(dets[di])

        # 4. Spawn new tracks for unmatched detections
        for di in unmatched_det:
            self.tracks.append(KalmanTrack(dets[di], self.dt))

        # 5. Prune dead tracks
        self.tracks = [
            t for t in self.tracks
            if t.time_since_update <= self.max_age
        ]

        # 6. Publish confirmed tracks
        out = ObstacleArray()
        out.header = msg.header
        for t in self.tracks:
            if t.hits >= self.min_hits or t.time_since_update == 0:
                out.obstacles.append(t.to_obstacle())
        self.pub.publish(out)

        self.get_logger().debug(
            f'Tracks: {len(self.tracks)}, dets: {len(dets)}, '
            f'matches: {len(matches)}, published: {len(out.obstacles)}')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
