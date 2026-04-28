#!/usr/bin/env python3
"""UKF Node - 15-DOF Unscented Kalman Filter
State: [x, y, z, roll, pitch, yaw, vx, vy, vz, bax, bay, baz, bgx, bgy, bgz]
"""
import numpy as np
from scipy.linalg import cholesky
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf2_ros
import math


class UKFNode(Node):
    N = 15
    ALPHA = 1e-3
    BETA = 2.0
    KAPPA = 0.0

    def __init__(self):
        super().__init__('ukf_node')
        self._declare_params()
        n = self.N
        lam = self.ALPHA**2 * (n + self.KAPPA) - n
        self.lam = lam
        self.Wm = np.full(2*n+1, 0.5 / (n + lam))
        self.Wm[0] = lam / (n + lam)
        self.Wc = self.Wm.copy()
        self.Wc[0] += (1 - self.ALPHA**2 + self.BETA)
        self.x = np.zeros(n)
        self.P = np.eye(n) * 0.1
        q_pos = self.get_parameter('process_noise_position').value
        q_vel = self.get_parameter('process_noise_velocity').value
        q_ori = self.get_parameter('process_noise_orientation').value
        self.Q = np.diag([q_pos]*3 + [q_ori]*3 + [q_vel]*3 + [1e-6]*3 + [1e-7]*3)
        self.last_time = None
        self.initialized = False
        self._setup_comms()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info('UKF Node initialized')

    def _declare_params(self):
        self.declare_parameter('process_noise_position', 0.05)
        self.declare_parameter('process_noise_velocity', 0.1)
        self.declare_parameter('process_noise_orientation', 0.01)
        self.declare_parameter('measurement_noise_gps', 0.5)
        self.declare_parameter('measurement_noise_imu', 0.01)
        self.declare_parameter('frequency', 50.0)

    def _setup_comms(self):
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(Odometry, '/agv/gps_enu', self.gps_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/agv/wheel_odometry', self.wheel_odom_callback, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/agv/pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/agv/odometry', 10)

    def sigma_points(self):
        n = self.N
        try:
            L = cholesky((n + self.lam) * self.P, lower=True)
        except Exception:
            self.P = (self.P + self.P.T) / 2 + np.eye(n) * 1e-6
            L = cholesky((n + self.lam) * self.P, lower=True)
        sigmas = np.zeros((2*n+1, n))
        sigmas[0] = self.x
        for i in range(n):
            sigmas[i+1] = self.x + L[:, i]
            sigmas[i+1+n] = self.x - L[:, i]
        return sigmas

    def process_model(self, state, dt, accel_body, gyro_body):
        s = state.copy()
        roll, pitch, yaw = s[3], s[4], s[5]
        bax, bay, baz = s[9], s[10], s[11]
        bgx, bgy, bgz = s[12], s[13], s[14]
        ax = accel_body[0] - bax
        ay = accel_body[1] - bay
        az = accel_body[2] - baz
        gx = gyro_body[0] - bgx
        gy = gyro_body[1] - bgy
        gz = gyro_body[2] - bgz
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        R = np.array([
            [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
            [-sp,   cp*sr,           cp*cr]
        ])
        accel_world = R @ np.array([ax, ay, az]) + np.array([0, 0, -9.81])
        s[0] += s[6]*dt + 0.5*accel_world[0]*dt**2
        s[1] += s[7]*dt + 0.5*accel_world[1]*dt**2
        s[2] += s[8]*dt + 0.5*accel_world[2]*dt**2
        s[6] += accel_world[0]*dt
        s[7] += accel_world[1]*dt
        s[8] += accel_world[2]*dt
        cp_safe = cp if abs(cp) > 1e-6 else 1e-6
        s[3] += (gx + gy*sr*sp/cp_safe + gz*cr*sp/cp_safe)*dt
        s[4] += (gy*cr - gz*sr)*dt
        s[5] += (gy*sr/cp_safe + gz*cr/cp_safe)*dt
        s[3] = math.atan2(math.sin(s[3]), math.cos(s[3]))
        s[4] = math.atan2(math.sin(s[4]), math.cos(s[4]))
        s[5] = math.atan2(math.sin(s[5]), math.cos(s[5]))
        return s

    def predict(self, dt, accel, gyro):
        sigmas = self.sigma_points()
        sigmas_pred = np.array([self.process_model(s, dt, accel, gyro) for s in sigmas])
        self.x = np.sum(self.Wm[:, None] * sigmas_pred, axis=0)
        diff = sigmas_pred - self.x
        self.P = sum(self.Wc[i] * np.outer(diff[i], diff[i]) for i in range(2*self.N+1)) + self.Q

    def update_linear(self, z, H, R):
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(self.N) - K @ H) @ self.P

    def imu_callback(self, msg: Imu):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_time is None:
            self.last_time = now
            if not self.initialized:
                self.initialized = True
            return
        dt = now - self.last_time
        if dt <= 0 or dt > 1.0:
            self.last_time = now
            return
        accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        self.predict(dt, accel, gyro)
        self.last_time = now
        self._publish()

    def gps_callback(self, msg: Odometry):
        if not self.initialized:
            self.x[0] = msg.pose.pose.position.x
            self.x[1] = msg.pose.pose.position.y
            self.x[2] = msg.pose.pose.position.z
            self.initialized = True
            return
        noise = self.get_parameter('measurement_noise_gps').value
        H = np.zeros((3, self.N)); H[0,0]=H[1,1]=H[2,2]=1.0
        z = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        self.update_linear(z, H, np.diag([noise, noise, noise*4]))

    def wheel_odom_callback(self, msg: Odometry):
        if not self.initialized:
            return
        H = np.zeros((2, self.N)); H[0,6]=H[1,7]=1.0
        z = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y])
        self.update_linear(z, H, np.eye(2)*0.1)

    def _publish(self):
        if not self.initialized:
            return
        stamp = self.get_clock().now().to_msg()
        roll, pitch, yaw = self.x[3], self.x[4], self.x[5]
        cy2, sy2 = math.cos(yaw/2), math.sin(yaw/2)
        cp2, sp2 = math.cos(pitch/2), math.sin(pitch/2)
        cr2, sr2 = math.cos(roll/2), math.sin(roll/2)
        qw = cr2*cp2*cy2 + sr2*sp2*sy2
        qx = sr2*cp2*cy2 - cr2*sp2*sy2
        qy = cr2*sp2*cy2 + sr2*cp2*sy2
        qz = cr2*cp2*sy2 - sr2*sp2*cy2

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = self.x[0]
        pose_msg.pose.pose.position.y = self.x[1]
        pose_msg.pose.pose.position.z = self.x[2]
        pose_msg.pose.pose.orientation.w = qw
        pose_msg.pose.pose.orientation.x = qx
        pose_msg.pose.pose.orientation.y = qy
        pose_msg.pose.pose.orientation.z = qz
        flat_cov = [0.0]*36
        for i in range(6):
            for j in range(6):
                flat_cov[i*6+j] = float(self.P[i, j])
        pose_msg.pose.covariance = flat_cov
        self.pose_pub.publish(pose_msg)

        odom = Odometry()
        odom.header = pose_msg.header
        odom.child_frame_id = 'base_link'
        odom.pose = pose_msg.pose
        odom.twist.twist.linear.x = self.x[6]
        odom.twist.twist.linear.y = self.x[7]
        odom.twist.twist.linear.z = self.x[8]
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x[0]
        t.transform.translation.y = self.x[1]
        t.transform.translation.z = self.x[2]
        t.transform.rotation = pose_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = UKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
