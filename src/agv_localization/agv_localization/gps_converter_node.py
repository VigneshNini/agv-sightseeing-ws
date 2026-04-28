#!/usr/bin/env python3
"""GPS Converter Node - converts WGS84 GPS fix to ENU frame Odometry."""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry


class GPSConverterNode(Node):
    def __init__(self):
        super().__init__('gps_converter_node')
        self.declare_parameter('datum_latitude', 13.0827)
        self.declare_parameter('datum_longitude', 80.2707)
        self.declare_parameter('datum_altitude', 0.0)
        self.declare_parameter('topic_in', '/fix')
        self.declare_parameter('topic_out', '/agv/gps_enu')

        self.datum_lat = math.radians(self.get_parameter('datum_latitude').value)
        self.datum_lon = math.radians(self.get_parameter('datum_longitude').value)
        self.datum_alt = self.get_parameter('datum_altitude').value

        self.datum_ecef = self._lla_to_ecef(self.datum_lat, self.datum_lon, self.datum_alt)

        topic_in = self.get_parameter('topic_in').value
        topic_out = self.get_parameter('topic_out').value

        self.pub = self.create_publisher(Odometry, topic_out, 10)
        self.sub = self.create_subscription(NavSatFix, topic_in, self.fix_callback, 10)
        self.get_logger().info(f'GPS Converter ready. Datum: {self.datum_lat:.4f}, {self.datum_lon:.4f}')

    def _lla_to_ecef(self, lat_rad, lon_rad, alt):
        a = 6378137.0
        e2 = 0.00669437999014
        N = a / math.sqrt(1 - e2 * math.sin(lat_rad)**2)
        x = (N + alt) * math.cos(lat_rad) * math.cos(lon_rad)
        y = (N + alt) * math.cos(lat_rad) * math.sin(lon_rad)
        z = (N * (1 - e2) + alt) * math.sin(lat_rad)
        return x, y, z

    def _ecef_to_enu(self, ecef, datum_ecef, datum_lat, datum_lon):
        dx = ecef[0] - datum_ecef[0]
        dy = ecef[1] - datum_ecef[1]
        dz = ecef[2] - datum_ecef[2]
        sla, cla = math.sin(datum_lat), math.cos(datum_lat)
        slo, clo = math.sin(datum_lon), math.cos(datum_lon)
        e = -slo * dx + clo * dy
        n = -sla * clo * dx - sla * slo * dy + cla * dz
        u = cla * clo * dx + cla * slo * dy + sla * dz
        return e, n, u

    def fix_callback(self, msg: NavSatFix):
        if msg.status.status < 0:
            return
        lat = math.radians(msg.latitude)
        lon = math.radians(msg.longitude)
        alt = msg.altitude
        ecef = self._lla_to_ecef(lat, lon, alt)
        e, n, u = self._ecef_to_enu(ecef, self.datum_ecef, self.datum_lat, self.datum_lon)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'gps'
        odom.pose.pose.position.x = e
        odom.pose.pose.position.y = n
        odom.pose.pose.position.z = u
        odom.pose.pose.orientation.w = 1.0
        cov = msg.position_covariance
        odom.pose.covariance[0] = cov[0] if cov[0] > 0 else 1.0
        odom.pose.covariance[7] = cov[4] if cov[4] > 0 else 1.0
        odom.pose.covariance[14] = cov[8] if cov[8] > 0 else 4.0
        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = GPSConverterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
