#!/usr/bin/env python3
"""
Convert the marshalled GPS dictionary published by nv08c_node
into standard ROS 2 messages (NavSatFix, TimeReference, TwistStamped, Marker).
"""

import math
import marshal
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from geometry_msgs.msg import TwistStamped
from visualization_msgs.msg import Marker

# tf in ROS 2 → use the standalone tf_transformations package
from tf_transformations import quaternion_from_euler


class GpsParser(Node):
    def __init__(self):
        super().__init__("gps_translator")

        qos = QoSProfile(depth=10)

        # Subscriber to raw GPS dictionary
        self.sub = self.create_subscription(
            String, "gps/dict", self.gps_callback, qos
        )

        # Publishers
        self.navsat_pub = self.create_publisher(NavSatFix, "gps/fix", qos)
        self.timeref_pub = self.create_publisher(TimeReference, "gps/timeref", qos)
        self.vel_pub = self.create_publisher(TwistStamped, "gps/vel", qos)
        self.marker_pub = self.create_publisher(Marker, "gps/marker", qos)

        self.frame_id = "gps_link"

    # ────────────────────────────────────────────────────────────────
    # Helper: translate GPS quality integer → NavSatStatus
    # ────────────────────────────────────────────────────────────────
    @staticmethod
    def _quality_to_status(qual: int) -> int:
        if qual == 1:
            return NavSatStatus.STATUS_FIX
        if qual == 2:
            return NavSatStatus.STATUS_SBAS_FIX
        if qual in (4, 5):
            return NavSatStatus.STATUS_GBAS_FIX
        if qual == 9:
            return NavSatStatus.STATUS_SBAS_FIX
        return NavSatStatus.STATUS_NO_FIX

    # ────────────────────────────────────────────────────────────────
    # Main callback
    # ────────────────────────────────────────────────────────────────
    def gps_callback(self, msg: String):
        # Unmarshal the dictionary produced by nv08c_node
        try:
            gps = marshal.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Could not unmarshal GPS data: {e}")
            return

        now_ros = self.get_clock().now().to_msg()

        # 1. TimeReference
        time_ref = TimeReference()
        time_ref.header.stamp = now_ros
        time_ref.header.frame_id = self.frame_id
        time_ref.source = self.frame_id
        # (Optional) If you later parse true GPS time, assign to time_ref.time_ref
        self.timeref_pub.publish(time_ref)

        # 2. NavSatFix
        fix = NavSatFix()
        fix.header.stamp = now_ros
        fix.header.frame_id = self.frame_id
        fix.status.status = self._quality_to_status(gps.get("qual", 0))
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = gps.get("latitude", float("nan"))
        fix.longitude = gps.get("longitude", float("nan"))
        # Some EKF versions choke on NaN altitude, so 0 is used here
        fix.altitude = 0.0
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.navsat_pub.publish(fix)

        # 3. TwistStamped (ground‑speed in X/Y : ENU)
        twist = TwistStamped()
        twist.header.stamp = now_ros
        twist.header.frame_id = self.frame_id
        sog = gps.get("speed_over_ground", 0.0)
        cog = gps.get("course_over_ground", 0.0)  # assume radians
        twist.twist.linear.x = sog * math.sin(cog)
        twist.twist.linear.y = sog * math.cos(cog)
        self.vel_pub.publish(twist)

        # 4. RViz marker (arrow facing course‑over‑ground)
        marker = Marker()
        marker.header = fix.header
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 1.0
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        quat = quaternion_from_euler(0.0, 0.0, cog)
        marker.pose.orientation.x, marker.pose.orientation.y, \
            marker.pose.orientation.z, marker.pose.orientation.w = quat
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = GpsParser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Ctrl‑C
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
