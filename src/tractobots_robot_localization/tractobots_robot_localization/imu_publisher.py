#!/usr/bin/env python3
"""
Rewrite of the ROS 1 IMU republisher:
 • Subscribes to /tfsensors/imu1  (frame & orientation may be odd)
 • Publishes corrected IMU on /imu
 • Publishes an RViz Marker (optional visual arrow)
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class ImuRepublisher(Node):
    def __init__(self):
        super().__init__("imu_publisher")
        qos = QoSProfile(depth=50)

        self.sub = self.create_subscription(
            Imu, "/tfsensors/imu1", self.cb, qos)

        self.pub_imu = self.create_publisher(Imu, "/imu", qos)
        self.pub_marker = self.create_publisher(Marker, "/imu/marker", qos)

        self.frame_id_out = "imu_link"

    # ──────────────────────────────────
    def cb(self, msg_in: Imu):
        # --- clone & fix header ---------------------------------
        msg_out = Imu()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.header.frame_id = self.frame_id_out

        # --- orientation fix: add 180 deg yaw (same logic as ROS 1 node)
        q_in = (
            msg_in.orientation.x,
            msg_in.orientation.y,
            msg_in.orientation.z,
            msg_in.orientation.w,
        )
        r, p, y = euler_from_quaternion(q_in)
        q_out = quaternion_from_euler(r, p, y + math.pi)

        msg_out.orientation.x, msg_out.orientation.y, \
            msg_out.orientation.z, msg_out.orientation.w = q_out
        msg_out.angular_velocity = msg_in.angular_velocity
        msg_out.linear_acceleration = msg_in.linear_acceleration

        self.pub_imu.publish(msg_out)

        # --- marker (blue arrow) --------------------------------
        marker = Marker()
        marker.header = msg_out.header
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x, marker.scale.y, marker.scale.z = (1.0, 0.1, 0.1)
        marker.color.a, marker.color.r, marker.color.g, marker.color.b = (1.0, 0, 0, 1)
        marker.pose.orientation = msg_out.orientation
        self.pub_marker.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ImuRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
