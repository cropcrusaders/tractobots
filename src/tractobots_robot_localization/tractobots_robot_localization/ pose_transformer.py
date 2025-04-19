#!/usr/bin/env python3
"""
Periodically publish the origin of `from_frame` expressed in `to_frame`.

Parameters:
  • from_frame (default 'base_link')
  • to_frame   (default 'utm')
"""
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_geometry_msgs import do_transform_pose


class PoseTransformer(Node):
    def __init__(self):
        super().__init__("pose_transformer")

        self.declare_parameter("from_frame", "base_link")
        self.declare_parameter("to_frame", "utm")
        self.from_frame = self.get_parameter("from_frame").value
        self.to_frame = self.get_parameter("to_frame").value

        self.pub = self.create_publisher(PoseStamped, "transformed_pose", 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # publish at 10 Hz
        self.timer = self.create_timer(0.1, self.handle)

    # ──────────────────────────────────
    def handle(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.to_frame, self.from_frame, rclpy.time.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        pose_in = PoseStamped()
        pose_in.header.frame_id = self.from_frame
        pose_in.header.stamp = self.get_clock().now().to_msg()
        pose_in.pose.orientation.w = 1.0  # identity

        pose_out = do_transform_pose(pose_in, trans)
        pose_out.header.frame_id = self.to_frame
        self.pub.publish(pose_out)


def main(args=None):
    rclpy.init(args=args)
    node = PoseTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
