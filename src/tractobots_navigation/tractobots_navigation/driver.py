#!/usr/bin/env python3
"""
Tractobots driver / tele‑op node (ROS 2 Humble).

Ported from the original ROS 1 script:
 • Listens to /joy (sensor_msgs/Joy) for manual and mode‑switch commands
 • Listens to /gps/dict (marshalled dict) for navigator feedback
 • Publishes raw actuator commands on /steering/put, /transmission/…, /throttle/…
 • Can spawn Line_Follower or Compass_Follower helper objects
"""

# Std lib
import math
import marshal
import signal
import sys
import threading
import time

# Third‑party
import PID                # unchanged dependency
import nvector            # unchanged dependency

# ROS 2
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile

from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8, Bool, String, Float64

# Constants
MAX_STAMP_AGE = Duration(seconds=2)
NORTH = 0.0               # magnetic correction handled elsewhere


# ──────────────────────────────────────────────────────────────────────
#  Utility helpers (unchanged from ROS 1)
# ──────────────────────────────────────────────────────────────────────
def minmax(_min, _max, val):
    return max(_min, min(_max, val))


def reverse_course(course):
    return (course + 180.0) % 360


class Scaler:
    """Linear mapping between two ranges."""

    def __init__(self, a_min, a_max, b_min, b_max):
        self.a_min = a_min
        self.a_span = a_max - a_min
        self.b_min = b_min
        self.b_span = b_max - b_min

    def __call__(self, x):
        """Scale value x from input range to output range."""
        return (x - self.a_min) / self.a_span * self.b_span + self.b_min


# ──────────────────────────────────────────────────────────────────────
#  Lightweight wrapper that publishes UInt8 actuator commands
# ──────────────────────────────────────────────────────────────────────
class SettingPublisher:
    def __init__(self, node: Node, topic: str, scaler: Scaler):
        self._pub = node.create_publisher(UInt8, topic, 1)
        self._scale = scaler

    def __call__(self, value: float):
        msg = UInt8(data=int(self._scale(float(value))))
        self._pub.publish(msg)


# ──────────────────────────────────────────────────────────────────────
#  Navigator helpers (largely unchanged, only ROS deps removed)
# ──────────────────────────────────────────────────────────────────────
class LineFollower:
    name = "line"

    def __init__(self, name=None):
        if name:
            self.name = name

        self.captured = 0
        self.captured_distance = 0.50
        self.offset = 0.0
        self.throttle = 0.50

        # PID tuning for cart
        self.pid_captured = PID.PID(
            P=100.0, I=70.0, D=50, Derivator=0,
            Integrator=0, Integrator_max=50, Integrator_min=-50
        )
        self.pid_seeking = PID.PID(
            P=10.0, I=0.0, D=4.0, Derivator=0,
            Integrator=0, Integrator_max=1, Integrator_min=-1
        )

        self.frame = nvector.FrameE(name="WGS84")

    # --- path helpers --------------------------------------------------
    def load_line(self, pointA, pointB):
        self.path = nvector.GeoPath(pointA, pointB)
        p_AB_E = nvector.diff_positions(pointA, pointB)
        frame_N = nvector.FrameN(pointA)
        p_AB_N = p_AB_E.change_frame(frame_N).pvector.ravel()
        azimuth = math.atan2(p_AB_N[1], p_AB_N[0])
        self.course = math.degrees(azimuth) % 360
        self.offset = 0.0

    def load_vector(self, pointA, azimuth_deg):
        pointB, _ = pointA.geo_point(distance=1_000, azimuth=azimuth_deg, degrees=True)
        self.load_line(pointA, pointB)

    # --- main call -----------------------------------------------------
    def __call__(self, gps):
        position = self.frame.GeoPoint(
            latitude=gps["latitude"], longitude=gps["longitude"], degrees=True
        )

        xtd = (
            self.path.cross_track_distance(position, method="greatcircle").ravel()
            - self.offset
        )

        speed = gps["speed_over_ground"]
        heading = gps["heading"]

        # PID
        pid_feed = xtd
        if abs(xtd) <= self.captured_distance:
            self.captured += 1
        else:
            self.captured = 0

        if self.captured > 15:
            pid_val = self.pid_captured(pid_feed)
        else:
            pid_val = self.pid_seeking(pid_feed)

        desired_heading = (self.course + minmax(-90, 90, pid_val)) % 360
        bearing = desired_heading - heading
        if bearing < -180:
            bearing += 360
        elif bearing > 180:
            bearing -= 360

        return {
            "bearing": bearing,
            "transmission": 1,
            "throttle": self.throttle,
        }


class CompassFollower:
    name = "compass"

    def __init__(self, course):
        self.course = course

    def __call__(self, gps):
        bearing = self.course - gps["heading"]
        if bearing < -180:
            bearing += 360
        elif bearing > 180:
            bearing -= 360
        return {
            "bearing": bearing,
            "transmission": 1,
        }


# ──────────────────────────────────────────────────────────────────────
#  Main driver node
# ──────────────────────────────────────────────────────────────────────
class TractobotDriver(Node):
    def __init__(self):
        super().__init__("tractobot_driver")

        self.frame = nvector.FrameE(name="WGS84")
        self.gps_reading = None
        self.joy_prev = None

        # Publishers (UInt8 scaled outputs)
        self.steering = SettingPublisher(
            self, "/steering/put", Scaler(+1, -1, 255, 75)
        )
        self.transmission = SettingPublisher(
            self, "/transmission/shuttle/put", Scaler(-1, +1, 10, 250)
        )
        self.throttle = SettingPublisher(
            self, "/throttle/put", Scaler(0, 1, 20, 135)
        )

        self.ignition_pub = self.create_publisher(Bool, "/ignition/put", 1)
        self.hitch_pub = self.create_publisher(UInt8, "/hitch/put", 1)
        self.passthrough_pub = self.create_publisher(Bool, "/passthrough/put", 1)

        # Subscribers
        self.create_subscription(Joy, "/joy", self.cb_joy, 10)
        self.create_subscription(String, "/gps/dict", self.cb_gps, 10)
        self.create_subscription(Float64, "/navigation/steering_pid/command",
                                  self.cb_nav_steering, 10)

        # PID to turn wheel toward bearing
        self.steer_for_bearing = PID.PID(
            P=0.02, I=0.02, D=0.02,
            Derivator=0, Integrator=0,
            Integrator_max=2, Integrator_min=-2,
            output_min=-0.6, output_max=0.6,
        )

        # State
        self._enabled = False
        self.navigator = None

        # Watchdog timer: if no joystick in N sec → neutral
        self.timeout_neutral = 2.0
        self.timeout_kill = 10.0
        self.last_cmd_time = self.get_clock().now()

        # Create a periodic watchdog check
        self.create_timer(0.5, self._watchdog_cb)

        self.get_logger().info("Tractobot driver ready")

    # ────────── utility ------------------------------------------------
    def _watchdog_cb(self):
        now = self.get_clock().now()
        if (now - self.last_cmd_time) > Duration(seconds=self.timeout_neutral):
            self._neutral()
        # kill switch
        if (now - self.last_cmd_time) > Duration(seconds=self.timeout_kill):
            self._enabled = False
            self.navigator = None
            self._neutral()

    def _neutral(self):
        self.transmission(0)
        self.steering(0)
        self.throttle(0)

    def _publish_ignition(self, on: bool):
        self.ignition_pub.publish(Bool(data=on))

    # ────────── joystick callback -------------------------------------
    def cb_joy(self, joy: Joy):
        self.last_cmd_time = self.get_clock().now()

        # Manual vs nav enable logic follows original script
        axes = joy.axes
        buttons = joy.buttons

        left_x = axes[0]
        left_y = axes[1]
        left_trigger = axes[2]
        left_trigger_btn = buttons[4]
        nav_set = buttons[5]
        back = buttons[6]
        start = buttons[7]
        left_stick_btn = buttons[9]

        # Kill
        if back:
            self._enabled = False
            self.navigator = None
            self._neutral()
            self._publish_ignition(False)
            return

        # Start engine / exit passthrough
        if start:
            self._enabled = True
            self.passthrough_pub.publish(Bool(data=False))
            self._publish_ignition(True)

        # Change hitch with right‑stick Y
        right_y = axes[4]
        if right_y < -0.5:
            self.hitch_pub.publish(UInt8(data=1))  # down
        elif right_y > 0.5:
            self.hitch_pub.publish(UInt8(data=2))  # up
        else:
            self.hitch_pub.publish(UInt8(data=0))

        if not self._enabled:
            return

        # Throttle and transmission positions
        throttle_pos = (1 - left_trigger) / 2.0
        trans_pos = (
            -1.0 if (left_trigger_btn and left_stick_btn) else
            +1.0 if left_trigger_btn else
            0.0
        )

        # If nav_set not held OR manual input moved → drop navigator
        if not nav_set or left_x or trans_pos or throttle_pos:
            if self.navigator:
                self.navigator = None

        # nav_set pressed → possibly create navigator
        elif nav_set:
            if buttons[3]:     # Y button → north line
                self.navigator = LineFollower()
                self.navigator.load_vector(self._current_position(), NORTH)
            elif buttons[1]:   # B → east
                self.navigator = LineFollower()
                self.navigator.load_vector(self._current_position(), (NORTH + 90) % 360)
            elif buttons[0]:   # A → south
                self.navigator = LineFollower()
                self.navigator.load_vector(self._current_position(), (NORTH + 180) % 360)
            elif buttons[2]:   # X → west
                self.navigator = LineFollower()
                self.navigator.load_vector(self._current_position(), (NORTH + 270) % 360)

        # If navigator active and D‑pad used → adjust offset / throttle
        if self.navigator and self.navigator.name == "line" and self.joy_prev:
            dpad_x = axes[6]
            dpad_y = axes[7]
            if self.joy_prev.axes[6] == 0 and dpad_x:
                self.navigator.offset += dpad_x * -0.2
            if self.joy_prev.axes[7] == 0 and dpad_y:
                self.navigator.throttle = minmax(0.0, 1.0,
                                                 self.navigator.throttle + 0.05 * dpad_y)

        self.joy_prev = joy

        # Manual or navigator control
        if self.navigator is None:
            # Manual
            self.transmission(trans_pos)
            self.steering(left_x)
            self.throttle(throttle_pos)
        else:
            # Let cb_gps handle steering/throttle each GPS update
            pass

    # ────────── GPS dict callback -------------------------------------
    def cb_gps(self, msg: String):
        self.gps_reading = marshal.loads(msg.data)
        if not self._enabled or self.navigator is None:
            return

        # Navigator outputs desired bearing etc.
        nav = self.navigator(self.gps_reading)
        if not nav:
            return

        # Steering PID for bearing
        bearing = nav["bearing"]
        steering_cmd = self.steer_for_bearing(bearing)
        self.steering(steering_cmd)
        self.transmission(nav.get("transmission", 0))
        self.throttle(nav.get("throttle", 0))

    # ────────── External steering PID command -------------------------
    def cb_nav_steering(self, steering: Float64):
        self.steering(steering.data)
        self.transmission(1)

    # ────────── helpers ------------------------------------------------
    def _current_position(self):
        if not self.gps_reading:
            return None
        return self.frame.GeoPoint(
            latitude=self.gps_reading["latitude"],
            longitude=self.gps_reading["longitude"],
            degrees=True,
        )


# ──────────────────────────────────────────────────────────────────────
#  Entry point
# ──────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = TractobotDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
