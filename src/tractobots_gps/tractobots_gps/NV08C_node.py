#!/usr/bin/env python3
"""
gps_nmea_node.py  –  ROS 2 node that reads an NMEA stream from a serial
port (e.g. NV08C receiver) and publishes a marshalled Python dict on
`gps/dict`, exactly like the old ROS 1 node.

Dependencies (debian packages or pip):
  • pyserial
  • pynmea2
  • nvector
"""

import math
import marshal
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import serial
import pynmea2
import nvector
import numpy as np
from std_msgs.msg import String


class GpsNmeaNode(Node):

    def __init__(self):
        super().__init__("gps_nmea_node")

        # ── parameters ────────────────────────────────────────────────
        self.declare_parameter("port", "/dev/gps_nmea")
        self.declare_parameter("baudrate", 115200)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baudrate").get_parameter_value().integer_value

        # ── serial port and NMEA parser ───────────────────────────────
        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=1.0)
            self.get_logger().info(f"Opened serial port {port} @ {baud}")
        except serial.SerialException as e:
            self.get_logger().fatal(f"Unable to open {port}: {e}")
            raise SystemExit(1)

        self.reader = pynmea2.NMEAStreamReader(self.ser)

        # ── publishers ────────────────────────────────────────────────
        self.pub = self.create_publisher(
            String, "gps/dict", QoSProfile(depth=10)
        )

        # ── working state ─────────────────────────────────────────────
        self.reset_sentence_state()
        self.frame = nvector.FrameE(name="WGS84")
        self.prev_update_time = time.time()

        # ── run loop timer @ 10 Hz ────────────────────────────────────
        self.create_timer(0.1, self.poll_serial)

    # ─────────────────────────────────────────────────────────────
    def reset_sentence_state(self):
        self.timestamp = None
        self.latitude = None
        self.longitude = None
        self.heading = None
        self.mode = None
        self.cog = None
        self.sog = None
        self.correction_age = None

    # ─────────────────────────────────────────────────────────────
    def poll_serial(self):
        """Read as many complete NMEA sentences as are available, then
        publish an update if we have a full set for the current epoch."""
        while True:
            try:
                sentence = self.reader.next()
            except serial.SerialException as e:
                self.get_logger().error(f"Serial error: {e}")
                return
            except pynmea2.nmea.ChecksumError:
                self.get_logger().warn("Bad NMEA checksum")
                continue
            except StopIteration:
                # no more data in buffer
                break

            if not sentence:
                break
            if len(sentence) != 1:
                continue

            self.handle_sentence(sentence[0])

    # ─────────────────────────────────────────────────────────────
    def handle_sentence(self, msg):
        # GPRMC – lat/long, SOG/COG, mode
        if isinstance(msg, pynmea2.types.talker.RMC):
            if not msg.is_valid:
                return

            if msg.mode and msg.mode not in ("R", "F", "D"):
                # Reject if no RTK/float/fixed
                return

            ts = msg.timestamp
            if ts != self.timestamp:
                # new epoch – reset partial state
                self.reset_sentence_state()
                self.timestamp = ts

            self.mode = msg.mode
            self.latitude = msg.latitude
            self.longitude = msg.longitude
            self.sog = float(msg.spd_over_grnd) * 1.852  # knots → km/h
            self.cog = float(msg.true_course or 0.0)

        # $PNVGBLS – heading + mode
        elif isinstance(msg, pynmea2.nmea.ProprietarySentence) and msg.data[0] == "BLS":
            if len(msg.data) < 9:
                return
            mode = msg.data[8]
            if mode not in ("R", "F"):
                return
            self.mode = mode
            self.heading = float(msg.data[6])

        # $PNVGBSS – correction age
        elif isinstance(msg, pynmea2.nmea.ProprietarySentence) and msg.data[0] == "BSS":
            if len(msg.data) >= 6 and msg.data[5]:
                try:
                    self.correction_age = float(msg.data[5])
                except ValueError:
                    pass

        # VTG – course/speed
        elif isinstance(msg, pynmea2.types.talker.VTG):
            try:
                self.cog = float(msg.true_track or 0.0)
                self.sog = float(msg.spd_over_grnd_kmph or 0.0)
            except ValueError:
                pass

        # HDT – heading
        elif isinstance(msg, pynmea2.types.talker.HDT):
            try:
                self.heading = float(msg.heading)
            except ValueError:
                pass

        # After each sentence, see if we have a full data set
        if (
            self.latitude is not None
            and self.longitude is not None
            and self.heading is not None
            and self.correction_age is not None
        ):
            now = time.time()
            period = now - self.prev_update_time
            self.prev_update_time = now

            update = {
                "latitude": self.latitude,
                "longitude": self.longitude,
                "heading": (self.heading + 180.0) % 360,
                "course_over_ground": self.cog,
                "speed_over_ground": self.sog,
                "mode_indicator": self.mode,
                "timestamp": str(self.timestamp),
                "period": period,
                "correction_age": self.correction_age,
            }

            self.pub.publish(String(data=marshal.dumps(update)))
            self.reset_sentence_state()

    # ─────────────────────────────────────────────────────────────
    def destroy_node(self):
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


# ──────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = GpsNmeaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
