#!/usr/bin/env python3
"""Tkinter-based Mission Control GUI node.

Provides a small window with buttons to start or stop a mission and to
trigger or reset the emergency stop. ROS 2 spinning happens in a
background thread so the GUI remains responsive.
"""

import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class MissionGuiNode(Node):
    def __init__(self):
        super().__init__('mission_gui_node')

        self.mission_pub = self.create_publisher(Bool, '/mission/start', 1)
        self.emergency_pub = self.create_publisher(Bool, '/emergency_stop', 1)

    def start_mission(self):
        self.mission_pub.publish(Bool(data=True))
        self.get_logger().info('Mission start requested via GUI')

    def stop_mission(self):
        self.mission_pub.publish(Bool(data=False))
        self.get_logger().info('Mission stop requested via GUI')

    def trigger_estop(self):
        self.emergency_pub.publish(Bool(data=True))
        self.get_logger().warn('Emergency stop triggered via GUI')

    def reset_estop(self):
        self.emergency_pub.publish(Bool(data=False))
        self.get_logger().info('Emergency stop cleared via GUI')


def main(args=None):
    rclpy.init(args=args)
    node = MissionGuiNode()

    # Spin node in background so Tkinter remains responsive
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    root.title('Mission Control')

    tk.Button(root, text='Start Mission', command=node.start_mission).pack(fill='x')
    tk.Button(root, text='Stop Mission', command=node.stop_mission).pack(fill='x')
    tk.Button(root, text='Emergency Stop', command=node.trigger_estop).pack(fill='x')
    tk.Button(root, text='Reset E-Stop', command=node.reset_estop).pack(fill='x')

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
