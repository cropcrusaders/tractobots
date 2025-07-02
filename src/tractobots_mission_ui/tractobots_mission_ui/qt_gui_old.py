#!/usr/bin/env python3
"""
Professional Qt-based Tractobots GUI
Advanced interface with real-time plotting, 3D visualization, and comprehensive controls
"""

import sys
import threading
import numpy as np
from datetime import datetime
import math

try:
    from PyQt5.QtWidgets import *  # type: ignore
    from PyQt5.QtCore import *  # type: ignore
    from PyQt5.QtGui import *  # type: ignore
    import pyqtgraph as pg  # type: ignore
    QT_AVAILABLE = True
except ImportError:
    QT_AVAILABLE = False
    print("PyQt5 not available. Install with: pip install PyQt5 pyqtgraph")
    # Exit early if PyQt is not available to avoid undefined variable errors
    import sys
    if __name__ == "__main__":
        print("Please install PyQt5 to use the Qt GUI:")
        print("pip install PyQt5 pyqtgraph")
        sys.exit(1)

import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import Bool, String  # type: ignore
from geometry_msgs.msg import Twist  # type: ignore
from sensor_msgs.msg import NavSatFix, Imu  # type: ignore
from nav_msgs.msg import Odometry  # type: ignore

# Only define classes if PyQt is available
if QT_AVAILABLE:
    class TractorControlNode(Node):
    def __init__(self):
        super().__init__('tractor_qt_gui')
        
        # Publishers
        self.mission_pub = self.create_publisher(Bool, '/mission/start', 1)
        self.emergency_pub = self.create_publisher(Bool, '/emergency_stop', 1)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        
        # Subscribers
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 1)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        
        # Data storage
        self.gps_data = []
        self.speed_data = []
        self.heading_data = []
        self.timestamps = []
        
        self.current_gps = {"lat": 0.0, "lon": 0.0, "alt": 0.0}
        self.current_speed = 0.0
        self.current_heading = 0.0
        self.mission_active = False
        self.emergency_stop = False

    def gps_callback(self, msg):
        self.current_gps = {
            "lat": msg.latitude,
            "lon": msg.longitude,
            "alt": msg.altitude
        }
        self.gps_data.append([msg.latitude, msg.longitude])
        if len(self.gps_data) > 1000:  # Keep last 1000 points
            self.gps_data.pop(0)

    def imu_callback(self, msg):
        q = msg.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.current_heading = math.degrees(yaw)
        
        self.heading_data.append(self.current_heading)
        self.timestamps.append(datetime.now().timestamp())
        if len(self.heading_data) > 500:
            self.heading_data.pop(0)
            self.timestamps.pop(0)

    def odom_callback(self, msg):
        self.current_speed = (msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)**0.5
        self.speed_data.append(self.current_speed)
        if len(self.speed_data) > 500:
            self.speed_data.pop(0)


class TractorGUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.mission_start_time = None
        self.init_ui()
        
        # Timer for updating GUI
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_displays)
        self.timer.start(100)  # Update every 100ms

    def init_ui(self):
        self.setWindowTitle('🚜 Tractobots Professional Control System')
        self.setGeometry(100, 100, 1400, 900)
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1a1a1a;
                color: white;
            }
            QGroupBox {
                font-weight: bold;
                border: 2px solid #555;
                border-radius: 10px;
                margin-top: 10px;
                padding-top: 10px;
                background-color: rgba(255, 255, 255, 0.1);
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
                color: #00ff88;
            }
            QPushButton {
                background-color: #4CAF50;
                border: none;
                color: white;
                padding: 10px;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3d8b40;
            }
            .emergency {
                background-color: #f44336;
            }
            .emergency:hover {
                background-color: #da190b;
            }
            QLabel {
                color: white;
                font-size: 12px;
            }
            .title {
                font-size: 24px;
                font-weight: bold;
                color: #00ff88;
            }
            .status {
                font-size: 18px;
                font-weight: bold;
                padding: 10px;
                border-radius: 5px;
            }
            .status-ready {
                background-color: #4CAF50;
            }
            .status-active {
                background-color: #2196F3;
            }
            .status-emergency {
                background-color: #f44336;
            }
        """)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Main layout
        main_layout = QVBoxLayout(central_widget)

        # Title and Status
        title_layout = QHBoxLayout()
        title = QLabel('🚜 TRACTOBOTS AUTONOMOUS CONTROL SYSTEM')
        title.setObjectName('title')
        title.setAlignment(Qt.AlignCenter)
        title_layout.addWidget(title)

        self.status_label = QLabel('🟡 READY')
        self.status_label.setObjectName('status')
        self.status_label.setAlignment(Qt.AlignCenter)
        title_layout.addWidget(self.status_label)

        main_layout.addLayout(title_layout)

        # Content layout
        content_layout = QHBoxLayout()

        # Left Panel - Controls
        left_panel = self.create_control_panel()
        content_layout.addWidget(left_panel, 1)

        # Right Panel - Displays
        right_panel = self.create_display_panel()
        content_layout.addWidget(right_panel, 2)

        main_layout.addLayout(content_layout)

    def create_control_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)

        # Mission Control
        mission_group = QGroupBox('Mission Control')
        mission_layout = QVBoxLayout(mission_group)

        self.start_btn = QPushButton('🚀 START MISSION')
        self.start_btn.clicked.connect(self.start_mission)
        mission_layout.addWidget(self.start_btn)

        self.stop_btn = QPushButton('⏹️ STOP MISSION')
        self.stop_btn.clicked.connect(self.stop_mission)
        mission_layout.addWidget(self.stop_btn)

        self.pause_btn = QPushButton('⏸️ PAUSE MISSION')
        self.pause_btn.clicked.connect(self.pause_mission)
        mission_layout.addWidget(self.pause_btn)

        layout.addWidget(mission_group)

        # Emergency Controls
        emergency_group = QGroupBox('Emergency Controls')
        emergency_layout = QVBoxLayout(emergency_group)

        self.estop_btn = QPushButton('🛑 EMERGENCY STOP')
        self.estop_btn.setObjectName('emergency')
        self.estop_btn.clicked.connect(self.trigger_estop)
        emergency_layout.addWidget(self.estop_btn)

        self.reset_btn = QPushButton('🔄 RESET E-STOP')
        self.reset_btn.clicked.connect(self.reset_estop)
        emergency_layout.addWidget(self.reset_btn)

        layout.addWidget(emergency_group)

        # Manual Control
        manual_group = QGroupBox('Manual Control')
        manual_layout = QGridLayout(manual_group)

        self.forward_btn = QPushButton('⬆️ FORWARD')
        self.forward_btn.pressed.connect(lambda: self.manual_control(1.0, 0.0))
        self.forward_btn.released.connect(lambda: self.manual_control(0.0, 0.0))
        manual_layout.addWidget(self.forward_btn, 0, 1)

        self.left_btn = QPushButton('⬅️ LEFT')
        self.left_btn.pressed.connect(lambda: self.manual_control(0.0, 1.0))
        self.left_btn.released.connect(lambda: self.manual_control(0.0, 0.0))
        manual_layout.addWidget(self.left_btn, 1, 0)

        self.stop_manual_btn = QPushButton('⏹️ STOP')
        self.stop_manual_btn.setObjectName('emergency')
        self.stop_manual_btn.clicked.connect(lambda: self.manual_control(0.0, 0.0))
        manual_layout.addWidget(self.stop_manual_btn, 1, 1)

        self.right_btn = QPushButton('➡️ RIGHT')
        self.right_btn.pressed.connect(lambda: self.manual_control(0.0, -1.0))
        self.right_btn.released.connect(lambda: self.manual_control(0.0, 0.0))
        manual_layout.addWidget(self.right_btn, 1, 2)

        self.backward_btn = QPushButton('⬇️ BACKWARD')
        self.backward_btn.pressed.connect(lambda: self.manual_control(-1.0, 0.0))
        self.backward_btn.released.connect(lambda: self.manual_control(0.0, 0.0))
        manual_layout.addWidget(self.backward_btn, 2, 1)

        layout.addWidget(manual_group)

        # System Info
        info_group = QGroupBox('System Information')
        info_layout = QVBoxLayout(info_group)

        self.mission_time_label = QLabel('Mission Time: 00:00:00')
        info_layout.addWidget(self.mission_time_label)

        self.gps_label = QLabel('GPS: Waiting for data...')
        info_layout.addWidget(self.gps_label)

        self.speed_label = QLabel('Speed: 0.0 m/s')
        info_layout.addWidget(self.speed_label)

        self.heading_label = QLabel('Heading: 0.0°')
        info_layout.addWidget(self.heading_label)

        layout.addWidget(info_group)

        return panel

    def create_display_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)

        # GPS Track Plot
        gps_group = QGroupBox('GPS Track')
        gps_layout = QVBoxLayout(gps_group)
        
        self.gps_plot = pg.PlotWidget(title="GPS Trajectory")
        self.gps_plot.setLabel('left', 'Latitude')
        self.gps_plot.setLabel('bottom', 'Longitude')
        self.gps_plot.setBackground('black')
        gps_layout.addWidget(self.gps_plot)
        
        layout.addWidget(gps_group)

        # Speed and Heading Plots
        plots_group = QGroupBox('Real-time Data')
        plots_layout = QHBoxLayout(plots_group)

        self.speed_plot = pg.PlotWidget(title="Speed (m/s)")
        self.speed_plot.setLabel('left', 'Speed')
        self.speed_plot.setLabel('bottom', 'Time')
        self.speed_plot.setBackground('black')
        plots_layout.addWidget(self.speed_plot)

        self.heading_plot = pg.PlotWidget(title="Heading (degrees)")
        self.heading_plot.setLabel('left', 'Heading')
        self.heading_plot.setLabel('bottom', 'Time')
        self.heading_plot.setBackground('black')
        plots_layout.addWidget(self.heading_plot)

        layout.addWidget(plots_group)

        return panel

    def update_displays(self):
        # Update labels
        self.gps_label.setText(f'GPS: {self.node.current_gps["lat"]:.6f}, {self.node.current_gps["lon"]:.6f}')
        self.speed_label.setText(f'Speed: {self.node.current_speed:.2f} m/s')
        self.heading_label.setText(f'Heading: {self.node.current_heading:.1f}°')

        # Update mission time
        if self.mission_start_time and self.node.mission_active:
            elapsed = datetime.now().timestamp() - self.mission_start_time
            hours = int(elapsed // 3600)
            minutes = int((elapsed % 3600) // 60)
            seconds = int(elapsed % 60)
            self.mission_time_label.setText(f'Mission Time: {hours:02d}:{minutes:02d}:{seconds:02d}')

        # Update plots
        if len(self.node.gps_data) > 1:
            gps_array = np.array(self.node.gps_data)
            self.gps_plot.clear()
            self.gps_plot.plot(gps_array[:, 1], gps_array[:, 0], pen='g', symbol='o', symbolSize=3)

        if len(self.node.speed_data) > 1:
            self.speed_plot.clear()
            self.speed_plot.plot(self.node.speed_data, pen='b')

        if len(self.node.heading_data) > 1:
            self.heading_plot.clear()
            self.heading_plot.plot(self.node.heading_data, pen='r')

        # Update status
        if self.node.emergency_stop:
            self.status_label.setText('🔴 EMERGENCY STOP')
            self.status_label.setObjectName('status-emergency')
        elif self.node.mission_active:
            self.status_label.setText('🟢 MISSION ACTIVE')
            self.status_label.setObjectName('status-active')
        else:
            self.status_label.setText('🟡 READY')
            self.status_label.setObjectName('status-ready')

    def start_mission(self):
        if not self.node.emergency_stop:
            self.node.mission_pub.publish(Bool(data=True))
            self.node.mission_active = True
            self.mission_start_time = datetime.now().timestamp()

    def stop_mission(self):
        self.node.mission_pub.publish(Bool(data=False))
        self.node.mission_active = False
        self.mission_start_time = None

    def pause_mission(self):
        # Implement pause logic
        pass

    def trigger_estop(self):
        self.node.emergency_pub.publish(Bool(data=True))
        self.node.emergency_stop = True
        self.node.mission_active = False

    def reset_estop(self):
        self.node.emergency_pub.publish(Bool(data=False))
        self.node.emergency_stop = False

    def manual_control(self, linear, angular):
        if not self.node.emergency_stop:
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.node.cmd_vel_pub.publish(twist)


def main(args=None):
    if not QT_AVAILABLE:
        print("Qt GUI not available. Please install PyQt5:")
        print("pip install PyQt5 pyqtgraph")
        return

    rclpy.init(args=args)
    node = TractorControlNode()

    # Start ROS2 spinning in background
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Create Qt application
    app = QApplication(sys.argv)
    gui = TractorGUI(node)
    gui.show()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
