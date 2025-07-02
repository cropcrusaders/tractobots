#!/usr/bin/env python3
"""
Professional Qt GUI for Tractobots Autonomous System
Provides real-time monitoring and control interface
"""

import sys
import os
import json
from datetime import datetime
import math

# Add proper type checking ignores for PyQt imports
try:
    from PyQt5.QtWidgets import *  # type: ignore
    from PyQt5.QtCore import *  # type: ignore  
    from PyQt5.QtGui import *  # type: ignore
    import pyqtgraph as pg  # type: ignore
    QT_AVAILABLE = True
except ImportError:
    QT_AVAILABLE = False
    print("⚠️  PyQt5 not available. Install with: pip install PyQt5 pyqtgraph")
    if __name__ == "__main__":
        print("Please install PyQt5 to use the Qt GUI:")
        print("pip install PyQt5 pyqtgraph")
        sys.exit(1)

# Add type ignore for ROS2 imports on Windows
import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import Bool, String  # type: ignore
from geometry_msgs.msg import Twist  # type: ignore
from sensor_msgs.msg import NavSatFix, Imu  # type: ignore
from nav_msgs.msg import Odometry  # type: ignore

class TractorControlNode(Node):
    """ROS2 node for tractor control and data collection"""
    
    def __init__(self):
        super().__init__('tractor_qt_gui')
        
        # Publishers
        self.mission_pub = self.create_publisher(Bool, '/mission/start', 1)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.emergency_pub = self.create_publisher(Bool, '/emergency_stop', 1)
        
        # Subscribers
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 1)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.status_sub = self.create_subscription(String, '/system/status', self.status_callback, 1)
        
        self.latest_gps = None
        self.latest_imu = None
        self.latest_odom = None
        self.system_status = "READY"
        
    def gps_callback(self, msg):
        self.latest_gps = msg
        
    def imu_callback(self, msg):
        self.latest_imu = msg
        
    def odom_callback(self, msg):
        self.latest_odom = msg
        
    def status_callback(self, msg):
        self.system_status = msg.data
        
    def publish_mission_start(self):
        msg = Bool()
        msg.data = True
        self.mission_pub.publish(msg)
        
    def publish_mission_stop(self):
        msg = Bool()
        msg.data = False
        self.mission_pub.publish(msg)
        
    def publish_emergency_stop(self):
        msg = Bool()
        msg.data = True
        self.emergency_pub.publish(msg)

# Only define GUI classes if PyQt is available
if QT_AVAILABLE:
    class TractorGUI(QMainWindow):  # type: ignore
        """Professional Qt interface for tractor control"""
        
        def __init__(self):
            super().__init__()
            
            # Initialize ROS node
            rclpy.init()
            self.ros_node = TractorControlNode()
            
            # Setup update timer
            self.timer = QTimer()  # type: ignore
            self.timer.timeout.connect(self.update_displays)
            self.timer.start(100)  # Update every 100ms

            self.init_ui()

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
                    border: 2px solid #4CAF50;
                    border-radius: 8px;
                    margin-top: 10px;
                    padding-top: 10px;
                    background-color: #2d2d2d;
                }
                QGroupBox::title {
                    subcontrol-origin: margin;
                    left: 10px;
                    padding: 0 5px 0 5px;
                    color: #4CAF50;
                }
                QPushButton {
                    background-color: #4CAF50;
                    border: none;
                    padding: 10px 20px;
                    border-radius: 5px;
                    font-weight: bold;
                    font-size: 14px;
                }
                QPushButton:hover {
                    background-color: #45a049;
                }
                QPushButton:pressed {
                    background-color: #3d8b40;
                }
                QPushButton:disabled {
                    background-color: #666666;
                    color: #999999;
                }
                QLabel {
                    color: white;
                    font-size: 12px;
                }
                #title {
                    font-size: 24px;
                    font-weight: bold;
                    color: #4CAF50;
                    margin: 10px;
                }
                #status {
                    font-size: 18px;
                    font-weight: bold;
                    margin: 5px;
                }
                .emergency {
                    background-color: #f44336;
                }
                .emergency:hover {
                    background-color: #da190b;
                }
            """)
            
            # Central widget and main layout
            central_widget = QWidget()  # type: ignore
            self.setCentralWidget(central_widget)
            
            main_layout = QVBoxLayout(central_widget)  # type: ignore
            
            # Title section
            title_layout = QHBoxLayout()  # type: ignore
            title = QLabel('🚜 TRACTOBOTS AUTONOMOUS CONTROL SYSTEM')  # type: ignore
            title.setObjectName('title')
            title.setAlignment(Qt.AlignCenter)  # type: ignore
            title_layout.addWidget(title)
            
            self.status_label = QLabel('🟡 READY')  # type: ignore
            self.status_label.setObjectName('status')
            self.status_label.setAlignment(Qt.AlignCenter)  # type: ignore
            title_layout.addWidget(self.status_label)
            
            main_layout.addLayout(title_layout)
            
            # Main content area
            content_layout = QHBoxLayout()  # type: ignore
            
            # Left control panel
            control_panel = self.create_control_panel()
            content_layout.addWidget(control_panel, 1)
            
            # Center monitoring area
            monitor_panel = self.create_monitor_panel()
            content_layout.addWidget(monitor_panel, 2)
            
            # Right status panel
            status_panel = self.create_status_panel()
            content_layout.addWidget(status_panel, 1)
            
            main_layout.addLayout(content_layout)

        def create_control_panel(self):
            """Create the left control panel"""
            panel = QWidget()  # type: ignore
            layout = QVBoxLayout(panel)  # type: ignore
            
            # Mission Control Group
            mission_group = QGroupBox('Mission Control')  # type: ignore
            mission_layout = QVBoxLayout(mission_group)  # type: ignore
            
            self.start_btn = QPushButton('🚀 START MISSION')  # type: ignore
            self.start_btn.clicked.connect(self.start_mission)
            mission_layout.addWidget(self.start_btn)
            
            self.stop_btn = QPushButton('⏹️ STOP MISSION')  # type: ignore
            self.stop_btn.clicked.connect(self.stop_mission)
            mission_layout.addWidget(self.stop_btn)
            
            self.pause_btn = QPushButton('⏸️ PAUSE MISSION')  # type: ignore
            self.pause_btn.clicked.connect(self.pause_mission)
            mission_layout.addWidget(self.pause_btn)
            
            layout.addWidget(mission_group)
            
            # Emergency Controls
            emergency_group = QGroupBox('Emergency Controls')  # type: ignore
            emergency_layout = QVBoxLayout(emergency_group)  # type: ignore
            
            self.emergency_btn = QPushButton('🚨 EMERGENCY STOP')  # type: ignore
            self.emergency_btn.setProperty('class', 'emergency')
            self.emergency_btn.clicked.connect(self.emergency_stop)
            emergency_layout.addWidget(self.emergency_btn)
            
            layout.addWidget(emergency_group)
            
            # System Controls
            system_group = QGroupBox('System Controls')  # type: ignore
            system_layout = QGridLayout(system_group)  # type: ignore
            
            self.auto_btn = QPushButton('🤖 AUTO MODE')  # type: ignore
            self.manual_btn = QPushButton('👤 MANUAL MODE')  # type: ignore
            
            system_layout.addWidget(self.auto_btn, 0, 0)
            system_layout.addWidget(self.manual_btn, 0, 1)
            
            layout.addWidget(system_group)
            
            return panel

        def create_monitor_panel(self):
            """Create the center monitoring panel"""
            panel = QWidget()  # type: ignore
            layout = QVBoxLayout(panel)  # type: ignore
            
            # Real-time data display
            data_group = QGroupBox('Real-time Monitoring')  # type: ignore
            data_layout = QVBoxLayout(data_group)  # type: ignore
            
            # GPS plot (if pyqtgraph available)
            if pg is not None:
                self.gps_plot = pg.PlotWidget(title="GPS Position")  # type: ignore
                self.gps_plot.setLabel('left', 'Latitude')
                self.gps_plot.setLabel('bottom', 'Longitude')
                data_layout.addWidget(self.gps_plot)
                
                # IMU plot
                self.imu_plot = pg.PlotWidget(title="IMU Data")  # type: ignore
                self.imu_plot.setLabel('left', 'Acceleration (m/s²)')
                self.imu_plot.setLabel('bottom', 'Time')
                data_layout.addWidget(self.imu_plot)
            else:
                placeholder = QLabel('📊 Real-time plots available with pyqtgraph')  # type: ignore
                data_layout.addWidget(placeholder)
            
            layout.addWidget(data_group)
            
            return panel

        def create_status_panel(self):
            """Create the right status panel"""
            panel = QWidget()  # type: ignore
            layout = QVBoxLayout(panel)  # type: ignore
            
            # GPS Status
            gps_group = QGroupBox('GPS Status')  # type: ignore
            gps_layout = QVBoxLayout(gps_group)  # type: ignore
            
            self.lat_label = QLabel('Latitude: --')  # type: ignore
            self.lon_label = QLabel('Longitude: --')  # type: ignore
            self.alt_label = QLabel('Altitude: --')  # type: ignore
            
            gps_layout.addWidget(self.lat_label)
            gps_layout.addWidget(self.lon_label)
            gps_layout.addWidget(self.alt_label)
            
            layout.addWidget(gps_group)
            
            # IMU Status
            imu_group = QGroupBox('IMU Status')  # type: ignore
            imu_layout = QVBoxLayout(imu_group)  # type: ignore
            
            self.roll_label = QLabel('Roll: --')  # type: ignore
            self.pitch_label = QLabel('Pitch: --')  # type: ignore
            self.yaw_label = QLabel('Yaw: --')  # type: ignore
            
            imu_layout.addWidget(self.roll_label)
            imu_layout.addWidget(self.pitch_label)
            imu_layout.addWidget(self.yaw_label)
            
            layout.addWidget(imu_group)
            
            # System Status
            sys_group = QGroupBox('System Status')  # type: ignore
            sys_layout = QVBoxLayout(sys_group)  # type: ignore
            
            self.system_status_label = QLabel('Status: READY')  # type: ignore
            self.connection_label = QLabel('Connection: OK')  # type: ignore
            self.battery_label = QLabel('Battery: --')  # type: ignore
            
            sys_layout.addWidget(self.system_status_label)
            sys_layout.addWidget(self.connection_label)
            sys_layout.addWidget(self.battery_label)
            
            layout.addWidget(sys_group)
            
            return panel

        def update_displays(self):
            """Update all display elements with latest data"""
            # Process ROS callbacks
            rclpy.spin_once(self.ros_node, timeout_sec=0.01)
            
            # Update GPS display
            if self.ros_node.latest_gps:
                gps = self.ros_node.latest_gps
                self.lat_label.setText(f'Latitude: {gps.latitude:.6f}°')
                self.lon_label.setText(f'Longitude: {gps.longitude:.6f}°')
                self.alt_label.setText(f'Altitude: {gps.altitude:.2f}m')
            
            # Update IMU display
            if self.ros_node.latest_imu:
                imu = self.ros_node.latest_imu
                # Convert quaternion to euler angles (simplified)
                q = imu.orientation
                roll = math.atan2(2*(q.w*q.x + q.y*q.z), 1-2*(q.x*q.x + q.y*q.y))
                pitch = math.asin(2*(q.w*q.y - q.z*q.x))
                yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
                
                self.roll_label.setText(f'Roll: {math.degrees(roll):.1f}°')
                self.pitch_label.setText(f'Pitch: {math.degrees(pitch):.1f}°')
                self.yaw_label.setText(f'Yaw: {math.degrees(yaw):.1f}°')
            
            # Update system status
            self.system_status_label.setText(f'Status: {self.ros_node.system_status}')
            
            # Update main status indicator
            if self.ros_node.system_status == "READY":
                self.status_label.setText('🟡 READY')
            elif self.ros_node.system_status == "RUNNING":
                self.status_label.setText('🟢 RUNNING')
            elif self.ros_node.system_status == "ERROR":
                self.status_label.setText('🔴 ERROR')
            else:
                self.status_label.setText(f'🔵 {self.ros_node.system_status}')

        # Control button handlers
        def start_mission(self):
            """Start autonomous mission"""
            self.ros_node.publish_mission_start()
            self.status_label.setText('🟢 MISSION STARTED')

        def stop_mission(self):
            """Stop current mission"""
            self.ros_node.publish_mission_stop()
            self.status_label.setText('🟡 MISSION STOPPED')

        def pause_mission(self):
            """Pause current mission"""
            # Implement pause logic
            self.status_label.setText('🟠 MISSION PAUSED')

        def emergency_stop(self):
            """Emergency stop all operations"""
            self.ros_node.publish_emergency_stop()
            self.status_label.setText('🔴 EMERGENCY STOP')

        def closeEvent(self, event):  # type: ignore
            """Clean shutdown"""
            self.timer.stop()
            if hasattr(self, 'ros_node'):
                self.ros_node.destroy_node()
            rclpy.shutdown()
            event.accept()

def main():
    """Main application entry point"""
    if not QT_AVAILABLE:
        print("❌ PyQt5 is required for the Qt GUI")
        print("Install with: pip install PyQt5 pyqtgraph")
        return False
        
    app = QApplication(sys.argv)  # type: ignore
    
    # Initialize ROS2
    try:
        window = TractorGUI()
        window.show()
        
        # Run the application
        sys.exit(app.exec_())
        
    except Exception as e:
        print(f"❌ Error starting Qt GUI: {e}")
        return False

if __name__ == '__main__':
    main()
