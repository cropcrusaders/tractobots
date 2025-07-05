#!/usr/bin/env python3
"""
Tractobots Professional Windows Dashboard - FIXED VERSION

Author: GitHub Copilot
Date: July 5, 2025

This application provides a native Windows GUI for monitoring and controlling
the Tractobots ROS2 system running in WSL.

ARCHITECTURE FIXES:
- All ROS communication runs in dedicated background threads
- No blocking operations on the GUI thread
- Proper error handling and logging
- Clear workflow: Launch Bridge -> Launch System -> Monitor
"""

import sys
import subprocess
import os
import threading
import json
import math
import time

# Check dependencies first
def check_dependencies():
    """Check for required packages before importing anything GUI-related."""
    missing_packages = []
    
    try:
        import PyQt5
    except ImportError:
        missing_packages.append("PyQt5")
    
    try:
        import matplotlib
    except ImportError:
        missing_packages.append("matplotlib")
    
    try:
        import numpy
    except ImportError:
        missing_packages.append("numpy")
    
    try:
        import roslibpy
    except ImportError:
        missing_packages.append("roslibpy")
    
    if missing_packages:
        print("❌ Missing required packages:")
        for pkg in missing_packages:
            print(f"   - {pkg}")
        print("\nPlease install them with:")
        print("python -m pip install PyQt5 numpy matplotlib roslibpy")
        input("\nPress Enter to exit...")
        return False
    
    print("✅ All dependencies found.")
    return True

# Only import GUI packages after dependency check
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGridLayout, QGroupBox, QTextEdit, QTabWidget,
    QFileDialog, QScrollArea, QSizePolicy, QSpacerItem
)
from PyQt5.QtGui import QFont, QIcon, QColor, QPalette
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, pyqtSlot, QThread
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np
import roslibpy

class WslCommandRunner:
    """Handles WSL command execution with proper error handling."""
    
    def __init__(self, log_callback):
        self.log_callback = log_callback
        
    def run_command(self, command, command_id=None):
        """Run a command in WSL and return the result."""
        # Ensure proper ROS environment sourcing
        full_command = f'wsl -e bash -c "source /opt/ros/humble/setup.bash 2>/dev/null || true; source ~/ros2_humble/install/local_setup.bash 2>/dev/null || true; {command}"'
        
        self.log_callback(f"🔧 Executing: {command}")
        
        try:
            process = subprocess.Popen(
                full_command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                shell=True,
                creationflags=subprocess.CREATE_NO_WINDOW if sys.platform == "win32" else 0
            )
            
            output_lines = []
            if process.stdout:
                for line in iter(process.stdout.readline, ''):
                    line = line.strip()
                    if line:
                        self.log_callback(f"   {line}")
                        output_lines.append(line)
                        
            return_code = process.wait()
            
            if return_code == 0:
                self.log_callback(f"✅ Command completed successfully: {command}")
            else:
                self.log_callback(f"❌ Command failed (exit code {return_code}): {command}")
                
            return return_code, "\n".join(output_lines)
            
        except FileNotFoundError:
            error_msg = "❌ WSL not found. Is WSL installed and in your PATH?"
            self.log_callback(error_msg)
            return -1, error_msg
        except Exception as e:
            error_msg = f"❌ Command execution failed: {e}"
            self.log_callback(error_msg)
            return -1, str(e)

class RosConnectionManager(QObject):
    """Manages ROS connection in a background thread."""
    
    log_signal = pyqtSignal(str)
    connection_status_signal = pyqtSignal(bool)
    odometry_signal = pyqtSignal(dict)
    battery_signal = pyqtSignal(dict)
    
    def __init__(self):
        super().__init__()
        self.client = None
        self.is_connected = False
        self.should_connect = False
        self.connection_thread = None
        
    def start_connection_attempts(self):
        """Start attempting to connect to ROS bridge."""
        self.should_connect = True
        if not self.connection_thread or not self.connection_thread.is_alive():
            self.connection_thread = threading.Thread(target=self._connection_loop, daemon=True)
            self.connection_thread.start()
            
    def stop_connection_attempts(self):
        """Stop attempting to connect."""
        self.should_connect = False
        if self.client and self.is_connected:
            try:
                self.client.terminate()
            except:
                pass
            
    def _connection_loop(self):
        """Background thread that handles ROS connection."""
        while self.should_connect:
            if not self.is_connected:
                try:
                    self.log_signal.emit("🔗 Attempting to connect to ROS bridge...")
                    
                    # Create client with timeout
                    self.client = roslibpy.Ros(host='localhost', port=9090)
                    
                    # Set up event handlers
                    self.client.on('connection', self._on_connect)
                    self.client.on('close', self._on_disconnect)
                    self.client.on('error', self._on_error)
                    
                    # Try to connect (with timeout)
                    self.client.run(timeout=5)
                    
                except Exception as e:
                    self.log_signal.emit(f"🔗 Connection attempt failed: {e}")
                    time.sleep(3)
            else:
                time.sleep(1)  # Keep the thread alive while connected
                
    def _on_connect(self):
        """Called when ROS connection is established."""
        self.is_connected = True
        self.log_signal.emit("✅ Connected to ROS bridge!")
        self.connection_status_signal.emit(True)
        self._setup_subscriptions()
        
    def _on_disconnect(self):
        """Called when ROS connection is lost."""
        if self.is_connected:
            self.log_signal.emit("🔗 Disconnected from ROS bridge")
        self.is_connected = False
        self.connection_status_signal.emit(False)
        
    def _on_error(self, error):
        """Called when ROS connection error occurs."""
        self.log_signal.emit(f"❌ ROS connection error: {error}")
        self.is_connected = False
        self.connection_status_signal.emit(False)
        
    def _setup_subscriptions(self):
        """Set up ROS topic subscriptions."""
        try:
            # Subscribe to odometry
            odom_topic = roslibpy.Topic(self.client, '/odometry/filtered', 'nav_msgs/Odometry')
            odom_topic.subscribe(self._handle_odometry)
            self.log_signal.emit("📡 Subscribed to odometry topic")
            
            # Subscribe to battery
            battery_topic = roslibpy.Topic(self.client, '/battery_state', 'sensor_msgs/BatteryState')
            battery_topic.subscribe(self._handle_battery)
            self.log_signal.emit("📡 Subscribed to battery topic")
            
        except Exception as e:
            self.log_signal.emit(f"❌ Failed to set up subscriptions: {e}")
            
    def _handle_odometry(self, message):
        """Handle incoming odometry data."""
        try:
            pose = message['pose']['pose']
            twist = message['twist']['twist']
            
            x = pose['position']['x']
            y = pose['position']['y']
            
            # Convert quaternion to yaw
            q = pose['orientation']
            _, _, yaw = self._euler_from_quaternion([q['x'], q['y'], q['z'], q['w']])
            
            # Calculate speed
            speed = math.sqrt(twist['linear']['x']**2 + twist['linear']['y']**2)
            
            data = {
                'x': x,
                'y': y,
                'speed': speed,
                'heading': math.degrees(yaw)
            }
            
            self.odometry_signal.emit(data)
            
        except Exception as e:
            self.log_signal.emit(f"❌ Error processing odometry: {e}")
            
    def _handle_battery(self, message):
        """Handle incoming battery data."""
        try:
            percentage = message.get('percentage', 0) * 100
            self.battery_signal.emit({'percentage': percentage})
        except Exception as e:
            self.log_signal.emit(f"❌ Error processing battery data: {e}")
            
    def _euler_from_quaternion(self, q):
        """Convert quaternion to euler angles."""
        x, y, z, w = q
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        
        return roll, pitch, yaw

class MapCanvas(FigureCanvas):
    """A matplotlib canvas for displaying the 2D map."""
    
    def __init__(self, parent=None):
        self.fig = Figure(figsize=(8, 6), dpi=100, tight_layout=True)
        self.axes = self.fig.add_subplot(111)
        super().__init__(self.fig)
        self.setParent(parent)
        self.init_plot()
        
    def init_plot(self):
        """Initialize the plot."""
        self.axes.clear()
        self.axes.set_title("Field Map")
        self.axes.set_xlabel("East (m)")
        self.axes.set_ylabel("North (m)")
        self.axes.grid(True, alpha=0.3)
        self.axes.set_facecolor('#f8f9fa')
        self.draw()
        
    def update_plot(self, x_data, y_data, boundary_x=None, boundary_y=None):
        """Update the plot with new data."""
        self.axes.clear()
        
        # Plot boundary if available
        if boundary_x and boundary_y:
            self.axes.plot(boundary_x, boundary_y, 'k--', linewidth=2, label='Field Boundary')
            
        # Plot path if available
        if x_data and y_data:
            self.axes.plot(x_data, y_data, 'b-', linewidth=1, alpha=0.7, label='Tractor Path')
            if len(x_data) > 0:
                self.axes.scatter(x_data[-1], y_data[-1], c='red', s=50, zorder=5, label='Current Position')
                
        self.axes.set_title("Field Map")
        self.axes.set_xlabel("East (m)")
        self.axes.set_ylabel("North (m)")
        self.axes.grid(True, alpha=0.3)
        self.axes.legend()
        self.axes.axis('equal')
        self.draw()

class TractobotsDashboard(QMainWindow):
    """Main dashboard window."""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Tractobots Control Dashboard - FIXED VERSION")
        self.setGeometry(100, 100, 1400, 800)
        
        # Initialize data
        self.system_status = {
            "ROS Master": False,
            "ROS Bridge": False,
            "Gazebo": False,
            "Navigation": False,
            "GPS Driver": False,
            "IMU Driver": False
        }
        
        self.active_commands = set()
        self.x_data = [0]
        self.y_data = [0]
        self.boundary_x = []
        self.boundary_y = []
        
        # Initialize components
        self.wsl_runner = WslCommandRunner(self.log_message)
        self.ros_manager = RosConnectionManager()
        
        # Set up UI
        self.setup_ui()
        self.setup_connections()
        self.setup_timers()
        
        self.log_message("🚀 Dashboard initialized successfully!")
        self.log_message("📋 To get started:")
        self.log_message("   1. Click 'Launch ROS Bridge' first")
        self.log_message("   2. Wait for connection (green status)")
        self.log_message("   3. Then launch other components")
        
    def setup_ui(self):
        """Set up the user interface."""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QHBoxLayout(central_widget)
        
        # Left panel for controls
        left_panel = QVBoxLayout()
        main_layout.addLayout(left_panel, 1)
        
        # Right panel for tabs
        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs, 2)
        
        # Create control panels
        self.create_control_panel(left_panel)
        self.create_status_panel(left_panel)
        self.create_data_panel(left_panel)
        
        left_panel.addStretch()
        
        # Create tabs
        self.create_map_tab()
        self.create_log_tab()
        
    def create_control_panel(self, layout):
        """Create the control panel."""
        group = QGroupBox("System Control")
        grid = QGridLayout()
        
        # Create buttons
        self.btn_launch_bridge = QPushButton("🚀 Launch ROS Bridge")
        self.btn_launch_system = QPushButton("🛰️ Launch Core System")
        self.btn_launch_gazebo = QPushButton("🌍 Launch Gazebo")
        self.btn_launch_nav = QPushButton("🗺️ Launch Navigation")
        self.btn_stop_all = QPushButton("🛑 Stop All")
        self.btn_load_boundary = QPushButton("📄 Load Boundary")
        self.btn_clear_path = QPushButton("🔄 Clear Path")
        
        # Style buttons
        self.btn_launch_bridge.setStyleSheet("background-color: #8e44ad; color: white; font-weight: bold; padding: 8px;")
        self.btn_launch_system.setStyleSheet("background-color: #27ae60; color: white; font-weight: bold; padding: 8px;")
        self.btn_stop_all.setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold; padding: 8px;")
        
        # Add to grid
        grid.addWidget(self.btn_launch_bridge, 0, 0, 1, 2)
        grid.addWidget(self.btn_launch_system, 1, 0, 1, 2)
        grid.addWidget(self.btn_stop_all, 2, 0, 1, 2)
        grid.addWidget(self.btn_launch_gazebo, 3, 0)
        grid.addWidget(self.btn_launch_nav, 3, 1)
        grid.addWidget(self.btn_load_boundary, 4, 0)
        grid.addWidget(self.btn_clear_path, 4, 1)
        
        group.setLayout(grid)
        layout.addWidget(group)
        
    def create_status_panel(self, layout):
        """Create the status panel."""
        group = QGroupBox("System Status")
        grid = QGridLayout()
        
        self.status_labels = {}
        for i, (name, _) in enumerate(self.system_status.items()):
            label_name = QLabel(name + ":")
            label_status = QLabel("🔴 Stopped")
            label_status.setStyleSheet("font-weight: bold; color: #e74c3c;")
            
            grid.addWidget(label_name, i, 0)
            grid.addWidget(label_status, i, 1)
            
            self.status_labels[name] = label_status
            
        group.setLayout(grid)
        layout.addWidget(group)
        
    def create_data_panel(self, layout):
        """Create the live data panel."""
        group = QGroupBox("Live Data")
        grid = QGridLayout()
        
        self.data_labels = {
            "Position": QLabel("N/A"),
            "Speed": QLabel("N/A"),
            "Heading": QLabel("N/A"),
            "Battery": QLabel("N/A")
        }
        
        for i, (name, label) in enumerate(self.data_labels.items()):
            grid.addWidget(QLabel(name + ":"), i, 0)
            grid.addWidget(label, i, 1)
            
        group.setLayout(grid)
        layout.addWidget(group)
        
    def create_map_tab(self):
        """Create the map tab."""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        self.map_canvas = MapCanvas(widget)
        layout.addWidget(self.map_canvas)
        
        self.tabs.addTab(widget, "🗺️ Field Map")
        
    def create_log_tab(self):
        """Create the log tab."""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Consolas", 9))
        self.log_text.setTextInteractionFlags(Qt.TextSelectableByMouse | Qt.TextSelectableByKeyboard)
        
        layout.addWidget(self.log_text)
        self.tabs.addTab(widget, "📜 System Logs")
        
    def setup_connections(self):
        """Set up signal connections."""
        # Button connections
        self.btn_launch_bridge.clicked.connect(self.launch_ros_bridge)
        self.btn_launch_system.clicked.connect(self.launch_core_system)
        self.btn_launch_gazebo.clicked.connect(self.launch_gazebo)
        self.btn_launch_nav.clicked.connect(self.launch_navigation)
        self.btn_stop_all.clicked.connect(self.stop_all)
        self.btn_load_boundary.clicked.connect(self.load_boundary)
        self.btn_clear_path.clicked.connect(self.clear_path)
        
        # ROS manager connections
        self.ros_manager.log_signal.connect(self.log_message)
        self.ros_manager.connection_status_signal.connect(self.on_ros_connection_status)
        self.ros_manager.odometry_signal.connect(self.on_odometry_data)
        self.ros_manager.battery_signal.connect(self.on_battery_data)
        
    def setup_timers(self):
        """Set up timers for periodic tasks."""
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.check_system_status)
        self.status_timer.start(3000)  # Check every 3 seconds
        
    def log_message(self, message):
        """Log a message to both console and GUI."""
        print(message)  # Console output
        self.log_text.append(f"{time.strftime('%H:%M:%S')} {message}")
        
    def run_command_async(self, command, command_id):
        """Run a command asynchronously."""
        if command_id in self.active_commands:
            self.log_message(f"⚠️ Command '{command_id}' is already running")
            return
            
        self.active_commands.add(command_id)
        
        def execute():
            try:
                return_code, output = self.wsl_runner.run_command(command, command_id)
                self.active_commands.discard(command_id)
                
                if command_id == "check_status":
                    self.process_status_output(output)
                    
            except Exception as e:
                self.log_message(f"❌ Command execution error: {e}")
                self.active_commands.discard(command_id)
                
        thread = threading.Thread(target=execute, daemon=True)
        thread.start()
        
    def process_status_output(self, output):
        """Process the output from ros2 node list."""
        lines = output.strip().split('\n')
        
        # Check if ROS is running
        if "Failed to get node list" in output or not output.strip():
            self.system_status["ROS Master"] = False
            self.system_status["Gazebo"] = False
            self.system_status["Navigation"] = False
            self.system_status["GPS Driver"] = False
            self.system_status["IMU Driver"] = False
        else:
            self.system_status["ROS Master"] = True
            
            # Check for specific nodes
            self.system_status["Gazebo"] = any("/gazebo" in line for line in lines)
            self.system_status["Navigation"] = any(any(node in line for node in ["/controller_server", "/planner_server"]) for line in lines)
            self.system_status["GPS Driver"] = any("navsat_transform_node" in line for line in lines)
            self.system_status["IMU Driver"] = any("advanced_navigation_driver" in line for line in lines)
            
        self.update_status_display()
        
    def update_status_display(self):
        """Update the status display."""
        for name, is_active in self.system_status.items():
            label = self.status_labels[name]
            if is_active:
                label.setText("🟢 Running")
                label.setStyleSheet("font-weight: bold; color: #27ae60;")
            else:
                label.setText("🔴 Stopped")
                label.setStyleSheet("font-weight: bold; color: #e74c3c;")
                
        # Update button states
        bridge_running = self.system_status["ROS Bridge"]
        master_running = self.system_status["ROS Master"]
        
        self.btn_launch_bridge.setEnabled("launch_bridge" not in self.active_commands)
        self.btn_launch_system.setEnabled(bridge_running and not master_running and "launch_system" not in self.active_commands)
        self.btn_launch_gazebo.setEnabled(master_running and not self.system_status["Gazebo"] and "launch_gazebo" not in self.active_commands)
        self.btn_launch_nav.setEnabled(master_running and not self.system_status["Navigation"] and "launch_nav" not in self.active_commands)
        self.btn_stop_all.setEnabled(master_running and "stop_all" not in self.active_commands)
        
    def check_system_status(self):
        """Check the system status."""
        if "check_status" not in self.active_commands:
            self.run_command_async("ros2 node list", "check_status")
            
    def on_ros_connection_status(self, connected):
        """Handle ROS connection status change."""
        self.system_status["ROS Bridge"] = connected
        self.update_status_display()
        
    def on_odometry_data(self, data):
        """Handle new odometry data."""
        self.data_labels["Position"].setText(f"({data['x']:.2f}, {data['y']:.2f})")
        self.data_labels["Speed"].setText(f"{data['speed']:.2f} m/s")
        self.data_labels["Heading"].setText(f"{data['heading']:.1f}°")
        
        # Update path
        self.x_data.append(data['x'])
        self.y_data.append(data['y'])
        
        # Limit path length
        if len(self.x_data) > 1000:
            self.x_data = self.x_data[-500:]
            self.y_data = self.y_data[-500:]
            
        self.map_canvas.update_plot(self.x_data, self.y_data, self.boundary_x, self.boundary_y)
        
    def on_battery_data(self, data):
        """Handle new battery data."""
        self.data_labels["Battery"].setText(f"{data['percentage']:.1f}%")
        
    def launch_ros_bridge(self):
        """Launch the ROS bridge."""
        self.log_message("🚀 Starting ROS bridge...")
        self.run_command_async("ros2 launch rosbridge_server rosbridge_websocket_launch.xml", "launch_bridge")
        
        # Start connection attempts after a delay
        QTimer.singleShot(3000, self.ros_manager.start_connection_attempts)
        
    def launch_core_system(self):
        """Launch the core system."""
        self.log_message("🛰️ Starting core system...")
        self.run_command_async("ros2 launch tractobots_launchers bringup.launch.py", "launch_system")
        
    def launch_gazebo(self):
        """Launch Gazebo."""
        self.log_message("🌍 Starting Gazebo...")
        self.run_command_async("ros2 launch tractobots_gazebo gazebo.launch.py", "launch_gazebo")
        
    def launch_navigation(self):
        """Launch navigation."""
        self.log_message("🗺️ Starting navigation...")
        self.run_command_async("ros2 launch tractobots_nav2 navigation.launch.py", "launch_nav")
        
    def stop_all(self):
        """Stop all processes."""
        self.log_message("🛑 Stopping all processes...")
        self.ros_manager.stop_connection_attempts()
        self.run_command_async("pkill -f 'ros2 launch' || true", "stop_all")
        
    def load_boundary(self):
        """Load boundary file."""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Load Boundary File", "", "CSV Files (*.csv);;All Files (*)"
        )
        
        if file_path:
            try:
                self.log_message(f"📄 Loading boundary from {file_path}")
                data = np.loadtxt(file_path, delimiter=',', ndmin=2)
                
                self.boundary_x = data[:, 0].tolist()
                self.boundary_y = data[:, 1].tolist()
                
                # Close the boundary if needed
                if self.boundary_x and (self.boundary_x[0] != self.boundary_x[-1] or self.boundary_y[0] != self.boundary_y[-1]):
                    self.boundary_x.append(self.boundary_x[0])
                    self.boundary_y.append(self.boundary_y[0])
                    
                self.map_canvas.update_plot(self.x_data, self.y_data, self.boundary_x, self.boundary_y)
                self.log_message("✅ Boundary loaded successfully")
                
            except Exception as e:
                self.log_message(f"❌ Failed to load boundary: {e}")
                
    def clear_path(self):
        """Clear the path data."""
        self.log_message("🔄 Clearing path...")
        current_x = self.x_data[-1] if self.x_data else 0
        current_y = self.y_data[-1] if self.y_data else 0
        
        self.x_data = [current_x]
        self.y_data = [current_y]
        
        self.map_canvas.update_plot(self.x_data, self.y_data, self.boundary_x, self.boundary_y)
        
    def closeEvent(self, event):
        """Handle application close."""
        self.log_message("🔚 Shutting down dashboard...")
        self.status_timer.stop()
        self.ros_manager.stop_connection_attempts()
        super().closeEvent(event)

def main():
    """Main application entry point."""
    print("🚀 Starting Tractobots Dashboard...")
    
    if not check_dependencies():
        return
        
    app = QApplication(sys.argv)
    
    # Set application style
    app.setStyle('Fusion')
    
    window = TractobotsDashboard()
    window.show()
    
    print("✅ Dashboard launched successfully!")
    print("📋 Check the GUI for next steps.")
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
