#!/usr/bin/env python3
"""
Tractobots Professional Windows Dashboard

Author: GitHub Copilot
Date: July 4, 2025

This application provides a native Windows GUI for monitoring and controlling
the Tractobots ROS2 system running in WSL.
"""

import sys
import subprocess
import os
import threading
import json
import math

# --- Pre-computation for imports ---
# We will define the dependencies here and check them in the main block.
REQUIRED_PACKAGES = {
    'PyQt5': 'PyQt5',
    'matplotlib': 'matplotlib',
    'numpy': 'numpy',
    'roslibpy': 'roslibpy'
}

# Placeholder for imports that will be checked later
QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, \
QPushButton, QLabel, QGridLayout, QGroupBox, QTextEdit, QTabWidget, \
QFileDialog, QFont, QIcon, Qt, QTimer, pyqtSignal, QObject, pyqtSlot, QThread, \
QScrollArea, QSizePolicy, QSpacerItem, QColor, QPalette, \
FigureCanvas, Figure, np, roslibpy = [None] * 29


def check_and_import_dependencies():
    """Checks for all required packages and imports them globally."""
    global QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, \
           QPushButton, QLabel, QGridLayout, QGroupBox, QTextEdit, QTabWidget, \
           QFileDialog, QFont, QIcon, Qt, QTimer, pyqtSignal, QObject, pyqtSlot, QThread, \
           QScrollArea, QSizePolicy, QSpacerItem, QColor, QPalette, \
           FigureCanvas, Figure, np, roslibpy

    try:
        from PyQt5.QtWidgets import (
            QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
            QPushButton, QLabel, QGridLayout, QGroupBox, QTextEdit, QTabWidget,
            QFileDialog
        )
        from PyQt5.QtGui import QFont, QIcon
        from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, pyqtSlot, QThread
        from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
        from matplotlib.figure import Figure
        import numpy as np
        import roslibpy
        print("[DEBUG] All dependencies imported successfully.")
        return True
    except ImportError as e:
        print("--- DEPENDENCY ERROR ---")
        error_message = (
            f"\n❌ Missing required Python package: '{e.name}'.\n\n"
            f"This dashboard cannot run without it. Please install the required packages\n"
            f"by opening a PowerShell or Command Prompt and running:\n\n"
            f"  python -m pip install PyQt5 numpy matplotlib roslibpy\n"
        )
        print(error_message)
        # In some environments, the console window closes instantly.
        # A raw input keeps it open so the user can read the error.
        input("\nPress Enter to exit...")
        return False

def main():
    """Main function to run the application after checking dependencies."""
    # --- Worker Classes (Depend on PyQt5) ---
    class WslRunner(QObject):
        """Worker to run a WSL command in a separate thread and stream output."""
        log_message = pyqtSignal(str)
        process_finished = pyqtSignal(str, int, str)  # command_id, return_code, output

        @pyqtSlot(str, str)
        def run_command(self, command_id, command):
            """Runs a command in WSL by spawning a new thread for each command."""
            thread = threading.Thread(target=self._execute, args=(command_id, command), daemon=True)
            thread.start()

        def _execute(self, command_id, command):
            # Ensure the ROS environment is sourced for every command
            full_command = f'wsl -e bash -c "source /opt/ros/humble/setup.bash && source ~/ros2_humble/install/local_setup.bash && {command}"'
            self.log_message.emit(f">>> Executing: {command}")
            output_lines = []
            try:
                creation_flags = subprocess.CREATE_NO_WINDOW if sys.platform == "win32" else 0
                process = subprocess.Popen(
                    full_command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, shell=True, creationflags=creation_flags,
                    encoding='utf-8', errors='replace'
                )
                if process.stdout:
                    for line in iter(process.stdout.readline, ''):
                        line = line.strip()
                        self.log_message.emit(line)
                        output_lines.append(line)
                    process.stdout.close()
                return_code = process.wait()
                self.process_finished.emit(command_id, return_code, "\n".join(output_lines))
            except FileNotFoundError:
                error_msg = "[ERROR] `wsl.exe` not found. Is WSL installed and in your PATH?"
                self.log_message.emit(error_msg)
                self.process_finished.emit(command_id, -1, error_msg)
            except Exception as e:
                error_msg = f"[ERROR] Command failed: {command}. Exception: {e}"
                self.log_message.emit(error_msg)
                self.process_finished.emit(command_id, -1, str(e))

    class RosWorker(QObject):
        """Worker to manage ROS connection and subscriptions in a background thread."""
        log_message = pyqtSignal(str)
        connection_status = pyqtSignal(bool)
        new_odometry_data = pyqtSignal(dict)
        new_battery_data = pyqtSignal(dict)

        def __init__(self, host='localhost', port=9090):
            super().__init__()
            self.host, self.port = host, port
            self.client = None
            self._is_running = True
            self._is_connected = False

        @pyqtSlot()
        def run(self):
            """Main loop for the ROS worker thread."""
            self.log_message.emit("[ROS_WORKER] Starting ROS worker thread.")
            while self._is_running:
                if not self._is_connected:
                    try:
                        self.log_message.emit(f"[ROS_WORKER] Attempting to connect to rosbridge at {self.host}:{self.port}...")
                        self.client = roslibpy.Ros(host=self.host, port=self.port)
                        self.client.on('ready', self.on_ros_connect)
                        self.client.on('close', self.on_ros_disconnect)
                        self.client.on('error', self.on_ros_error)
                        self.client.run()  # This is a blocking call
                        self.log_message.emit("[ROS_WORKER] roslibpy client terminated.")
                    except Exception as e:
                        self.log_message.emit(f"[ROS_WORKER-ERROR] Connection failed: {e}")
                        self.on_ros_disconnect() # Ensure status is updated
                # Wait before retrying to avoid spamming connection attempts
                QThread.sleep(5)

        def on_ros_connect(self):
            """Callback for successful connection."""
            self.log_message.emit("[ROS_WORKER] Successfully connected to rosbridge.")
            self._is_connected = True
            self.connection_status.emit(True)
            self.setup_subscriptions()

        def on_ros_disconnect(self, _=None):
            """Callback for disconnection."""
            if self._is_connected:
                self.log_message.emit("[ROS_WORKER] Disconnected from rosbridge.")
            self._is_connected = False
            self.connection_status.emit(False)

        def on_ros_error(self, error):
            """Callback for connection errors."""
            self.log_message.emit(f"[ROS_WORKER-ERROR] {error}")
            self._is_connected = False
            self.connection_status.emit(False)

        def setup_subscriptions(self):
            """Sets up ROS topic subscriptions."""
            if not self.client or not self._is_connected:
                self.log_message.emit("[ROS_WORKER-WARN] Cannot subscribe, client not connected.")
                return
            try:
                odom_topic = roslibpy.Topic(self.client, '/odometry/filtered', 'nav_msgs/Odometry')
                odom_topic.subscribe(self.handle_odometry)
                self.log_message.emit(f"[ROS_WORKER] Subscribed to {odom_topic.name}")

                battery_topic = roslibpy.Topic(self.client, '/battery_state', 'sensor_msgs/BatteryState')
                battery_topic.subscribe(self.handle_battery)
                self.log_message.emit(f"[ROS_WORKER] Subscribed to {battery_topic.name}")
            except Exception as e:
                self.log_message.emit(f"[ROS_WORKER-ERROR] Failed to subscribe to topics: {e}")

        def handle_odometry(self, message):
            pose, twist = message['pose']['pose'], message['twist']['twist']
            x, y = pose['position']['x'], pose['position']['y']
            q = pose['orientation']
            _, _, yaw = self.euler_from_quaternion([q['x'], q['y'], q['z'], q['w']])
            speed = math.sqrt(twist['linear']['x']**2 + twist['linear']['y']**2)
            self.new_odometry_data.emit({'x': x, 'y': y, 'speed': speed, 'heading': math.degrees(yaw)})

        def handle_battery(self, message):
            self.new_battery_data.emit({'percentage': message['percentage'] * 100})

        def euler_from_quaternion(self, q):
            x, y, z, w = q
            t0, t1 = +2.0 * (w * x + y * z), +1.0 - 2.0 * (x * x + y * y)
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else (-1.0 if t2 < -1.0 else t2)
            t3, t4 = +2.0 * (w * z + x * y), +1.0 - 2.0 * (y * y + z * z)
            return math.atan2(t0, t1), math.asin(t2), math.atan2(t3, t4)

        @pyqtSlot()
        def stop(self):
            """Stops the worker thread."""
            self.log_message.emit("[ROS_WORKER] Stopping ROS worker thread.")
            self._is_running = False
            if self.client and self._is_connected:
                self.client.terminate()

    class MapCanvas(FigureCanvas):
        """A matplotlib canvas to display the 2D field map."""
        def __init__(self, parent=None, width=5, height=4, dpi=100):
            self.fig = Figure(figsize=(width, height), dpi=dpi, tight_layout=True)
            self.axes = self.fig.add_subplot(111)
            super(MapCanvas, self).__init__(self.fig)
            self.setParent(parent)
            self.plot_initial_state()

        def plot_initial_state(self):
            self.axes.clear()
            self.axes.set_facecolor('#f0f0f0')
            self.axes.set_title("Field Map"); self.axes.set_xlabel("East (m)"); self.axes.set_ylabel("North (m)")
            self.axes.grid(True, linestyle='--', alpha=0.6); self.draw()

        def update_plot(self, x_data, y_data, boundary_x=None, boundary_y=None):
            self.axes.clear()
            if boundary_x is not None and boundary_y is not None and len(boundary_x) > 0:
                self.axes.plot(boundary_x, boundary_y, 'k--', label='Field Boundary')
            if x_data and y_data:
                self.axes.plot(x_data, y_data, 'b-', marker='o', markersize=2, linewidth=1, label='Tractor Path')
                self.axes.scatter(x_data[-1:], y_data[-1:], c='red', s=40, zorder=5, label='Current Position')
            self.axes.set_title("Field Map"); self.axes.set_xlabel("East (m)"); self.axes.set_ylabel("North (m)")
            self.axes.legend(loc='upper right'); self.axes.grid(True, linestyle='--', alpha=0.6)
            self.axes.axis('equal'); self.draw()

        def clear_path(self, boundary_x=None, boundary_y=None):
            self.update_plot([], [], boundary_x, boundary_y)

    class DashboardWindow(QMainWindow):
        """The main application window."""
        execute_wsl_command = pyqtSignal(str, str)

        def __init__(self):
            super().__init__()
            self.setWindowTitle("Tractobots Control Dashboard")
            self.setGeometry(100, 100, 1600, 900)
            # Add ROS Bridge to the status dictionary
            self.system_status = {
                "ROS Master": False, "ROS Bridge": False, "Gazebo": False,
                "Navigation": False, "GPS Driver": False, "IMU Driver": False
            }
            self.active_commands = set()
            self.sim_x, self.sim_y, self.boundary_x, self.boundary_y = [0], [0], [], []
            self.setup_ui()
            self.setup_workers()
            self.connect_controls()
            self.check_system_status()

        def setup_ui(self):
            self.central_widget = QWidget()
            self.setCentralWidget(self.central_widget)
            self.main_layout = QHBoxLayout(self.central_widget)
            self.left_panel_layout = QVBoxLayout()
            self.main_layout.addLayout(self.left_panel_layout, 1)
            self.right_panel_tabs = QTabWidget()
            self.main_layout.addWidget(self.right_panel_tabs, 3)
            self.create_control_panel(); self.create_status_panel(); self.create_live_data_panel()
            self.left_panel_layout.addStretch()
            self.create_map_tab(); self.create_log_tab()

        def setup_workers(self):
            self.wsl_thread = QThread(); self.wsl_worker = WslRunner()
            self.wsl_worker.moveToThread(self.wsl_thread)
            self.execute_wsl_command.connect(self.wsl_worker.run_command)
            self.wsl_worker.log_message.connect(self.log_message)
            self.wsl_worker.process_finished.connect(self.on_process_finished)
            self.wsl_thread.start()

            self.ros_thread = QThread(); self.ros_worker = RosWorker()
            self.ros_worker.moveToThread(self.ros_thread)
            self.ros_worker.log_message.connect(self.log_message)
            self.ros_worker.connection_status.connect(self.on_ros_connection_status)
            self.ros_worker.new_odometry_data.connect(self.on_new_odometry)
            self.ros_worker.new_battery_data.connect(self.on_new_battery)
            # Connect thread management signals
            self.ros_thread.started.connect(self.ros_worker.run)
            self.ros_thread.start()

            self.status_timer = QTimer(self)
            self.status_timer.timeout.connect(self.check_system_status)
            self.status_timer.start(5000)

        def run_wsl_command(self, cmd_id, cmd, btn=None):
            if cmd_id in self.active_commands:
                self.log_message(f"[WARN] Command '{cmd_id}' is already running.")
                return
            if btn: btn.setEnabled(False)
            self.active_commands.add(cmd_id)
            self.execute_wsl_command.emit(cmd_id, cmd)

        @pyqtSlot(str)
        def log_message(self, msg):
            """Logs a message to the GUI and the console."""
            print(msg)  # Print to console for debugging
            self.log_text_edit.append(msg)

        @pyqtSlot(str, int, str)
        def on_process_finished(self, cmd_id, _, output):
            self.log_message(f"[INFO] Process '{cmd_id}' finished.")
            self.active_commands.discard(cmd_id)
            if cmd_id == 'check_status': self.update_status_labels(output)
            else: self.check_system_status()

        def connect_controls(self):
            self.btn_launch_ros_bridge.clicked.connect(self.launch_ros_bridge)
            self.btn_launch_all.clicked.connect(self.launch_all)
            self.btn_stop_all.clicked.connect(self.stop_all)
            self.btn_launch_gazebo.clicked.connect(self.launch_gazebo)
            self.btn_launch_nav.clicked.connect(self.launch_nav)
            self.btn_load_boundary.clicked.connect(self.load_boundary)
            self.btn_clear_path.clicked.connect(self.clear_path)

        def create_control_panel(self):
            control_group = QGroupBox("System Control")
            layout = QGridLayout()
            # Add the new ROS Bridge button
            self.btn_launch_ros_bridge = QPushButton("🚀 Launch ROS Bridge")
            self.btn_launch_all = QPushButton("🛰️ Launch Core System")
            self.btn_stop_all = QPushButton("🛑 Terminate All")
            self.btn_launch_gazebo = QPushButton("🌍 Launch Gazebo")
            self.btn_launch_nav = QPushButton("🗺️ Launch Navigation")
            self.btn_load_boundary = QPushButton("🔲 Load Boundary")
            self.btn_clear_path = QPushButton("🔄 Clear Path")
            
            # Set styles for new buttons
            self.btn_launch_ros_bridge.setStyleSheet("background-color: #8e44ad; color: white; font-weight: bold;")
            self.btn_launch_all.setStyleSheet("background-color: #27ae60; color: white; font-weight: bold;")
            self.btn_stop_all.setStyleSheet("background-color: #c0392b; color: white; font-weight: bold;")

            # Add buttons to layout
            layout.addWidget(self.btn_launch_ros_bridge, 0, 0, 1, 2)
            layout.addWidget(self.btn_launch_all, 1, 0, 1, 2)
            layout.addWidget(self.btn_stop_all, 2, 0, 1, 2)
            layout.addWidget(self.btn_launch_gazebo, 3, 0)
            layout.addWidget(self.btn_launch_nav, 3, 1)
            layout.addWidget(self.btn_load_boundary, 4, 0)
            layout.addWidget(self.btn_clear_path, 4, 1)

            control_group.setLayout(layout)
            self.left_panel_layout.addWidget(control_group)

        def create_status_panel(self):
            status_group = QGroupBox("System Status")
            layout = QGridLayout()
            self.status_labels = {name: QLabel("⚫ Unknown") for name in self.system_status}
            for row, (name, label) in enumerate(self.status_labels.items()):
                layout.addWidget(QLabel(name), row, 0); layout.addWidget(label, row, 1)
            status_group.setLayout(layout); self.left_panel_layout.addWidget(status_group)

        def create_live_data_panel(self):
            data_group = QGroupBox("Live Data")
            layout = QGridLayout()
            self.live_data_labels = {name: QLabel("N/A") for name in ["Position (X, Y)", "Speed (m/s)", "Heading (°)", "Battery (%)"]}
            for row, (name, label) in enumerate(self.live_data_labels.items()):
                layout.addWidget(QLabel(name), row, 0); layout.addWidget(label, row, 1)
            data_group.setLayout(layout); self.left_panel_layout.addWidget(data_group)

        def create_map_tab(self):
            map_widget = QWidget(); layout = QVBoxLayout(map_widget)
            self.map_canvas = MapCanvas(map_widget); layout.addWidget(self.map_canvas)
            self.right_panel_tabs.addTab(map_widget, "🗺️ 2D Field Map")

        def create_log_tab(self):
            log_widget = QWidget(); layout = QVBoxLayout(log_widget)
            self.log_text_edit = QTextEdit()
            self.log_text_edit.setReadOnly(True)
            # Make the log text selectable
            self.log_text_edit.setTextInteractionFlags(Qt.TextSelectableByMouse | Qt.TextSelectableByKeyboard)
            self.log_text_edit.setFont(QFont("Courier", 9))
            layout.addWidget(self.log_text_edit)
            self.right_panel_tabs.addTab(log_widget, "📜 System Logs")

        def check_system_status(self):
            if "check_status" not in self.active_commands:
                self.run_wsl_command("check_status", "ros2 node list")

        @pyqtSlot(str)
        def update_status_labels(self, node_list_output):
            nodes = node_list_output.strip().splitlines()
            is_master_running = not ("Failed to get node list" in node_list_output or not node_list_output)
            self.system_status["ROS Master"] = is_master_running
            if is_master_running:
                self.system_status["Gazebo"] = any("/gazebo" in n for n in nodes)
                self.system_status["Navigation"] = any(n in nodes for n in ["/controller_server", "/planner_server"])
                self.system_status["GPS Driver"] = any("navsat_transform_node" in n for n in nodes)
                self.system_status["IMU Driver"] = any("advanced_navigation_driver" in n for n in nodes)
            else:
                for key in ["Gazebo", "Navigation", "GPS Driver", "IMU Driver"]: self.system_status[key] = False
            self.update_gui_elements()

        def update_gui_elements(self):
            for name, is_active in self.system_status.items():
                label = self.status_labels[name]
                text, style = ("🟢 Running", "color: #27ae60;") if is_active else ("🔴 Stopped", "color: #c0392b;")
                label.setText(text)
                label.setStyleSheet(f"font-weight: bold; {style}")

            ros_is_running = self.system_status["ROS Master"]
            bridge_is_running = self.system_status["ROS Bridge"]

            self.btn_launch_ros_bridge.setEnabled(not bridge_is_running and "launch_ros_bridge" not in self.active_commands)
            self.btn_launch_all.setEnabled(bridge_is_running and not ros_is_running and "launch_all" not in self.active_commands)
            self.btn_stop_all.setEnabled(ros_is_running and "stop_all" not in self.active_commands)
            self.btn_launch_gazebo.setEnabled(ros_is_running and not self.system_status["Gazebo"] and "launch_gazebo" not in self.active_commands)
            self.btn_launch_nav.setEnabled(ros_is_running and not self.system_status["Navigation"] and "launch_nav" not in self.active_commands)

        @pyqtSlot(dict)
        def on_new_odometry(self, data):
            self.live_data_labels["Position (X, Y)"].setText(f"{data['x']:.2f}, {data['y']:.2f}")
            self.live_data_labels["Speed (m/s)"].setText(f"{data['speed']:.2f}")
            self.live_data_labels["Heading (°)"].setText(f"{data['heading']:.1f}")
            self.sim_x.append(data['x']); self.sim_y.append(data['y'])
            self.map_canvas.update_plot(self.sim_x, self.sim_y, self.boundary_x, self.boundary_y)

        @pyqtSlot(dict)
        def on_new_battery(self, data):
            self.live_data_labels["Battery (%)"].setText(f"{data['percentage']:.1f}")

        @pyqtSlot(bool)
        def on_ros_connection_status(self, is_connected):
            self.system_status["ROS Bridge"] = is_connected
            if not is_connected:
                # If bridge disconnects, assume ROS master is also gone for status purposes
                self.system_status["ROS Master"] = False
            self.update_gui_elements()

        def launch_ros_bridge(self):
            """Launches the rosbridge server."""
            cmd = "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
            self.run_wsl_command("launch_ros_bridge", cmd, self.btn_launch_ros_bridge)

        def launch_all(self):
            self.run_wsl_command("launch_all", "ros2 launch tractobots_launchers bringup.launch.py", self.btn_launch_all)

        def launch_gazebo(self):
            self.run_wsl_command("launch_gazebo", "ros2 launch tractobots_gazebo gazebo.launch.py", self.btn_launch_gazebo)

        def launch_nav(self):
            self.run_wsl_command("launch_nav", "ros2 launch tractobots_nav2 navigation.launch.py", self.btn_launch_nav)

        def stop_all(self):
            self.run_wsl_command("stop_all", "pkill -f 'ros2 launch' || true", self.btn_stop_all)

        def load_boundary(self):
            filePath, _ = QFileDialog.getOpenFileName(self, "Load Boundary File", "", "CSV Files (*.csv);;All Files (*)")
            if not filePath: return
            try:
                self.log_message(f"[INFO] Loading boundary from {filePath}")
                data = np.loadtxt(filePath, delimiter=',', ndmin=2)
                self.boundary_x, self.boundary_y = data[:, 0].tolist(), data[:, 1].tolist()
                if self.boundary_x and (self.boundary_x[0] != self.boundary_x[-1] or self.boundary_y[0] != self.boundary_y[-1]):
                    self.boundary_x.append(self.boundary_x[0]); self.boundary_y.append(self.boundary_y[0])
                self.map_canvas.update_plot(self.sim_x, self.sim_y, self.boundary_x, self.boundary_y)
                self.log_message("[INFO] Boundary loaded successfully.")
            except Exception as e:
                self.log_message(f"[ERROR] Failed to load boundary file: {e}")

        def clear_path(self):
            self.log_message("[INFO] Clearing tractor path.")
            self.sim_x = [self.sim_x[-1]] if self.sim_x else [0]
            self.sim_y = [self.sim_y[-1]] if self.sim_y else [0]
            self.map_canvas.update_plot(self.sim_x, self.sim_y, self.boundary_x, self.boundary_y)

        def closeEvent(self, event):
            self.log_message("[INFO] Closing dashboard...")
            self.status_timer.stop()
            # Cleanly stop worker threads
            self.ros_worker.stop()
            self.ros_thread.quit()
            self.ros_thread.wait(2000) # Wait up to 2s
            self.wsl_thread.quit()
            self.wsl_thread.wait(2000) # Wait up to 2s
            super(DashboardWindow, self).closeEvent(event)

    # --- Application Entry Point ---
    app = QApplication(sys.argv)
    window = DashboardWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    print("[DEBUG] Starting dashboard script...")
    # First, check dependencies. If they are not met, the script will exit.
    if check_and_import_dependencies():
        print("[DEBUG] Dependencies met. Launching application...")
        main()
    else:
        print("[DEBUG] Dependency check failed. Exiting.")
