#!/usr/bin/env python3
"""
Tractobots Professional Windows Dashboard

This application provides a native Windows GUI for monitoring and controlling
the Tractobots ROS2 system running in WSL.
"""

import sys
import subprocess
import os
import threading
import json
import math
import time
from datetime import datetime

# Import required packages directly
try:
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QPushButton, QLabel, QGridLayout, QGroupBox, QTextEdit, QTabWidget,
        QFileDialog, QScrollArea, QSizePolicy, QSpacerItem, QShortcut
    )
    from PyQt5.QtGui import QFont, QIcon, QColor, QPalette, QKeySequence
    from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, pyqtSlot, QThread
    
    import matplotlib
    matplotlib.use('Qt5Agg')
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.figure import Figure
    
    import numpy as np
    import roslibpy
    
    DEPENDENCIES_MET = True
    print("[INFO] All dependencies loaded successfully")
    
except ImportError as e:
    print(f"[ERROR] Missing dependency: {e}")
    DEPENDENCIES_MET = False
    error_message = (
        f"\n❌ Missing required Python package.\n\n"
        f"This dashboard cannot run without it. Please install the required packages\n"
        f"by opening a PowerShell or Command Prompt and running:\n\n"
        f"  python -m pip install PyQt5 numpy matplotlib roslibpy\n"
    )
    print(error_message)
    input("\nPress Enter to exit...")
    sys.exit(1)


# Set up Unicode handling for Windows console
if sys.platform.startswith('win'):
    try:
        import codecs
        import io
        # Try to set UTF-8 encoding for stdout/stderr
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
        sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')
    except Exception:
        # If that fails, we'll handle Unicode errors in individual print statements
        pass


# --- Worker Classes ---
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
        # Enhanced ROS environment sourcing with error handling
        setup_commands = [
            "source /opt/ros/jazzy/setup.bash 2>/dev/null || echo 'WARNING: ROS Jazzy not found in /opt/ros/jazzy'",
            "source ~/tractobots/install/setup.bash 2>/dev/null || echo 'WARNING: Tractobots workspace not found'"
        ]
        
        # Combine environment setup with user command
        full_command = "; ".join(setup_commands + [command])
        
        try:
            # Run the command in WSL
            process = subprocess.Popen(
                ["wsl", "bash", "-c", full_command],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )
            
            # Collect all output
            all_output = []
            
            # Stream output as it comes
            for line in iter(process.stdout.readline, ''):
                # Remove ANSI color codes and other terminal control sequences
                clean_line = self._clean_terminal_output(line)
                if clean_line:
                    self.log_message.emit(clean_line)
                    all_output.append(clean_line)
            
            # Wait for process to finish and get return code
            return_code = process.wait()
            
            # Emit the completion signal with command ID, return code and all output
            self.process_finished.emit(
                command_id, 
                return_code,
                "\n".join(all_output)
            )
            
        except Exception as e:
            error_msg = f"Error executing command: {e}"
            self.log_message.emit(error_msg)
            self.process_finished.emit(command_id, 1, error_msg)
    
    def _clean_terminal_output(self, line):
        """Remove ANSI color codes and other terminal control sequences."""
        import re
        # Pattern to match ANSI escape sequences
        ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
        # Replace all emoji with simpler ASCII alternatives
        line = line.replace('✓', '[OK]')
        line = line.replace('✅', '[OK]')
        line = line.replace('❌', '[ERROR]')
        line = line.replace('⚠️', '[WARNING]')
        line = line.replace('🚀', '[LAUNCH]')
        line = line.replace('🔧', '[CONFIG]')
        line = line.replace('🔍', '[SEARCH]')
        line = line.replace('📋', '[LIST]')
        # Strip ANSI codes
        return ansi_escape.sub('', line).strip()


class RosWorker(QThread):
    """Worker thread for ROS communication."""
    connection_status = pyqtSignal(bool, str)
    node_status_updated = pyqtSignal(list)
    topic_list_updated = pyqtSignal(list)
    system_status_updated = pyqtSignal(dict)
    message_received = pyqtSignal(str, object)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.running = True
        self.connected = False
        self.client = None
        self.retry_count = 0
        self.retry_max = 10
        self.retry_delay = 2  # Start with 2 seconds
        self.subscriptions = {}
        self.wsl_ip = None
        
    def run(self):
        """Main thread loop that handles connection and status checks."""
        while self.running:
            if not self.connected:
                self.connect_to_rosbridge()
            else:
                try:
                    # Check if the connection is still alive
                    if not self.client.is_connected:
                        self.connected = False
                        self.connection_status.emit(False, "Connection lost. Reconnecting...")
                        time.sleep(1)
                        continue
                    
                    # Get nodes list
                    self.get_node_list()
                    
                    # Get system stats
                    self.get_system_stats()
                    
                    # Sleep to avoid CPU usage
                    time.sleep(2)
                
                except Exception as e:
                    self.connection_status.emit(False, f"Error in ROS communication: {str(e)}")
                    self.connected = False
                    time.sleep(2)
            
            time.sleep(0.1)  # Small sleep to avoid CPU hammering
        
        # Clean up when thread is stopping
        if self.client and self.client.is_connected:
            try:
                self.client.terminate()
            except:
                pass
    
    def get_wsl_ip(self):
        """Get the IP address of the WSL instance."""
        try:
            result = subprocess.run(
                ["wsl", "-e", "bash", "-c", "hostname -I | awk '{print $1}'"],
                capture_output=True, text=True, timeout=10
            )
            if result.returncode == 0:
                wsl_ip = result.stdout.strip()
                return wsl_ip
            else:
                return None
        except Exception:
            return None
    
    def connect_to_rosbridge(self):
        """Connect to the rosbridge server with retry logic."""
        if self.client and self.client.is_connected:
            # Already connected, avoid creating multiple clients
            return
            
        # Clean up any existing client
        if self.client:
            try:
                self.client.terminate()
            except:
                pass
            self.client = None
        
        try:
            # Get WSL IP if we don't have it yet
            if not self.wsl_ip:
                self.wsl_ip = self.get_wsl_ip() or "127.0.0.1"  # Fallback to localhost
            
            # Create a new client
            self.client = roslibpy.Ros(host=self.wsl_ip, port=9090)
            
            # Try to connect
            self.client.on_ready(lambda: self.on_connection_success())
            self.client.on_error(lambda: self.on_connection_error())
            
            # Connect with a timeout
            self.client.run(timeout=5)
            
            # Reset retry counter on success
            self.retry_count = 0
            
        except Exception as e:
            # Handle connection failure with exponential backoff
            self.retry_count += 1
            backoff = min(30, self.retry_delay * (2 ** min(self.retry_count, 4)))  # Cap at ~30s
            
            if self.retry_count <= self.retry_max:
                status_msg = f"Connection failed. Retry {self.retry_count}/{self.retry_max} in {backoff}s... ({str(e)})"
                self.connection_status.emit(False, status_msg)
                time.sleep(backoff)
            else:
                status_msg = f"Connection failed after {self.retry_count} attempts. Check if rosbridge is running. ({str(e)})"
                self.connection_status.emit(False, status_msg)
                time.sleep(5)  # Longer sleep after max retries
                self.retry_count = 0  # Reset to try again
    
    def on_connection_success(self):
        """Called when connection is successful."""
        self.connected = True
        self.connection_status.emit(True, f"Connected to ROS (WSL IP: {self.wsl_ip})")
        self.get_topic_list()
    
    def on_connection_error(self):
        """Called when connection fails."""
        self.connected = False
        self.connection_status.emit(False, "ROS connection error")
        
    def stop(self):
        """Stop the worker thread."""
        self.running = False
        if self.client and self.client.is_connected:
            try:
                self.client.terminate()
            except:
                pass
        self.wait()
    
    def get_node_list(self):
        """Get list of running ROS nodes."""
        if not self.connected:
            return
            
        try:
            service = roslibpy.Service(self.client, '/rosapi/nodes', 'rosapi/Nodes')
            request = roslibpy.ServiceRequest()
            service.call(request, lambda result: self.handle_node_list(result))
        except Exception as e:
            print(f"Error getting node list: {e}")
    
    def handle_node_list(self, result):
        """Handle node list result."""
        if 'nodes' in result:
            self.node_status_updated.emit(result['nodes'])
    
    def get_topic_list(self):
        """Get list of available ROS topics."""
        if not self.connected:
            return
            
        try:
            service = roslibpy.Service(self.client, '/rosapi/topics', 'rosapi/Topics')
            request = roslibpy.ServiceRequest()
            service.call(request, lambda result: self.handle_topic_list(result))
        except Exception as e:
            print(f"Error getting topic list: {e}")
    
    def handle_topic_list(self, result):
        """Handle topic list result."""
        if 'topics' in result and 'types' in result:
            # Create a list of (topic, type) tuples
            topic_info = list(zip(result['topics'], result['types']))
            self.topic_list_updated.emit(topic_info)
    
    def subscribe_to_topic(self, topic, msg_type, callback_id):
        """Subscribe to a ROS topic."""
        if not self.connected:
            return False
        
        # Create a unique key for this subscription
        sub_key = f"{topic}_{callback_id}"
        
        # Check if we're already subscribed
        if sub_key in self.subscriptions:
            return True
        
        try:
            # Create the subscriber
            subscriber = roslibpy.Topic(self.client, topic, msg_type)
            
            # Set up the callback
            def message_callback(message):
                self.message_received.emit(callback_id, message)
            
            subscriber.subscribe(message_callback)
            
            # Store the subscription
            self.subscriptions[sub_key] = subscriber
            return True
            
        except Exception as e:
            print(f"Error subscribing to {topic}: {e}")
            return False
    
    def unsubscribe_from_topic(self, topic, callback_id):
        """Unsubscribe from a ROS topic."""
        sub_key = f"{topic}_{callback_id}"
        
        if sub_key in self.subscriptions:
            try:
                self.subscriptions[sub_key].unsubscribe()
                del self.subscriptions[sub_key]
                return True
            except Exception as e:
                print(f"Error unsubscribing from {topic}: {e}")
        
        return False
    
    def get_system_stats(self):
        """Get system statistics."""
        if not self.connected:
            return
            
        try:
            # Create a simple service proxy for ROS system stats
            # This would typically call a custom service in your ROS system
            # For this example, we'll simulate system stats
            
            # In a real system, you would call something like:
            # service = roslibpy.Service(self.client, '/system/stats', 'std_srvs/Trigger')
            # request = roslibpy.ServiceRequest()
            # service.call(request, lambda result: self.handle_system_stats(result))
            
            # Simulated system stats
            stats = {
                'cpu_usage': min(100, 40 + 30 * math.sin(time.time() / 10)),
                'memory_usage': min(100, 50 + 20 * math.sin(time.time() / 15)),
                'disk_usage': min(100, 60 + 5 * math.sin(time.time() / 30)),
                'network': min(100, 30 + 40 * math.sin(time.time() / 8)),
                'timestamp': time.time()
            }
            self.system_status_updated.emit(stats)
            
        except Exception as e:
            print(f"Error getting system stats: {e}")


class TractobotsDashboard(QMainWindow):
    """Main dashboard window."""
    
    def __init__(self):
        super().__init__()
        self.wsl_runner = WslRunner()
        self.ros_worker = RosWorker()
        self.setup_ui()
        self.setup_signals()
        
        # Start the ROS worker thread
        self.ros_worker.start()
        
        # Set up auto-refresh timer
        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.check_ros_status)
        self.refresh_timer.start(5000)  # Check every 5 seconds
        
        # Initialize the system
        self.initialize_system()
    
    def setup_ui(self):
        """Set up the UI components."""
        # Set window properties
        self.setWindowTitle("Tractobots Professional Dashboard")
        self.setGeometry(100, 100, 1200, 800)
        self.setMinimumSize(800, 600)
        
        # Create central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Create header
        header_layout = QHBoxLayout()
        title_label = QLabel("TRACTOBOTS PROFESSIONAL DASHBOARD")
        title_label.setFont(QFont("Arial", 16, QFont.Bold))
        header_layout.addWidget(title_label)
        
        # Add status label
        self.status_label = QLabel("Initializing...")
        self.status_label.setFont(QFont("Arial", 10))
        header_layout.addStretch(1)
        header_layout.addWidget(self.status_label)
        main_layout.addLayout(header_layout)
        
        # Create horizontal split layout for main content
        content_layout = QHBoxLayout()
        
        # Create left panel for controls and stats
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        
        # Control panel
        control_group = QGroupBox("Control Panel")
        control_layout = QVBoxLayout(control_group)
        
        # Connection controls
        connection_layout = QHBoxLayout()
        self.connect_button = QPushButton("Connect to ROS")
        self.connect_button.clicked.connect(self.on_connect_clicked)
        connection_layout.addWidget(self.connect_button)
        
        self.rosbridge_button = QPushButton("Start Rosbridge")
        self.rosbridge_button.clicked.connect(self.on_rosbridge_clicked)
        connection_layout.addWidget(self.rosbridge_button)
        control_layout.addLayout(connection_layout)
        
        # Robot model controls
        model_layout = QHBoxLayout()
        self.start_model_button = QPushButton("Launch Robot Model")
        self.start_model_button.clicked.connect(self.on_start_model_clicked)
        model_layout.addWidget(self.start_model_button)
        
        self.stop_model_button = QPushButton("Stop Robot Model")
        self.stop_model_button.clicked.connect(self.on_stop_model_clicked)
        model_layout.addWidget(self.stop_model_button)
        control_layout.addLayout(model_layout)
        
        # Navigation controls
        nav_layout = QHBoxLayout()
        self.start_nav_button = QPushButton("Launch Navigation")
        self.start_nav_button.clicked.connect(self.on_start_nav_clicked)
        nav_layout.addWidget(self.start_nav_button)
        
        self.stop_nav_button = QPushButton("Stop Navigation")
        self.stop_nav_button.clicked.connect(self.on_stop_nav_clicked)
        nav_layout.addWidget(self.stop_nav_button)
        control_layout.addLayout(nav_layout)
        
        # System management
        system_layout = QHBoxLayout()
        self.restart_button = QPushButton("Restart ROS")
        self.restart_button.clicked.connect(self.on_restart_clicked)
        system_layout.addWidget(self.restart_button)
        
        self.stop_all_button = QPushButton("Stop All Processes")
        self.stop_all_button.clicked.connect(self.on_stop_all_clicked)
        self.stop_all_button.setStyleSheet("background-color: #ffcccc;")
        system_layout.addWidget(self.stop_all_button)
        control_layout.addLayout(system_layout)
        
        left_layout.addWidget(control_group)
        
        # System status panel with gauges
        status_group = QGroupBox("System Status")
        status_layout = QVBoxLayout(status_group)
        
        # Create a Figure for matplotlib
        self.figure = Figure(figsize=(5, 3), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setMinimumHeight(150)
        status_layout.addWidget(self.canvas)
        
        # Create the gauges
        self.axes = {}
        self.gauge_values = {
            'CPU': 0,
            'Memory': 0,
            'Disk': 0,
            'Network': 0
        }
        
        # Create a grid of 2x2 for the gauges
        gs = self.figure.add_gridspec(2, 2, hspace=0.4, wspace=0.3)
        positions = [(0, 0), (0, 1), (1, 0), (1, 1)]
        
        for idx, (name, value) in enumerate(self.gauge_values.items()):
            row, col = positions[idx]
            ax = self.figure.add_subplot(gs[row, col], projection='polar')
            self.axes[name] = ax
            self.setup_gauge(ax, name, value)
        
        self.figure.tight_layout(pad=1.5)
        
        # Add node list
        node_label = QLabel("ROS Nodes:")
        status_layout.addWidget(node_label)
        
        self.node_list = QTextEdit()
        self.node_list.setReadOnly(True)
        self.node_list.setMaximumHeight(100)
        status_layout.addWidget(self.node_list)
        
        left_layout.addWidget(status_group)
        
        # Tab widget for different tools
        self.tools_tabs = QTabWidget()
        
        # Terminal tab
        terminal_widget = QWidget()
        terminal_layout = QVBoxLayout(terminal_widget)
        
        # Add terminal output
        self.terminal_output = QTextEdit()
        self.terminal_output.setReadOnly(True)
        self.terminal_output.setFont(QFont("Consolas", 9))
        self.terminal_output.setStyleSheet("background-color: #1E1E1E; color: #DCDCDC;")
        terminal_layout.addWidget(self.terminal_output)
        
        # Add terminal input section
        input_layout = QHBoxLayout()
        self.terminal_input = QTextEdit()
        self.terminal_input.setMaximumHeight(60)
        self.terminal_input.setPlaceholderText("Enter WSL/ROS2 commands here...")
        self.terminal_input.setFont(QFont("Consolas", 9))
        input_layout.addWidget(self.terminal_input)
        
        self.execute_button = QPushButton("Execute")
        self.execute_button.clicked.connect(self.on_execute_command)
        input_layout.addWidget(self.execute_button)
        
        terminal_layout.addLayout(input_layout)
        
        # Add a keyboard shortcut (Ctrl+Enter) to execute command
        self.execute_shortcut = QShortcut(QKeySequence("Ctrl+Return"), self.terminal_input)
        self.execute_shortcut.activated.connect(self.on_execute_command)
        
        self.tools_tabs.addTab(terminal_widget, "Terminal")
        
        # Cheat codes tab (command reference)
        cheatsheet_widget = QWidget()
        cheatsheet_layout = QVBoxLayout(cheatsheet_widget)
        
        # Create tabs for different command categories
        command_tabs = QTabWidget()
        
        # ROS2 commands
        ros_commands = QTextEdit()
        ros_commands.setReadOnly(True)
        ros_commands.setHtml(self._get_ros2_commands_html())
        command_tabs.addTab(ros_commands, "ROS2")
        
        # Tractobots commands
        tractobots_commands = QTextEdit()
        tractobots_commands.setReadOnly(True)
        tractobots_commands.setHtml(self._get_tractobots_commands_html())
        command_tabs.addTab(tractobots_commands, "Tractobots")
        
        # System commands
        system_commands = QTextEdit()
        system_commands.setReadOnly(True)
        system_commands.setHtml(self._get_system_commands_html())
        command_tabs.addTab(system_commands, "System")
        
        cheatsheet_layout.addWidget(command_tabs)
        self.tools_tabs.addTab(cheatsheet_widget, "Command Reference")
        
        # Add the tools tabs to the left panel
        left_layout.addWidget(self.tools_tabs)
        
        # Right panel for visualization
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        
        # Topic browser
        topic_group = QGroupBox("ROS Topics")
        topic_layout = QVBoxLayout(topic_group)
        
        self.topic_list = QTextEdit()
        self.topic_list.setReadOnly(True)
        topic_layout.addWidget(self.topic_list)
        
        right_layout.addWidget(topic_group)
        
        # Visualization area
        viz_group = QGroupBox("Visualization")
        viz_layout = QVBoxLayout(viz_group)
        viz_label = QLabel("Robot visualization will appear here.")
        viz_layout.addWidget(viz_label)
        
        # Placeholder for future 3D visualization
        viz_placeholder = QWidget()
        viz_placeholder.setStyleSheet("background-color: #f0f0f0;")
        viz_placeholder.setMinimumHeight(300)
        viz_layout.addWidget(viz_placeholder)
        
        right_layout.addWidget(viz_group)
        
        # Set content layout
        content_layout.addWidget(left_panel, 3)  # Left panel takes 3 parts
        content_layout.addWidget(right_panel, 2)  # Right panel takes 2 parts
        main_layout.addLayout(content_layout)
        
        # Status bar
        self.statusBar().showMessage("System initializing...")
        
        # Initialize UI state
        self.update_ui_state(False)
    
    def setup_gauge(self, ax, name, value):
        """Set up a gauge on the given axis."""
        # Normalize the value to be between 0 and 100
        norm_value = min(max(value, 0), 100) / 100.0
        
        # Create the gauge
        ax.set_theta_zero_location('N')  # 0 at the top
        ax.set_theta_direction(-1)  # clockwise
        
        # Remove yticks
        ax.set_yticks([])
        
        # Set the limits
        ax.set_xlim(0, 2 * np.pi)
        ax.set_ylim(0, 1)
        
        # Create the colored bands
        theta = np.linspace(0, 2 * np.pi, 100)
        radii = np.zeros_like(theta) + 0.85
        
        # Background (gray)
        ax.bar(theta, radii, width=theta[1]-theta[0], bottom=0.15, 
               color='#EEEEEE', edgecolor='none', alpha=0.8)
        
        # Value display (using a colored partial ring)
        value_theta = np.linspace(0, 2 * np.pi * norm_value, 100)
        
        # Choose color based on value
        if norm_value < 0.6:  # Less than 60%
            color = 'green'
        elif norm_value < 0.8:  # Between 60% and 80%
            color = 'orange'
        else:  # More than 80%
            color = 'red'
            
        if len(value_theta) > 0:  # Make sure we have points to plot
            ax.bar(value_theta, radii[:len(value_theta)], 
                   width=theta[1]-theta[0], bottom=0.15, 
                   color=color, edgecolor='none', alpha=0.8)
        
        # Add a needle
        needle_theta = 2 * np.pi * norm_value
        ax.plot([0, needle_theta], [0, 0.7], 'k-', linewidth=2)
        
        # Add a center dot
        ax.scatter(0, 0, s=80, color='black', zorder=3)
        
        # Add label and value
        ax.text(0, -0.2, f"{name}", ha='center', va='center', 
                fontsize=9, fontweight='bold')
        ax.text(0, -0.1, f"{int(value)}%", ha='center', va='center', 
                fontsize=10, fontweight='bold')
    
    def update_gauge(self, name, value):
        """Update a gauge with a new value."""
        if name in self.axes and name in self.gauge_values:
            # Update stored value
            self.gauge_values[name] = value
            
            # Clear the axis
            self.axes[name].clear()
            
            # Redraw the gauge
            self.setup_gauge(self.axes[name], name, value)
    
    def update_gauges(self):
        """Update all gauges with current values."""
        for name, value in self.gauge_values.items():
            self.update_gauge(name, value)
        
        # Redraw canvas
        self.canvas.draw_idle()
    
    def setup_signals(self):
        """Set up signal connections."""
        # WSL runner signals
        self.wsl_runner.log_message.connect(self.log_text_edit)
        self.wsl_runner.process_finished.connect(self.on_process_finished)
        
        # ROS worker signals
        self.ros_worker.connection_status.connect(self.on_connection_status)
        self.ros_worker.node_status_updated.connect(self.on_node_status_updated)
        self.ros_worker.topic_list_updated.connect(self.on_topic_list_updated)
        self.ros_worker.system_status_updated.connect(self.on_system_status_updated)
        self.ros_worker.message_received.connect(self.on_message_received)
    
    def log_text_edit(self, text):
        """Add text to the terminal output."""
        self.terminal_output.append(text)
        
        # Auto-scroll to bottom
        cursor = self.terminal_output.textCursor()
        cursor.movePosition(cursor.End)
        self.terminal_output.setTextCursor(cursor)
    
    def initialize_system(self):
        """Initialize the dashboard system."""
        self.log_text_edit("---- TRACTOBOTS PROFESSIONAL DASHBOARD ----")
        self.log_text_edit(f"Initializing at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        self.log_text_edit("Checking WSL availability...")
        
        # Test WSL availability
        threading.Thread(target=self._test_wsl, daemon=True).start()
    
    def _test_wsl(self):
        """Test if WSL is available."""
        try:
            result = subprocess.run(
                ["wsl", "echo", "WSL Test Successful"], 
                capture_output=True, text=True, timeout=5
            )
            if result.returncode == 0:
                self.log_text_edit("✓ WSL is available")
                
                # Check for ROS2 installation
                self.log_text_edit("Checking ROS2 installation...")
                cmd_id = "check_ros"
                self.wsl_runner.run_command(cmd_id, "ls /opt/ros/jazzy &>/dev/null && echo '✓ ROS2 Jazzy installed' || echo '✗ ROS2 Jazzy not found'")
            else:
                self.log_text_edit("✗ WSL test failed. Please ensure WSL is installed and working.")
                self.update_ui_state(False)
        except Exception as e:
            self.log_text_edit(f"✗ Error testing WSL: {str(e)}")
            self.update_ui_state(False)
    
    def update_ui_state(self, connected):
        """Update UI state based on connection status."""
        # Update buttons based on connection state
        self.connect_button.setText("Reconnect" if connected else "Connect to ROS")
        self.connect_button.setEnabled(True)
        
        # System action buttons need connection
        self.start_model_button.setEnabled(connected)
        self.stop_model_button.setEnabled(connected)
        self.start_nav_button.setEnabled(connected)
        self.stop_nav_button.setEnabled(connected)
        self.restart_button.setEnabled(connected)
        
        # Stop all is always available if WSL is
        self.stop_all_button.setEnabled(True)
    
    def check_ros_status(self):
        """Periodically check ROS status."""
        if not self.ros_worker.connected:
            self.statusBar().showMessage("Disconnected from ROS")
    
    # ---- Button Click Handlers ----
    
    def on_connect_clicked(self):
        """Handle connect button click."""
        if not self.ros_worker.connected:
            self.log_text_edit("Connecting to ROS...")
            self.ros_worker.wsl_ip = None  # Reset IP to trigger re-detection
        else:
            self.log_text_edit("Reconnecting to ROS...")
            self.ros_worker.stop()
            self.ros_worker = RosWorker()
            self.setup_signals()
            self.ros_worker.start()
    
    def on_rosbridge_clicked(self):
        """Handle start rosbridge button click."""
        self.log_text_edit("Starting rosbridge server...")
        cmd_id = "start_rosbridge"
        self.wsl_runner.run_command(
            cmd_id, 
            "ros2 launch rosbridge_server rosbridge_websocket_launch.xml & echo 'Rosbridge started'"
        )
        self.rosbridge_button.setEnabled(False)
    
    def on_start_model_clicked(self):
        """Handle start robot model button click."""
        self.log_text_edit("Launching robot model...")
        cmd_id = "start_model"
        
        # Use a single line command to avoid bash syntax issues
        self.wsl_runner.run_command(
            cmd_id,
            "source /opt/ros/jazzy/setup.bash && ros2 launch tractobots_launchers model.launch.py"
        )
    
    def on_stop_model_clicked(self):
        """Handle stop robot model button click."""
        self.log_text_edit("Stopping robot model...")
        cmd_id = "stop_model"
        self.wsl_runner.run_command(
            cmd_id,
            "pkill -f 'ros2 launch tractobots_launchers model.launch.py' || echo 'No model running'"
        )
    
    def on_start_nav_clicked(self):
        """Handle start navigation button click."""
        self.log_text_edit("Launching navigation...")
        cmd_id = "start_nav"
        self.wsl_runner.run_command(
            cmd_id,
            "source /opt/ros/jazzy/setup.bash && ros2 launch tractobots_nav2 navigation.launch.py"
        )
    
    def on_stop_nav_clicked(self):
        """Handle stop navigation button click."""
        self.log_text_edit("Stopping navigation...")
        cmd_id = "stop_nav"
        self.wsl_runner.run_command(
            cmd_id,
            "pkill -f 'ros2 launch tractobots_nav2 navigation.launch.py' || echo 'No navigation running'"
        )
    
    def on_restart_clicked(self):
        """Handle restart ROS button click."""
        self.log_text_edit("Restarting ROS services...")
        cmd_id = "restart_ros"
        self.wsl_runner.run_command(
            cmd_id,
            "sudo systemctl restart ros2 && echo 'ROS services restarted'"
        )
    
    def on_stop_all_clicked(self):
        """Handle stop all processes button click."""
        self.log_text_edit("Stopping all ROS processes...")
        cmd_id = "stop_all"
        
        # Use a proper command with logical AND operators
        self.wsl_runner.run_command(
            cmd_id,
            "pkill -f 'ros2 launch' && echo 'Stopped all launch processes' && pkill -f 'gazebo' && echo 'Stopped Gazebo' && pkill -f 'rviz' && echo 'Stopped RViz'"
        )
    
    def on_execute_command(self):
        """Execute the command entered in the terminal input."""
        command = self.terminal_input.toPlainText().strip()
        if not command:
            return
            
        self.log_text_edit(f"\n> {command}")
        cmd_id = f"custom_{int(time.time())}"
        self.wsl_runner.run_command(cmd_id, command)
        
        # Clear input field
        self.terminal_input.clear()
    
    # ---- Signal Handlers ----
    
    def on_connection_status(self, connected, message):
        """Handle connection status updates."""
        if connected:
            self.status_label.setText("Connected to ROS")
            self.statusBar().showMessage(message)
            self.log_text_edit(f"✓ {message}")
        else:
            self.status_label.setText("Disconnected from ROS")
            self.statusBar().showMessage(message)
            self.log_text_edit(f"⚠️ {message}")
        
        self.update_ui_state(connected)
    
    def on_node_status_updated(self, nodes):
        """Handle node status updates."""
        if nodes:
            self.node_list.clear()
            for node in sorted(nodes):
                self.node_list.append(f"• {node}")
    
    def on_topic_list_updated(self, topics):
        """Handle topic list updates."""
        if topics:
            self.topic_list.clear()
            for topic, topic_type in sorted(topics, key=lambda x: x[0]):
                self.topic_list.append(f"<b>{topic}</b> [{topic_type}]")
    
    def on_system_status_updated(self, stats):
        """Handle system status updates."""
        if 'cpu_usage' in stats:
            self.gauge_values['CPU'] = stats['cpu_usage']
        
        if 'memory_usage' in stats:
            self.gauge_values['Memory'] = stats['memory_usage']
        
        if 'disk_usage' in stats:
            self.gauge_values['Disk'] = stats['disk_usage']
        
        if 'network' in stats:
            self.gauge_values['Network'] = stats['network']
        
        self.update_gauges()
    
    def on_message_received(self, callback_id, message):
        """Handle message received from ROS topics."""
        # This method would process messages from subscribed topics
        # Currently not used directly
        pass
    
    def on_process_finished(self, command_id, return_code, output):
        """Handle process finished signal."""
        if return_code == 0:
            status = "✓ Command completed successfully"
        else:
            status = f"⚠️ Command failed with return code {return_code}"
        
        self.log_text_edit(f"{status}")
        
        # Special handling for specific commands
        if command_id == "check_ros" and return_code != 0:
            self.log_text_edit("ROS2 Jazzy not found. Please install ROS2.")
        elif command_id == "start_rosbridge":
            # Re-enable the rosbridge button after a delay
            QTimer.singleShot(5000, lambda: self.rosbridge_button.setEnabled(True))
    
    # ---- Helper methods ----
    
    def _get_ros2_commands_html(self):
        """Generate HTML for ROS2 commands cheat sheet."""
        return """
        <style>
            table { border-collapse: collapse; width: 100%; }
            th, td { padding: 8px; text-align: left; border-bottom: 1px solid #ddd; }
            th { background-color: #4CAF50; color: white; }
            tr:hover { background-color: #f5f5f5; }
            .command { font-family: monospace; font-weight: bold; color: #0066cc; }
        </style>
        <h3>ROS2 Basic Commands</h3>
        <table>
            <tr><th>Command</th><th>Description</th></tr>
            <tr><td class="command">ros2 node list</td><td>List active ROS2 nodes</td></tr>
            <tr><td class="command">ros2 topic list</td><td>List active ROS2 topics</td></tr>
            <tr><td class="command">ros2 topic echo /topic_name</td><td>Display messages from a topic</td></tr>
            <tr><td class="command">ros2 param list</td><td>List parameters from all nodes</td></tr>
            <tr><td class="command">ros2 service list</td><td>List active ROS2 services</td></tr>
            <tr><td class="command">ros2 launch pkg_name file.launch.py</td><td>Launch nodes from a launch file</td></tr>
        </table>
        
        <h3>ROS2 Node Management</h3>
        <table>
            <tr><th>Command</th><th>Description</th></tr>
            <tr><td class="command">ros2 run pkg_name node_name</td><td>Run a ROS2 node</td></tr>
            <tr><td class="command">ros2 node info /node_name</td><td>Display node information</td></tr>
            <tr><td class="command">ros2 topic info /topic_name</td><td>Display topic information</td></tr>
            <tr><td class="command">ros2 interface show msg_type</td><td>Display message interface</td></tr>
        </table>
        """
    
    def _get_tractobots_commands_html(self):
        """Generate HTML for Tractobots commands cheat sheet."""
        return """
        <style>
            table { border-collapse: collapse; width: 100%; }
            th, td { padding: 8px; text-align: left; border-bottom: 1px solid #ddd; }
            th { background-color: #FF8C00; color: white; }
            tr:hover { background-color: #f5f5f5; }
            .command { font-family: monospace; font-weight: bold; color: #CC6600; }
        </style>
        <h3>Tractobots Launch Commands</h3>
        <table>
            <tr><th>Command</th><th>Description</th></tr>
            <tr><td class="command">ros2 launch tractobots_launchers model.launch.py</td><td>Launch the tractor model in Gazebo</td></tr>
            <tr><td class="command">ros2 launch tractobots_nav2 navigation.launch.py</td><td>Launch the navigation stack</td></tr>
            <tr><td class="command">ros2 launch tractobots_robot_localization ekf.launch.py</td><td>Launch the EKF for localization</td></tr>
            <tr><td class="command">ros2 launch tractobots_mission_ui web_ui.launch.py</td><td>Launch the web-based mission UI</td></tr>
        </table>
        
        <h3>Tractobots Topics</h3>
        <table>
            <tr><th>Topic</th><th>Description</th></tr>
            <tr><td class="command">/tractobots/cmd_vel</td><td>Velocity commands for the tractor</td></tr>
            <tr><td class="command">/tractobots/odom</td><td>Odometry data</td></tr>
            <tr><td class="command">/tractobots/gps/fix</td><td>GPS position data</td></tr>
            <tr><td class="command">/tractobots/imu/data</td><td>IMU sensor data</td></tr>
            <tr><td class="command">/tractobots/mission/status</td><td>Mission status updates</td></tr>
        </table>
        """
    
    def _get_system_commands_html(self):
        """Generate HTML for System commands cheat sheet."""
        return """
        <style>
            table { border-collapse: collapse; width: 100%; }
            th, td { padding: 8px; text-align: left; border-bottom: 1px solid #ddd; }
            th { background-color: #6495ED; color: white; }
            tr:hover { background-color: #f5f5f5; }
            .command { font-family: monospace; font-weight: bold; color: #3366CC; }
        </style>
        <h3>WSL Commands</h3>
        <table>
            <tr><th>Command</th><th>Description</th></tr>
            <tr><td class="command">wsl --status</td><td>Check WSL status</td></tr>
            <tr><td class="command">wsl --shutdown</td><td>Shut down all WSL instances</td></tr>
            <tr><td class="command">wsl --list --verbose</td><td>List installed WSL distributions</td></tr>
            <tr><td class="command">wsl --terminate Ubuntu</td><td>Terminate the Ubuntu WSL instance</td></tr>
        </table>
        
        <h3>System Management</h3>
        <table>
            <tr><th>Command</th><th>Description</th></tr>
            <tr><td class="command">pkill -f 'ros2'</td><td>Kill all ros2 processes</td></tr>
            <tr><td class="command">pkill -f 'gazebo'</td><td>Kill all Gazebo processes</td></tr>
            <tr><td class="command">pkill -f 'rviz'</td><td>Kill all RViz processes</td></tr>
            <tr><td class="command">top</td><td>Monitor system processes</td></tr>
            <tr><td class="command">htop</td><td>Interactive process monitor</td></tr>
            <tr><td class="command">ip addr</td><td>Show network interfaces</td></tr>
        </table>
        """
    
    def closeEvent(self, event):
        """Handle window close event."""
        self.log_text_edit("Shutting down dashboard...")
        
        # Stop the ROS worker thread
        if self.ros_worker:
            self.ros_worker.stop()
        
        # Stop any running processes
        cmd_id = "shutdown"
        try:
            subprocess.run(
                ["wsl", "pkill", "-f", "ros2"],
                capture_output=True, timeout=2
            )
        except:
            pass
        
        event.accept()


# Main entry point
if __name__ == "__main__":
    # Create the application
    app = QApplication(sys.argv)
    
    # Create and display the dashboard
    dashboard = TractobotsDashboard()
    dashboard.show()
    
    # Start the event loop
    sys.exit(app.exec_())
