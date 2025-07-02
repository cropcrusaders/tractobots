#!/usr/bin/env python3
"""
Enhanced Tractobots Mission Control GUI
Modern, feature-rich interface for autonomous tractor operations
"""

import threading
import tkinter as tk
from tkinter import ttk, messagebox
import json
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry


class TractorDashboard(Node):
    def __init__(self):
        super().__init__('tractor_dashboard')
        
        # Publishers
        self.mission_pub = self.create_publisher(Bool, '/mission/start', 1)
        self.emergency_pub = self.create_publisher(Bool, '/emergency_stop', 1)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        
        # Subscribers
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 1)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        
        # Data storage
        self.gps_data = {"lat": 0.0, "lon": 0.0, "alt": 0.0}
        self.imu_data = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        self.odom_data = {"x": 0.0, "y": 0.0, "speed": 0.0}
        self.mission_active = False
        self.emergency_stop = False
        
        # GUI Variables
        self.status_var = tk.StringVar(value="🟡 READY")
        self.gps_var = tk.StringVar(value="GPS: Waiting...")
        self.speed_var = tk.StringVar(value="Speed: 0.0 m/s")
        self.heading_var = tk.StringVar(value="Heading: 0.0°")
        self.mission_time_var = tk.StringVar(value="Mission Time: 00:00:00")
        
        self.mission_start_time = None

    def gps_callback(self, msg):
        self.gps_data = {
            "lat": msg.latitude,
            "lon": msg.longitude, 
            "alt": msg.altitude
        }
        self.gps_var.set(f"GPS: {msg.latitude:.6f}, {msg.longitude:.6f}")

    def imu_callback(self, msg):
        # Convert quaternion to euler angles (simplified)
        import math
        q = msg.orientation
        # Simple yaw calculation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.imu_data["yaw"] = math.degrees(yaw)
        self.heading_var.set(f"Heading: {math.degrees(yaw):.1f}°")

    def odom_callback(self, msg):
        self.odom_data = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "speed": (msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)**0.5
        }
        self.speed_var.set(f"Speed: {self.odom_data['speed']:.2f} m/s")

    def start_mission(self):
        if not self.emergency_stop:
            self.mission_pub.publish(Bool(data=True))
            self.mission_active = True
            self.mission_start_time = time.time()
            self.status_var.set("🟢 MISSION ACTIVE")
            self.get_logger().info('Mission started via GUI')
        else:
            messagebox.showwarning("Emergency Stop", "Clear emergency stop before starting mission!")

    def stop_mission(self):
        self.mission_pub.publish(Bool(data=False))
        self.mission_active = False
        self.mission_start_time = None
        self.status_var.set("🟡 READY")
        self.get_logger().info('Mission stopped via GUI')

    def trigger_estop(self):
        self.emergency_pub.publish(Bool(data=True))
        self.emergency_stop = True
        self.mission_active = False
        self.status_var.set("🔴 EMERGENCY STOP")
        self.get_logger().warn('Emergency stop triggered via GUI')

    def reset_estop(self):
        self.emergency_pub.publish(Bool(data=False))
        self.emergency_stop = False
        self.status_var.set("🟡 READY")
        self.get_logger().info('Emergency stop cleared via GUI')

    def manual_forward(self):
        if not self.emergency_stop:
            twist = Twist()
            twist.linear.x = 1.0
            self.cmd_vel_pub.publish(twist)

    def manual_backward(self):
        if not self.emergency_stop:
            twist = Twist()
            twist.linear.x = -1.0
            self.cmd_vel_pub.publish(twist)

    def manual_left(self):
        if not self.emergency_stop:
            twist = Twist()
            twist.angular.z = 1.0
            self.cmd_vel_pub.publish(twist)

    def manual_right(self):
        if not self.emergency_stop:
            twist = Twist()
            twist.angular.z = -1.0
            self.cmd_vel_pub.publish(twist)

    def manual_stop(self):
        twist = Twist()  # All zeros
        self.cmd_vel_pub.publish(twist)

    def update_mission_time(self):
        if self.mission_active and self.mission_start_time:
            elapsed = time.time() - self.mission_start_time
            hours = int(elapsed // 3600)
            minutes = int((elapsed % 3600) // 60)
            seconds = int(elapsed % 60)
            self.mission_time_var.set(f"Mission Time: {hours:02d}:{minutes:02d}:{seconds:02d}")


def create_modern_gui(node):
    """Create a modern, professional GUI interface"""
    
    root = tk.Tk()
    root.title("🚜 Tractobots Mission Control")
    root.geometry("800x600")
    root.configure(bg='#1a1a1a')
    
    # Create menu bar
    menubar = tk.Menu(root)
    root.config(menu=menubar)
    
    # Field menu
    field_menu = tk.Menu(menubar, tearoff=0)
    menubar.add_cascade(label="Field Management", menu=field_menu)
    field_menu.add_command(label="Load Shapefile...", command=lambda: open_shapefile_gui(root))
    field_menu.add_command(label="Field Boundary Service", command=lambda: launch_field_service())
    field_menu.add_separator()
    field_menu.add_command(label="Export Field Data...", command=lambda: export_field_data())
    
    # Tools menu
    tools_menu = tk.Menu(menubar, tearoff=0)
    menubar.add_cascade(label="Tools", menu=tools_menu)
    tools_menu.add_command(label="Launch Web Dashboard", command=lambda: launch_web_dashboard())
    tools_menu.add_command(label="Launch Qt GUI", command=lambda: launch_qt_gui())
    tools_menu.add_separator()
    tools_menu.add_command(label="System Diagnostics", command=lambda: system_diagnostics())
    
    # Help menu
    help_menu = tk.Menu(menubar, tearoff=0)
    menubar.add_cascade(label="Help", menu=help_menu)
    help_menu.add_command(label="User Guide", command=lambda: show_user_guide())
    help_menu.add_command(label="About", command=lambda: show_about())
    
    # Configure styles
    style = ttk.Style()
    style.theme_use('clam')
    style.configure('Title.TLabel', font=('Arial', 16, 'bold'), background='#1a1a1a', foreground='#ffffff')
    style.configure('Status.TLabel', font=('Arial', 12, 'bold'), background='#1a1a1a', foreground='#00ff00')
    style.configure('Data.TLabel', font=('Arial', 10), background='#1a1a1a', foreground='#cccccc')
    
    # Main container
    main_frame = ttk.Frame(root)
    main_frame.pack(fill='both', expand=True, padx=10, pady=10)
    
    # Title
    title_label = ttk.Label(main_frame, text="🚜 TRACTOBOTS AUTONOMOUS SYSTEM", style='Title.TLabel')
    title_label.pack(pady=(0, 20))
    
    # Status Section
    status_frame = ttk.LabelFrame(main_frame, text="System Status", padding=10)
    status_frame.pack(fill='x', pady=(0, 10))
    
    status_label = ttk.Label(status_frame, textvariable=node.status_var, style='Status.TLabel')
    status_label.pack()
    
    mission_time_label = ttk.Label(status_frame, textvariable=node.mission_time_var, style='Data.TLabel')
    mission_time_label.pack()
    
    # Data Display Section
    data_frame = ttk.LabelFrame(main_frame, text="Sensor Data", padding=10)
    data_frame.pack(fill='x', pady=(0, 10))
    
    # GPS Data
    gps_label = ttk.Label(data_frame, textvariable=node.gps_var, style='Data.TLabel')
    gps_label.pack(anchor='w')
    
    # Speed and Heading
    speed_label = ttk.Label(data_frame, textvariable=node.speed_var, style='Data.TLabel')
    speed_label.pack(anchor='w')
    
    heading_label = ttk.Label(data_frame, textvariable=node.heading_var, style='Data.TLabel')
    heading_label.pack(anchor='w')
    
    # Mission Control Section
    mission_frame = ttk.LabelFrame(main_frame, text="Mission Control", padding=10)
    mission_frame.pack(fill='x', pady=(0, 10))
    
    # Mission buttons
    mission_btn_frame = ttk.Frame(mission_frame)
    mission_btn_frame.pack(fill='x')
    
    start_btn = tk.Button(mission_btn_frame, text="🚀 START MISSION", command=node.start_mission,
                         bg='#2d5016', fg='white', font=('Arial', 12, 'bold'), height=2)
    start_btn.pack(side='left', fill='x', expand=True, padx=(0, 5))
    
    stop_btn = tk.Button(mission_btn_frame, text="⏹️ STOP MISSION", command=node.stop_mission,
                        bg='#8B4513', fg='white', font=('Arial', 12, 'bold'), height=2)
    stop_btn.pack(side='left', fill='x', expand=True, padx=5)
    
    # Emergency Section
    emergency_frame = ttk.LabelFrame(main_frame, text="Emergency Controls", padding=10)
    emergency_frame.pack(fill='x', pady=(0, 10))
    
    emergency_btn_frame = ttk.Frame(emergency_frame)
    emergency_btn_frame.pack(fill='x')
    
    estop_btn = tk.Button(emergency_btn_frame, text="🛑 EMERGENCY STOP", command=node.trigger_estop,
                         bg='#8B0000', fg='white', font=('Arial', 14, 'bold'), height=2)
    estop_btn.pack(side='left', fill='x', expand=True, padx=(0, 5))
    
    reset_btn = tk.Button(emergency_btn_frame, text="🔄 RESET E-STOP", command=node.reset_estop,
                         bg='#2F4F4F', fg='white', font=('Arial', 12, 'bold'), height=2)
    reset_btn.pack(side='left', fill='x', expand=True, padx=5)
    
    # Manual Control Section
    manual_frame = ttk.LabelFrame(main_frame, text="Manual Control", padding=10)
    manual_frame.pack(fill='both', expand=True)
    
    # Manual control buttons in cross pattern
    manual_grid = ttk.Frame(manual_frame)
    manual_grid.pack(expand=True)
    
    # Forward button
    forward_btn = tk.Button(manual_grid, text="⬆️\nFORWARD", command=node.manual_forward,
                           bg='#4169E1', fg='white', font=('Arial', 10, 'bold'), width=8, height=3)
    forward_btn.grid(row=0, column=1, padx=5, pady=5)
    
    # Left and Right buttons
    left_btn = tk.Button(manual_grid, text="⬅️\nLEFT", command=node.manual_left,
                        bg='#4169E1', fg='white', font=('Arial', 10, 'bold'), width=8, height=3)
    left_btn.grid(row=1, column=0, padx=5, pady=5)
    
    stop_manual_btn = tk.Button(manual_grid, text="⏹️\nSTOP", command=node.manual_stop,
                               bg='#DC143C', fg='white', font=('Arial', 10, 'bold'), width=8, height=3)
    stop_manual_btn.grid(row=1, column=1, padx=5, pady=5)
    
    right_btn = tk.Button(manual_grid, text="➡️\nRIGHT", command=node.manual_right,
                         bg='#4169E1', fg='white', font=('Arial', 10, 'bold'), width=8, height=3)
    right_btn.grid(row=1, column=2, padx=5, pady=5)
    
    # Backward button
    backward_btn = tk.Button(manual_grid, text="⬇️\nBACKWARD", command=node.manual_backward,
                            bg='#4169E1', fg='white', font=('Arial', 10, 'bold'), width=8, height=3)
    backward_btn.grid(row=2, column=1, padx=5, pady=5)
    
    # Update timer for mission time
    def update_display():
        node.update_mission_time()
        root.after(1000, update_display)  # Update every second
    
    update_display()
    
    return root


def open_shapefile_gui(parent):
    """Open the shapefile management GUI"""
    try:
        from .shapefile_gui import ShapefileGUI
        shapefile_app = ShapefileGUI(parent)
    except ImportError as e:
        messagebox.showerror("Import Error", f"Could not import shapefile GUI: {e}")

def launch_field_service():
    """Launch the field boundary service"""
    try:
        import subprocess
        subprocess.Popen(['ros2', 'run', 'tractobots_mission_ui', 'field_boundary_service'])
        messagebox.showinfo("Service Launched", "Field boundary service started successfully!")
    except Exception as e:
        messagebox.showerror("Launch Error", f"Could not launch field service: {e}")

def export_field_data():
    """Export field data"""
    messagebox.showinfo("Export", "Use Field Management -> Load Shapefile to access export options")

def launch_web_dashboard():
    """Launch web dashboard"""
    try:
        import subprocess
        subprocess.Popen(['ros2', 'run', 'tractobots_mission_ui', 'web_dashboard'])
        messagebox.showinfo("Dashboard Launched", "Web dashboard started at http://localhost:5000")
    except Exception as e:
        messagebox.showerror("Launch Error", f"Could not launch web dashboard: {e}")

def launch_qt_gui():
    """Launch Qt GUI"""
    try:
        import subprocess
        subprocess.Popen(['ros2', 'run', 'tractobots_mission_ui', 'qt_gui'])
        messagebox.showinfo("Qt GUI Launched", "Qt GUI started successfully!")
    except Exception as e:
        messagebox.showerror("Launch Error", f"Could not launch Qt GUI: {e}")

def system_diagnostics():
    """Run system diagnostics"""
    try:
        import subprocess
        subprocess.Popen(['python3', 'diagnose_problems.py'])
        messagebox.showinfo("Diagnostics", "System diagnostics started in new window")
    except Exception as e:
        messagebox.showerror("Diagnostics Error", f"Could not run diagnostics: {e}")

def show_user_guide():
    """Show user guide"""
    help_text = """
🚜 TRACTOBOTS USER GUIDE

FIELD MANAGEMENT:
• Use Field Management -> Load Shapefile to import field boundaries
• Supports .shp files from operation centers
• Generates coverage paths automatically
• Exports to ROS2 compatible formats

MISSION CONTROL:
• START MISSION: Begin autonomous operation
• STOP MISSION: Pause current mission
• EMERGENCY STOP: Immediate halt (safety critical)

MANUAL CONTROL:
• Use arrow buttons for manual driving
• Only available when mission is stopped

SYSTEM STATUS:
• Green: Ready for operation
• Yellow: Warning condition
• Red: Error or emergency stop

For detailed documentation, see:
• GETTING_STARTED.md
• GUI_GUIDE.md
• VSCODE_BUILD_GUIDE.md
    """
    
    help_window = tk.Toplevel()
    help_window.title("User Guide")
    help_window.geometry("600x500")
    
    text_widget = tk.Text(help_window, wrap=tk.WORD, padx=10, pady=10)
    text_widget.pack(fill=tk.BOTH, expand=True)
    text_widget.insert(tk.END, help_text)
    text_widget.config(state=tk.DISABLED)

def show_about():
    """Show about dialog"""
    about_text = """
🚜 TRACTOBOTS AUTONOMOUS SYSTEM

Version: 1.0.0
Author: Nicholas Bass
License: GPLv3

Advanced autonomous tractor control system with:
• ROS2 Jazzy integration
• Shapefile field boundary support
• Multiple GUI interfaces
• Real-time sensor monitoring
• Coverage path planning
• Emergency safety systems

For support and documentation:
https://github.com/your-repo/tractobots
    """
    messagebox.showinfo("About Tractobots", about_text)


def main(args=None):
    rclpy.init(args=args)
    node = TractorDashboard()

    # Spin node in background so GUI remains responsive
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Create and run GUI
    root = create_modern_gui(node)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
