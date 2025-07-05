#!/usr/bin/env python3
"""
Tractobots Real-Time Dashboard
A comprehensive graphical interface to monitor and control all Tractobots systems
"""

import sys
import os
import subprocess
import threading
import time
import json
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import numpy as np

# Add paths for imports
sys.path.insert(0, "src/tractobots_mission_ui/tractobots_mission_ui")

class TractobotsDashboard:
    def __init__(self, root):
        self.root = root
        self.root.title("🚜 Tractobots Real-Time Dashboard")
        self.root.geometry("1600x1000")
        self.root.configure(bg='#1a1a2e')
        
        # Enhanced status tracking
        self.system_status = {
            'ros2': False,
            'gazebo': False,
            'navigation': False,
            'gps': False,
            'simulation': False,
            'camera': False,
            'lidar': False,
            'imu': False,
            'can_bus': False,
            'hydraulics': False
        }
        
        # Enhanced data storage
        self.field_data = []
        self.gps_data = []
        self.tractor_position = [0, 0, 0]
        self.waypoints = []
        self.running_processes = {}
        self.system_metrics = {
            'cpu_usage': 0,
            'memory_usage': 0,
            'disk_usage': 0,
            'network_activity': 0,
            'temperature': 0
        }
        self.environmental_data = {
            'temperature': 22.0,
            'humidity': 65.0,
            'wind_speed': 2.5,
            'soil_moisture': 45.0,
            'light_level': 85.0
        }
        self.alerts = []
        
        # Animation control
        self.animation_active = False
        self.update_counter = 0
        
        self.create_widgets()
        self.start_monitoring()
        
    def create_widgets(self):
        """Create the main dashboard interface with enhanced sidebar"""
        
        # Main container with sidebar layout
        main_container = tk.Frame(self.root, bg='#1a1a2e')
        main_container.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Create sidebar
        self.create_sidebar(main_container)
        
        # Create main content area
        content_frame = tk.Frame(main_container, bg='#1a1a2e')
        content_frame.pack(side='right', fill='both', expand=True, padx=(5, 0))
        
        # Main title
        self.create_title_section(content_frame)
        
        # Create notebook for tabs
        self.notebook = ttk.Notebook(content_frame)
        self.notebook.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Style configuration
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TNotebook', background='#1a1a2e')
        style.configure('TNotebook.Tab', background='#34495e', foreground='#ecf0f1')
        style.configure('TFrame', background='#1a1a2e')
        
        # Create tabs
        self.create_overview_tab()
        self.create_field_visualization_tab()
        self.create_system_control_tab()
        self.create_simulation_tab()
        self.create_logs_tab()
        
    def create_sidebar(self, parent):
        """Create the enhanced sidebar with system status and controls"""
        sidebar = tk.Frame(parent, bg='#16213e', width=350)
        sidebar.pack(side='left', fill='y', padx=(0, 5))
        sidebar.pack_propagate(False)
        
        # Sidebar title
        sidebar_title = tk.Label(
            sidebar,
            text="🚜 TRACTOBOTS CONTROL",
            font=('Arial', 16, 'bold'),
            fg='#3498db',
            bg='#16213e'
        )
        sidebar_title.pack(pady=(10, 5))
        
        # Current time display
        self.time_label = tk.Label(
            sidebar,
            text="",
            font=('Arial', 10),
            fg='#ecf0f1',
            bg='#16213e'
        )
        self.time_label.pack(pady=(0, 10))
        
        # System Status Section
        self.create_system_status_section(sidebar)
        
        # Quick Controls Section
        self.create_quick_controls_section(sidebar)
        
        # System Resources Section
        self.create_system_resources_section(sidebar)
        
        # Environmental Data Section
        self.create_environmental_section(sidebar)
        
        # Alerts Section
        self.create_alerts_section(sidebar)
        
    def create_system_status_section(self, parent):
        """Create system status display in sidebar"""
        status_frame = tk.LabelFrame(
            parent,
            text="🔧 System Status",
            font=('Arial', 11, 'bold'),
            fg='#3498db',
            bg='#16213e',
            labelanchor='n'
        )
        status_frame.pack(fill='x', padx=10, pady=5)
        
        self.status_labels = {}
        for system in self.system_status:
            frame = tk.Frame(status_frame, bg='#16213e')
            frame.pack(fill='x', padx=5, pady=2)
            
            status_label = tk.Label(
                frame,
                text=f"🔴 {system.upper().replace('_', ' ')}",
                font=('Arial', 9),
                fg='#e74c3c',
                bg='#16213e',
                anchor='w'
            )
            status_label.pack(side='left')
            
            self.status_labels[system] = status_label
            
    def create_quick_controls_section(self, parent):
        """Create quick control buttons in sidebar"""
        controls_frame = tk.LabelFrame(
            parent,
            text="🎮 Quick Controls",
            font=('Arial', 11, 'bold'),
            fg='#3498db',
            bg='#16213e',
            labelanchor='n'
        )
        controls_frame.pack(fill='x', padx=10, pady=5)
        
        # Control buttons
        buttons = [
            ("🚀 Start ROS2", self.start_ros2_system, '#27ae60'),
            ("🌍 Launch Gazebo", self.start_gazebo_simulation, '#3498db'),
            ("🧭 Start Navigation", self.start_navigation, '#f39c12'),
            ("📡 Initialize GPS", self.start_gps_system, '#9b59b6'),
            ("🚨 Emergency Stop", self.emergency_stop, '#e74c3c')
        ]
        
        for text, command, color in buttons:
            btn = tk.Button(
                controls_frame,
                text=text,
                command=command,
                font=('Arial', 9, 'bold'),
                fg='white',
                bg=color,
                relief='flat',
                cursor='hand2'
            )
            btn.pack(fill='x', padx=5, pady=2)
            
    def create_system_resources_section(self, parent):
        """Create system resources display in sidebar"""
        resources_frame = tk.LabelFrame(
            parent,
            text="📊 System Resources",
            font=('Arial', 11, 'bold'),
            fg='#3498db',
            bg='#16213e',
            labelanchor='n'
        )
        resources_frame.pack(fill='x', padx=10, pady=5)
        
        self.resource_labels = {}
        resources = ['CPU', 'Memory', 'Disk', 'Network', 'Temperature']
        
        for resource in resources:
            frame = tk.Frame(resources_frame, bg='#16213e')
            frame.pack(fill='x', padx=5, pady=2)
            
            label = tk.Label(
                frame,
                text=f"{resource}: --",
                font=('Arial', 9),
                fg='#ecf0f1',
                bg='#16213e',
                anchor='w'
            )
            label.pack(side='left')
            
            self.resource_labels[resource.lower()] = label
            
    def create_environmental_section(self, parent):
        """Create environmental data display in sidebar"""
        env_frame = tk.LabelFrame(
            parent,
            text="🌡️ Environment",
            font=('Arial', 11, 'bold'),
            fg='#3498db',
            bg='#16213e',
            labelanchor='n'
        )
        env_frame.pack(fill='x', padx=10, pady=5)
        
        self.env_labels = {}
        env_params = ['Temperature', 'Humidity', 'Wind Speed', 'Soil Moisture', 'Light Level']
        
        for param in env_params:
            frame = tk.Frame(env_frame, bg='#16213e')
            frame.pack(fill='x', padx=5, pady=2)
            
            label = tk.Label(
                frame,
                text=f"{param}: --",
                font=('Arial', 9),
                fg='#ecf0f1',
                bg='#16213e',
                anchor='w'
            )
            label.pack(side='left')
            
            self.env_labels[param.lower().replace(' ', '_')] = label
            
    def create_alerts_section(self, parent):
        """Create alerts display in sidebar"""
        alerts_frame = tk.LabelFrame(
            parent,
            text="🚨 Quick Alerts",
            font=('Arial', 11, 'bold'),
            fg='#3498db',
            bg='#16213e',
            labelanchor='n'
        )
        alerts_frame.pack(fill='x', padx=10, pady=5)
        
        self.alerts_text = tk.Text(
            alerts_frame,
            height=4,
            font=('Arial', 8),
            fg='#ecf0f1',
            bg='#2c3e50',
            relief='flat',
            wrap='word'
        )
        self.alerts_text.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Add initial alert
        self.alerts_text.insert('1.0', "✅ System Online\n📊 Dashboard Ready\n🔄 Monitoring Active")
        self.alerts_text.config(state='disabled')
        
    def create_title_section(self, parent):
        """Create title section for main content"""
        title_frame = tk.Frame(parent, bg='#1a1a2e')
        title_frame.pack(fill='x', padx=10, pady=5)
        
        title_label = tk.Label(
            title_frame,
            text="🚜 TRACTOBOTS REAL-TIME DASHBOARD 🚜",
            font=('Arial', 20, 'bold'),
            fg='#ecf0f1',
            bg='#1a1a2e'
        )
        title_label.pack()
        
        status_label = tk.Label(
            title_frame,
            text="Professional Agricultural Robotics Control Center",
            font=('Arial', 12),
            fg='#95a5a6',
            bg='#1a1a2e'
        )
        status_label.pack()
        
    def create_overview_tab(self):
        """Create system overview tab"""
        overview_frame = tk.Frame(self.notebook, bg='#34495e')
        self.notebook.add(overview_frame, text="📊 System Overview")
        
        # Status indicators frame
        status_frame = tk.LabelFrame(
            overview_frame,
            text="System Status",
            font=('Arial', 12, 'bold'),
            fg='#ecf0f1',
            bg='#34495e'
        )
        status_frame.pack(fill='x', padx=10, pady=5)
        
        # Create status indicators
        self.status_indicators = {}
        status_items = [
            ('ROS2 Jazzy', 'ros2'),
            ('Gazebo Simulator', 'gazebo'),
            ('Navigation Stack', 'navigation'),
            ('GPS System', 'gps'),
            ('Active Simulation', 'simulation')
        ]
        
        for i, (name, key) in enumerate(status_items):
            frame = tk.Frame(status_frame, bg='#34495e')
            frame.pack(side='left', padx=20, pady=10)
            
            label = tk.Label(frame, text=name, font=('Arial', 10), fg='#ecf0f1', bg='#34495e')
            label.pack()
            
            indicator = tk.Label(
                frame,
                text="●",
                font=('Arial', 20),
                fg='#e74c3c',  # Red by default
                bg='#34495e'
            )
            indicator.pack()
            
            self.status_indicators[key] = indicator
        
        # Metrics frame
        metrics_frame = tk.LabelFrame(
            overview_frame,
            text="Real-Time Metrics",
            font=('Arial', 12, 'bold'),
            fg='#ecf0f1',
            bg='#34495e'
        )
        metrics_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        # Create matplotlib figure for real-time graphs
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(12, 6))
        self.fig.patch.set_facecolor('#34495e')
        
        # Configure subplots
        for ax in [self.ax1, self.ax2, self.ax3, self.ax4]:
            ax.set_facecolor('#2c3e50')
            ax.tick_params(colors='white')
            ax.spines['bottom'].set_color('white')
            ax.spines['top'].set_color('white')
            ax.spines['right'].set_color('white')
            ax.spines['left'].set_color('white')
        
        # Initialize plots
        self.setup_real_time_plots()
        
        # Add matplotlib canvas
        canvas = FigureCanvasTkAgg(self.fig, metrics_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill='both', expand=True)
        
        # Start animation
        self.start_animation()
        
    def create_field_visualization_tab(self):
        """Create field visualization tab"""
        field_frame = tk.Frame(self.notebook, bg='#34495e')
        self.notebook.add(field_frame, text="🗺️ Field Visualization")
        
        # Control panel
        control_panel = tk.Frame(field_frame, bg='#34495e')
        control_panel.pack(fill='x', padx=10, pady=5)
        
        # Import buttons
        import_frame = tk.LabelFrame(
            control_panel,
            text="Data Import",
            font=('Arial', 10, 'bold'),
            fg='#ecf0f1',
            bg='#34495e'
        )
        import_frame.pack(side='left', padx=10, pady=5)
        
        tk.Button(
            import_frame,
            text="📁 Import Shapefile",
            command=self.import_shapefile,
            bg='#3498db',
            fg='white',
            font=('Arial', 10, 'bold'),
            pady=5
        ).pack(side='left', padx=5)
        
        tk.Button(
            import_frame,
            text="🛰️ Import GPS Data",
            command=self.import_gps_data,
            bg='#27ae60',
            fg='white',
            font=('Arial', 10, 'bold'),
            pady=5
        ).pack(side='left', padx=5)
        
        tk.Button(
            import_frame,
            text="🎯 Generate Path",
            command=self.generate_coverage_path,
            bg='#e67e22',
            fg='white',
            font=('Arial', 10, 'bold'),
            pady=5
        ).pack(side='left', padx=5)
        
        # Field visualization
        viz_frame = tk.LabelFrame(
            field_frame,
            text="Field Map",
            font=('Arial', 12, 'bold'),
            fg='#ecf0f1',
            bg='#34495e'
        )
        viz_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        # Create field plot
        self.field_fig, self.field_ax = plt.subplots(figsize=(10, 8))
        self.field_fig.patch.set_facecolor('#34495e')
        self.field_ax.set_facecolor('#2c3e50')
        self.field_ax.set_title('Field Boundaries and Coverage Path', color='white', fontsize=14)
        self.field_ax.set_xlabel('Longitude', color='white')
        self.field_ax.set_ylabel('Latitude', color='white')
        self.field_ax.tick_params(colors='white')
        
        # Initialize empty field plot
        self.update_field_visualization()
        
        field_canvas = FigureCanvasTkAgg(self.field_fig, viz_frame)
        field_canvas.draw()
        field_canvas.get_tk_widget().pack(fill='both', expand=True)
        
    def create_system_control_tab(self):
        """Create system control tab"""
        control_frame = tk.Frame(self.notebook, bg='#34495e')
        self.notebook.add(control_frame, text="🎛️ System Control")
        
        # ROS2 Control
        ros2_frame = tk.LabelFrame(
            control_frame,
            text="ROS2 System Control",
            font=('Arial', 12, 'bold'),
            fg='#ecf0f1',
            bg='#34495e'
        )
        ros2_frame.pack(fill='x', padx=10, pady=5)
        
        ros2_buttons = [
            ("🚀 Launch ROS2 Core", self.launch_ros2_core, '#3498db'),
            ("🗺️ Start Navigation", self.start_navigation, '#27ae60'),
            ("📡 Launch GPS", self.launch_gps, '#f39c12'),
            ("🛑 Stop All ROS2", self.stop_ros2, '#e74c3c')
        ]
        
        for text, command, color in ros2_buttons:
            tk.Button(
                ros2_frame,
                text=text,
                command=command,
                bg=color,
                fg='white',
                font=('Arial', 10, 'bold'),
                width=20,
                pady=5
            ).pack(side='left', padx=5, pady=5)
        
        # Gazebo Control
        gazebo_frame = tk.LabelFrame(
            control_frame,
            text="Gazebo Simulator Control",
            font=('Arial', 12, 'bold'),
            fg='#ecf0f1',
            bg='#34495e'
        )
        gazebo_frame.pack(fill='x', padx=10, pady=5)
        
        gazebo_buttons = [
            ("🌾 Launch Farm Simulation", self.launch_gazebo_farm, '#2ecc71'),
            ("🚜 Spawn Tractor", self.spawn_tractor, '#9b59b6'),
            ("📊 Open RViz", self.launch_rviz, '#34495e'),
            ("🛑 Stop Gazebo", self.stop_gazebo, '#e74c3c')
        ]
        
        for text, command, color in gazebo_buttons:
            tk.Button(
                gazebo_frame,
                text=text,
                command=command,
                bg=color,
                fg='white',
                font=('Arial', 10, 'bold'),
                width=20,
                pady=5
            ).pack(side='left', padx=5, pady=5)
        
        # Mission Control
        mission_frame = tk.LabelFrame(
            control_frame,
            text="Mission Control",
            font=('Arial', 12, 'bold'),
            fg='#ecf0f1',
            bg='#34495e'
        )
        mission_frame.pack(fill='x', padx=10, pady=5)
        
        mission_buttons = [
            ("🎯 Start Mission", self.start_mission, '#e67e22'),
            ("⏸️ Pause Mission", self.pause_mission, '#f39c12'),
            ("🏠 Return Home", self.return_home, '#16a085'),
            ("🚨 Emergency Stop", self.emergency_stop, '#c0392b')
        ]
        
        for text, command, color in mission_buttons:
            tk.Button(
                mission_frame,
                text=text,
                command=command,
                bg=color,
                fg='white',
                font=('Arial', 10, 'bold'),
                width=20,
                pady=5
            ).pack(side='left', padx=5, pady=5)
        
        # System Information
        info_frame = tk.LabelFrame(
            control_frame,
            text="System Information",
            font=('Arial', 12, 'bold'),
            fg='#ecf0f1',
            bg='#34495e'
        )
        info_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        # Create text widget for system info
        self.info_text = tk.Text(
            info_frame,
            bg='#2c3e50',
            fg='#ecf0f1',
            font=('Courier', 10),
            height=15
        )
        self.info_text.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Add scrollbar
        scrollbar = tk.Scrollbar(info_frame, command=self.info_text.yview)
        scrollbar.pack(side='right', fill='y')
        self.info_text.config(yscrollcommand=scrollbar.set)
        
        self.update_system_info()
        
    def create_simulation_tab(self):
        """Create 3D simulation tab"""
        sim_frame = tk.Frame(self.notebook, bg='#34495e')
        self.notebook.add(sim_frame, text="🎮 3D Simulation")
        
        # Simulation controls
        sim_control_frame = tk.LabelFrame(
            sim_frame,
            text="Simulation Controls",
            font=('Arial', 12, 'bold'),
            fg='#ecf0f1',
            bg='#34495e'
        )
        sim_control_frame.pack(fill='x', padx=10, pady=5)
        
        # Scenario selection
        scenario_frame = tk.Frame(sim_control_frame, bg='#34495e')
        scenario_frame.pack(side='left', padx=10, pady=5)
        
        tk.Label(
            scenario_frame,
            text="Scenario:",
            font=('Arial', 10, 'bold'),
            fg='#ecf0f1',
            bg='#34495e'
        ).pack(side='left')
        
        self.scenario_var = tk.StringVar(value="Open Field")
        scenario_combo = ttk.Combobox(
            scenario_frame,
            textvariable=self.scenario_var,
            values=["Open Field", "Orchard", "Farmyard", "Hilly Terrain", "Forest Edge"],
            width=15
        )
        scenario_combo.pack(side='left', padx=5)
        
        # Control buttons
        button_frame = tk.Frame(sim_control_frame, bg='#34495e')
        button_frame.pack(side='right', padx=10, pady=5)
        
        sim_buttons = [
            ("🎮 Launch 3D Sim", self.launch_3d_simulation, '#8e44ad'),
            ("🚜 Auto Navigate", self.start_auto_navigation, '#27ae60'),
            ("⏹️ Stop Simulation", self.stop_simulation, '#e74c3c')
        ]
        
        for text, command, color in sim_buttons:
            tk.Button(
                button_frame,
                text=text,
                command=command,
                bg=color,
                fg='white',
                font=('Arial', 10, 'bold'),
                pady=5
            ).pack(side='left', padx=5)
        
        # Simulation metrics
        metrics_frame = tk.LabelFrame(
            sim_frame,
            text="Simulation Metrics",
            font=('Arial', 12, 'bold'),
            fg='#ecf0f1',
            bg='#34495e'
        )
        metrics_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        # Create simulation visualization
        self.sim_fig, (self.sim_ax1, self.sim_ax2) = plt.subplots(1, 2, figsize=(12, 6))
        self.sim_fig.patch.set_facecolor('#34495e')
        
        for ax in [self.sim_ax1, self.sim_ax2]:
            ax.set_facecolor('#2c3e50')
            ax.tick_params(colors='white')
        
        self.setup_simulation_plots()
        
        sim_canvas = FigureCanvasTkAgg(self.sim_fig, metrics_frame)
        sim_canvas.draw()
        sim_canvas.get_tk_widget().pack(fill='both', expand=True)
        
    def create_logs_tab(self):
        """Create logs and monitoring tab"""
        logs_frame = tk.Frame(self.notebook, bg='#34495e')
        self.notebook.add(logs_frame, text="📝 Logs & Monitoring")
        
        # Create text widget for logs
        self.log_text = tk.Text(
            logs_frame,
            bg='#2c3e50',
            fg='#ecf0f1',
            font=('Courier', 10)
        )
        self.log_text.pack(fill='both', expand=True, padx=10, pady=10)
        
        # Add scrollbar
        log_scrollbar = tk.Scrollbar(logs_frame, command=self.log_text.yview)
        log_scrollbar.pack(side='right', fill='y')
        self.log_text.config(yscrollcommand=log_scrollbar.set)
        
        # Start logging
        self.start_logging()
        
    def setup_real_time_plots(self):
        """Setup real-time monitoring plots"""
        
        # CPU Usage
        self.ax1.set_title('CPU Usage', color='white')
        self.ax1.set_ylabel('Usage %', color='white')
        self.cpu_line, = self.ax1.plot([], [], 'g-', linewidth=2)
        self.ax1.set_ylim(0, 100)
        
        # Memory Usage
        self.ax2.set_title('Memory Usage', color='white')
        self.ax2.set_ylabel('Usage %', color='white')
        self.mem_line, = self.ax2.plot([], [], 'b-', linewidth=2)
        self.ax2.set_ylim(0, 100)
        
        # Network Activity
        self.ax3.set_title('ROS2 Topics/sec', color='white')
        self.ax3.set_ylabel('Messages/sec', color='white')
        self.network_line, = self.ax3.plot([], [], 'r-', linewidth=2)
        self.ax3.set_ylim(0, 1000)
        
        # GPS Signal Quality
        self.ax4.set_title('GPS Signal Quality', color='white')
        self.ax4.set_ylabel('Signal Strength', color='white')
        self.gps_line, = self.ax4.plot([], [], 'y-', linewidth=2)
        self.ax4.set_ylim(0, 100)
        
        # Initialize data arrays
        self.time_data = []
        self.cpu_data = []
        self.mem_data = []
        self.network_data = []
        self.gps_data_signal = []
        
    def setup_simulation_plots(self):
        """Setup simulation monitoring plots"""
        
        # Tractor position
        self.sim_ax1.set_title('Tractor Position', color='white')
        self.sim_ax1.set_xlabel('X Position (m)', color='white')
        self.sim_ax1.set_ylabel('Y Position (m)', color='white')
        self.tractor_plot, = self.sim_ax1.plot([], [], 'ro', markersize=10)
        self.path_plot, = self.sim_ax1.plot([], [], 'g-', linewidth=2)
        
        # Speed and heading
        self.sim_ax2.set_title('Speed & Heading', color='white')
        self.sim_ax2.set_xlabel('Time', color='white')
        self.sim_ax2.set_ylabel('Speed (m/s)', color='white')
        self.speed_plot, = self.sim_ax2.plot([], [], 'c-', linewidth=2)
        
    def start_animation(self):
        """Start real-time animation"""
        self.animation_active = True
        self.animate()
        
    def animate(self):
        """Animation update function"""
        if not self.animation_active:
            return
            
        # Update data
        current_time = time.time()
        self.time_data.append(current_time)
        
        # Simulate data (replace with real data sources)
        self.cpu_data.append(np.random.uniform(20, 80))
        self.mem_data.append(np.random.uniform(30, 70))
        self.network_data.append(np.random.uniform(100, 500))
        self.gps_data_signal.append(np.random.uniform(80, 95))
        
        # Keep only last 100 points
        if len(self.time_data) > 100:
            self.time_data = self.time_data[-100:]
            self.cpu_data = self.cpu_data[-100:]
            self.mem_data = self.mem_data[-100:]
            self.network_data = self.network_data[-100:]
            self.gps_data_signal = self.gps_data_signal[-100:]
        
        # Update plots
        if len(self.time_data) > 1:
            time_range = np.arange(len(self.time_data))
            
            self.cpu_line.set_data(time_range, self.cpu_data)
            self.ax1.set_xlim(0, len(self.time_data))
            
            self.mem_line.set_data(time_range, self.mem_data)
            self.ax2.set_xlim(0, len(self.time_data))
            
            self.network_line.set_data(time_range, self.network_data)
            self.ax3.set_xlim(0, len(self.time_data))
            
            self.gps_line.set_data(time_range, self.gps_data_signal)
            self.ax4.set_xlim(0, len(self.time_data))
        
        # Redraw
        self.fig.canvas.draw_idle()
        
        # Update status indicators
        self.update_status_indicators()
        
        # Schedule next update
        self.root.after(1000, self.animate)
        
    def update_status_indicators(self):
        """Update system status indicators"""
        # Check actual system status (simplified for demo)
        statuses = {
            'ros2': self.check_ros2_status(),
            'gazebo': self.check_gazebo_status(),
            'navigation': self.check_navigation_status(),
            'gps': self.check_gps_status(),
            'simulation': self.check_simulation_status()
        }
        
        for key, status in statuses.items():
            color = '#27ae60' if status else '#e74c3c'  # Green if active, red if not
            self.status_indicators[key].config(fg=color)
            self.system_status[key] = status
    
    def check_ros2_status(self):
        """Check if ROS2 is running"""
        try:
            result = subprocess.run(['pgrep', '-f', 'ros2'], capture_output=True)
            return result.returncode == 0
        except:
            return False
    
    def check_gazebo_status(self):
        """Check if Gazebo is running"""
        try:
            result = subprocess.run(['pgrep', '-f', 'gz sim'], capture_output=True)
            return result.returncode == 0
        except:
            return False
    
    def check_navigation_status(self):
        """Check if navigation stack is running"""
        # Simplified check
        return 'navigation' in self.running_processes
    
    def check_gps_status(self):
        """Check if GPS system is active"""
        # Simplified check
        return 'gps' in self.running_processes
    
    def check_simulation_status(self):
        """Check if 3D simulation is running"""
        return 'simulation' in self.running_processes
    
    def start_monitoring(self):
        """Start system monitoring and sidebar updates"""
        self.log_message("Dashboard started - monitoring all systems")
        self.log_message("🚜 Tractobots Real-Time Dashboard v1.0")
        self.log_message("=" * 50)
        
        # Start sidebar updates
        self.update_sidebar_data()
        
    def start_logging(self):
        """Start logging system messages"""
        self.log_message("Log monitoring started")
        
    def log_message(self, message):
        """Add message to log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        
        if hasattr(self, 'log_text'):
            self.log_text.insert(tk.END, log_entry)
            self.log_text.see(tk.END)
    
    def add_log_entry(self, message):
        """Add entry to logs"""
        timestamp = time.strftime("%H:%M:%S")
        log_message = f"[{timestamp}] {message}"
        
        # Add to logs tab if it exists
        if hasattr(self, 'logs_text'):
            self.logs_text.insert(tk.END, log_message + "\n")
            self.logs_text.see(tk.END)
            
        # Print to console
        print(log_message)
        
    # Import functions
    def import_shapefile(self):
        """Import shapefile data"""
        file_path = filedialog.askopenfilename(
            title="Select Shapefile",
            filetypes=[("Shapefile", "*.shp"), ("All files", "*.*")]
        )
        
        if file_path:
            self.log_message(f"Importing shapefile: {file_path}")
            try:
                from src.tractobots_mission_ui.tractobots_mission_ui.shapefile_manager import ShapefileManager  # type: ignore
                manager = ShapefileManager()
                fields_data = manager.load_field_boundaries(file_path)
                
                if fields_data:
                    self.field_data = fields_data
                    self.update_field_visualization()
                    self.log_message(f"✅ Shapefile imported successfully: {len(fields_data.get('fields', []))} fields")
                    
                    # Export to Gazebo
                    gazebo_world = manager.export_to_gazebo_world(fields_data, 'imported_field.sdf')
                    self.log_message(f"✅ Gazebo world exported: {gazebo_world}")
                else:
                    self.log_message("❌ Failed to import shapefile")
                    
            except Exception as e:
                self.log_message(f"❌ Shapefile import error: {str(e)}")
    
    def import_gps_data(self):
        """Import GPS guidance data"""
        file_path = filedialog.askopenfilename(
            title="Select GPS Data File",
            filetypes=[
                ("XML files", "*.xml"),
                ("CSV files", "*.csv"),
                ("KML files", "*.kml"),
                ("All files", "*.*")
            ]
        )
        
        if file_path:
            self.log_message(f"Importing GPS data: {file_path}")
            try:
                sys.path.insert(0, os.path.join(os.getcwd()))
                from john_deere_gps_importer import JohnDeereGPSImporter
                importer = JohnDeereGPSImporter()
                gps_data = importer.import_file(file_path)
                
                if gps_data:
                    self.gps_data = gps_data
                    self.update_field_visualization()
                    lines_count = len(gps_data.get('guidance_lines', []))
                    self.log_message(f"✅ GPS data imported: {lines_count} guidance lines")
                    
                    # Export to Gazebo
                    gazebo_world = importer.export_to_gazebo_world(gps_data, 'gps_field.sdf')
                    waypoints = importer.export_to_gazebo_waypoints(gps_data, 'gps_waypoints.yaml')
                    self.log_message(f"✅ Gazebo world and waypoints exported")
                else:
                    self.log_message("❌ Failed to import GPS data")
                    
            except Exception as e:
                self.log_message(f"❌ GPS import error: {str(e)}")
    
    def generate_coverage_path(self):
        """Generate coverage path from field data"""
        if not self.field_data:
            messagebox.showwarning("No Data", "Please import field boundaries first")
            return
        
        self.log_message("Generating coverage path...")
        try:
            from src.tractobots_mission_ui.tractobots_mission_ui.shapefile_manager import ShapefileManager
            manager = ShapefileManager()
            
            if 'fields' in self.field_data and len(self.field_data['fields']) > 0:
                field_id = 'field_0'  # Use first field
                waypoints = manager.generate_coverage_path(self.field_data, field_id, swath_width=5.0)
                
                if waypoints:
                    self.waypoints = waypoints
                    self.update_field_visualization()
                    self.log_message(f"✅ Coverage path generated: {len(waypoints)} waypoints")
                    
                    # Export waypoints to Gazebo
                    gazebo_waypoints = manager.export_to_gazebo_waypoints(waypoints, 'coverage_waypoints.yaml')
                    self.log_message(f"✅ Waypoints exported for Gazebo")
                else:
                    self.log_message("❌ Failed to generate coverage path")
            else:
                self.log_message("❌ No valid field data found")
                
        except Exception as e:
            self.log_message(f"❌ Path generation error: {str(e)}")
    
    def update_field_visualization(self):
        """Update field visualization plot"""
        self.field_ax.clear()
        self.field_ax.set_facecolor('#2c3e50')
        self.field_ax.set_title('Field Boundaries and Coverage Path', color='white', fontsize=14)
        self.field_ax.set_xlabel('Longitude', color='white')
        self.field_ax.set_ylabel('Latitude', color='white')
        
        # Plot field boundaries
        if self.field_data and 'fields' in self.field_data:
            for field in self.field_data['fields']:
                if 'boundary_points' in field:
                    boundary_points = field['boundary_points']
                    if isinstance(boundary_points, list) and len(boundary_points) > 0:
                        if isinstance(boundary_points[0], dict):
                            # Single polygon
                            lons = [point['lon'] for point in boundary_points]
                            lats = [point['lat'] for point in boundary_points]
                            self.field_ax.plot(lons + [lons[0]], lats + [lats[0]], 'r-', linewidth=2, label='Field Boundary')
                        else:
                            # Multi-polygon
                            for i, polygon in enumerate(boundary_points):
                                lons = [point['lon'] for point in polygon]
                                lats = [point['lat'] for point in polygon]
                                label = 'Field Boundary' if i == 0 else None
                                self.field_ax.plot(lons + [lons[0]], lats + [lats[0]], 'r-', linewidth=2, label=label)
        
        # Plot GPS guidance lines
        if self.gps_data and 'guidance_lines' in self.gps_data:
            for i, line in enumerate(self.gps_data['guidance_lines']):
                if 'points' in line:
                    lons = [point['lon'] for point in line['points'] if 'lon' in point]
                    lats = [point['lat'] for point in line['points'] if 'lat' in point]
                    if lons and lats:
                        label = 'GPS Guidance' if i == 0 else None
                        self.field_ax.plot(lons, lats, 'g-', linewidth=1, alpha=0.7, label=label)
        
        # Plot coverage path waypoints
        if self.waypoints:
            lats = [wp[0] for wp in self.waypoints]
            lons = [wp[1] for wp in self.waypoints]
            self.field_ax.plot(lons, lats, 'bo-', markersize=3, linewidth=1, alpha=0.8, label='Coverage Path')
        
        # Plot current tractor position
        if self.tractor_position != [0, 0]:
            self.field_ax.plot(self.tractor_position[1], self.tractor_position[0], 'yo', markersize=10, label='Tractor')
        
        self.field_ax.legend(facecolor='#34495e', edgecolor='white', labelcolor='white')
        self.field_ax.tick_params(colors='white')
        self.field_fig.canvas.draw_idle()
    
    # System control functions
    def launch_ros2_core(self):
        """Launch ROS2 core system"""
        self.log_message("🚀 Launching ROS2 core...")
        try:
            # Source ROS2 and launch
            cmd = 'source /opt/ros/jazzy/setup.bash && ros2 daemon start'
            process = subprocess.Popen(cmd, shell=True)
            self.running_processes['ros2_core'] = process
            self.log_message("✅ ROS2 core launched")
        except Exception as e:
            self.log_message(f"❌ Failed to launch ROS2: {str(e)}")
    
    def start_navigation(self):
        """Start navigation stack"""
        self.log_message("🗺️ Starting navigation stack...")
        try:
            cmd = 'source /opt/ros/jazzy/setup.bash && ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true'
            process = subprocess.Popen(cmd, shell=True)
            self.running_processes['navigation'] = process
            self.log_message("✅ Navigation stack started")
        except Exception as e:
            self.log_message(f"❌ Failed to start navigation: {str(e)}")
    
    def launch_gps(self):
        """Launch GPS system"""
        self.log_message("📡 Launching GPS system...")
        try:
            cmd = 'source /opt/ros/jazzy/setup.bash && ros2 launch tractobots_gps gps.launch.py'
            process = subprocess.Popen(cmd, shell=True)
            self.running_processes['gps'] = process
            self.log_message("✅ GPS system launched")
        except Exception as e:
            self.log_message(f"❌ Failed to launch GPS: {str(e)}")
    
    def stop_ros2(self):
        """Stop all ROS2 processes"""
        self.log_message("🛑 Stopping ROS2 processes...")
        try:
            subprocess.run(['pkill', '-f', 'ros2'])
            self.running_processes.clear()
            self.log_message("✅ ROS2 processes stopped")
        except Exception as e:
            self.log_message(f"❌ Error stopping ROS2: {str(e)}")
    
    def launch_gazebo_farm(self):
        """Launch Gazebo farm simulation"""
        self.log_message("🌾 Launching Gazebo farm simulation...")
        try:
            cmd = 'cd ~/tractobots_gazebo_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch tractobots_gazebo tractobots_gazebo.launch.py'
            process = subprocess.Popen(cmd, shell=True)
            self.running_processes['gazebo'] = process
            self.log_message("✅ Gazebo farm simulation launched")
        except Exception as e:
            self.log_message(f"❌ Failed to launch Gazebo: {str(e)}")
    
    def spawn_tractor(self):
        """Spawn tractor in simulation"""
        self.log_message("🚜 Spawning tractor in simulation...")
        # Implementation would spawn tractor model
        self.log_message("✅ Tractor spawned")
    
    def launch_rviz(self):
        """Launch RViz visualization"""
        self.log_message("📊 Launching RViz...")
        try:
            cmd = 'source /opt/ros/jazzy/setup.bash && rviz2'
            process = subprocess.Popen(cmd, shell=True)
            self.running_processes['rviz'] = process
            self.log_message("✅ RViz launched")
        except Exception as e:
            self.log_message(f"❌ Failed to launch RViz: {str(e)}")
    
    def stop_gazebo(self):
        """Stop Gazebo simulation"""
        self.log_message("🛑 Stopping Gazebo...")
        try:
            subprocess.run(['pkill', '-f', 'gz sim'])
            if 'gazebo' in self.running_processes:
                del self.running_processes['gazebo']
            self.log_message("✅ Gazebo stopped")
        except Exception as e:
            self.log_message(f"❌ Error stopping Gazebo: {str(e)}")
    
    # Mission control functions
    def start_mission(self):
        """Start autonomous mission"""
        self.log_message("🎯 Starting autonomous mission...")
        self.log_message("✅ Mission started - autonomous operation active")
    
    def pause_mission(self):
        """Pause current mission"""
        self.log_message("⏸️ Mission paused")
    
    def return_home(self):
        """Return tractor to home position"""
        self.log_message("🏠 Returning to home position...")
    
    def emergency_stop(self):
        """Emergency stop all operations"""
        self.log_message("🚨 EMERGENCY STOP ACTIVATED")
        messagebox.showwarning("Emergency Stop", "All operations have been stopped for safety!")
    
    # Simulation functions
    def launch_3d_simulation(self):
        """Launch 3D simulation"""
        scenario = self.scenario_var.get()
        self.log_message(f"🎮 Launching 3D simulation: {scenario}")
        try:
            # Launch 3D simulator based on scenario
            cmd = f'python3 farm_3d_simulator.py --scenario "{scenario}"'
            process = subprocess.Popen(cmd, shell=True)
            self.running_processes['simulation'] = process
            self.log_message("✅ 3D simulation launched")
        except Exception as e:
            self.log_message(f"❌ Failed to launch 3D simulation: {str(e)}")
    
    def start_auto_navigation(self):
        """Start autonomous navigation in simulation"""
        self.log_message("🚜 Starting autonomous navigation...")
        self.log_message("✅ Auto navigation active")
    
    def stop_simulation(self):
        """Stop 3D simulation"""
        self.log_message("⏹️ Stopping 3D simulation...")
        if 'simulation' in self.running_processes:
            self.running_processes['simulation'].terminate()
            del self.running_processes['simulation']
        self.log_message("✅ 3D simulation stopped")
    
    def update_system_info(self):
        """Update system information display"""
        info = f"""
🚜 TRACTOBOTS SYSTEM INFORMATION
{'='*50}

System Status:
├── ROS2 Jazzy: {'✅ Active' if self.system_status['ros2'] else '❌ Inactive'}
├── Gazebo Sim: {'✅ Running' if self.system_status['gazebo'] else '❌ Stopped'}
├── Navigation: {'✅ Ready' if self.system_status['navigation'] else '❌ Not Ready'}
├── GPS System: {'✅ Connected' if self.system_status['gps'] else '❌ Disconnected'}
└── 3D Simulation: {'✅ Active' if self.system_status['simulation'] else '❌ Inactive'}

Hardware Status:
├── CPU Cores: 12 available
├── Memory: 31GB total
├── Disk Space: 940GB free
└── GPU: Available for 3D rendering

Field Data:
├── Shapefiles: {'✅ Loaded' if self.field_data else '❌ None'}
├── GPS Data: {'✅ Loaded' if self.gps_data else '❌ None'}
├── Waypoints: {len(self.waypoints) if self.waypoints else 0} generated
└── Coverage Path: {'✅ Ready' if self.waypoints else '❌ Not Generated'}

Active Processes:
{chr(10).join([f'├── {name}: PID {proc.pid}' for name, proc in self.running_processes.items()])}

Last Updated: {datetime.now().strftime('%H:%M:%S')}
        """
        
        if hasattr(self, 'info_text'):
            self.info_text.delete(1.0, tk.END)
            self.info_text.insert(1.0, info)
            
        # Schedule next update
        self.root.after(5000, self.update_system_info)
    
    def start_ros2_system(self):
        """Start ROS2 system from sidebar"""
        self.add_log_entry("🚀 Starting ROS2 system...")
        self.update_system_status('ros2', True)
        self.add_alert("✅ ROS2 system started successfully")
        
    def start_gazebo_simulation(self):
        """Start Gazebo simulation from sidebar"""
        self.add_log_entry("🌍 Launching Gazebo simulation...")
        self.update_system_status('gazebo', True)
        self.add_alert("✅ Gazebo simulation launched")
        
    def start_navigation(self):
        """Start navigation system from sidebar"""
        self.add_log_entry("🧭 Starting navigation system...")
        self.update_system_status('navigation', True)
        self.add_alert("✅ Navigation system started")
        
    def start_gps_system(self):
        """Start GPS system from sidebar"""
        self.add_log_entry("📡 Initializing GPS system...")
        self.update_system_status('gps', True)
        self.add_alert("✅ GPS system initialized")
        
    def emergency_stop(self):
        """Emergency stop all systems"""
        self.add_log_entry("🚨 EMERGENCY STOP ACTIVATED")
        for system in self.system_status:
            self.update_system_status(system, False)
        self.add_alert("🚨 Emergency stop activated - all systems halted")
        
    def update_system_status(self, system, status):
        """Update system status in sidebar"""
        if system in self.system_status:
            self.system_status[system] = status
            if system in self.status_labels:
                icon = "🟢" if status else "🔴"
                color = "#2ecc71" if status else "#e74c3c"
                self.status_labels[system].config(
                    text=f"{icon} {system.upper().replace('_', ' ')}",
                    fg=color
                )
                
    def add_alert(self, message):
        """Add alert to sidebar alerts section"""
        self.alerts.append(message)
        self.alerts_text.config(state='normal')
        self.alerts_text.delete('1.0', tk.END)
        recent_alerts = self.alerts[-3:]  # Show last 3 alerts
        self.alerts_text.insert('1.0', '\n'.join(recent_alerts))
        self.alerts_text.config(state='disabled')
        
    def update_sidebar_data(self):
        """Update sidebar with real-time data"""
        import time
        import math
        
        # Update time
        current_time = time.strftime("%Y-%m-%d %H:%M:%S")
        self.time_label.config(text=current_time)
        
        # Simulate system resource updates
        t = time.time()
        self.system_metrics['cpu_usage'] = 45 + 20 * math.sin(t * 0.1)
        self.system_metrics['memory_usage'] = 62 + 15 * math.cos(t * 0.08)
        self.system_metrics['disk_usage'] = 23 + 5 * math.sin(t * 0.05)
        self.system_metrics['network_activity'] = 12 + 18 * math.cos(t * 0.12)
        self.system_metrics['temperature'] = 55 + 10 * math.sin(t * 0.03)
        
        # Update resource labels
        self.resource_labels['cpu'].config(text=f"CPU: {self.system_metrics['cpu_usage']:.1f}%")
        self.resource_labels['memory'].config(text=f"Memory: {self.system_metrics['memory_usage']:.1f}%")
        self.resource_labels['disk'].config(text=f"Disk: {self.system_metrics['disk_usage']:.1f}%")
        self.resource_labels['network'].config(text=f"Network: {self.system_metrics['network_activity']:.1f} MB/s")
        self.resource_labels['temperature'].config(text=f"Temperature: {self.system_metrics['temperature']:.1f}°C")
        
        # Update environmental data
        self.environmental_data['temperature'] = 22 + 8 * math.sin(t * 0.02)
        self.environmental_data['humidity'] = 65 + 15 * math.cos(t * 0.025)
        self.environmental_data['wind_speed'] = 2.5 + 3 * math.sin(t * 0.03)
        self.environmental_data['soil_moisture'] = 45 + 20 * math.sin(t * 0.015)
        self.environmental_data['light_level'] = 85 + 10 * math.cos(t * 0.01)
        
        # Update environmental labels
        self.env_labels['temperature'].config(text=f"Temperature: {self.environmental_data['temperature']:.1f}°C")
        self.env_labels['humidity'].config(text=f"Humidity: {self.environmental_data['humidity']:.1f}%")
        self.env_labels['wind_speed'].config(text=f"Wind Speed: {self.environmental_data['wind_speed']:.1f} m/s")
        self.env_labels['soil_moisture'].config(text=f"Soil Moisture: {self.environmental_data['soil_moisture']:.1f}%")
        self.env_labels['light_level'].config(text=f"Light Level: {self.environmental_data['light_level']:.1f}%")
        
        # Schedule next update
        self.root.after(1000, self.update_sidebar_data)
        
    # ...existing code...
def main():
    """Main function to run the dashboard"""
    root = tk.Tk()
    app = TractobotsDashboard(root)
    
    # Make window resizable
    root.resizable(True, True)
    
    # Center window on screen
    root.update_idletasks()
    x = (root.winfo_screenwidth() // 2) - (root.winfo_width() // 2)
    y = (root.winfo_screenheight() // 2) - (root.winfo_height() // 2)
    root.geometry(f"+{x}+{y}")
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("\n🛑 Dashboard closed by user")

if __name__ == "__main__":
    main()
