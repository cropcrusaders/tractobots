#!/usr/bin/env python3
"""
Tractobots Live Dashboard - Minimal Working Version
A simple but functional graphical interface to watch your Tractobots system
"""

import sys
import os
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from datetime import datetime

class TractobotsDashboard:
    def __init__(self, root):
        self.root = root
        self.root.title("🚜 Tractobots Live Dashboard")
        self.root.geometry("1200x800")
        self.root.configure(bg='#2c3e50')
        
        # Status tracking
        self.system_status = {
            'ros2': False,
            'gazebo': False,
            'navigation': False,
            'gps': False
        }
        
        # Data for visualization
        self.time_data = []
        self.position_data = []
        self.speed_data = []
        self.battery_data = []
        
        self.create_interface()
        self.start_updates()
        
    def create_interface(self):
        """Create the main dashboard interface"""
        
        # Header
        header_frame = tk.Frame(self.root, bg='#34495e', height=80)
        header_frame.pack(fill='x', padx=5, pady=5)
        header_frame.pack_propagate(False)
        
        title_label = tk.Label(
            header_frame,
            text="🚜 TRACTOBOTS LIVE DASHBOARD",
            font=('Arial', 24, 'bold'),
            fg='#ecf0f1',
            bg='#34495e'
        )
        title_label.pack(pady=15)
        
        # Main content area
        main_frame = tk.Frame(self.root, bg='#2c3e50')
        main_frame.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Left panel - System Status
        left_panel = tk.Frame(main_frame, bg='#34495e', width=300)
        left_panel.pack(side='left', fill='y', padx=5, pady=5)
        left_panel.pack_propagate(False)
        
        status_label = tk.Label(
            left_panel,
            text="SYSTEM STATUS",
            font=('Arial', 16, 'bold'),
            fg='#ecf0f1',
            bg='#34495e'
        )
        status_label.pack(pady=10)
        
        # Status indicators
        self.status_indicators = {}
        for system, status in self.system_status.items():
            frame = tk.Frame(left_panel, bg='#34495e')
            frame.pack(fill='x', padx=10, pady=5)
            
            label = tk.Label(
                frame,
                text=f"{system.upper()}:",
                font=('Arial', 12),
                fg='#ecf0f1',
                bg='#34495e'
            )
            label.pack(side='left')
            
            indicator = tk.Label(
                frame,
                text="●",
                font=('Arial', 16),
                fg='#e74c3c',
                bg='#34495e'
            )
            indicator.pack(side='right')
            
            self.status_indicators[system] = indicator
            
        # Control buttons
        control_frame = tk.Frame(left_panel, bg='#34495e')
        control_frame.pack(fill='x', padx=10, pady=20)
        
        tk.Button(
            control_frame,
            text="Start ROS2",
            command=self.start_ros2,
            bg='#27ae60',
            fg='white',
            font=('Arial', 10, 'bold')
        ).pack(fill='x', pady=2)
        
        tk.Button(
            control_frame,
            text="Start Gazebo",
            command=self.start_gazebo,
            bg='#3498db',
            fg='white',
            font=('Arial', 10, 'bold')
        ).pack(fill='x', pady=2)
        
        tk.Button(
            control_frame,
            text="Stop All",
            command=self.stop_all,
            bg='#e74c3c',
            fg='white',
            font=('Arial', 10, 'bold')
        ).pack(fill='x', pady=2)
        
        # Right panel - Live Data
        right_panel = tk.Frame(main_frame, bg='#34495e')
        right_panel.pack(side='right', fill='both', expand=True, padx=5, pady=5)
        
        # Create matplotlib figure
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.patch.set_facecolor('#34495e')  # type: ignore
        
        # Setup plots
        self.setup_plots()
        
        # Add to tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, right_panel)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill='both', expand=True)
        
        # Bottom status bar
        self.status_bar = tk.Label(
            self.root,
            text="Dashboard started - Ready for operation",
            relief='sunken',
            bg='#34495e',
            fg='#ecf0f1',
            font=('Arial', 10)
        )
        self.status_bar.pack(side='bottom', fill='x')
        
    def setup_plots(self):
        """Setup the matplotlib plots"""
        # Position plot
        self.ax1.set_title('Tractor Position', color='white', fontsize=12)
        self.ax1.set_xlabel('X (meters)', color='white')
        self.ax1.set_ylabel('Y (meters)', color='white')
        self.ax1.set_facecolor('#2c3e50')
        self.ax1.tick_params(colors='white')
        
        # Speed plot
        self.ax2.set_title('Speed Over Time', color='white', fontsize=12)
        self.ax2.set_xlabel('Time', color='white')
        self.ax2.set_ylabel('Speed (m/s)', color='white')
        self.ax2.set_facecolor('#2c3e50')
        self.ax2.tick_params(colors='white')
        
        # Battery plot
        self.ax3.set_title('Battery Level', color='white', fontsize=12)
        self.ax3.set_xlabel('Time', color='white')
        self.ax3.set_ylabel('Battery (%)', color='white')
        self.ax3.set_facecolor('#2c3e50')
        self.ax3.tick_params(colors='white')
        
        # System resources
        self.ax4.set_title('System Resources', color='white', fontsize=12)
        self.ax4.set_xlabel('Component', color='white')
        self.ax4.set_ylabel('Usage (%)', color='white')
        self.ax4.set_facecolor('#2c3e50')
        self.ax4.tick_params(colors='white')
        
    def start_updates(self):
        """Start periodic updates"""
        self.update_status()
        self.update_data()
        self.update_plots()
        
    def update_status(self):
        """Update system status indicators"""
        # Check ROS2
        try:
            result = subprocess.run(['ros2', 'node', 'list'], 
                                  capture_output=True, text=True, timeout=2)
            self.system_status['ros2'] = result.returncode == 0
        except:
            self.system_status['ros2'] = False
            
        # Check Gazebo (simplified)
        try:
            result = subprocess.run(['pgrep', '-f', 'gazebo'], 
                                  capture_output=True, text=True, timeout=2)
            self.system_status['gazebo'] = len(result.stdout.strip()) > 0
        except:
            self.system_status['gazebo'] = False
            
        # Update indicators
        for system, status in self.system_status.items():
            if system in self.status_indicators:
                color = '#27ae60' if status else '#e74c3c'
                self.status_indicators[system].config(fg=color)
                
        # Update status bar
        active_systems = sum(self.system_status.values())
        self.status_bar.config(text=f"Active Systems: {active_systems}/4 | Last Update: {datetime.now().strftime('%H:%M:%S')}")
        
        # Schedule next update
        self.root.after(5000, self.update_status)
        
    def update_data(self):
        """Update data arrays with simulated/real data"""
        current_time = datetime.now()
        
        # Simulate tractor movement
        t = time.time()
        x = 50 + 20 * np.sin(t * 0.1)
        y = 30 + 15 * np.cos(t * 0.1)
        speed = 2.5 + 0.5 * np.sin(t * 0.2)
        battery = 85 + 10 * np.sin(t * 0.05)
        
        # Keep last 50 data points
        self.time_data.append(current_time)
        self.position_data.append((x, y))
        self.speed_data.append(speed)
        self.battery_data.append(battery)
        
        if len(self.time_data) > 50:
            self.time_data.pop(0)
            self.position_data.pop(0)
            self.speed_data.pop(0)
            self.battery_data.pop(0)
            
        # Schedule next update
        self.root.after(1000, self.update_data)
        
    def update_plots(self):
        """Update the matplotlib plots"""
        if not self.time_data:
            self.root.after(2000, self.update_plots)
            return
            
        # Clear plots
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()
        self.ax4.clear()
        
        # Position plot
        if self.position_data:
            x_coords = [pos[0] for pos in self.position_data]
            y_coords = [pos[1] for pos in self.position_data]
            self.ax1.plot(x_coords, y_coords, 'g-', linewidth=2)
            self.ax1.scatter(x_coords[-1], y_coords[-1], color='red', s=100, marker='o')
            self.ax1.set_title('Tractor Position', color='white', fontsize=12)
            self.ax1.set_xlabel('X (meters)', color='white')
            self.ax1.set_ylabel('Y (meters)', color='white')
            self.ax1.grid(True, alpha=0.3)
            
        # Speed plot
        if len(self.time_data) > 1:
            self.ax2.plot(self.time_data, self.speed_data, 'b-', linewidth=2)
            self.ax2.set_title('Speed Over Time', color='white', fontsize=12)
            self.ax2.set_ylabel('Speed (m/s)', color='white')
            self.ax2.grid(True, alpha=0.3)
            
        # Battery plot
        if len(self.time_data) > 1:
            self.ax3.plot(self.time_data, self.battery_data, 'orange', linewidth=2)
            self.ax3.set_title('Battery Level', color='white', fontsize=12)
            self.ax3.set_ylabel('Battery (%)', color='white')
            self.ax3.set_ylim(0, 100)
            self.ax3.grid(True, alpha=0.3)
            
        # System resources (bar chart)
        components = ['CPU', 'Memory', 'Disk', 'Network']
        usage = [45, 62, 23, 78]  # Simulated data
        colors = ['#e74c3c', '#f39c12', '#27ae60', '#3498db']
        self.ax4.bar(components, usage, color=colors, alpha=0.7)
        self.ax4.set_title('System Resources', color='white', fontsize=12)
        self.ax4.set_ylabel('Usage (%)', color='white')
        self.ax4.set_ylim(0, 100)
        
        # Apply dark theme to all plots
        for ax in [self.ax1, self.ax2, self.ax3, self.ax4]:
            ax.set_facecolor('#2c3e50')
            ax.tick_params(colors='white')
            for spine in ax.spines.values():
                spine.set_color('white')
                
        # Update canvas
        self.canvas.draw()
        
        # Schedule next update
        self.root.after(2000, self.update_plots)
        
    def start_ros2(self):
        """Start ROS2 system"""
        self.status_bar.config(text="Starting ROS2 system...")
        threading.Thread(target=self._start_ros2_thread, daemon=True).start()
        
    def _start_ros2_thread(self):
        """Start ROS2 in background thread"""
        try:
            # This would be replaced with actual ROS2 startup commands
            time.sleep(2)  # Simulate startup time
            self.root.after(0, lambda: self.status_bar.config(text="ROS2 startup attempted"))
        except Exception as e:
            self.root.after(0, lambda: self.status_bar.config(text=f"ROS2 startup failed: {e}"))
            
    def start_gazebo(self):
        """Start Gazebo simulation"""
        self.status_bar.config(text="Starting Gazebo simulation...")
        threading.Thread(target=self._start_gazebo_thread, daemon=True).start()
        
    def _start_gazebo_thread(self):
        """Start Gazebo in background thread"""
        try:
            # This would be replaced with actual Gazebo startup commands
            time.sleep(3)  # Simulate startup time
            self.root.after(0, lambda: self.status_bar.config(text="Gazebo startup attempted"))
        except Exception as e:
            self.root.after(0, lambda: self.status_bar.config(text=f"Gazebo startup failed: {e}"))
            
    def stop_all(self):
        """Stop all systems"""
        self.status_bar.config(text="Stopping all systems...")
        # This would include actual shutdown commands
        
def main():
    """Main function"""
    print("🚜 Starting Tractobots Live Dashboard...")
    
    root = tk.Tk()
    app = TractobotsDashboard(root)
    
    print("✅ Dashboard created successfully!")
    print("📊 Live data visualization active")
    print("🎮 Control buttons ready")
    print("🔄 Status monitoring started")
    print("\n🚀 Dashboard is now running!")
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("\n🛑 Dashboard closed by user")
    except Exception as e:
        print(f"\n❌ Dashboard error: {e}")
        
if __name__ == "__main__":
    main()
