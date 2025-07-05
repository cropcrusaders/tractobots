#!/usr/bin/env python3
"""
Tractobots Live Terminal Dashboard
Real-time system monitoring in the terminal
"""

import sys
import os
import subprocess
import time
import threading
from datetime import datetime
import json

class TractobotsCLIDashboard:
    def __init__(self):
        self.running = True
        self.system_status = {
            'ros2': False,
            'gazebo': False,
            'navigation': False,
            'gps': False,
            'simulation': False
        }
        self.stats = {
            'uptime': 0.0,
            'cpu_usage': 0.0,
            'memory_usage': 0.0,
            'disk_usage': 0.0,
            'network_activity': 0.0
        }
        self.ros2_nodes = 0
        
    def clear_screen(self):
        """Clear the terminal screen"""
        os.system('clear' if os.name == 'posix' else 'cls')
        
    def check_system_status(self):
        """Check the status of all systems"""
        # Check ROS2
        try:
            result = subprocess.run(['ros2', 'node', 'list'], 
                                  capture_output=True, text=True, timeout=2)
            self.system_status['ros2'] = result.returncode == 0
            if self.system_status['ros2']:
                node_count = len(result.stdout.strip().split('\n')) if result.stdout.strip() else 0
                self.ros2_nodes = node_count
        except:
            self.system_status['ros2'] = False
            self.ros2_nodes = 0
            
        # Check Gazebo
        try:
            result = subprocess.run(['pgrep', '-f', 'gazebo'], 
                                  capture_output=True, text=True, timeout=2)
            self.system_status['gazebo'] = len(result.stdout.strip()) > 0
        except:
            self.system_status['gazebo'] = False
            
        # Check navigation (simplified)
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                  capture_output=True, text=True, timeout=2)
            self.system_status['navigation'] = '/cmd_vel' in result.stdout
        except:
            self.system_status['navigation'] = False
            
        # Check GPS (simplified)
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                  capture_output=True, text=True, timeout=2)
            self.system_status['gps'] = '/fix' in result.stdout or '/gps' in result.stdout
        except:
            self.system_status['gps'] = False
            
    def get_system_stats(self):
        """Get system statistics"""
        try:
            # CPU usage
            result = subprocess.run(['top', '-bn1'], capture_output=True, text=True, timeout=3)
            for line in result.stdout.split('\n'):
                if 'Cpu(s)' in line:
                    # Extract CPU usage percentage
                    parts = line.split(',')
                    for part in parts:
                        if 'us' in part:
                            self.stats['cpu_usage'] = float(part.split('%')[0].strip())
                            break
        except:
            pass
            
        try:
            # Memory usage
            result = subprocess.run(['free', '-m'], capture_output=True, text=True, timeout=2)
            lines = result.stdout.split('\n')
            if len(lines) > 1:
                mem_line = lines[1].split()
                if len(mem_line) > 2:
                    total_mem = int(mem_line[1])
                    used_mem = int(mem_line[2])
                    self.stats['memory_usage'] = (used_mem / total_mem) * 100
        except:
            pass
            
        try:
            # Disk usage
            result = subprocess.run(['df', '-h', '/'], capture_output=True, text=True, timeout=2)
            lines = result.stdout.split('\n')
            if len(lines) > 1:
                disk_line = lines[1].split()
                if len(disk_line) > 4:
                    usage_str = disk_line[4].replace('%', '')
                    self.stats['disk_usage'] = int(usage_str)
        except:
            pass
            
    def display_dashboard(self):
        """Display the dashboard in terminal"""
        self.clear_screen()
        
        # Header
        print("=" * 80)
        print("🚜 TRACTOBOTS LIVE TERMINAL DASHBOARD")
        print("=" * 80)
        print(f"Last Update: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print()
        
        # System Status
        print("📊 SYSTEM STATUS")
        print("-" * 40)
        
        status_symbols = {True: "🟢", False: "🔴"}
        
        for system, status in self.system_status.items():
            if system.endswith('_nodes'):
                continue
            symbol = status_symbols[status]
            status_text = "ONLINE" if status else "OFFLINE"
            print(f"{symbol} {system.upper():12} : {status_text}")
            
        if hasattr(self, 'ros2_nodes'):
            print(f"   ROS2 Nodes: {self.ros2_nodes}")
            
        print()
        
        # System Statistics
        print("📈 SYSTEM STATISTICS")
        print("-" * 40)
        print(f"CPU Usage    : {self.stats['cpu_usage']:.1f}%")
        print(f"Memory Usage : {self.stats['memory_usage']:.1f}%")
        print(f"Disk Usage   : {self.stats['disk_usage']:.1f}%")
        print()
        
        # Live Data Simulation
        print("🎯 LIVE TRACTOR DATA")
        print("-" * 40)
        
        # Simulate some live data
        import math
        t = time.time()
        
        # Simulated position
        x = 50 + 20 * math.sin(t * 0.1)
        y = 30 + 15 * math.cos(t * 0.1)
        
        # Simulated speed
        speed = 2.5 + 0.5 * math.sin(t * 0.2)
        
        # Simulated battery
        battery = 85 + 10 * math.sin(t * 0.05)
        
        # Simulated heading
        heading = (t * 10) % 360
        
        print(f"Position     : ({x:.1f}, {y:.1f}) meters")
        print(f"Speed        : {speed:.1f} m/s")
        print(f"Heading      : {heading:.1f}°")
        print(f"Battery      : {battery:.1f}%")
        print(f"GPS Status   : {'LOCKED' if self.system_status['gps'] else 'SEARCHING'}")
        print()
        
        # Control Commands
        print("🎮 AVAILABLE COMMANDS")
        print("-" * 40)
        print("Press 'q' to quit")
        print("Press 'r' to restart ROS2")
        print("Press 'g' to start Gazebo")
        print("Press 's' to stop all systems")
        print()
        
        # Footer
        print("=" * 80)
        print("Dashboard running... (Updates every 2 seconds)")
        
    def run_dashboard(self):
        """Run the dashboard loop"""
        print("🚀 Starting Tractobots Terminal Dashboard...")
        print("Press Ctrl+C to exit")
        print()
        
        try:
            while self.running:
                self.check_system_status()
                self.get_system_stats()
                self.display_dashboard()
                time.sleep(2)
                
        except KeyboardInterrupt:
            print("\n\n🛑 Dashboard stopped by user")
            self.running = False
            
        except Exception as e:
            print(f"\n\n❌ Dashboard error: {e}")
            self.running = False
            
    def start_ros2(self):
        """Start ROS2 system"""
        print("🚀 Starting ROS2 system...")
        # This would include actual ROS2 startup commands
        
    def start_gazebo(self):
        """Start Gazebo simulation"""
        print("🚀 Starting Gazebo simulation...")
        # This would include actual Gazebo startup commands
        
    def stop_all(self):
        """Stop all systems"""
        print("🛑 Stopping all systems...")
        # This would include actual shutdown commands

def main():
    """Main function"""
    dashboard = TractobotsCLIDashboard()
    dashboard.run_dashboard()

if __name__ == "__main__":
    main()
