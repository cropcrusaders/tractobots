#!/usr/bin/env python3
"""
GUI Launcher for Tractobots
Easy way to launch different GUI interfaces
"""

import sys
import subprocess
import argparse

def launch_gui(gui_type):
    """Launch the specified GUI type"""
    
    commands = {
        'simple': 'ros2 run tractobots_mission_ui mission_gui_node',
        'enhanced': 'ros2 run tractobots_mission_ui enhanced_gui', 
        'web': 'ros2 run tractobots_mission_ui web_dashboard',
        'qt': 'ros2 run tractobots_mission_ui qt_gui'
    }
    
    if gui_type not in commands:
        print(f"❌ Unknown GUI type: {gui_type}")
        print(f"Available options: {', '.join(commands.keys())}")
        return False
    
    print(f"🚜 Launching {gui_type.upper()} GUI for Tractobots...")
    
    if gui_type == 'web':
        print("🌐 Web dashboard will be available at: http://localhost:5000")
    
    try:
        subprocess.run(commands[gui_type], shell=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"❌ Failed to launch GUI: {e}")
        return False
    except KeyboardInterrupt:
        print("\n🛑 GUI stopped by user")
        return True
    
    return True

def main():
    parser = argparse.ArgumentParser(description='Launch Tractobots GUI interfaces')
    parser.add_argument('gui_type', 
                       choices=['simple', 'enhanced', 'web', 'qt'],
                       help='Type of GUI to launch')
    
    args = parser.parse_args()
    
    print("🚜 TRACTOBOTS GUI LAUNCHER")
    print("=" * 40)
    
    gui_descriptions = {
        'simple': '📱 Simple Tkinter GUI - Basic mission controls',
        'enhanced': '🎨 Enhanced Tkinter GUI - Modern interface with real-time data',
        'web': '🌐 Web Dashboard - Browser-based interface with live updates',
        'qt': '💎 Professional Qt GUI - Advanced interface with plots and 3D visualization'
    }
    
    print(f"Selected: {gui_descriptions[args.gui_type]}")
    print()
    
    success = launch_gui(args.gui_type)
    
    if success:
        print("✅ GUI session completed successfully")
    else:
        print("❌ GUI session failed")
        sys.exit(1)

if __name__ == '__main__':
    main()
