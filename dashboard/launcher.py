#!/usr/bin/env python3
"""
Tractobots Dashboard Launcher
Final version with complete integration
"""

import sys
import os
import subprocess

def check_prerequisites():
    """Check if all prerequisites are met"""
    print("🔍 Checking prerequisites...")
    
    # Check Python
    try:
        import tkinter
        print("   ✅ Python and tkinter available")
    except ImportError:
        print("   ❌ tkinter not available")
        return False
    
    # Check WSL
    try:
        result = subprocess.run(["wsl", "echo", "test"], 
                              capture_output=True, timeout=10)
        if result.returncode == 0:
            print("   ✅ WSL is accessible")
        else:
            print("   ⚠️  WSL may have issues")
    except:
        print("   ⚠️  WSL not accessible or not installed")
    
    # Check dashboard file
    dashboard_path = os.path.join(os.path.dirname(__file__), "dashboard.py")
    if os.path.exists(dashboard_path):
        print("   ✅ Dashboard file found")
    else:
        print("   ❌ Dashboard file not found")
        return False
    
    return True

def main():
    print("🚜 TRACTOBOTS WINDOWS DASHBOARD")
    print("=" * 50)
    print("Professional ROS2 Dashboard for Windows/WSL")
    print("=" * 50)
    
    if not check_prerequisites():
        print("\n❌ Prerequisites not met. Please check your setup.")
        return
    
    print("\n🎯 READY TO LAUNCH!")
    print("\n📋 FEATURES AVAILABLE:")
    print("   • Real-time ROS2 node monitoring")
    print("   • One-click robot model launch")
    print("   • WSL/ROS2 command execution")
    print("   • Professional GUI interface")
    print("   • Background process management")
    
    print("\n🚀 Starting dashboard...")
    
    try:
        # Import and run the dashboard
        from dashboard.dashboard import TractobotsDashboard
        from PyQt5.QtWidgets import QApplication
        
        app = QApplication(sys.argv)
        dashboard = TractobotsDashboard()
        dashboard.show()
        sys.exit(app.exec_())
        
    except KeyboardInterrupt:
        print("\n👋 Dashboard closed by user")
    except Exception as e:
        print(f"\n❌ Error launching dashboard: {e}")
        print("\n🔧 TROUBLESHOOTING:")
        print("   1. Ensure WSL is installed and working")
        print("   2. Check ROS2 Jazzy installation")
        print("   3. Run: python dashboard/tests/integration_test.py")
        print("   4. See: dashboard/docs/TROUBLESHOOTING.md")

if __name__ == "__main__":
    main()
