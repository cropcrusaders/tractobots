#!/usr/bin/env python3
"""
GUI Demo and Test Script
Quick test of all GUI interfaces without requiring full ROS2 system
"""

import time
import threading
import random

class MockNode:
    """Mock ROS2 node for testing GUIs without full system"""
    
    def __init__(self):
        self.gps_data = {"lat": 40.7128, "lon": -74.0060, "alt": 10.0}  # NYC coordinates
        self.speed = 0.0
        self.heading = 0.0
        self.mission_active = False
        self.emergency_stop = False
        
        # Start data simulation
        self.running = True
        self.sim_thread = threading.Thread(target=self.simulate_data, daemon=True)
        self.sim_thread.start()
    
    def simulate_data(self):
        """Simulate realistic tractor sensor data"""
        while self.running:
            if self.mission_active:
                # Simulate movement
                self.speed = random.uniform(0.5, 2.0)  # 0.5-2.0 m/s
                self.heading += random.uniform(-5, 5)  # Small heading changes
                self.heading = self.heading % 360
                
                # Simulate GPS movement (very small changes)
                self.gps_data["lat"] += random.uniform(-0.0001, 0.0001)
                self.gps_data["lon"] += random.uniform(-0.0001, 0.0001)
            else:
                self.speed = max(0, self.speed - 0.1)  # Slow down when stopped
            
            time.sleep(0.5)
    
    def start_mission(self):
        if not self.emergency_stop:
            self.mission_active = True
            print("🚀 Mission started (demo mode)")
            return True
        return False
    
    def stop_mission(self):
        self.mission_active = False
        print("⏹️ Mission stopped (demo mode)")
    
    def trigger_estop(self):
        self.emergency_stop = True
        self.mission_active = False
        print("🛑 Emergency stop triggered (demo mode)")
    
    def reset_estop(self):
        self.emergency_stop = False
        print("🔄 Emergency stop reset (demo mode)")
    
    def send_cmd_vel(self, linear, angular):
        print(f"🎮 Manual control: linear={linear:.1f}, angular={angular:.1f}")


def test_enhanced_gui():
    """Test the enhanced Tkinter GUI"""
    try:
        import tkinter as tk
        from tkinter import ttk
        
        print("🎨 Testing Enhanced GUI...")
        
        # Create mock node
        mock_node = MockNode()
        
        # Simple test window
        root = tk.Tk()
        root.title("🚜 Enhanced GUI Test")
        root.geometry("400x300")
        root.configure(bg='#1a1a1a')
        
        # Test interface
        ttk.Label(root, text="🚜 Enhanced GUI Test", font=('Arial', 16, 'bold')).pack(pady=20)
        
        status_var = tk.StringVar(value="🟡 DEMO MODE")
        ttk.Label(root, textvariable=status_var, font=('Arial', 12)).pack(pady=10)
        
        def update_status():
            if mock_node.emergency_stop:
                status_var.set("🔴 EMERGENCY STOP")
            elif mock_node.mission_active:
                status_var.set("🟢 MISSION ACTIVE")
            else:
                status_var.set("🟡 READY")
            root.after(1000, update_status)
        
        ttk.Button(root, text="🚀 Start Mission", command=mock_node.start_mission).pack(pady=5)
        ttk.Button(root, text="⏹️ Stop Mission", command=mock_node.stop_mission).pack(pady=5)
        ttk.Button(root, text="🛑 Emergency Stop", command=mock_node.trigger_estop).pack(pady=5)
        ttk.Button(root, text="🔄 Reset E-Stop", command=mock_node.reset_estop).pack(pady=5)
        
        update_status()
        
        print("✅ Enhanced GUI test window opened")
        print("🎯 Test the buttons to see mock responses")
        
        root.mainloop()
        
    except ImportError as e:
        print(f"❌ Enhanced GUI test failed: {e}")
    except Exception as e:
        print(f"❌ Enhanced GUI error: {e}")


def test_web_dashboard():
    """Test web dashboard components"""
    try:
        print("🌐 Testing Web Dashboard components...")
        
        # Test Flask imports
        from flask import Flask
        print("✅ Flask available")
        
        # Test SocketIO
        from flask_socketio import SocketIO
        print("✅ SocketIO available")
        
        # Create simple test app
        app = Flask(__name__)
        
        @app.route('/')
        def test_page():
            return """
            <html>
            <head><title>🚜 Tractobots Dashboard Test</title></head>
            <body style="background: #1a1a1a; color: white; font-family: Arial; text-align: center; padding: 50px;">
                <h1>🚜 TRACTOBOTS WEB DASHBOARD TEST</h1>
                <p>✅ Web server is working!</p>
                <p>🎯 This is a test page to verify web dashboard functionality</p>
                <button onclick="alert('🚀 Test button clicked!')" style="padding: 10px 20px; font-size: 16px;">🚀 Test Button</button>
            </body>
            </html>
            """
        
        print("✅ Web dashboard test server ready")
        print("🌐 Starting test server at http://localhost:5001")
        print("🎯 Open your browser to test the web interface")
        print("⏹️ Press Ctrl+C to stop")
        
        app.run(host='0.0.0.0', port=5001, debug=False)
        
    except ImportError as e:
        print(f"❌ Web dashboard test failed - missing dependency: {e}")
        print("💡 Install with: pip install flask flask-socketio")
    except Exception as e:
        print(f"❌ Web dashboard error: {e}")


def test_qt_gui():
    """Test Qt GUI components"""
    try:
        print("💎 Testing Qt GUI components...")
        
        from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget, QPushButton
        from PyQt5.QtCore import Qt
        import sys
        
        print("✅ PyQt5 available")
        
        app = QApplication(sys.argv)
        
        window = QMainWindow()
        window.setWindowTitle("🚜 Qt GUI Test")
        window.setGeometry(300, 300, 400, 300)
        
        central_widget = QWidget()
        window.setCentralWidget(central_widget)
        
        layout = QVBoxLayout(central_widget)
        
        title = QLabel("🚜 Qt GUI Test")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 20px; font-weight: bold; color: #00ff88; margin: 20px;")
        layout.addWidget(title)
        
        status = QLabel("✅ Qt GUI is working!")
        status.setAlignment(Qt.AlignCenter)
        status.setStyleSheet("font-size: 14px; margin: 10px;")
        layout.addWidget(status)
        
        test_btn = QPushButton("🎯 Test Button")
        test_btn.clicked.connect(lambda: status.setText("🚀 Button clicked!"))
        test_btn.setStyleSheet("padding: 10px; font-size: 14px; font-weight: bold;")
        layout.addWidget(test_btn)
        
        window.show()
        
        print("✅ Qt GUI test window opened")
        print("🎯 Test the button to verify functionality")
        
        sys.exit(app.exec_())
        
    except ImportError as e:
        print(f"❌ Qt GUI test failed - missing dependency: {e}")
        print("💡 Install with: pip install PyQt5")
    except Exception as e:
        print(f"❌ Qt GUI error: {e}")


def main():
    """Main test function"""
    print("🚜 TRACTOBOTS GUI DEMO & TEST")
    print("=" * 40)
    print("🎯 This script tests GUI components without requiring full ROS2 system")
    print()
    
    while True:
        print("Choose a GUI to test:")
        print("1. 🎨 Enhanced Tkinter GUI")
        print("2. 🌐 Web Dashboard")
        print("3. 💎 Professional Qt GUI")
        print("4. 🔧 Check all dependencies")
        print("5. ❌ Exit")
        
        choice = input("\nEnter your choice (1-5): ").strip()
        
        if choice == '1':
            test_enhanced_gui()
        elif choice == '2':
            test_web_dashboard()
        elif choice == '3':
            test_qt_gui()
        elif choice == '4':
            check_dependencies()
        elif choice == '5':
            print("👋 Goodbye!")
            break
        else:
            print("❌ Invalid choice. Please try again.")


def check_dependencies():
    """Check all GUI dependencies"""
    print("\n🔧 Checking GUI Dependencies...")
    print("-" * 30)
    
    # Check tkinter (built-in)
    try:
        import tkinter
        print("✅ Tkinter (built-in)")
    except ImportError:
        print("❌ Tkinter (should be built-in)")
    
    # Check Flask
    try:
        import flask
        print("✅ Flask")
    except ImportError:
        print("❌ Flask - install with: pip install flask")
    
    # Check SocketIO
    try:
        import flask_socketio
        print("✅ Flask-SocketIO")
    except ImportError:
        print("❌ Flask-SocketIO - install with: pip install flask-socketio")
    
    # Check PyQt5
    try:
        import PyQt5
        print("✅ PyQt5")
    except ImportError:
        print("❌ PyQt5 - install with: pip install PyQt5")
    
    # Check pyqtgraph
    try:
        import pyqtgraph
        print("✅ PyQtGraph")
    except ImportError:
        print("❌ PyQtGraph - install with: pip install pyqtgraph")
    
    # Check numpy
    try:
        import numpy
        print("✅ NumPy")
    except ImportError:
        print("❌ NumPy - install with: pip install numpy")
    
    print("\n💡 To install all GUI dependencies:")
    print("   ./install_gui.sh")
    print("   or: pip install -r gui_requirements.txt")


if __name__ == '__main__':
    main()
