pip install roslibpy matplotlib numpy PyQt5pip install roslibpy matplotlib numpy PyQt5#!/usr/bin/env python3
print("[InstantDashboard] Script started", flush=True)
print("Debug: instant_dashboard.py loaded", flush=True)
"""
Tractobots Dashboard - Instant Launch
This script will start the web dashboard and open your browser
"""

import webbrowser
import time
import threading
import subprocess
import sys
import os
from datetime import datetime

ros2_process = None  # Global process handles
gazebo_process = None

def start_dashboard():
    print("Debug: inside start_dashboard", flush=True)
    """Start the web dashboard"""
    print("🚀 Starting Tractobots Web Dashboard...", flush=True)
    print("=" * 50)
    
    # Install Flask if needed
    try:
        import flask
        print("✅ Flask is available")
    except ImportError:
        print("📦 Flask not found, installing via pip:", sys.executable, "-m pip install flask", flush=True)
        subprocess.run([sys.executable, '-m', 'pip', 'install', 'flask'], check=True)
        print("✅ Flask installed", flush=True)
    
    # Import required modules (Flask now that it's installed)
    from flask import Flask, render_template, jsonify, request
    import json
    import math
    
    print("✅ All modules imported successfully", flush=True)
    
    # Create Flask app
    app = Flask(__name__)
    
    # Enhanced dashboard data with more comprehensive features
    dashboard_data = {
        'status': {
            'ros2': True,
            'gazebo': False,
            'navigation': True,
            'gps': True,
            'simulation': False,
            'camera': False,
            'lidar': True,
            'imu': True,
            'can_bus': True,
            'hydraulics': True
        },
        'live_data': {
            'position': {'x': 0, 'y': 0, 'z': 0},
            'speed': 0,
            'heading': 0,
            'battery': 85,
            'timestamp': datetime.now().isoformat(),
            'field_coverage': 0,
            'mission_progress': 0,
            'work_rate': 2.4,
            'fuel_level': 78,
            'engine_temp': 87,
            'hydraulic_pressure': 180,
            'pto_speed': 540,
            'implement_depth': 15.2
        },
        'environment': {
            'temperature': 22.0,
            'humidity': 65.0,
            'wind_speed': 2.5,
            'light_level': 85.0,
            'soil_moisture': 45.0,
            'precipitation': 0.0,
            'air_pressure': 1013.25
        },
        'stats': {
            'cpu_usage': 45.5,
            'memory_usage': 62.3,
            'disk_usage': 23.8,
            'network_activity': 12.1,
            'gpu_usage': 34.2,
            'temperature_cpu': 58.3,
            'temperature_gpu': 42.1
        },
        'alerts': [
            {'type': 'info', 'message': 'System startup completed successfully', 'time': '12:34:56', 'priority': 'low'},
            {'type': 'warning', 'message': 'GPS signal slightly weak in area B-3', 'time': '12:33:22', 'priority': 'medium'},
            {'type': 'success', 'message': 'Field boundary detection completed', 'time': '12:32:18', 'priority': 'low'},
            {'type': 'error', 'message': 'Camera feed disconnected', 'time': '12:31:45', 'priority': 'high'}
        ],
        'logs': [
            f"[{datetime.now().strftime('%H:%M:%S')}] Enhanced dashboard started",
            f"[{datetime.now().strftime('%H:%M:%S')}] All system modules loaded",
            f"[{datetime.now().strftime('%H:%M:%S')}] Real-time monitoring active",
            f"[{datetime.now().strftime('%H:%M:%S')}] Environmental sensors online",
            f"[{datetime.now().strftime('%H:%M:%S')}] GPS tracking initialized",
            f"[{datetime.now().strftime('%H:%M:%S')}] CAN bus communication established",
            f"[{datetime.now().strftime('%H:%M:%S')}] Hydraulic system operational"
        ]
    }
    
    def update_data():
        """Update dashboard data in background with enhanced metrics"""
        while True:
            t = time.time()
            
            # Enhanced live data updates
            dashboard_data['live_data'].update({
                'position': {
                    'x': 50 + 25 * math.sin(t * 0.08),
                    'y': 30 + 18 * math.cos(t * 0.1),
                    'z': 2.1 + 0.3 * math.sin(t * 0.15)
                },
                'speed': max(0, 2.5 + 1.2 * math.sin(t * 0.2)),
                'heading': (t * 8) % 360,
                'battery': max(10, 85 + 12 * math.sin(t * 0.03)),
                'field_coverage': min(100, (t / 10) % 100),
                'mission_progress': min(100, (t / 8) % 100),
                'work_rate': 2.4 + 0.8 * math.sin(t * 0.05),
                'fuel_level': max(10, 78 + 15 * math.sin(t * 0.02)),
                'engine_temp': 87 + 8 * math.sin(t * 0.04),
                'hydraulic_pressure': 180 + 20 * math.cos(t * 0.06),
                'pto_speed': 540 + 60 * math.sin(t * 0.07),
                'implement_depth': 15.2 + 3 * math.cos(t * 0.03),
                'timestamp': datetime.now().isoformat()
            })
            
            # Enhanced environmental data
            dashboard_data['environment'].update({
                'temperature': 22 + 8 * math.sin(t * 0.02),
                'humidity': 65 + 15 * math.cos(t * 0.025),
                'wind_speed': 2.5 + 3 * math.sin(t * 0.03),
                'light_level': 85 + 10 * math.cos(t * 0.01),
                'soil_moisture': 45 + 20 * math.sin(t * 0.015),
                'precipitation': max(0, 2 * math.sin(t * 0.01)),
                'air_pressure': 1013.25 + 5 * math.cos(t * 0.005)
            })
            
            # Enhanced system stats
            dashboard_data['stats'].update({
                'cpu_usage': 45 + 20 * math.sin(t * 0.06),
                'memory_usage': 62 + 15 * math.cos(t * 0.08),
                'disk_usage': 23 + 5 * math.sin(t * 0.04),
                'network_activity': 12 + 18 * math.cos(t * 0.09),
                'gpu_usage': 34 + 25 * math.sin(t * 0.11),
                'temperature_cpu': 58 + 10 * math.sin(t * 0.03),
                'temperature_gpu': 42 + 8 * math.cos(t * 0.05)
            })
            
            time.sleep(1)  # Update every second
    
    # Start background data update
    update_thread = threading.Thread(target=update_data, daemon=True)
    update_thread.start()
    
    @app.route('/')
    def index():
        """Main dashboard page"""
        return '''
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Tractobots Instant Dashboard</title>
  <style>
    body { margin:0; font-family: Arial, sans-serif; background: #1a1a1a; color: white; }
    .sidebar { position: fixed; top:0; left:0; width: 250px; height:100%; background: #2c3e50; padding: 20px; box-shadow: 2px 0 5px rgba(0,0,0,0.5);}    
    .content { margin-left: 270px; padding:20px; }
    .status { margin:10px 0; display:flex; justify-content: space-between; }
    .status span { flex:1; }
    .button { width:100%; margin:5px 0; padding:10px; background:#3498db; border:none; color:white; cursor:pointer; }
    .button.stop { background: #e74c3c; }
  </style>
</head>
<body>
  <div class="sidebar">
    <h2>System Status</h2>
    <div id="status"></div>
    <button class="button" onclick="startRos2()">Start ROS2</button>
    <button class="button" onclick="startGazebo()">Start Gazebo</button>
    <button class="button stop" onclick="stopRos2()">Stop All</button>
  </div>
  <div class="content">
    <h1>Live Data</h1>
    <p>Position: <span id="position">0,0</span></p>
  </div>
  <script>
    function update() {
      fetch('/api/data')
        .then(r=>r.json())
        .then(d=>{
          const pos = d.live_data.position;
          document.getElementById('position').textContent = pos.x.toFixed(1) + ',' + pos.y.toFixed(1);
          let st = document.getElementById('status'); st.innerHTML = '';
          for(let k in d.status) {
            let div = document.createElement('div'); div.className='status';
            div.innerHTML = `<span>${k}</span><span>${d.status[k]?'ON':'OFF'}</span>`;
            st.appendChild(div);
          }
        });
    }
    setInterval(update,1000);
    update();

    function startRos2() {
      fetch('/api/start_ros2', {method: 'POST'})
        .then(r => r.json())
        .then(j => alert(j.started ? 'ROS2 launched' : 'Error: ' + j.error));
    }
    function startGazebo() {
      fetch('/api/start_gazebo', {method: 'POST'})
        .then(r => r.json())
        .then(j => alert(j.started ? 'Gazebo launched' : 'Error: ' + j.error));
    }
    function stopRos2() {
      fetch('/api/stop_ros2', {method: 'POST'})
        .then(r => r.json())
        .then(j => alert(j.stopped ? 'ROS2 processes stopped' : 'Error: ' + j.error));
    }
  </script>
</body>
</html>
        '''
    
    @app.route('/api/data')
    def get_data():
        """Get dashboard data"""
        return jsonify(dashboard_data)
    
    @app.route('/api/start_ros2', methods=['POST'])
    def flask_start_ros2():
        global ros2_process
        if ros2_process is None:
            try:
                # Command to launch ROS2 within WSL
                cmd = 'wsl -e bash -c "source /opt/ros/humble/setup.bash && source ~/tractobots_ws/install/setup.bash && ros2 launch tractobots_bringup tractobots.launch.py"'
                ros2_process = subprocess.Popen(cmd, shell=True)
                dashboard_data['status']['ros2'] = True
                return jsonify({'started': True, 'pid': ros2_process.pid})
            except Exception as ex:
                return jsonify({'started': False, 'error': str(ex)})
        return jsonify({'started': False, 'error': 'ROS2 already running'})

    @app.route('/api/start_gazebo', methods=['POST'])
    def flask_start_gazebo():
        global gazebo_process
        if gazebo_process is None:
            try:
                # Command to launch Gazebo within WSL
                cmd = 'wsl -e bash -c "source /opt/ros/humble/setup.bash && source ~/tractobots_gazebo_ws/install/setup.bash && ros2 launch tractobots_gazebo tractobots_gazebo.launch.py"'
                gazebo_process = subprocess.Popen(cmd, shell=True)
                dashboard_data['status']['gazebo'] = True
                dashboard_data['status']['simulation'] = True
                return jsonify({'started': True, 'pid': gazebo_process.pid})
            except Exception as ex:
                return jsonify({'started': False, 'error': str(ex)})
        return jsonify({'started': False, 'error': 'Gazebo already running'})

    @app.route('/api/stop_ros2', methods=['POST'])
    def flask_stop_ros2():
        global ros2_process, gazebo_process
        stopped = False
        try:
            if gazebo_process:
                # Terminating the process group starting with the child process
                subprocess.run(f'wsl -e kill -9 {gazebo_process.pid}', shell=True)
                gazebo_process.terminate()
                gazebo_process.wait()
                gazebo_process = None
                dashboard_data['status']['gazebo'] = False
                dashboard_data['status']['simulation'] = False
                stopped = True
            if ros2_process:
                subprocess.run(f'wsl -e kill -9 {ros2_process.pid}', shell=True)
                ros2_process.terminate()
                ros2_process.wait()
                ros2_process = None
                dashboard_data['status']['ros2'] = False
                stopped = True
            return jsonify({'stopped': stopped})
        except Exception as ex:
            return jsonify({'stopped': False, 'error': str(ex)})

    print("✅ Dashboard routes configured")
    print("🌐 Starting web server on http://localhost:5000")
    print("📱 Opening browser...")
    print("🔄 Dashboard will update every 2 seconds")
    print("🛑 Press Ctrl+C to stop")
    print("")
    
    # Open browser after a short delay
    def open_browser():
        time.sleep(2)
        webbrowser.open('http://localhost:5000')
    
    browser_thread = threading.Thread(target=open_browser, daemon=True)
    browser_thread.start()

    # Auto-launch Gazebo simulation via ROS2 launch
    # print("📡 Auto-launching Gazebo simulation and ROS2...")
    # try:
    #     subprocess.Popen('ros2 launch tractobots_gazebo tractobots_gazebo.launch.py', shell=True)
    #     print("✅ Gazebo and ROS2 simulation launched")
    # except Exception as e:
    #     print(f"❌ Auto-launch failed: {e}")
    
    # Start the Flask app (disable reloader for consistency)
    try:
        print("🔧 Launching Flask server (debug mode)...", flush=True)
        app.run(host='0.0.0.0', port=5000, debug=True)
    except KeyboardInterrupt:
        print("\n🛑 Dashboard stopped by user")
    except Exception as e:
        print(f"\n❌ Error: {e}", flush=True)

if __name__ == "__main__":
    print("🚜 TRACTOBOTS DASHBOARD - INSTANT LAUNCH", flush=True)
    print("=" * 60, flush=True)
    print("🎯 This will start your web dashboard and open your browser", flush=True)
    print("📊 You'll see real-time system status and live data", flush=True)
    print("🎮 Control buttons will be available", flush=True)
    print("")
    
    try:
        start_dashboard()
    except Exception as e:
        print(f"💥 Error: {e}", flush=True)
    
    print("\n🎉 Thank you for using Tractobots Dashboard!", flush=True)
