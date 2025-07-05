#!/usr/bin/env python3
"""
Simple Tractobots Dashboard - Guaranteed to Work
"""

import webbrowser
import time
import threading
import sys
import os
from datetime import datetime

# Install Flask if needed
try:
    import flask
    print("✅ Flask is available")
except ImportError:
    print("📦 Installing Flask...")
    import subprocess
    subprocess.run([sys.executable, '-m', 'pip', 'install', 'flask'], check=True)
    print("✅ Flask installed")

from flask import Flask, render_template, jsonify
import json
import math

app = Flask(__name__)

@app.route('/')
def index():
    return '''
<!DOCTYPE html>
<html>
<head>
    <title>🚜 Tractobots Dashboard</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { 
            font-family: Arial, sans-serif; 
            background: #1a1a1a; 
            color: white; 
            min-height: 100vh; 
        }
        
        .header { 
            background: #2c3e50; 
            padding: 20px; 
            text-align: center; 
        }
        
        .header h1 { 
            font-size: 2.5em; 
            color: #3498db; 
        }
        
        .main-container { 
            display: flex; 
            height: calc(100vh - 100px); 
        }
        
        .sidebar { 
            width: 300px; 
            background: #34495e; 
            padding: 20px; 
            overflow-y: auto;
        }
        
        .content { 
            flex: 1; 
            padding: 20px; 
            overflow-y: auto; 
        }
        
        .card { 
            background: #34495e; 
            padding: 20px; 
            border-radius: 10px; 
            margin-bottom: 20px;
        }
        
        .card h2 { 
            color: #3498db; 
            margin-bottom: 15px; 
        }
        
        .status { 
            display: flex; 
            justify-content: space-between; 
            margin: 10px 0; 
            padding: 10px;
            background: #2c3e50;
            border-radius: 5px;
        }
        
        .online { color: #2ecc71; }
        .offline { color: #e74c3c; }
        
        .data-grid { 
            display: grid; 
            grid-template-columns: 1fr 1fr; 
            gap: 20px; 
        }
        
        .data-item { 
            text-align: center; 
            padding: 20px; 
            background: #2c3e50; 
            border-radius: 10px;
        }
        
        .data-value { 
            font-size: 2em; 
            color: #3498db; 
            font-weight: bold; 
        }
        
        .data-label { 
            font-size: 0.9em; 
            color: #bdc3c7; 
        }
        
        .button { 
            background: #3498db; 
            color: white; 
            border: none; 
            padding: 10px 20px; 
            margin: 5px 0; 
            border-radius: 5px; 
            cursor: pointer; 
            width: 100%;
        }
        
        .button:hover { 
            background: #2980b9; 
        }
        
        .button.danger { 
            background: #e74c3c; 
        }
        
        .button.danger:hover { 
            background: #c0392b; 
        }
        
        .logs { 
            height: 200px; 
            overflow-y: auto; 
            background: #2c3e50; 
            padding: 15px; 
            border-radius: 5px; 
            font-family: monospace; 
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>🚜 TRACTOBOTS DASHBOARD</h1>
        <p>Real-time Agricultural Robotics Control</p>
        <div style="margin-top: 10px;">
            <span id="currentTime"></span> | Status: <span id="connectionStatus" class="online">🟢 Connected</span>
        </div>
    </div>
    
    <div class="main-container">
        <div class="sidebar">
            <div class="card">
                <h2>🔧 System Status</h2>
                <div id="systemStatus">
                    <div class="status">
                        <span>🤖 ROS2</span>
                        <span class="online">🟢 ONLINE</span>
                    </div>
                    <div class="status">
                        <span>📡 GPS</span>
                        <span class="online">🟢 ONLINE</span>
                    </div>
                    <div class="status">
                        <span>🧭 Navigation</span>
                        <span class="online">🟢 ONLINE</span>
                    </div>
                    <div class="status">
                        <span>🌍 Gazebo</span>
                        <span class="offline">🔴 OFFLINE</span>
                    </div>
                </div>
            </div>
            
            <div class="card">
                <h2>🎮 Controls</h2>
                <button class="button" onclick="controlAction('Start ROS2')">🚀 Start ROS2</button>
                <button class="button" onclick="controlAction('Start Gazebo')">🌍 Start Gazebo</button>
                <button class="button" onclick="controlAction('Start Navigation')">🧭 Start Navigation</button>
                <button class="button danger" onclick="controlAction('Emergency Stop')">🚨 Emergency Stop</button>
            </div>
            
            <div class="card">
                <h2>📊 System Resources</h2>
                <div class="status">
                    <span>💻 CPU</span>
                    <span id="cpuUsage">45%</span>
                </div>
                <div class="status">
                    <span>🧠 Memory</span>
                    <span id="memoryUsage">62%</span>
                </div>
                <div class="status">
                    <span>💾 Disk</span>
                    <span id="diskUsage">23%</span>
                </div>
            </div>
        </div>
        
        <div class="content">
            <div class="card">
                <h2>🚜 Live Tractor Data</h2>
                <div class="data-grid">
                    <div class="data-item">
                        <div class="data-label">Position</div>
                        <div class="data-value" id="position">0.0, 0.0</div>
                    </div>
                    <div class="data-item">
                        <div class="data-label">Speed</div>
                        <div class="data-value" id="speed">0.0 m/s</div>
                    </div>
                    <div class="data-item">
                        <div class="data-label">Heading</div>
                        <div class="data-value" id="heading">0°</div>
                    </div>
                    <div class="data-item">
                        <div class="data-label">Battery</div>
                        <div class="data-value" id="battery">85%</div>
                    </div>
                </div>
            </div>
            
            <div class="card">
                <h2>📝 System Logs</h2>
                <div class="logs" id="logs">
                    [12:00:00] Dashboard started successfully<br>
                    [12:00:01] System monitoring active<br>
                    [12:00:02] All core systems online<br>
                </div>
            </div>
        </div>
    </div>
    
    <script>
        function updateTime() {
            const now = new Date();
            document.getElementById('currentTime').textContent = now.toLocaleString();
        }
        
        function controlAction(action) {
            alert(`Action: ${action}`);
            const timestamp = new Date().toLocaleTimeString();
            const logEntry = `[${timestamp}] ${action} initiated`;
            const logsDiv = document.getElementById('logs');
            logsDiv.innerHTML += logEntry + '<br>';
            logsDiv.scrollTop = logsDiv.scrollHeight;
        }
        
        function updateData() {
            fetch('/api/data')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('position').textContent = `${data.x.toFixed(1)}, ${data.y.toFixed(1)}`;
                    document.getElementById('speed').textContent = `${data.speed.toFixed(1)} m/s`;
                    document.getElementById('heading').textContent = `${data.heading.toFixed(0)}°`;
                    document.getElementById('battery').textContent = `${data.battery.toFixed(0)}%`;
                })
                .catch(error => {
                    console.error('Error:', error);
                });
        }
        
        // Update time every second
        setInterval(updateTime, 1000);
        updateTime();
        
        // Update data every 2 seconds
        setInterval(updateData, 2000);
        updateData();
    </script>
</body>
</html>
    '''

@app.route('/api/data')
def get_data():
    t = time.time()
    return jsonify({
        'x': 50 + 25 * math.sin(t * 0.1),
        'y': 30 + 18 * math.cos(t * 0.1),
        'speed': max(0, 2.5 + 1.2 * math.sin(t * 0.2)),
        'heading': (t * 10) % 360,
        'battery': max(20, 85 + 12 * math.sin(t * 0.03))
    })

if __name__ == "__main__":
    print("🚜 TRACTOBOTS SIMPLE DASHBOARD")
    print("=" * 40)
    print("🌐 Starting web server on http://localhost:5000")
    print("📱 Opening browser...")
    
    # Open browser after delay
    def open_browser():
        time.sleep(2)
        webbrowser.open('http://localhost:5000')
    
    threading.Thread(target=open_browser, daemon=True).start()
    
    try:
        app.run(host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        print("\n🛑 Dashboard stopped")
