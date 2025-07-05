#!/usr/bin/env python3
"""
Tractobots Web Dashboard
A web-based graphical interface for monitoring Tractobots systems
"""

from flask import Flask, render_template, jsonify, request
import json
import time
import threading
import subprocess
from datetime import datetime
import os
import signal
import math

app = Flask(__name__)

# Global data storage
system_data = {
    'status': {
        'ros2': False,
        'gazebo': False,
        'navigation': False,
        'gps': False,
        'simulation': False
    },
    'stats': {
        'uptime': 0,
        'cpu_usage': 0,
        'memory_usage': 0,
        'disk_usage': 0,
        'network_activity': 0
    },
    'live_data': {
        'position': {'x': 0, 'y': 0},
        'speed': 0,
        'heading': 0,
        'battery': 85,
        'timestamp': datetime.now().isoformat()
    },
    'logs': []
}

def log_message(message):
    """Add a log message"""
    timestamp = datetime.now().strftime("%H:%M:%S")
    log_entry = f"[{timestamp}] {message}"
    system_data['logs'].append(log_entry)
    
    # Keep only last 100 log entries
    if len(system_data['logs']) > 100:
        system_data['logs'] = system_data['logs'][-100:]
    
    print(log_entry)

def update_system_status():
    """Update system status in background"""
    while True:
        try:
            # Check ROS2
            try:
                result = subprocess.run(['ros2', 'node', 'list'], 
                                      capture_output=True, text=True, timeout=2)
                system_data['status']['ros2'] = result.returncode == 0
            except:
                system_data['status']['ros2'] = False
                
            # Check Gazebo
            try:
                result = subprocess.run(['pgrep', '-f', 'gazebo'], 
                                      capture_output=True, text=True, timeout=2)
                system_data['status']['gazebo'] = len(result.stdout.strip()) > 0
            except:
                system_data['status']['gazebo'] = False
                
            # Update live data with simulation
            t = time.time()
            system_data['live_data'] = {
                'position': {
                    'x': 50 + 20 * math.sin(t * 0.1),
                    'y': 30 + 15 * math.cos(t * 0.1)
                },
                'speed': 2.5 + 0.5 * math.sin(t * 0.2),
                'heading': (t * 10) % 360,
                'battery': 85 + 10 * math.sin(t * 0.05),
                'timestamp': datetime.now().isoformat()
            }
            
            # Get system stats
            try:
                # CPU usage (simplified)
                result = subprocess.run(['top', '-bn1'], capture_output=True, text=True, timeout=2)
                system_data['stats']['cpu_usage'] = 45.5  # Simulated
                system_data['stats']['memory_usage'] = 62.3  # Simulated
                system_data['stats']['disk_usage'] = 23.8  # Simulated
            except:
                pass
                
        except Exception as e:
            log_message(f"Error updating system status: {e}")
            
        time.sleep(2)

# Start background thread
status_thread = threading.Thread(target=update_system_status, daemon=True)
status_thread.start()

@app.route('/')
def index():
    """Main dashboard page"""
    return render_template('dashboard.html')

@app.route('/api/status')
def get_status():
    """Get system status"""
    return jsonify(system_data)

@app.route('/api/control/<action>')
def control_action(action):
    """Control system actions"""
    if action == 'start_ros2':
        log_message("Starting ROS2 system...")
        # Add actual ROS2 startup commands here
        return jsonify({'status': 'success', 'message': 'ROS2 startup initiated'})
    elif action == 'start_gazebo':
        log_message("Starting Gazebo simulation...")
        # Add actual Gazebo startup commands here
        return jsonify({'status': 'success', 'message': 'Gazebo startup initiated'})
    elif action == 'stop_all':
        log_message("Stopping all systems...")
        # Add actual shutdown commands here
        return jsonify({'status': 'success', 'message': 'All systems stop initiated'})
    else:
        return jsonify({'status': 'error', 'message': 'Unknown action'})

# Create templates directory and HTML template
if not os.path.exists('templates'):
    os.makedirs('templates')

html_template = '''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>🚜 Tractobots Dashboard</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 0;
            background-color: #2c3e50;
            color: #ecf0f1;
        }
        .header {
            background-color: #34495e;
            padding: 20px;
            text-align: center;
            border-bottom: 3px solid #3498db;
        }
        .container {
            display: flex;
            gap: 20px;
            padding: 20px;
            max-width: 1400px;
            margin: 0 auto;
        }
        .left-panel {
            flex: 1;
            background-color: #34495e;
            padding: 20px;
            border-radius: 10px;
            max-width: 300px;
        }
        .right-panel {
            flex: 2;
            background-color: #34495e;
            padding: 20px;
            border-radius: 10px;
        }
        .status-item {
            display: flex;
            justify-content: space-between;
            padding: 10px;
            margin: 5px 0;
            background-color: #2c3e50;
            border-radius: 5px;
        }
        .status-online {
            color: #27ae60;
        }
        .status-offline {
            color: #e74c3c;
        }
        .button {
            background-color: #3498db;
            color: white;
            border: none;
            padding: 10px 20px;
            margin: 5px;
            border-radius: 5px;
            cursor: pointer;
            width: 100%;
        }
        .button:hover {
            background-color: #2980b9;
        }
        .button.danger {
            background-color: #e74c3c;
        }
        .button.danger:hover {
            background-color: #c0392b;
        }
        .button.success {
            background-color: #27ae60;
        }
        .button.success:hover {
            background-color: #229954;
        }
        .data-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
            gap: 20px;
            margin-top: 20px;
        }
        .data-card {
            background-color: #2c3e50;
            padding: 20px;
            border-radius: 10px;
            text-align: center;
        }
        .data-value {
            font-size: 2em;
            font-weight: bold;
            color: #3498db;
        }
        .logs {
            background-color: #2c3e50;
            padding: 15px;
            border-radius: 5px;
            height: 200px;
            overflow-y: auto;
            font-family: monospace;
            font-size: 0.9em;
        }
        .title {
            color: #3498db;
            margin-bottom: 15px;
        }
        @keyframes pulse {
            0% { opacity: 1; }
            50% { opacity: 0.5; }
            100% { opacity: 1; }
        }
        .updating {
            animation: pulse 2s infinite;
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>🚜 TRACTOBOTS LIVE DASHBOARD</h1>
        <p>Real-time Agricultural Robotics Control Center</p>
    </div>
    
    <div class="container">
        <div class="left-panel">
            <h2 class="title">System Status</h2>
            <div id="system-status">
                <!-- Status items will be populated by JavaScript -->
            </div>
            
            <h2 class="title">Control Panel</h2>
            <button class="button success" onclick="controlAction('start_ros2')">Start ROS2</button>
            <button class="button success" onclick="controlAction('start_gazebo')">Start Gazebo</button>
            <button class="button danger" onclick="controlAction('stop_all')">Stop All</button>
            
            <h2 class="title">System Stats</h2>
            <div id="system-stats">
                <!-- Stats will be populated by JavaScript -->
            </div>
        </div>
        
        <div class="right-panel">
            <h2 class="title">Live Tractor Data</h2>
            <div class="data-grid">
                <div class="data-card">
                    <h3>Position</h3>
                    <div class="data-value" id="position">0, 0</div>
                    <small>X, Y (meters)</small>
                </div>
                <div class="data-card">
                    <h3>Speed</h3>
                    <div class="data-value" id="speed">0.0</div>
                    <small>m/s</small>
                </div>
                <div class="data-card">
                    <h3>Heading</h3>
                    <div class="data-value" id="heading">0°</div>
                    <small>degrees</small>
                </div>
                <div class="data-card">
                    <h3>Battery</h3>
                    <div class="data-value" id="battery">85%</div>
                    <small>remaining</small>
                </div>
            </div>
            
            <h2 class="title">System Logs</h2>
            <div id="logs" class="logs">
                <!-- Logs will be populated by JavaScript -->
            </div>
        </div>
    </div>
    
    <script>
        function updateDashboard() {
            fetch('/api/status')
                .then(response => response.json())
                .then(data => {
                    // Update system status
                    const statusDiv = document.getElementById('system-status');
                    statusDiv.innerHTML = '';
                    
                    for (const [system, status] of Object.entries(data.status)) {
                        const statusItem = document.createElement('div');
                        statusItem.className = 'status-item';
                        statusItem.innerHTML = `
                            <span>${system.toUpperCase()}</span>
                            <span class="${status ? 'status-online' : 'status-offline'}">
                                ${status ? '🟢 ONLINE' : '🔴 OFFLINE'}
                            </span>
                        `;
                        statusDiv.appendChild(statusItem);
                    }
                    
                    // Update system stats
                    const statsDiv = document.getElementById('system-stats');
                    statsDiv.innerHTML = '';
                    
                    for (const [stat, value] of Object.entries(data.stats)) {
                        const statItem = document.createElement('div');
                        statItem.className = 'status-item';
                        statItem.innerHTML = `
                            <span>${stat.replace('_', ' ').toUpperCase()}</span>
                            <span>${typeof value === 'number' ? value.toFixed(1) : value}%</span>
                        `;
                        statsDiv.appendChild(statItem);
                    }
                    
                    // Update live data
                    const liveData = data.live_data;
                    document.getElementById('position').textContent = 
                        `${liveData.position.x.toFixed(1)}, ${liveData.position.y.toFixed(1)}`;
                    document.getElementById('speed').textContent = liveData.speed.toFixed(1);
                    document.getElementById('heading').textContent = `${liveData.heading.toFixed(0)}°`;
                    document.getElementById('battery').textContent = `${liveData.battery.toFixed(0)}%`;
                    
                    // Update logs
                    const logsDiv = document.getElementById('logs');
                    logsDiv.innerHTML = data.logs.join('<br>');
                    logsDiv.scrollTop = logsDiv.scrollHeight;
                    
                    // Add updating animation
                    document.body.classList.add('updating');
                    setTimeout(() => document.body.classList.remove('updating'), 500);
                })
                .catch(error => {
                    console.error('Error updating dashboard:', error);
                });
        }
        
        function controlAction(action) {
            fetch(`/api/control/${action}`)
                .then(response => response.json())
                .then(data => {
                    alert(data.message);
                })
                .catch(error => {
                    console.error('Error:', error);
                    alert('Error executing action');
                });
        }
        
        // Update dashboard every 2 seconds
        setInterval(updateDashboard, 2000);
        
        // Initial load
        updateDashboard();
    </script>
</body>
</html>
'''

# Write HTML template
with open('templates/dashboard.html', 'w') as f:
    f.write(html_template)

if __name__ == '__main__':
    log_message("🚀 Starting Tractobots Web Dashboard...")
    log_message("📊 Dashboard will be available at: http://localhost:5000")
    log_message("🔄 Real-time updates every 2 seconds")
    log_message("🎮 Control buttons active")
    
    try:
        app.run(host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        log_message("🛑 Dashboard stopped by user")
    except Exception as e:
        log_message(f"❌ Dashboard error: {e}")
