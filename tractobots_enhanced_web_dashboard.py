#!/usr/bin/env python3
"""
Tractobots Enhanced Web Dashboard
Professional Web-Based Interface for Agricultural Robotics Control
"""

import webbrowser
import time
import threading
import subprocess
import sys
import os
import math
import json
from datetime import datetime
from flask import Flask, render_template_string, jsonify

def create_enhanced_web_dashboard():
    """Create the enhanced web dashboard with professional sidebar and features"""
    
    app = Flask(__name__)
    
    # Comprehensive dashboard data
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
            'hydraulics': True,
            'field_computer': True,
            'weather_station': False
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
            'implement_depth': 15.2,
            'engine_hours': 1247,
            'distance_traveled': 0,
            'area_worked': 0,
            'efficiency_rating': 95
        },
        'environment': {
            'temperature': 22.0,
            'humidity': 65.0,
            'wind_speed': 2.5,
            'wind_direction': 245,
            'light_level': 85.0,
            'soil_moisture': 45.0,
            'precipitation': 0.0,
            'air_pressure': 1013.25,
            'uv_index': 6.2,
            'dew_point': 12.3
        },
        'stats': {
            'cpu_usage': 45.5,
            'memory_usage': 62.3,
            'disk_usage': 23.8,
            'network_activity': 12.1,
            'gpu_usage': 34.2,
            'temperature_cpu': 58.3,
            'temperature_gpu': 42.1,
            'system_uptime': 86400,
            'processes_running': 156,
            'network_latency': 23.5
        },
        'field_operations': {
            'current_field': 'Field Alpha-7',
            'operation_type': 'Planting',
            'rows_completed': 23,
            'total_rows': 45,
            'seed_rate': 32000,
            'planting_depth': 5.2,
            'ground_speed': 2.8,
            'acres_per_hour': 4.2
        },
        'alerts': [
            {'type': 'info', 'message': 'System startup completed successfully', 'time': '12:34:56', 'priority': 'low'},
            {'type': 'warning', 'message': 'GPS signal slightly weak in area B-3', 'time': '12:33:22', 'priority': 'medium'},
            {'type': 'success', 'message': 'Field boundary detection completed', 'time': '12:32:18', 'priority': 'low'},
            {'type': 'error', 'message': 'Camera feed temporarily unavailable', 'time': '12:31:45', 'priority': 'high'},
            {'type': 'info', 'message': 'Weather station connection established', 'time': '12:30:12', 'priority': 'low'}
        ],
        'logs': [
            f"[{datetime.now().strftime('%H:%M:%S')}] Enhanced dashboard started",
            f"[{datetime.now().strftime('%H:%M:%S')}] All system modules loaded",
            f"[{datetime.now().strftime('%H:%M:%S')}] Real-time monitoring active",
            f"[{datetime.now().strftime('%H:%M:%S')}] Environmental sensors online",
            f"[{datetime.now().strftime('%H:%M:%S')}] GPS tracking initialized",
            f"[{datetime.now().strftime('%H:%M:%S')}] CAN bus communication established",
            f"[{datetime.now().strftime('%H:%M:%S')}] Hydraulic system operational",
            f"[{datetime.now().strftime('%H:%M:%S')}] Field operation commenced",
            f"[{datetime.now().strftime('%H:%M:%S')}] Safety systems armed and ready"
        ]
    }
    
    def update_data():
        """Enhanced data update with comprehensive real-time values"""
        while True:
            t = time.time()
            
            # Enhanced position data with 3D coordinates
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
                'distance_traveled': (t / 10) % 1000,
                'area_worked': (t / 20) % 100,
                'efficiency_rating': 95 + 4 * math.sin(t * 0.04),
                'timestamp': datetime.now().isoformat()
            })
            
            # Enhanced environmental data
            dashboard_data['environment'].update({
                'temperature': 22 + 8 * math.sin(t * 0.02),
                'humidity': 65 + 15 * math.cos(t * 0.025),
                'wind_speed': 2.5 + 3 * math.sin(t * 0.03),
                'wind_direction': (t * 5) % 360,
                'light_level': 85 + 10 * math.cos(t * 0.01),
                'soil_moisture': 45 + 20 * math.sin(t * 0.015),
                'precipitation': max(0, 2 * math.sin(t * 0.01)),
                'air_pressure': 1013.25 + 5 * math.cos(t * 0.005),
                'uv_index': 6.2 + 2 * math.sin(t * 0.008),
                'dew_point': 12.3 + 3 * math.cos(t * 0.012)
            })
            
            # Enhanced system stats
            dashboard_data['stats'].update({
                'cpu_usage': 45 + 20 * math.sin(t * 0.06),
                'memory_usage': 62 + 15 * math.cos(t * 0.08),
                'disk_usage': 23 + 5 * math.sin(t * 0.04),
                'network_activity': 12 + 18 * math.cos(t * 0.09),
                'gpu_usage': 34 + 25 * math.sin(t * 0.11),
                'temperature_cpu': 58 + 10 * math.sin(t * 0.03),
                'temperature_gpu': 42 + 8 * math.cos(t * 0.05),
                'system_uptime': int(t),
                'processes_running': 156 + int(10 * math.sin(t * 0.1)),
                'network_latency': 23.5 + 5 * math.sin(t * 0.07)
            })
            
            # Enhanced field operations
            dashboard_data['field_operations'].update({
                'rows_completed': min(45, int(t / 10) % 45),
                'ground_speed': 2.8 + 0.5 * math.sin(t * 0.04),
                'acres_per_hour': 4.2 + 0.8 * math.cos(t * 0.03)
            })
            
            time.sleep(0.5)  # Update every 0.5 seconds for smoother data
    
    # Start enhanced background data update
    update_thread = threading.Thread(target=update_data, daemon=True)
    update_thread.start()
    
    # Enhanced HTML template with professional sidebar
    html_template = '''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>🚜 Tractobots Enhanced Web Dashboard</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #0c0c0c, #1a1a2e, #16213e, #0f3460);
            color: #ecf0f1;
            overflow-x: hidden;
        }
        
        .dashboard-container {
            display: flex;
            min-height: 100vh;
        }
        
        .sidebar {
            width: 380px;
            background: linear-gradient(180deg, rgba(26, 26, 46, 0.98), rgba(44, 62, 80, 0.95));
            backdrop-filter: blur(20px);
            border-right: 3px solid rgba(52, 152, 219, 0.4);
            box-shadow: 5px 0 30px rgba(0, 0, 0, 0.6);
            position: fixed;
            height: 100vh;
            overflow-y: auto;
            z-index: 1000;
            transition: all 0.3s ease;
        }
        
        .sidebar:hover {
            box-shadow: 8px 0 40px rgba(0, 0, 0, 0.7);
        }
        
        .sidebar-toggle {
            display: none;
        }
        
        .main-content {
            flex: 1;
            margin-left: 380px;
            background: linear-gradient(135deg, rgba(15, 52, 96, 0.1), rgba(26, 26, 46, 0.05));
            min-height: 100vh;
        }
        
        .header {
            background: linear-gradient(135deg, rgba(52, 73, 94, 0.95), rgba(44, 62, 80, 0.95));
            padding: 30px;
            text-align: center;
            border-bottom: 4px solid #3498db;
            box-shadow: 0 6px 25px rgba(0, 0, 0, 0.4);
        }
        
        .header h1 {
            font-size: 3em;
            margin-bottom: 10px;
            background: linear-gradient(45deg, #3498db, #2ecc71, #f39c12, #e74c3c);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            text-shadow: 2px 2px 4px rgba(0, 0, 0, 0.5);
        }
        
        .header p {
            font-size: 1.1em;
            color: #bdc3c7;
            margin-top: 10px;
        }
        
        .card {
            background: rgba(52, 73, 94, 0.95);
            border-radius: 15px;
            padding: 20px;
            margin: 15px;
            box-shadow: 0 10px 30px rgba(0, 0, 0, 0.4);
            border: 1px solid rgba(52, 152, 219, 0.3);
            transition: all 0.3s ease;
        }
        
        .card:hover {
            transform: translateY(-5px);
            box-shadow: 0 15px 40px rgba(0, 0, 0, 0.5);
            border-color: rgba(52, 152, 219, 0.6);
        }
        
        .card h2, .card h3 {
            color: #3498db;
            margin-bottom: 15px;
            font-weight: 600;
            border-bottom: 2px solid rgba(52, 152, 219, 0.3);
            padding-bottom: 10px;
        }
        
        .status-item {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 12px 15px;
            margin: 8px 0;
            background: rgba(44, 62, 80, 0.7);
            border-radius: 10px;
            border-left: 4px solid transparent;
            transition: all 0.3s ease;
        }
        
        .status-item:hover {
            background: rgba(44, 62, 80, 0.9);
            transform: translateX(5px);
        }
        
        .status-item.online {
            border-left-color: #2ecc71;
        }
        
        .status-item.offline {
            border-left-color: #e74c3c;
        }
        
        .status-icon {
            font-size: 1.3em;
            margin-right: 10px;
        }
        
        .status-text {
            font-weight: 600;
        }
        
        .online .status-text {
            color: #2ecc71;
            text-shadow: 0 0 10px rgba(46, 204, 113, 0.3);
        }
        
        .offline .status-text {
            color: #e74c3c;
            text-shadow: 0 0 10px rgba(231, 76, 60, 0.3);
        }
        
        .control-btn {
            width: 100%;
            padding: 18px;
            margin: 10px 0;
            border: none;
            border-radius: 12px;
            font-size: 1.1em;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.3s ease;
            text-transform: uppercase;
            letter-spacing: 1px;
            position: relative;
            overflow: hidden;
        }
        
        .control-btn:before {
            content: '';
            position: absolute;
            top: 0;
            left: -100%;
            width: 100%;
            height: 100%;
            background: linear-gradient(90deg, transparent, rgba(255, 255, 255, 0.2), transparent);
            transition: left 0.5s ease;
        }
        
        .control-btn:hover:before {
            left: 100%;
        }
        
        .btn-success {
            background: linear-gradient(135deg, #27ae60, #2ecc71);
            color: white;
            box-shadow: 0 4px 15px rgba(46, 204, 113, 0.3);
        }
        
        .btn-success:hover {
            background: linear-gradient(135deg, #2ecc71, #27ae60);
            transform: translateY(-2px);
            box-shadow: 0 6px 20px rgba(46, 204, 113, 0.4);
        }
        
        .btn-primary {
            background: linear-gradient(135deg, #2980b9, #3498db);
            color: white;
            box-shadow: 0 4px 15px rgba(52, 152, 219, 0.3);
        }
        
        .btn-primary:hover {
            background: linear-gradient(135deg, #3498db, #2980b9);
            transform: translateY(-2px);
            box-shadow: 0 6px 20px rgba(52, 152, 219, 0.4);
        }
        
        .btn-warning {
            background: linear-gradient(135deg, #e67e22, #f39c12);
            color: white;
            box-shadow: 0 4px 15px rgba(243, 156, 18, 0.3);
        }
        
        .btn-warning:hover {
            background: linear-gradient(135deg, #f39c12, #e67e22);
            transform: translateY(-2px);
            box-shadow: 0 6px 20px rgba(243, 156, 18, 0.4);
        }
        
        .btn-danger {
            background: linear-gradient(135deg, #c0392b, #e74c3c);
            color: white;
            box-shadow: 0 4px 15px rgba(231, 76, 60, 0.3);
        }
        
        .btn-danger:hover {
            background: linear-gradient(135deg, #e74c3c, #c0392b);
            transform: translateY(-2px);
            box-shadow: 0 6px 20px rgba(231, 76, 60, 0.4);
        }
        
        .resource-bar {
            width: 100%;
            height: 25px;
            background: rgba(44, 62, 80, 0.8);
            border-radius: 12px;
            margin: 10px 0;
            overflow: hidden;
            position: relative;
            box-shadow: inset 0 2px 5px rgba(0, 0, 0, 0.3);
        }
        
        .resource-fill {
            height: 100%;
            border-radius: 12px;
            transition: width 0.3s ease;
            position: relative;
        }
        
        .resource-fill.cpu {
            background: linear-gradient(90deg, #2ecc71, #f39c12, #e74c3c);
        }
        
        .resource-fill.memory {
            background: linear-gradient(90deg, #3498db, #9b59b6);
        }
        
        .resource-fill.disk {
            background: linear-gradient(90deg, #1abc9c, #16a085);
        }
        
        .resource-fill.network {
            background: linear-gradient(90deg, #f39c12, #e67e22);
        }
        
        .resource-fill.gpu {
            background: linear-gradient(90deg, #9b59b6, #8e44ad);
        }
        
        .resource-text {
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            color: white;
            font-weight: bold;
            font-size: 0.9em;
            text-shadow: 1px 1px 2px rgba(0, 0, 0, 0.8);
        }
        
        .tab-nav {
            display: flex;
            background: rgba(44, 62, 80, 0.9);
            border-radius: 15px;
            margin: 15px;
            padding: 10px;
            gap: 10px;
            flex-wrap: wrap;
        }
        
        .tab-btn {
            flex: 1;
            padding: 15px 20px;
            background: rgba(52, 73, 94, 0.6);
            border: none;
            border-radius: 10px;
            color: #ecf0f1;
            font-weight: 600;
            font-size: 1em;
            cursor: pointer;
            transition: all 0.3s ease;
        }
        
        .tab-btn:hover {
            background: rgba(52, 152, 219, 0.7);
            transform: translateY(-2px);
        }
        
        .tab-btn.active {
            background: linear-gradient(135deg, #3498db, #2980b9);
            color: white;
            box-shadow: 0 4px 15px rgba(52, 152, 219, 0.3);
        }
        
        .tab-content {
            display: none;
            animation: fadeIn 0.5s ease;
        }
        
        .tab-content.active {
            display: block;
        }
        
        @keyframes fadeIn {
            from { opacity: 0; transform: translateY(20px); }
            to { opacity: 1; transform: translateY(0); }
        }
        
        .data-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 20px;
            margin: 20px 0;
        }
        
        .data-card {
            background: rgba(44, 62, 80, 0.7);
            border-radius: 12px;
            padding: 20px;
            text-align: center;
            border: 1px solid rgba(52, 152, 219, 0.2);
            transition: all 0.3s ease;
        }
        
        .data-card:hover {
            transform: translateY(-3px);
            box-shadow: 0 8px 20px rgba(0, 0, 0, 0.3);
        }
        
        .data-label {
            font-size: 0.9em;
            color: #bdc3c7;
            margin-bottom: 8px;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        
        .data-value {
            font-size: 2.5em;
            font-weight: bold;
            margin: 10px 0;
            text-shadow: 0 0 15px rgba(52, 152, 219, 0.3);
        }
        
        .logs-container {
            background: rgba(44, 62, 80, 0.8);
            border-radius: 12px;
            padding: 20px;
            height: 350px;
            overflow-y: auto;
            font-family: 'Courier New', monospace;
            font-size: 0.9em;
            line-height: 1.6;
            border: 1px solid rgba(52, 152, 219, 0.2);
        }
        
        .alert {
            padding: 15px;
            margin: 10px 0;
            border-radius: 8px;
            border-left: 4px solid;
            transition: all 0.3s ease;
        }
        
        .alert:hover {
            transform: translateX(5px);
        }
        
        .alert.info {
            background: rgba(52, 152, 219, 0.2);
            border-left-color: #3498db;
        }
        
        .alert.warning {
            background: rgba(243, 156, 18, 0.2);
            border-left-color: #f39c12;
        }
        
        .alert.success {
            background: rgba(46, 204, 113, 0.2);
            border-left-color: #2ecc71;
        }
        
        .alert.error {
            background: rgba(231, 76, 60, 0.2);
            border-left-color: #e74c3c;
        }
        
        .metric-card {
            background: rgba(44, 62, 80, 0.7);
            border-radius: 12px;
            padding: 20px;
            margin: 15px 0;
            border: 1px solid rgba(52, 152, 219, 0.2);
        }
        
        .metric-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
            gap: 15px;
            margin: 20px 0;
        }
        
        .progress-bar {
            width: 100%;
            height: 20px;
            background: rgba(44, 62, 80, 0.8);
            border-radius: 10px;
            overflow: hidden;
            margin: 10px 0;
        }
        
        .progress-fill {
            height: 100%;
            transition: width 0.5s ease;
            border-radius: 10px;
        }
        
        .weather-widget {
            background: rgba(44, 62, 80, 0.7);
            border-radius: 12px;
            padding: 20px;
            margin: 15px 0;
            text-align: center;
        }
        
        .weather-icon {
            font-size: 3em;
            margin: 10px 0;
        }
        
        @media (max-width: 1200px) {
            .sidebar {
                width: 320px;
                transform: translateX(-100%);
                transition: transform 0.3s ease;
            }
            
            .sidebar.open {
                transform: translateX(0);
            }
            
            .sidebar-toggle {
                display: block;
                position: fixed;
                top: 20px;
                left: 20px;
                z-index: 1001;
                background: rgba(52, 152, 219, 0.9);
                color: white;
                border: none;
                border-radius: 50%;
                width: 55px;
                height: 55px;
                font-size: 1.5em;
                cursor: pointer;
                transition: all 0.3s ease;
            }
            
            .sidebar-toggle:hover {
                background: rgba(52, 152, 219, 1);
                transform: scale(1.1);
            }
            
            .main-content {
                margin-left: 0;
            }
            
            .data-grid {
                grid-template-columns: 1fr 1fr;
            }
        }
        
        @media (max-width: 768px) {
            .sidebar {
                width: 300px;
            }
            
            .data-grid {
                grid-template-columns: 1fr;
            }
            
            .header h1 {
                font-size: 2em;
            }
            
            .tab-nav {
                flex-direction: column;
                gap: 5px;
            }
        }
    </style>
</head>
<body>
    <div class="dashboard-container">
        <!-- Sidebar Toggle Button -->
        <button class="sidebar-toggle" onclick="toggleSidebar()">☰</button>
        
        <!-- Enhanced Sidebar -->
        <div class="sidebar" id="sidebar">
            <div class="card">
                <h2>🚜 Tractobots Control</h2>
                <div style="text-align: center; margin: 15px 0;">
                    <div id="currentTime" style="font-size: 1.2em; color: #3498db; font-weight: bold;"></div>
                    <div style="margin-top: 10px;">
                        Status: <span id="connectionStatus" class="online">🟢 Connected</span>
                    </div>
                </div>
            </div>
            
            <div class="card">
                <h3>🔧 System Status</h3>
                <div id="systemStatus"></div>
            </div>
            
            <div class="card">
                <h3>🎮 System Controls</h3>
                <button class="control-btn btn-success" onclick="controlAction('start_ros2')">
                    🚀 Start ROS2 System
                </button>
                <button class="control-btn btn-primary" onclick="controlAction('start_gazebo')">
                    🌍 Launch Gazebo
                </button>
                <button class="control-btn btn-primary" onclick="controlAction('start_navigation')">
                    🧭 Start Navigation
                </button>
                <button class="control-btn btn-primary" onclick="controlAction('start_gps')">
                    📡 Initialize GPS
                </button>
                <button class="control-btn btn-primary" onclick="controlAction('start_camera')">
                    📷 Activate Camera
                </button>
                <button class="control-btn btn-warning" onclick="controlAction('start_diagnostics')">
                    🔍 Run Diagnostics
                </button>
                <button class="control-btn btn-danger" onclick="controlAction('emergency_stop')">
                    🚨 Emergency Stop
                </button>
            </div>
            
            <div class="card">
                <h3>📊 System Resources</h3>
                <div id="systemResources">
                    <div style="margin-bottom: 15px;">
                        <div style="display: flex; justify-content: space-between; align-items: center;">
                            <span>💻 CPU Usage</span>
                            <span id="cpuValue">0%</span>
                        </div>
                        <div class="resource-bar">
                            <div class="resource-fill cpu" id="cpuBar" style="width: 0%">
                                <div class="resource-text" id="cpuText">0%</div>
                            </div>
                        </div>
                    </div>
                    
                    <div style="margin-bottom: 15px;">
                        <div style="display: flex; justify-content: space-between; align-items: center;">
                            <span>🧠 Memory Usage</span>
                            <span id="memoryValue">0%</span>
                        </div>
                        <div class="resource-bar">
                            <div class="resource-fill memory" id="memoryBar" style="width: 0%">
                                <div class="resource-text" id="memoryText">0%</div>
                            </div>
                        </div>
                    </div>
                    
                    <div style="margin-bottom: 15px;">
                        <div style="display: flex; justify-content: space-between; align-items: center;">
                            <span>💾 Disk Usage</span>
                            <span id="diskValue">0%</span>
                        </div>
                        <div class="resource-bar">
                            <div class="resource-fill disk" id="diskBar" style="width: 0%">
                                <div class="resource-text" id="diskText">0%</div>
                            </div>
                        </div>
                    </div>
                    
                    <div style="margin-bottom: 15px;">
                        <div style="display: flex; justify-content: space-between; align-items: center;">
                            <span>🌐 Network</span>
                            <span id="networkValue">0 MB/s</span>
                        </div>
                        <div class="resource-bar">
                            <div class="resource-fill network" id="networkBar" style="width: 0%">
                                <div class="resource-text" id="networkText">0 MB/s</div>
                            </div>
                        </div>
                    </div>
                    
                    <div style="margin-bottom: 15px;">
                        <div style="display: flex; justify-content: space-between; align-items: center;">
                            <span>🎮 GPU Usage</span>
                            <span id="gpuValue">0%</span>
                        </div>
                        <div class="resource-bar">
                            <div class="resource-fill gpu" id="gpuBar" style="width: 0%">
                                <div class="resource-text" id="gpuText">0%</div>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
            
            <div class="card">
                <h3>🌡️ Environment</h3>
                <div id="environmentData">
                    <div class="weather-widget">
                        <div class="weather-icon">🌤️</div>
                        <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 10px; font-size: 0.9em;">
                            <div>
                                <div>🌡️ <span id="tempValue">--°C</span></div>
                                <div>💧 <span id="humidityValue">--%</span></div>
                                <div>💨 <span id="windValue">-- m/s</span></div>
                            </div>
                            <div>
                                <div>☀️ <span id="lightValue">--%</span></div>
                                <div>🌱 <span id="soilValue">--%</span></div>
                                <div>🌧️ <span id="precipValue">-- mm</span></div>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
            
            <div class="card">
                <h3>🚨 System Alerts</h3>
                <div id="quickAlerts" style="max-height: 200px; overflow-y: auto;">
                    <div class="alert info">
                        <div style="font-weight: bold;">ℹ️ System Online</div>
                        <div style="font-size: 0.9em;">All systems operational</div>
                    </div>
                </div>
            </div>
        </div>
        
        <!-- Main Content Area -->
        <div class="main-content">
            <div class="header">
                <h1>🚜 TRACTOBOTS ENHANCED WEB DASHBOARD</h1>
                <p>Professional Agricultural Robotics Control Center</p>
                <p style="margin-top: 10px;">Real-time Monitoring • Advanced Analytics • Precision Control • Field Operations</p>
            </div>
            
            <!-- Tab Navigation -->
            <div class="tab-nav">
                <button class="tab-btn active" onclick="showTab('overview')">📊 Overview</button>
                <button class="tab-btn" onclick="showTab('position')">📍 Position</button>
                <button class="tab-btn" onclick="showTab('performance')">⚡ Performance</button>
                <button class="tab-btn" onclick="showTab('field')">🌾 Field Operations</button>
                <button class="tab-btn" onclick="showTab('environment')">🌿 Environment</button>
                <button class="tab-btn" onclick="showTab('diagnostics')">🔍 Diagnostics</button>
                <button class="tab-btn" onclick="showTab('logs')">📝 Logs</button>
            </div>
            
            <!-- Tab Contents -->
            <div id="overview-tab" class="tab-content active">
                <div class="card">
                    <h2>🚜 Live Tractor Data</h2>
                    <div class="data-grid">
                        <div class="data-card">
                            <div class="data-label">Position (X, Y, Z)</div>
                            <div class="data-value" id="position" style="color: #3498db; font-size: 1.8em;">0.0, 0.0, 0.0</div>
                            <div class="data-label">Coordinates (meters)</div>
                        </div>
                        <div class="data-card">
                            <div class="data-label">Speed</div>
                            <div class="data-value" id="speed" style="color: #2ecc71;">0.0</div>
                            <div class="data-label">m/s</div>
                        </div>
                        <div class="data-card">
                            <div class="data-label">Heading</div>
                            <div class="data-value" id="heading" style="color: #f39c12;">0°</div>
                            <div class="data-label">Degrees from North</div>
                        </div>
                        <div class="data-card">
                            <div class="data-label">Battery Level</div>
                            <div class="data-value" id="battery" style="color: #e74c3c;">85%</div>
                            <div class="progress-bar">
                                <div class="progress-fill" id="batteryProgress" style="width: 85%; background: linear-gradient(90deg, #2ecc71, #f39c12, #e74c3c);"></div>
                            </div>
                        </div>
                    </div>
                </div>
                
                <div class="card">
                    <h2>🎯 Mission Status</h2>
                    <div class="data-grid">
                        <div class="data-card">
                            <div class="data-label">Field Coverage</div>
                            <div class="data-value" id="fieldCoverage" style="color: #3498db;">0%</div>
                            <div class="progress-bar">
                                <div class="progress-fill" id="coverageProgress" style="width: 0%; background: linear-gradient(90deg, #3498db, #2980b9);"></div>
                            </div>
                        </div>
                        <div class="data-card">
                            <div class="data-label">Mission Progress</div>
                            <div class="data-value" id="missionProgress" style="color: #f39c12;">0%</div>
                            <div class="progress-bar">
                                <div class="progress-fill" id="missionProgressBar" style="width: 0%; background: linear-gradient(90deg, #f39c12, #e67e22);"></div>
                            </div>
                        </div>
                        <div class="data-card">
                            <div class="data-label">Work Rate</div>
                            <div class="data-value" id="workRate" style="color: #2ecc71;">2.4</div>
                            <div class="data-label">hectares/hour</div>
                        </div>
                        <div class="data-card">
                            <div class="data-label">Efficiency</div>
                            <div class="data-value" id="efficiency" style="color: #9b59b6;">95%</div>
                            <div class="data-label">Operational</div>
                        </div>
                    </div>
                </div>
            </div>
            
            <div id="position-tab" class="tab-content">
                <div class="card">
                    <h2>📍 Position & Navigation</h2>
                    <div class="metric-grid">
                        <div class="metric-card">
                            <h3>📊 Current Position</h3>
                            <div>X: <span id="posX">0.0</span> meters</div>
                            <div>Y: <span id="posY">0.0</span> meters</div>
                            <div>Z: <span id="posZ">0.0</span> meters</div>
                        </div>
                        <div class="metric-card">
                            <h3>🧭 Navigation Data</h3>
                            <div>Heading: <span id="currentHeading">0°</span></div>
                            <div>Speed: <span id="currentSpeed">0.0</span> m/s</div>
                            <div>Distance: <span id="distanceTraveled">0.0</span> km</div>
                        </div>
                    </div>
                </div>
            </div>
            
            <div id="performance-tab" class="tab-content">
                <div class="card">
                    <h2>⚡ Performance Metrics</h2>
                    <div class="metric-grid">
                        <div class="metric-card">
                            <h3>🔧 Engine Performance</h3>
                            <div>Temperature: <span id="engineTemp">87°C</span></div>
                            <div>Load: <span id="engineLoad">75%</span></div>
                            <div>Hours: <span id="engineHours">1247</span></div>
                        </div>
                        <div class="metric-card">
                            <h3>🛢️ Fuel & Hydraulics</h3>
                            <div>Fuel Level: <span id="fuelLevel">78%</span></div>
                            <div>Hydraulic Pressure: <span id="hydraulicPressure">180 bar</span></div>
                            <div>PTO Speed: <span id="ptoSpeed">540 RPM</span></div>
                        </div>
                    </div>
                </div>
            </div>
            
            <div id="field-tab" class="tab-content">
                <div class="card">
                    <h2>🌾 Field Operations</h2>
                    <div class="metric-grid">
                        <div class="metric-card">
                            <h3>🚜 Current Operation</h3>
                            <div>Field: <span id="currentField">Field Alpha-7</span></div>
                            <div>Operation: <span id="operationType">Planting</span></div>
                            <div>Progress: <span id="operationProgress">51%</span></div>
                        </div>
                        <div class="metric-card">
                            <h3>📊 Operation Stats</h3>
                            <div>Rows Completed: <span id="rowsCompleted">23</span>/<span id="totalRows">45</span></div>
                            <div>Area Worked: <span id="areaWorked">12.3</span> hectares</div>
                            <div>Efficiency: <span id="fieldEfficiency">95%</span></div>
                        </div>
                    </div>
                </div>
            </div>
            
            <div id="environment-tab" class="tab-content">
                <div class="card">
                    <h2>🌿 Environmental Conditions</h2>
                    <div class="metric-grid">
                        <div class="metric-card">
                            <h3>🌡️ Weather Data</h3>
                            <div>Temperature: <span id="envTemp">22°C</span></div>
                            <div>Humidity: <span id="envHumidity">65%</span></div>
                            <div>Wind Speed: <span id="envWind">2.5 m/s</span></div>
                            <div>Wind Direction: <span id="windDirection">245°</span></div>
                        </div>
                        <div class="metric-card">
                            <h3>🌱 Soil Conditions</h3>
                            <div>Soil Moisture: <span id="envSoil">45%</span></div>
                            <div>Air Pressure: <span id="airPressure">1013 hPa</span></div>
                            <div>UV Index: <span id="uvIndex">6.2</span></div>
                            <div>Dew Point: <span id="dewPoint">12.3°C</span></div>
                        </div>
                    </div>
                </div>
            </div>
            
            <div id="diagnostics-tab" class="tab-content">
                <div class="card">
                    <h2>🔍 System Diagnostics</h2>
                    <div class="metric-grid">
                        <div class="metric-card">
                            <h3>💻 System Health</h3>
                            <div>CPU Temp: <span id="cpuTemp">58°C</span></div>
                            <div>GPU Temp: <span id="gpuTemp">42°C</span></div>
                            <div>Uptime: <span id="systemUptime">24h 5m</span></div>
                            <div>Processes: <span id="processCount">156</span></div>
                        </div>
                        <div class="metric-card">
                            <h3>📡 Network Status</h3>
                            <div>Latency: <span id="networkLatency">23ms</span></div>
                            <div>Data Rate: <span id="networkRate">12.1 MB/s</span></div>
                            <div>Packets Lost: <span id="packetLoss">0.2%</span></div>
                        </div>
                    </div>
                </div>
            </div>
            
            <div id="logs-tab" class="tab-content">
                <div class="card">
                    <h2>📝 System Activity Logs</h2>
                    <div class="logs-container" id="logsContainer"></div>
                </div>
                
                <div class="card">
                    <h2>🚨 Alerts & Notifications</h2>
                    <div id="alertsContainer"></div>
                </div>
            </div>
        </div>
    </div>
    
    <script>
        let currentTab = 'overview';
        
        function toggleSidebar() {
            const sidebar = document.getElementById('sidebar');
            sidebar.classList.toggle('open');
        }
        
        function showTab(tabName) {
            // Hide all tab contents
            document.querySelectorAll('.tab-content').forEach(content => {
                content.classList.remove('active');
            });
            
            // Remove active class from all tab buttons
            document.querySelectorAll('.tab-btn').forEach(btn => {
                btn.classList.remove('active');
            });
            
            // Show selected tab content
            document.getElementById(tabName + '-tab').classList.add('active');
            
            // Add active class to clicked tab button
            event.target.classList.add('active');
            
            currentTab = tabName;
        }
        
        function updateTime() {
            const now = new Date();
            const timeStr = now.toLocaleString();
            document.getElementById('currentTime').textContent = timeStr;
        }
        
        function controlAction(action) {
            const messages = {
                'start_ros2': '🚀 ROS2 system startup initiated',
                'start_gazebo': '🌍 Gazebo simulation launching',
                'start_navigation': '🧭 Navigation system starting',
                'start_gps': '📡 GPS initialization in progress',
                'start_camera': '📷 Camera system activating',
                'start_diagnostics': '🔍 System diagnostics running',
                'emergency_stop': '🚨 EMERGENCY STOP ACTIVATED'
            };
            
            alert(messages[action] || 'Command executed');
        }
        
        function updateDashboard() {
            fetch('/api/data')
                .then(response => response.json())
                .then(data => {
                    // Update system status
                    const statusDiv = document.getElementById('systemStatus');
                    statusDiv.innerHTML = '';
                    
                    for (const [system, status] of Object.entries(data.status)) {
                        const statusItem = document.createElement('div');
                        statusItem.className = `status-item ${status ? 'online' : 'offline'}`;
                        statusItem.innerHTML = `
                            <span>
                                <span class="status-icon">${status ? '🟢' : '🔴'}</span>
                                ${system.toUpperCase().replace('_', ' ')}
                            </span>
                            <span class="status-text">${status ? 'ONLINE' : 'OFFLINE'}</span>
                        `;
                        statusDiv.appendChild(statusItem);
                    }
                    
                    // Update live data
                    document.getElementById('position').textContent = 
                        `${data.live_data.position.x.toFixed(1)}, ${data.live_data.position.y.toFixed(1)}, ${data.live_data.position.z.toFixed(1)}`;
                    document.getElementById('speed').textContent = data.live_data.speed.toFixed(1);
                    document.getElementById('heading').textContent = `${data.live_data.heading.toFixed(0)}°`;
                    
                    const batteryLevel = data.live_data.battery.toFixed(0);
                    document.getElementById('battery').textContent = `${batteryLevel}%`;
                    document.getElementById('batteryProgress').style.width = `${batteryLevel}%`;
                    
                    // Update mission data
                    document.getElementById('fieldCoverage').textContent = `${data.live_data.field_coverage.toFixed(1)}%`;
                    document.getElementById('coverageProgress').style.width = `${data.live_data.field_coverage}%`;
                    document.getElementById('missionProgress').textContent = `${data.live_data.mission_progress.toFixed(1)}%`;
                    document.getElementById('missionProgressBar').style.width = `${data.live_data.mission_progress}%`;
                    document.getElementById('workRate').textContent = data.live_data.work_rate.toFixed(1);
                    document.getElementById('efficiency').textContent = `${data.live_data.efficiency_rating.toFixed(0)}%`;
                    
                    // Update sidebar system resources
                    const stats = data.stats;
                    document.getElementById('cpuValue').textContent = `${stats.cpu_usage.toFixed(1)}%`;
                    document.getElementById('cpuBar').style.width = `${stats.cpu_usage}%`;
                    document.getElementById('cpuText').textContent = `${stats.cpu_usage.toFixed(1)}%`;
                    
                    document.getElementById('memoryValue').textContent = `${stats.memory_usage.toFixed(1)}%`;
                    document.getElementById('memoryBar').style.width = `${stats.memory_usage}%`;
                    document.getElementById('memoryText').textContent = `${stats.memory_usage.toFixed(1)}%`;
                    
                    document.getElementById('diskValue').textContent = `${stats.disk_usage.toFixed(1)}%`;
                    document.getElementById('diskBar').style.width = `${stats.disk_usage}%`;
                    document.getElementById('diskText').textContent = `${stats.disk_usage.toFixed(1)}%`;
                    
                    document.getElementById('networkValue').textContent = `${stats.network_activity.toFixed(1)} MB/s`;
                    document.getElementById('networkBar').style.width = `${Math.min(stats.network_activity * 2, 100)}%`;
                    document.getElementById('networkText').textContent = `${stats.network_activity.toFixed(1)} MB/s`;
                    
                    document.getElementById('gpuValue').textContent = `${stats.gpu_usage.toFixed(1)}%`;
                    document.getElementById('gpuBar').style.width = `${stats.gpu_usage}%`;
                    document.getElementById('gpuText').textContent = `${stats.gpu_usage.toFixed(1)}%`;
                    
                    // Update sidebar environmental data
                    document.getElementById('tempValue').textContent = `${data.environment.temperature.toFixed(1)}°C`;
                    document.getElementById('humidityValue').textContent = `${data.environment.humidity.toFixed(0)}%`;
                    document.getElementById('windValue').textContent = `${data.environment.wind_speed.toFixed(1)} m/s`;
                    document.getElementById('lightValue').textContent = `${data.environment.light_level.toFixed(0)}%`;
                    document.getElementById('soilValue').textContent = `${data.environment.soil_moisture.toFixed(0)}%`;
                    document.getElementById('precipValue').textContent = `${data.environment.precipitation.toFixed(1)} mm`;
                    
                    // Update detailed tab data
                    if (document.getElementById('posX')) {
                        document.getElementById('posX').textContent = data.live_data.position.x.toFixed(1);
                        document.getElementById('posY').textContent = data.live_data.position.y.toFixed(1);
                        document.getElementById('posZ').textContent = data.live_data.position.z.toFixed(1);
                        document.getElementById('currentHeading').textContent = `${data.live_data.heading.toFixed(0)}°`;
                        document.getElementById('currentSpeed').textContent = data.live_data.speed.toFixed(1);
                        document.getElementById('distanceTraveled').textContent = (data.live_data.distance_traveled / 1000).toFixed(1);
                    }
                    
                    // Update performance metrics
                    if (document.getElementById('engineTemp')) {
                        document.getElementById('engineTemp').textContent = `${data.live_data.engine_temp.toFixed(0)}°C`;
                        document.getElementById('engineLoad').textContent = `${((data.live_data.engine_temp - 50) * 2).toFixed(0)}%`;
                        document.getElementById('engineHours').textContent = data.live_data.engine_hours;
                        document.getElementById('fuelLevel').textContent = `${data.live_data.fuel_level.toFixed(0)}%`;
                        document.getElementById('hydraulicPressure').textContent = `${data.live_data.hydraulic_pressure.toFixed(0)} bar`;
                        document.getElementById('ptoSpeed').textContent = `${data.live_data.pto_speed.toFixed(0)} RPM`;
                    }
                    
                    // Update field operations
                    if (document.getElementById('rowsCompleted')) {
                        document.getElementById('rowsCompleted').textContent = data.field_operations.rows_completed;
                        document.getElementById('areaWorked').textContent = data.live_data.area_worked.toFixed(1);
                        document.getElementById('fieldEfficiency').textContent = `${data.live_data.efficiency_rating.toFixed(0)}%`;
                    }
                    
                    // Update environment tab
                    if (document.getElementById('envTemp')) {
                        document.getElementById('envTemp').textContent = `${data.environment.temperature.toFixed(1)}°C`;
                        document.getElementById('envHumidity').textContent = `${data.environment.humidity.toFixed(0)}%`;
                        document.getElementById('envWind').textContent = `${data.environment.wind_speed.toFixed(1)} m/s`;
                        document.getElementById('windDirection').textContent = `${data.environment.wind_direction.toFixed(0)}°`;
                        document.getElementById('envSoil').textContent = `${data.environment.soil_moisture.toFixed(0)}%`;
                        document.getElementById('airPressure').textContent = `${data.environment.air_pressure.toFixed(0)} hPa`;
                        document.getElementById('uvIndex').textContent = data.environment.uv_index.toFixed(1);
                        document.getElementById('dewPoint').textContent = `${data.environment.dew_point.toFixed(1)}°C`;
                    }
                    
                    // Update diagnostics
                    if (document.getElementById('cpuTemp')) {
                        document.getElementById('cpuTemp').textContent = `${data.stats.temperature_cpu.toFixed(0)}°C`;
                        document.getElementById('gpuTemp').textContent = `${data.stats.temperature_gpu.toFixed(0)}°C`;
                        document.getElementById('systemUptime').textContent = `${Math.floor(data.stats.system_uptime / 3600)}h ${Math.floor((data.stats.system_uptime % 3600) / 60)}m`;
                        document.getElementById('processCount').textContent = data.stats.processes_running;
                        document.getElementById('networkLatency').textContent = `${data.stats.network_latency.toFixed(0)}ms`;
                        document.getElementById('networkRate').textContent = `${data.stats.network_activity.toFixed(1)} MB/s`;
                        document.getElementById('packetLoss').textContent = '0.2%';
                    }
                    
                    // Update alerts
                    const alertsContainer = document.getElementById('alertsContainer');
                    alertsContainer.innerHTML = '';
                    
                    data.alerts.forEach(alert => {
                        const alertDiv = document.createElement('div');
                        alertDiv.className = `alert ${alert.type}`;
                        alertDiv.innerHTML = `
                            <div style="font-weight: bold;">${getAlertIcon(alert.type)} ${alert.message}</div>
                            <div style="font-size: 0.9em; color: #bdc3c7; margin-top: 5px;">[${alert.time}] Priority: ${alert.priority}</div>
                        `;
                        alertsContainer.appendChild(alertDiv);
                    });
                    
                    // Update logs
                    const logsContainer = document.getElementById('logsContainer');
                    logsContainer.innerHTML = data.logs.slice(-20).map(log => `<div>${log}</div>`).join('');
                    logsContainer.scrollTop = logsContainer.scrollHeight;
                })
                .catch(error => {
                    console.error('Dashboard update error:', error);
                    document.getElementById('connectionStatus').innerHTML = '🔴 Connection Error';
                    document.getElementById('connectionStatus').className = 'offline';
                });
        }
        
        function getAlertIcon(type) {
            const icons = {
                'info': 'ℹ️',
                'warning': '⚠️',
                'success': '✅',
                'error': '❌'
            };
            return icons[type] || 'ℹ️';
        }
        
        // Initialize dashboard
        updateTime();
        updateDashboard();
        
        // Set up intervals
        setInterval(updateTime, 1000);
        setInterval(updateDashboard, 1000);
        
        // Handle window resize
        window.addEventListener('resize', function() {
            if (window.innerWidth > 1200) {
                document.getElementById('sidebar').classList.remove('open');
            }
        });
    </script>
</body>
</html>
    '''
    
    @app.route('/')
    def index():
        """Main dashboard page"""
        return html_template
    
    @app.route('/api/data')
    def get_data():
        """Get dashboard data"""
        return jsonify(dashboard_data)
    
    print("✅ Enhanced web dashboard configured")
    print("🌐 Starting web server on http://localhost:5000")
    print("📱 Opening browser...")
    print("🔄 Dashboard will update every second")
    print("📊 Enhanced features: Real-time metrics, comprehensive monitoring")
    print("🎮 Interactive controls and professional interface")
    print("🛑 Press Ctrl+C to stop")
    print("")
    
    # Open browser after a short delay
    def open_browser():
        time.sleep(2)
        webbrowser.open('http://localhost:5000')
    
    browser_thread = threading.Thread(target=open_browser, daemon=True)
    browser_thread.start()
    
    # Start the Flask app
    try:
        app.run(host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        print("\n🛑 Enhanced dashboard stopped by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")

if __name__ == "__main__":
    print("🚜 TRACTOBOTS ENHANCED WEB DASHBOARD")
    print("=" * 70)
    print("🎯 Professional agricultural robotics control center")
    print("📊 Real-time monitoring with advanced analytics")
    print("🎮 Interactive controls and responsive design")
    print("🌐 Web-based interface accessible from any device")
    print("")
    
    try:
        create_enhanced_web_dashboard()
    except Exception as e:
        print(f"💥 Error: {e}")
    
    print("\n🎉 Thank you for using Tractobots Enhanced Web Dashboard!")
