#!/usr/bin/env python3
"""
Tractobots Enhanced Dashboard - Professional Interface
Real-time monitoring with improved sidebar layout and enhanced features
"""

import webbrowser
import time
import threading
import subprocess
import sys
import os
import math
from datetime import datetime
from flask import Flask, render_template_string, jsonify

def create_enhanced_dashboard():
    """Create the enhanced dashboard with proper sidebar layout"""
    
    app = Flask(__name__)
    
    # Enhanced dashboard data with more features
    dashboard_data = {
        'status': {
            'ros2': True,
            'gazebo': False,
            'navigation': True,
            'gps': True,
            'simulation': False,
            'camera': False,
            'lidar': True,
            'imu': True
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
            'fuel_consumption': 12.3,
            'efficiency': 95
        },
        'environment': {
            'temperature': 22.0,
            'humidity': 65.0,
            'wind_speed': 2.5,
            'light_level': 85.0,
            'soil_moisture': 45.0
        },
        'stats': {
            'cpu_usage': 45.5,
            'memory_usage': 62.3,
            'disk_usage': 23.8,
            'network_activity': 12.1,
            'gpu_usage': 34.2
        },
        'alerts': [
            {'type': 'info', 'message': 'System startup completed successfully', 'time': '12:34:56'},
            {'type': 'warning', 'message': 'GPS signal slightly weak in area B-3', 'time': '12:33:22'},
            {'type': 'success', 'message': 'Field boundary detection completed', 'time': '12:32:18'}
        ],
        'logs': [
            f"[{datetime.now().strftime('%H:%M:%S')}] Enhanced dashboard started",
            f"[{datetime.now().strftime('%H:%M:%S')}] All system modules loaded",
            f"[{datetime.now().strftime('%H:%M:%S')}] Real-time monitoring active",
            f"[{datetime.now().strftime('%H:%M:%S')}] Environmental sensors online",
            f"[{datetime.now().strftime('%H:%M:%S')}] GPS tracking initialized"
        ]
    }
    
    def update_data():
        """Enhanced data update with more realistic values"""
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
                'fuel_consumption': 12.3 + 2.1 * math.cos(t * 0.07),
                'efficiency': 95 + 4 * math.sin(t * 0.04),
                'timestamp': datetime.now().isoformat()
            })
            
            # Enhanced environmental data
            dashboard_data['environment'].update({
                'temperature': 22 + 8 * math.sin(t * 0.02),
                'humidity': 65 + 15 * math.cos(t * 0.025),
                'wind_speed': 2.5 + 3 * math.sin(t * 0.03),
                'light_level': 85 + 10 * math.cos(t * 0.01),
                'soil_moisture': 45 + 20 * math.sin(t * 0.015)
            })
            
            # Enhanced system stats
            dashboard_data['stats'].update({
                'cpu_usage': 45 + 20 * math.sin(t * 0.06),
                'memory_usage': 62 + 15 * math.cos(t * 0.08),
                'disk_usage': 23 + 5 * math.sin(t * 0.04),
                'network_activity': 12 + 18 * math.cos(t * 0.09),
                'gpu_usage': 34 + 25 * math.sin(t * 0.11)
            })
            
            time.sleep(1)  # Update every second for smoother data
    
    # Start enhanced background data update
    update_thread = threading.Thread(target=update_data, daemon=True)
    update_thread.start()
    
    # Enhanced HTML template with proper sidebar
    html_template = '''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>🚜 Tractobots Enhanced Dashboard</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #1a1a2e, #16213e, #0f3460);
            color: #ecf0f1;
            overflow-x: hidden;
        }
        
        .sidebar-toggle {
            display: none;
        }
        
        .dashboard-container {
            display: flex;
            min-height: 100vh;
        }
        
        .sidebar {
            width: 350px;
            background: linear-gradient(180deg, rgba(26, 26, 46, 0.98), rgba(44, 62, 80, 0.95));
            backdrop-filter: blur(15px);
            border-right: 3px solid rgba(52, 152, 219, 0.4);
            box-shadow: 5px 0 30px rgba(0, 0, 0, 0.5);
            position: fixed;
            height: 100vh;
            overflow-y: auto;
            z-index: 1000;
            transition: all 0.3s ease;
        }
        
        .sidebar:hover {
            box-shadow: 8px 0 40px rgba(0, 0, 0, 0.6);
        }
        
        .main-content {
            flex: 1;
            margin-left: 350px;
            background: linear-gradient(135deg, rgba(15, 52, 96, 0.1), rgba(26, 26, 46, 0.05));
            min-height: 100vh;
        }
        
        .header {
            background: linear-gradient(135deg, rgba(52, 73, 94, 0.9), rgba(44, 62, 80, 0.9));
            padding: 25px;
            text-align: center;
            border-bottom: 3px solid #3498db;
            box-shadow: 0 4px 20px rgba(0, 0, 0, 0.3);
        }
        
        .header h1 {
            font-size: 2.8em;
            margin-bottom: 10px;
            background: linear-gradient(45deg, #3498db, #2ecc71, #f39c12);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            text-shadow: 2px 2px 4px rgba(0, 0, 0, 0.5);
        }
        
        .card {
            background: rgba(52, 73, 94, 0.9);
            border-radius: 15px;
            padding: 20px;
            margin: 15px;
            box-shadow: 0 8px 25px rgba(0, 0, 0, 0.3);
            border: 1px solid rgba(52, 152, 219, 0.2);
            transition: all 0.3s ease;
        }
        
        .card:hover {
            transform: translateY(-5px);
            box-shadow: 0 12px 35px rgba(0, 0, 0, 0.4);
            border-color: rgba(52, 152, 219, 0.5);
        }
        
        .card h2, .card h3 {
            color: #3498db;
            margin-bottom: 15px;
            font-weight: 600;
            border-bottom: 2px solid rgba(52, 152, 219, 0.3);
            padding-bottom: 8px;
        }
        
        .status-item {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 12px 15px;
            margin: 8px 0;
            background: rgba(44, 62, 80, 0.6);
            border-radius: 10px;
            border-left: 4px solid transparent;
            transition: all 0.3s ease;
        }
        
        .status-item:hover {
            background: rgba(44, 62, 80, 0.8);
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
            height: 20px;
            background: rgba(44, 62, 80, 0.8);
            border-radius: 10px;
            margin: 8px 0;
            overflow: hidden;
            position: relative;
        }
        
        .resource-fill {
            height: 100%;
            border-radius: 10px;
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
            text-shadow: 1px 1px 2px rgba(0, 0, 0, 0.7);
        }
            border: none;
            border-radius: 10px;
            font-size: 1em;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.3s ease;
            position: relative;
            overflow: hidden;
        }
        
        .control-btn::before {
            content: '';
            position: absolute;
            top: 0;
            left: -100%;
            width: 100%;
            height: 100%;
            background: linear-gradient(90deg, transparent, rgba(255,255,255,0.2), transparent);
            transition: left 0.5s;
        }
        
        .control-btn:hover::before {
            left: 100%;
        }
        
        .btn-primary {
            background: linear-gradient(135deg, #3498db, #2980b9);
            color: white;
            box-shadow: 0 4px 15px rgba(52, 152, 219, 0.3);
        }
        
        .btn-primary:hover {
            background: linear-gradient(135deg, #2980b9, #3498db);
            transform: translateY(-2px);
            box-shadow: 0 6px 20px rgba(52, 152, 219, 0.4);
        }
        
        .btn-success {
            background: linear-gradient(135deg, #2ecc71, #27ae60);
            color: white;
            box-shadow: 0 4px 15px rgba(46, 204, 113, 0.3);
        }
        
        .btn-success:hover {
            background: linear-gradient(135deg, #27ae60, #2ecc71);
            transform: translateY(-2px);
            box-shadow: 0 6px 20px rgba(46, 204, 113, 0.4);
        }
        
        .btn-danger {
            background: linear-gradient(135deg, #e74c3c, #c0392b);
            color: white;
            box-shadow: 0 4px 15px rgba(231, 76, 60, 0.3);
        }
        
        .btn-danger:hover {
            background: linear-gradient(135deg, #c0392b, #e74c3c);
            transform: translateY(-2px);
            box-shadow: 0 6px 20px rgba(231, 76, 60, 0.4);
        }
        
        .data-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 20px;
            margin: 20px 0;
        }
        
        .data-card {
            background: rgba(44, 62, 80, 0.6);
            padding: 20px;
            border-radius: 12px;
            text-align: center;
            border: 1px solid rgba(52, 152, 219, 0.2);
            transition: all 0.3s ease;
        }
        
        .data-card:hover {
            transform: scale(1.05);
            border-color: rgba(52, 152, 219, 0.5);
        }
        
        .data-value {
            font-size: 2.5em;
            font-weight: bold;
            margin: 10px 0;
            text-shadow: 0 0 15px rgba(52, 152, 219, 0.5);
        }
        
        .data-label {
            font-size: 0.9em;
            color: #bdc3c7;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        
        .progress-bar {
            background: rgba(44, 62, 80, 0.8);
            border-radius: 10px;
            height: 12px;
            margin: 10px 0;
            overflow: hidden;
        }
        
        .progress-fill {
            height: 100%;
            border-radius: 8px;
            transition: width 0.5s ease;
            position: relative;
        }
        
        .progress-fill::after {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            background: linear-gradient(90deg, transparent, rgba(255,255,255,0.3), transparent);
            animation: shimmer 2s infinite;
        }
        
        @keyframes shimmer {
            0% { transform: translateX(-100%); }
            100% { transform: translateX(100%); }
        }
        
        .tab-nav {
            display: flex;
            background: rgba(52, 73, 94, 0.8);
            border-radius: 10px;
            margin: 20px;
            overflow: hidden;
        }
        
        .tab-btn {
            flex: 1;
            padding: 15px;
            background: transparent;
            border: none;
            color: #bdc3c7;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.3s ease;
        }
        
        .tab-btn:hover {
            background: rgba(52, 152, 219, 0.2);
            color: #3498db;
        }
        
        .tab-btn.active {
            background: linear-gradient(135deg, #3498db, #2980b9);
            color: white;
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
        
        .gauge {
            width: 150px;
            height: 150px;
            border-radius: 50%;
            background: conic-gradient(from 0deg, #e74c3c 0%, #f39c12 40%, #2ecc71 70%, #3498db 100%);
            display: flex;
            align-items: center;
            justify-content: center;
            margin: 20px auto;
            position: relative;
        }
        
        .gauge-inner {
            width: 80%;
            height: 80%;
            border-radius: 50%;
            background: #2c3e50;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
        }
        
        .logs-container {
            background: rgba(44, 62, 80, 0.8);
            border-radius: 10px;
            padding: 15px;
            height: 300px;
            overflow-y: auto;
            font-family: 'Courier New', monospace;
            font-size: 0.9em;
            border: 1px solid rgba(52, 152, 219, 0.3);
        }
        
        .log-entry {
            padding: 5px 0;
            border-bottom: 1px solid rgba(52, 152, 219, 0.1);
        }
        
        .alert {
            padding: 12px;
            margin: 8px 0;
            border-radius: 8px;
            border-left: 4px solid;
        }
        
        .alert.info {
            background: rgba(52, 152, 219, 0.1);
            border-left-color: #3498db;
        }
        
        .alert.warning {
            background: rgba(243, 156, 18, 0.1);
            border-left-color: #f39c12;
        }
        
        .alert.success {
            background: rgba(46, 204, 113, 0.1);
            border-left-color: #2ecc71;
        }
        
        .time-display {
            font-family: 'Courier New', monospace;
            font-size: 1.1em;
            color: #3498db;
        }
        
        @media (max-width: 1024px) {
            .sidebar {
                width: 280px;
            }
            .main-content {
                margin-left: 280px;
            }
        }
        
        @media (max-width: 1200px) {
            .sidebar {
                width: 300px;
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
                width: 50px;
                height: 50px;
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
                width: 280px;
                height: auto;
                position: relative;
                transform: none;
            }
            
            .sidebar-toggle {
                display: none;
            }
            
            .main-content {
                margin-left: 0;
            }
            
            .dashboard-container {
                flex-direction: column;
            }
            
            .data-grid {
                grid-template-columns: 1fr;
            }
            
            .header h1 {
                font-size: 2em;
            }
            
            .tab-nav {
                flex-wrap: wrap;
                gap: 5px;
            }
            
            .tab-btn {
                flex: 1;
                min-width: 120px;
                padding: 10px 5px;
                font-size: 0.9em;
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
                <div class="time-display" id="currentTime"></div>
                <div style="margin-top: 10px;">
                    Status: <span id="connectionStatus" class="online">🟢 Connected</span>
                </div>
            </div>
            
            <div class="card">
                <h3>🔧 System Status</h3>
                <div id="systemStatus"></div>
            </div>
            
            <div class="card">
                <h3>🎮 System Controls</h3>
                <button class="control-btn btn-success" onclick="controlAction('start_ros2')">
                    🤖 Start ROS2 System
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
                            <span>🌐 Network Activity</span>
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
                    <div style="margin-bottom: 10px;">
                        <div style="display: flex; justify-content: space-between; align-items: center;">
                            <span>🌡️ Temperature</span>
                            <span id="tempValue">--°C</span>
                        </div>
                    </div>
                    
                    <div style="margin-bottom: 10px;">
                        <div style="display: flex; justify-content: space-between; align-items: center;">
                            <span>💧 Humidity</span>
                            <span id="humidityValue">--%</span>
                        </div>
                    </div>
                    
                    <div style="margin-bottom: 10px;">
                        <div style="display: flex; justify-content: space-between; align-items: center;">
                            <span>💨 Wind Speed</span>
                            <span id="windValue">-- m/s</span>
                        </div>
                    </div>
                    
                    <div style="margin-bottom: 10px;">
                        <div style="display: flex; justify-content: space-between; align-items: center;">
                            <span>☀️ Light Level</span>
                            <span id="lightValue">--%</span>
                        </div>
                    </div>
                    
                    <div style="margin-bottom: 10px;">
                        <div style="display: flex; justify-content: space-between; align-items: center;">
                            <span>🌱 Soil Moisture</span>
                            <span id="soilValue">--%</span>
                        </div>
                    </div>
                </div>
            </div>
            
            <div class="card">
                <h3>🚨 Quick Alerts</h3>
                <div id="quickAlerts">
                    <div style="background: rgba(46, 204, 113, 0.2); padding: 10px; border-radius: 8px; margin-bottom: 8px; border-left: 4px solid #2ecc71;">
                        <div style="font-weight: bold; color: #2ecc71;">✅ System Online</div>
                        <div style="font-size: 0.9em; color: #bdc3c7;">All systems operational</div>
                    </div>
                    
                    <div style="background: rgba(241, 196, 15, 0.2); padding: 10px; border-radius: 8px; margin-bottom: 8px; border-left: 4px solid #f1c40f;">
                        <div style="font-weight: bold; color: #f1c40f;">⚠️ GPS Signal</div>
                        <div style="font-size: 0.9em; color: #bdc3c7;">Weak signal detected</div>
                    </div>
                    
                    <div style="background: rgba(52, 152, 219, 0.2); padding: 10px; border-radius: 8px; margin-bottom: 8px; border-left: 4px solid #3498db;">
                        <div style="font-weight: bold; color: #3498db;">ℹ️ Data Sync</div>
                        <div style="font-size: 0.9em; color: #bdc3c7;">Syncing field data</div>
                    </div>
                </div>
            </div>
        </div>
        
        <!-- Main Content Area -->
        <div class="main-content">
            <div class="header">
                <h1>🚜 TRACTOBOTS ENHANCED DASHBOARD</h1>
                <p>Professional Agricultural Robotics Control Center</p>
                <p style="margin-top: 10px; color: #bdc3c7;">Real-time Monitoring • Advanced Analytics • Precision Control</p>
            </div>
            
            <!-- Tab Navigation -->
            <div class="tab-nav">
                <button class="tab-btn active" onclick="showTab('overview')">📊 Overview</button>
                <button class="tab-btn" onclick="showTab('position')">📍 Position</button>
                <button class="tab-btn" onclick="showTab('performance')">⚡ Performance</button>
                <button class="tab-btn" onclick="showTab('environment')">🌿 Environment</button>
                <button class="tab-btn" onclick="showTab('logs')">📝 Logs</button>
            </div>
            
            <!-- Tab Contents -->
            <div id="overview-tab" class="tab-content active">
                <div class="card">
                    <h2>🚜 Live Tractor Data</h2>
                    <div class="data-grid">
                        <div class="data-card">
                            <div class="data-label">Position (X, Y, Z)</div>
                            <div class="data-value" id="position" style="color: #3498db;">0.0, 0.0, 0.0</div>
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
                            <div class="data-label">Battery</div>
                            <div class="data-value" id="battery" style="color: #e74c3c;">85%</div>
                            <div class="progress-bar">
                                <div class="progress-fill" id="batteryProgress" style="width: 85%; background: linear-gradient(90deg, #2ecc71, #27ae60);"></div>
                            </div>
                        </div>
                    </div>
                </div>
                
                <div class="card">
                    <h2>📈 Mission Progress</h2>
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
                    <h2>📍 Position Tracking</h2>
                    <div style="background: rgba(44, 62, 80, 0.6); height: 400px; border-radius: 10px; padding: 20px; text-align: center;">
                        <div style="margin-top: 150px; font-size: 1.2em; color: #bdc3c7;">
                            🗺️ Interactive Position Map<br>
                            📍 Real-time tractor location<br>
                            🛤️ Planned and completed paths<br>
                            🎯 Waypoint navigation
                        </div>
                    </div>
                </div>
                
                <div class="card">
                    <h2>📊 Position History</h2>
                    <div class="data-grid">
                        <div class="data-card">
                            <div class="gauge">
                                <div class="gauge-inner">
                                    <div class="data-value" style="font-size: 1.8em;" id="speedGauge">2.5</div>
                                    <div class="data-label">m/s</div>
                                </div>
                            </div>
                            <div class="data-label">Current Speed</div>
                        </div>
                        <div class="data-card">
                            <div class="data-label">Distance Traveled</div>
                            <div class="data-value" id="distanceTraveled" style="color: #3498db;">1.2</div>
                            <div class="data-label">kilometers</div>
                        </div>
                    </div>
                </div>
            </div>
            
            <div id="performance-tab" class="tab-content">
                <div class="card">
                    <h2>⚡ Performance Analytics</h2>
                    <div class="data-grid">
                        <div class="data-card">
                            <div class="data-label">Fuel Consumption</div>
                            <div class="data-value" id="fuelConsumption" style="color: #e74c3c;">12.3</div>
                            <div class="data-label">L/hour</div>
                        </div>
                        <div class="data-card">
                            <div class="data-label">Engine Load</div>
                            <div class="data-value" id="engineLoad" style="color: #f39c12;">75%</div>
                            <div class="progress-bar">
                                <div class="progress-fill" style="width: 75%; background: linear-gradient(90deg, #f39c12, #e67e22);"></div>
                            </div>
                        </div>
                        <div class="data-card">
                            <div class="data-label">Hydraulic Pressure</div>
                            <div class="data-value" id="hydraulicPressure" style="color: #9b59b6;">180</div>
                            <div class="data-label">bar</div>
                        </div>
                        <div class="data-card">
                            <div class="data-label">PTO Speed</div>
                            <div class="data-value" id="ptoSpeed" style="color: #1abc9c;">540</div>
                            <div class="data-label">rpm</div>
                        </div>
                    </div>
                </div>
            </div>
            
            <div id="environment-tab" class="tab-content">
                <div class="card">
                    <h2>🌿 Environmental Monitoring</h2>
                    <div class="data-grid">
                        <div class="data-card">
                            <div class="data-label">Temperature</div>
                            <div class="data-value" id="temperature" style="color: #e74c3c;">22°C</div>
                        </div>
                        <div class="data-card">
                            <div class="data-label">Humidity</div>
                            <div class="data-value" id="humidity" style="color: #3498db;">65%</div>
                        </div>
                        <div class="data-card">
                            <div class="data-label">Wind Speed</div>
                            <div class="data-value" id="windSpeed" style="color: #95a5a6;">2.5</div>
                            <div class="data-label">m/s</div>
                        </div>
                        <div class="data-card">
                            <div class="data-label">Soil Moisture</div>
                            <div class="data-value" id="soilMoisture" style="color: #8e44ad;">45%</div>
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
            document.getElementById('currentTime').textContent = now.toLocaleString();
        }
        
        function controlAction(action) {
            const messages = {
                'start_ros2': '🤖 ROS2 system startup initiated...',
                'start_gazebo': '🌍 Gazebo simulation launching...',
                'start_navigation': '🧭 Navigation system starting...',
                'start_gps': '📡 GPS initialization in progress...',
                'start_camera': '📷 Camera system activating...',
                'emergency_stop': '🚨 EMERGENCY STOP - All systems halting immediately!'
            };
            
            const message = messages[action] || 'Unknown action';
            alert(message);
            
            // Add to logs
            const timestamp = new Date().toLocaleTimeString();
            const logEntry = `[${timestamp}] ${message}`;
            addLogEntry(logEntry);
        }
        
        function addLogEntry(entry) {
            const logsContainer = document.getElementById('logsContainer');
            const logDiv = document.createElement('div');
            logDiv.className = 'log-entry';
            logDiv.textContent = entry;
            logsContainer.appendChild(logDiv);
            logsContainer.scrollTop = logsContainer.scrollHeight;
        }
        
        function getSystemIcon(system) {
            const icons = {
                'ros2': '🤖',
                'gazebo': '🌍',
                'navigation': '🧭',
                'gps': '📡',
                'simulation': '🎮',
                'camera': '📷',
                'lidar': '📡',
                'imu': '⚖️'
            };
            return icons[system] || '⚙️';
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
                            <div>
                                <span class="status-icon">${getSystemIcon(system)}</span>
                                ${system.toUpperCase()}
                            </div>
                            <div class="status-text">${status ? '🟢 ONLINE' : '🔴 OFFLINE'}</div>
                        `;
                        statusDiv.appendChild(statusItem);
                    }
                    
                    // Update live data
                    const pos = data.live_data.position;
                    document.getElementById('position').textContent = `${pos.x.toFixed(1)}, ${pos.y.toFixed(1)}, ${pos.z.toFixed(1)}`;
                    document.getElementById('speed').textContent = data.live_data.speed.toFixed(1);
                    document.getElementById('heading').textContent = `${data.live_data.heading.toFixed(0)}°`;
                    
                    const batteryLevel = data.live_data.battery.toFixed(0);
                    document.getElementById('battery').textContent = `${batteryLevel}%`;
                    document.getElementById('batteryProgress').style.width = `${batteryLevel}%`;
                    
                    // Update battery color
                    const batteryProgress = document.getElementById('batteryProgress');
                    if (batteryLevel > 60) {
                        batteryProgress.style.background = 'linear-gradient(90deg, #2ecc71, #27ae60)';
                    } else if (batteryLevel > 30) {
                        batteryProgress.style.background = 'linear-gradient(90deg, #f39c12, #e67e22)';
                    } else {
                        batteryProgress.style.background = 'linear-gradient(90deg, #e74c3c, #c0392b)';
                    }
                    
                    // Update mission progress
                    const coverage = data.live_data.field_coverage.toFixed(0);
                    document.getElementById('fieldCoverage').textContent = `${coverage}%`;
                    document.getElementById('coverageProgress').style.width = `${coverage}%`;
                    
                    const mission = data.live_data.mission_progress.toFixed(0);
                    document.getElementById('missionProgress').textContent = `${mission}%`;
                    document.getElementById('missionProgressBar').style.width = `${mission}%`;
                    
                    document.getElementById('workRate').textContent = data.live_data.work_rate.toFixed(1);
                    document.getElementById('efficiency').textContent = `${data.live_data.efficiency.toFixed(0)}%`;
                    document.getElementById('fuelConsumption').textContent = data.live_data.fuel_consumption.toFixed(1);
                    
                    // Update environmental data
                    document.getElementById('temperature').textContent = `${data.environment.temperature.toFixed(1)}°C`;
                    document.getElementById('humidity').textContent = `${data.environment.humidity.toFixed(0)}%`;
                    document.getElementById('windSpeed').textContent = data.environment.wind_speed.toFixed(1);
                    document.getElementById('soilMoisture').textContent = `${data.environment.soil_moisture.toFixed(0)}%`;
                    
                    // Update system resources with enhanced progress bars
                    const stats = data.stats;
                    
                    // Update CPU
                    document.getElementById('cpuValue').textContent = `${stats.cpu_usage.toFixed(1)}%`;
                    document.getElementById('cpuBar').style.width = `${stats.cpu_usage}%`;
                    document.getElementById('cpuText').textContent = `${stats.cpu_usage.toFixed(1)}%`;
                    
                    // Update Memory
                    document.getElementById('memoryValue').textContent = `${stats.memory_usage.toFixed(1)}%`;
                    document.getElementById('memoryBar').style.width = `${stats.memory_usage}%`;
                    document.getElementById('memoryText').textContent = `${stats.memory_usage.toFixed(1)}%`;
                    
                    // Update Disk
                    document.getElementById('diskValue').textContent = `${stats.disk_usage.toFixed(1)}%`;
                    document.getElementById('diskBar').style.width = `${stats.disk_usage}%`;
                    document.getElementById('diskText').textContent = `${stats.disk_usage.toFixed(1)}%`;
                    
                    // Update Network
                    document.getElementById('networkValue').textContent = `${stats.network_activity.toFixed(1)} MB/s`;
                    document.getElementById('networkBar').style.width = `${Math.min(stats.network_activity * 2, 100)}%`;
                    document.getElementById('networkText').textContent = `${stats.network_activity.toFixed(1)} MB/s`;
                    
                    // Update GPU
                    document.getElementById('gpuValue').textContent = `${stats.gpu_usage.toFixed(1)}%`;
                    document.getElementById('gpuBar').style.width = `${stats.gpu_usage}%`;
                    document.getElementById('gpuText').textContent = `${stats.gpu_usage.toFixed(1)}%`;
                    
                    // Update environmental data in sidebar
                    document.getElementById('tempValue').textContent = `${data.environment.temperature.toFixed(1)}°C`;
                    document.getElementById('humidityValue').textContent = `${data.environment.humidity.toFixed(0)}%`;
                    document.getElementById('windValue').textContent = `${data.environment.wind_speed.toFixed(1)} m/s`;
                    document.getElementById('lightValue').textContent = `${data.environment.light_level.toFixed(0)}%`;
                    document.getElementById('soilValue').textContent = `${data.environment.soil_moisture.toFixed(0)}%`;
                    
                    // Update gauges
                    document.getElementById('speedGauge').textContent = data.live_data.speed.toFixed(1);
                    
                    // Update other metrics
                    document.getElementById('distanceTraveled').textContent = (data.live_data.speed * 0.1).toFixed(1);
                    document.getElementById('engineLoad').textContent = '75';
                    document.getElementById('hydraulicPressure').textContent = '180';
                    document.getElementById('ptoSpeed').textContent = '540';
                    
                    // Update alerts
                    const alertsContainer = document.getElementById('alertsContainer');
                    alertsContainer.innerHTML = '';
                    
                    data.alerts.forEach(alert => {
                        const alertDiv = document.createElement('div');
                        alertDiv.className = `alert ${alert.type}`;
                        alertDiv.innerHTML = `
                            <strong>[${alert.time}]</strong> ${alert.message}
                        `;
                        alertsContainer.appendChild(alertDiv);
                    });
                    
                    // Update logs
                    const logsContainer = document.getElementById('logsContainer');
                    logsContainer.innerHTML = '';
                    data.logs.forEach(log => {
                        const logDiv = document.createElement('div');
                        logDiv.className = 'log-entry';
                        logDiv.textContent = log;
                        logsContainer.appendChild(logDiv);
                    });
                    logsContainer.scrollTop = logsContainer.scrollHeight;
                })
                .catch(error => {
                    console.error('Error:', error);
                    document.getElementById('connectionStatus').className = 'offline';
                    document.getElementById('connectionStatus').innerHTML = '🔴 Disconnected';
                });
        }
        
        // Initialize
        setInterval(updateTime, 1000);
        setInterval(updateDashboard, 1000);
        updateTime();
        updateDashboard();
    </script>
</body>
</html>
    '''
    
    @app.route('/')
    def index():
        return html_template
    
    @app.route('/api/data')
    def get_data():
        return jsonify(dashboard_data)
    
    return app

def main():
    """Main function to start the enhanced dashboard"""
    print("🚜 TRACTOBOTS ENHANCED DASHBOARD")
    print("=" * 60)
    print("🎯 Creating professional interface with fixed sidebar...")
    print("📊 Enhanced features and real-time monitoring...")
    print("")
    
    try:
        # Install Flask if needed
        try:
            import flask
            print("✅ Flask is available")
        except ImportError:
            print("📦 Installing Flask...")
            subprocess.run([sys.executable, '-m', 'pip', 'install', 'flask'], 
                          capture_output=True)
            print("✅ Flask installed successfully")
        
        # Create and start the enhanced dashboard
        app = create_enhanced_dashboard()
        
        print("✅ Enhanced dashboard created successfully!")
        print("🌐 Starting web server on http://localhost:5000")
        print("📱 Opening browser automatically...")
        print("🔄 Real-time updates every second")
        print("🛑 Press Ctrl+C to stop")
        print("")
        
        # Open browser after a short delay
        def open_browser():
            time.sleep(2)
            webbrowser.open('http://localhost:5000')
        
        browser_thread = threading.Thread(target=open_browser, daemon=True)
        browser_thread.start()
        
        # Start the Flask app
        app.run(host='0.0.0.0', port=5000, debug=False)
        
    except KeyboardInterrupt:
        print("\n🛑 Enhanced dashboard stopped by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        print("Please check the error message above for troubleshooting.")

if __name__ == "__main__":
    main()
