#!/usr/bin/env python3
"""
Web-based Tractobots Dashboard
Modern web interface using Flask and WebSocket for real-time updates
"""

import json
import threading
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry


class WebDashboardNode(Node):
    def __init__(self, socketio):
        super().__init__('web_dashboard_node')
        self.socketio = socketio
        
        # Publishers
        self.mission_pub = self.create_publisher(Bool, '/mission/start', 1)
        self.emergency_pub = self.create_publisher(Bool, '/emergency_stop', 1)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        
        # Subscribers
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 1)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        
        # State
        self.data = {
            'gps': {'lat': 0.0, 'lon': 0.0, 'alt': 0.0},
            'speed': 0.0,
            'heading': 0.0,
            'mission_active': False,
            'emergency_stop': False,
            'status': 'READY'
        }

    def gps_callback(self, msg):
        self.data['gps'] = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude
        }
        self.socketio.emit('sensor_data', {'type': 'gps', 'data': self.data['gps']})

    def imu_callback(self, msg):
        import math
        q = msg.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.data['heading'] = math.degrees(yaw)
        self.socketio.emit('sensor_data', {'type': 'heading', 'data': self.data['heading']})

    def odom_callback(self, msg):
        self.data['speed'] = (msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)**0.5
        self.socketio.emit('sensor_data', {'type': 'speed', 'data': self.data['speed']})

    def start_mission(self):
        if not self.data['emergency_stop']:
            self.mission_pub.publish(Bool(data=True))
            self.data['mission_active'] = True
            self.data['status'] = 'MISSION ACTIVE'
            self.socketio.emit('status_update', self.data)
            return True
        return False

    def stop_mission(self):
        self.mission_pub.publish(Bool(data=False))
        self.data['mission_active'] = False
        self.data['status'] = 'READY'
        self.socketio.emit('status_update', self.data)

    def trigger_estop(self):
        self.emergency_pub.publish(Bool(data=True))
        self.data['emergency_stop'] = True
        self.data['mission_active'] = False
        self.data['status'] = 'EMERGENCY STOP'
        self.socketio.emit('status_update', self.data)

    def reset_estop(self):
        self.emergency_pub.publish(Bool(data=False))
        self.data['emergency_stop'] = False
        self.data['status'] = 'READY'
        self.socketio.emit('status_update', self.data)

    def send_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        if not self.data['emergency_stop']:
            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            self.cmd_vel_pub.publish(twist)


# Flask app setup
app = Flask(__name__)
app.config['SECRET_KEY'] = 'tractobots_secret_key'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global variables
node = None

@app.route('/')
def dashboard():
    return render_template('dashboard.html')

@app.route('/api/mission/start', methods=['POST'])
def start_mission():
    if node and node.start_mission():
        return jsonify({'success': True, 'message': 'Mission started'})
    return jsonify({'success': False, 'message': 'Cannot start mission'})

@app.route('/api/mission/stop', methods=['POST'])
def stop_mission():
    if node:
        node.stop_mission()
        return jsonify({'success': True, 'message': 'Mission stopped'})
    return jsonify({'success': False, 'message': 'Error stopping mission'})

@app.route('/api/emergency/trigger', methods=['POST'])
def trigger_emergency():
    if node:
        node.trigger_estop()
        return jsonify({'success': True, 'message': 'Emergency stop triggered'})
    return jsonify({'success': False, 'message': 'Error triggering emergency stop'})

@app.route('/api/emergency/reset', methods=['POST'])
def reset_emergency():
    if node:
        node.reset_estop()
        return jsonify({'success': True, 'message': 'Emergency stop reset'})
    return jsonify({'success': False, 'message': 'Error resetting emergency stop'})

@app.route('/api/manual', methods=['POST'])
def manual_control():
    if node:
        data = request.json
        linear_x = data.get('linear_x', 0.0)
        angular_z = data.get('angular_z', 0.0)
        node.send_cmd_vel(linear_x, angular_z)
        return jsonify({'success': True})
    return jsonify({'success': False})

@app.route('/api/status')
def get_status():
    if node:
        return jsonify(node.data)
    return jsonify({'error': 'Node not initialized'})

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    if node:
        emit('status_update', node.data)

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')


def main(args=None):
    global node
    
    rclpy.init(args=args)
    node = WebDashboardNode(socketio)

    # Spin ROS2 in background thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Run Flask app
    print("🚜 Tractobots Web Dashboard starting...")
    print("🌐 Open your browser to: http://localhost:5000")
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)


if __name__ == '__main__':
    main()
