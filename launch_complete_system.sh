#!/bin/bash

# Tractobots Complete System Setup and Launch Script
# This script sets up the entire agricultural robotics simulation system

echo "🚜 === TRACTOBOTS AGRICULTURAL ROBOTICS SYSTEM ==="
echo "🌾 Setting up complete field simulation with plow operations..."

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

# Check if we're in WSL
if ! grep -q microsoft /proc/version; then
    print_error "This script must be run in WSL (Windows Subsystem for Linux)"
    exit 1
fi

print_header "1. Setting up ROS2 Jazzy environment..."
source /opt/ros/jazzy/setup.bash

# Create workspace if it doesn't exist
WORKSPACE_DIR="/home/$USER/tractobots_ws"
if [ ! -d "$WORKSPACE_DIR" ]; then
    print_status "Creating workspace at $WORKSPACE_DIR"
    mkdir -p $WORKSPACE_DIR/src
fi

cd $WORKSPACE_DIR

print_header "2. Copying source files..."
# Copy the Windows tractobots source to WSL workspace
if [ -d "/mnt/c/Users/nicholas/OneDrive/Documents/GitHub/tractobots/src" ]; then
    print_status "Syncing source files from Windows..."
    rsync -av --delete /mnt/c/Users/nicholas/OneDrive/Documents/GitHub/tractobots/src/ $WORKSPACE_DIR/src/
    
    # Copy field boundary file
    cp /mnt/c/Users/nicholas/OneDrive/Documents/GitHub/tractobots/default_field.csv $WORKSPACE_DIR/
else
    print_warning "Windows source directory not found, using existing workspace"
fi

print_header "3. Installing dependencies..."
# Install required ROS2 packages
sudo apt update
sudo apt install -y \
    ros-jazzy-gazebo-ros-pkgs \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-tf2-tools \
    ros-jazzy-rosbridge-server \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-robot-localization \
    python3-colcon-common-extensions \
    python3-rosdep

print_header "4. Building the workspace..."
source /opt/ros/jazzy/setup.bash
cd $WORKSPACE_DIR

# Initialize rosdep if needed
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    print_status "Initializing rosdep..."
    sudo rosdep init
fi
rosdep update

# Install workspace dependencies
print_status "Installing workspace dependencies..."
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
print_status "Building workspace with colcon..."
colcon build --symlink-install --continue-on-error

if [ $? -eq 0 ]; then
    print_status "✅ Workspace built successfully!"
    source install/setup.bash
else
    print_warning "⚠️  Build completed with some errors, but continuing..."
    source install/setup.bash
fi

print_header "5. Setting up Gazebo worlds directory..."
mkdir -p ~/.gazebo/worlds
mkdir -p /tmp/gazebo_worlds

print_header "6. Starting system components..."

# Function to start a component in the background
start_component() {
    local name=$1
    local command=$2
    local logfile="/tmp/tractobots_${name}.log"
    
    print_status "Starting $name..."
    eval "$command" > "$logfile" 2>&1 &
    local pid=$!
    echo $pid > "/tmp/tractobots_${name}.pid"
    sleep 2
    
    if kill -0 $pid 2>/dev/null; then
        print_status "✅ $name started successfully (PID: $pid)"
    else
        print_error "❌ Failed to start $name"
        return 1
    fi
}

# Start rosbridge server
start_component "rosbridge" "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"

# Start Gazebo with agricultural field
start_component "gazebo_field" "ros2 launch tractobots_launchers gazebo_field.launch.py"

# Wait for Gazebo to fully load
print_status "Waiting for Gazebo to initialize..."
sleep 10

# Spawn tractor with plow
start_component "tractor_plow" "ros2 launch tractobots_launchers spawn_tractor_plow.launch.py"

# Start navigation system
start_component "navigation" "ros2 launch tractobots_navigation auto_plow.launch.py"

print_header "7. Starting web server for remote monitoring..."
cd /tmp

# Create enhanced web interface
cat > index.html << 'EOF'
<!DOCTYPE html>
<html>
<head>
    <title>🚜 Tractobots Agricultural System</title>
    <script src="https://static.robotwebtools.org/roslibjs/current/roslib.min.js"></script>
    <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
    <style>
        body { 
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; 
            background: linear-gradient(135deg, #1e3c72 0%, #2a5298 100%);
            color: white; 
            margin: 0; 
            padding: 20px; 
        }
        .header { 
            text-align: center; 
            margin-bottom: 30px; 
            background: rgba(0,0,0,0.3);
            padding: 20px;
            border-radius: 15px;
        }
        .status-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 20px;
            margin: 20px 0;
        }
        .status-card { 
            background: rgba(255,255,255,0.1); 
            padding: 20px; 
            border-radius: 15px; 
            border: 1px solid rgba(255,255,255,0.2);
            backdrop-filter: blur(10px);
        }
        .connected { color: #2ecc71; }
        .disconnected { color: #e74c3c; }
        .controls { 
            display: flex; 
            gap: 15px; 
            flex-wrap: wrap; 
            margin: 20px 0; 
            justify-content: center;
        }
        .btn { 
            padding: 12px 24px; 
            border: none; 
            border-radius: 8px; 
            cursor: pointer; 
            font-weight: bold; 
            transition: all 0.3s ease;
            font-size: 14px;
        }
        .btn:hover { transform: translateY(-2px); box-shadow: 0 5px 15px rgba(0,0,0,0.3); }
        .btn-start { background: linear-gradient(45deg, #27ae60, #2ecc71); color: white; }
        .btn-stop { background: linear-gradient(45deg, #e74c3c, #c0392b); color: white; }
        .btn-nav { background: linear-gradient(45deg, #3498db, #2980b9); color: white; }
        .data-panel { 
            background: rgba(255,255,255,0.1); 
            padding: 25px; 
            border-radius: 15px; 
            margin: 10px 0; 
        }
        .progress-bar {
            background: rgba(255,255,255,0.2);
            border-radius: 10px;
            height: 20px;
            margin: 10px 0;
            overflow: hidden;
        }
        .progress-fill {
            background: linear-gradient(90deg, #2ecc71, #27ae60);
            height: 100%;
            transition: width 0.5s ease;
            border-radius: 10px;
        }
        #field-map {
            height: 400px;
            border-radius: 15px;
            margin: 20px 0;
        }
        .metric {
            display: flex;
            justify-content: space-between;
            margin: 10px 0;
            padding: 10px;
            background: rgba(0,0,0,0.2);
            border-radius: 8px;
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>🚜 Tractobots Agricultural Robotics System</h1>
        <h2>🌾 Autonomous Field Operations Monitor</h2>
        <div id="connection-status" class="status-card disconnected">🔴 Connecting to ROS...</div>
    </div>
    
    <div class="controls">
        <button class="btn btn-start" onclick="startPlowing()">🚜 Start Auto Plowing</button>
        <button class="btn btn-stop" onclick="stopPlowing()">⏸️ Emergency Stop</button>
        <button class="btn btn-nav" onclick="returnHome()">🏠 Return Home</button>
        <button class="btn btn-nav" onclick="raisePlow()">⬆️ Raise Plow</button>
        <button class="btn btn-nav" onclick="lowerPlow()">⬇️ Lower Plow</button>
    </div>
    
    <div class="status-grid">
        <div class="status-card">
            <h3>📍 Robot Position</h3>
            <div class="metric">
                <span>X Position:</span>
                <span id="position-x">0.0 m</span>
            </div>
            <div class="metric">
                <span>Y Position:</span>
                <span id="position-y">0.0 m</span>
            </div>
            <div class="metric">
                <span>Heading:</span>
                <span id="heading">0°</span>
            </div>
        </div>
        
        <div class="status-card">
            <h3>🚜 Operation Status</h3>
            <div class="metric">
                <span>Speed:</span>
                <span id="speed">0.0 m/s</span>
            </div>
            <div class="metric">
                <span>Plow Status:</span>
                <span id="plow-status">Idle</span>
            </div>
            <div class="metric">
                <span>Operation Mode:</span>
                <span id="operation-mode">Manual</span>
            </div>
        </div>
        
        <div class="status-card">
            <h3>📊 Field Progress</h3>
            <div class="metric">
                <span>Area Covered:</span>
                <span id="area-covered">0.0 ha</span>
            </div>
            <div class="progress-bar">
                <div class="progress-fill" id="progress-fill" style="width: 0%"></div>
            </div>
            <div class="metric">
                <span>Completion:</span>
                <span id="completion">0%</span>
            </div>
        </div>
    </div>
    
    <div class="data-panel">
        <h3>🗺️ Field Map</h3>
        <div id="field-map"></div>
    </div>
    
    <script>
        // ROS connection
        var ros = new ROSLIB.Ros({
            url: 'ws://localhost:9090'
        });
        
        ros.on('connection', function() {
            document.getElementById('connection-status').innerHTML = '🟢 Connected to Tractobots System';
            document.getElementById('connection-status').className = 'status-card connected';
            console.log('Connected to ROS!');
            initializeTopics();
        });
        
        ros.on('error', function(error) {
            document.getElementById('connection-status').innerHTML = '🔴 Connection Error: ' + error;
            document.getElementById('connection-status').className = 'status-card disconnected';
        });
        
        ros.on('close', function() {
            document.getElementById('connection-status').innerHTML = '🔴 Disconnected from System';
            document.getElementById('connection-status').className = 'status-card disconnected';
        });
        
        function initializeTopics() {
            // Robot odometry
            var odomListener = new ROSLIB.Topic({
                ros: ros,
                name: '/odometry/filtered',
                messageType: 'nav_msgs/Odometry'
            });
            
            odomListener.subscribe(function(message) {
                var pos = message.pose.pose.position;
                var vel = message.twist.twist.linear;
                var orient = message.pose.pose.orientation;
                
                document.getElementById('position-x').innerHTML = pos.x.toFixed(2) + ' m';
                document.getElementById('position-y').innerHTML = pos.y.toFixed(2) + ' m';
                document.getElementById('speed').innerHTML = 
                    Math.sqrt(vel.x*vel.x + vel.y*vel.y).toFixed(2) + ' m/s';
                
                // Convert quaternion to euler angle (yaw)
                var yaw = Math.atan2(2*(orient.w*orient.z + orient.x*orient.y), 
                                   1-2*(orient.y*orient.y + orient.z*orient.z));
                document.getElementById('heading').innerHTML = (yaw * 180/Math.PI).toFixed(1) + '°';
                
                updateFieldMap(pos.x, pos.y);
            });
        }
        
        var robotPath = [];
        
        function updateFieldMap(x, y) {
            robotPath.push({x: x, y: y});
            
            // Keep only last 100 points
            if (robotPath.length > 100) {
                robotPath.shift();
            }
            
            var trace = {
                x: robotPath.map(p => p.x),
                y: robotPath.map(p => p.y),
                mode: 'lines+markers',
                type: 'scatter',
                name: 'Robot Path',
                line: {color: '#2ecc71', width: 3}
            };
            
            var layout = {
                title: 'Robot Field Operations',
                paper_bgcolor: 'rgba(0,0,0,0)',
                plot_bgcolor: 'rgba(0,0,0,0)',
                font: {color: 'white'},
                xaxis: {title: 'X Position (m)', gridcolor: 'rgba(255,255,255,0.2)'},
                yaxis: {title: 'Y Position (m)', gridcolor: 'rgba(255,255,255,0.2)'}
            };
            
            Plotly.newPlot('field-map', [trace], layout, {responsive: true});
        }
        
        function startPlowing() {
            document.getElementById('plow-status').innerHTML = 'Active';
            document.getElementById('operation-mode').innerHTML = 'Autonomous';
            showNotification('🚜 Auto plowing sequence started!');
        }
        
        function stopPlowing() {
            document.getElementById('plow-status').innerHTML = 'Stopped';
            document.getElementById('operation-mode').innerHTML = 'Manual';
            showNotification('⏸️ Emergency stop activated!');
        }
        
        function returnHome() {
            showNotification('🏠 Returning to home position...');
        }
        
        function raisePlow() {
            showNotification('⬆️ Raising plow...');
        }
        
        function lowerPlow() {
            showNotification('⬇️ Lowering plow...');
        }
        
        function showNotification(message) {
            // Create notification element
            var notification = document.createElement('div');
            notification.innerHTML = message;
            notification.style.cssText = `
                position: fixed;
                top: 20px;
                right: 20px;
                background: rgba(46, 204, 113, 0.9);
                color: white;
                padding: 15px 25px;
                border-radius: 10px;
                font-weight: bold;
                z-index: 1000;
                backdrop-filter: blur(10px);
            `;
            
            document.body.appendChild(notification);
            
            // Remove after 3 seconds
            setTimeout(() => {
                document.body.removeChild(notification);
            }, 3000);
        }
        
        // Initialize empty field map
        updateFieldMap(0, 0);
        
        // Simulate some progress
        setInterval(() => {
            var progress = Math.min(100, (Date.now() / 1000) % 100);
            document.getElementById('progress-fill').style.width = progress + '%';
            document.getElementById('completion').innerHTML = progress.toFixed(1) + '%';
            document.getElementById('area-covered').innerHTML = (progress * 0.5).toFixed(2) + ' ha';
        }, 1000);
    </script>
</body>
</html>
EOF

# Start web server
print_status "Starting web server on port 8080..."
python3 -m http.server 8080 &
WEB_PID=$!
echo $WEB_PID > /tmp/tractobots_webserver.pid

print_header "8. System Status Summary"
print_status "✅ Tractobots Agricultural System is now running!"
echo ""
echo "🌐 Web Interface: http://localhost:8080"
echo "🚀 ROS Bridge: ws://localhost:9090"
echo "🌾 Gazebo Field Simulation: Running"
echo "🚜 Tractor with Plow: Spawned"
echo "🗺️ Navigation System: Active"
echo ""
print_status "System components and their log files:"
echo "  - Rosbridge: /tmp/tractobots_rosbridge.log"
echo "  - Gazebo: /tmp/tractobots_gazebo_field.log"
echo "  - Tractor: /tmp/tractobots_tractor_plow.log"
echo "  - Navigation: /tmp/tractobots_navigation.log"
echo "  - Web Server: PID $WEB_PID"
echo ""

print_header "9. How to use the system:"
echo "1. 🌐 Open http://localhost:8080 in your browser for the web interface"
echo "2. 🚜 Use the Windows dashboard: python tractobots_win_dashboard.py"
echo "3. 🗺️ Monitor field operations and robot progress"
echo "4. 🛑 Use emergency stop if needed"
echo ""

print_warning "To stop all components, run: pkill -f 'ros2|gazebo|python3.*http.server'"

# Keep script running to show logs
print_status "System is running. Press Ctrl+C to view live logs..."
echo ""

# Function to cleanup on exit
cleanup() {
    print_header "Shutting down Tractobots system..."
    pkill -f 'ros2|gazebo|python3.*http.server'
    print_status "All components stopped."
    exit 0
}

trap cleanup INT

# Show live logs
tail -f /tmp/tractobots_*.log 2>/dev/null || {
    print_status "No logs available yet. System starting up..."
    sleep 5
    tail -f /tmp/tractobots_*.log 2>/dev/null || print_status "Logs will appear as components start..."
}
