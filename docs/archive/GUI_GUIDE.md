# 🎨 TRACTOBOTS GUI INTERFACES

Your autonomous tractor system now includes **4 different GUI options** for controlling and monitoring operations!

## 🚀 **AVAILABLE GUI OPTIONS**

### 1. 📱 **Simple GUI** (Basic Tkinter)
- **Command**: `ros2 run tractobots_mission_ui mission_gui_node`
- **VS Code Task**: "📱 Launch Simple GUI"
- **Features**: Basic mission start/stop, emergency controls
- **Best for**: Quick testing, minimal interface
- **Requirements**: Built-in (no extra dependencies)

### 2. 🎨 **Enhanced GUI** (Modern Tkinter)
- **Command**: `ros2 run tractobots_mission_ui enhanced_gui`
- **VS Code Task**: "🎨 Launch Enhanced GUI"
- **Features**: 
  - Real-time sensor data display
  - Manual tractor controls (WASD-style)
  - Mission timer
  - Modern dark theme
  - GPS coordinates display
- **Best for**: Daily operations, comprehensive control
- **Requirements**: Built-in libraries only

### 3. 🌐 **Web Dashboard** (Browser-based)
- **Command**: `ros2 run tractobots_mission_ui web_dashboard`
- **VS Code Task**: "🌐 Launch Web Dashboard"
- **URL**: http://localhost:5000
- **Features**:
  - Real-time web interface
  - Works on any device with browser
  - Live sensor data updates
  - Touch-friendly mobile interface
  - System log display
  - Modern responsive design
- **Best for**: Remote monitoring, mobile access, multiple users
- **Requirements**: `flask`, `flask-socketio`

### 4. 💎 **Professional Qt GUI** (Advanced)
- **Command**: `ros2 run tractobots_mission_ui qt_gui`
- **VS Code Task**: "💎 Launch Qt Professional GUI"
- **Features**:
  - Real-time plotting (GPS track, speed, heading)
  - Professional interface design
  - 3D visualizations
  - Advanced data analysis
  - Highly responsive
- **Best for**: Advanced users, data analysis, professional operations
- **Requirements**: `PyQt5`, `pyqtgraph`, `numpy`

## 🛠️ **HOW TO USE**

### **From VS Code (RECOMMENDED)**
1. Press `Ctrl+Shift+P`
2. Type "Tasks: Run Task"
3. Choose your preferred GUI:
   - **"🎨 Launch Enhanced GUI"** - Most popular choice
   - **"🌐 Launch Web Dashboard"** - For browser/mobile access
   - **"💎 Launch Qt Professional GUI"** - For advanced features
   - **"📱 Launch Simple GUI"** - For basic testing

### **From Terminal**
```bash
# Enhanced GUI (recommended)
ros2 run tractobots_mission_ui enhanced_gui

# Web Dashboard
ros2 run tractobots_mission_ui web_dashboard
# Then open: http://localhost:5000

# Professional Qt GUI
ros2 run tractobots_mission_ui qt_gui

# Simple GUI
ros2 run tractobots_mission_ui mission_gui_node
```

### **Full System + GUI**
Launch everything at once:
- **VS Code Task**: "🚀 Launch Full System + GUI"
- **Manual**: `ros2 launch tractobots_launchers bringup.launch.py` + GUI

## 🎮 **GUI FEATURES**

### **Mission Control**
- ✅ **Start Mission** - Begin autonomous operations
- ⏹️ **Stop Mission** - End current mission
- ⏸️ **Pause Mission** - Temporary halt (some GUIs)

### **Emergency Controls**
- 🛑 **Emergency Stop** - Immediate system halt
- 🔄 **Reset E-Stop** - Clear emergency state
- 🔧 **System Check** - Diagnostic status

### **Manual Control**
- ⬆️ **Forward/Backward** - Linear movement
- ⬅️➡️ **Left/Right** - Steering control
- ⏹️ **Stop** - Immediate halt

### **Real-time Data**
- 📍 **GPS Position** - Latitude, longitude, altitude
- 🧭 **Heading** - Current direction in degrees
- 🏃 **Speed** - Current velocity in m/s
- ⏱️ **Mission Timer** - Elapsed mission time
- 📊 **System Status** - Current operational state

## 📦 **INSTALLATION REQUIREMENTS**

### **Built-in GUIs** (No extra install needed)
- Simple GUI
- Enhanced GUI

### **Web Dashboard**
```bash
pip install flask flask-socketio
```

### **Professional Qt GUI**
```bash
pip install PyQt5 pyqtgraph numpy
```

## 🔧 **TROUBLESHOOTING**

### **GUI Won't Start**
1. **Build first**: Run "🚜 Build Tractobots" task
2. **Check ROS2**: Ensure ROS2 is sourced in terminal
3. **Dependencies**: Install required packages (see above)

### **Web Dashboard Issues**
- **Port conflict**: Change port in `web_dashboard.py` if 5000 is busy
- **Access denied**: Try `http://127.0.0.1:5000` instead
- **No data**: Ensure ROS2 system is running first

### **Qt GUI Issues**
- **Import errors**: Install PyQt5 with `pip install PyQt5`
- **Display issues**: Set `export DISPLAY=:0` for WSL
- **Performance**: Qt GUI is resource-intensive

### **Connection Issues**
- **No sensor data**: Check if hardware nodes are running
- **Commands not working**: Verify ROS2 topic connections
- **Emergency stop stuck**: Use reset button or restart system

## 🎯 **RECOMMENDED WORKFLOWS**

### **Development/Testing**
1. Use **Enhanced GUI** for day-to-day development
2. Quick testing with **Simple GUI**
3. Remote monitoring with **Web Dashboard**

### **Field Operations**
1. **Web Dashboard** on tablet/phone for portability
2. **Enhanced GUI** on laptop for full control
3. **Professional Qt GUI** for data analysis

### **Multi-user Setup**
1. **Web Dashboard** for multiple simultaneous users
2. Each user can access http://localhost:5000
3. Real-time updates shared across all connections

## 🎉 **GUI GALLERY**

### **Enhanced GUI Features**
- 🎨 Modern dark theme with green accents
- 📊 Real-time sensor data cards
- 🎮 Intuitive manual control layout
- ⏱️ Live mission timer
- 🚨 Color-coded emergency controls

### **Web Dashboard Features**
- 📱 Mobile-responsive design
- 🌐 Cross-platform browser compatibility
- 📈 Live system log with timestamps
- 🗺️ GPS coordinate display
- ⚡ Real-time WebSocket updates

### **Professional Qt GUI Features**
- 📈 Live plotting of GPS track
- 📊 Real-time speed and heading graphs
- 💎 Professional interface design
- 🎯 Advanced data visualization
- 🖥️ Native desktop application feel

---

**Ready to control your autonomous tractor with style!** 🚜✨

Choose your preferred interface and start farming the future! 🌾🤖
