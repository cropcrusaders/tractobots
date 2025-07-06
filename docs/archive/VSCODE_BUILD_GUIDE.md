# 🚜 Building Tractobots in VS Code

This guide shows you how to build and run your autonomous tractor system directly in VS Code.

## 🚀 Quick Start

### 1. Open in VS Code
- Open this folder in VS Code
- VS Code will detect the project and configure automatically
- Open integrated terminal (Ctrl+` or View → Terminal)

### 2. One-Click Build
Press `Ctrl+Shift+P` and type "Tasks: Run Task", then select:
- **🚜 Build Tractobots (Full Setup)** - Complete build and setup
- **🔨 Colcon Build Only** - Quick rebuild after changes

Or use the shortcut: `Ctrl+Shift+B` to run the default build task.

### 3. Test Your System
After building, run:
- **🧪 Test Tractor System** - Verify everything works

## 🎮 Running the System

### Option 1: Use VS Code Tasks
Press `Ctrl+Shift+P` → "Tasks: Run Task" and select:

1. **🚀 Launch Tractor Bringup** - Main system (sensors, localization)
2. **🎮 Start Joy Node** - Joystick input (optional)
3. **🧭 Start Navigation Driver** - Tractor control and navigation
4. **🗺️ Launch MapViz Visualization** - Visual monitoring

### Option 2: Use Terminal Commands
```bash
# Terminal 1 - Main system
source ~/ros2_tractobots/install/setup.bash
ros2 launch tractobots_launchers bringup.launch.py

# Terminal 2 - Joystick (optional)
ros2 run joy joy_node

# Terminal 3 - Navigation
ros2 run tractobots_navigation driver

# Terminal 4 - Visualization (optional)
export DISPLAY=:0
ros2 launch tractobots_launchers mapviz.launch.py
```

## 🐛 Debugging

### Debug Python Nodes
1. Go to Run and Debug view (Ctrl+Shift+D)
2. Select a debug configuration:
   - **🧪 Debug System Test**
   - **🧭 Debug Navigation Driver** 
   - **🎮 Debug GPS Parser**
3. Press F5 to start debugging

### View Logs
- Build logs: Check the Terminal output
- Runtime logs: Each task runs in its own terminal panel
- ROS logs: `~/.ros/log/` directory

## ⚙️ Development Workflow

### 1. Edit Code
- Python files: Auto-formatted on save
- C++ files: IntelliSense and auto-completion
- Launch files: XML syntax highlighting

### 2. Build
- Press `Ctrl+Shift+B` for quick build
- Build errors appear in Problems panel (Ctrl+Shift+M)

### 3. Test
- Use **🧪 Test Tractor System** task
- Or run individual nodes for testing

### 4. Debug
- Set breakpoints (F9)
- Use debug configurations (F5)
- Inspect variables and call stack

## 🛑 Stopping Everything

Use the task: **🛑 Stop All Tractor Nodes**

Or in terminal:
```bash
pkill -f 'ros2|python3.*ros'
```

## 📁 Project Structure

```
.vscode/
├── tasks.json       # Build and run tasks
├── launch.json      # Debug configurations  
└── settings.json    # VS Code settings

src/
├── tractobots_bringup/     # Main launch files
├── tractobots_navigation/  # Tractor control
├── tractobots_gps/         # GPS interface
└── ...                     # Other packages
```

## 🔧 Keyboard Shortcuts

- `Ctrl+Shift+B` - Build project
- `Ctrl+Shift+P` - Command palette (run tasks)
- `Ctrl+Shift+D` - Debug view
- `Ctrl+`` - Toggle terminal
- `F5` - Start debugging
- `Ctrl+F5` - Run without debugging

## ⚡ Tips

1. **Multi-root workspace**: You can add other ROS workspaces to compare
2. **Terminal management**: Each task gets its own terminal panel
3. **Problem matching**: Build errors are automatically parsed
4. **IntelliSense**: Works for Python, C++, and ROS message types
5. **Git integration**: Built-in version control

## 🆘 Troubleshooting

### Build Issues
1. Check the Problems panel (Ctrl+Shift+M)
2. Look at Terminal output for detailed errors
3. Try "🔨 Colcon Build Only" task

### Runtime Issues  
1. Check each terminal panel for error messages
2. Verify ROS environment: `echo $ROS_DISTRO`
3. Test with "🧪 Test Tractor System"

### VS Code Issues
1. Reload window: Ctrl+Shift+P → "Developer: Reload Window"
2. Check .vscode/settings.json for configuration
3. Ensure ROS2 extensions are installed
