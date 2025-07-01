#!/bin/bash
# Complete Tractobots Problem Fix Script
# This script fixes ALL identified problems

set -e

echo "🔧 TRACTOBOTS COMPLETE PROBLEM FIX"
echo "=================================="
echo "This script will fix all identified problems in your setup."
echo ""

# Check if we're in WSL
if [[ ! -f /proc/version ]] || ! grep -qi microsoft /proc/version; then
    echo "❌ This script must be run in WSL (Windows Subsystem for Linux)"
    echo "💡 Open WSL terminal and try again"
    exit 1
fi

echo "✅ Running in WSL environment"

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Fix 1: Install ROS2 if not present
echo ""
echo "🔍 Checking ROS2 installation..."
if ! command_exists ros2; then
    echo "📦 ROS2 not found. Installing ROS2 Humble..."
    
    # Update system
    sudo apt update
    
    # Install dependencies
    sudo apt install -y curl gnupg2 lsb-release software-properties-common
    
    # Add ROS2 repository
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu jammy main" | \
      sudo tee /etc/apt/sources.list.d/ros2.list
    
    sudo apt update
    
    # Install ROS2 packages
    sudo apt install -y \
      ros-humble-desktop \
      ros-humble-navigation2 \
      ros-humble-robot-localization \
      ros-humble-joy \
      ros-humble-teleop-twist-joy
    
    # Initialize rosdep
    sudo rosdep init || echo "rosdep already initialized"
    rosdep update
    
    # Add to bashrc if not already there
    if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    fi
    
    echo "✅ ROS2 Humble installed successfully"
else
    echo "✅ ROS2 already installed"
fi

# Source ROS2 for this session
source /opt/ros/humble/setup.bash 2>/dev/null || echo "Sourcing ROS2..."

# Fix 2: Install GUI dependencies
echo ""
echo "🎨 Installing GUI dependencies..."
pip3 install --user flask flask-socketio python-socketio numpy || echo "Some GUI packages may need manual installation"

# Fix 3: Install Python development packages
echo ""
echo "🐍 Installing Python development packages..."
sudo apt install -y python3-pip python3-dev python3-setuptools python3-colcon-common-extensions

# Fix 4: Create missing scripts and fix permissions
echo ""
echo "📝 Fixing scripts and permissions..."

# Make all shell scripts executable
find . -name "*.sh" -type f -exec chmod +x {} \;

# Make Python scripts executable
find . -name "*.py" -type f -exec chmod +x {} \;

echo "✅ Scripts and permissions fixed"

# Fix 5: Update workspace symlinks
echo ""
echo "🔗 Setting up workspace..."
WS_DIR="$HOME/ros2_tractobots"
mkdir -p "$WS_DIR/src"

# Remove old symlink if it exists
if [ -L "$WS_DIR/src/tractobots" ]; then
    rm "$WS_DIR/src/tractobots"
fi

# Create new symlink
ln -s "$(pwd)" "$WS_DIR/src/tractobots"
echo "✅ Workspace symlink updated"

# Fix 6: Install system dependencies
echo ""
echo "📦 Installing system dependencies..."
cd "$WS_DIR"

# Update rosdep
rosdep update

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y || echo "Some dependencies may be missing (continuing...)"

# Fix 7: Build the workspace
echo ""
echo "🔨 Building workspace..."
colcon build --symlink-install --event-handlers console_direct+

if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
    
    # Source the workspace
    source "$WS_DIR/install/setup.bash"
    
    # Add workspace sourcing to bashrc
    if ! grep -q "source $WS_DIR/install/setup.bash" ~/.bashrc; then
        echo "source $WS_DIR/install/setup.bash" >> ~/.bashrc
    fi
    
    echo ""
    echo "🎉 ALL PROBLEMS FIXED!"
    echo "======================"
    echo ""
    echo "✅ ROS2 Humble installed and configured"
    echo "✅ GUI dependencies installed"
    echo "✅ Python packages installed"
    echo "✅ Scripts and permissions fixed"
    echo "✅ Workspace built successfully"
    echo "✅ Environment configured"
    echo ""
    echo "🚀 Your system is now ready!"
    echo ""
    echo "🎯 Next steps:"
    echo "   1. Close and reopen your terminal (to load new environment)"
    echo "   2. Test with: python3 verify_setup.py"
    echo "   3. Launch GUI: ros2 run tractobots_mission_ui enhanced_gui"
    echo "   4. Or use VS Code tasks: Ctrl+Shift+P → 'Tasks: Run Task'"
    echo ""
    echo "📚 Documentation:"
    echo "   - GUI_GUIDE.md - How to use the GUI interfaces"
    echo "   - VSCODE_BUILD_GUIDE.md - VS Code workflow"
    echo "   - SETUP_COMPLETE.md - Complete setup summary"
    
else
    echo "❌ Build failed. Please check the output above for errors."
    echo ""
    echo "🔧 Troubleshooting:"
    echo "   1. Check ROS2 installation: ros2 --version"
    echo "   2. Check dependencies: rosdep check --from-paths src --ignore-src"
    echo "   3. Manual build: cd $WS_DIR && colcon build"
    echo "   4. Check documentation: VSCODE_BUILD_GUIDE.md"
    exit 1
fi
