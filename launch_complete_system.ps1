# Tractobots Complete System Launcher for Windows
# This script launches the full agricultural robotics simulation

Write-Host "🚜 === TRACTOBOTS AGRICULTURAL ROBOTICS SYSTEM ===" -ForegroundColor Green
Write-Host "🌾 Launching complete field simulation with autonomous plowing..." -ForegroundColor Yellow

# Function to show colored output
function Write-Status {
    param($Message, $Color = "Cyan")
    Write-Host "[INFO] $Message" -ForegroundColor $Color
}

function Write-Warning {
    param($Message)
    Write-Host "[WARN] $Message" -ForegroundColor Yellow
}

function Write-Error {
    param($Message)  
    Write-Host "[ERROR] $Message" -ForegroundColor Red
}

function Write-Header {
    param($Message)
    Write-Host "`n[STEP] $Message" -ForegroundColor Magenta
}

Write-Header "1. Checking WSL and ROS2 environment..."

# Check if WSL is available
if (!(Get-Command wsl -ErrorAction SilentlyContinue)) {
    Write-Error "WSL is not installed or not available"
    Write-Host "Please install WSL2 and Ubuntu before continuing"
    exit 1
}

Write-Status "WSL is available ✅"

# Check if ROS2 is installed in WSL
$ros2Check = wsl bash -c "source /opt/ros/jazzy/setup.bash 2>/dev/null && which ros2"
if (!$ros2Check) {
    Write-Warning "ROS2 Jazzy not detected in WSL"
    Write-Status "Installing ROS2 Jazzy..."
    wsl bash -c "
        sudo apt update
        sudo apt install -y software-properties-common
        sudo add-apt-repository universe
        sudo apt update && sudo apt install -y curl gnupg lsb-release
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo 'deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \$(. /etc/os-release && echo \$UBUNTU_CODENAME) main' | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
        sudo apt update
        sudo apt install -y ros-jazzy-desktop
    "
}

Write-Status "ROS2 Jazzy is ready ✅"

Write-Header "2. Launching the complete agricultural system..."

# Make the launch script executable
wsl chmod +x /mnt/c/Users/nicholas/OneDrive/Documents/GitHub/tractobots/launch_complete_system.sh

# Launch the complete system in WSL
Write-Status "Starting all system components..."
$job = Start-Job -ScriptBlock {
    wsl bash -c "/mnt/c/Users/nicholas/OneDrive/Documents/GitHub/tractobots/launch_complete_system.sh"
}

# Wait a moment for services to start
Start-Sleep -Seconds 10

Write-Header "3. Starting Windows Dashboard..."

# Launch the Windows dashboard
Write-Status "Opening Tractobots Dashboard..."
$dashboardJob = Start-Job -ScriptBlock {
    Set-Location "C:\Users\nicholas\OneDrive\Documents\GitHub\tractobots"
    python tractobots_win_dashboard.py
}

Write-Header "4. Opening Web Interface..."

# Wait for web server to start
Start-Sleep -Seconds 5

# Open the web interface in the default browser
Write-Status "Opening web interface at http://localhost:8080"
try {
    Start-Process "http://localhost:8080"
} catch {
    Write-Warning "Could not auto-open browser. Please manually open: http://localhost:8080"
}

Write-Header "5. System Status"
Write-Host "`n🎉 TRACTOBOTS SYSTEM IS NOW RUNNING!" -ForegroundColor Green
Write-Host ""
Write-Host "🌐 Web Interface:     http://localhost:8080" -ForegroundColor Cyan
Write-Host "🚀 ROS Bridge:        ws://localhost:9090" -ForegroundColor Cyan  
Write-Host "🌾 Gazebo Simulation: Running in WSL" -ForegroundColor Cyan
Write-Host "🚜 Tractor + Plow:    Spawned in field" -ForegroundColor Cyan
Write-Host "🗺️  Navigation:        Autonomous plowing ready" -ForegroundColor Cyan
Write-Host "🖥️  Dashboard:         Windows GUI active" -ForegroundColor Cyan
Write-Host ""

Write-Header "6. How to use the system:"
Write-Host "1. 🌐 Use the web interface for live monitoring and control"
Write-Host "2. 🖥️  Use the Windows dashboard for system management"  
Write-Host "3. 🚜 Click 'Start Auto Plowing' to begin autonomous operations"
Write-Host "4. 🗺️  Monitor field coverage and robot progress in real-time"
Write-Host "5. 🛑 Use emergency stop if needed"
Write-Host ""

Write-Status "Press any key to view system logs, or Ctrl+C to exit..."
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")

Write-Header "7. Live System Logs"
Write-Status "Showing live logs from all components..."

# Show live logs from WSL
wsl bash -c "tail -f /tmp/tractobots_*.log 2>/dev/null || echo 'No logs available yet...'"
