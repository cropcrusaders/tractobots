# 🚀 Tractobots Complete Environment Setup Script
# This script creates a fully functional Tractobots installation
# Last Updated: January 2025

param(
    [switch]$SkipWSL,
    [switch]$SkipPython,
    [switch]$SkipROS2,
    [switch]$TestOnly,
    [switch]$Verbose
)

# Script configuration
$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"

# Color functions
function Write-ColorText {
    param([string]$Text, [string]$Color = "White")
    $colors = @{
        "Red" = "Red"
        "Green" = "Green" 
        "Yellow" = "Yellow"
        "Blue" = "Blue"
        "Magenta" = "Magenta"
        "Cyan" = "Cyan"
        "White" = "White"
    }
    Write-Host $Text -ForegroundColor $colors[$Color]
}

function Write-Success { param([string]$Text) Write-ColorText "✅ $Text" "Green" }
function Write-Error { param([string]$Text) Write-ColorText "❌ $Text" "Red" }
function Write-Warning { param([string]$Text) Write-ColorText "⚠️ $Text" "Yellow" }
function Write-Info { param([string]$Text) Write-ColorText "ℹ️ $Text" "Blue" }
function Write-Header { param([string]$Text) Write-ColorText "`n🚀 $Text" "Cyan" }

# Logging
$LogFile = "tractobots_setup_$(Get-Date -Format 'yyyyMMdd_HHmmss').log"

function Write-Log {
    param([string]$Message, [string]$Level = "INFO")
    $timestamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
    $logEntry = "[$timestamp] [$Level] $Message"
    Add-Content -Path $LogFile -Value $logEntry
    if ($Verbose) { Write-Host $logEntry -ForegroundColor Gray }
}

# Main setup function
function Start-TractobotsSetup {
    Write-Header "Tractobots Complete Environment Setup"
    Write-Info "This script will set up a complete Tractobots development environment"
    Write-Info "Log file: $LogFile"
    
    try {
        Test-Prerequisites
        if (-not $SkipWSL) { Setup-WSLEnvironment }
        if (-not $SkipPython) { Setup-PythonEnvironment }
        if (-not $SkipROS2) { Setup-ROS2Environment }
        Setup-NetworkConfiguration
        Setup-TractobotsWorkspace
        Test-CompleteInstallation
        Show-CompletionMessage
    }
    catch {
        Write-Error "Setup failed: $($_.Exception.Message)"
        Write-Log "Setup failed: $($_.Exception.Message)" "ERROR"
        exit 1
    }
}

function Test-Prerequisites {
    Write-Header "Testing Prerequisites"
    
    # Test admin privileges
    if (-not ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole] "Administrator")) {
        Write-Error "This script requires Administrator privileges"
        Write-Info "Please run PowerShell as Administrator and try again"
        exit 1
    }
    Write-Success "Administrator privileges confirmed"
    Write-Log "Administrator privileges confirmed"
    
    # Test Windows version
    $version = [System.Environment]::OSVersion.Version
    if ($version.Major -lt 10) {
        Write-Error "Windows 10 or later is required"
        exit 1
    }
    Write-Success "Windows version compatible: $($version.Major).$($version.Minor)"
    Write-Log "Windows version: $($version.Major).$($version.Minor)"
    
    # Test internet connectivity
    try {
        Test-Connection "8.8.8.8" -Count 1 -Quiet
        Write-Success "Internet connectivity confirmed"
        Write-Log "Internet connectivity confirmed"
    }
    catch {
        Write-Error "No internet connectivity detected"
        exit 1
    }
}

function Setup-WSLEnvironment {
    Write-Header "Setting Up WSL Environment"
    
    # Enable WSL features
    Write-Info "Enabling WSL features..."
    Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux -NoRestart
    Enable-WindowsOptionalFeature -Online -FeatureName VirtualMachinePlatform -NoRestart
    Write-Success "WSL features enabled"
    Write-Log "WSL features enabled"
    
    # Set WSL default version
    wsl --set-default-version 2
    Write-Success "WSL 2 set as default"
    Write-Log "WSL 2 set as default"
    
    # Install Ubuntu 24.04 if not present
    $wslDistros = wsl --list --quiet
    if ($wslDistros -notcontains "Ubuntu-24.04") {
        Write-Info "Installing Ubuntu 24.04..."
        wsl --install Ubuntu-24.04
        Write-Success "Ubuntu 24.04 installed"
        Write-Log "Ubuntu 24.04 installed"
    }
    else {
        Write-Success "Ubuntu 24.04 already installed"
        Write-Log "Ubuntu 24.04 already present"
    }
    
    # Update Ubuntu
    Write-Info "Updating Ubuntu packages..."
    wsl -d Ubuntu-24.04 -- bash -c "sudo apt update && sudo apt upgrade -y"
    Write-Success "Ubuntu packages updated"
    Write-Log "Ubuntu packages updated"
}

function Setup-PythonEnvironment {
    Write-Header "Setting Up Python Environment"
    
    # Check Python installation
    try {
        $pythonVersion = python --version 2>&1
        if ($pythonVersion -match "Python (\d+)\.(\d+)") {
            $major = [int]$Matches[1]
            $minor = [int]$Matches[2]
            if ($major -ge 3 -and $minor -ge 8) {
                Write-Success "Python $pythonVersion already installed"
                Write-Log "Python version: $pythonVersion"
            }
            else {
                Write-Warning "Python version too old: $pythonVersion"
                Install-Python
            }
        }
    }
    catch {
        Write-Info "Python not found, installing..."
        Install-Python
    }
    
    # Install pip packages
    Write-Info "Installing Python dependencies..."
    $packages = @(
        "PyQt5>=5.15.0",
        "websocket-client>=1.0.0", 
        "roslibpy>=1.3.0",
        "matplotlib>=3.5.0",
        "numpy>=1.21.0",
        "requests>=2.25.0",
        "psutil>=5.8.0"
    )
    
    foreach ($package in $packages) {
        try {
            pip install $package --upgrade
            Write-Success "Installed: $package"
            Write-Log "Installed Python package: $package"
        }
        catch {
            Write-Warning "Failed to install: $package"
            Write-Log "Failed to install Python package: $package" "WARNING"
        }
    }
    
    # Create requirements.txt
    Write-Info "Creating requirements.txt..."
    $requirementsContent = @"
PyQt5>=5.15.0
websocket-client>=1.0.0
roslibpy>=1.3.0
matplotlib>=3.5.0
numpy>=1.21.0
requests>=2.25.0
psutil>=5.8.0
"@
    Set-Content -Path "requirements.txt" -Value $requirementsContent
    Write-Success "Requirements.txt created"
    Write-Log "Requirements.txt created"
}

function Install-Python {
    Write-Info "Downloading and installing Python 3.11..."
    $pythonUrl = "https://www.python.org/ftp/python/3.11.7/python-3.11.7-amd64.exe"
    $pythonInstaller = "$env:TEMP\python-installer.exe"
    
    # Download Python installer
    Invoke-WebRequest -Uri $pythonUrl -OutFile $pythonInstaller
    
    # Install Python
    Start-Process -FilePath $pythonInstaller -ArgumentList "/quiet InstallAllUsers=1 PrependPath=1" -Wait
    
    # Remove installer
    Remove-Item $pythonInstaller
    
    # Refresh environment variables
    $env:Path = [System.Environment]::GetEnvironmentVariable("Path","Machine") + ";" + [System.Environment]::GetEnvironmentVariable("Path","User")
    
    Write-Success "Python 3.11 installed"
    Write-Log "Python 3.11 installed"
}

function Setup-ROS2Environment {
    Write-Header "Setting Up ROS2 Environment"
    
    # Copy and make install script executable
    Write-Info "Setting up ROS2 installation script..."
    $installScript = @"
#!/bin/bash
set -e

echo "🤖 Installing ROS2 Jazzy..."

# Update system
sudo apt update && sudo apt upgrade -y

# Install prerequisites
sudo apt install -y curl gnupg2 lsb-release software-properties-common

# Add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=`$(dpkg --print-architecture`) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu `$(. /etc/os-release && echo `$UBUNTU_CODENAME`) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package index
sudo apt update

# Install ROS2
sudo apt install -y ros-jazzy-desktop-full
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup
sudo apt install -y ros-jazzy-gazebo-* ros-jazzy-robot-localization
sudo apt install -y python3-rosdep python3-colcon-common-extensions
sudo apt install -y ros-jazzy-rosbridge-server

# Initialize rosdep
sudo rosdep init || true
rosdep update

# Setup environment
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

echo "✅ ROS2 Jazzy installation completed!"
"@
    
    Set-Content -Path "install_ros2_jazzy.sh" -Value $installScript
    Write-Success "ROS2 install script created"
    Write-Log "ROS2 install script created"
    
    # Run installation in WSL
    Write-Info "Installing ROS2 in WSL..."
    wsl -d Ubuntu-24.04 -- bash -c "cd /mnt/c/Users/$env:USERNAME/OneDrive/Documents/GitHub/tractobots && chmod +x install_ros2_jazzy.sh && ./install_ros2_jazzy.sh"
    Write-Success "ROS2 Jazzy installed in WSL"
    Write-Log "ROS2 Jazzy installed in WSL"
    
    # Create environment setup script
    Write-Info "Creating ROS environment setup script..."
    $envScript = @"
#!/bin/bash
set -e

echo "🌉 Setting up ROS environment..."

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Source workspace if it exists
if [ -f "/mnt/c/Users/$env:USERNAME/OneDrive/Documents/GitHub/tractobots/install/setup.bash" ]; then
    source /mnt/c/Users/$env:USERNAME/OneDrive/Documents/GitHub/tractobots/install/setup.bash
fi

# Start rosbridge server
echo "Starting rosbridge server..."
pkill -f rosbridge_websocket || true
sleep 2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

echo "✅ ROS environment ready!"
echo "📡 Rosbridge server started on port 9090"
"@
    
    Set-Content -Path "setup_ros_environment.sh" -Value $envScript
    Write-Success "ROS environment script created"
    Write-Log "ROS environment script created"
}

function Setup-NetworkConfiguration {
    Write-Header "Setting Up Network Configuration"
    
    # Get WSL IP
    Write-Info "Configuring WSL port forwarding..."
    $wslIP = (wsl -d Ubuntu-24.04 -- hostname -I).Trim()
    Write-Success "WSL IP detected: $wslIP"
    Write-Log "WSL IP: $wslIP"
    
    # Setup port forwarding
    try {
        netsh interface portproxy delete v4tov4 listenport=9090 listenaddress=0.0.0.0 | Out-Null
    }
    catch {
        # Port forwarding rule didn't exist, that's fine
    }
    
    netsh interface portproxy add v4tov4 listenport=9090 listenaddress=0.0.0.0 connectport=9090 connectaddress=$wslIP
    Write-Success "Port forwarding configured for ROS bridge (port 9090)"
    Write-Log "Port forwarding configured: 9090 -> $wslIP:9090"
    
    # Configure firewall
    Write-Info "Configuring Windows Firewall..."
    try {
        Remove-NetFirewallRule -DisplayName "ROS Bridge Port 9090" -ErrorAction SilentlyContinue
    }
    catch {
        # Rule didn't exist, that's fine
    }
    
    New-NetFirewallRule -DisplayName "ROS Bridge Port 9090" -Direction Inbound -Protocol TCP -LocalPort 9090 -Action Allow | Out-Null
    Write-Success "Firewall rule created for port 9090"
    Write-Log "Firewall rule created for port 9090"
}

function Setup-TractobotsWorkspace {
    Write-Header "Setting Up Tractobots Workspace"
    
    # Build workspace
    Write-Info "Building Tractobots workspace..."
    wsl -d Ubuntu-24.04 -- bash -c @"
cd /mnt/c/Users/$env:USERNAME/OneDrive/Documents/GitHub/tractobots
source /opt/ros/jazzy/setup.bash

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install

# Add workspace to bashrc
if ! grep -q "source.*tractobots.*install/setup.bash" ~/.bashrc; then
    echo "source /mnt/c/Users/$env:USERNAME/OneDrive/Documents/GitHub/tractobots/install/setup.bash" >> ~/.bashrc
fi

echo "✅ Tractobots workspace built successfully!"
"@
    
    Write-Success "Tractobots workspace built"
    Write-Log "Tractobots workspace built"
    
    # Create launcher scripts
    Write-Info "Creating launcher scripts..."
    
    # PowerShell launcher
    $psLauncher = @"
# Tractobots Dashboard Launcher
# Auto-generated by setup script

Write-Host "🚜 Starting Tractobots Dashboard..." -ForegroundColor Green

# Change to project directory
Set-Location "$PSScriptRoot"

# Check if ROS environment is running
try {
    `$response = Invoke-WebRequest -Uri "http://localhost:9090" -TimeoutSec 5 -ErrorAction Stop
    Write-Host "✅ ROS Bridge already running" -ForegroundColor Green
}
catch {
    Write-Host "🌉 Starting ROS environment..." -ForegroundColor Yellow
    wsl -d Ubuntu-24.04 -- bash -c "cd /mnt/c/Users/$env:USERNAME/OneDrive/Documents/GitHub/tractobots && ./setup_ros_environment.sh"
    Start-Sleep 5
}

# Launch dashboard
Write-Host "🖥️ Launching dashboard..." -ForegroundColor Blue
python tractobots_win_dashboard.py

Read-Host "Press Enter to exit"
"@
    
    Set-Content -Path "run_dashboard.ps1" -Value $psLauncher
    Write-Success "PowerShell launcher created"
    Write-Log "PowerShell launcher created"
    
    # Batch launcher
    $batchLauncher = @"
@echo off
echo 🚜 Starting Tractobots Dashboard...

cd /d "%~dp0"

echo 🌉 Starting ROS environment...
wsl -d Ubuntu-24.04 -- bash -c "cd /mnt/c/Users/$env:USERNAME/OneDrive/Documents/GitHub/tractobots && ./setup_ros_environment.sh"
timeout /t 5 /nobreak > nul

echo 🖥️ Launching dashboard...
python tractobots_win_dashboard.py

pause
"@
    
    Set-Content -Path "run_dashboard.bat" -Value $batchLauncher
    Write-Success "Batch launcher created"
    Write-Log "Batch launcher created"
}

function Test-CompleteInstallation {
    Write-Header "Testing Complete Installation"
    
    if ($TestOnly) {
        Write-Info "Running in test-only mode"
    }
    
    # Test WSL
    Write-Info "Testing WSL connectivity..."
    try {
        $wslTest = wsl -d Ubuntu-24.04 -- echo "WSL connection test"
        if ($wslTest -eq "WSL connection test") {
            Write-Success "WSL connectivity confirmed"
            Write-Log "WSL connectivity test passed"
        }
        else {
            throw "WSL test failed"
        }
    }
    catch {
        Write-Error "WSL connectivity test failed"
        Write-Log "WSL connectivity test failed" "ERROR"
        return
    }
    
    # Test ROS2
    Write-Info "Testing ROS2 installation..."
    try {
        $rosTest = wsl -d Ubuntu-24.04 -- bash -c "source /opt/ros/jazzy/setup.bash && ros2 --version"
        if ($rosTest -match "ros2 run") {
            Write-Success "ROS2 installation confirmed"
            Write-Log "ROS2 installation test passed"
        }
        else {
            throw "ROS2 test failed"
        }
    }
    catch {
        Write-Error "ROS2 installation test failed"
        Write-Log "ROS2 installation test failed" "ERROR"
        return
    }
    
    # Test Python dependencies
    Write-Info "Testing Python dependencies..."
    try {
        python -c "import PyQt5, websocket, roslibpy, matplotlib, numpy, requests, psutil; print('All dependencies OK')"
        Write-Success "Python dependencies confirmed"
        Write-Log "Python dependencies test passed"
    }
    catch {
        Write-Error "Python dependencies test failed"
        Write-Log "Python dependencies test failed" "ERROR"
        return
    }
    
    # Test workspace build
    Write-Info "Testing workspace build..."
    try {
        $buildTest = wsl -d Ubuntu-24.04 -- bash -c "cd /mnt/c/Users/$env:USERNAME/OneDrive/Documents/GitHub/tractobots && source install/setup.bash && ros2 pkg list | grep tractobots"
        if ($buildTest) {
            Write-Success "Workspace build confirmed"
            Write-Log "Workspace build test passed"
        }
        else {
            throw "Workspace build test failed"
        }
    }
    catch {
        Write-Error "Workspace build test failed"
        Write-Log "Workspace build test failed" "ERROR"
        return
    }
    
    # Test network configuration
    Write-Info "Testing network configuration..."
    try {
        $portTest = netsh interface portproxy show all | Select-String "9090"
        if ($portTest) {
            Write-Success "Port forwarding configured"
            Write-Log "Port forwarding test passed"
        }
        else {
            throw "Port forwarding not configured"
        }
    }
    catch {
        Write-Error "Network configuration test failed"
        Write-Log "Network configuration test failed" "ERROR"
        return
    }
    
    Write-Success "All installation tests passed!"
    Write-Log "All installation tests passed"
}

function Show-CompletionMessage {
    Write-Header "Setup Complete!"
    
    Write-Success "Tractobots environment setup completed successfully!"
    Write-Info ""
    Write-Info "🚀 Quick Start:"
    Write-Info "   1. Double-click 'run_dashboard.bat' to start the dashboard"
    Write-Info "   2. Or run 'powershell -ExecutionPolicy Bypass -File run_dashboard.ps1'"
    Write-Info "   3. Or manually: 'python tractobots_win_dashboard.py'"
    Write-Info ""
    Write-Info "📂 Project Location: $PWD"
    Write-Info "📄 Setup Log: $LogFile"
    Write-Info "📚 Documentation: .github/INSTRUCTIONS.md"
    Write-Info ""
    Write-Info "🔧 Manual Commands:"
    Write-Info "   Start ROS Environment: wsl -- ./setup_ros_environment.sh"
    Write-Info "   Launch Dashboard: python tractobots_win_dashboard.py"
    Write-Info "   Test Connection: python test_rosbridge_connection.py"
    Write-Info ""
    Write-Warning "⚠️ Important Notes:"
    Write-Warning "   - ROS2 environment runs in WSL Ubuntu 24.04"
    Write-Warning "   - Dashboard runs on Windows with Python 3.11+"
    Write-Warning "   - Port 9090 is used for ROS Bridge communication"
    Write-Warning "   - First launch may take longer while starting services"
    Write-Info ""
    Write-Success "🎉 Ready to start autonomous agriculture!"
    
    Write-Log "Setup completed successfully"
}

# Script execution
if ($TestOnly) {
    Write-Header "Test Mode - Validating Existing Installation"
    Test-CompleteInstallation
}
else {
    Start-TractobotsSetup
}
