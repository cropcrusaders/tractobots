# Tractobots WSL Port Forwarding Setup
# This script sets up port forwarding from Windows to WSL for ROS Bridge

# Ensure script is run as admin
$currentPrincipal = New-Object Security.Principal.WindowsPrincipal([Security.Principal.WindowsIdentity]::GetCurrent())
$isAdmin = $currentPrincipal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)

if (-not $isAdmin) {
    Write-Host "This script needs to be run as Administrator." -ForegroundColor Red
    Write-Host "Please restart it with admin privileges." -ForegroundColor Red
    Pause
    exit
}

Write-Host "=== WSL Port Forwarding Setup ===" -ForegroundColor Cyan
Write-Host "Setting up port forwarding from Windows to WSL for ROS Bridge" -ForegroundColor Cyan
Write-Host ""

# Get WSL IP address
Write-Host "Getting WSL IP address..." -ForegroundColor Yellow
try {
    $wslIP = wsl -e bash -c "hostname -I | awk '{print `$1}'" | Out-String
    $wslIP = $wslIP.Trim()
    
    if ([string]::IsNullOrEmpty($wslIP)) {
        Write-Host "Could not get WSL IP address. Make sure WSL is running." -ForegroundColor Red
        Pause
        exit
    }
    
    Write-Host "WSL IP address: $wslIP" -ForegroundColor Green
}
catch {
    Write-Host "Error: $_" -ForegroundColor Red
    Write-Host "Failed to get WSL IP. Make sure WSL is installed and running." -ForegroundColor Red
    Pause
    exit
}

# Set up port forwarding
$port = 9090

Write-Host "`nSetting up port forwarding for port $port..." -ForegroundColor Yellow

# Delete any existing port proxy for this port
try {
    netsh interface portproxy delete v4tov4 listenport=$port listenaddress=127.0.0.1 | Out-Null
    Write-Host "Removed any existing port forwarding rule." -ForegroundColor Gray
}
catch {
    # Ignore errors if the rule doesn't exist
}

# Add new port proxy
try {
    $result = netsh interface portproxy add v4tov4 listenport=$port listenaddress=127.0.0.1 connectport=$port connectaddress=$wslIP
    Write-Host "Port forwarding rule added: localhost:$port -> $wslIP`:$port" -ForegroundColor Green
}
catch {
    Write-Host "Error: $_" -ForegroundColor Red
    Write-Host "Failed to set up port forwarding." -ForegroundColor Red
    Pause
    exit
}

# Show current port proxies
Write-Host "`nCurrent port forwarding rules:" -ForegroundColor Yellow
netsh interface portproxy show v4tov4

# Test connection (optional)
Write-Host "`nNote: The port may not respond if rosbridge is not running in WSL." -ForegroundColor Yellow
Write-Host "To start rosbridge, run this command in WSL:" -ForegroundColor Yellow
Write-Host "ros2 launch rosbridge_server rosbridge_websocket_launch.xml" -ForegroundColor Cyan

# Setup success
Write-Host "`n=== PORT FORWARDING SETUP COMPLETE ===" -ForegroundColor Green
Write-Host "You can now connect to ROS Bridge WebSocket on:" -ForegroundColor Green
Write-Host "ws://localhost:9090" -ForegroundColor Cyan
Write-Host "`nNOTE: If WSL IP changes after restart, run this script again." -ForegroundColor Yellow

Pause
