# Tractobots WSL Port Forwarding Setup
# Run this as Administrator to enable Windows -> WSL port forwarding

Write-Host "🚜 === TRACTOBOTS WSL PORT FORWARDING SETUP ===" -ForegroundColor Green
Write-Host ""

# Check if running as administrator
if (-NOT ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole] "Administrator")) {
    Write-Host "❌ This script requires Administrator privileges" -ForegroundColor Red
    Write-Host "💡 Right-click PowerShell and select 'Run as Administrator'" -ForegroundColor Yellow
    Read-Host "Press Enter to exit"
    exit 1
}

Write-Host "✅ Running as Administrator" -ForegroundColor Green
Write-Host ""

# Get WSL IP address
Write-Host "🔍 Getting WSL IP address..." -ForegroundColor Cyan
try {
    $wslIP = (wsl hostname -I).Trim()
    Write-Host "📍 WSL IP: $wslIP" -ForegroundColor Green
} catch {
    Write-Host "❌ Failed to get WSL IP address" -ForegroundColor Red
    Write-Host "💡 Make sure WSL is running" -ForegroundColor Yellow
    Read-Host "Press Enter to exit"
    exit 1
}

# Remove existing port forwarding
Write-Host ""
Write-Host "🔧 Removing existing port forwarding for port 9090..." -ForegroundColor Cyan
netsh interface portproxy delete v4tov4 listenport=9090 2>$null

# Add new port forwarding
Write-Host "🔧 Setting up port forwarding: localhost:9090 -> $wslIP`:9090" -ForegroundColor Cyan
$result = netsh interface portproxy add v4tov4 listenport=9090 listenaddress=127.0.0.1 connectport=9090 connectaddress=$wslIP

if ($LASTEXITCODE -eq 0) {
    Write-Host "✅ Port forwarding configured successfully!" -ForegroundColor Green
} else {
    Write-Host "❌ Port forwarding failed" -ForegroundColor Red
    Read-Host "Press Enter to exit"
    exit 1
}

# Start rosbridge in WSL
Write-Host ""
Write-Host "🚀 Starting rosbridge in WSL..." -ForegroundColor Cyan
wsl -e bash -c "source /opt/ros/jazzy/setup.bash && nohup ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /tmp/rosbridge.log 2>&1 &"

Write-Host "⏳ Waiting for rosbridge to start..." -ForegroundColor Yellow
Start-Sleep -Seconds 5

# Show current port forwarding rules
Write-Host ""
Write-Host "📋 Current port forwarding rules:" -ForegroundColor Cyan
netsh interface portproxy show v4tov4

# Test connection
Write-Host ""
Write-Host "🔍 Testing connection..." -ForegroundColor Cyan
try {
    $tcpClient = New-Object System.Net.Sockets.TcpClient
    $tcpClient.ConnectAsync("127.0.0.1", 9090).Wait(3000)
    if ($tcpClient.Connected) {
        Write-Host "✅ Connection successful! Rosbridge is accessible" -ForegroundColor Green
        $tcpClient.Close()
    } else {
        Write-Host "⚠️ Connection test inconclusive" -ForegroundColor Yellow
    }
} catch {
    Write-Host "⚠️ Connection test failed, but rosbridge may still be working" -ForegroundColor Yellow
}

Write-Host ""
Write-Host "🎉 Setup complete!" -ForegroundColor Green
Write-Host "🌐 Rosbridge should now be accessible at ws://localhost:9090" -ForegroundColor Cyan
Write-Host "🚜 You can now run: python tractobots_win_dashboard.py" -ForegroundColor Cyan
Write-Host ""
Read-Host "Press Enter to exit"
