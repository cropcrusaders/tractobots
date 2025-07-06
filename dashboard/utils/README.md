# Tractobots Dashboard Utilities

This directory contains utility scripts and tools for the Tractobots Windows Dashboard.

## Available Utilities

### WSL Port Forwarding

The following tools help set up port forwarding between Windows and WSL for ROS Bridge connections:

#### `wsl_port_forwarding.py`

Python script that detects the WSL IP address and configures port forwarding.

Usage:
```
python wsl_port_forwarding.py
```

#### `Setup-WSL-PortForwarding.ps1`

PowerShell script for configuring port forwarding as Administrator.

Usage:
```powershell
# Run as Administrator
.\Setup-WSL-PortForwarding.ps1
```

#### `setup_wsl_port_forwarding.bat`

Batch file wrapper for the PowerShell script.

Usage:
```
# Run as Administrator
setup_wsl_port_forwarding.bat
```

## How Port Forwarding Works

1. The script detects the current WSL IP address (which can change on restarts)
2. It removes any existing port forwarding rules for port 9090
3. It creates a new rule forwarding localhost:9090 to WSL_IP:9090
4. This allows Windows applications to connect to ROS Bridge on localhost

## Troubleshooting

If the dashboard cannot connect to ROS Bridge:

1. Ensure WSL is running
2. Run the port forwarding setup again (WSL IP may have changed)
3. Verify ROS Bridge is running in WSL with:
   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```
4. Check if port forwarding is active:
   ```powershell
   netsh interface portproxy show v4tov4
   ```
