# Tractobots Windows Dashboard

A professional Windows native dashboard for controlling and monitoring Tractobots ROS2 system running in WSL.

## Features

- Native Windows GUI (PyQt5) for ROS2 monitoring and control
- Direct WSL command execution from Windows
- Real-time ROS node and topic monitoring
- Interactive terminal with command history
- System status visualization with gauges
- Automatic WSL port forwarding setup
- Comprehensive command reference

## Requirements

- Windows 10/11 with WSL2
- ROS2 Jazzy installed in WSL
- Python 3.8 or newer
- Required Python packages:
  - PyQt5
  - matplotlib
  - numpy
  - roslibpy

## Quick Start

1. Run the dashboard:
   ```
   start_tractobots_dashboard.cmd
   ```
   (Run as Administrator for WSL port forwarding setup)

2. Connect to ROS2:
   - Click "Start Rosbridge" to start the WebSocket server in WSL
   - Click "Connect to ROS" to establish the connection

3. Launch robot model:
   - Click "Launch Robot Model" to start Gazebo with the Tractobots model

## Directory Structure

```
dashboard/
├── __init__.py             # Package initialization
├── __main__.py             # Entry point for running as a module
├── dashboard.py            # Main dashboard implementation
├── launcher.py             # Dashboard launcher logic
├── utils/                  # Utility modules
│   ├── __init__.py
│   ├── wsl_port_forwarding.py          # WSL port forwarding utilities
│   ├── Setup-WSL-PortForwarding.ps1    # PowerShell port forwarding script
│   └── setup_wsl_port_forwarding.bat   # Batch file for port forwarding
├── tests/                  # Test modules
│   ├── __init__.py
│   ├── integration_test.py             # Integration tests
│   └── simple_dashboard.py             # Minimal dashboard for testing
└── scripts/                # Helper scripts
    ├── __init__.py
    └── launch_dashboard.bat            # Dashboard launcher script
```

## Development

### Testing

Run integration tests to verify the dashboard functionality:

```
python -m dashboard.tests.integration_test
```

### Customization

The dashboard can be extended with additional features by modifying `dashboard.py`.

## Troubleshooting

If you encounter connection issues:

1. Verify WSL is running: `wsl --status`
2. Check if ROS2 is installed: `wsl -e ls /opt/ros/jazzy`
3. Ensure port forwarding is set up: `netsh interface portproxy show v4tov4`
4. Start rosbridge manually: `wsl -e ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
