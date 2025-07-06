# Tractobots Dashboard Tests

This directory contains test scripts and tools for the Tractobots Windows Dashboard.

## Available Tests

### `integration_test.py`

Comprehensive integration tests for the Tractobots Dashboard.

Usage:
```
python integration_test.py
```

Tests include:
- Dashboard imports verification
- WSL availability check
- Dashboard instantiation test
- ROS Bridge connection test
- Port forwarding utility test

### `simple_dashboard.py`

A simplified dashboard for testing basic functionality.

Usage:
```
python simple_dashboard.py
```

Features:
- Minimal dashboard with only essential components
- Interactive terminal for testing commands
- Basic UI elements to verify PyQt5 functionality

## Running All Tests

To run all tests:

```bash
# From the dashboard directory
python -m tests.integration_test
```

## Troubleshooting

If tests fail:

1. Ensure all dependencies are installed:
   ```
   pip install PyQt5 matplotlib numpy roslibpy
   ```
2. Verify WSL is running and accessible
3. Check if your system has proper permissions for GUI applications
